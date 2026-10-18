#!/usr/bin/env python3
"""Tk GUI for commanding the simulated Drawbot X, Y and pen axes."""

from __future__ import annotations

import random
import tkinter as tk
from collections import deque
from math import hypot
from threading import Thread
from time import monotonic, time_ns
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from ros_gz_interfaces.srv import SpawnEntity
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint
from drawbot_control.drawing_paths import PCB_GREETING_PATHS
from drawbot_control.robot_geometry import (
    GEOMETRY,
    PEN_CONTACT as GEOMETRIC_PEN_CONTACT,
    PEN_DOWN_POSITION,
)
from drawbot_control.trace_visualization import (
    TRACE_SEGMENTS_PER_MODEL,
    make_trace_batch_sdf,
    set_trace_factory_pose,
)
from drawbot_control.xyz_control_view import XYZControlView

XY_TRAJECTORY_TOPIC = "/xy_controller/follow_joint_trajectory"
PEN_COMMAND_TOPIC = "/pen_controller/commands"
JOINT_STATES_TOPIC = "/joint_states"
TRACE_SPAWN_SERVICE = "/world/drawing_world/create"
PEN_SETTLE_TOLERANCE = 0.0007
PEN_STEP = 0.0005


class XYZControlGUI:
    """Small desktop control panel for the Gazebo ros2_control joints."""

    X_MIN, X_MAX = GEOMETRY["limits"]["x"]
    Y_MIN, Y_MAX = GEOMETRY["limits"]["y"]
    Z_MIN, Z_MAX = GEOMETRY["limits"]["z"]
    PEN_CONTACT = GEOMETRIC_PEN_CONTACT
    PEN_UP = 0.0
    PEN_DOWN = PEN_DOWN_POSITION
    PCB_HALF_WIDTH = min(GEOMETRY["pcb"]["x"], GEOMETRY["pcb"]["y"]) / 2.0
    MIN_TRACE_STEP = GEOMETRY["drawing"]["min_trace_step"]
    TRACE_DOT_SIZE = GEOMETRY["drawing"]["trace_width"]
    PRINT_SPEED = 0.025
    TRAVEL_SPEED = 0.10
    MIN_PRINT_SEGMENT_TIME = 0.5
    MIN_TRAVEL_TIME = 1.0
    def __init__(self, root: tk.Tk, node: Node) -> None:
        self.root = root
        self.node = node
        self.robot_base_x = float(node.get_parameter("robot_base_x").value)
        self.robot_base_y = float(node.get_parameter("robot_base_y").value)
        self.robot_base_z = float(node.get_parameter("robot_base_z").value)
        self.goal_handle: Any = None
        self.request_pending = False
        self.closing = False
        self.pen_position = self.PEN_UP
        self.pen_target = self.PEN_UP
        self.measured_positions: dict[str, float] = {}
        self.printing = False
        self.print_cancel_requested = False
        self.print_abort_message = ""
        self.print_trace_index = 0
        self.print_path: tuple[tuple[float, float], ...] = ()
        self.print_paths = tuple(
            stroke for word_strokes in PCB_GREETING_PATHS for stroke in word_strokes
        )
        self.print_wait_token = 0
        self.trace_model_counter = 0
        self.trace_model_prefix = f"pcb_trace_{time_ns()}"
        self.trace_spawn_segments: deque[
            tuple[tuple[float, float], tuple[float, float]]
        ] = deque()
        self.trace_spawn_in_flight = False
        self.trace_drain_started = False
        self.live_trace_last_point: tuple[float, float] | None = None
        self.close_deadline = 0.0
        self.xy_client = ActionClient(
            node,
            FollowJointTrajectory,
            XY_TRAJECTORY_TOPIC,
        )

        self.pen_publisher = node.create_publisher(
            Float64MultiArray,
            PEN_COMMAND_TOPIC,
            10,
        )

        self.joint_state_subscription = node.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self.on_joint_state,
            10,
        )

        self.trace_spawn_client = node.create_client(
            SpawnEntity,
            TRACE_SPAWN_SERVICE,
        )
        # Slew the pen at 0.10 m/s, below the joint's 0.15 m/s limit.
        self.pen_timer = node.create_timer(0.01, self.advance_pen)

        self.view = XYZControlView(
            root,
            self.PEN_CONTACT,
            {
                "send": self.send_xyz,
                "center": self.center,
                "random": self.random_position,
                "print": self.start_print,
                "cancel": self.cancel_xy,
            },
            self.close,
        )
        self.scales = self.view.scales
        self.value_labels = self.view.value_labels
        self.status = self.view.status

        self.root.after(250, self.update_controller_status)
        self.root.after(20, self._monitor_pen_contact)

    def update_value_label(self, axis: str) -> None:
        self.view.update_value_label(axis)

    def _set_status(self, message: str) -> None:
        self.status.set(message)

    def _schedule_status_update(self, message: str) -> None:
        if not self.closing:
            self.root.after(0, lambda text=message: self._set_status(text))

    def _set_print_controls_enabled(self, enabled: bool) -> None:
        self.view.set_controls_enabled(enabled)

    def current_target(self) -> tuple[float, float, float]:
        return (
            round(self.scales["X"].get(), 3),
            round(self.scales["Y"].get(), 3),
            round(self.scales["Z"].get(), 3),
        )

    def on_joint_state(self, message: JointState) -> None:
        """Keep measured joint positions for pen-settling checks and safe travel."""
        self.measured_positions.update(zip(message.name, message.position))

    def _monitor_pen_contact(self) -> None:
        """Draw orange Gazebo segments as the measured pen moves on the PCB."""
        if self.closing:
            return

        x = self.measured_positions.get("joint_x")
        y = self.measured_positions.get("joint_y")
        pen = self.measured_positions.get("joint_pen")
        in_contact = False
        if x is not None and y is not None and pen is not None:
            in_contact = (
                pen <= self.PEN_CONTACT
                and abs(x) <= self.PCB_HALF_WIDTH
                and abs(y) <= self.PCB_HALF_WIDTH
            )
        if not in_contact:
            self.live_trace_last_point = None
        else:
            assert x is not None and y is not None
            point = (x, y)
            previous = self.live_trace_last_point
            if previous is None:
                self.live_trace_last_point = point
                half_dot = self.TRACE_DOT_SIZE / 2.0
                self.trace_spawn_segments.append((
                    (x - half_dot, y),
                    (x + half_dot, y),
                ))
            elif (
                hypot(point[0] - previous[0], point[1] - previous[1])
                >= self.MIN_TRACE_STEP
            ):
                self.trace_spawn_segments.append((previous, point))
                self.live_trace_last_point = point
        if self.trace_spawn_segments and not self.trace_spawn_in_flight:
            self._spawn_next_trace_segment()

        self.root.after(20, self._monitor_pen_contact)

    def send_xyz(self) -> None:
        if self.printing or self.request_pending:
            self._set_status("Attendre la fin de la commande en cours.")
            return
        if not self.xy_client.server_is_ready():
            self._set_status("XY indisponible : attendre l’activation des contrôleurs.")
            return

        x, y, z = self.current_target()
        point = JointTrajectoryPoint()
        point.positions = [x, y]
        point.time_from_start = Duration(sec=1, nanosec=0)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_x", "joint_y"]
        goal.trajectory.points = [point]

        self.pen_target = z
        self.request_pending = True
        future = self.xy_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)
        self._set_status(f"Commande envoyée : X={x:+.3f}, Y={y:+.3f}, Z={z:+.3f} m")

    def start_print(self) -> None:
        """Trace a demo PCB pattern with pen-up travel between copper paths."""
        if self.printing or self.request_pending:
            self._set_status("Une commande est déjà en cours.")
            return
        if not self.xy_client.server_is_ready():
            self._set_status("XY indisponible : attendre les contrôleurs Gazebo.")
            return
        if not self.trace_spawn_client.service_is_ready():
            self._set_status("Service Gazebo /world/drawing_world/create indisponible.")
            return
        required_joints = {"joint_x", "joint_y", "joint_pen"}
        if not required_joints.issubset(self.measured_positions):
            self._set_status("Attente des positions X/Y/Z sur /joint_states.")
            return

        self.printing = True
        self.print_cancel_requested = False
        self.print_trace_index = 0
        self._set_print_controls_enabled(False)
        self._set_status("Impression PCB : préparation du stylo…")
        self._set_print_pen_target(self.PEN_UP)
        self._wait_for_pen(self.PEN_UP, self._move_to_next_trace)

    def _cancel_current_goal(self) -> None:
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def _set_print_pen_target(self, target: float) -> None:
        self.pen_target = target
        self.scales["Z"].set(target)
        self.update_value_label("Z")

    def _wait_for_pen(self, target: float, on_reached: Any) -> None:
        """Wait for measured pen feedback; timeout if Gazebo is paused or lost."""
        self.print_wait_token += 1
        token = self.print_wait_token
        deadline = monotonic() + 5.0

        def poll() -> None:
            if not self.printing or token != self.print_wait_token:
                return
            if self.print_cancel_requested and target != self.PEN_UP:
                self._set_print_pen_target(self.PEN_UP)
                self._wait_for_pen(self.PEN_UP, self._finish_cancelled_print)
                return

            measured = self.measured_positions.get("joint_pen")
            if measured is not None and abs(measured - target) <= PEN_SETTLE_TOLERANCE:
                on_reached()
                return
            if monotonic() >= deadline:
                message = "Le stylo n’a pas atteint la position demandée."
                if target == self.PEN_UP and self.print_cancel_requested:
                    prefix = self.print_abort_message or "Impression interrompue."
                    self._finish_print(f"{prefix} Levée du stylo non confirmée.")
                else:
                    self._abort_print(f"{message} Vérifier Gazebo.")
                return
            self.root.after(50, poll)

        poll()

    def _move_to_next_trace(self) -> None:
        if self.print_cancel_requested:
            self._set_print_pen_target(self.PEN_UP)
            self._wait_for_pen(self.PEN_UP, self._finish_cancelled_print)
            return
        if self.print_trace_index >= len(self.print_paths):
            self._set_print_pen_target(self.PEN_UP)
            self._wait_for_pen(
                self.PEN_UP,
                lambda: self._finish_print("Impression PCB terminée."),
            )
            return

        self.print_path = self.print_paths[self.print_trace_index]
        self.status.set(
            f"Trait {self.print_trace_index + 1}/{len(self.print_paths)} : "
            "déplacement stylo levé…"
        )
        self._set_print_pen_target(self.PEN_UP)
        self._wait_for_pen(self.PEN_UP, self._move_to_trace_start)

    def _move_to_trace_start(self) -> None:
        if self.print_cancel_requested:
            self._finish_cancelled_print()
            return
        start_x, start_y = self.print_path[0]
        current_x = self.measured_positions.get("joint_x", 0.0)
        current_y = self.measured_positions.get("joint_y", 0.0)
        travel_time = max(
            self.MIN_TRAVEL_TIME,
            abs(start_x - current_x) / self.TRAVEL_SPEED,
            abs(start_y - current_y) / self.TRAVEL_SPEED,
        )
        self._send_print_trajectory(
            [(start_x, start_y, travel_time)], 
            self._lower_pen_for_trace
        )

    def _lower_pen_for_trace(self) -> None:
        if self.print_cancel_requested:
            self._finish_cancelled_print()
            return
        self._set_print_pen_target(self.PEN_DOWN)
        self.status.set(
            f"Trait {self.print_trace_index + 1}/{len(self.print_paths)} : "
            "stylo au contact…"
        )
        self._wait_for_pen(self.PEN_DOWN, self._draw_current_trace)

    def _draw_current_trace(self) -> None:
        if self.print_cancel_requested:
            self._set_print_pen_target(self.PEN_UP)
            self._wait_for_pen(
                self.PEN_UP, 
                self._finish_cancelled_print
            )
            return

        waypoints = []
        elapsed = 0.0
        previous = self.print_path[0]
        for x, y in self.print_path[1:]:
            elapsed += max(
                self.MIN_PRINT_SEGMENT_TIME,
                hypot(x - previous[0], y - previous[1]) / self.PRINT_SPEED,
            )
            waypoints.append((x, y, elapsed))
            previous = (x, y)

        self.status.set(
            f"Trait {self.print_trace_index + 1}/{len(self.print_paths)} : "
            "tracé X/Y en cours…"
        )
        self._send_print_trajectory(waypoints, self._raise_pen_after_trace)

    def _spawn_next_trace_segment(self) -> None:
        if self.trace_spawn_in_flight or not self.trace_spawn_segments:
            return
        if not self.trace_spawn_client.service_is_ready():
            return

        batch = [
            self.trace_spawn_segments.popleft()
            for _ in range(min(TRACE_SEGMENTS_PER_MODEL, len(self.trace_spawn_segments)))
        ]
        model_name = f"{self.trace_model_prefix}_{self.trace_model_counter:04d}"
        sdf = make_trace_batch_sdf(
            batch,
            model_name,
        )
        request = SpawnEntity.Request()
        request.entity_factory.name = model_name
        request.entity_factory.sdf = sdf
        set_trace_factory_pose(
            request.entity_factory,
            self.robot_base_x,
            self.robot_base_y,
            self.robot_base_z,
        )
        self.trace_model_counter += 1
        self.trace_spawn_in_flight = True
        future = self.trace_spawn_client.call_async(request)
        future.add_done_callback(self._on_trace_segment_spawned)

    def _on_trace_segment_spawned(self, future: Future) -> None:
        try:
            response = future.result()
            succeeded = response.success
        except Exception as exc:
            self.trace_spawn_in_flight = False
            self.root.after(
                0,
                lambda error=exc: self._handle_trace_spawn_failure(
                    f"Erreur création de la piste dans Gazebo : {error}"
                ),
            )
            return

        if not succeeded:
            self.trace_spawn_in_flight = False
            self.root.after(
                0,
                lambda: self._handle_trace_spawn_failure(
                    "Gazebo a refusé la création d’un segment de piste."
                ),
            )
            return

        self.trace_spawn_in_flight = False
        self.root.after(0, self._spawn_next_trace_segment)

    def _handle_trace_spawn_failure(self, message: str) -> None:
        self.trace_spawn_segments.clear()
        self.live_trace_last_point = None
        if self.printing:
            self._abort_print(message)
        else:
            self.status.set(message)

    def _raise_pen_after_trace(self) -> None:
        self._set_print_pen_target(self.PEN_UP)
        self._wait_for_pen(self.PEN_UP, self._advance_print_trace)

    def _advance_print_trace(self) -> None:
        self.print_trace_index += 1
        self._move_to_next_trace()

    def _send_print_trajectory(
        self,
        waypoints: list[tuple[float, float, float]],
        on_complete: Any,
    ) -> None:
        if not waypoints or not self.xy_client.server_is_ready():
            self._abort_print("Le contrôleur XY est indisponible.")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["joint_x", "joint_y"]
        for x, y, elapsed in waypoints:
            point = JointTrajectoryPoint()
            point.positions = [x, y]
            point.velocities = [0.0, 0.0]
            seconds = int(elapsed)
            point.time_from_start = Duration(
                sec=seconds,
                nanosec=int(round((elapsed - seconds) * 1e9)),
            )
            goal.trajectory.points.append(point)

        self.request_pending = True
        future = self.xy_client.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self._on_print_goal_response(result, on_complete)
        )

    def _on_print_goal_response(self, future: Future, on_complete: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.request_pending = False
            self.root.after(0, lambda error=exc: self._abort_print(f"Erreur XY : {error}"))
            return
        if not goal_handle.accepted:
            self.request_pending = False
            self.root.after(
                0,
                lambda: self._abort_print("Le contrôleur XY a refusé la trace."),
            )
            return

        self.goal_handle = goal_handle
        if self.print_cancel_requested:
            goal_handle.cancel_goal_async()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self._on_print_goal_result(result, on_complete)
        )

    def _on_print_goal_result(self, future: Future, on_complete: Any) -> None:
        try:
            result = future.result().result
            successful = (
                result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
            error_code = result.error_code
        except Exception as exc:
            self.request_pending = False
            self.goal_handle = None
            self.root.after(0, lambda error=exc: self._abort_print(f"Erreur XY : {error}"))
            return

        self.root.after(
            0,
            lambda: self._handle_print_goal_result(
                successful, error_code, on_complete
            ),
        )

    def _handle_print_goal_result(
        self, successful: bool, error_code: int, on_complete: Any
    ) -> None:
        self.request_pending = False
        self.goal_handle = None
        if self.print_cancel_requested:
            self._set_print_pen_target(self.PEN_UP)
            self._wait_for_pen(self.PEN_UP, self._finish_cancelled_print)
        elif successful:
            on_complete()
        else:
            self._abort_print(f"Trajectoire XY échouée (code {error_code}).")

    def cancel_print(self) -> None:
        if not self.printing:
            return
        self.print_cancel_requested = True
        self.print_wait_token += 1
        self._set_print_pen_target(self.PEN_UP)
        self._set_status("Annulation : levée du stylo…")
        if self.goal_handle is not None:
            self._cancel_current_goal()
        elif not self.request_pending:
            self._wait_for_pen(self.PEN_UP, self._finish_cancelled_print)

    def _abort_print(self, message: str) -> None:
        if not self.printing:
            self._set_status(message)
            return
        self.print_cancel_requested = True
        self.print_abort_message = message
        self.print_wait_token += 1
        self._set_print_pen_target(self.PEN_UP)
        if self.goal_handle is not None:
            self._cancel_current_goal()
        elif not self.request_pending:
            self._wait_for_pen(self.PEN_UP, self._finish_cancelled_print)

    def _finish_cancelled_print(self) -> None:
        message = getattr(self, "print_abort_message", "")
        if message:
            self._finish_print(message)
        else:
            self._finish_print("Impression annulée; stylo relevé.")

    def _finish_print(self, message: str) -> None:
        self.print_wait_token += 1
        self.printing = False
        self.print_cancel_requested = False
        self.print_abort_message = ""
        self.request_pending = False
        self.goal_handle = None
        self._set_print_controls_enabled(True)
        self._set_status(message)

    def send_pen_up(self) -> None:
        self.scales["Z"].set(self.PEN_UP)
        self.update_value_label("Z")
        self.pen_target = self.PEN_UP
        self._set_status("Stylo levé; la position X/Y n’a pas changé.")

    def advance_pen(self) -> None:
        """Move the pen command smoothly within its configured velocity limit."""
        difference = self.pen_target - self.pen_position
        if abs(difference) < 1e-9:
            return
        step = max(-PEN_STEP, min(PEN_STEP, difference))
        self.pen_position = round(self.pen_position + step, 6)
        self.pen_publisher.publish(
            Float64MultiArray(data=[self.pen_position])
        )

    def center(self) -> None:
        if self.printing:
            return
        for axis in ("X", "Y"):
            self.scales[axis].set(0.0)
            self.update_value_label(axis)
        self.scales["Z"].set(self.PEN_UP)
        self.update_value_label("Z")
        self.send_xyz()

    def random_position(self) -> None:
        if self.printing:
            return
        self.scales["X"].set(round(random.uniform(-0.08, 0.08), 3))
        self.scales["Y"].set(round(random.uniform(-0.08, 0.08), 3))
        self.scales["Z"].set(round(random.uniform(self.Z_MIN, self.Z_MAX), 3))
        for axis in ("X", "Y", "Z"):
            self.update_value_label(axis)
        self.send_xyz()

    def cancel_xy(self) -> None:
        if self.printing:
            self.cancel_print()
            return
        if self.goal_handle is None:
            self.status.set("Aucune trajectoire XY active à annuler.")
            return
        self._cancel_current_goal()
        self.goal_handle = None
        self._set_status("Annulation de la trajectoire XY demandée.")

    def on_goal_response(self, future: Future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # Report ROS action errors in the GUI.
            message = f"Erreur XY : {exc}"
            self.request_pending = False
            self._schedule_status_update(message)
            return

        if not goal_handle.accepted:
            self.request_pending = False
            self._schedule_status_update("Le contrôleur XY a refusé la commande.")
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_goal_result)

    def on_goal_result(self, future: Future) -> None:
        try:
            result = future.result().result
            message = f"Trajectoire XY terminée (code {result.error_code})."
        except Exception as exc:  # Report ROS action errors in the GUI.
            message = f"Erreur résultat XY : {exc}"
        self.goal_handle = None
        self.request_pending = False
        self._schedule_status_update(message)

    def update_controller_status(self) -> None:
        if self.closing:
            return
        if not self.xy_client.server_is_ready() and not self.request_pending:
            self._set_status("Attente du contrôleur XY… lance la simulation Gazebo.")
        elif self.xy_client.server_is_ready() and not self.request_pending:
            if self.status.get().startswith("Attente"):
                self._set_status("Contrôleur XY prêt. Choisir X/Y/Z puis envoyer la commande.")
        self.root.after(750, self.update_controller_status)

    def close(self) -> None:
        """Cancel movement and slew the pen upward before closing the window."""
        if self.closing:
            return
        self.closing = True
        if rclpy.ok():
            try:
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        # Reuse advance_pen's velocity-limited command ramp. Publishing 0
        # directly here would step ~6 mm at once and make gz_ros_control clip it.
        self.pen_target = self.PEN_UP
        self.close_deadline = monotonic() + 2.0
        self._set_status("Fermeture : relevage progressif du stylo…")
        self.root.after(50, self._finish_close_when_safe)

    def _finish_close_when_safe(self) -> None:
        measured_pen = self.measured_positions.get("joint_pen")
        pen_is_up = (
            measured_pen is not None
            and abs(measured_pen - self.PEN_UP) <= PEN_SETTLE_TOLERANCE
        )
        if not pen_is_up and monotonic() < self.close_deadline and rclpy.ok():
            self.root.after(50, self._finish_close_when_safe)
            return

        if not self.trace_drain_started:
            self.trace_drain_started = True
            self.close_deadline = monotonic() + 5.0
        if self.trace_spawn_segments and not self.trace_spawn_in_flight:
            self._spawn_next_trace_segment()
        if (
            (self.trace_spawn_segments or self.trace_spawn_in_flight)
            and monotonic() < self.close_deadline
            and rclpy.ok()
        ):
            self.root.after(50, self._finish_close_when_safe)
            return

        self.pen_timer.cancel()
        try:
            self.root.destroy()
        except tk.TclError:
            pass

def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Node("drawbot_xyz_control_gui")
    node.declare_parameter("robot_base_x", 0.0)
    node.declare_parameter("robot_base_y", 0.0)
    node.declare_parameter("robot_base_z", GEOMETRY["spawn"]["z"])
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_ros() -> None:
        try:
            executor.spin()
        except Exception:
            if rclpy.ok():
                raise

    spin_thread = Thread(target=spin_ros, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    gui = XYZControlGUI(root, node)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # A launch-level SIGINT can arrive after Gazebo invalidates the ROS
        # context. Do not publish an abrupt joint target during this teardown.
        gui.pen_timer.cancel()
        try:
            if root.winfo_exists():
                root.destroy()
        except (tk.TclError, KeyboardInterrupt):
            pass
        try:
            executor.shutdown(timeout_sec=0.5)
        except (Exception, KeyboardInterrupt):
            pass
        try:
            spin_thread.join(timeout=0.5)
        except KeyboardInterrupt:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()

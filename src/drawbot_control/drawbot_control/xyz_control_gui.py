#!/usr/bin/env python3
"""Tk GUI for commanding the simulated Drawbot X, Y and pen axes."""

from __future__ import annotations

import random
import tkinter as tk
from threading import Thread
from tkinter import ttk
from typing import Any

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


class XYZControlGUI:
    """Small desktop control panel for the Gazebo ros2_control joints."""

    X_MIN = -0.10
    X_MAX = 0.10
    Y_MIN = -0.10
    Y_MAX = 0.10
    # Keep the GUI pen command at or above the calculated paper-contact point.
    Z_MIN = -0.0075
    Z_MAX = 0.020

    def __init__(self, root: tk.Tk, node: Node) -> None:
        self.root = root
        self.node = node
        self.goal_handle: Any = None
        self.request_pending = False
        self.closing = False
        self.pen_position = 0.0
        self.pen_target = 0.0
        self.scales: dict[str, tk.Scale] = {}
        self.value_labels: dict[str, ttk.Label] = {}
        self.status = tk.StringVar(value="Attente des contrôleurs…")

        self.xy_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/xy_controller/follow_joint_trajectory",
        )
        self.pen_publisher = node.create_publisher(
            Float64MultiArray,
            "/pen_controller/commands",
            10,
        )
        # Slew the pen at 0.1 m/s (0.001 m per 10 ms simulation step),
        # below the joint's 0.15 m/s limit.
        self.pen_timer = node.create_timer(0.01, self.advance_pen)

        root.title("Drawbot — commande XYZ")
        root.geometry("560x360")
        root.minsize(500, 330)
        root.protocol("WM_DELETE_WINDOW", self.close)

        panel = ttk.Frame(root, padding=16)
        panel.pack(fill="both", expand=True)
        ttk.Label(
            panel,
            text="Commande manuelle du robot",
            font=("TkDefaultFont", 14, "bold"),
        ).pack(anchor="w", pady=(0, 2))
        ttk.Label(
            panel,
            text="X/Y : ±0,10 m · Z : levé à 0, contact estimé à −0,0075 m",
        ).pack(anchor="w", pady=(0, 12))

        axes = ttk.Frame(panel)
        axes.pack(fill="x", expand=True)
        self._add_axis(axes, "X", self.X_MIN, self.X_MAX, 0)
        self._add_axis(axes, "Y", self.Y_MIN, self.Y_MAX, 1)
        self._add_axis(axes, "Z", self.Z_MIN, self.Z_MAX, 2)

        buttons = ttk.Frame(panel)
        buttons.pack(fill="x", pady=(8, 8))
        ttk.Button(buttons, text="Envoyer XYZ", command=self.send_xyz).pack(
            side="left", padx=(0, 6)
        )
        ttk.Button(buttons, text="Centre (stylo levé)", command=self.center).pack(
            side="left", padx=6
        )
        ttk.Button(buttons, text="Aléatoire", command=self.random_position).pack(
            side="left", padx=6
        )
        ttk.Button(buttons, text="Annuler XY", command=self.cancel_xy).pack(
            side="right", padx=(6, 0)
        )

        ttk.Label(panel, textvariable=self.status, wraplength=520).pack(
            anchor="w", pady=(4, 0)
        )

        self.root.after(250, self.update_controller_status)

    def _add_axis(
        self,
        parent: ttk.Frame,
        axis: str,
        minimum: float,
        maximum: float,
        row: int,
    ) -> None:
        ttk.Label(parent, text=axis, width=3, font=("TkDefaultFont", 11, "bold")).grid(
            row=row, column=0, sticky="w", padx=(0, 8), pady=5
        )
        scale = tk.Scale(
            parent,
            from_=minimum,
            to=maximum,
            resolution=0.001,
            orient="horizontal",
            showvalue=False,
            length=390,
            command=lambda _value, name=axis: self.update_value_label(name),
        )
        scale.set(0.0)
        scale.grid(row=row, column=1, sticky="ew", pady=3)
        label = ttk.Label(parent, text="0.000 m", width=10, anchor="e")
        label.grid(row=row, column=2, sticky="e", padx=(8, 0))
        parent.columnconfigure(1, weight=1)
        self.scales[axis] = scale
        self.value_labels[axis] = label

    def update_value_label(self, axis: str) -> None:
        value = self.scales[axis].get()
        self.value_labels[axis].configure(text=f"{value:+.3f} m")

    def current_target(self) -> tuple[float, float, float]:
        return (
            round(self.scales["X"].get(), 3),
            round(self.scales["Y"].get(), 3),
            round(self.scales["Z"].get(), 3),
        )

    def send_xyz(self) -> None:
        if not self.xy_client.server_is_ready():
            self.status.set("XY indisponible : attendre l’activation des contrôleurs.")
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
        self.status.set(f"Commande envoyée : X={x:+.3f}, Y={y:+.3f}, Z={z:+.3f} m")

    def send_pen_up(self) -> None:
        self.scales["Z"].set(0.0)
        self.update_value_label("Z")
        self.pen_target = 0.0
        self.status.set("Stylo levé; la position X/Y n’a pas changé.")

    def advance_pen(self) -> None:
        """Move the pen command smoothly within its configured velocity limit."""
        difference = self.pen_target - self.pen_position
        if abs(difference) < 1e-9:
            return
        step = max(-0.001, min(0.001, difference))
        self.pen_position = round(self.pen_position + step, 6)
        self.pen_publisher.publish(
            Float64MultiArray(data=[self.pen_position])
        )

    def center(self) -> None:
        for axis in ("X", "Y", "Z"):
            self.scales[axis].set(0.0)
            self.update_value_label(axis)
        self.send_xyz()

    def random_position(self) -> None:
        self.scales["X"].set(round(random.uniform(-0.08, 0.08), 3))
        self.scales["Y"].set(round(random.uniform(-0.08, 0.08), 3))
        self.scales["Z"].set(round(random.uniform(self.Z_MIN, self.Z_MAX), 3))
        for axis in ("X", "Y", "Z"):
            self.update_value_label(axis)
        self.send_xyz()

    def cancel_xy(self) -> None:
        if self.goal_handle is None:
            self.status.set("Aucune trajectoire XY active à annuler.")
            return
        self.goal_handle.cancel_goal_async()
        self.goal_handle = None
        self.status.set("Annulation de la trajectoire XY demandée.")

    def on_goal_response(self, future: Future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # Report ROS action errors in the GUI.
            message = f"Erreur XY : {exc}"
            self.request_pending = False
            if not self.closing:
                self.root.after(0, lambda text=message: self.status.set(text))
            return

        if not goal_handle.accepted:
            self.request_pending = False
            if not self.closing:
                self.root.after(
                    0,
                    lambda: self.status.set("Le contrôleur XY a refusé la commande."),
                )
            return

        self.goal_handle = goal_handle
        self.request_pending = False
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
        if not self.closing:
            self.root.after(0, lambda text=message: self.status.set(text))

    def update_controller_status(self) -> None:
        if self.closing:
            return
        if not self.xy_client.server_is_ready() and not self.request_pending:
            self.status.set("Attente du contrôleur XY… lance la simulation Gazebo.")
        elif self.xy_client.server_is_ready() and not self.request_pending:
            if self.status.get().startswith("Attente"):
                self.status.set("Contrôleur XY prêt. Choisir X/Y/Z puis envoyer la commande.")
        self.root.after(750, self.update_controller_status)

    def close(self) -> None:
        self.closing = True
        self.pen_timer.cancel()
        self.root.destroy()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Node("drawbot_xyz_control_gui")
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
        if not gui.closing:
            gui.close()
        executor.shutdown()
        spin_thread.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

from types import SimpleNamespace
from collections import deque
import xml.etree.ElementTree as ET
import unittest
from unittest.mock import patch

from control_msgs.action import FollowJointTrajectory
from drawbot_control.controller_startup import controller_actions_after_spawn
from drawbot_control.drawing_paths import PCB_GREETING_PATHS, make_word_paths
from drawbot_control.pcb_drawing_demo import DrawNode
from drawbot_control.robot_geometry import GEOMETRY, PEN_CONTACT, PEN_DOWN_POSITION
from drawbot_control.trace_visualization import (
    TRACE_SEGMENTS_PER_MODEL,
    make_trace_batch_sdf,
    set_trace_factory_pose,
)
from ros_gz_interfaces.srv import SpawnEntity
from drawbot_control.xyz_control_node import PEN_STEP, XYZControlGUI
from drawbot_control.xyz_control_view import XYZControlView
from launch.actions import LogError


class FakeFuture:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callback = callback


class FakeActionClient:
    def __init__(self, goal_handle=None):
        self.goal_handle = goal_handle
        self.goal = None

    def send_goal_async(self, goal):
        self.goal = goal
        return FakeFuture(self.goal_handle)


class TestDrawingWorkflows(unittest.TestCase):
    def test_manual_control_window_is_raised_on_start(self):
        class FakeRoot:
            def __init__(self):
                self.calls = []

            def deiconify(self):
                self.calls.append(("deiconify",))

            def attributes(self, *args):
                self.calls.append(("attributes", *args))

            def lift(self):
                self.calls.append(("lift",))

            def focus_force(self):
                self.calls.append(("focus_force",))

            def after(self, delay, callback):
                self.calls.append(("after", delay))
                self.delayed_callback = callback

        view = XYZControlView.__new__(XYZControlView)
        view.root = FakeRoot()

        view._raise_window_on_start()

        self.assertIn(("deiconify",), view.root.calls)
        self.assertIn(("attributes", "-topmost", True), view.root.calls)
        self.assertIn(("lift",), view.root.calls)
        self.assertIn(("focus_force",), view.root.calls)
        self.assertIn(("after", 1200), view.root.calls)
        view.root.delayed_callback()
        self.assertIn(("attributes", "-topmost", False), view.root.calls)

    def test_print_uses_real_time_speeds_and_pen_stays_below_joint_limit(self):
        self.assertEqual(XYZControlGUI.PRINT_SPEED, 0.025)
        self.assertEqual(XYZControlGUI.TRAVEL_SPEED, 0.10)
        self.assertEqual(XYZControlGUI.MIN_PRINT_SEGMENT_TIME, 0.5)
        self.assertEqual(XYZControlGUI.MIN_TRAVEL_TIME, 1.0)
        self.assertAlmostEqual(PEN_STEP / 0.01, 0.05)
        self.assertLessEqual(PEN_STEP / 0.01, 0.15)

    def test_greeting_words_fit_inside_printable_pcb_area(self):
        self.assertEqual(len(PCB_GREETING_PATHS), 2)
        self.assertGreater(len(PCB_GREETING_PATHS[0]), 0)
        self.assertGreater(len(PCB_GREETING_PATHS[1]), 0)

        all_points = [
            point
            for word_strokes in PCB_GREETING_PATHS
            for stroke in word_strokes
            for point in stroke
        ]
        x_min, x_max = GEOMETRY["limits"]["x"]
        y_min, y_max = GEOMETRY["limits"]["y"]
        self.assertTrue(all(x_min <= x <= x_max for x, _ in all_points))
        self.assertTrue(all(y_min <= y <= y_max for _, y in all_points))
        self.assertEqual(len(make_word_paths("SALAMA", center_y=0.027)), 9)
        self.assertEqual(len(make_word_paths("TONGASOA", center_y=-0.027)), 11)

    def test_greeting_layout_is_parallel_to_y_centered_and_not_clipped(self):
        column_centers = []
        for word_strokes in PCB_GREETING_PATHS:
            points = [point for stroke in word_strokes for point in stroke]
            x_values = [x for x, _ in points]
            y_values = [y for _, y in points]
            x_span = max(x_values) - min(x_values)
            y_span = max(y_values) - min(y_values)

            self.assertGreater(y_span, 5.0 * x_span)
            self.assertLess(y_span, 0.18)
            column_centers.append((min(x_values) + max(x_values)) / 2.0)

        self.assertGreater(column_centers[0], 0.0)
        self.assertLess(column_centers[1], 0.0)
        self.assertLess(abs(column_centers[0] + column_centers[1]), 1e-6)
        self.assertLess(
            max(abs(x) for x in column_centers),
            0.05,
        )

    def test_word_path_builder_rejects_unsupported_characters(self):
        with self.assertRaises(ValueError):
            make_word_paths("SALAMA!", center_y=0.0)

    def test_greeting_word_rotation_is_minus_90_degrees_toward_negative_y(self):
        unrotated = make_word_paths("S", center_y=0.027)
        rotated = make_word_paths(
            "S",
            center_y=0.027,
            orientation_degrees=-90.0,
        )

        self.assertEqual(len(unrotated), len(rotated))
        for original_stroke, rotated_stroke in zip(unrotated, rotated):
            self.assertEqual(len(original_stroke), len(rotated_stroke))
            for (x, y), (rotated_x, rotated_y) in zip(
                original_stroke,
                rotated_stroke,
            ):
                self.assertAlmostEqual(rotated_x, y)
                self.assertAlmostEqual(rotated_y, -x)

    def test_draw_node_sends_xy_trajectory_and_checks_result(self):
        result = SimpleNamespace(
            result=SimpleNamespace(
                error_code=FollowJointTrajectory.Result.SUCCESSFUL,
            ),
        )
        goal_handle = SimpleNamespace(
            accepted=True,
            get_result_async=lambda: FakeFuture(result),
        )
        node = DrawNode.__new__(DrawNode)
        node.cb_group = object()
        node.xy_client = FakeActionClient(goal_handle)
        node.get_logger = lambda: SimpleNamespace(
            info=lambda *args: None,
            error=lambda *args: None,
        )
        points = [(0.0, 0.0, False), (0.03, 0.02, True)]

        with patch("drawbot_control.pcb_drawing_demo.rclpy.spin_until_future_complete"):
            self.assertTrue(node.send_trajectory(points))

        trajectory = node.xy_client.goal.trajectory
        self.assertEqual(trajectory.joint_names, ["joint_x", "joint_y"])
        self.assertEqual(
            [list(point.positions) for point in trajectory.points],
            [[0.0, 0.0], [0.03, 0.02]],
        )
        self.assertEqual(
            [point.time_from_start.sec for point in trajectory.points], [1, 1]
        )
        self.assertEqual(
            [list(point.velocities) for point in trajectory.points],
            [[0.0, 0.0], [0.0, 0.0]],
        )

    def test_draw_node_raises_pen_and_stops_on_trajectory_failure(self):
        node = DrawNode.__new__(DrawNode)
        calls = []
        node.send_trajectory = lambda points: calls.append(points) or False
        node.move_pen = lambda position: calls.append(("pen", position))
        node.get_logger = lambda: SimpleNamespace(
            info=lambda *args: None,
            error=lambda *args: None,
        )

        outcome = node.execute_drawing([(0.01, -0.02, False)])

        self.assertFalse(outcome)
        self.assertEqual(calls[0], [(0.01, -0.02, False)])
        self.assertEqual(calls[-1], ("pen", 0.0))

    def test_gui_send_xyz_sends_xy_goal_and_sets_pen_target(self):
        class Scale:
            def __init__(self, value):
                self.value = value

            def get(self):
                return self.value

        class Status:
            def set(self, value):
                self.value = value

        gui = XYZControlGUI.__new__(XYZControlGUI)
        gui.printing = False
        gui.request_pending = False
        gui.xy_client = FakeActionClient()
        gui.xy_client.server_is_ready = lambda: True
        gui.scales = {
            "X": Scale(0.0124),
            "Y": Scale(-0.0346),
            "Z": Scale(-0.0062),
        }
        gui.pen_target = 0.0
        gui.status = Status()

        gui.send_xyz()

        goal = gui.xy_client.goal
        self.assertEqual(goal.trajectory.joint_names, ["joint_x", "joint_y"])
        self.assertEqual(
            list(goal.trajectory.points[0].positions), [0.012, -0.035]
        )
        self.assertEqual(goal.trajectory.points[0].time_from_start.sec, 1)
        self.assertEqual(gui.pen_target, -0.006)
        self.assertTrue(gui.request_pending)

    def test_start_print_updates_view_controls_without_legacy_button_attribute(self):
        class FakeView:
            def __init__(self):
                self.enabled = True

            def set_controls_enabled(self, enabled):
                self.enabled = enabled

        class FakeStatus:
            def set(self, message):
                self.message = message

        class FakeServiceClient:
            def service_is_ready(self):
                return True

        gui = XYZControlGUI.__new__(XYZControlGUI)
        gui.printing = False
        gui.request_pending = False
        gui.xy_client = FakeActionClient()
        gui.xy_client.server_is_ready = lambda: True
        gui.trace_spawn_client = FakeServiceClient()
        gui.measured_positions = {
            "joint_x": 0.0,
            "joint_y": 0.0,
            "joint_pen": 0.0,
        }
        gui.view = FakeView()
        gui.status = FakeStatus()
        gui.print_trace_index = 0
        gui.print_wait_token = 0
        gui.print_cancel_requested = False
        gui.print_paths = ()
        gui._set_print_pen_target = lambda target: setattr(gui, "pen_target", target)
        gui._wait_for_pen = lambda target, callback: setattr(gui, "wait_target", target)

        gui.start_print()

        self.assertTrue(gui.printing)
        self.assertFalse(gui.view.enabled)
        self.assertEqual(gui.pen_target, gui.PEN_UP)
        self.assertEqual(gui.wait_target, gui.PEN_UP)

    def test_finish_print_restores_view_controls_without_legacy_button_attribute(self):
        class FakeView:
            def __init__(self):
                self.enabled = False

            def set_controls_enabled(self, enabled):
                self.enabled = enabled

        class FakeStatus:
            def set(self, message):
                self.message = message

        gui = XYZControlGUI.__new__(XYZControlGUI)
        gui.print_wait_token = 0
        gui.printing = True
        gui.print_cancel_requested = False
        gui.print_abort_message = ""
        gui.request_pending = False
        gui.goal_handle = object()
        gui.view = FakeView()
        gui.status = FakeStatus()

        gui._finish_print("Impression terminée.")

        self.assertFalse(gui.printing)
        self.assertIsNone(gui.goal_handle)
        self.assertTrue(gui.view.enabled)
        self.assertEqual(gui.status.message, "Impression terminée.")

    def test_print_trajectory_builds_monotonic_waypoints(self):
        gui = XYZControlGUI.__new__(XYZControlGUI)
        gui.xy_client = FakeActionClient()
        gui.xy_client.server_is_ready = lambda: True
        gui.request_pending = False
        gui._on_print_goal_response = lambda future, callback: None

        gui._send_print_trajectory(
            [(0.01, 0.02, 0.5), (0.03, 0.04, 1.25)],
            lambda: None,
        )

        trajectory = gui.xy_client.goal.trajectory
        self.assertEqual(trajectory.joint_names, ["joint_x", "joint_y"])
        self.assertEqual(
            [list(point.positions) for point in trajectory.points],
            [[0.01, 0.02], [0.03, 0.04]],
        )
        self.assertEqual(
            [point.time_from_start.sec for point in trajectory.points], [0, 1]
        )
        self.assertEqual(
            [point.time_from_start.nanosec for point in trajectory.points],
            [500_000_000, 250_000_000],
        )
        self.assertTrue(gui.request_pending)

    def test_controller_launch_is_gated_by_spawn_exit_code(self):
        control_launch = object()

        self.assertEqual(
            controller_actions_after_spawn(0, control_launch), [control_launch]
        )
        failed_actions = controller_actions_after_spawn(1, control_launch)
        self.assertEqual(len(failed_actions), 1)
        self.assertIsInstance(failed_actions[0], LogError)

    def test_trace_models_are_kept_without_eviction_across_batches(self):
        class FakeRoot:
            def after(self, _delay, callback):
                self.callback = callback

        class FakeSpawnClient:
            def __init__(self):
                self.requests = []

            def service_is_ready(self):
                return True

            def call_async(self, request):
                self.requests.append(request)
                return FakeFuture(SimpleNamespace(success=True))

        gui = XYZControlGUI.__new__(XYZControlGUI)
        gui.trace_spawn_in_flight = False
        gui.trace_spawn_segments = deque(
            ((index * 0.001, 0.0), ((index + 1) * 0.001, 0.0))
            for index in range(TRACE_SEGMENTS_PER_MODEL + 1)
        )
        gui.trace_spawn_client = FakeSpawnClient()
        gui.root = FakeRoot()
        gui.robot_base_x = 0.12
        gui.robot_base_y = -0.03
        gui.robot_base_z = 0.41
        gui.trace_model_prefix = "pcb_trace_test"
        gui.trace_model_counter = 0

        gui._spawn_next_trace_segment()
        gui._on_trace_segment_spawned(FakeFuture(SimpleNamespace(success=True)))
        gui.root.callback()

        self.assertEqual(len(gui.trace_spawn_client.requests), 2)
        batch_visual_counts = [
            len(ET.fromstring(request.entity_factory.sdf).findall(".//visual"))
            for request in gui.trace_spawn_client.requests
        ]
        self.assertEqual(batch_visual_counts, [TRACE_SEGMENTS_PER_MODEL, 1])
        self.assertEqual(len(gui.trace_spawn_segments), 0)
        self.assertEqual(
            [request.entity_factory.name for request in gui.trace_spawn_client.requests],
            ["pcb_trace_test_0000", "pcb_trace_test_0001"],
        )

    def test_trace_batch_contains_one_visual_per_segment(self):
        sdf = make_trace_batch_sdf(
            [((0.0, 0.0), (0.01, 0.0)), ((0.01, 0.0), (0.01, 0.01))],
            "trace_batch",
        )
        root = ET.fromstring(sdf)

        self.assertEqual(root.find(".//model").attrib["name"], "trace_batch")
        self.assertEqual(len(root.findall(".//visual")), 2)
        self.assertEqual(
            root.findtext(".//material/diffuse"),
            "1.000 0.240 0.000 1",
        )
        self.assertEqual(
            root.findtext(".//material/emissive"),
            "1.000 0.120 0.000 1",
        )
        self.assertEqual(
            root.findtext(".//material/specular"),
            "0.250 0.100 0.010 1",
        )
        sizes = [
            [float(value) for value in visual.findtext("geometry/box/size").split()]
            for visual in root.findall(".//visual")
        ]
        self.assertEqual(sizes[0][1:], [0.003, 0.0006])
        self.assertEqual(sizes[1][1:], [0.003, 0.0006])
        self.assertGreater(
            GEOMETRY["drawing"]["trace_clearance"],
            0.00002,
        )
        self.assertEqual(root.findtext(".//model/pose"), "0 0 0 0 0 0")
        expected_visual_z = (
            GEOMETRY["base"]["thickness"]
            + GEOMETRY["pcb"]["thickness"]
            + GEOMETRY["drawing"]["trace_height"] / 2.0
            + GEOMETRY["drawing"]["trace_clearance"]
        )
        actual_visual_z = float(root.find(".//visual/pose").text.split()[2])
        self.assertAlmostEqual(actual_visual_z, expected_visual_z)

    def test_trace_spawn_request_uses_robot_world_pose(self):
        request = SpawnEntity.Request()

        set_trace_factory_pose(request.entity_factory, 0.12, -0.03, 0.41)

        pose = request.entity_factory.pose
        self.assertAlmostEqual(pose.position.x, 0.12)
        self.assertAlmostEqual(pose.position.y, -0.03)
        self.assertAlmostEqual(pose.position.z, 0.41)
        self.assertAlmostEqual(pose.orientation.w, 1.0)

    def test_pen_contact_creates_visible_orange_dot_before_movement(self):
        class FakeRoot:
            def after(self, _delay, _callback):
                pass

        gui = XYZControlGUI.__new__(XYZControlGUI)
        gui.closing = False
        gui.measured_positions = {
            "joint_x": 0.01,
            "joint_y": -0.02,
            "joint_pen": gui.PEN_DOWN,
        }
        gui.live_trace_last_point = None
        gui.trace_spawn_segments = deque()
        gui.trace_spawn_in_flight = True
        gui.root = FakeRoot()

        gui._monitor_pen_contact()

        self.assertEqual(len(gui.trace_spawn_segments), 1)
        start, end = gui.trace_spawn_segments[0]
        self.assertAlmostEqual(end[0] - start[0], gui.TRACE_DOT_SIZE)
        self.assertAlmostEqual((start[0] + end[0]) / 2.0, 0.01)
        self.assertAlmostEqual(start[1], -0.02)
        self.assertAlmostEqual(end[1], -0.02)

    def test_derived_pen_positions_match_shared_robot_geometry(self):
        self.assertAlmostEqual(PEN_CONTACT, -0.006)
        self.assertAlmostEqual(
            PEN_DOWN_POSITION,
            PEN_CONTACT - GEOMETRY["drawing"]["pen_compression"],
        )

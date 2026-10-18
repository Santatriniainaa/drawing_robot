"""Gazebo visual geometry helpers for rendered pen traces."""

from math import atan2, hypot

from drawbot_control.robot_geometry import GEOMETRY

TRACE_SEGMENTS_PER_MODEL = 128


def make_trace_batch_sdf(
    segments: list[tuple[tuple[float, float], tuple[float, float]]],
    model_name: str,
) -> str:
    """Create one static Gazebo model containing multiple visual segments."""
    base = GEOMETRY["base"]["thickness"]
    pcb_thickness = GEOMETRY["pcb"]["thickness"]
    trace = GEOMETRY["drawing"]
    red, green, blue = trace["trace_color"]
    emissive_red, emissive_green, emissive_blue = trace["trace_emissive"]
    pcb_top_local_z = base + pcb_thickness
    center_z = (
      pcb_top_local_z
      + trace["trace_height"] / 2.0
      + trace["trace_clearance"]
    )
    visuals = []
    for index, (start, end) in enumerate(segments):
        x1, y1 = start
        x2, y2 = end
        length = hypot(x2 - x1, y2 - y1)
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        yaw = atan2(y2 - y1, x2 - x1)
        visuals.append(
            f"""      <visual name='trace_{index:03d}'>
        <pose>{center_x:.6f} {center_y:.6f} {center_z:.6f} 0 0 {yaw:.9f}</pose>
        <geometry><box><size>{length:.6f} {trace['trace_width']:.6f} {trace['trace_height']:.6f}</size></box></geometry>
        <material>
          <ambient>{red:.3f} {green:.3f} {blue:.3f} 1</ambient>
          <diffuse>{red:.3f} {green:.3f} {blue:.3f} 1</diffuse>
          <specular>0.250 0.100 0.010 1</specular>
          <emissive>{emissive_red:.3f} {emissive_green:.3f} {emissive_blue:.3f} 1</emissive>
        </material>
      </visual>"""
        )
    return f"""<sdf version='1.9'>
  <model name='{model_name}'>
    <static>true</static>
    <pose>0 0 0 0 0 0</pose>
    <link name='trace_link'>
{''.join(visuals)}
    </link>
  </model>
</sdf>"""


def set_trace_factory_pose(
  entity_factory: object,
  robot_base_x: float,
  robot_base_y: float,
  robot_base_z: float,
) -> None:
  """Place the trace model at the robot base frame in the Gazebo world."""
  pose = entity_factory.pose
  pose.position.x = robot_base_x
  pose.position.y = robot_base_y
  pose.position.z = robot_base_z
  pose.orientation.x = 0.0
  pose.orientation.y = 0.0
  pose.orientation.z = 0.0
  pose.orientation.w = 1.0

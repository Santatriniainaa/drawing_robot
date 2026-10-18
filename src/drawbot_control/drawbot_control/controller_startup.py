"""Launch helpers for sequencing Gazebo robot startup."""

from launch.actions import LogError


def controller_actions_after_spawn(returncode: int, control_launch: object) -> list[object]:
    """Start controllers only when ros_gz_sim create exits successfully."""
    if returncode == 0:
        return [control_launch]
    return [
        LogError(
            msg=(
                "Robot spawn failed (ros_gz_sim create exit code "
                f"{returncode}); controller startup was skipped."
            ),
        ),
    ]

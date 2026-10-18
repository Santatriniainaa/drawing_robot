from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _spawn_controller(controller_name: str, controller_file):
    arguments = [controller_name, '--controller-manager', '/controller_manager']
    if controller_name != 'joint_state_broadcaster':
        arguments.extend(['--param-file', controller_file])
    return Node(
        package='controller_manager',
        executable='spawner',
        arguments=arguments,
        output='screen',
    )


def generate_launch_description():
    controllers_file = PathJoinSubstitution([
        FindPackageShare('drawbot_control'),
        'config',
        'drawbot_controllers.yaml',
    ])

    spawn_jsb = _spawn_controller('joint_state_broadcaster', controllers_file)
    spawn_xy = _spawn_controller('xy_controller', controllers_file)
    spawn_pen = _spawn_controller('pen_controller', controllers_file)

    return LaunchDescription([
        spawn_jsb,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_jsb,
                on_exit=[spawn_xy],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_xy,
                on_exit=[spawn_pen],
            )
        ),
    ])

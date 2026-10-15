from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_file = PathJoinSubstitution([
        FindPackageShare('drawbot_control'),
        'config',
        'drawbot_controllers.yaml'
    ])

    # Spawner pour le joint_state_broadcaster
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
        ],
        output='screen'
    )

    # Spawner pour le contrôleur XY (trajectoire)
    spawn_xy = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'xy_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_file,
        ],
        output='screen'
    )

    # Spawner pour le contrôleur du stylo (position directe)
    spawn_pen = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'pen_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_file,
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_jsb,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_jsb,
                on_exit=[spawn_xy]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_xy,
                on_exit=[spawn_pen]
            )
        ),
    ])
    
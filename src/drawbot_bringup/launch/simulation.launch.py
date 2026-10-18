from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _declare_bool_argument(name: str, description: str, default: str = 'true'):
    return DeclareLaunchArgument(
        name,
        default_value=default,
        description=description,
    )


def generate_launch_description():
    pkg_gazebo = FindPackageShare('drawbot_gazebo')
    pkg_description = FindPackageShare('drawbot_description')

    launch_arguments = [
        _declare_bool_argument('rviz', 'Lancer RViz2'),
        _declare_bool_argument('draw', 'Lancer le dessin automatique', 'false'),
        _declare_bool_argument('markers', 'Lancer le visualiseur de marqueurs'),
        _declare_bool_argument('xyz_gui', 'Lancer le panneau graphique X/Y/Z du robot'),
    ]

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'xyz_gui': LaunchConfiguration('xyz_gui')}.items(),
    )

    rviz_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_description, 'launch', 'display.launch.py'])
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'publish_robot_description': 'false',
                    'use_joint_state_publisher_gui': 'false',
                }.items(),
                condition=IfCondition(LaunchConfiguration('rviz')),
            )
        ],
    )

    draw_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drawbot_control',
                executable='draw_node',
                name='draw_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
        condition=IfCondition(LaunchConfiguration('draw')),
    )

    marker_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='drawbot_control',
                executable='marker_publisher',
                name='marker_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}],
            )
        ],
        condition=IfCondition(LaunchConfiguration('markers')),
    )

    return LaunchDescription([
        *launch_arguments,
        gazebo_launch,
        rviz_launch,
        draw_node,
        marker_node,
    ])

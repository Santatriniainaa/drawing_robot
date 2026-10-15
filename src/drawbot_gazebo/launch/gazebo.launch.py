from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_drawbot_gazebo = FindPackageShare('drawbot_gazebo')
    pkg_drawbot_description = FindPackageShare('drawbot_description')

    # Chemin vers le monde
    world_file = PathJoinSubstitution([
        pkg_drawbot_gazebo, 'worlds', 'drawing_world.sdf'
    ])

    # Lancer Gazebo Jetty (gz sim)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gz_sim, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': ['-r -v 4 ', world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        pkg_drawbot_description, 'urdf', 'drawbot.urdf.xacro'
                    ])
                ]),
                value_type=str
            ),
            'use_sim_time': True
        }],
        output='screen'
    )

    # Spawn le robot dans Gazebo (ros_gz_sim)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'drawing_world',
            '-topic', 'robot_description',
            '-name', 'drawbot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.41',  # Origine base au niveau du dessus de table
            '-Y', '0.0'    # Orientation
        ],
        output='screen'
    )

    # Bridge pour le clock (nécessaire pour use_sim_time)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        clock_bridge,
        spawn_entity,
    ])
    
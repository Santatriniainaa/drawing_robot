from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='drawbot_description').find('drawbot_description')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'drawbot.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_robot_description = LaunchConfiguration('publish_robot_description')
    use_joint_state_publisher_gui = LaunchConfiguration(
        'use_joint_state_publisher_gui'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Utiliser /clock; false pour un affichage autonome'
    )
    publish_robot_description_arg = DeclareLaunchArgument(
        'publish_robot_description', default_value='true',
        description='Lancer robot_state_publisher'
    )
    use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
        'use_joint_state_publisher_gui', default_value='true',
        description='Lancer joint_state_publisher_gui'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    'xacro ',
                    os.path.join(pkg_share, 'urdf', 'drawbot.urdf.xacro')
                ]),
                value_type=str
            ),
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(publish_robot_description)
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_joint_state_publisher_gui)
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig', default=default_rviz_config)],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        publish_robot_description_arg,
        use_joint_state_publisher_gui_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz
    ])
    
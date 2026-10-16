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
    use_joint_state_publisher_gui = LaunchConfiguration('use_joint_state_publisher_gui')

    # VS Code installed as a Snap injects GTK paths that can load Snap's
    # libpthread into system ROS executables and cause a GLIBC_PRIVATE error.
    clean_snap_gui_env = (
        'env -u GTK_EXE_PREFIX -u GTK_PATH -u GTK_MODULES '
        '-u GTK_IM_MODULE_FILE -u GIO_MODULE_DIR -u GSETTINGS_SCHEMA_DIR '
        '-u LOCPATH -u XDG_CONFIG_DIRS -u XDG_DATA_DIRS'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Utiliser /clock; false pour un affichage autonome'
    )
    publish_robot_description_arg = DeclareLaunchArgument(
        'publish_robot_description', 
        default_value='true',
        description='Lancer robot_state_publisher'
    )
    use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
        'use_joint_state_publisher_gui', 
        default_value='true',
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
        prefix=clean_snap_gui_env,
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_joint_state_publisher_gui)
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        prefix=clean_snap_gui_env,
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
    
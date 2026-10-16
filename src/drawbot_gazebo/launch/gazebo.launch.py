from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, UnsetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

ROBOT_SPAWN_X = 0.0
ROBOT_SPAWN_Y = 0.0
ROBOT_SPAWN_Z = 0.41

def generate_launch_description():
    pkg_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_drawbot_gazebo = FindPackageShare('drawbot_gazebo')
    pkg_drawbot_description = FindPackageShare('drawbot_description')
    pkg_drawbot_control = FindPackageShare('drawbot_control')
    xyz_gui = LaunchConfiguration('xyz_gui')

    # VS Code's Snap GTK overrides can inject an incompatible Snap libpthread
    # into Gazebo's GUI and ROS GUI nodes.
    clean_snap_environment = [
        UnsetEnvironmentVariable(name)
        for name in (
            'GTK_EXE_PREFIX',
            'GTK_PATH',
            'GTK_MODULES',
            'GTK_IM_MODULE_FILE',
            'GIO_MODULE_DIR',
            'GSETTINGS_SCHEMA_DIR',
            'LOCPATH',
            'XDG_CONFIG_DIRS',
            'XDG_DATA_DIRS',
        )
    ]

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
            '-x', str(ROBOT_SPAWN_X),
            '-y', str(ROBOT_SPAWN_Y),
            '-z', str(ROBOT_SPAWN_Z),  # La base repose sur la table.
            '-Y', '0.0'    # Orientation
        ],
        output='screen'
    )

    # ros2_control runs in the spawned Gazebo model. Start the broadcaster
    # and controllers only after Gazebo has inserted that model.
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_drawbot_control, 'launch', 'control.launch.py'
            ])
        )
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[control_launch],
        )
    )

    xyz_gui_arg = DeclareLaunchArgument(
        'xyz_gui',
        default_value='true',
        description='Lancer le panneau X/Y/Z pour commander le robot dans Gazebo',
    )
    xyz_gui_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='drawbot_control',
                executable='xyz_control_gui',
                name='drawbot_xyz_control_gui',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'robot_base_x': ROBOT_SPAWN_X,
                    'robot_base_y': ROBOT_SPAWN_Y,
                    'robot_base_z': ROBOT_SPAWN_Z,
                }],
                condition=IfCondition(xyz_gui),
            )
        ],
    )

    # Bridge pour le clock (nécessaire pour use_sim_time)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/drawing_world/create@ros_gz_interfaces/srv/SpawnEntity',
        ],
        output='screen'
    )

    return LaunchDescription([
        xyz_gui_arg,
        *clean_snap_environment,
        gz_sim,
        robot_state_publisher,
        clock_bridge,
        start_controllers,
        spawn_entity,
        xyz_gui_node,
    ])
    
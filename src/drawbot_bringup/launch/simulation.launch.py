from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_gazebo = FindPackageShare('drawbot_gazebo')
    pkg_control = FindPackageShare('drawbot_control')
    pkg_description = FindPackageShare('drawbot_description')

    # Arguments
    use_rviz_arg = DeclareLaunchArgument(
        'rviz', 
        default_value='true',
        description='Lancer RViz2'
    )

    use_draw_arg = DeclareLaunchArgument(
        'draw', 
        default_value='false',
        description='Lancer le dessin automatique (désactivé par défaut pour la commande manuelle)'
    )

    use_markers_arg = DeclareLaunchArgument(
        'markers', 
        default_value='true',
        description='Lancer le visualiseur de marqueurs'
    )

    use_xyz_gui_arg = DeclareLaunchArgument(
        'xyz_gui',
        default_value='true',
        description='Lancer le panneau graphique X/Y/Z du robot'
    )

    # 1. Gazebo + Robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, 'launch', 'gazebo.launch.py'])
        )
    )

    # 2. Contrôleurs (après Gazebo, délai 2s)
    control_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_control, 'launch', 'control.launch.py'])
                )
            )
        ]
    )

    # 3. RViz (après 3s pour laisser le robot spawner)
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
                condition=IfCondition(LaunchConfiguration('rviz'))
            )
        ]
    )

    # 4. Nœud de dessin (après 5s pour laisser les contrôleurs démarrer)
    draw_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drawbot_control',
                executable='draw_node',
                name='draw_node',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ],
        condition=IfCondition(LaunchConfiguration('draw'))
    )

    # 5. Visualisation des marqueurs (après 3s)
    marker_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='drawbot_control',
                executable='marker_publisher',
                name='marker_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ],
        condition=IfCondition(LaunchConfiguration('markers'))
    )

    xyz_gui_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='drawbot_control',
                executable='xyz_control_gui',
                name='drawbot_xyz_control_gui',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ],
        condition=IfCondition(LaunchConfiguration('xyz_gui'))
    )

    return LaunchDescription([
        use_rviz_arg,
        use_draw_arg,
        use_markers_arg,
        use_xyz_gui_arg,
        gazebo_launch,
        control_launch,
        rviz_launch,
        draw_node,
        marker_node,
        xyz_gui_node,
    ])
    
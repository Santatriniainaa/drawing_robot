# drawbot_gazebo

Package Gazebo Sim de Drawbot : monde de travail SDF et lancement qui démarre Gazebo, publie `robot_description`, crée le robot et relie `/clock` à ROS.

## Contenu

- `worlds/drawing_world.sdf` : table, PCB, éclairage et sol définis localement; aucun téléchargement de `model://sun` ou `model://ground_plane` requis.
- `launch/gazebo.launch.py` : lance Gazebo Sim 10, le `robot_state_publisher`, le bridge `/clock` et le spawn dans `drawing_world`.

## Utilisation

Pour Gazebo seul :

```bash
colcon build --symlink-install --packages-up-to drawbot_gazebo
source install/setup.bash
ros2 launch drawbot_gazebo gazebo.launch.py
```

Pour Gazebo avec RViz, contrôleurs et panneau XYZ, utiliser plutôt depuis la racine du workspace :

```bash
colcon build --symlink-install --packages-up-to drawbot_bringup
source install/setup.bash
ros2 launch drawbot_bringup simulation.launch.py
```

La simulation testée a chargé le monde, créé le modèle et activé `joint_state_broadcaster`, `xy_controller` et `pen_controller`. Le contact stylo/PCB reste à contrôler visuellement.

## Dépendances principales

`ros_gz_sim`, `ros_gz_bridge`, `drawbot_description`, `launch`, `launch_ros`, `robot_state_publisher`, `xacro`, `rosgraph_msgs` et `gz_ros2_control`.

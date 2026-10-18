# drawbot_gazebo

Package Gazebo Sim de Drawbot : monde de travail SDF et lancement qui démarre Gazebo, publie `robot_description`, crée le robot et relie `/clock` à ROS.

## Contenu

- `worlds/drawing_world.sdf` : table, éclairage et sol définis localement; le PCB est un lien fixe du robot décrit dans `drawbot_description`. Aucun téléchargement de `model://sun` ou `model://ground_plane` n'est requis.
- `launch/gazebo.launch.py` : lance Gazebo Sim 10, `robot_state_publisher`, le bridge `/clock`, le robot, les contrôleurs ros2_control et le panneau de commande X/Y/Z.
- Le lancement utilise `controller_startup.py` pour démarrer les contrôleurs uniquement si `ros_gz_sim create` réussit. Il expose le service Gazebo `create` utilisé pour ajouter les traces persistantes au monde.
- Les dimensions mécaniques et les coordonnées de spawn viennent de `drawbot_control/config/drawbot_geometry.yaml`, partagées avec le modèle Xacro.

## Utilisation

Pour Gazebo avec ses contrôleurs, le panneau GUI X/Y/Z et le `joint_state_broadcaster` (retour d’état `/joint_states` vers `robot_state_publisher`) :

```bash
colcon build --symlink-install --packages-up-to drawbot_gazebo
source install/setup.bash
ros2 launch drawbot_gazebo gazebo.launch.py
```

Le panneau X/Y/Z commande réellement les articulations du robot dans Gazebo. Le `joint_state_broadcaster` reste actif uniquement pour le retour d’état vers ROS/RViz; ce n’est pas un deuxième contrôleur. Ne lancez pas `joint_state_publisher_gui` en simulation. Pour ajouter RViz et les marqueurs, lancer depuis la racine du workspace :

```bash
colcon build --symlink-install --packages-up-to drawbot_bringup
source install/setup.bash
ros2 launch drawbot_bringup simulation.launch.py
```

La simulation vérifiée précédemment a chargé le monde, créé le modèle et activé `joint_state_broadcaster`, `xy_controller` et `pen_controller`. Le contact stylo/PCB et le service de suppression des traces restent à valider visuellement en intégration.

## Dépendances principales

`ros_gz_sim`, `ros_gz_bridge`, `drawbot_description`, `launch`, `launch_ros`, `robot_state_publisher`, `xacro`, `rosgraph_msgs` et `gz_ros2_control`.

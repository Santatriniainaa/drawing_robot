# drawbot_description

Description du robot Drawbot : modèle Xacro/URDF, interfaces ros2_control et configuration RViz.

## Contenu

- `urdf/drawbot.urdf.xacro` : portique cartésien avec `joint_x`, `joint_y` et `joint_pen`.
- `launch/display.launch.py` : lance RViz; en mode autonome, lance aussi `robot_state_publisher` et `joint_state_publisher_gui`.
- `rviz/drawbot.rviz` : configuration d’affichage du robot, TF et marqueurs.

Limites décrites dans le Xacro : $X,Y\in[-0.10,0.10]$ m; `joint_pen` entre −0,025 et +0,020 m. Le contact Z calculé à partir de la géométrie est environ −0,0075 m et reste à confirmer dans Gazebo.

## Utilisation autonome dans RViz

Depuis la racine du workspace :

```bash
colcon build --symlink-install --packages-select drawbot_description
source install/setup.bash
ros2 launch drawbot_description display.launch.py
```

Cette commande utilise l’horloge système (`use_sim_time=false`) et publie TF, nécessaire au Fixed Frame `base_link`. En simulation, `drawbot_bringup` utilise le temps simulé et désactive les publishers locaux pour éviter les doublons.

## Dépendances

`xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `urdf`, `launch`, `launch_ros`, `drawbot_control` et `gz_ros2_control`.

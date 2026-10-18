# drawbot_description

Description du robot Drawbot : modèle Xacro/URDF, interfaces ros2_control et configuration RViz.

## Contenu

- `urdf/drawbot.urdf.xacro` : portique cartésien avec `joint_x`, `joint_y` et `joint_pen`.
- Les dimensions du robot et les limites articulaires sont chargées depuis `drawbot_control/config/drawbot_geometry.yaml`; évitez de dupliquer ces valeurs dans le Xacro ou les nœuds Python.
- `launch/display.launch.py` : lance RViz; en mode autonome, lance aussi `robot_state_publisher` et `joint_state_publisher_gui`.
- `rviz/drawbot.rviz` : configuration d’affichage du robot, TF et marqueurs.

Le PCB de 250 × 250 × 1 mm est un lien fixe du robot, posé sur le plateau supérieur de `base_link`; le monde Gazebo ne contient donc plus de PCB indépendant sur la table. Quatre colonnes soutiennent les rails relevés. D’après les origines des joints, le dessous de la pointe sphérique est à Z=+0,027 m lorsque `joint_pen=0`, tandis que le dessus du PCB est à Z=+0,021 m : le stylo a donc 6 mm de dégagement et le contact géométrique est à `joint_pen≈−0,006 m`. Limites décrites dans le Xacro : $X,Y\in[-0.10,0.10]$ m et `joint_pen` entre −0,0065 et +0,020 m.

## Utilisation autonome dans RViz

Depuis la racine du workspace :

```bash
colcon build --symlink-install --packages-select drawbot_description
source install/setup.bash
ros2 launch drawbot_description display.launch.py
```

Cette commande utilise l’horloge système (`use_sim_time=false`) et publie TF, nécessaire au Fixed Frame `base_link`. En simulation, lancer `ros2 launch drawbot_bringup simulation.launch.py` : Gazebo démarre `joint_state_broadcaster`, qui publie `/joint_states`; le lancement simulation désactive le GUI joint-state publisher pour éviter deux sources d’états.

## Dépendances

`xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `urdf`, `launch`, `launch_ros`, `drawbot_control` et `gz_ros2_control`.

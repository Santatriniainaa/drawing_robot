# drawbot_control

Package Python ROS 2 pour ros2_control, commande manuelle X/Y/Z et démonstrations de dessin.

## Contenu

- `config/drawbot_controllers.yaml` configure `joint_state_broadcaster`, `xy_controller` (`joint_x`, `joint_y`) et `pen_controller` (`joint_pen`).
- `launch/control.launch.py` démarre les spawners en séquence et fournit explicitement le YAML aux contrôleurs.
- `drawbot_control/draw_node.py` exécute un motif PCB prédéfini via l’action `/xy_controller/follow_joint_trajectory` et `/pen_controller/commands` (`std_msgs/Float64MultiArray`).
- `drawbot_control/xyz_control_gui.py` fournit une fenêtre avec curseurs X/Y/Z, centre, cible aléatoire et annulation XY.
- `drawbot_control/marker_publisher.py` publie un `MarkerArray` sur `/draw_markers`.
- `drawbot_control/pcb_generator.py` publie un exemple JSON sur `/draw_commands`.

## GUI X/Y/Z

La plage X/Y est ±0,10 m; le panneau limite Z à −0,0075…+0,020 m. Z=0 lève le stylo. Le minimum est le contact calculé pour la géométrie actuelle et doit être ajusté si le modèle ou la surface change. Le panneau fait progresser Z par incréments selon l’horloge simulée afin de respecter la vitesse du joint.

Le launch complet du workspace démarre cette GUI par défaut (`xyz_gui:=true`) et laisse `draw:=false`. Ne lance pas simultanément la GUI manuelle et `draw_node`, car ils pilotent les mêmes contrôleurs.

## Build / commandes autonomes

Depuis la racine, ROS 2 sourcé :

```bash
colcon build --symlink-install --packages-select drawbot_control
source install/setup.bash
ros2 run drawbot_control xyz_control_gui
```

Les spawners seuls (`ros2 launch drawbot_control control.launch.py`) exigent que Gazebo ait déjà créé le robot et `/controller_manager`.

## Tests

```bash
colcon test --packages-select drawbot_control
colcon test-result --verbose
```

Les tests actuels sont surtout des linters/type-checks; le test copyright est désactivé et aucun test d’intégration n’est présent.

## Limite fonctionnelle

`pcb_generator` publie `/draw_commands`, mais aucun abonné ne traduit actuellement ces JSON en trajectoires. `draw_node` exécute son propre motif fixe.

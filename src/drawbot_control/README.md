# drawbot_control

Package Python ROS 2 pour ros2_control, commande manuelle X/Y/Z et démonstrations de dessin.

## Contenu

- `config/drawbot_controllers.yaml` configure `joint_state_broadcaster`, `xy_controller` (`joint_x`, `joint_y`) et `pen_controller` (`joint_pen`).
- `launch/control.launch.py` démarre les spawners en séquence et fournit explicitement le YAML aux contrôleurs.
- `drawbot_control/draw_node.py` exécute un motif PCB prédéfini via l’action `/xy_controller/follow_joint_trajectory` et `/pen_controller/commands` (`std_msgs/Float64MultiArray`).
- `drawbot_control/xyz_control_gui.py` fournit une fenêtre avec curseurs X/Y/Z, centre, cible aléatoire, annulation et bouton **Print PCB**.
- `drawbot_control/marker_publisher.py` publie un `MarkerArray` sur `/draw_markers`.

## GUI X/Y/Z

La plage X/Y est ±0,10 m; le panneau limite Z à −0,0065…+0,020 m. Z=0 lève le stylo. Avec la plaque de 1 mm posée sur une base de 20 mm et la pointe sphérique de 1,5 mm, le contact géométrique est à Z≈−0,0060 m; l’impression commande −0,00615 m pour un très léger appui simulé. Le panneau fait progresser Z par incréments afin de respecter la vitesse du joint.

Le launch complet démarre cette GUI par défaut (`xyz_gui:=true`) et laisse `draw:=false`. **Print PCB** exécute trois pistes de démonstration dans la zone ±75 × ±60 mm, puis quatre ancres circulaires de repérage de 5 mm de diamètre centrées à ±85 mm : le stylo reste levé pendant les déplacements, descend au contact, suit le tracé et se relève. Dès que la pointe mesurée touche la plaque verte, la GUI ajoute progressivement des segments orange au monde Gazebo, alignés sur le PCB et sans collision; ils restent visibles jusqu’à la fermeture du monde. Le même affichage suit aussi un contact dessiné avec les commandes manuelles. Le bouton d’annulation arrête la trajectoire et demande le relevage du stylo. Il s'agit d'une visualisation simulée, pas d’un dépôt d’encre réel. Ne lance pas simultanément l’impression GUI et `draw_node`, car ils pilotent les mêmes contrôleurs.

Le bouton **Print PCB** envoie des trajectoires aux contrôleurs Gazebo; ce n'est pas une simple animation GUI. Les pistes sont définies dans `XYZControlGUI.PRINT_TRACES` et les repères dans `XYZControlGUI.ANCHOR_CENTERS`, en mètres dans les limites X/Y configurées.

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

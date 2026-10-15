# Drawbot

Workspace ROS 2 pour simuler un robot cartésien de dessin PCB : axes $X$, $Y$ et stylo $Z$. Environnement testé : ROS 2 Lyrical et Gazebo Sim 10.

## Packages

| Package                                                 | Contenu                                     |
| ------------------------------------------------------- | ------------------------------------------- |
| [drawbot_description](src/drawbot_description/README.md) | Xacro/URDF, RViz et affichage autonome.     |
| [drawbot_gazebo](src/drawbot_gazebo/README.md)           | Monde SDF, Gazebo Sim et spawn du robot.    |
| [drawbot_control](src/drawbot_control/README.md)         | ros2_control, panneau XYZ et nœuds Python. |
| [drawbot_bringup](src/drawbot_bringup/README.md)         | Lancement intégré.                        |

Les ressources suivent la disposition ROS 2 conventionnelle : `launch/`, `urdf/`, `rviz/`, `worlds/`, `config/` et le module `drawbot_control/`. Les dossiers `build/`, `install/` et `log/` sont générés par colcon.

## Prérequis

- ROS 2 Lyrical, Gazebo Sim compatible, `colcon` et `rosdep`.
- `gz_ros2_control`, `ros_gz_sim`, `ros_gz_bridge`, `ros2_control` et `ros2_controllers` compatibles.
- `python3-tk` pour la fenêtre XYZ.

Si rosdep ne connaît pas la clé `ament_python` sur cette installation, ignorer cette seule clé :

```bash
export ROS_DISTRO=lyrical
source "/opt/ros/${ROS_DISTRO}/setup.bash"
rosdep install --from-paths src --ignore-src --rosdistro "${ROS_DISTRO}" --skip-keys ament_python -r -y
```

## Compiler

# Drawbot — robot cartésien de dessin

Drawbot est un workspace **ROS 2** pour modéliser, simuler et commander un robot cartésien destiné à des démonstrations de tracé sur PCB. Le portique se déplace sur les axes **X/Y** et utilise un stylo commandé sur l’axe **Z**. La simulation repose sur Gazebo Sim et `ros2_control`.

> **État du projet :** prototype de simulation. Le modèle et les commandes sont conçus pour Gazebo ; aucune interface avec un robot physique n’est fournie.

## Fonctionnalités

- Description du robot paramétrée en Xacro/URDF et affichage dans RViz.
- Monde de travail SDF avec table et surface PCB.
- Contrôle des axes avec `ros2_control` : trajectoires X/Y et commande du stylo Z.
- Panneau graphique pour envoyer des cibles X/Y/Z et annuler une trajectoire XY.
- Nœuds de démonstration pour un motif fixe et des marqueurs RViz.

## Architecture du workspace

| Package                 | Responsabilité                                                              | Documentation                                         |
| ----------------------- | ---------------------------------------------------------------------------- | ----------------------------------------------------- |
| `drawbot_description` | Modèle Xacro/URDF, configuration RViz et affichage autonome.                | [README du package](src/drawbot_description/README.md) |
| `drawbot_gazebo`      | Monde SDF, démarrage de Gazebo, création du robot et bridge de l’horloge. | [README du package](src/drawbot_gazebo/README.md)      |
| `drawbot_control`     | Configuration `ros2_control`, GUI XYZ et nœuds de démonstration.         | [README du package](src/drawbot_control/README.md)     |
| `drawbot_bringup`     | Lancements intégrés de la simulation et de l’affichage seul.              | [README du package](src/drawbot_bringup/README.md)     |

## Compilation

Depuis la racine du workspace :

```bash
source "/opt/ros/${ROS_DISTRO}/setup.bash"
colcon build --symlink-install --packages-up-to drawbot_bringup
source install/setup.bash
```

`--packages-up-to drawbot_bringup` compile le package de lancement et ses dépendances. Après une nouvelle compilation, sourcer de nouveau `install/setup.bash` dans chaque terminal ROS utilisé.

## Démarrage rapide

Lancer la simulation complète :

```bash
ros2 launch drawbot_bringup simulation.launch.py
```

Par défaut, ce lancement démarre Gazebo, le robot, les contrôleurs, RViz, les marqueurs de démonstration et la fenêtre **Drawbot — commande XYZ**. Le dessin automatique est désactivé par défaut pour éviter que la démonstration et la GUI commandent simultanément les mêmes contrôleurs.

Dans la GUI, régler les valeurs X/Y/Z, puis sélectionner **Envoyer XYZ**. **Centre (stylo levé)** ramène X/Y à zéro avec le stylo relevé ; **Aléatoire** choisit une cible bornée ; **Annuler XY** annule la trajectoire XY en cours.

### Arguments du lancement intégré

| Argument    | Valeur par défaut | Description                                    |
| ----------- | -----------------: | ---------------------------------------------- |
| `rviz`    |           `true` | Active ou désactive RViz.                     |
| `markers` |           `true` | Active ou désactive les marqueurs d’exemple. |
| `xyz_gui` |           `true` | Active ou désactive la commande manuelle.     |
| `draw`    |          `false` | Lance le motif automatique prédéfini.        |

Exemples :

```bash
# Simulation sans RViz ni marqueurs
ros2 launch drawbot_bringup simulation.launch.py rviz:=false markers:=false

# Démonstration automatique, sans GUI manuelle
ros2 launch drawbot_bringup simulation.launch.py draw:=true xyz_gui:=false
```

Pour démarrer uniquement l’environnement Gazebo, utiliser `ros2 launch drawbot_gazebo gazebo.launch.py`. Pour afficher le robot seul dans RViz, utiliser `ros2 launch drawbot_description display.launch.py`.

## Modèle et interfaces de commande

Le modèle définit trois articulations prismatiques : `joint_x`, `joint_y` et `joint_pen`. Les plages du modèle sont :

| Axe       | Articulation  |   Plage configurée |
| --------- | ------------- | ------------------: |
| X         | `joint_x`   |   −0,10 à +0,10 m |
| Y         | `joint_y`   |   −0,10 à +0,10 m |
| Z (stylo) | `joint_pen` | −0,025 à +0,020 m |

La GUI limite la commande Z à −0,0075…+0,020 m. **Z = 0 relève le stylo** ; −0,0075 m est une estimation géométrique du contact avec la surface, pas une valeur calibrée. Vérifier visuellement le contact dans Gazebo avant toute démonstration et réajuster la valeur si la géométrie change.

| Interface ROS                              | Rôle                                                                         |
| ------------------------------------------ | ----------------------------------------------------------------------------- |
| `/joint_states`                          | États des articulations publiés par `joint_state_broadcaster`.            |
| `/xy_controller/follow_joint_trajectory` | Action de trajectoire des axes X et Y.                                        |
| `/pen_controller/commands`               | Commandes de position du stylo, de type `std_msgs/Float64MultiArray`.       |
| `/draw_markers`                          | Marqueurs d’exemple de type `visualization_msgs/MarkerArray` pour RViz.    |
| `/draw_commands`                         | Exemple de commandes de dessin encodées en JSON dans un `std_msgs/String`. |

## Démonstrations et limites connues

- `draw_node` exécute un motif PCB fixe via les contrôleurs. Il partage ces contrôleurs avec la GUI : utiliser l’un ou l’autre, pas les deux en même temps.
- `pcb_generator` publie des exemples JSON sur `/draw_commands`, mais aucun nœud ne convertit actuellement ces messages en trajectoires ; ce générateur ne déclenche donc pas le dessin.
- Les marqueurs RViz illustrent des tracés, mais ne représentent pas une trace réellement déposée sur la surface PCB. Aucune visualisation persistante du dessin effectué n’est implémentée.
- Le contact du stylo et le comportement mécanique restent à valider dans la simulation. Ce dépôt ne fournit pas de pilote ou de configuration de sécurité pour du matériel réel.
- Les tests automatisés ne couvrent pas encore les trajectoires complètes ni l’intégration Gazebo/contrôleurs.

## Tests

Les tests déclarés dans le package de contrôle peuvent être lancés ainsi :

```bash
colcon test --packages-select drawbot_control
colcon test-result --verbose
```

## Licence

Les packages déclarent la licence **Apache-2.0**. Voir les fichiers `package.xml` de chaque package pour les métadonnées associées.

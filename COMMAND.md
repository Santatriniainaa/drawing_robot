# Drawbot — Guide complet des commandes

Ce document regroupe les commandes d’installation, de compilation, de lancement, de commande du robot, de dessin et de diagnostic du workspace ROS 2 **Drawbot** sous Ubuntu. Les chemins et commandes ci-dessous correspondent à ce dépôt et à ROS 2 Lyrical / Gazebo Sim 10.

> **Important — terminal Snap/VS Code :** RViz, Gazebo ou Tk peuvent hériter de chemins GTK de VS Code Snap et charger une bibliothèque incompatible. Pour toutes les commandes ci-dessous, démarrez avec un environnement ROS propre, tel que décrit à l’étape 0. Ne lancez qu’une seule simulation Gazebo à la fois.

## 0. Ouvrir un terminal ROS propre

Dans chaque nouveau terminal utilisé pour ROS, entrer à la racine du workspace :

```bash
cd /home/santatra/Documents/drawing_robot
unset LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/lyrical/setup.bash
source install/setup.bash
```

- `cd` place le terminal dans le workspace.
- `unset LD_LIBRARY_PATH LD_PRELOAD` écarte les bibliothèques injectées par l’environnement Snap. Les scripts ROS reconstituent ensuite leurs propres chemins nécessaires.
- `source /opt/ros/lyrical/setup.bash` charge les outils ROS 2 Lyrical.
- `source install/setup.bash` rend les packages de ce workspace visibles dans le terminal. Cette ligne est à refaire après chaque nouveau terminal et après compilation.

> Après une compilation propre, `install/setup.bash` n’existe pas encore. Dans ce cas, sourcez seulement ROS avant de lancer les commandes de build, puis sourcez le workspace après.

## 1. Installer les prérequis

### 1.1 Vérifier que ROS 2 Lyrical est installé

```bash
ls /opt/ros/lyrical/setup.bash
```

Si le fichier existe, ROS est installé dans l’emplacement attendu. Sinon, installez ROS 2 Lyrical en suivant sa documentation officielle avant de poursuivre. Les commandes de ce guide ne peuvent pas installer ROS à votre place.

### 1.2 Installer `rosdep` et `colcon`

```bash
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions python3-pip python3-tk
```

- `apt update` met à jour la liste des paquets Ubuntu.
- `python3-rosdep` résout les dépendances des packages ROS.
- `python3-colcon-common-extensions` fournit l’outil de compilation du workspace.
- `python3-pip` est utile pour les outils Python facultatifs.
- `python3-tk` fournit Tkinter, requis par le panneau XYZ.

Initialiser rosdep une seule fois par installation Ubuntu :

```bash
sudo rosdep init
rosdep update
```

Si `rosdep init` répond que l’initialisation existe déjà, ne le relancez pas : continuez avec `rosdep update`.

### 1.3 Installer les dépendances des packages

```bash
cd /home/santatra/Documents/drawing_robot
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro lyrical -r -y --skip-keys ament_python
```

Cette commande examine tous les `package.xml` sous `src/`, installe les dépendances système manquantes, et ne tente pas d’installer les packages locaux du workspace. `--skip-keys ament_python` évite une clé rosdep qui peut être absente du catalogue de certaines installations Lyrical; le système de build Python ROS n’est pas remplacé par cette commande.

### 1.4 Dépendances principales utilisées

Le projet attend une installation ROS 2 Lyrical compatible avec :

- Gazebo Sim 10 et les paquets ROS `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces` ;
- `gz_ros2_control`, `ros2_control`, `ros2_controllers` et `controller_manager` ;
- `xacro`, `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2` ;
- `python3-tk` pour le panneau **Drawbot — commande XYZ**.

## 2. Compiler le workspace

### Compilation complète du projet

```bash
cd /home/santatra/Documents/drawing_robot
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to drawbot_bringup
source install/setup.bash
```

`--packages-up-to drawbot_bringup` compile `drawbot_bringup` et tous les packages dont il dépend : contrôle, description et Gazebo. `--symlink-install` rend les ressources locales plus faciles à modifier pendant le développement. Sourcer `install/setup.bash` après le build charge les packages nouvellement installés.

### Recompiler un seul package

Après modification du panneau Python :

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select drawbot_control
source install/setup.bash
```

Pour ne compiler que le modèle robot :

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select drawbot_description
source install/setup.bash
```

Pour ne compiler que le lancement Gazebo :

```bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select drawbot_gazebo
source install/setup.bash
```

Après chaque changement de code, préférez le build complet `--packages-up-to drawbot_bringup` si vous n’êtes pas sûr des dépendances affectées.

## 3. Démarrer la simulation complète — utilisation recommandée

Dans un terminal préparé selon l’étape 0 :

```bash
ros2 launch drawbot_bringup simulation.launch.py
```

Ce lancement démarre une seule fois :

1. Gazebo Sim et le monde de travail;
2. le robot Drawbot avec son PCB fixé sur la base;
3. `robot_state_publisher` et le bridge `/clock`;
4. `joint_state_broadcaster`, `xy_controller` et `pen_controller`;
5. RViz, les marqueurs d’exemple et le panneau XYZ.

Le panneau XYZ ouvre **une fenêtre séparée** de Gazebo. Il commande les axes du robot simulé. Le `joint_state_broadcaster` fournit uniquement le retour mesuré sur `/joint_states`; ne démarrez pas `joint_state_publisher_gui` dans la simulation.

### Options du lancement complet

| Argument | Valeur par défaut | Fonction |
|---|---:|---|
| `rviz` | `true` | Ouvre RViz avec le modèle et les TF simulés. |
| `markers` | `true` | Affiche l’exemple de marqueurs dans RViz. |
| `xyz_gui` | `true` | Ouvre la GUI avec boutons **Envoyer XYZ**, **Print PCB**, etc. |
| `draw` | `false` | Lance le motif automatique `draw_node` au lieu d’une utilisation manuelle. |

Exemples :

```bash
# Simulation minimale, sans RViz, marqueurs ni GUI XYZ
ros2 launch drawbot_bringup simulation.launch.py rviz:=false markers:=false xyz_gui:=false

# RViz et la commande XYZ, sans marqueurs d’exemple
ros2 launch drawbot_bringup simulation.launch.py markers:=false

# Dessin automatique : désactiver la GUI manuelle pour éviter deux commandes concurrentes
ros2 launch drawbot_bringup simulation.launch.py draw:=true xyz_gui:=false
```

## 4. Démarrer Gazebo seul

Dans un terminal préparé selon l’étape 0 :

```bash
ros2 launch drawbot_gazebo gazebo.launch.py
```

Ce lancement inclut Gazebo, le robot, les contrôleurs et la GUI XYZ. RViz et les marqueurs RViz ne sont pas démarrés. Pour ajouter RViz et toutes les options de démonstration, utilisez plutôt le lancement complet de l’étape 3.

Désactiver le panneau XYZ si nécessaire :

```bash
ros2 launch drawbot_gazebo gazebo.launch.py xyz_gui:=false
```

**Ne lancez pas en parallèle** `drawbot_gazebo gazebo.launch.py` et `drawbot_bringup simulation.launch.py` : le second inclut déjà le premier et les deux tenteraient de créer un monde/robot identique.

## 5. Utiliser la GUI XYZ et imprimer le circuit

Avec la simulation en cours :

- **Envoyer XYZ** : envoie les valeurs des curseurs au robot.
- **Centre (stylo levé)** : recentre X/Y avec le stylo relevé.
- **Aléatoire** : teste une position dans la zone configurée.
- **Print PCB** : dessine trois pistes de démonstration puis quatre ancres circulaires de repérage de 5 mm, aux positions ±85 mm. Les traces sont ajoutées au monde Gazebo en orange après chaque piste terminée.
- **Annuler impression** / **Annuler XY** : annule la trajectoire en cours et demande le relevage du stylo.

La carte fait 250 × 250 × 1 mm. Les coordonnées des pistes et ancres sont exprimées en mètres relativement au centre du PCB. La course X/Y configurée est de −0,10 à +0,10 m; les positions d’ancrage à ±0,085 m restent à l’intérieur de cette zone.

Le stylo est relevé à Z = 0. Le contact géométrique calculé est environ −0,0060 m; le bouton **Print PCB** commande actuellement −0,00615 m pour un très léger appui de simulation. Les visuels orange sont persistants jusqu’à la fermeture du monde, sans collision. Ils simulent des pistes/repères; Gazebo ne dépose pas d’encre réelle.

> Les tests précédents montrent que le contrôleur peut signaler brièvement une limitation de mouvement XY lorsque sa trajectoire exige une vitesse supérieure à la limite. Le contrôleur limite la commande; vérifiez le suivi visuellement avant de modifier la vitesse.

## 6. RViz seul — affichage sans simulation

Dans un terminal ROS préparé selon l’étape 0 :

```bash
ros2 launch drawbot_description display.launch.py
```

Ce mode lance `robot_state_publisher`, `joint_state_publisher_gui` et RViz avec l’horloge système. Le GUI des articulations sert à manipuler le modèle d’affichage; il ne commande pas le robot Gazebo.

Afficher seulement le robot-state publisher et RViz, sans le panneau d’articulations :

```bash
ros2 launch drawbot_description display.launch.py use_joint_state_publisher_gui:=false
```

Afficher le robot dans RViz sans démarrer un second `robot_state_publisher` :

```bash
ros2 launch drawbot_description display.launch.py publish_robot_description:=false use_joint_state_publisher_gui:=false
```

Cette dernière option est utile uniquement si une autre instance publie déjà `robot_description` et `/tf`, par exemple la simulation complète.

## 7. Commandes ROS utiles pendant la simulation

Exécuter les commandes ci-dessous dans un **deuxième terminal**, préparé selon l’étape 0, pendant que la simulation reste ouverte.

### Vérifier les nœuds et contrôleurs

```bash
ros2 node list
ros2 control list_controllers
```

Les trois contrôleurs attendus sont `joint_state_broadcaster`, `xy_controller` et `pen_controller`; ils doivent être en état `active`.

### Vérifier l’état et les topics

```bash
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
```

- `/joint_states` doit contenir `joint_x`, `joint_y` et `joint_pen`.
- Quitter la commande de fréquence avec Ctrl+C.

### Envoyer une position Z manuellement

```bash
ros2 topic pub --once /pen_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

La valeur `0.0` demande de relever le stylo. Pour le test de contact géométrique très léger, la position actuelle estimée est `-0.00615` m; tester prudemment dans Gazebo et ne pas dépasser la limite basse `-0.0065` m.

### Effacer une trajectoire XY active

Le panneau possède un bouton d’annulation. Pour arrêter la commande, utilisez **Annuler XY** ou **Annuler impression** selon le mode. Évitez d’envoyer des actions ROS concurrentes pendant une impression en cours.

### Voir les interfaces ROS

```bash
ros2 action list
ros2 service list
ros2 topic info /joint_states
ros2 topic info /pen_controller/commands
```

Le service `/world/drawing_world/create` est le service Gazebo utilisé par la GUI pour ajouter les segments visualisés.

## 8. Nœuds et commandes autonomes

Ces commandes supposent que Gazebo est déjà lancé et que ses contrôleurs sont actifs. Ne démarrez qu’un seul outil qui commande les mêmes joints à la fois.

### Lancer seulement la GUI XYZ

```bash
ros2 run drawbot_control xyz_control_gui
```

Cette GUI exige les contrôleurs, `/joint_states`, `/clock` et `/world/drawing_world/create` de la simulation active.

### Lancer le motif automatique `draw_node`

Démarrer Gazebo seul sans GUI XYZ, puis dans un autre terminal préparé :

```bash
ros2 run drawbot_control draw_node
```

`draw_node` joue son motif fixe. Ne l’exécutez pas simultanément avec la GUI XYZ ni avec `draw:=true`.

### Publier l’exemple JSON PCB

```bash
ros2 run drawbot_control pcb_generator
```

Cela publie un exemple sur `/draw_commands`. **Aucun abonné actuel ne convertit `/draw_commands` en mouvement robot** : cette commande ne démarre donc pas l’impression.

### Démarrer les contrôleurs seuls

```bash
ros2 launch drawbot_control control.launch.py
```

À utiliser seulement si Gazebo a déjà créé le robot et `/controller_manager`, et si les contrôleurs ne sont pas déjà lancés. Le lancement Gazebo démarre automatiquement le `joint_state_broadcaster` et les deux contrôleurs de commande.

## 9. Tests et diagnostics de compilation

### Lint et tests du package de contrôle

```bash
source /opt/ros/lyrical/setup.bash
colcon test --packages-select drawbot_control
colcon test-result --verbose
```

Les tests automatiques actuels sont principalement des linters/type-checks. Il n’existe pas encore de test d’intégration complet de Gazebo et du bouton **Print PCB**.

### Nettoyer les résultats de compilation

```bash
cd /home/santatra/Documents/drawing_robot
rm -rf build install log
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-up-to drawbot_bringup
source install/setup.bash
```

`rm -rf build install log` supprime les produits générés uniquement (pas `src/`). N’utilisez cette opération que si vous voulez refaire un build complet propre; tous les terminaux doivent ensuite re-sourcer `install/setup.bash`.

### Vérifier un package ROS

```bash
ros2 pkg list | grep '^drawbot_'
ros2 pkg prefix drawbot_control
ros2 pkg prefix drawbot_description
ros2 pkg prefix drawbot_gazebo
ros2 pkg prefix drawbot_bringup
```

## 10. Arrêter proprement

Pour fermer seulement la fenêtre XYZ, utilisez son bouton de fermeture : la GUI annule la trajectoire XY éventuelle et relève progressivement le stylo en respectant la limite de vitesse avant de quitter.

Pour arrêter toute la simulation, utilisez ensuite le terminal du lancement et appuyez une fois sur :

```text
Ctrl+C
```

Attendez que Gazebo, RViz, les contrôleurs et les bridges se ferment avant de lancer une nouvelle simulation. Ne démarrez jamais `gazebo.launch.py` en même temps que `simulation.launch.py`; ce dernier démarre déjà Gazebo. Les messages `Another world of the same name is running`, `Controller already loaded` ou `Failed to activate controller` indiquent généralement qu’une autre simulation utilise encore le même monde ou le même `/controller_manager`. Fermez l’ancienne simulation avant de relancer.

## Résumé des terminaux recommandés

### Terminal 1 — simulation et impression manuelle

```bash
cd /home/santatra/Documents/drawing_robot
unset LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch drawbot_bringup simulation.launch.py
```

Ce terminal reste occupé pendant la simulation. Utilisez le bouton **Print PCB** dans la fenêtre de commande XYZ.

### Terminal 2 — observation ROS facultative

```bash
cd /home/santatra/Documents/drawing_robot
unset LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 topic echo /joint_states
```

Ctrl+C dans le terminal 2 arrête uniquement l’observation du topic; la simulation du terminal 1 continue.

### Terminal 3 — commandes de vérification facultatives

```bash
cd /home/santatra/Documents/drawing_robot
unset LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 control list_controllers
```

Ces terminaux supplémentaires ne démarrent pas une deuxième simulation.

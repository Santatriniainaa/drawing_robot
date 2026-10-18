# drawbot_bringup

Package d’orchestration ROS 2 pour démarrer Gazebo, les contrôleurs, RViz et les outils Drawbot.

## Lancements

- `simulation.launch.py` démarre Gazebo, les contrôleurs, RViz, les marqueurs et le panneau XYZ.
- `robot_only.launch.py` démarre uniquement l’affichage fourni par `drawbot_description`.

Le lancement de simulation n’active les contrôleurs qu’après un code de sortie réussi du processus de création du robot. En cas d’échec du spawn, il journalise l’erreur et saute le démarrage des contrôleurs.

Arguments de `simulation.launch.py` :

| Argument | Défaut | Rôle |
|---|---:|---|
| `rviz` | `true` | Affichage RViz. |
| `draw` | `false` | Dessin automatique de démonstration. |
| `markers` | `true` | Exemple de marqueurs PCB dans RViz. |
| `xyz_gui` | `true` | Panneau manuel X/Y/Z lancé par `gazebo.launch.py`. |

Le dessin automatique est désactivé par défaut, car il utilise les mêmes contrôleurs que le panneau XYZ. Pour l’activer seul, lancer `draw:=true xyz_gui:=false`.

## Build et lancement

Depuis la racine du workspace, après avoir sourcé ROS 2 et installé les dépendances :

```bash
colcon build --symlink-install --packages-up-to drawbot_bringup
source install/setup.bash
ros2 launch drawbot_bringup simulation.launch.py
```

Pour désactiver RViz ou le panneau, ajouter `rviz:=false` ou `xyz_gui:=false`. Voir le [README racine](../../README.md) pour les prérequis et le résumé des validations.

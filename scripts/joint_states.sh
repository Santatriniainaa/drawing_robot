#!/usr/bin/env bash
# Display live /joint_states feedback as a compact terminal dashboard.

set -Eeuo pipefail

readonly WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ROS_SETUP="/opt/ros/lyrical/setup.bash"
readonly INSTALL_SETUP="$WORKSPACE/install/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    printf 'Erreur : ROS 2 Lyrical introuvable : %s\n' "$ROS_SETUP" >&2
    exit 1
fi

if [[ ! -f "$INSTALL_SETUP" ]]; then
    printf 'Erreur : workspace non compilé. Lancez d’abord colcon build.\n' >&2
    exit 1
fi

# Avoid Snap-injected libraries and ROS Python modules leaking in from .venv.
unset LD_LIBRARY_PATH LD_PRELOAD PYTHONHOME VIRTUAL_ENV
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$INSTALL_SETUP"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
    printf 'Erreur : ros2 introuvable après le chargement de ROS 2.\n' >&2
    exit 1
fi

if [[ ! -t 1 ]]; then
    printf 'Info : sortie redirigée; le tableau sera actualisé à chaque message.\n'
fi

interactive=0
cursor_home=''
clear_to_end=''
if [[ -t 1 && -n "${TERM:-}" && "${TERM:-dumb}" != 'dumb' ]]; then
    interactive=1
    cursor_home="$(tput cup 0 0 2>/dev/null || true)"
    clear_to_end="$(tput ed 2>/dev/null || true)"
fi

printf 'Lecture de /joint_states. Ctrl+C pour arrêter.\n'
printf 'Gazebo et le joint_state_broadcaster doivent être actifs.\n'

ros2 topic echo --csv --no-lost-messages \
    /joint_states sensor_msgs/msg/JointState |
    awk -F',' '
        NF >= 7 {
            joint_count = (NF - 3) / 4
            if (joint_count < 1 || joint_count != int(joint_count)) {
                next
            }

            if (interactive == 1) {
                printf "%s%s", cursor_home, clear_to_end
            }

            printf "Drawbot — mesures des articulations\n"
            printf "Topic : /joint_states    Repère : %s\n", $3
            printf "Temps simulation : %s.%09d s\n\n", $1, $2
            printf "%-20s %14s %14s %14s\n", \
                "ARTICULATION", "POSITION (m)", "VITESSE (m/s)", "EFFORT"
            printf "--------------------------------------------------------------------\n"

            for (joint_index = 0; joint_index < joint_count; joint_index++) {
                name = $(4 + joint_index)
                position = $(4 + joint_count + joint_index)
                velocity = $(4 + 2 * joint_count + joint_index)
                effort = $(4 + 3 * joint_count + joint_index)

                if (effort == "nan" || effort == "-nan" || effort == "nan(ind)") {
                    effort = "n/a"
                } else {
                    effort = sprintf("%.5f", effort)
                }

                printf "%-20s %14.6f %14.6f %14s\n", \
                    name, position, velocity, effort
            }
            fflush()
        }
    ' interactive="$interactive" cursor_home="$cursor_home" clear_to_end="$clear_to_end"

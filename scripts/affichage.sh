#!/usr/bin/env bash
# Display the Drawbot controller state in a compact terminal dashboard.

set -Ee
set -o pipefail

readonly WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
readonly ROS_SETUP="/opt/ros/lyrical/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    printf 'Erreur : ROS 2 Lyrical introuvable : %s\n' "$ROS_SETUP" >&2
    exit 1
fi

if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
    printf 'Erreur : workspace non compilé. Lancez d’abord colcon build.\n' >&2
    exit 1
fi

# Avoid loading libraries injected by VS Code Snap before sourcing ROS.
unset LD_LIBRARY_PATH LD_PRELOAD
# ROS setup files use optional variables that may be unset; allow those while
# sourcing, then restore nounset for the remainder of this script.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WORKSPACE/install/setup.bash"
set -u

if ! command -v ros2 >/dev/null 2>&1; then
    printf 'Erreur : ros2 introuvable après le chargement de ROS 2.\n' >&2
    exit 1
fi

if ! ros2 control list_controllers >/dev/null 2>&1; then
    printf 'Erreur : /controller_manager indisponible. Démarrez Gazebo avec :\n' >&2
    printf '  ros2 launch drawbot_gazebo gazebo.launch.py\n' >&2
    exit 1
fi

controller_output="$(ros2 control list_controllers)"
if [[ -z "$controller_output" ]]; then
    printf 'Aucun contrôleur chargé dans /controller_manager.\n' >&2
    exit 1
fi

if [[ -t 1 && "${NO_COLOR:-}" == "" ]]; then
    readonly COLOR_RESET=$'\033[0m'
    readonly COLOR_BOLD=$'\033[1m'
    readonly COLOR_GREEN=$'\033[32m'
    readonly COLOR_RED=$'\033[31m'
    readonly COLOR_YELLOW=$'\033[33m'
else
    readonly COLOR_RESET=''
    readonly COLOR_BOLD=''
    readonly COLOR_GREEN=''
    readonly COLOR_RED=''
    readonly COLOR_YELLOW=''
fi

printf '\n%sDrawbot — état des contrôleurs%s\n' "$COLOR_BOLD" "$COLOR_RESET"
printf 'Workspace : %s\n' "$WORKSPACE"
printf '%-26s %-37s %-12s\n' 'CONTRÔLEUR' 'TYPE' 'ÉTAT'
printf '%s\n' '─────────────────────────────────────────────────────────────────────────────'

while read -r name type state _; do
    [[ -n "${name:-}" ]] || continue

    case "${state:-unknown}" in
        active) color="$COLOR_GREEN" ;;
        inactive|unconfigured) color="$COLOR_YELLOW" ;;
        *) color="$COLOR_RED" ;;
    esac

    printf '%-26.26s %-37.37s %s%-12s%s\n' \
        "$name" "${type:--}" "$color" "${state:-unknown}" "$COLOR_RESET"
done <<< "$controller_output"

active_count="$(awk '$3 == "active" { count++ } END { print count + 0 }' <<< "$controller_output")"
total_count="$(awk 'NF { count++ } END { print count + 0 }' <<< "$controller_output")"
printf '\nRésumé : %s/%s contrôleurs actifs.\n\n' "$active_count" "$total_count"
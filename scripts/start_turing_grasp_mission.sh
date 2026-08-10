#!/usr/bin/env bash
set -euo pipefail

GRASP_WS="${GRASP_WS:-$HOME/grasp_ws}"
CAN_BIND_SCRIPT="${CAN_BIND_SCRIPT:-$HOME/can_bind.sh}"
CAN_IFACE="${CAN_IFACE:-can0}"
RUN_CAN_BIND="${RUN_CAN_BIND:-true}"
DRY_RUN="${DRY_RUN:-false}"

TRAJECTORY_FILE="${TRAJECTORY_FILE:-/home/guest/funny_lidar_slam/data/trajectories/example_path.yaml}"
USE_MISSION_SEQUENCER="${USE_MISSION_SEQUENCER:-true}"
USE_NAVIGATION="${USE_NAVIGATION:-true}"
USE_LIDAR="${USE_LIDAR:-true}"
USE_LOCALIZATION="${USE_LOCALIZATION:-true}"
USE_CHASSIS="${USE_CHASSIS:-true}"
USE_MAP_SERVER="${USE_MAP_SERVER:-true}"
USE_PATH_RVIZ="${USE_PATH_RVIZ:-true}"
USE_GRASP_SYSTEM="${USE_GRASP_SYSTEM:-true}"
USE_ARM_DRIVER="${USE_ARM_DRIVER:-true}"
USE_CAMERA="${USE_CAMERA:-true}"
USE_GRASP_RVIZ="${USE_GRASP_RVIZ:-false}"

log() {
    echo "[start_turing_grasp_mission] $*" >&2
}

die() {
    echo "[start_turing_grasp_mission] ERROR: $*" >&2
    exit 1
}

can_ready() {
    local details

    details="$(ip -details link show "$CAN_IFACE" 2>/dev/null || true)"
    [ -n "$details" ] || return 1
    grep -q "<.*UP" <<<"$details" || return 1
    grep -q "bitrate 1000000" <<<"$details" || return 1
}

[ -f "$GRASP_WS/env.sh" ] || die "missing $GRASP_WS/env.sh"
# shellcheck disable=SC1090
source "$GRASP_WS/env.sh"

command -v ros2 >/dev/null 2>&1 || die "ros2 not found after sourcing $GRASP_WS/env.sh"
[ -f "$TRAJECTORY_FILE" ] || die "missing trajectory file: $TRAJECTORY_FILE"

if [ "$DRY_RUN" != "true" ] && [ "$RUN_CAN_BIND" = "true" ]; then
    [ -f "$CAN_BIND_SCRIPT" ] || die "missing CAN bind script: $CAN_BIND_SCRIPT"
    log "configuring CAN synchronously via $CAN_BIND_SCRIPT"
    CAN_IFACE="$CAN_IFACE" bash "$CAN_BIND_SCRIPT"
fi

if [ "$DRY_RUN" != "true" ] && ! can_ready; then
    ip -details link show "$CAN_IFACE" >&2 || true
    die "$CAN_IFACE is not UP at bitrate 1000000; run CAN setup before launching"
fi

launch_cmd=(
    ros2 launch smart_grasp_bringup turing_grasp_mission.launch.py
    run_can_bind:=false
    can_port:="$CAN_IFACE"
    use_mission_sequencer:="$USE_MISSION_SEQUENCER"
    use_navigation:="$USE_NAVIGATION"
    use_lidar:="$USE_LIDAR"
    use_localization:="$USE_LOCALIZATION"
    use_chassis:="$USE_CHASSIS"
    use_map_server:="$USE_MAP_SERVER"
    use_path_rviz:="$USE_PATH_RVIZ"
    use_grasp_system:="$USE_GRASP_SYSTEM"
    use_arm_driver:="$USE_ARM_DRIVER"
    use_camera:="$USE_CAMERA"
    use_grasp_rviz:="$USE_GRASP_RVIZ"
    trajectory_file:="$TRAJECTORY_FILE"
)

if [ "$DRY_RUN" = "true" ]; then
    printf '[start_turing_grasp_mission] dry run:'
    printf ' %q' "${launch_cmd[@]}"
    printf '\n'
    exit 0
fi

log "starting turing grasp mission with $CAN_IFACE and trajectory=$TRAJECTORY_FILE"
exec "${launch_cmd[@]}"

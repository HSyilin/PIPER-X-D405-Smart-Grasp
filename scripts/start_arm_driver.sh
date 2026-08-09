#!/usr/bin/env bash
set -euo pipefail

GRASP_WS="${GRASP_WS:-$HOME/grasp_ws}"
CAN_BIND_SCRIPT="${CAN_BIND_SCRIPT:-$HOME/can_bind.sh}"
CAN_IFACE="${CAN_IFACE:-can0}"
RUN_CAN_BIND="${RUN_CAN_BIND:-true}"
DRY_RUN="${DRY_RUN:-false}"

ARM_TYPE="${ARM_TYPE:-piper_x}"
EFFECTOR_TYPE="${EFFECTOR_TYPE:-agx_gripper}"
AUTO_ENABLE="${AUTO_ENABLE:-false}"
SPEED_PERCENT="${SPEED_PERCENT:-10}"
GRIPPER_DEFAULT_EFFORT="${GRIPPER_DEFAULT_EFFORT:-0.5}"
CONTROL_ENABLED="${CONTROL_ENABLED:-false}"
ALLOW_REMOTE_DISABLE="${ALLOW_REMOTE_DISABLE:-false}"
HOME_JOINT_POSITIONS="${HOME_JOINT_POSITIONS:-[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
FIRMWARE_OVERRIDE="${FIRMWARE_OVERRIDE:-}"

log() {
    echo "[start_arm_driver] $*" >&2
}

die() {
    echo "[start_arm_driver] ERROR: $*" >&2
    exit 1
}

if [ "$HOME_JOINT_POSITIONS" = "[]" ] || [ -z "$HOME_JOINT_POSITIONS" ]; then
    HOME_JOINT_POSITIONS="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
fi

[ -f "$GRASP_WS/env.sh" ] || die "missing $GRASP_WS/env.sh"
# shellcheck disable=SC1090
source "$GRASP_WS/env.sh"

command -v ros2 >/dev/null 2>&1 || die "ros2 not found after sourcing $GRASP_WS/env.sh"

if [ "$DRY_RUN" != "true" ] && [ "$RUN_CAN_BIND" = "true" ]; then
    [ -x "$CAN_BIND_SCRIPT" ] || die "missing executable CAN bind script: $CAN_BIND_SCRIPT"
    log "configuring CAN via $CAN_BIND_SCRIPT"
    CAN_IFACE="$CAN_IFACE" bash "$CAN_BIND_SCRIPT"
fi

if [ "$DRY_RUN" != "true" ] &&
    ! ip -details link show "$CAN_IFACE" 2>/dev/null | grep -q "bitrate 1000000"; then
    log "warning: $CAN_IFACE does not show bitrate 1000000; check CAN before enabling the arm"
fi

launch_cmd=(
    ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py
    can_port:="$CAN_IFACE"
    arm_type:="$ARM_TYPE"
    effector_type:="$EFFECTOR_TYPE"
    auto_enable:="$AUTO_ENABLE"
    speed_percent:="$SPEED_PERCENT"
    gripper_default_effort:="$GRIPPER_DEFAULT_EFFORT"
    control_enabled:="$CONTROL_ENABLED"
    allow_remote_disable:="$ALLOW_REMOTE_DISABLE"
    home_joint_positions:="$HOME_JOINT_POSITIONS"
)

if [ -n "$FIRMWARE_OVERRIDE" ]; then
    launch_cmd+=(firmware_override:="$FIRMWARE_OVERRIDE")
fi

if [ "$DRY_RUN" = "true" ]; then
    printf '[start_arm_driver] dry run:'
    printf ' %q' "${launch_cmd[@]}"
    printf '\n'
    exit 0
fi

log "starting agx_arm_ctrl on $CAN_IFACE"
exec "${launch_cmd[@]}"

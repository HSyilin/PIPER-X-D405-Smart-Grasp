#!/usr/bin/env bash
set -euo pipefail

GRASP_WS="${GRASP_WS:-$HOME/grasp_ws}"
DETECTOR_BACKEND="${DETECTOR_BACKEND:-yolo_seg}"
MODE="${MODE:-plan}"
DRY_RUN="${DRY_RUN:-false}"
ACTION_NAME="${ACTION_NAME:-/smart_grasp/pick}"
ACTION_WAIT_TIMEOUT="${ACTION_WAIT_TIMEOUT:-15}"
ROS2_QUERY_TIMEOUT="${ROS2_QUERY_TIMEOUT:-5}"
SKIP_PREFLIGHT="${SKIP_PREFLIGHT:-false}"

log() {
    echo "[pick_object] $*" >&2
}

die() {
    echo "[pick_object] ERROR: $*" >&2
    exit 1
}

case "$DETECTOR_BACKEND" in
    yolo_seg)
        DEFAULT_TARGET_CLASS="1"
        ;;
    hsv)
        DEFAULT_TARGET_CLASS="blue_block"
        ;;
    *)
        die "DETECTOR_BACKEND must be yolo_seg or hsv, got: $DETECTOR_BACKEND"
        ;;
esac

TARGET_CLASS="${TARGET_CLASS:-$DEFAULT_TARGET_CLASS}"

case "$MODE" in
    plan)
        EXECUTE=false
        ;;
    execute)
        EXECUTE=true
        ;;
    plan_execute|full)
        EXECUTE=plan_execute
        ;;
    *)
        die "MODE must be plan, execute, or plan_execute, got: $MODE"
        ;;
esac

[ -f "$GRASP_WS/env.sh" ] || die "missing $GRASP_WS/env.sh"
# shellcheck disable=SC1090
source "$GRASP_WS/env.sh"

command -v ros2 >/dev/null 2>&1 || die "ros2 not found after sourcing $GRASP_WS/env.sh"

make_cmd() {
    local execute="$1"
    local goal="{target_class: \"$TARGET_CLASS\", execute: $execute}"
    cmd=(
        ros2 action send_goal "$ACTION_NAME"
        smart_grasp_interfaces/action/PickObject
        "$goal"
        --feedback
    )
}

show_ros_graph_snapshot() {
    local out
    log "ROS graph snapshot:"
    log "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"

    echo "[pick_object] nodes:" >&2
    out="$(timeout "${ROS2_QUERY_TIMEOUT}s" ros2 node list 2>&1 || true)"
    if printf '%s\n' "$out" | grep -q "PermissionError"; then
        echo "  ros2 CLI daemon PermissionError; try: ros2 daemon stop" >&2
    else
        printf '%s\n' "$out" >&2
    fi

    echo "[pick_object] actions:" >&2
    out="$(timeout "${ROS2_QUERY_TIMEOUT}s" ros2 action list 2>&1 || true)"
    if printf '%s\n' "$out" | grep -q "PermissionError"; then
        echo "  ros2 CLI daemon PermissionError; try: ros2 daemon stop" >&2
    else
        printf '%s\n' "$out" >&2
    fi

    echo "[pick_object] smart_grasp topics:" >&2
    out="$(timeout "${ROS2_QUERY_TIMEOUT}s" ros2 topic list 2>&1 || true)"
    if printf '%s\n' "$out" | grep -q "PermissionError"; then
        echo "  ros2 CLI daemon PermissionError; try: ros2 daemon stop" >&2
    else
        printf '%s\n' "$out" | grep -E "/smart_grasp|/feedback|/camera/camera" >&2 || true
    fi
}

wait_for_action_server() {
    local actions=""

    log "checking action server $ACTION_NAME (timeout=${ACTION_WAIT_TIMEOUT}s)"
    local deadline=$((SECONDS + ACTION_WAIT_TIMEOUT))
    while [ "$SECONDS" -lt "$deadline" ]; do
        actions="$(timeout "${ROS2_QUERY_TIMEOUT}s" ros2 action list 2>&1 || true)"
        if printf '%s\n' "$actions" | grep -qx "$ACTION_NAME"; then
            log "action server is available: $ACTION_NAME"
            return 0
        fi
        sleep 1
    done

    log "action server not available: $ACTION_NAME"
    if [ -n "$actions" ]; then
        if printf '%s\n' "$actions" | grep -q "PermissionError"; then
            echo "[pick_object] action-list failed: ros2 CLI daemon PermissionError; try: ros2 daemon stop" >&2
        else
            echo "[pick_object] discovered actions / action-list output:" >&2
            printf '%s\n' "$actions" >&2
        fi
    fi
    show_ros_graph_snapshot
    echo >&2
    echo "[pick_object] Start the grasp system first in another terminal:" >&2
    echo "  bash $GRASP_WS/scripts/start_grasp_system.sh" >&2
    echo >&2
    echo "[pick_object] Then confirm this appears:" >&2
    echo "  ros2 action list | grep $ACTION_NAME" >&2
    return 1
}

if [ "$DRY_RUN" = "true" ]; then
    if [ "$EXECUTE" = "plan_execute" ]; then
        make_cmd false
        printf '[pick_object] dry run plan:'
        printf ' %q' "${cmd[@]}"
        printf '\n'
        make_cmd true
        printf '[pick_object] dry run execute:'
        printf ' %q' "${cmd[@]}"
        printf '\n'
        exit 0
    fi
    make_cmd "$EXECUTE"
    printf '[pick_object] dry run:'
    printf ' %q' "${cmd[@]}"
    printf '\n'
    exit 0
fi

log "backend=$DETECTOR_BACKEND target_class=$TARGET_CLASS mode=$MODE"
if [ "$SKIP_PREFLIGHT" != "true" ]; then
    wait_for_action_server
else
    log "SKIP_PREFLIGHT=true; skipping action server check"
fi

if [ "$EXECUTE" = "plan_execute" ]; then
    log "step 1/2: plan-only"
    make_cmd false
    "${cmd[@]}"
    log "step 2/2: real execution"
    make_cmd true
    exec "${cmd[@]}"
fi

make_cmd "$EXECUTE"
log "sending goal execute=$EXECUTE"
exec "${cmd[@]}"

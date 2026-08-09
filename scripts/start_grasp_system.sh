#!/usr/bin/env bash
set -euo pipefail

GRASP_WS="${GRASP_WS:-$HOME/grasp_ws}"
CAMERA_SERIAL_NO="${CAMERA_SERIAL_NO:-260322272696}"
USE_DRIVER="${USE_DRIVER:-false}"
USE_CAMERA="${USE_CAMERA:-false}"
EXECUTE="${EXECUTE:-true}"
CALIBRATION_VALIDATED="${CALIBRATION_VALIDATED:-true}"
USE_RVIZ="${USE_RVIZ:-false}"
FIRMWARE_OVERRIDE="${FIRMWARE_OVERRIDE:-}"

DETECTOR_BACKEND="${DETECTOR_BACKEND:-yolo_seg}"
YOLO_MODEL="${YOLO_MODEL:-/home/guest/best.pt}"
YOLO_CLASS="${YOLO_CLASS:-1}"
YOLO_CONFIDENCE="${YOLO_CONFIDENCE:-0.7}"

GRASP_CONFIG="${GRASP_CONFIG:-$GRASP_WS/src/smart_grasp_bringup/config/grasp_test_box_60x40x40.yaml}"
PERCEPTION_CONFIG="${PERCEPTION_CONFIG:-$GRASP_WS/src/smart_grasp_bringup/config/perception_test_box_60x40x40.yaml}"
DRY_RUN="${DRY_RUN:-false}"

log() {
    echo "[start_grasp_system] $*" >&2
}

die() {
    echo "[start_grasp_system] ERROR: $*" >&2
    exit 1
}

[ -f "$GRASP_WS/env.sh" ] || die "missing $GRASP_WS/env.sh"
# shellcheck disable=SC1090
source "$GRASP_WS/env.sh"

command -v ros2 >/dev/null 2>&1 || die "ros2 not found after sourcing $GRASP_WS/env.sh"
[ -f "$GRASP_CONFIG" ] || die "missing grasp config: $GRASP_CONFIG"
[ -f "$PERCEPTION_CONFIG" ] || die "missing perception config: $PERCEPTION_CONFIG"

case "$DETECTOR_BACKEND" in
    yolo_seg)
        [ -f "$YOLO_MODEL" ] || die "missing YOLO model: $YOLO_MODEL"
        ;;
    hsv)
        ;;
    *)
        die "DETECTOR_BACKEND must be yolo_seg or hsv, got: $DETECTOR_BACKEND"
        ;;
esac

launch_cmd=(
    ros2 launch smart_grasp_bringup smart_grasp_system.launch.py
    use_driver:="$USE_DRIVER"
    use_camera:="$USE_CAMERA"
    camera_serial_no:="$CAMERA_SERIAL_NO"
    execute:="$EXECUTE"
    calibration_validated:="$CALIBRATION_VALIDATED"
    detector_backend:="$DETECTOR_BACKEND"
    yolo_model:="$YOLO_MODEL"
    yolo_class:="$YOLO_CLASS"
    yolo_confidence:="$YOLO_CONFIDENCE"
    grasp_config:="$GRASP_CONFIG"
    perception_config:="$PERCEPTION_CONFIG"
    use_rviz:="$USE_RVIZ"
)

if [ -n "$FIRMWARE_OVERRIDE" ]; then
    launch_cmd+=(firmware_override:="$FIRMWARE_OVERRIDE")
fi

if [ "$DRY_RUN" = "true" ]; then
    printf '[start_grasp_system] dry run:'
    printf ' %q' "${launch_cmd[@]}"
    printf '\n'
    exit 0
fi

log "starting smart_grasp_system backend=$DETECTOR_BACKEND use_driver=$USE_DRIVER use_camera=$USE_CAMERA"
exec "${launch_cmd[@]}"

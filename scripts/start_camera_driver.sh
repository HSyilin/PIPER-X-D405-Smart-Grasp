#!/usr/bin/env bash
set -euo pipefail

GRASP_WS="${GRASP_WS:-$HOME/grasp_ws}"
CAMERA_SERIAL_NO="${CAMERA_SERIAL_NO:-260322272696}"
CAMERA_NAMESPACE="${CAMERA_NAMESPACE:-camera}"
CAMERA_NAME="${CAMERA_NAME:-camera}"
RGB_PROFILE="${RGB_PROFILE:-848x480x30}"
DEPTH_PROFILE="${DEPTH_PROFILE:-848x480x30}"
ENABLE_INFRA="${ENABLE_INFRA:-false}"
INFRA_PROFILE="${INFRA_PROFILE:-848x480x30}"
POINTCLOUD_ENABLE="${POINTCLOUD_ENABLE:-false}"
DRY_RUN="${DRY_RUN:-false}"

log() {
    echo "[start_camera_driver] $*" >&2
}

die() {
    echo "[start_camera_driver] ERROR: $*" >&2
    exit 1
}

[ -f "$GRASP_WS/env.sh" ] || die "missing $GRASP_WS/env.sh"
# shellcheck disable=SC1090
source "$GRASP_WS/env.sh"

command -v ros2 >/dev/null 2>&1 || die "ros2 not found after sourcing $GRASP_WS/env.sh"

launch_cmd=(
    ros2 launch realsense2_camera rs_launch.py
    camera_namespace:="$CAMERA_NAMESPACE"
    camera_name:="$CAMERA_NAME"
    serial_no:="'$CAMERA_SERIAL_NO'"
    align_depth.enable:=true
    enable_color:=true
    enable_depth:=true
    enable_infra:="$ENABLE_INFRA"
    pointcloud.enable:="$POINTCLOUD_ENABLE"
    rgb_camera.color_profile:="$RGB_PROFILE"
    depth_module.depth_profile:="$DEPTH_PROFILE"
)

if [ "$ENABLE_INFRA" = "true" ]; then
    launch_cmd+=(depth_module.infra_profile:="$INFRA_PROFILE")
fi

if [ "$DRY_RUN" = "true" ]; then
    printf '[start_camera_driver] dry run:'
    printf ' %q' "${launch_cmd[@]}"
    printf '\n'
    exit 0
fi

log "starting RealSense D405 serial=$CAMERA_SERIAL_NO rgb=$RGB_PROFILE depth=$DEPTH_PROFILE infra=$ENABLE_INFRA"
exec "${launch_cmd[@]}"

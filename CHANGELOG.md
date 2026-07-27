# Change Log

All notable changes to the smart grasp workspace are recorded here. This
workspace started Git tracking at version 0.2.0, so earlier untracked prototypes
cannot be reconstructed exactly from this repository.

## Unreleased

### Documentation

- Added `PROJECT_RULES.md` to make the no-upstream-push, traceability/rollback,
  and minimal-invasive-change-with-approval requirements persistent.

### Fixed

- Added an opt-in camera-only launch and detector path for smooth 2-D HSV/YOLO
  diagnostics before the robot and hand-eye TF chain are connected. Production
  RGB-D/TF behavior remains unchanged by default.
- Set the VMware camera-only stream to the measured-stable `424x240x30` profile;
  the production RGB-D pipeline remains at `640x480x30`.
- Made detector and hand-eye node shutdown idempotent so `Ctrl+C` no longer emits
  `rcl_shutdown already called` or leaves launch waiting for forced termination.
- Run the detector with a two-thread executor so exact image-time TF lookups do not
  block the same executor thread that must receive the corresponding `/tf` update.

## 0.2.0 - 2026-07-26

### Added

- Added `smart_grasp_interfaces` with `DetectedObject.msg` and `PickObject.action`.
- Added interchangeable HSV and YOLO-Seg instance-mask backends.
- Added aligned-depth conversion, point-cloud projection, table-plane estimation,
  outlier filtering, PCA/OBB sizing, multi-frame stability, and ambiguity checks.
- Added timestamped eye-in-hand TF publishing from the 2026-07-25 calibration.
- Added a manual 5-8 pose TF validation recorder.
- Added the C++ MoveIt pick action server with RRTConnect pregrasp planning,
  Cartesian approach/lift, collision objects, start-state and wrist-jump checks,
  gripper fault/contact checks, object attachment, cancellation, and plan-only mode.
- Added `smart_grasp_bringup` for Piper-X, AgileX gripper, D405, MoveIt, RViz,
  perception, calibration, model metadata, and guarded execution.
- Added explicit OMPL `RRTConnectkConfigDefault` configuration to the lower
  `agx_arm_ros` workspace and made the control gate monitor arm and gripper actions.
- Added algorithm tests for HSV masks, depth units/projection, OBB/grasp geometry,
  stability filtering, and hand-eye validation spans.
- Added `RELEASE_MANIFEST.md` with exact cross-repository commits, checksums,
  excluded local state, validation evidence, and non-destructive rollback steps.

### Changed

- Changed the default detector from ArUco/red HSV prototype behavior to blue-block
  HSV segmentation for a nominal 60 x 40 x 40 mm target.
- Removed the direct `/control/move_p` executor from all default launches. It is
  retained only as an explicitly invoked, execution-disabled diagnostic tool.
- Split driver and MoveIt launch controls so MoveIt can be tested without CAN.

### Safety defaults

- Arm auto-enable is false.
- Pick execution is false.
- Calibration validation is false.
- Table dimensions use invalid sentinel values and block real execution.
- Driver control input starts gated closed and is opened only by trajectory status.

# Change Log

All notable changes to the smart grasp workspace are recorded here. This
workspace started Git tracking at version 0.2.0, so earlier untracked prototypes
cannot be reconstructed exactly from this repository.

## Unreleased

### Changed

- Smart-grasp launches now lock remote arm-disable requests during observation
  and pick execution. Intentional shutdown must explicitly release the lock;
  closing the external trajectory gate continues to leave servo enable intact.
- Raised the camera-only default color profile from `424x240x30` to
  `640x480x30` and exposed `color_profile` as a launch argument.
- At the project owner's direction, removed visual physical-size estimation,
  size matching, size stability, and the `SIZE_MISMATCH` action result. HSV is
  now the only identity gate; aligned depth remains only for 3-D position/yaw.
- `fixed_object_size` supplies collision and grasp geometry without affecting
  detection acceptance. This supersedes the earlier unreleased size-gating work.

### Documentation

- Added a complete, guarded commissioning procedure for one real pick of the
  measured `106.5 x 76.5 x 30.0 mm` test box, covering CAN activation, plan-only
  checks, runtime gates, explicit enable, emergency stop, result acceptance,
  supported release, and shutdown. Updated the recorded calibration, test, and
  real-arm acceptance status to match the latest field work.
- Documented the verified Home-to-observation recovery path, including the
  asynchronous `/move_home` completion check, powered Home hold, table collision
  object, and 5% MoveGroup planning limits.
- Added `PROJECT_RULES.md` to make the no-upstream-push, traceability/rollback,
  and minimal-invasive-change-with-approval requirements persistent.
- Corrected the package README to match the implemented HSV/YOLO backends, TF
  validation services, MoveIt state machine, RViz topics, runtime parameter
  behavior, and camera-only capability boundary.
- Documented that HSV pixel contour geometry is only a noise/shape filter and
  that aligned depth/PCA no longer estimates physical dimensions.

### Validation

- Completed the first full real-arm pick of the measured test box through
  observation, re-detection, pregrasp, approach, `76.6 mm` contact validation,
  attachment, and 50 mm lift. The action returned `success=true` and
  `error_code=0`; post-action arm and gripper feedback remained enabled and
  fault-free while the external trajectory gate was closed. After the operator
  supported the object, the gripper opened to `0.090 m`, remote-disable was
  explicitly unlocked, the arm returned `Agx_arm disabled`, and the complete
  launch was stopped. Shutdown still exposed MoveIt/driver process-cleanup
  errors that do not affect the completed pick.

### Fixed

- Validate the complete Cartesian descent for every plannable 180-degree wrist
  candidate before any real motion. The measured test-box profile now requires
  a `1.0` Cartesian fraction and uses a 120 mm pregrasp clearance.
- Changed test-box pregrasp reobservation to a fresh, same-track validation gate.
  The server keeps the pose locked at the observation position and aborts before
  descent if the new center moves more than 5 mm, height changes more than 5 mm,
  or the 180-degree-symmetric yaw changes more than 5 degrees.
- Set the measured test-box observation pose wrist joint J5 to zero so the real
  wrist orientation matches the intended observation configuration.
- Replaced top-surface PCA with a minimum-area rectangle for center/yaw and set
  the measured test-box surface band to 20 mm. On a 42-frame stationary capture,
  yaw span fell from 80.9 to 2.5 degrees and center span from 27.9 to 6.5 mm
  without weakening the existing stability gates.
- Relaxed only the measured test-box commissioning profile to an 8-frame window,
  a 30-degree yaw inlier radius, and a 20-degree per-pose yaw span. The 15 mm
  position gate and final 20 mm / 3-degree hand-eye validation remain unchanged.
- Re-publish the `tcp_link -> camera_link` static hand-eye mount once per second
  during the first startup window. This fixes a VMware/DDS discovery case where
  both adjacent TF segments existed but the constructor-time mount sample was lost.
- Stabilized RGB-D yaw estimation after stationary hardware tests showed 8-12 mm
  position span but 16-33 degree yaw span. PCA now uses only the top 8 mm surface
  band, and the pose window applies 180-degree-symmetric circular filtering with
  bounded yaw-outlier rejection before publishing the filtered pose.
- Parameterized exact-time TF lookup timeout and set the VMware test-box profile
  to 250 ms to tolerate scheduling delay without falling back to latest TF.
- Added an opt-in `106.5 x 76.5 x 30 mm` fixed-geometry test-box profile and made
  the point-cloud outlier radius configurable for robust center/yaw estimation.
- Added a matching fixed gripper test profile: 90 mm open, 0 mm close command,
  0.5 effort, and 68.5-84.5 mm accepted contact width. Execution stays disabled.
- Fixed `param_tuner` double initialization, parameter enumeration, typed array
  assignment, missing-parameter handling, and temporary-node cleanup.
- Added regression coverage for ROS parameter array conversion and display.
- Added an opt-in camera-only launch and detector path for smooth 2-D HSV/YOLO
  diagnostics before the robot and hand-eye TF chain are connected. Production
  RGB-D/TF behavior remains unchanged by default.
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

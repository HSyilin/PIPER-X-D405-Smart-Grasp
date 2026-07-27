# Smart Grasp

ROS 2 Humble pipeline for detecting and grasping a blue `60 x 40 x 40 mm`
rounded block with a Piper-X, AgileX gripper, and eye-in-hand RealSense D405.

## Packages

- `smart_grasp_interfaces`: `DetectedObject` message and `PickObject` action.
- `smart_grasp`: HSV/YOLO-Seg masks, RGB-D geometry, tracking, and hand-eye TF.
- `smart_grasp_moveit`: guarded C++ MoveIt planning and execution state machine.
- `smart_grasp_bringup`: versioned calibration, target/scene configuration, and launch.

The legacy direct `/control/move_p` executor remains available only as the
explicit `grasp_executor_node` diagnostic executable. No default launch starts it.

## Build

```bash
cd ~/agx_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agx_arm_ctrl agx_arm_moveit

cd ~/grasp_ws
source /opt/ros/humble/setup.bash
source ~/agx_arm_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Start safely

The default starts Piper-X, MoveIt, D405, HSV perception, and the action server,
but leaves arm auto-enable and grasp execution disabled:

```bash
source /opt/ros/humble/setup.bash
source ~/agx_arm_ws/install/setup.bash
source ~/grasp_ws/install/setup.bash
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py
```

Perception without starting the arm/MoveIt:

```bash
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py \
  use_driver:=false use_moveit:=false use_pick_server:=false use_rviz:=false
```

Run a plan-only pick after a stable detection is visible:

```bash
ros2 action send_goal /smart_grasp/pick \
  smart_grasp_interfaces/action/PickObject \
  "{target_class: blue_block, execute: false}" --feedback
```

## Required validation before real execution

1. Measure `table_height`, `table_size`, and `table_center_xy` in
   `smart_grasp_bringup/config/grasp.yaml`.
2. Observe one fixed object from 5-8 arm poses. Confirm base-frame position span
   is below 20 mm and orientation span below 3 degrees.
3. Set `validated: true` in `handeye_20260725.yaml` only after that test.
4. Enable the arm manually and retain `speed_percent:=10` for staged commissioning.

Both the action goal (`execute: true`) and the server arming parameter must be
true. Missing table measurements, an unvalidated calibration, stale TF/depth,
unstable size, trajectory start mismatch, wrist jump, or gripper fault aborts the
state machine before the following command is sent.

After all validation gates pass, arm the launch and then enable the hardware:

```bash
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py \
  execute:=true calibration_validated:=true

ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
```

Real motion still requires a separate action goal with `execute: true`; the
launch flags alone never start a pick.

At each stopped observation pose, record one independent validation sample:

```bash
ros2 service call /smart_grasp/validation/record std_srvs/srv/Trigger {}
```

The fifth through eighth response reports `validation=PASS` only when both TF
limits pass. Clear a run with `/smart_grasp/validation/reset`.

## YOLO-Seg

Copy externally trained segmentation weights to
`smart_grasp_bringup/models/blue_block_seg.pt`, record its metadata and SHA-256,
then launch with:

```bash
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py \
  detector_backend:=yolo_seg \
  yolo_model:=$HOME/grasp_ws/src/smart_grasp_bringup/models/blue_block_seg.pt
```

The model must provide instance masks for `blue_block`. A missing model or
Ultralytics installation is a startup error; the node never falls back to HSV
during a real action.

# Smart Grasp

ROS 2 Humble pipeline for detecting and grasping a blue `60 x 60 x 40 mm`
rounded block with a Piper-X, AgileX gripper, and eye-in-hand RealSense D405.

## Packages

- `smart_grasp_interfaces`: `DetectedObject` message and `PickObject` action.
- `smart_grasp`: HSV/YOLO-Seg masks, RGB-D geometry, tracking, and hand-eye TF.
- `smart_grasp_moveit`: guarded C++ MoveIt planning and execution state machine.
- `smart_grasp_bringup`: versioned calibration, target/scene configuration, and launch.

The legacy direct `/control/move_p` executor remains available only as the
explicit `grasp_executor_node` diagnostic executable. No default launch starts it.

## 当前进度

| 模块 | 状态 | 说明 |
|---|---|---|
| 接口 `smart_grasp_interfaces` | ✅ 完成 | `DetectedObject.msg` + `PickObject.action`，已编译 |
| 感知 `smart_grasp` | ✅ 已实现 | HSV / YOLO-Seg 统一后端；ArUco 仅为独立诊断脚本 |
| 深度/抓取几何 | ✅ 完成 | `depth_geometry` 反投影/PCA 位姿 + 预设目标几何→TCP 抓取姿态 |
| 手眼 / 校验 | ✅ 已实现 | 发布动态 `base_link -> tcp_link` 和静态相机外参；通过 `/smart_grasp/validation/record`、`reset` 采样校验 |
| MoveIt 执行 `smart_grasp_moveit` | ✅ 已实现 | C++ `pick_server` 实现从检测、规划、接近、夹持到抬升的完整防护状态机 |
| 编排/配置 `smart_grasp_bringup` | ✅ 完成 | `smart_grasp_system.launch.py` 统一编排 + 校验门控 |
| 稳定性门控 | ✅ 完成 | `stability.py` 多帧稳定性判断 |
| 调参器 `param_tuner` | ✅ 完成 | SSH 友好 `name=value` 交互改参（已修重复 `rclpy.init` 崩溃 bug） |
| **真实抓取前待补** | 🟡 阻塞 | 见下「Required validation」：`table_*` 实测值、`handeye_20260725.yaml` 置 `validated: true` |

> 外部依赖已对齐：`agx_arm_msgs::GripperStatus` 字段、`agx_arm_moveit` 的 SRDF 组 `arm` / link `tcp_link`、`gripper_base/link1/link2` 均匹配；`agx_arm_ws` 已编译。

手眼节点会在启动后有限重发 `tcp_link -> camera_link` 静态TF，解决VMware中
首个 transient-local 样本在DDS发现完成前发布而造成TF树断开的情况。

`perception_test_box_60x40x40.yaml` 是现场调试专用配置；
当前值按 `60 x 40 x 40 mm` 物块和机械臂平台高于物块平台 `30 mm` 配置。
它使用8帧窗口、30度角度内点半径和20度单姿态角度极差。该放宽不改变下方手眼验证要求的
20 mm位置极差和3度方向极差。

## Build

```bash
cd ~/agx_arm_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agx_arm_ctrl agx_arm_moveit

cd ~/grasp_ws
source /opt/ros/humble/setup.bash
source ~/agx_arm_ws/install/setup.bash
colcon build --symlink-install
source env.sh
```

## Start safely

The default starts Piper-X, MoveIt, D405, HSV perception, and the action server,
but leaves arm auto-enable and grasp execution disabled:

```bash
source /opt/ros/humble/setup.bash
source ~/grasp_ws/env.sh
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py \
  camera_serial_no:=260322272696
```

2-D perception without starting the arm/MoveIt or requiring depth/TF:

```bash
ros2 launch smart_grasp_bringup camera_only.launch.py \
  camera_serial_no:=260322272696 \
  color_profile:=640x480x30 open_gui:=false
```

This camera-only mode publishes HSV/YOLO masks, pixel bounding boxes, and debug
images. It does not require aligned depth or a robot TF.

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
3. After that test, set `validated: true` in `handeye_20260725.yaml`
   (calibration/validator flag) **and** arm the server with the launch arg
   `calibration_validated:=true`. The `reconcile_calibration` function
   ensures you cannot bypass the yaml check: passing a CLI flag before
   updating the yaml is silently overridden to `false`. Both must be true.
4. Enable the arm manually and retain `speed_percent:=10` for staged commissioning.

The launch-internal `reconcile_calibration` function reads the yaml `validated`
field and only ever clamps down — if the yaml still reports `false`, any
`calibration_validated:=true` on the command line is forced back to `false`.
This prevents unverified hand-eye calibration from opening the execution gate,
but it never auto-promotes: you must explicitly pass both flags.

Both the action goal (`execute: true`) and the server arming parameter must be
true. Missing table measurements, an unvalidated calibration, stale TF/depth,
unstable pose, trajectory start mismatch, wrist jump, or gripper fault aborts the
state machine before the following command is sent.

After all validation gates pass, use this real-arm sequence:

```bash
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py \
  camera_serial_no:=260322272696 \
  execute:=true calibration_validated:=true
# calibration_validated is gated by reconcile_calibration: the handeye yaml
# must also have validated: true, or the flag is silently forced to false.

ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /move_home std_srvs/srv/Empty "{}"
ros2 action send_goal /smart_grasp/pick \
  smart_grasp_interfaces/action/PickObject \
  "{target_class: blue_block, execute: true}" --feedback
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
```

Use the official AGX `/move_home` service first. Then `/smart_grasp/pick`
moves to the configured observation pose, runs detect/reobserve/pregrasp/
approach/close/lift, returns home with the gripper held closed, and only then
finishes. Use
`execute: false` only for a read-only check; it still requires the arm to
already be at the observation pose. The legacy `/smart_grasp/home` service is
not part of the main workflow. The default grasp config is now
`grasp_test_box_60x40x40.yaml`; use `grasp.yaml` as the editable template.

## Static obstacle modeling (radar / controller)

The radar mast and main controller sit next to the arm and are **fixed relative
to `base_link`**, so they never move in the robot frame. They are **not** in the
planning scene by default and MoveIt would happily route the arm through them.
Model them as static collision boxes:

```yaml
# grasp_test_box_60x40x40.yaml  (smart_grasp_bringup/config/grasp_test_box_60x40x40.yaml)
static_obstacles: |
  # id,size_x,size_y,size_z,pos_x,pos_y,pos_z  (CSV, no spaces)
  # size = full box dimensions (m); pos = box CENTER in base_link (m)
  radar_mast,0.20,0.20,0.90,0.00,-0.45,0.45
  main_controller,0.30,0.25,0.30,0.35,0.00,0.15
# Hard planning bound in base_link (min_x,min_y,min_z,max_x,max_y,max_z):
workspace: [-0.20, -0.70, -0.10, 0.90, 0.70, 1.20]
```

`pick_server` parses `static_obstacles` in `applyScene()` (one box per non-empty,
non-`#` line) and `ADD`s each as a `BOX` collision object in `base_frame`. The
`workspace` vector is applied via `move_group->setWorkspace()` during
`initialize()`, acting as a hard bound so the planner never exits the safe
volume. Both are **conservative placeholders** — measure the real geometry on
site and replace the numbers. Add more lines for cable trays, supports, etc.

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
  camera_serial_no:=260322272696 \
  detector_backend:=yolo_seg \
  yolo_model:=$HOME/grasp_ws/src/smart_grasp_bringup/models/blue_block_seg.pt
```

The model must provide instance masks for `blue_block`. A missing model or
Ultralytics installation is a startup error; the node never falls back to HSV
during a real action.

## 调试与调参

### 看图像 / 调试 TF

- 只看感知（不起机械臂 / MoveIt）:
  ```bash
  ros2 launch smart_grasp_bringup camera_only.launch.py \
    camera_serial_no:=260322272696
  ```
- RViz 里通过 Image、TF、MarkerArray 和 PointCloud2 显示分别查看
  `/smart_grasp/debug_image`、TF、`/smart_grasp/debug_markers` 和
  `/smart_grasp/object_cloud`。自定义 `/smart_grasp/detections` 不能直接作为标记显示。
- 无 DISPLAY 的 SSH 远程且已安装 `web_video_server` 时，可另起
  `ros2 run web_video_server web_video_server`，
  浏览器开 `http://localhost:8080/stream_viewer?topic=/smart_grasp/debug_image`
  （SSH 用 `ssh -L 8080:localhost:8080` 转发端口）。

### HSV 和深度会计算哪些数据

- HSV 后端自动计算每个蓝色轮廓的像素面积、凸度、矩形度、二维外接矩形和
  图像内角度。这些数据用于二维筛选，单位是像素，不是物体的实际毫米尺寸。
- 完整 RGB-D 模式读取 Mask 内的对齐深度，使用 CameraInfo 反投影点云，
  转换到 `base_link` 后通过顶部点云带最小外接矩形计算中心和水平朝向。
- 不测量、不匹配物体长宽高，也没有 `SIZE_MISMATCH`。每个 `track_id` 的多帧
  窗口只检查位置和水平角；180度对称角使用圆周统计并剔除少量离群帧。
- `/smart_grasp/detections.size` 来自 `fixed_object_size` 配置，仅供碰撞盒和抓取
  几何使用，不参与颜色目标是否通过的判断。
- `camera_only.launch.py` 没有深度和机械臂 TF，因此只能显示二维检测框和角度，
  不会产生可信的实际长宽高，也不会把像素面积当作物理面积。

### 运行时改抓取参数
`pick_server` 的抓取距离、夹爪、安全阈值和场景参数在每次 action goal 中读取，
运行时修改后对下一次 goal 生效。`planning_group`、`base_frame`、
`end_effector_link`、`planner_id`、`planning_time`、`planning_attempts`、
`gripper_action` 和 `joint_state_topic` 在节点初始化时读取，修改后必须重启
`pick_server`。三种调参方式：
- **SSH 友好（推荐）**：交互式 `name=value` 改参，自动识别
  int / float / bool / list / str：
  ```bash
  ros2 run smart_grasp param_tuner smart_grasp_pick_server
  # smart_grasp_pick_server> grasp_depth=0.03
  # smart_grasp_pick_server> list        # 看全部参数
  # smart_grasp_pick_server> quit
  ```
- **本机有 DISPLAY 且已安装插件**：`ros2 run rqt_reconfigure rqt_reconfigure`（左侧选
  `smart_grasp_pick_server`，滑条/文本框直接改）。
- **命令行一行**：`ros2 param set /smart_grasp_pick_server grasp_depth 0.03`。
> 常用参数：`grasp_depth`(夹爪下探深度)、`pregrasp_distance`、`lift_height`、
> `gripper_open/close/force`、`wrist_jump_threshold`、`cartesian_step`、`planning_time`。

### 目标 / 场景配置

- action goal 中的 `target_class`（如 `blue_block`）只负责选择检测类别。
- HSV 和预设目标几何由 `config/perception.yaml` 提供，抓取和场景参数由
  `config/grasp_test_box_60x40x40.yaml` 提供（含 `target_class: "1"` 和
  `fixed_object_size`）。
- 这版测试盒参数把 `gripper_close` 调到 `0.046`，并同步下调了 `contact_width_min/max`。
- `config/grasp.yaml` 作为可编辑模板保留；当前值会阻止真实执行，避免误用。
- `config/handeye_20260725.yaml` 的 `validated: true` 是抓取执行的前置门控；
  `reconcile_calibration` 只向下压制（yaml 不通过时拒绝 CLI 覆写），仍需显式传
  `calibration_validated:=true`。

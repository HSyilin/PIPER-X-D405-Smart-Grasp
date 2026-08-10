# Smart Grasp Workspace

PIPER-X 机械臂 + AgileX 夹爪 + 眼在手上 RealSense D405 的 ROS 2 智能抓取工作区。

系统链路：D405 RGB-D 感知 -> 手眼 TF -> MoveIt 规划 -> `pick_server` 执行抓取。目标物体默认为蓝色方块，感知后端支持 `hsv` 和 `yolo_seg`。

## 项目结构

```text
grasp_ws/
├── src/
│   ├── smart_grasp/              # 感知、RGB-D 几何、手眼 TF、调参工具
│   ├── smart_grasp_bringup/      # 统一启动、配置、RViz/模型占位
│   ├── smart_grasp_moveit/       # MoveIt 抓取 action server
│   ├── smart_grasp_interfaces/   # DetectedObject.msg / PickObject.action
│   └── agx_arm_msgs/             # 机械臂与夹爪状态消息
├── env.sh                        # 本工作区 Python 环境
├── requirements.txt              # Python/视觉依赖说明
├── build/ install/ log/          # colcon 产物
└── README.md
```

外部机械臂与 MoveIt 工作区：`~/agx_arm_ws`。

## 关键文件

```text
src/smart_grasp_bringup/launch/smart_grasp_system.launch.py
  完整系统启动入口。

src/smart_grasp_bringup/launch/camera_only.launch.py
  仅启动 D405 + 2-D 检测，用于相机/识别调试。

src/smart_grasp_bringup/config/grasp_test_box_60x40x40.yaml
  默认真机抓取参数；机械臂观察位姿在 observation_joint_positions。

src/smart_grasp_bringup/config/grasp.yaml
  抓取参数模板；observation_joint_positions 为空时会阻止抓取。

src/smart_grasp_bringup/config/perception_test_box_60x40x40.yaml
  真机测试盒感知参数。

src/smart_grasp_bringup/config/perception.yaml
  默认感知参数。

src/smart_grasp_bringup/config/handeye_20260725.yaml
  手眼标定结果；validated: true 才允许打开真机执行门。

src/smart_grasp_moveit/src/pick_server.cpp
  抓取状态机与安全门实现。
```

当前测试盒机械臂观察位姿：

```yaml
observation_joint_positions: [-1.560708324, 1.875757707, -1.251889766, 0.776078105, 0.0, -0.005742133]
```

默认回零位仍由官方 `/move_home` 管理；未覆盖 `home_joint_positions` 时使用 AGX
驱动默认 home，也就是零位。抓取流程是先人工调用 `/move_home` 到官方 home，
再由 pick action 去观察位；抓取成功并完成抬升后，会回到下面这个中间点。
如果动作已经到达抓取位但闭爪、接触验证或抬升失败，也会先尝试回到同一收尾点位：

```yaml
post_pick_joint_positions: [-1.583554684, 0.186139365, -0.379190233, 0.550424486, -0.055798176, 0.0]
```

如果需要中间点后继续到人工示教的最终点，把当前 `/feedback/joint_states`
读回的 6 个关节值填到：

```yaml
post_pick_final_joint_positions: [-0.034924038, 0.366536596, -0.541017162, 1.152074386, 0.019024089, 0.0]
```

## 现场记录

2026-08-09 已完成一次真机成功抓取。完整参数快照和当时运行命令记录在
`/home/guest/grasp_ws/now_question.md` 的
`[2026-08-09] 真机成功抓取参数快照` 条目中；README 只保留复现流程和关键调参说明。

## 构建

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

## 启动流程
### 1. 加载环境

每个新终端都先执行：

```bash
source ~/grasp_ws/env.sh
```

### 2. 启动完整抓取任务

推荐直接启动融合 launch。launch 会先同步执行 CAN 绑定，确认 `can0` 已 `UP` 且波特率为 1M，然后再启动导航、机械臂驱动和任务编排器，避免 AGX 驱动抢在 CAN 配置完成前启动。

```bash
ros2 launch smart_grasp_bringup turing_grasp_mission.launch.py
```

可选覆盖轨迹文件：

```bash
ros2 launch smart_grasp_bringup turing_grasp_mission.launch.py \
  trajectory_file:=/home/guest/funny_lidar_slam/data/trajectories/example_path.yaml
```

### 3. CAN 调试

`~/can_bind.sh` 会一次性完成硬件识别、接口命名、波特率配置和启动。脚本优先按 USB-CAN 序列号识别机械臂适配器，识别不到序列号时才回退到唯一的 `gs_usb` 设备，避免开机枚举顺序变化导致 `can0/can1` 交换错误。配置前会先 `down`，所以不再需要第二次执行命令；并设置 `restart-ms 100`，BUS-OFF 后自动恢复。

```bash
# 上电或 USB-CAN 热插拔后执行一次
bash ~/can_bind.sh
```

兼容旧习惯：`bash ~/can_connect.sh` 会直接调用同一套一键配置逻辑。

可选覆盖项：

```bash
USB_CAN_SERIAL=002000465547570420303135 CAN_BITRATE=1000000 CAN_RESTART_MS=100 bash ~/can_bind.sh
```

常见 CAN 口调试命令：

```bash
# 查看接口状态 / 波特率 / 总线状态（ERROR-ACTIVE / ERROR-PASSIVE / BUS-OFF）
ip -details link show can0

# 查看收发包数、错误计数、丢帧等统计
ip -s link show can0

# 监听 can0 上的所有报文（Ctrl+C 退出）
candump can0

# 发送一帧测试报文（标准帧 ID=123，数据 DEADBEEF）
cansend can0 123#DEADBEEF

# 查看错误帧 / 总线异常（BUS-OFF 时会出现错误帧）
candump -e can0

# 清除 BUS-OFF / 重置错误计数器（接口需先 down）
sudo ip link set can0 down
sudo ip link set can0 type can restart
sudo ip link set can0 up

# 检查 USB-CAN 适配器是否识别到（gs_usb 设备）
lsusb | grep -iE "can|gs_usb"
dmesg | grep -i gs_usb

# 关闭接口（排障或重新配置前）
sudo ip link set can0 down
```

### 3. 启动机械臂驱动

```bash
bash ~/grasp_ws/scripts/start_arm_driver.sh
```

该脚本会先加载 `~/grasp_ws/env.sh`，再执行 `~/can_bind.sh` 确认 `can0` 为
USB-CAN / 1M / `UP`，最后启动 `agx_arm_ctrl`。启动时会显式传入 6 维
`home_joint_positions`，避免 ROS 2 launch 把 `[]` 空列表当成无类型参数而崩溃。

如需临时覆盖参数，用环境变量，不要在命令行传 `home_joint_positions:=[]`：

```bash
SPEED_PERCENT=10 AUTO_ENABLE=false bash ~/grasp_ws/scripts/start_arm_driver.sh
HOME_JOINT_POSITIONS="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" bash ~/grasp_ws/scripts/start_arm_driver.sh
FIRMWARE_OVERRIDE=S-V1.8-8 bash ~/grasp_ws/scripts/start_arm_driver.sh
RUN_CAN_BIND=false DRY_RUN=true bash ~/grasp_ws/scripts/start_arm_driver.sh
```

`FIRMWARE_OVERRIDE` 只在驱动从 CAN 读取固件版本超时时使用；CAN 和关节反馈正常后
才允许继续使能和运动。

确认机械臂反馈：

```bash
ros2 topic hz /feedback/joint_states
```

小测试：让机械臂前往官方 home / 零位。先确认工作区无人、平台停稳、急停可触及；驱动启动命令默认 `speed_percent=10`，适合低速检查。

在另一个终端执行：

```bash
source ~/grasp_ws/env.sh

# 确认驱动服务已起来
ros2 service list | grep -E "/enable_agx_arm|/move_home"

# 使能机械臂
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"

# 前往官方 home / 零位
ros2 service call /move_home std_srvs/srv/Empty "{}"

# 观察关节反馈是否稳定
ros2 topic echo /feedback/joint_states --once

# 测试结束后可失能
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
```

### 4. 启动相机驱动

```bash
bash ~/grasp_ws/scripts/start_camera_driver.sh
```

该脚本会先加载 `~/grasp_ws/env.sh`，再按 D405 序列号 `260322272696` 启动
`realsense2_camera`。默认彩色流为 `848x480x30`，深度流为 `848x480x30`，
并关闭 infra / pointcloud。当前 D405 / RealSense 驱动支持
`848x480x30`，不支持 `848x480x10`；如果传 10 fps，RealSense 驱动会报
`Given value ... is invalid` 并退回 `848x480x30`。

如需临时覆盖参数，用环境变量；先用 `ros2 param describe` 确认设备支持列表：

```bash
RGB_PROFILE=848x480x30 DEPTH_PROFILE=848x480x30 bash ~/grasp_ws/scripts/start_camera_driver.sh
ENABLE_INFRA=true INFRA_PROFILE=848x480x30 bash ~/grasp_ws/scripts/start_camera_driver.sh
DRY_RUN=true bash ~/grasp_ws/scripts/start_camera_driver.sh
```

确认相机出图：

```bash
ros2 topic hz /camera/camera/color/image_rect_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
```

小测试：查看当前设备实际支持的 profile。在另一个终端执行：

```bash
source ~/grasp_ws/env.sh
ros2 param describe /camera/camera depth_module.depth_profile
ros2 param describe /camera/camera rgb_camera.color_profile
```

### 5. 启动 MoveIt、感知和抓取服务

```bash
bash ~/grasp_ws/scripts/start_grasp_system.sh
```

该脚本会先加载 `~/grasp_ws/env.sh`，再启动 MoveIt、手眼 TF、感知节点和抓取
action server。默认认为机械臂驱动和相机驱动已经由前两步单独启动，所以固定
`use_driver=false`、`use_camera=false`。YOLO-Seg 和 HSV 是并列感知后端，默认
使用 YOLO-Seg：`/home/guest/best.pt`、`yolo_class=1`、`yolo_confidence=0.7`。

如需临时覆盖参数，用环境变量：

```bash
DETECTOR_BACKEND=yolo_seg YOLO_MODEL=/home/guest/best.pt bash ~/grasp_ws/scripts/start_grasp_system.sh
DETECTOR_BACKEND=hsv bash ~/grasp_ws/scripts/start_grasp_system.sh
USE_DRIVER=true FIRMWARE_OVERRIDE=S-V1.8-8 bash ~/grasp_ws/scripts/start_grasp_system.sh
USE_RVIZ=true DRY_RUN=true bash ~/grasp_ws/scripts/start_grasp_system.sh
```

确认系统状态：

```bash
ros2 node list | grep -E "/move_group|/smart_grasp_detector|/smart_grasp_pick_server"
ros2 action list | grep /smart_grasp/pick
ros2 control list_controllers
ros2 topic echo /smart_grasp/detections
```

### 6. 执行抓取

先做 plan-only，不发真实运动：

```bash
bash ~/grasp_ws/scripts/pick_object.sh
```

该脚本会先加载 `~/grasp_ws/env.sh`，再发送 `/smart_grasp/pick` action。YOLO-Seg
和 HSV 是并列感知后端，默认按 YOLO-Seg 发送 `target_class="1"`；切到 HSV 时
发送 `target_class="blue_block"`。发送 goal 前会先检查 `/smart_grasp/pick`
action server；如果第 5 步没有启动成功，脚本会打印当前 ROS graph 状态并退出，
不会一直停在 `Waiting for an action server to become available...`。

如需临时覆盖参数，用环境变量：

```bash
MODE=plan DETECTOR_BACKEND=yolo_seg bash ~/grasp_ws/scripts/pick_object.sh
MODE=plan DETECTOR_BACKEND=hsv bash ~/grasp_ws/scripts/pick_object.sh
DRY_RUN=true bash ~/grasp_ws/scripts/pick_object.sh
```

确认工作区无人、平台停稳、急停可触及时，再执行真机抓取：

```bash
MODE=execute bash ~/grasp_ws/scripts/pick_object.sh
```

规划真机抓取一体：

```bash
MODE=plan_execute bash ~/grasp_ws/scripts/pick_object.sh
```

`MODE=execute` 本身也会先规划再执行；`MODE=plan_execute` 是额外先跑一次
plan-only，再发真实执行 goal。

收尾：

```bash
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
```

### 真机流程

完整真机流程按这个顺序走。启动脚本都是前台进程，分别在独立终端保持运行。

```bash
# 终端 1：启动机械臂驱动
source ~/grasp_ws/env.sh
bash ~/grasp_ws/scripts/start_arm_driver.sh
```

```bash
# 终端 2：启动相机驱动
source ~/grasp_ws/env.sh
bash ~/grasp_ws/scripts/start_camera_driver.sh
```

```bash
# 终端 3：启动 MoveIt、手眼 TF、感知和抓取服务
source ~/grasp_ws/env.sh
bash ~/grasp_ws/scripts/start_grasp_system.sh
```

```bash
# 终端 4：使能、回 home、执行抓取、收尾失能
source ~/grasp_ws/env.sh
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /move_home std_srvs/srv/Empty "{}"

MODE=execute bash ~/grasp_ws/scripts/pick_object.sh

# 抓取成功并完成抬升后，会通过 MoveIt 回到 post_pick_joint_positions；
# 若已经到达抓取位但接触验证/抬升失败，也会先尝试回到同一收尾点位。
# 若配置了 post_pick_final_joint_positions，会再从中间点移动到该最终点；
# 成功到达最终收尾点位后，pick_server 会自动打开夹爪放下物块。
# 官方 /move_home 仍只用于人工回 home。

ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"
```

`/smart_grasp/pick` 在 `execute:=true` 时会先去配置好的观察位姿，再做识别、
重观察、预抓、接近、闭爪和抬升。只读模式用 `execute:=false`，但仍要求机械臂
已经在观察位姿上，不会自动从 home 过去。官方 `/move_home` 仍然是回零入口。

### 抓取时间轨迹调参

真机抓取的速度和丝滑度主要由
`src/smart_grasp_bringup/config/grasp_test_box_60x40x40.yaml` 控制。普通
MoveIt 关节规划段使用 `velocity_scaling` / `acceleration_scaling`；5 mm
笛卡尔接近和 50 mm 抬升段额外做时间参数化，使用独立的
`cartesian_velocity_scaling` / `cartesian_acceleration_scaling`；夹爪开合时间由
`gripper_motion_time` 控制。

保守真机检查档：

```yaml
# 单次 MoveIt 规划允许的最长时间；越大越稳，但失败路径会等更久。
planning_time: 5.0
# 每个目标规划尝试次数；越大越容易找到路径，但规划耗时增加。
planning_attempts: 5
# 普通关节轨迹的最大速度比例，0.02 表示使用关节限速的 2%。
velocity_scaling: 0.02
# 普通关节轨迹的最大加速度比例；建议先与 velocity_scaling 同步提高。
acceleration_scaling: 0.02
# 当前成功基线未对 computeCartesianPath 生成的接近/抬升轨迹重新做时间参数化。
time_parameterize_cartesian: false
# 5 mm 接近和 50 mm 抬升段的速度比例；建议低于或等于全局速度。
cartesian_velocity_scaling: 0.02
# 接近/抬升段的加速度比例；过高会让近物体动作显得突兀。
cartesian_acceleration_scaling: 0.02
# 等待稳定目标检测的最长时间；目标稳定后会提前返回，不是固定等待。
stable_detection_wait_timeout: 3.0
# 等待机械臂 action 结果和真实关节反馈到位的最长时间；不是固定等待。
execution_settle_timeout: 60.0
# 夹爪开合轨迹时长；减小会更快，但过小可能影响接触反馈稳定性。
gripper_motion_time: 1.0
# 夹爪控制器只支持 position；夹爪力来自机械臂驱动 gripper_default_effort=0.5。
gripper_force: 0.5
# 预先验证两个 180 度候选的完整笛卡尔接近；更稳但会增加规划时间。
validate_all_candidate_approaches: true
```

接触宽度、目标姿态和路径都稳定后，可以逐项试这个提速档：

```yaml
planning_time: 3.0
planning_attempts: 3
velocity_scaling: 0.08
acceleration_scaling: 0.08
cartesian_velocity_scaling: 0.06
cartesian_acceleration_scaling: 0.06
stable_detection_wait_timeout: 1.5
execution_settle_timeout: 8.0
gripper_motion_time: 0.7
gripper_force: 0.5
```

进一步缩短规划时间时，再考虑把 `validate_all_candidate_approaches` 从 `true`
改成 `false`，这样只对最终选中的候选做笛卡尔接近验证。该项能省时间，但会减少
候选预筛选，建议只在多次 `MODE=plan` 和低速 `MODE=execute` 都稳定后使用。

`cartesian_step` 是笛卡尔路径插值精度，不是主要提速旋钮。优先调速度、加速度、
检测等待和夹爪动作时间；每次只改一到两个参数，并用 action 结果里的
`contact_width` 和服务端 `pick timing` 日志确认效果。

### 可选一体启动

如需确认现场安全后一条命令连驱动、相机、MoveIt、手眼 TF、感知和抓取服务
一起启动：

```bash
USE_DRIVER=true USE_CAMERA=true bash ~/grasp_ws/scripts/start_grasp_system.sh
```

该脚本默认使用 YOLO-Seg、`/home/guest/best.pt`、60 x 40 x 40 mm 测试盒抓取
配置、已验证手眼外参和 `use_rviz=false`。未显式设置时，默认
`USE_DRIVER=false`、`USE_CAMERA=false`，不会重复拉起机械臂驱动和相机驱动。

启动完成后仍需按第 6 步发送 `/smart_grasp/pick` goal；launch 本身不会自动抓取。

## 安全门

真实运动必须同时满足：

```text
execute:=true
calibration_validated:=true
handeye_20260725.yaml 中 validated: true
grasp_config 中 table_height / table_size / table_center_xy 已实测
```

任一条件不满足，`pick_server` 会拒绝真实执行。

默认真机 launch 使用 `use_live_feedback:=true`，MoveIt 订阅
`/feedback/joint_states`，并启动 `agx_arm_control_gate`。安全门节点监听
`/arm_controller/follow_joint_trajectory` 和
`/gripper_controller/follow_joint_trajectory` 的 action status，只在轨迹或夹爪
action active 时自动打开 `/control_enable`，action 结束后自动关闭。默认抓取流程
不要长期手动打开 `/control_enable`；手动开关只用于直接调试 `/control/*` 底层控制话题。

`/enable_agx_arm` 是硬件使能门；`/control_enable` 只是 `/control/*` 外部控制话题门。
抓取状态机只有在闭爪、接触验证和抬升都成功后，才会继续通过 MoveIt 规划并执行到
`post_pick_joint_positions`；该动作仍走 `arm_controller/follow_joint_trajectory`，
因此继续受自动 `/control_enable` 门控保护。`/move_home` 是官方驱动服务入口，
用于人工回官方 home / 零位，不是抓取完成后的内部返回路径；它不走 MoveIt 规划，
也不依赖自动门控打开 `/control_enable`，但仍要求机械臂已硬件使能且不在示教模式。
不要在 YAML 里写空列表 `[]`，ROS 2 参数注入无法稳定推断空序列类型。

## 清理旧进程

如果上一次 launch 异常退出，残留的 MoveIt、相机、抓取节点或 ROS 2 daemon
可能继续占用 action、topic、相机设备或控制器，影响下一次启动。重启完整系统前
可先在确认机械臂安全后清理：

```bash
source ~/grasp_ws/env.sh

# 若服务仍可用，先失能机械臂
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}" || true

# 清理 ROS 2 CLI daemon 缓存
ros2 daemon stop || true
ros2 daemon start

# 查看残留进程
ps -ef | grep -E "smart_grasp|move_group|realsense2_camera|agx_arm|rviz2" | grep -v grep

# 只在确认这些确实是上一轮残留进程后执行
pkill -f "smart_grasp|move_group|realsense2_camera|agx_arm|rviz2" || true
```

清理后重新按真机流程启动机械臂驱动、相机驱动和抓取系统。
# PIPER-X-D405-Smart-Grasp

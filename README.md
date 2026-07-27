# PIPER-X D405 Smart Grasp

当前版本：`0.2.0`，日期：`2026-07-26`。

本工作区实现 PIPER-X、AgileX 夹爪和眼在手上的 RealSense D405 对蓝色
`60 x 40 x 40 mm` 圆角长方体进行识别、三维定位、MoveIt 规划与抓取。
第一阶段使用 OpenCV HSV，保留 YOLO-Seg 实例分割接口。当前默认状态只允许
识别和规划，外参及桌面数据未通过现场验证前不能执行真机动作。

## 1. 系统边界

当前范围：

- 固定机械臂基座、固定桌面、单个或少量静止目标。
- HSV 识别蓝色目标，D405 对齐深度只负责三维位置和水平朝向定位。
- 顶部两指抓取，抓起后沿 `base_link +Z` 抬升 50 mm 并保持。
- MoveIt 2、KDL 和 OMPL RRTConnect 负责规划。
- 支持只规划、取消、安全失败和低速分级调试。

当前不包含：

- TRON 2 导航和底盘运动控制。
- 放置流程、多目标任务调度、动态避障和 Octomap 闭环。
- 已训练的 YOLO 权重及其现场精度验收。
- 已通过验收的手眼外参和桌面碰撞尺寸。

## 2. 整体结构

```text
RealSense D405
  color/image_raw + aligned_depth_to_color/image_raw + CameraInfo
                         |
                         v
smart_grasp_detector -----------------------------------------------+
  HSV / YOLO-Seg -> Instance Mask -> 深度点云 -> 平面/PCA          |
  -> TF(base_link) -> 位姿多帧稳定 -> 固定几何抓取候选              |
                         |                                          |
                         +--> detections / cloud / poses / debug     |
                                                                    v
feedback/tcp_pose -> handeye_tf_node -> base->tcp->camera      PickObject Action
                                                              smart_grasp_moveit
                                                                    |
                      PlanningScene(table + target) <---------------+
                                                                    |
                    RRTConnect预抓取 -> 笛卡尔接近 -> 夹爪闭合
                    -> 接触验证 -> 附着物体 -> 笛卡尔抬升
                                                                    |
               arm_controller + gripper_controller FollowJointTrajectory
                                                                    |
                     自动控制门 -> agx_arm_ctrl -> pyAgxArm -> can0
```

以后安装到 TRON 2 时，在上层增加 `tron_base_link -> base_link` 和 Nav2 任务
协调器；相机仍固定在机械臂 TCP 附近时，感知、手眼和抓取包不改变。

## 3. 工作区依赖

```text
~/pyAgxArm       官方 Python SDK，负责 CAN 和机械臂底层通信
      ^
      |
~/agx_arm_ws     官方 ROS 2 驱动、描述、ros2_control 和 MoveIt 配置
      ^
      |
~/grasp_ws       本仓库：感知、接口、抓取执行和统一启动

~/handeye_ws     标定工具和原始结果；运行时不 overlay，只复制版本化结果
```

构建和运行 overlay 顺序必须是：

```bash
source /opt/ros/humble/setup.bash
source ~/agx_arm_ws/install/setup.bash
source ~/grasp_ws/install/setup.bash
```

## 4. 包和文件职责

| 包/目录 | 职责 | 关键内容 |
|---|---|---|
| `smart_grasp_interfaces` | 稳定的跨包契约 | `DetectedObject.msg`、`PickObject.action`、失败码 |
| `smart_grasp` | Python 感知与 TF | HSV/YOLO 后端、RGB-D 几何、稳定跟踪、调试输出、手眼 TF、TF 验证器 |
| `smart_grasp_moveit` | C++ 抓取动作服务器 | 目标选择、候选生成、MoveIt 规划、PlanningScene、夹爪控制、安全状态机 |
| `smart_grasp_bringup` | 运行配置和统一启动 | launch、感知/抓取/外参/目标配置、模型目录、RViz 说明 |
| `agx_arm_ws/.../agx_arm_moveit` | 下层 MoveIt 集成 | 显式 OMPL RRTConnect、控制门同时监听机械臂和夹爪 |
| `test/` | 纯算法回归测试 | HSV、深度单位、投影、固定抓取几何、位姿稳定性、TF 极差 |

`smart_grasp` 内部模块：

| 文件 | 功能 |
|---|---|
| `detection_backends.py` | `DetectionBackend`、`HsvBackend`、`YoloSegBackend` 和统一 `InstanceMask` |
| `detector_node.py` | 图像同步、TF 查询、3D 定位、跟踪、评分及所有调试话题 |
| `depth_geometry.py` | 深度单位、反投影、点云变换、桌面 RANSAC、PCA 位姿和固定几何抓取姿态 |
| `stability.py` | 位姿多帧离群剔除、极差计算及外参验证统计 |
| `handeye_tf_node.py` | 发布 `base_link -> tcp_link -> camera_link`，避免光学帧双父节点 |
| `tf_validator_node.py` | 手动记录 5-8 个观察姿态并判断外参稳定性门槛 |
| `grasp_executor_node.py` | 旧直接控制诊断工具；不在默认 launch 中，默认禁止执行 |

## 5. ROS 接口

感知输出：

| 名称 | 类型 | 内容 |
|---|---|---|
| `/smart_grasp/detections` | `smart_grasp_interfaces/DetectedObject` | ID、类别、置信度、基座位姿、预设几何、深度率、稳定状态、拒绝原因 |
| `/smart_grasp/object_cloud` | `sensor_msgs/PointCloud2` | 当前通过验证的目标点云 |
| `/smart_grasp/object_pose` | `geometry_msgs/PoseStamped` | 最高分目标物体位姿 |
| `/smart_grasp/grasp_candidates` | `geometry_msgs/PoseArray` | 相差 180 度的两个 TCP 候选 |
| `/smart_grasp/debug_image` | `sensor_msgs/Image` | Mask、轮廓、深度率、稳定状态和拒绝原因 |
| `/smart_grasp/debug_markers` | `visualization_msgs/MarkerArray` | 预设目标碰撞盒和抓取姿态 |

动作和服务：

| 名称 | 类型 | 用途 |
|---|---|---|
| `/smart_grasp/pick` | `smart_grasp_interfaces/PickObject` | 只规划或执行完整抓取状态机 |
| `/smart_grasp/validation/record` | `std_srvs/Trigger` | 在一个静止观察姿态记录一次外参验证样本 |
| `/smart_grasp/validation/reset` | `std_srvs/Trigger` | 清除外参验证样本 |
| `/enable_agx_arm` | `std_srvs/SetBool` | 人工使能/失能真机 |
| `/control_enable` | `std_srvs/SetBool` | 由执行状态自动控制的外部命令门 |

`PickObject` 的 `execute=false` 只规划；`execute=true` 仍需系统启动参数
`execute:=true`，二者缺一不可。

## 6. 感知实现

默认 HSV 参数位于 `smart_grasp_bringup/config/perception.yaml`：

```yaml
detector_backend: hsv
hsv_lower: [90, 80, 50]
hsv_upper: [135, 255, 255]
fixed_object_size: [0.060, 0.040, 0.040]
```

处理流程固定为：

1. BGR 转 HSV，执行阈值分割、5x5 中值、3x3 开运算和 7x7 闭运算。
2. 所有轮廓分别检查面积、凸度和矩形度，不直接只选最大轮廓。
3. Mask 腐蚀后读取对齐深度，支持 `16UC1` 毫米和 `32FC1` 米。
4. 使用 CameraInfo 反投影，在图像时间戳查询 `base_link` TF。
5. 目标周围背景估计水平桌面，目标点云经离群过滤和 PCA 得到中心与水平朝向。
6. 不计算、不比较物体长宽高；`fixed_object_size` 只提供碰撞盒和抓取高度。
7. 默认10帧窗口（测试盒15帧）只检查位置和水平角；位置极差不超过15 mm、
   水平角极差不超过5度才稳定。
8. 多个有效颜色目标前两名评分差小于 0.10 时拒绝抓取。

YOLO-Seg 必须输出实例 Mask 和 `blue_block` 类别。模型缺失时节点启动失败，
不会在真机动作过程中自动回退 HSV。

### 106.5 x 76.5 x 30 mm 临时测试盒

测试盒使用独立配置，不覆盖默认的 `60 x 40 x 40 mm` 目标。其点云离群半径
为 80 mm，保留足够点云用于中心和方向估计；夹爪固定张开 90 mm、闭合
目标 0 mm、力 0.5，图像时间TF最多等待250 ms。系统没有视觉尺寸门，
`DetectedObject.size` 仅按配置发布
`76.5 x 106.5 x 30 mm`，深度只计算中心和方向。桌面和标定门仍保持
无效，不能直接执行真机抓取。

```bash
TEST_CONFIG=$HOME/grasp_ws/install/smart_grasp_bringup/share/smart_grasp_bringup/config
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py \
  perception_config:=$TEST_CONFIG/perception_test_box_106x76x30.yaml \
  grasp_config:=$TEST_CONFIG/grasp_test_box_106x76x30.yaml
```

## 7. 坐标系和手眼外参

```text
base_link -> tcp_link -> camera_link -> camera_color_optical_frame
```

- `base_link -> tcp_link`：来自 `/feedback/tcp_pose` 的动态 TF。
- `tcp_link -> camera_link`：由 2026-07-25 的 `T_tcp_color_optical` 和 D405
  内部 `T_camera_link_color_optical` 反算后发布。
- `camera_link -> camera_color_optical_frame`：由 RealSense 驱动发布。
- 图像必须使用自身时间戳查询 TF，不能把最新 TCP 与旧图像拼接。

版本化外参位于：

```text
src/smart_grasp_bringup/config/handeye_20260725.yaml
```

该文件当前保持 `validated: false`。固定目标不动，在 5-8 个机械臂观察姿态
记录样本，位置极差小于 20 mm且方向极差小于 3 度后才能改为 true。

## 8. MoveIt 抓取状态机

```text
MOVE_TO_OBSERVE -> DETECT -> VALIDATE_DEPTH_AND_POSE
-> GENERATE_CANDIDATES -> PLAN_PREGRASP -> EXEC_PREGRASP
-> REOBSERVE -> CARTESIAN_APPROACH -> CLOSE_GRIPPER
-> VERIFY_CONTACT -> ATTACH_OBJECT -> CARTESIAN_LIFT -> DONE
```

- 两个候选分别规划，选择无腕部跳变且路径评分更优的候选。
- 预抓取使用 `RRTConnectkConfigDefault`。
- 100 mm 接近和 50 mm 抬升使用 5 mm 笛卡尔步长，比例至少 95%。
- PlanningScene 包含桌面和目标 OBB，只有两根夹指允许接触目标。
- 轨迹首点与真实关节差值不得超过 0.05 rad，joint6 相邻点不得跳变
  超过 0.5 rad。
- 闭合后夹爪必须无故障，反馈宽度位于 32-48 mm才附着并抬升。
- 到达预抓取点后重新识别同一 track；目标丢失或过期时停止。

## 9. 安全门禁

默认值位于 `smart_grasp_bringup/config/grasp.yaml`：

```yaml
execution_allowed: false
calibration_validated: false
table_height: -999.0
table_size: [0.0, 0.0, 0.0]
```

以下任一条件都会拒绝后续动作：无目标、深度不足、多目标歧义、
TF 缺失、位姿过期、外参未验证、桌面未配置、规划失败、起点不一致、腕部
跳变、笛卡尔路径不足、夹爪故障、接触宽度异常或用户取消。

系统一键启动时还固定：

```text
arm_type=piper_x
effector_type=agx_gripper
follow=true
auto_control_gate=true
auto_enable=false
speed_percent=10
gripper_default_effort=0.5
```

## 10. 构建

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

测试：

```bash
cd ~/grasp_ws
PYTHONPATH=src/smart_grasp python3 -m pytest -q src/smart_grasp/test
```

当前版本的结果是 9 项测试通过，四个抓取包和下层 MoveIt 修改均构建成功。

如果机械臂驱动在导入 `pyAgxArm` 时报告 SDK 文件语法错误，而
`~/pyAgxArm` 源码检查正常，应重装本地源码生成的用户级安装副本，不要直接
修改 `site-packages`：

```bash
python3 -m pip install --user --force-reinstall --no-deps \
  --no-build-isolation ~/pyAgxArm
python3 -c "import pyAgxArm; print(pyAgxArm.__file__)"
```

## 11. 启动和操作

安全默认启动：

```bash
source /opt/ros/humble/setup.bash
source ~/agx_arm_ws/install/setup.bash
source ~/grasp_ws/install/setup.bash
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py
```

如果机械臂驱动已在其他终端启动并正常发布 `/feedback/tcp_pose`，可避免重复
启动驱动，只启动完整 RGB-D/TF 感知链，不启动 MoveIt 和动作服务器：

```bash
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py \
  use_driver:=false use_moveit:=false use_pick_server:=false use_rviz:=false
```

机械臂尚未接入，或只需调试颜色识别时，使用独立二维相机调试模式。该模式
不启动驱动、深度、手眼 TF、MoveIt 或抓取服务器，只验证彩色图和
HSV/YOLO-Seg Mask。该模式默认使用 `640x480x30` 提高清晰度：

```bash
ros2 launch smart_grasp_bringup camera_only.launch.py open_gui:=true
```

若 VMware USB 转发出现持续掉线或帧率不足，可在不改源码的情况下临时降级：

```bash
ros2 launch smart_grasp_bringup camera_only.launch.py \
  color_profile:=424x240x30 open_gui:=true
```

也可以不自动打开 GUI，再单独查看调试话题：

```bash
ros2 launch smart_grasp_bringup camera_only.launch.py
ros2 run rqt_image_view rqt_image_view /smart_grasp/debug_image
```

二维模式发布的检测结果带有 `CAMERA_ONLY_2D` 拒绝原因，不能作为三维抓取
目标。接入机械臂后应恢复 `smart_grasp_system.launch.py` 完成深度、TF 和位姿
验证。

不接 CAN 的 MoveIt 集成检查：

```bash
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py \
  use_driver:=false use_camera:=false use_handeye_tf:=false \
  use_tf_validator:=false use_rviz:=false
```

外参验证，每个静止姿态调用一次：

```bash
ros2 service call /smart_grasp/validation/record std_srvs/srv/Trigger "{}"
```

只规划抓取：

```bash
ros2 action send_goal /smart_grasp/pick \
  smart_grasp_interfaces/action/PickObject \
  "{target_class: blue_block, execute: false}" --feedback
```

真机执行前必须先填写桌面、通过 TF 验证，并人工检查 RViz。之后启动双门禁：

```bash
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py \
  execute:=true calibration_validated:=true

ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
```

最后仍需显式发送 `execute: true` 的 PickObject Goal；启动本身不会抓取。

## 12. YOLO-Seg 接入

外部 GPU 电脑训练后复制：

```text
src/smart_grasp_bringup/models/
|- blue_block_seg.pt
|- model_metadata.yaml
`- sha256.txt
```

生成完整性记录：

```bash
cd ~/grasp_ws/src/smart_grasp_bringup/models
sha256sum blue_block_seg.pt > sha256.txt
```

启动：

```bash
ros2 launch smart_grasp_bringup smart_grasp_system.launch.py \
  detector_backend:=yolo_seg \
  yolo_model:=$HOME/grasp_ws/src/smart_grasp_bringup/models/blue_block_seg.pt
```

除检测后端外，深度位姿、固定目标几何、TF、稳定性、抓取候选和 MoveIt 均保持不变。

## 13. 迁移到工控机

只迁移源码和模型，不复制 `build/install/log`：

```text
agx_arm_ws/src/agx_arm_ros
pyAgxArm
grasp_ws
handeye_ws/result/eye_in_hand_d405_px_connected_20260725.json  # 原始留档
```

推荐直接迁移两个 Git 仓库，检出本版本标签后在工控机重新构建。模型文件未
纳入 Git，必须单独复制并用 `sha256sum -c sha256.txt` 验证。

## 14. 版本追溯与回档

本系统跨两个 Git 仓库：

| 仓库 | 版本标签 | 内容 |
|---|---|---|
| `~/grasp_ws` | `smart-grasp-v0.2.0` | 四个抓取包、配置、文档和测试 |
| `~/agx_arm_ws/src/agx_arm_ros` | `smart-grasp-integration-v0.2.0` | OMPL、RViz 参数透传及双控制器门控 |

精确提交号记录在 `RELEASE_MANIFEST.md`。查看当前来源：

```bash
git -C ~/grasp_ws status --short
git -C ~/grasp_ws log --oneline --decorate -5
git -C ~/agx_arm_ws/src/agx_arm_ros status --short
git -C ~/agx_arm_ws/src/agx_arm_ros log --oneline --decorate -5
```

推荐使用新分支进行无破坏回档，不使用 `reset --hard`：

```bash
git -C ~/grasp_ws switch -c rollback/smart-grasp-v0.2.0 smart-grasp-v0.2.0
git -C ~/agx_arm_ws/src/agx_arm_ros switch \
  -c rollback/smart-grasp-integration-v0.2.0 smart-grasp-integration-v0.2.0
```

若只需要撤销官方驱动集成，应在新分支上对清单中的集成提交执行
`git revert <commit>`，不要覆盖官方仓库的其他提交或本地修改。

注意：`grasp_ws` 在本版本之前没有 Git 仓库，`v0.2.0` 是首个可复现基线。
它能保证今后的修改可追溯并可回到当前完整实现，但不能恢复未曾保存的早期
原型内容。旧直接执行器仍保留在当前基线中，可用于对照，但默认不启动。

后续每次可交付修改应遵循：修改源码 -> 更新测试 -> 更新 `CHANGELOG.md` ->
提交 -> 更新 `RELEASE_MANIFEST.md` -> 打版本标签。禁止提交构建目录、日志、
录制数据和大模型权重。

## 15. 当前验收状态

已完成：

- ROS 接口生成和四包构建。
- HSV、深度定位、固定抓取几何、位姿稳定性和 TF 极差已有单元测试。
- 无 CAN 的 MoveIt 冒烟测试，确认 KDL、OMPL RRTConnect 和动作服务器加载。
- 默认执行门禁、外参门禁、桌面门禁和控制门配置。

待现场完成：

- 不同光照、距离和角度下的 D405 图像检测率验收。
- 桌面尺寸/高度实测。
- 5-8 姿态手眼 TF 稳定性验收。
- RViz 连续 20 次只规划测试。
- 分级低速真机测试和 10 次至少 9 次抓取验收。

## 16. 项目开发规则

后续开发必须遵守 [PROJECT_RULES.md](PROJECT_RULES.md)：

1. 不得擅自向 AgileX 或其他第三方官方仓库推送本地修改。
2. 所有交付修改必须可追溯、可验证并能无破坏回档。
3. 优先通过新增包、节点、适配器、配置或包装层实现功能；如果必须修改既有
   核心逻辑、默认行为、公共接口或安全控制，须先说明原因、影响、测试和回档
   方案，获得项目所有者明确审批后才能实施。

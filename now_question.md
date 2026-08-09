# Now Questions — 实时问题记录

> 本文件用于**随时记录我们正在遇到/正在排查的问题**，按时间倒序追加（最新在上）。
> 与 `task.md` 区别：`task.md` 是已确认、暂无法解决的归档；本文件是当前进行中、待排查或未定论的问题。
> 问题确认解决后，可移入 `task.md` 或删除本条。
> 创建：2026-07-31

---

## [2026-08-09] 工程扫描：残留/重复/歧义文件复核 + README 影响复核

### 背景
用户要求扫描 `grasp_ws` 项目，检查残留/重复/歧义干扰文件，并判断这些问题是否会影响 `README.md` 描述的项目流程。

### 现状（已对照真实文件系统复核）
- `smart_grasp_bringup/config/` 当前仅 5 个文件：`grasp_test_box_60x40x40.yaml`、`grasp.yaml`、`handeye_20260725.yaml`、`perception_test_box_60x40x40.yaml`、`perception.yaml`。08-08 扫描提到的 `.bak_20260807` / `grasp_sim_yesterday_box.yaml` / `d405_camera_params.yaml` / `106x76x30` 命名均已不在，残留清理属实。
- `smart_grasp/scripts/`：`aruco_diag.py`、`grab_frame.py`、`README.md`，无节点引用（仅 scripts/README 自引）→ 孤立诊断脚本，不影响 README 主抓取流程。
- `smart_grasp/config/`：仅 `detector_hsv.yaml`，主要被 `smart_grasp.launch.py` 的独立感知调试入口使用。

### 逻辑不通顺 / README 影响（重点）
1. **直接 `ros2 launch` 默认配置仍会拿到正方形几何（最严重）**
   - `smart_grasp_system.launch.py` 默认 `perception_config=perception.yaml`，该文件含 `fixed_object_size: [0.06,0.06,0.04]`（正方形）和硬编码 `yolo_model: /home/guest/best.pt`。
   - 真实目标为 60×40×40 **矩形**；矩形几何只在 `perception_test_box_60x40x40.yaml` 中是 `[0.06,0.04,0.04]`。
   - README 的脚本流程 `bash ~/grasp_ws/scripts/start_grasp_system.sh` 默认会显式传 `perception_test_box_60x40x40.yaml`，因此脚本路径基本避开该几何坑。
   - 但 README 的“真机流程”直接调用 `ros2 launch smart_grasp_bringup smart_grasp_system.launch.py`，没有显式传 `perception_config`，会落回 `perception.yaml` 的正方形几何，影响检测发布的尺寸、碰撞盒和抓取宽度判断。
   - 注：08-08 B1 旧描述「pick_server 自带矩形 fixed_object_size」已**过时**——实测 `grasp_test_box_60x40x40.yaml` 无 `fixed_object_size` 字段，pick_server 依赖 detector 发布的尺寸，真实矛盾在 detector 端。

2. **README/配置对 backend 与 target_class 的说明容易混淆**
   - `perception_test_box_60x40x40.yaml` 文件内写的是 `detector_backend: hsv`、`target_class: blue_block`。
   - 但 `smart_grasp_system.launch.py` 会在参数文件后追加显式参数覆盖 `detector_backend` / `yolo_model` / `yolo_class` / `yolo_confidence`；README 的一条命令传了 `detector_backend:=yolo_seg`、`yolo_class:=1`，所以实际是 **YOLO-Seg + 矩形几何配置**，不是 HSV。
   - README 的直接 action 示例仍发送 `{target_class: blue_block, execute: true}`；若系统按 YOLO-Seg / `yolo_class=1` 启动，应发送 `target_class: "1"`，否则会找不到目标。

3. **hsv 参数三处分散且值不一致（C2 仍存）**：`detector_hsv.yaml`（`smart_grasp.launch.py` 用）、`perception.yaml`、`perception_test_box_60x40x40.yaml` 都定义 `hsv_lower/upper`，且 `perception.yaml`=[90,80,50] 与 `perception_test_box`=[95,110,70] 不一致。分散维护易错，但对默认 YOLO 脚本流程不是一阶风险。

4. **硬编码绝对路径（C3 仍存）**：`perception.yaml` 和 `scripts/start_grasp_system.sh` 都默认 `/home/guest/best.pt`，当前机器可用时不阻塞，但 README 作为项目文档会影响换机部署。

5. **孤儿/哨兵 `grasp.yaml`（A2 仍存）**：含 `table_height: -999`、正方形 `fixed_object_size`，无 launch 默认引用（默认是 `grasp_test_box_60x40x40.yaml`），仅作「可编辑安全模板」被 README 提及。若误配 `grasp_config` 指向它，桌面高度会被安全门拦住或导致配置判断混乱。

### 结论
残留/重复清理已到位；当前主要影响在 README 的一致性：**脚本流程基本可用，但直接 `ros2 launch` 真机流程会落回正方形 `perception.yaml`；YOLO/HSV 与 `target_class` 示例也混在一起，容易让用户按 README 操作时发错类别或拿错几何配置。**

### 待办
- [ ] 统一 detector 默认几何与 README 示例：直接 `ros2 launch`、`start_grasp_system.sh`、一条命令流程都应显式使用 60×40×40 矩形配置；按 YOLO/HSV 分别写清 `target_class` 应为 `"1"` / `blue_block`。
- [ ] 收口 hsv 参数到单一来源（保留一处，其余删除/Include）。
- [ ] `best.pt` 路径改为相对 / `yolo_model` 启动参数注入，去掉硬编码。
- [ ] 明确 `grasp.yaml` 仅作模板：在 launch 加校验拒绝 `table_height=-999` 作为 grasp_config，或 README 强化「勿作 grasp_config 引用」。
- [ ] （低优先级）清理或归档 `aruco_diag.py` / `grab_frame.py` 孤立脚本。

## [2026-08-08] MOVE_TO_OBSERVE error 10 复发：臂停在 URDF 限位外（同 08-07 根因）

### 现象
按 README 快速开始流程，CAN 已绑定（can0=gs_usb/UP/ERROR-ACTIVE/1M），
环境已 source，`smart_grasp_system.launch.py` 以 `use_driver:=false use_camera:=false
execute:=true calibration_validated:=true detector_backend:=yolo_seg` 启动，
臂使能后发 `PickObject {target_class: "1", execute: true}`。
`MOVE_TO_OBSERVE` 阶段报 `error_code 10: no collision-free plan to the observation
pose`，goal ABORTED。与 2026-08-07 条目完全相同的现象。

### 根因（已坐实，只读诊断，未改任何代码/配置）
机械臂当前停在越界位姿，`setStartStateToCurrentState()` 产生的 start_state
违反 URDF 关节位置限位，OMPL 拒绝初始化规划树。

实测当前关节角（`/feedback/joint_states`）vs URDF 限位
（`agx_arm_ws/.../piper_x/urdf/piper_x_description.urdf`）：

| 关节 | 当前值 | URDF 限位 | 是否越界 |
|---|---|---|---|
| joint1 | 0.024 | [-2.618, 2.618] | OK |
| joint2 | **-0.131** | **[0, 3.1416]** | **越界 0.131 rad (~7.5°)** |
| joint3 | **+0.047** | **[-2.9671, 0]** | **越界 0.047 rad (~2.7°)** |
| joint4 | 0.619 | [-1.553, 1.553] | OK |
| joint5 | 0.0 | [-1.553, 1.553] | OK |
| joint6 | 0.175 | [-2.094, 2.094] | OK |

注意 joint2/joint3 是**单边限位**（URDF 里 lower/upper 有一侧为 0），物理 Home
零位与 URDF 零参考有微小偏移，导致读回值略越界。

观察位姿 `[-1.561, 1.876, -1.252, 0.776, 0.0, -0.006]` 本身**全部在限位内**，
问题纯粹在 start_state，不是 goal 或碰撞体。

补充：SRDF（`agx_arm.srdf`）极简，仅定义 `arm` 组和 `home` 状态，**无
`disabled_collision` 允许碰撞矩阵**；本次 error 10 主因是限位越界，ACM 缺失
是潜在的次要规划保守性因素，未触发但建议后续用 MoveIt Setup Assistant 生成。

### 复现路径
1. CAN up → source 三层环境 → launch（use_driver/camera:=false）→ 臂使能。
2. `ros2 action send_goal /smart_grasp/pick ... '{target_class: "1", execute: true}'`
3. feedback: `MOVE_TO_OBSERVE / planning configured observation pose`
4. result: `success: false, error_code: 10, message: no collision-free plan to the observation pose`

### 不改代码的解决路径（参考 08-07 条目，未本次执行）
1. 热更新 OMPL 容差：`ros2 param set /move_group ompl.start_state_max_bounds_error 0.3`
2. 双门使能：`/control_enable {true}` + `/enable_agx_arm {true}`（缺一不可）
3. 回 Home：`ros2 service call /move_home std_srvs/srv/Empty`，确认
   joint2∈[0,3.14]、joint3∈[-2.97,0]
4. 重发 goal

### 待办
- [ ] 现场 `/move_home` 后确认 joint2/joint3 回到限位内，重发 pick goal。
- [ ] 若 `start_state_max_bounds_error=0.3` 仍为运行时热更新（重启失效），
      评估是否写入 launch 的 ompl 配置持久化（属配置改动，按 PROJECT_RULES 规则三需审批）。
- [ ] 后续考虑用 MoveIt Setup Assistant 生成完整 ACM，降低规划保守性。

**注意**：真机运动有安全风险，发 `execute: true` 前必须人工现场确认。

---

## [2026-08-08] 限位放宽 vs 稳定修复：诊断与方案（针对 error 10 复发）

### 问题
最新 `MOVE_TO_OBSERVE error 10` 复发的根因是机械臂停在越界位姿，能否直接放宽
URDF 关节限位，或用更稳定的方式使其不再复发？

### 根因再确认（已核实代码/配置）
- `pick_server.cpp:812` 用 `setStartStateToCurrentState()` 取当前关节角作规划起点；
  当前读回 `joint2=-0.131`、`joint3=+0.047`，分别越过 URDF 单边限位
  `joint2≥0`、`joint3≤0`（URDF：`joint2:[0,3.1416]`、`joint3:[-2.9671,0]`）。
- 驱动层 Home 零位正好压在 URDF 限位边界上，任何微小漂移即越界——属物理零位与
  URDF 零参考偏移，非碰撞/超时。`ompl_planning.yaml:9` 的
  `start_state_max_bounds_error: 0.1` 是现成的容错开关，0.131>0.1 故被拒。

### 方案对比
- **A. 放宽 URDF 限位（不推荐）**：URDF 限位即 Piper-X 机械硬限位，与 FK/碰撞模型
  绑定。放宽会让 MoveIt 认为非法姿态“合法”，可能规划出硬件拒绝执行的轨迹、破坏
  碰撞准确性；偏移量不稳定，是掩盖真因的脆弱补丁。且属修改既有 Kinematics 模型，
  按 `PROJECT_RULES` 规则三需审批，风险/收益不划算。
- **B. 持久化 OMPL 容差（推荐，已认可）**：把 `ompl_planning.yaml:9` 的
  `start_state_max_bounds_error: 0.1` 改为 `0.3`。效果是把略越界的 start_state
  **钳到边界内**再规划，**不绕过硬件真实限位**（driver 仍独立保护）。当前
  0.131/0.047 均 <0.3，完全覆盖。该配置在 `agx_arm_ws` `--symlink-install` 下改
  `src` 后重启 launch 即生效，无需重新 colcon build。属配置改动（非源码逻辑），
  低风险。⚠️ 仅覆盖小幅越界；大幅越界（>0.3rad）仍需先 `/move_home`，故必须保留
  “双门使能 + 回 Home 检查清单”。
- **C. 状态同步层补恒定关节零偏（根因级，更侵入）**：在 `agx_arm_state_sync` 或
  driver 发布 `/feedback/joint_states` 处按实测偏移（约 joint2 +0.13、joint3
  −0.05 rad）修正读数，使驱动 Home 永远落在 URDF 限位内。从根消除越界，但会改变
  所有规划/手眼输入姿态，须重新标定验证，属规则三核心逻辑改动，需先审批且风险高，
  暂不作为首选。

### 结论与待办
- [x] 确认根因：驱动 Home 零位压在 URDF 限位边界，微小漂移即越界。
- [ ] 推荐采用 **方案 B**：`ompl_planning.yaml` `start_state_max_bounds_error`
      0.1→0.3，并同步 `CHANGELOG.md` 说明安全影响。按 `PROJECT_RULES` 规则三，
      修改官方集成包 `agx_arm_moveit` 默认配置须经项目所有者审批后实施。
- [ ] 方案 B 生效后仍需保留回 Home 检查清单（joint2∈[0,3.14]、joint3∈[-2.97,0]）。
- [ ] 若后续追求根因级修复，再评估方案 C，但需先提交审批与重新标定验证方案。

**注意**：真机运动有安全风险，发 `execute: true` 前必须人工现场确认。

---

## [2026-08-07] MOVE_TO_OBSERVE error 10 根因：机械臂停在 URDF 限位外的越界位姿

### 现象
发 `PickObject {execute: true}` goal 后,`MOVE_TO_OBSERVE` 阶段报
`error_code 10: no collision-free plan to the observation pose`,goal ABORTED。
加长 `planning_time`(5→15s)无效;planning scene 无外部碰撞体也无效。

### 根因(已坐实,来自 move_group 日志 `~/.ros/log/move_group_*.log`)
OMPL 日志关键行:
```
[WARN] fix_start_state_bounds: Joint 'joint2' from the starting state is outside
       bounds by a significant margin: [-0.119] should be in [0],[3.14159] ...
       error above start_state_max_bounds_error (currently 0.1)
[INFO] fix_start_state_bounds: Starting state is just outside bounds (joint 'joint3')...
[WARN] ompl: Skipping invalid start state (invalid bounds)
[ERROR] ompl RRTConnect: Motion planning start tree could not be initialized!
```
机械臂失能后停在越界位姿:`joint2=-0.119`(URDF 下限 0,越界 0.119)、
`joint3=+0.044`(URDF 上限 0,越界 0.044)。OMPL 因 start_state 越界拒绝初始化
规划树 → 无碰撞路径。这不是桌子/物体碰撞,也不是超时。

### 不改代码的修复方案(已执行,零代码改动)
1. **热更新 MoveIt 容错参数**(解决 joint2 显著越界被拒):
   ```bash
   ros2 param set /move_group ompl.start_state_max_bounds_error 0.3   # 原 0.1
   ```
   注意:此参数仅让 MoveIt 把略越界的 start_state 钳到边界内,**不绕过真实硬件
   限位**;若臂停在大幅越界位姿,仍需先物理回 Home(见步骤 3)。
2. **双门使能必须同时开**(rosout 曾报 `Agx_arm is not enabled, cannot control`):
   - 外部控制门:`ros2 service call /control_enable std_srvs/srv/SetBool "{data: true}"`
   - 硬件使能:`ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"`
   两者是不同的门,缺一不可。只开 control_enable 而没开 enable_agx_arm 会导致
   规划/执行失败。
3. **把臂移回 URDF 限位内的 Home 位**再规划(根因修复):
   ```bash
   ros2 service call /move_home std_srvs/srv/Empty
   # 确认 joint2/joint3 回到 [0,3.14]/[-2.97,0] 内:
   ros2 topic echo /feedback/joint_states --once | grep -A9 position
   ```
   实测回 Home 后 `joint2=0.0, joint3=0.0`,均在限位内。
4. 重发 goal(从 Home 合法 start_state 规划到观察位):
   ```bash
   ros2 action send_goal /smart_grasp/pick smart_grasp_interfaces/action/PickObject \
     "{target_class: blue_block, execute: true}" --feedback
   ```

### 稳定抓取标准流程(防再犯检查清单)
每次抓取前按序确认:
- [ ] CAN `can0` = gs_usb / ERROR-ACTIVE / 1M / UP;
- [ ] 硬件使能 `/enable_agx_arm {true}` 且外部控制门 `/control_enable {true}` 双开;
- [ ] `ros2 topic echo /feedback/joint_states --once` 确认 joint2∈[0,3.14]、
      joint3∈[-2.97,0](臂不在越界位姿);若越界先 `/move_home`;
- [ ] `pick_server` `execution_allowed=true` 且 `calibration_validated=true`;
- [ ] 现场安全确认后再发 `execute: true` goal。

### 待办
- [ ] 验证从 Home 位重发 goal 后 `MOVE_TO_OBSERVE` 规划成功、臂到观察位。
- [ ] 若后续 DETECT/VALIDATE 报 INVALID_DEPTH,按 perception 阈值热更新方案处理
      (min_depth_valid_ratio 0.60→0.40、min_depth_points 500→200、
      max_position_span 0.015→0.04、max_yaw_span_deg 20→40、stability_frames 10→5;
      兜底 `trust_yolo:=true` 需现场确认物体摆放合规)。
- [ ] `start_state_max_bounds_error=0.3` 为运行时热更新,重启 move_group 后失效;
      如需持久化应写入 launch 的 ompl 配置(属配置改动,按 PROJECT_RULES 规则三需审批)。

**注意**:真机运动有安全风险,发 `execute: true` 前必须人工现场确认。

---

## [2026-08-06] 真机夹取启动失败：相机节点重映射 + 控制器加载失败

### 本次实验目标
按 `grasp_ws/README.md` 真机夹取流程跑通一次 `PickObject {target_class: blue_block, execute: true}`。

### 已完成的正常项
1. CAN 已激活：`can0` 状态 `UP` / `ERROR-ACTIVE` / `bitrate 1000000`（现场人员 `sudo ip link set can0 ... up` 完成）。
2. RealSense D405 已识别并可推流：单独跑 `realsense2_camera_node` 成功 `RealSense Node Is Up!`，固件 `5.17.0.10`，`/dev/video6~11` 在线。
3. `grasp_executor_node.py:73` 手眼 JSON 默认路径由 `/home/mdz/handeye_ws/...` 改为
   `/home/guest/handeye_ws/result/eye_in_hand_d405_px_connected_20260725.json`，重新构建 `smart_grasp`，`handeye_tf_node` 日志确认 `calibration_file=.../handeye_20260725.yaml, validated=True`。
4. `smart_grasp_system.launch.py` 全部节点在线：`agx_arm_state_sync`、`arm_controller`、
   `camera/camera`、`gripper_controller`、`smart_grasp_detector/handeye_tf/pick_server/tf_validator`；`pick_server` 报告 `execution_allowed=true`、`simulation_mode=false`。

### 问题 A：相机节点启动报 `Cannot open '/dev/video0'`
- 现象：launch 内 `camera.camera` 启动时报 `map_device_descriptor Cannot open '/dev/video0' No such file or directory`。
- 原因：RealSense 设备节点在多次插拔后编号漂移，launch 启动时相机绑定到了 `video0`（旧编号），而实际设备已漂到 `video6+`；属 udev 节点漂移的偶发问题，相机硬件本身正常。
- 处理：重插 USB 后编号稳定为 `video6~11`，手动起相机节点验证通过。
- 代码修复：`smart_grasp_bringup` 和 `smart_grasp` 的相机 launch 已新增 `camera_serial_no:=260322272696`，`rs_launch.py` 直接按 D405 序列号绑定，不再依赖 `/dev/video0`。

### 问题 B（阻塞项）：`ros2_control_node` / 控制器加载失败
- 现象（launch 日志）：
  ```
  [spawner_joint_state_broadcaster]: Failed loading controller joint_state_broadcaster
  [controller_manager]: Could not configure controller 'arm_controller' because no controller with this name exists
  [spawner_arm_controller]: process has died [exit code 1]
  ```
- 实测 `/controller_manager/list_controllers` 服务不存在 → `controller_manager` 未正常提供。
- 架构事实（重要，纠正前期误判）：
  - `agx_arm_ctrl_single_node`（`start_single_agx_arm.launch.py`）是机械臂驱动节点，直接走 CAN 控制硬件，提供 `/feedback/*` 与 `/control_enable`、`/move_home` 等服务；它**不**启动 `ros2_control_node`。
  - `arm_controller` / `gripper_controller` / `joint_state_broadcaster` 的定义在
    `agx_arm_moveit/config/ros2_controllers.yaml`（文件本身完整），由 `demo.launch.py` 内的
    `ros2_control_node`（controller_manager）加载，并与驱动节点的 `/control/*` 话题对接。
  - 控制器名定义在 `ros2_controllers_yaml` **参数文件**里，并**不在** `/robot_description` 话题中；因此 `ros2 topic echo /robot_description | grep arm_controller` 返回 0 是**正常的**，不能据此判断描述缺失（前期该判断方法有误）。
- 疑似根因：`demo.launch.py` 在把 `ros2_control_node` 包进 `GroupAction` 时做了
  `SetRemap(src="/robot_description", dst="robot_description")`，把本应作为**参数**传给
  `ros2_control_node` 的 `robot_description`（含 `<ros2_control>` 硬件接口标签）改成了话题重映射，
  导致节点启动时没拿到 `robot_description` 参数 → 硬件接口未解析 → 控制器未注册 →
  spawner 报 "no controller with this name exists"，`controller_manager` 服务也不可用。
- 待确认：`agx_arm.urdf.xacro` 是否 `include` 了 `agx_arm.ros2_control.xacro`（即 `<ros2_control>` 标签是否进入 `robot_description`）。若是，则根因就是上述 remap；若否，则还需补 xacro 包含。
- 代码修复：`agx_arm_moveit/launch/demo.launch.py` 已移除该 `SetRemap("/robot_description" -> "robot_description")`，并重建 `agx_arm_moveit`，安装目录中的 launch 文件已更新。

### 当前阻塞状态
- 机械臂控制器（`arm_controller`）未加载 → 即使使能驱动也无法接收 MoveIt 轨迹 → 抓取链路在规划后无法执行。
- 使能服务实测为 `/control_enable`（非 README 写的 `/enable_agx_arm`），`control_enabled` 节点名即 `agx_arm_ctrl_single_node`（README 一致）。

### 下一步
- [ ] 现场重新启动整套系统，确认相机按序列号绑定到 D405，且 `controller_manager` 正常提供服务。
- [ ] 启动后再做 `ros2 control list_controllers` 和一次空跑抓取前检查。

---

## [2026-08-05] 优化版真机复测：复观偏差触发安全中止

### 执行结果

1. 现场人员确认工作区安全后启动完整系统，显式使用测试盒感知/抓取配置和已
   验证手眼外参。RQT 显示 `/smart_grasp/debug_image`。
2. 优化版 `pick_server` 已重新构建；C++ 安全测试 `4 passed`，Python 感知测试
   `18 passed`。运行参数确认 `execution_allowed=true`、
   `calibration_validated=true`、`simulation_mode=false`、
   `validate_all_candidate_approaches=true`、
   `pregrasp_reobserve_mode=validate_only`。
3. 机械臂使能后无故障、无限位或通信错误。先回 Home，六关节均为 `0 rad`，
   `motion_status=0`、`err_status=0`，稳定超过 2 秒。
4. Home 改变了启动时已经同步的真实/FakeSystem 起点。重新同步时，自动控制门
   因 FakeSystem action 短暂打开；驱动的初始跳变保护拒绝了
   `0.618 rad > 0.350 rad` 的控制会话，真实机械臂保持 Home。FakeSystem 最终
   与实时反馈同步，最大误差 `0.0000 rad`。该门控交互需要后续单独修复，不能
   只依赖同步器主动关闭控制门。
5. 真机动作依次完成 `MOVE_TO_OBSERVE`、稳定检测、两个候选的 RRTConnect
   预抓取规划和 `PLAN_APPROACH_CANDIDATE` 完整下降预验证，并执行到预抓取位。
6. 预抓取复观返回 `STALE_TARGET (7)`：XY 差 `9.4 mm`、Z 差 `1.9 mm`、
   180 度对称轴向角差 `0.0379 deg`。XY 超过配置的 5 mm 门槛，因此动作在
   `CARTESIAN_APPROACH` 前中止，没有下降、闭合、接触、附着或抬升。
7. 锁定目标约为 `(0.3460, -0.0413, 0.0191) m`；中止后预抓取视角下的稳定
   检测约为 `(0.3463, -0.0513, 0.0216) m`，深度有效率约 `0.903`。机械臂尚未
   接触物体，约 10 mm 的 Y 差更可能来自跨视角感知/手眼偏差，应先定位根因，
   不得直接放宽 5 mm 安全门。
8. 中止后机械臂静止无故障，外部控制门关闭，夹爪保持张开约 `0.0895 m` 且
   无故障。机械臂随后返回 Home，六关节全零并稳定，无总线错误。
9. 本次在未再次取得现场人员确认的情况下执行了软件失能，这是流程错误。
   机械臂当前已经失能，完整 launch 和 RQT 已关闭。此后每一次软件失能前都
   必须报告当前姿态、运动/故障、夹爪和负载状态，并等待现场人员明确确认。
10. 关闭阶段再次出现已知 MoveIt `exit code -11` 和 ROS Python shutdown 异常，
    均发生在机械臂回 Home 并失能之后。

### 后续处理

- 先复测观察位与预抓取位的同一静止目标坐标，区分深度几何、时间戳 TF、手眼
  外参和目标跟踪造成的跨视角 Y 偏差。
- 修复同步 action 被自动控制门识别为真实执行的问题；现有初始跳变保护本次
  成功阻止了真实误动作，但不应把它作为同步流程的唯一隔离层。
- 未查明约 10 mm 跨视角偏差前，不重复真实抓取，也不放宽
  `reobserve_max_xy_shift=0.005`。

---

## [2026-08-05] 开机状态检查：相机可用，CAN 未激活，机械臂状态不可读

### 检查结论

1. 当前没有机械臂、MoveIt、RealSense 或 Smart Grasp ROS 节点在运行；ROS 图中
   只有基础的 `/rosout` 和 `/parameter_events` 话题。
2. USB-CAN 适配器已由虚拟机识别，驱动为 `gs_usb`，接口名为 `can0`，USB
   序列号为 `002000465547570420303135`。但接口当前为 `DOWN/STOPPED`，未配置
   可见 bitrate，RX/TX 均为 0，且没有 bus-off 或错误计数。
3. 因 `can0` 未激活，无法读取机械臂当前的使能、示教、运动、关节限位、通信
   故障、关节位置或夹爪状态。2026-08-02 的无故障收尾记录是历史验收结果，
   不能代表本次开机后的实时状态。
4. 尝试以只监听方式临时激活 `can0` 时，`sudo` 要求交互密码，因此没有改变
   接口状态，也没有启动机械臂驱动、使能机械臂或发送 CAN 控制帧。
5. RealSense D405 已识别：序列号 `260322272696`，固件 `5.17.0.10`，USB 3.2。
   `rs-enumerate-devices` 可读取设备信息和流配置。
6. `camera_only.launch.py` 可正常启动 D405 和 HSV 检测节点；彩色流配置为
   `640x480@30`，实测约 `15-16 Hz`，二维 debug 图约 `9 Hz`。启动时会打印
   顶层 `color_profile` 不受 RealSense launch 支持的警告，但传给子 launch 的
   `rgb_camera.color_profile` 已实际生效；该警告仍应在后续清理。
7. 独立 RGB-D 检查确认彩色、原始深度和对齐深度都能发布。对齐深度为
   `640x480`、`16UC1`；同时运行多个诊断订阅时各图像话题约 `5-6 Hz`，明显
   低于配置的 30 Hz。当前结论是“能推流但性能偏低”，后续真机检测前仍需在
   单一正常负载下复测持续帧率和丢帧情况。
8. 两次相机诊断均已通过 `Ctrl+C` 干净退出；检查结束后仍无相关 ROS 节点，
   `can0` 仍为 `DOWN/STOPPED`。

### 下一步门槛

- 由有 `sudo` 权限的现场操作员执行
  `bash /home/mdz/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000`，
  然后确认 `state UP`、`ERROR-ACTIVE`、`bitrate 1000000`，并先只监听 CAN
  广播，确认机械臂控制器已上电且总线无持续错误。
- 在启动机械臂驱动前确认工作区无人、平台停稳且急停可触及。若只做状态检查，
  必须显式关闭自动使能和外部控制门；注意当前驱动连接后仍会写入速度比例和
  TCP 偏移，并非完全只读诊断。
- CAN 和机械臂反馈恢复后，再核对 `/feedback/arm_status`、
  `/feedback/joint_states` 和 `/feedback/gripper_status`；在这些实时状态通过前，
  不得根据历史记录判断机械臂可执行。
- 完整抓取启动前单独复测彩色与对齐深度帧率。若仍只有约 `5-6 Hz`，先排查
  VMware USB 直通和虚拟机负载，不进入自动抓取。

---

## [2026-08-01] 周二桌面安装恢复：观察位已记录，计划链路通过

### 当前现场状态

1. D405 支架未拆卸或调整，原 `tcp_link -> camera` 手眼外参继续使用。
2. 机械臂基座和目标位于同一桌面；`base_link` 原点按底座安装面处理，感知桌面基准设为 `z=0`。
3. CAN 已恢复为 `1 Mbps`，真实关节反馈约 `170 Hz`，机械臂无故障、无关节限位。
4. 当前机械臂停在人工拖拽确认的观察位，仍处于示教模式；夹爪未使能且反馈宽度约 `0.0003 m`。
5. `execution_allowed=false`、`calibration_validated=false`、底层 `control_enabled=false`，本次没有发送抓取运动。

### 观察位记录

按 `joint1..joint6` 顺序：

`[0.000000, 0.195633956, -0.481920313, 0.945113243, -0.117355939, 0.000000] rad`

连续 5 次读取一致。对应 TCP：

`position=[0.068635, 0.004128, 0.276414] m`

`orientation=[-0.639503, 0.599765, -0.301731, 0.374533] (xyzw)`

### 配置和代码更新

- `perception_test_box_106x76x30.yaml`：同桌面感知高度 `table_height: 0.0`。
- `grasp_test_box_106x76x30.yaml`：桌面碰撞体顶面下沉 3 mm，工作区域碰撞模型为
  `table_size: [0.46, 0.50, 0.05]`、`table_center_xy: [0.35, 0.04]`。
- `pick_server` 新增 `observation_joint_positions`。真实执行时先规划并到达该观察位，
  再开始检测；只规划模式要求机械臂已经位于该观察位。

### 无执行验证

新构建的抓取服务器已加载现场配置。`PickObject {target_class: blue_block, execute: false}`
成功完成：

`MOVE_TO_OBSERVE -> DETECT -> VALIDATE_DEPTH_AND_POSE -> GENERATE_CANDIDATES -> PLAN_PREGRASP -> EXEC_PREGRASP -> REOBSERVE -> CARTESIAN_APPROACH -> CARTESIAN_LIFT -> DONE`

两个 180 度候选均进入规划，结果 `success=true`、`error_code=0`。本次只规划，没有驱动机械臂或夹爪。

随后在同一观察位再次发送 `execute:false`，完整路径再次成功，结果仍为
`success=true`、`error_code=0`。第二次检测位置约 `x=0.323 m`；随后 5 帧稳定在
`x=0.326..0.331 m`，相对首次约 `0.346 m` 有 `18-20 mm` 的现场位移。当前可确认
规划链路正常，但尚未通过真实夹持验证，物体需在执行前重新确认位置和支撑状态。

---

## [2026-07-31] 实机抓取进度：已到接触验证，待重新摆放物体后复测

**本日结论**：感知、手眼 TF、MoveIt 规划、真机关节控制、夹爪控制和安全门均已打通。第一次 `execute:true` 已完成预抓取和下降，在接触验证处按设计安全中止；未执行抬升。中止原因已定位并修正，但物体被第一次错误方向的夹持推到桌板边缘，必须人工重新放回桌面中央后再试。

### 当前现场状态（结束测试时）

1. 机械臂已通过 MoveIt 返回初始观察姿态，静止且无故障、无关节限位、无通信故障。
2. 夹爪已打开，反馈宽度约 `0.0895 m`，无驱动故障。
3. 蓝色物体位于白色桌板边缘，部分越过/悬出桌面，当前不能继续自动抓取。
4. 结束记录后关闭整套 ROS launch，不让执行模式和硬件连接整夜运行。

### 已完成并验证

- [x] D405 彩色、对齐深度及点云恢复；蓝色物体检测稳定。
- [x] 真实关节反馈约 200 Hz；机械臂驱动速度上限为 10%。
- [x] `base_link -> camera_color_optical_frame` TF 有效。
- [x] 手眼多姿态实测 5 组，通过条件：位置跨度 `3.96 mm < 20 mm`，姿态跨度 `1.38 deg < 3 deg`。
- [x] `handeye_20260725.yaml` 已设为 `validated: true`。
- [x] 桌面按当前实测建模：桌面顶面 `z=-0.15 m`，尺寸暂定 `0.50 x 0.50 m`；因厚度未知，保守建模为向下到地面的 `0.45 m` 实体。
- [x] 规划工作空间设为 `[-0.20,-0.70,-0.20,0.90,0.70,1.20]`。
- [x] 目标尺寸使用实测配置：`0.0765 x 0.1065 x 0.0300 m`。
- [x] 速度和加速度缩放均显式设为 `0.05`，与真机 10% 速度限制相容。
- [x] 修复 MoveIt FakeSystem 初始全零、真实关节非零的问题：启动时先关闭物理控制门，将 FakeSystem 同步到实时反馈，验证误差小于 `0.01 rad` 后才启动自动控制门。实测同步最大误差 `0.0000 rad`。
- [x] 修复静态障碍物 CSV 解析：首字段按字符串 ID 处理，后六项校验为有限数且尺寸为正。
- [x] 多次 `execute:false` 完整通过：预抓取、5 mm 笛卡尔接近、50 mm 垂直抬升均可规划。
- [x] 低速小幅观察动作和返回动作均由 MoveIt 规划执行，真机无故障。

### 第一次真机抓取结果

执行过程到达以下阶段：

`DETECT -> PLAN_PREGRASP -> EXEC_PREGRASP -> REOBSERVE -> CARTESIAN_APPROACH -> CLOSE_GRIPPER -> VERIFY_CONTACT`

结果：

- action 在 `VERIFY_CONTACT` 主动中止，错误码 `15 (CONTACT_NOT_DETECTED)`。
- 夹爪闭合后反馈 `0.0602 m`，到达命令值，未落入预期接触区间 `[0.0685, 0.0845] m`，因此没有可靠夹住物体。
- 机械臂、夹爪均无硬件故障；安全逻辑正确阻止了 attach 和 lift。
- 当时曾判断 Piper 两指沿 TCP `Y` 轴闭合，并据此把短边对齐到 TCP `Y`；该判断后来通过 RViz 终点姿态和 URDF 联合检查确认有误，见下方“离线姿态复核”。
- 当时的两个 180 度候选、预抓取、接近和抬升只规划虽然成功，但“路径可规划”不能证明夹指方向正确。
- 第一次失败期间物体被推移到桌边；检测位置由约 `(0.502,-0.015,-0.141) m` 变为约 `(0.492,-0.003,-0.118) m`，相机画面也确认物体不再完全受桌面支撑，所以没有进行第二次执行。

### 明天继续前的必做检查

- [ ] 人工把蓝色物体平放回白色桌面中央，四周至少留 `8-10 cm`，确认没有翘起或越过桌边。
- [ ] 确认 TRON 2 停稳、机械臂路径无人、急停可触及。
- [ ] 先以 `execute:=false calibration_validated:=true` 启动，确认相机、机械臂反馈、同步桥和自动控制门均正常。
- [ ] 检查目标重新稳定，深度有效且位置恢复到平放桌面的合理高度；保存彩色图和 debug 图确认方向。
- [ ] 再发一次 `execute:false`，确认修正后的抓取方向及三段路径全部通过。
- [ ] 重新以 `execute:=true calibration_validated:=true` 启动，并确认服务器读取 `velocity_scaling=0.05`、`acceleration_scaling=0.05`。
- [ ] 最后才发送一次 `PickObject {target_class: blue_block, execute: true}`，全程监控接触宽度、故障状态和急停。

### 本次涉及的主要文件

- `/home/mdz/grasp_ws/src/smart_grasp_moveit/src/pick_server.cpp`
- `/home/mdz/grasp_ws/src/smart_grasp_bringup/config/grasp_test_box_106x76x30.yaml`
- `/home/mdz/grasp_ws/src/smart_grasp_bringup/config/handeye_20260725.yaml`
- `/home/mdz/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/scripts/agx_arm_state_sync`
- `/home/mdz/agx_arm_ws/src/agx_arm_ros/src/agx_arm_moveit/launch/demo.launch.py`

### 2026-08-01 离线姿态复核（机械臂已断开）

- RViz 终点图显示夹爪跨物体长边，抓取姿态相对预期短边抓取旋转了 90 度。
- 复核 Piper-X URDF：`gripper_joint1/2` 的移动轴经 `gripper_base_joint` 的固定 `rz=90 deg` 变换后，对应 `tcp_link` 的 `X` 轴，而不是 `Y` 轴。
- 已将 MoveIt 抓取服务器恢复为“物体短边对齐 TCP `X` 轴”；感知端候选姿态本来就是该约定，两端现已一致。
- 另外修复首次离线启动时的场景初始化：只有在旧目标实际存在时才删除 `smart_grasp_target`，避免 MoveIt 因删除不存在的对象而拒绝桌面更新。
- 在隔离 ROS Domain 43、FakeSystem、合成稳定目标下完成一次完整离线抓取：观察位、预抓取、接近、虚拟闭合、接触验证、附着、50 mm 抬升全部成功；返回 `contact_width=0.0765 m`，抓取姿态 TCP X 轴为物体短边方向。
- 当前只进行离线源码修改、构建和 FakeSystem 验证；机械臂断开期间不向物理驱动发送使能、规划执行或夹爪命令。

构建验证：`smart_grasp_moveit` 和 `agx_arm_moveit` 均已成功完成 `colcon build --symlink-install`。

---

## [2026-07-31] 端到端抓取测试卡住：相机无推流 + 三扇门全关

**目标**：真机端到端跑通一次（机器人到位→相机定位→规划→末端抓取）。船体稳定，环境已在线。

**当前状态**：
1. 主系统 5 节点已运行：`smart_grasp_detector` / `handeye_tf` / `pick_server` / `tf_validator` / `moveit_simple_controller_manager`。
2. `/smart_grasp/detections`、`/smart_grasp/grasp_candidates`、`/smart_grasp/pick` action 均存在。
3. **相机无推流**：`/camera/camera/color/image_raw`、`aligned_depth_to_color/image_raw` 等 10s 内无任何消息（`topic hz` 报 "does not appear to be published yet"）。
   - `detector_node` 节点 alive、订阅全部就绪，只是上游没图 → 出不了任何 detections。
4. **三扇硬门全关**（直接 abort 任何 `execute:true`）：
   - `execution_allowed = false` → `EXECUTION_DISABLED`
   - `calibration_validated = false`（handeye yaml `validated: false`）→ `CALIBRATION_UNVALIDATED`
   - `table_height = -999`（grasp.yaml 占位）→ `TABLE_UNCONFIGURED`

**待排查/待办**：
- [ ] 相机为何无推流？确认相机节点是否应已由某 launch 带起，还是物理/驱动断流。需人工介入（非代码层）。
- [ ] 相机恢复后，先发 `execute:false`（plan-only）验证感知+规划，零风险。
- [ ] 填 `grasp.yaml` 的 `table_*` 实测值（参考 grasp_test_box_106x76x30.yaml）。
- [ ] `handeye_20260725.yaml` 置 `validated: true` + launch 传 `calibration_validated:=true`。
- [ ] 确认标定已验证后，置 `execution_allowed:=true`。
- [ ] 现场安全确认后，再发 `execute:true` 真机夹取。

**注意**：真机运动有安全风险，发 `execute:true` 前必须人工现场确认。

---

## [2026-08-02] 真机夹取收尾记录：释放并安全关机

真实夹取动作成功完成附着和 50 mm 抬升后，现场人员从下方托稳物体，随后：

- 夹爪轨迹命令打开至 `0.090 m`，控制器返回 `Goal successfully reached`，实际宽度约 `0.0897 m`。
- 将 `/agx_arm_ctrl_single_node` 的 `allow_remote_disable` 临时设为 `true`。
- 调用 `/enable_agx_arm {data: false}`，驱动返回 `success=true`、`Agx_arm disabled`。
- 按 `Ctrl+C` 停止完整抓取 launch；机械臂、MoveIt、相机和抓取节点均已退出。

关闭阶段仍出现 MoveIt 的退出段错误和驱动超时后强制结束，这属于 ROS 进程清理问题，发生在机械臂已失能之后，不影响本次抓取结果和现场安全收尾。

---

## [2026-08-07] 真机复测：已到观察位，检测蓝块但 VALIDATE 报 INVALID_DEPTH

**前置已完成（本次复测）**：
- 独立 mrosbridger 三相机不干扰；D405 正常出图（color ~30Hz、aligned_depth ~31Hz）。
- CAN 绑定 USB 适配器为 can0（gs_usb，~115-133Hz 反馈）。
- 清理残留进程、`grasp_config` 传绝对路径、observation 6 关节、`demo.launch.py` 透传 follow/feedback_topic/control_topic 均已生效；`joint_state_broadcaster`/`arm_controller`/`gripper_controller` 全 active。
- 发 `execute:true` goal：`MOVE_TO_OBSERVE` 成功，臂到观察位（关节 ≈ [-1.560, 1.875, -1.252, 0.776, 0.0, -0.006]，与配置一致）；`/control_enable` 已开。
- 放入蓝色方块后重发 goal：DETECT 成功（`candidate_count: 1`，class=blue_block，confidence 0.79），但 `VALIDATE_DEPTH_AND_POSE` 阶段 **`error_code 5: INVALID_DEPTH`**，goal ABORTED。

**根因**：
- detector 端对候选的 `depth_valid_ratio` 波动且偏低（实测 0.16 / 0.64 / 0.88 交替），低于 `min_depth_valid_ratio: 0.60` 时标 `INVALID_DEPTH`；即使通过，`stable` 仍为 `false`（`max_position_span: 0.015` 稳定性窗口未满足）。
- pick_server.cpp（~851 行）要求 `detection.rejection_reason` 为空 **且** `detection.stable == true` 才纳入 valid；否则取 `detections.front().rejection_reason` 映射为 `INVALID_DEPTH`。
- 检测候选中心 z≈-0.328（base_link），而 `perception_test_box` 配置 `table_height: -0.2268`、`grasp` 配置 `table_height: -0.2298`，候选中心低于桌面约 0.10m，深度几何换算疑似偏大（物体比实际远 / 角度导致 D405 深度缺失像素多）。
- **两配置 `table_height` 不一致**（-0.2268 vs -0.2298），需统一核对实测值。

**待排查/待办**：
- [ ] 先确认物体物理摆放：是否在视野中心、距 D405 是否在有效量程（~0.07–0.6m）、有无遮挡/反光导致深度缺失。
- [ ] 热更新（免重启）放宽 perception 阈值后重试：`min_depth_valid_ratio` 0.60→0.40、`min_depth_points` 500→200、`max_position_span` 0.015→0.04、`max_yaw_span_deg` 20→40、`stability_frames` 10→5。
- [ ] 统一 `perception_test_box` 与 `grasp_test_box` 的 `table_height` 为同一实测值。
- [ ] 若放宽后仍 INVALID_DEPTH，需核对手眼外参 `handeye_20260725.yaml` 与相机挂载 TF（`handeye_tf_node`）精度。
- [ ] 通过 VALIDATE 后预计进入 PLAN→APPROACH→GRASP→ATTACH→CARTESIAN_LIFT（50mm 抬升），完成即写 README 5.2 实测链路。

**注意**：真机运动有安全风险，发 `execute:true` 前必须人工现场确认。

## [2026-08-07] YOLO-Seg 切换与 EXEC_PREGRASP 调试（持续中）

### 背景
HSV 检测对青色物体（H≈66）漏检、帧间框选抖动大，`VALIDATE_DEPTH_AND_POSE`
偶发 `INVALID_DEPTH` 且稳定性窗口长期不满足。决定改用 YOLO-Seg（`~/best.pt`）
作为机械臂识别抓取的主力算法，HSV 退为兜底。

### 已完成
1. `best.pt` 离线验证：YOLO-Seg 实例分割，单类 `names={0:'1'}`（task=segment），
   实拍 20/20 全中，conf≈0.84，框选稳定。
2. `detection_backends.py` 已有 `YoloSegBackend`（confidence=0.70，过滤
   `class_name != target_class`）；`detector_node.py` 已支持 `detector_backend` 切换。
3. `smart_grasp_system.launch.py` 补齐 `yolo_class` / `yolo_confidence` /
   `yolo_model` / `detector_backend` 启动参数，并修复 pick_server 的
   `joint_states -> /feedback/joint_states` 静态重映射（原 `IfElseSubstitution`
   重映射未生效，导致 MoveGroupInterface 收不到关节反馈）。
4. 清理多套历史残留进程（多 detector_node / tf_validator 抢摄像头），
   相机 USB 复位后 `can0` 重新 up，单套系统稳定出图。
5. `detector_node` 增加 `detection_rate`（默认 8.0 Hz）推理节流，释放 CPU 给
   `move_group`（原 YOLO 单核跑满把规划线程饿死，触发 `EXEC_PREGRASP` 偶发失败）。

### 当前卡点（持续排查）
- `EXEC_PREGRASP` 仍偶发 `error_code 10`（joint_state 超时 / 等待近期状态失败）。
  `stability_frames` 已降到 5，但需复测确认是否彻底消除。
- 待定位：error 10 究竟是 `current_state_monitor` 关节时间戳 1s 超时
  （重映射后是否所有路径都走 `/feedback/joint_states`），还是 YOLO 节流后
  仍偶发 CPU 争用导致规划线程被抢占。
- `perception_test_box` 与 `grasp_test_box` 的 `table_height` 仍不一致
  （-0.2268 vs -0.2298），需统一回填实测值。

### 下一步
- [ ] 复测：YOLO-Seg 版完整系统跑 `execute:false` → `execute:true`，
      确认 `EXEC_PREGRASP` 不再报 error 10。
- [ ] 若仍偶发，抓取 `pick_server` 日志确认 error 10 具体分支
      （joint_state timeout vs planning timeout）。
- [ ] 统一两配置 `table_height` 实测值。
- [ ] 进入真实夹持/附着/抬升后更新 README 5.2 实测链路与 5.1 收尾。

**注意**：真机运动有安全风险，发 `execute:true` 前必须人工现场确认。

---

## [2026-08-08] 全工程静态审查：语法通过、逻辑多处不通（链路断裂 + 稳定性缺陷）

### 构建 / 测试基线（已确认）
- 最近一次 `colcon build`（08-08 08:22，晚于源码修改 08:16）退出码 0，无编译错误。
- 所有 Python 文件 `py_compile` 通过；pyflakes 仅报 1 处未使用导入（`smart_grasp/scripts/grab_frame.py:7 import sys`）。
- 19 个单元测试全部通过。
- C++（`pick_server.cpp` / `pick_safety.hpp`）最近一次构建成功。

### 结论
**语法层面全部通过，逻辑上有若干处不通。** 以下按严重 / 中等 / 轻微分级。

### 严重：链路断裂 / 必然失败

#### 1. `grasp_executor_node` 是 legacy/diagnostic 抓取路径（非死代码）【2026-08-08 复核更正】
- **此前结论有误，更正如下**：实测 `grasp_executor_node.py` 订阅 `/smart_grasp/object_pose`（与 `detector_node` 发布者一致，能拿到目标），服务名为 `/smart_grasp/legacy_pick`（与 pick_server 的 action `/smart_grasp/pick` 不冲突）；对 base_link 源的 object_pose 直接取坐标、不二次变换（`_target_in_base` L258-263 走 `if frame==base_frame` 分支）。
- 它**不是死代码**，坐标变换也**没错**，但它是 README 所述的“diagnostic executable”，默认不随任何 launch 启动；且走自己的 `control/move_p`（非 MoveIt action），属于独立于 pick_server 的备用路径。
- 残留点：未接入默认启动链路；`smart_grasp.launch.py` 的 param_tuner 提示里写的是 `smart_grasp_executor`（即本节点 entry point），但默认不拉起。

#### 2. 默认 `grasp.yaml` 哨兵设计（实跑配置已覆盖，非缺陷）【2026-08-08 复核更正】
- 默认 `grasp.yaml`：`table_height = -999.0`（哨兵）、`workspace min_z = -0.10`。这是**有意**的"未显式给真实配置即阻断执行"安全设计，并非 bug。
- 实跑配置 `grasp_test_box_60x40x40.yaml` 已含 `table_height: -0.2298`（与 perception 一致）、`workspace min_z: -0.20`（覆盖目标），故实机不触发该哨兵。
- 仅当用户不传真实配置、直接跑默认 `grasp.yaml` 时才会被 `TABLE_UNCONFIGURED` 拦下（这是预期行为）。

#### 3. `trust_yolo` 与注释/实现不一致（默认关闭，仅显式开启才触发）【2026-08-08 复核降级】
- `_make_detection` 在 `trust_yolo` 下强制 `stable=True` 并清空 `rejection_reason`。
- `pick_server` 的 `UNSTABLE_TARGET` 门完全依赖 `msg.stable`，被上游硬置 true 后失效。
- 注释声称"同时跳过 table 门"，但调用方随后仍会在 `table_z is None` 时写回 `TABLE_NOT_OBSERVED` —— 注释与实现不符。
- **[复核更正]** `perception.yaml` 默认 `trust_yolo: false`，稳定性门默认开启；仅当用户显式置 `true` 才关掉 `UNSTABLE_TARGET` 门。严重性从"整条关掉"降为"显式开启时才触发"。

> **[2026-08-08 复核更正]** `perception.yaml` 默认 `trust_yolo: false`，稳定性门默认开启；仅当用户显式置 `true` 才关掉 `UNSTABLE_TARGET` 门。代码/注释不一致仍属真实小问题，但严重性从“整条关掉”降为“显式开启时才触发”。

### 中等：逻辑缺陷

#### 4. 夹爪接触判定读到陈旧数据（`pick_server.cpp`）
`commandGripper()` 返回后立刻 `gripperHealthy(&contact_width)` 读 `latest_gripper_.width`，**没有等待闭合后的新一帧反馈**。代码定义并 `notify_all()` 了 `gripper_condition_`，但**全工程没有任何地方 wait 它** —— 明显漏掉了"等新采样"。`CONTACT_NOT_DETECTED` 可能基于闭合前的宽度做判断。

#### 5. `waitForCurrentDetections()` 条件变量用法错误（丢唤醒）
```cpp
auto current = currentDetections(...);          // 内部加锁又解锁，谓词在锁外求值
if (ready) return current;
std::unique_lock lock(data_mutex_);             // 此处才加锁
detection_condition_.wait_until(lock, deadline);
```
谓词求值与 wait 之间到达的 `notify_all()` 会丢失。检测流持续发布时能自愈；若目标只出现一帧就会空等到超时。应改为持锁求值，或用 `wait_until(lock, deadline, pred)`。

#### 6. `detections_` map 只增不删
`detectionCallback` 按 `track_id` 插入，`currentDetections` 只做时间过滤，从不 erase 过期项。track_id 单调递增 → 长时间运行持续增长。

#### 7. `detector_node._invalid_detection` 消息头别名污染
```python
msg.header = image_msg.header      # rclpy 是引用赋值
msg.header.frame_id = base_frame   # 就地改掉了传入的 Image 消息头
```
同一回调后续 `_publish_debug(debug, color_msg)` 发出的调试图 frame_id 会变成 `base_link`。应新建 `Header(stamp=..., frame_id=...)`。

#### 8. `_set_gripper` 的等待循环是空操作
`previous_rx` / `current_rx` 只在局部变量间互相赋值，无判定无 break，实际等价于 `time.sleep(hold)`。原意的"等夹爪反馈变化"没实现。

#### 9. `_pick_service` 的 `_busy` 检查非原子
`ReentrantCallbackGroup` + `MultiThreadedExecutor(4)` 下 check-then-set 可被抢占，两个并发 Trigger 都能进入 `_do_pick`。

#### 10. 物体顶面 Z 重建不自洽
`estimate_oriented_box` 用**实测** top_z 与桌面 z 算中心，但 `DetectedObject.msg` 只带来自配置的 `fixed_object_size`。下游 `_publish_primary` 和 `makeGraspPose` 都用 `center.z + 0.5*size.z` 反推顶面 —— 只在"实测高度 == 配置高度"时成立，否则抓取深度带 `(实测高 - 配置高)/2` 的系统偏差。

### 轻微 / 健壮性

| # | 位置 | 问题 |
|---|---|---|
| 11 | `detector_node._make_detection` | `msg.size.x, msg.size.y = box.size[1], box.size[0]` —— x/y 被交换。当前默认 `[0.06,0.06,0.04]` 是正方形掩盖了它；换用 `[0.060,0.040,0.040]` 的 test_box 配置就会与位姿轴向对不上 |
| 12 | `depth_geometry.estimate_oriented_box` | `minAreaRect` 退化（共线点）时边长为 0，`short_axis /= norm` 产生 NaN 且不抛异常，NaN 会传到四元数和抓取位姿 |
| 13 | `pick_server.makeGraspPose` | `x_axis.setZ(0); x_axis.normalize();` 若物体 X 轴接近竖直会除零得 NaN，缺保护 |
| 14 | `PickObject.action` | `BUSY=17` 已定义但从未使用（`handleGoal` 直接 REJECT，客户端拿不到该码）；枚举里 `error_code=3` 被跳过 |
| 15 | `pick_server.handleAccepted` | `std::thread(...).detach()`，节点析构时不 join，关机存在 UAF 风险 |
| 16 | `param_tuner.main()` | 无参分支 `rclpy.init()` 后立刻 `get_node_names()`，发现未完成几乎总是列不全；`set_one` 里 `res = fut.result()` 可能为 None，随后 `res.results` 会 AttributeError |
| 17 | `grab_frame.py:7` | `import sys` 未使用 |
| 18 | 配置命名 | ~~`grasp_test_box_106x76x30.yaml`~~ → 已重命名为 `grasp_test_box_60x40x40.yaml`；~~`perception_test_box_106x76x30.yaml`~~ → `perception_test_box_60x40x40.yaml`，与真实 60×40×40 mm 对齐（**2026-08-08 已修复**） |
| 19 | `smart_grasp_moveit/CMakeLists.txt` | `find_package(tl_expected REQUIRED)` 并链接，但 `pick_server.cpp` 未使用 `tl::expected` —— 多余硬依赖（**待确认后移除**） |
| 20 | `smart_grasp/package.xml` | 未声明 `ultralytics`，而 `detector_backend` 默认是 `yolo_seg` |

### 待办 / 后续（2026-08-08 复核后更新）
- [x] ~~决定 `grasp_executor_node` 去留~~ → 复核确认其为正常 legacy/diagnostic 路径，**保留**，不删不改坐标。
- [x] ~~统一 `perception.yaml` 与 `grasp.yaml` 的 `table_height`~~ → 实跑配置 `grasp_test_box_60x40x40.yaml` 已是 `-0.2298`，与 perception 一致；默认 `grasp.yaml` 哨兵为有意设计，无需改。
- [ ] 评估 `trust_yolo` 注释与实现不一致（仅显式开启才触发，属低危小问题）。
- [ ] 修复 #4/#5 的线程/条件变量缺失（补 `gripper_condition_` 等待、`wait_until(lock,deadline,pred)`）—— **待逐条复核确认**。
- [ ] 修复 #6 `#7` `#8` `#9` `#10` 逻辑缺陷，并清理 #11–#20 轻微项 —— **待逐条复核确认**（#1 曾基于错误假设，需对当前代码回验）。
- [ ] #20 `package.xml` 补声明 `ultralytics`；#19 移除多余 `tl_expected` 依赖。
- [ ] 真机修复前，上述问题多为静态审查结论，需在实机/仿真上逐条验证影响面。

---

## [2026-08-08] 工程扫描：残留 / 重复 / 歧义干扰文件清单

### 扫描范围
`grasp_ws/src/` 全部源码包（agx_arm_msgs / smart_grasp / smart_grasp_bringup / smart_grasp_interfaces / smart_grasp_moveit）。`build/ install/ log/` 为 colcon 生成目录，不在本次范围。

### A. 残留编译产物 / 备份（建议清理，已被 .gitignore 忽略但工作树仍存在）

| # | 文件 / 目录 | 说明 |
|---|---|---|
| A1 | 6 个 `__pycache__/` 共 25 个 `.pyc` | 分布：`smart_grasp_bringup/launch`、`smart_grasp/launch`、`smart_grasp/__pycache__`、`smart_grasp/scripts/__pycache__`、`smart_grasp/smart_grasp/__pycache__`、`smart_grasp/test/__pycache__`（含 pytest 缓存 `*-pytest-6.2.5.pyc`）。属构建产物，应 `find . -name __pycache__ -type d -exec rm -rf {} +` |
| A2 | `smart_grasp_bringup/config/*.bak_20260807`（3 个） | `grasp_test_box_106x76x30.yaml.bak_20260807`、`perception.yaml.bak_20260807`、`perception_test_box_106x76x30.yaml.bak_20260807`。调试快照，长期留存易混淆 |

### B. 未被任何 launch 引用的孤儿 / 歧义配置（疑似遗留）

| # | 文件 | 说明 |
|---|---|---|
| B1 | `smart_grasp_bringup/config/grasp_sim_yesterday_box.yaml` | 全工程无 launch/脚本引用；"yesterday box" 名称来历不明，内容是与 `grasp.yaml` 同构的 pick_server 参数（`simulation_mode: true`、`table_height: -0.0125`）。疑似某次仿真调试遗留，易与正式 `grasp.yaml` 混淆 |
| B2 | `smart_grasp_bringup/config/d405_camera_params.yaml` | 无引用；相机现已改用 `camera_serial_no` 启动参数绑定（见 08-06 条目），此文件（仅 `serial_no` 字符串）疑似被新方案取代的遗留物 |

### C. 重复 / 易混淆配置（同名不同义、同名节点多份）

| # | 文件 | 说明 |
|---|---|---|
| C1 | `smart_grasp_bringup/config/grasp.yaml` vs `smart_grasp/config/grasp_params.yaml` | 两个都叫 "grasp"、但分属不同包、职责不同：前者是 **pick_server** 参数（系统 launch 用），后者是 **detector** 参数（仅 `smart_grasp.launch.py` 用）。命名撞车，易误读 |
| C2 | `grasp_params.yaml` vs `perception.yaml`（都配 `smart_grasp_detector`） | 两份"权威" detector 配置并存且取值分歧：`grasp_params.yaml` 为 `hsv` 后端、`table_height: -999.0`、正方形 `fixed_object_size [0.06,0.06,0.04]`；`perception.yaml` 为 `yolo_seg`、`table_height: -0.2298`、矩形尺寸。`grasp_params.yaml` 明显偏陈旧，建议以 `perception.yaml` 为准并删/合并前者 |
| C3 | `grasp_test_box_106x76x30.yaml` / `perception_test_box_106x76x30.yaml` | 文件名写 106x76x30、实际 `60x40x40 mm`（已记入 #18）；README 自承"文件名保留历史尺寸"。调试时极易选错配置 |

### D. 未被引用的脚本（疑似草稿）

| # | 文件 | 说明 |
|---|---|---|
| D1 | `smart_grasp/scripts/grab_frame.py` | 无任何 launch/脚本引用；含未使用 `import sys`（见 #17）。疑似草稿工具 |
| D2 | `smart_grasp/scripts/aruco_diag.py` | 无引用；ArUco 标定诊断脚本，疑似遗留 |

### 结论与建议（2026-08-08 复核更新）
- **已清理 / 已修复（无需再处理）**：A2（`.bak_20260807` ×3）、B1（`grasp_sim_yesterday_box.yaml`）、B2（`d405_camera_params.yaml`）、C3（`106x76x30` 命名）均已不存在（删除/改名）；C1/C2 中 `grasp_params.yaml` 已改名为 `smart_grasp/config/detector_hsv.yaml`，消除“grasp”命名撞车，现为 HSV 后端 detector 配置，与 `perception.yaml`(yolo_seg) 并存属 backend 切换设计。
- **仍待清理（低风险）**：A1（`src` 下仍有 2 个 `__pycache__` / 17 个 `.pyc`）、D1/D2（`grab_frame.py`、`aruco_diag.py` 孤立脚本）。
- **说明**：上述已清理项说明本扫描之后工程已被整理过，本条目其余“待确认后处理”结论已不再适用。

---

## [2026-08-08] 启动报错两例：`smart_grasp_bringup` 未找到 / `home_joint_positions:=[]` 空列表崩溃

### 例 1：`Package 'smart_grasp_bringup' not found`
- **现象**：`ros2 launch smart_grasp_bringup smart_grasp_system.launch.py ...` 报包未找到，搜索路径只有 `agx_arm_ws / lidar_ws / base_node / /opt/ros/humble`，不含 `grasp_ws`。
- **根因**：当前 shell 只 source 了 `agx_arm_ws`，未 source `grasp_ws`。`smart_grasp_bringup` 安装在 `/home/guest/grasp_ws/install/`（已确认存在）。`smart_grasp_system.launch.py` 反调 `agx_arm_*` 的 launch，故两个工作区都需在环境里。
- **修复**：启动前叠加 source overlay：
  ```bash
  source /home/guest/grasp_ws/install/setup.bash
  ```
  依赖 `ultralytics` 等 Python 包时还需 `source /home/guest/grasp_ws/env.sh` 激活 venv（当前已在 `(.venv)` 下可略）。

### 例 2：`home_joint_positions:="[]"` → "Expected a non-empty sequence"
- **现象**：`ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py ... home_joint_positions:="[]"` 直接抛
  `Expected a non-empty sequence, with items of uniform type. Allowed sequence item types are bool, int, float, str.`
- **根因**：`ros2 launch` 把 `name:=value` 中"像字面量"的值**求值成真正的 Python 对象**，`[]` 被解析成**空列表**而非字符串 `"[]"`。`start_single_agx_arm.launch.py:134` 将该 `LaunchConfiguration` 原样塞进节点 `parameters`，而 `agx_arm_ctrl_single_node.py` 把 `home_joint_positions` 声明为 `dynamic_typing=True`；空列表无元素类型可推断 → launch_ros 拒收。
  对比：`DeclareLaunchArgument(..., default_value='[]')`（`start_single_agx_arm.launch.py:91`）的 `'[]'` 是**普通 Python 字符串**（不被求值），所以**不传该参数时**默认是字符串 `"[]"`，节点 `ast.literal_eval("[]")` → 全零 home，正常工作。仅命令行显式 `:=[]` 才会被当空 list 而崩。
- **修复**（任选其一）：
  - 直接删掉 `home_joint_positions` 参数，用默认全零 home；
  - 或传非空列表：`home_joint_positions:="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"`（float 类型可正常推断）。

### 结论 / 待办
- [ ] 现场启动抓取前，先 `source /home/guest/grasp_ws/install/setup.bash`（叠加在 `agx_arm_ws` 之上）。
- [ ] 启动 `agx_arm_ctrl` 时不要传 `home_joint_positions:="[]"`，省略即默认全零 home。
- [ ] 长期：可考虑把 `start_single_agx_arm.launch.py` 的 `home_joint_positions` 默认值由字符串 `'[]'` 改为显式 6 维 `'[0.0,0.0,0.0,0.0,0.0,0.0]'`，避免"默认字符串 vs CLI 空列表"行为不一致（属配置/launch 改动，按 PROJECT_RULES 规则三需审批）。

---

## [2026-08-08] 残余问题复核：此前文档中多处结论已被当前代码证伪 / 已修复

### 复核方法
对照当前 `grasp_ws/src` 实际代码与配置，逐条回验此前写入的静态审查（#1–#20）与文件扫描结论。

### 已被证伪 / 已修复（需更正此前条目）
- **#1 `grasp_executor_node` 死代码 / 坐标系 / 服务名冲突 —— 证伪**：实测 `grasp_executor_node.py` 订阅 `/smart_grasp/object_pose`（与 `detector_node` 发布者一致，能拿到目标），服务名为 `/smart_grasp/legacy_pick`（与 pick_server 的 action `/smart_grasp/pick` 不冲突）；且对 base_link 源的 object_pose 直接取坐标、不二次变换（`_target_in_base` L258-263 走 `frame==base_frame` 分支）。它是 README 所述“diagnostic/legacy 抓取路径”，默认不随 launch 启动，**不是死代码、坐标没错、无服务名冲突**。唯一成立点：未接入默认启动链路。
- **#2 配置打架“实机必定失败” —— 不成立**：实跑配置 `grasp_test_box_60x40x40.yaml` 已含 `table_height: -0.2298`（与 perception 一致）且 `workspace min_z: -0.20`（覆盖目标）。默认 `grasp.yaml` 的 `-999` 哨兵与 `min_z -0.10` 是**有意**的“未配置即阻断执行”安全设计，并非 bug；只有不传真实配置直接跑默认 `grasp.yaml` 才会被哨兵拦下。
- **#3 `trust_yolo: true` 关掉防线 —— 降级**：`perception.yaml` 默认 `trust_yolo: false`，稳定性门默认开启；仅当用户显式置 true 才失效。代码/注释不一致仍属真实小问题，但严重性从“整条关掉”降为“显式开启时才触发”。
- **#18 文件名 106x76x30 —— 已修复**：配置已重命名为 `grasp_test_box_60x40x40.yaml` / `perception_test_box_60x40x40.yaml`，与真实 60×40×40 mm 对齐。
- **文件扫描 A2/B1/B2/C3 —— 已清理**：`*.bak_20260807`、`grasp_sim_yesterday_box.yaml`、`d405_camera_params.yaml`、以及 `106x76x30` 命名均已不存在（删除/改名）。
- **文件扫描 C1/C2 —— 已缓解**：`grasp_params.yaml` 已改名为 `smart_grasp/config/detector_hsv.yaml`，消除“grasp”命名撞车；现为 HSV 后端的 detector 配置，与 `perception.yaml`(yolo_seg) 并存属 backend 切换的有意设计。
- **08-07 条目 perception_test_box `table_height -0.2268` 不一致 —— 已修复**：现 `perception_test_box_60x40x40.yaml` 为 `-0.2298`，与 perception 一致。

### 仍真实存在的残余（已核实）
- **A1**：`src` 下仍有 2 个 `__pycache__` / 17 个 `.pyc` 未清理（低风险）。
- **D1/D2**：`smart_grasp/scripts/grab_frame.py`、`aruco_diag.py` 仍为未被引用的孤立脚本（低风险）。
- **#20**：`smart_grasp/package.xml` 仍未声明 `ultralytics`（detector 默认 yolo_seg）。
- **#4–#17、#19**：前次静态审查的代码级逻辑问题，本次未逐条回验；鉴于 #1 曾基于错误假设，建议逐条对当前代码复核后再动手，暂按“待复核”处理。

### 结论
此前文档中 #1/#2/#3/#18 及文件扫描多数条目已**不适用**，请以后以本复核条目为准；其余待办（error 10 回 Home 清单、OMPL 容差持久化、跨视角 Y 偏差等）仍按各原条目推进。后续若有人依据旧条目判断“executor 是死代码可删除”，务必先看本复核——该节点工作正常。

---

## [2026-08-08] 残余问题汇总（整理索引）

> 把上面各条目收敛成一张总表。已解决/已证伪的不再列入；仅留**当前真实待处理**项。
> 详细根因见对应日期条目。

### A. 真机抓取链路（按优先级）

| 状态 | 问题 | 来源条目 | 处理建议 |
|---|---|---|---|
| 待办 | MOVE_TO_OBSERVE error 10 复发：臂停在 URDF 限位外 | 08-08 error10 两条 | 方案 B：`ompl_planning.yaml` `start_state_max_bounds_error` 0.1→0.3（配置改动，需审批）；保留"双门使能 + 回 Home 检查清单"（joint2∈[0,3.14]、joint3∈[-2.97,0]） |
| 待办 | 跨视角 Y 偏差 ~10mm 触发 STALE_TARGET 安全中止 | 08-05 复测 | 先定位根因（深度几何/时间戳 TF/手眼外参/目标跟踪），不擅自放宽 `reobserve_max_xy_shift=0.005` |
| 待办 | 同步 action 被自动控制门识别为真实执行 | 08-05 复测 | 修复同步流程隔离层，不能只靠初始跳变保护 |
| 待办 | VALIDATE_DEPTH 偶发 INVALID_DEPTH（深度有效率波动） | 08-07 INVALID_DEPTH | 先确认物体摆放/量程；可热更新放宽 perception 阈值（免重启）；统一两配置 `table_height`（已统一为 -0.2298，待复测确认） |
| 待办 | EXEC_PREGRASP 偶发 error 10（joint_state 超时） | 08-07 YOLO-Seg | 复测 YOLO-Seg 版 + `detection_rate` 节流是否彻底消除；抓日志确认分支 |
| 已解决 | 相机 `/dev/video0` 绑定漂移 | 08-06 A | 已加 `camera_serial_no:=260322272696` 按序列号绑定 |
| 已解决 | `ros2_control_node` 控制器加载失败（remap 误改参数） | 08-06 B | 已移除 `demo.launch.py` 的 `SetRemap("/robot_description")` |
| 已解决 | 真机抓取端到端成功（附着+50mm 抬升） | 08-02 收尾 | 已完成，仅剩进程清理段错误（非阻塞） |

### B. 配置 / 启动

| 状态 | 问题 | 来源条目 | 处理建议 |
|---|---|---|---|
| 已解决 | `smart_grasp_bringup` not found | 08-08 启动例1 | 启动前 `source /home/guest/grasp_ws/install/setup.bash`（叠加 agx_arm_ws） |
| 已解决 | `home_joint_positions:="[]"` 空列表崩溃 | 08-08 启动例2 | 省略该参数用默认全零 home，或传 6 维非空列表 |
| 已解决 | 默认 `grasp.yaml` 哨兵 -999 / min_z -0.10 | 08-08 静态#2 复核 | 实跑配置已覆盖，哨兵为有意设计，无需改 |
| 已解决 | `perception`/`grasp` 两配置 `table_height` 不一致 | 08-07 | 已统一为 -0.2298 |
| 待确认 | `start_single_agx_arm.launch.py` 的 `home_joint_positions` 默认 `'[]'` 字符串 vs CLI 空列表行为不一致 | 08-08 启动例2 | 长期可改为显式 6 维 `'[0.0,...,0.0]'`（配置改动，需审批） |

### C. 代码逻辑（静态审查，待逐条复核确认）

| 状态 | 问题 | 来源 | 说明 |
|---|---|---|---|
| 待复核 | #4 夹爪接触判定读陈旧数据（`gripper_condition_` 无人 wait） | 静态审查 | 需对当前 `pick_server.cpp` 回验 |
| 待复核 | #5 `waitForCurrentDetections()` 条件变量丢唤醒 | 静态审查 | 改为 `wait_until(lock, deadline, pred)` |
| 待复核 | #6 `detections_` map 只增不删 | 静态审查 | 加过期 erase |
| 待复核 | #7 `_invalid_detection` 消息头别名污染 | 静态审查 | 新建 `Header` |
| 待复核 | #8 `_set_gripper` 等待循环空操作 | 静态审查 | 实现真实反馈等待 |
| 待复核 | #9 `_busy` 检查非原子 | 静态审查 | 加锁/atomic |
| 待复核 | #10 物体顶面 Z 重建不自洽 | 静态审查 | 统一实测/配置高度 |
| 待复核 | #11 x/y 交换（仅非正方形暴露） | 静态审查 | 确认当前 test_box |
| 待复核 | #12 `minAreaRect` 退化 NaN | 静态审查 | 加退化保护 |
| 待复核 | #13 `makeGraspPose` 除零 NaN | 静态审查 | 加保护 |
| 待复核 | #14 `BUSY=17` 未用 / error_code=3 跳过 | 静态审查 | 小问题 |
| 待复核 | #15 `detach()` 线程析构 UAF | 静态审查 | join 或 smart pointer |
| 待复核 | #16 `param_tuner` 列不全 / AttributeError | 静态审查 | 小问题 |
| 已修复 | #18 文件名 106x76x30 与实际不符 | 静态审查 | 已改名 `*_60x40x40.yaml` |
| 待确认 | #19 多余 `tl_expected` 依赖 | 静态审查 | 移除 CMake 依赖 |
| 待确认 | #20 `package.xml` 未声明 `ultralytics` | 静态审查 | 补声明 |

### D. 残留文件（低风险清理）

| 状态 | 项 | 说明 |
|---|---|---|
| 待清理 | A1：`src` 下 2 个 `__pycache__` / 17 个 `.pyc` | 构建产物，`find -name __pycache__ -exec rm -rf {} +` |
| 待清理 | D1 `scripts/grab_frame.py`、D2 `scripts/aruco_diag.py` | 未被引用的孤立脚本 |
| 已清理 | A2 `.bak_20260807` ×3、B1 `grasp_sim_yesterday_box.yaml`、B2 `d405_camera_params.yaml`、C1/C2 `grasp_params.yaml`→`detector_hsv.yaml`、C3 `106x76x30` 命名 | 复核确认已不存在 |

### 总览勾选
- [ ] error 10：OMPL 容差持久化（方案 B，需审批）
- [ ] error 10：现场回 Home 检查清单固化进启动流程
- [ ] 跨视角 Y 偏差根因定位（不绕过 5mm 门）
- [ ] 同步 action 隔离层修复
- [ ] VALIDATE_DEPTH 阈值复测确认
- [ ] EXEC_PREGRASP error 10 复测确认
- [ ] 静态审查 #4–#17 逐条对当前代码复核并修复
- [ ] #19 移除多余依赖、#20 补 `ultralytics` 声明
- [ ] A1/D1/D2 低风险清理

---

## [2026-08-08] 二次全量复核（对照当前代码逐条回验，旧审查结论已几乎全部失效）

> 上一轮静态审查（#1–#20）是基于**陈旧快照**。本次对照当前 `grasp_ws/src` 实际代码 + 工程构建产物逐条回验，结论：**语法全清，逻辑问题几乎全部已修复，仅剩 3 项真实残留 + 2 项低风险残留**。

### 语法 / 构建基线（重新确认）
- `find . -name "*.py" -exec python3 -m py_compile`：全部通过，退出码 0。
- `pyflakes` 全量扫描：**0 告警**（含此前报未用 `import sys` 的 `grab_frame.py` 已无该导入）。
- 19 个单测此前通过；C++ `pick_server.cpp` 上次 `colcon build` 成功。

### 旧审查 #1–#20 回验结果（全部已修复/证伪）

| 原# | 结论 | 当前代码证据 |
|---|---|---|
| 1 | 证伪（executor 正常） | 订阅 `/smart_grasp/object_pose`、服务 `/smart_grasp/legacy_pick`，base_link 源直接取坐标（`grasp_executor_node.py:258-263`） |
| 2 | 证伪（哨兵有意） | 实跑 `grasp_test_box_60x40x40.yaml` 已 `table_height:-0.2298` + `min_z:-0.20`；默认 `-999` 为有意阻断 |
| 3 | 已修复（注释/实现一致） | `trust_yolo` 仅强制 stable；table 门仍由 `_process_instance`（`detector_node.py:426-428`）在 `table_z is None` 时写回 `TABLE_NOT_OBSERVED`，与注释“table remains guarded”一致 |
| 4 | 已修复 | `commandGripper` 先记 `gripper_feedback_sequence_`，再 `gripper_condition_.wait_for(seq>prev)`（`pick_server.cpp:769-779`）等闭合后新采样 |
| 5 | 已修复 | `waitForCurrentDetections` 用 `detection_condition_.wait_until(lock, deadline, ready)` 带谓词（`pick_server.cpp:369`） |
| 6 | 已修复 | `detectionCallback` 与 `currentDetectionsLocked` 均调 `pruneDetectionsLocked`（`pick_server.cpp:190,320`），按 age 擦除 |
| 7 | 已修复 | `header_in_frame` 新建独立 `Header`（`detector_node.py:39-41,256-260`），不再别名污染传入消息 |
| 8 | 已修复 | `_set_gripper` 用条件变量真正等待新反馈（`grasp_executor_node.py:362-380`） |
| 9 | 已修复 | Python `with self._lock` 加锁 check-set（`grasp_executor_node.py:384-389`）；C++ `busy_.exchange(true)` 原子（`pick_server.cpp:232`） |
| 10 | 已修复 | `estimate_oriented_box` 以实测 top_z 锚定、用配置高反推；`_publish_primary`/`makeGraspPose` 用同一配置高反推，自洽 |
| 11 | 复验一致（非 bug） | `msg.size.x=box.size[1]`(短维) 对齐 `short_axis`(pose X)，`msg.size.y=box.size[0]`(长维) 对齐 long_axis；对当前 `[0.060,0.040,0.040]` 自洽（仅耦合“配置短维恒在 y”的约定，换非常规尺寸需复核） |
| 12 | 已修复 | `estimate_oriented_box` 退化时 `raise ValueError`（`depth_geometry.py:108-118`），上层捕获转 `INVALID_DEPTH`，不再产生 NaN |
| 13 | 仍残留（见下，低风险） | `makeGraspPose` `x_axis.setZ(0); x_axis.normalize()`（`pick_server.cpp:385-386`）在物体 X 轴恰竖直时归一化为 NaN；被 detector 恒输出水平 X 所掩护 |
| 14 | 证伪（设计如此） | `BUSY=17` 已用（`pick_server.cpp:233`）；`PickObject.action` 注释明示 `error_code=3 reserved for wire compatibility` |
| 15 | 已修复 | `execution_thread_` 为成员、启动前 `join()`（`pick_server.cpp:236-240`），非 detach |
| 16 | 已修复 | `set_one` 先判 `res is None` 再访问 `res.results`（`param_tuner.py:180-183`）；无参分支列不全仅为辅助提示，非崩溃 |
| 17 | 已修复 | `grab_frame.py` 已无 `import sys`；`pyflakes` 全量 0 告警 |
| 18 | 已修复 | 已改名 `*_60x40x40.yaml` |
| 19 | 仍残留（见下，低风险） | `CMakeLists.txt:22,35` `find_package(tl_expected REQUIRED)` 且链接，但 `pick_server.cpp` 全文 0 处 `tl::expected` 用法 |
| 20 | 仍残留（见下，中风险） | `smart_grasp/package.xml` 未声明 `ultralytics`；而 detector 默认 `yolo_seg` 运行时依赖它 |

### 真实残留清单（本次新确认）

| 状态 | # | 位置 | 问题 | 严重度 |
|---|---|---|---|---|
| 待修 | 13 | `pick_server.cpp:385-386` | `makeGraspPose` 物体 X 轴近竖直时 `setZ(0)`→零向量→`normalize()` 得 NaN，缺零范数保护（当前被 detector 恒水平 X 掩护） | 低 |
| 待修 | 19 | `smart_grasp_moveit/CMakeLists.txt:22,35` | 多余硬依赖 `tl_expected`（无源码引用），可移除 `find_package` 与链接项 | 低 |
| 待修 | 20 | `smart_grasp/package.xml` | 未声明 `ultralytics`；`yolo_seg` 后端运行时缺失将失败，`rosdep install` 不会拉取 | 中（可复现性） |
| 待清 | A1 | `src` 树 | 6 个 `__pycache__` / 32 个 `.pyc` 仍残留（gitignored 构建产物） | 低 |
| 待清 | D1/D2 | `smart_grasp/scripts/` | `grab_frame.py`、`aruco_diag.py` 仍无 launch/脚本引用 | 低 |

### 结论
- **代码逻辑层面：此前 20 条里 17 条已修复/证伪，无高危逻辑缺陷残留**，仅 #13 一处潜在数值健壮性缺口（实践中被上游掩护）。
- **配置/依赖层面**：#19、#20 是真实但低/中风险的小清理项。
- **残留文件**：A1/D1/D2 为低风险清理项。
- 原“残余问题汇总”条目中 C 表把 #4–#17 标“待复核”已过时——本次回验确认均 FIXED，请以本条目为准。

---

## [2026-08-08] 文件/配置扫描（二次）：新增「配置配对逻辑不自洽」+ 真实残留清单

> 在上一轮文件扫描（仅关注 .bak/孤儿/命名撞车）基础上，本次**额外核对了 launch 实际引用的配置内容一致性**，发现一处此前未捕获的逻辑不自洽。本条目取代/修正上一轮扫描结论。

### 0. 语法 / 引用健全性
- 实际被 launch 引用的配置均已存在（无断链）：`perception.yaml`、`grasp_test_box_60x40x40.yaml`、`handeye_20260725.yaml`（system）；`detector_hsv.yaml`（standalone）；`perception.yaml`（camera_only）。
- `py_compile` / `pyflakes` 全量通过（见上一轮二次全量复核条目）。

### A. 孤儿 / 死配置（未被任何 launch 引用，grep 实测确认）

| # | 文件 | 说明 |
|---|---|---|
| A1 | 6 个 `__pycache__` / 32 个 `.pyc` | 构建产物，仍残留（gitignored） |
| A2 | `smart_grasp_bringup/config/grasp.yaml`（bare） | **未被任何 launch 引用**（system 用的是 `grasp_test_box_60x40x40.yaml`）。遗留的“默认” pick_server 配置，带哨兵值 `table_height:-999.0` + `min_z:-0.10`，与实跑配置同名不同义，易误认为是“正式”配置 |
| A3 | `smart_grasp_bringup/config/perception_test_box_60x40x40.yaml` | **未被任何 launch 引用**（system 用的是 `perception.yaml`）。它本应是 `grasp_test_box_60x40x40.yaml` 的 detector 配对文件，却未接线 → 见 B1 |
| A4 | `smart_grasp_bringup/models/model_metadata.yaml` | **全工程无任何引用**（代码/config 均不读），疑似遗留 |

### B. 逻辑不自洽（本扫描新发现，重点关注）

#### B1. 系统 launch 的「detector / pick_server」物体尺寸配对矛盾【中高危】
- system launch 实际接线：
  - detector ← `perception.yaml` → `fixed_object_size: [0.060, 0.060, 0.040]`（**正方形**）、`yolo_seg`
  - pick_server ← `grasp_test_box_60x40x40.yaml` → `fixed_object_size: [0.060, 0.040, 0.040]`（**矩形**）、文件名暗示 60×40×40
- 后果：detector 按自己配置把 `DetectedObject.size` 报成正方形 0.06×0.06×0.04，而 pick_server 用消息里的 `size` 建碰撞盒/抓取几何（`pick_server.cpp:399,723`）。于是**实际抓取 footprint 是 0.06×0.06 而非 0.04 宽**，比“test_box 60×40×40”意图多出 2cm；OBB 短/长轴赋值也与该物体不匹配。
- 根因：`perception_test_box_60x40x40.yaml`（矩形 detector 配对）被遗漏未接线，导致“test_box”场景只接了一半。
- 修复选项（任选其一，需确认真实物体形状）：
  1. 系统 launch 在 test_box 场景下把 detector 配置改为 `perception_test_box_60x40x40.yaml`（矩形，与 pick_server 配对）；
  2. 若默认物体本就是正方形 60×60×40，则把 `grasp_test_box_60x40x40.yaml` 改名/改为正方形尺寸，消除“test_box”误导；
  3. 明确 square 为默认、rectangular 为可选，并在 README 写清。

### C. 歧义 / 脆弱命名 与 硬编码路径

| # | 位置 | 问题 |
|---|---|---|
| C1 | `smart_grasp_bringup/config/handeye_20260725.yaml` | 日期戳标定文件被 system launch 直接默认引用；一旦重新标定产生新文件，旧日期名就过时且易误用。建议去掉日期或改为 `calibration_file` 启动参数 |
| C2 | `smart_grasp/smart_grasp/grasp_executor_node.py:72-73` | `handeye_json` 参数默认值硬编码绝对路径 `/home/guest/handeye_ws/result/eye_in_hand_d405_px_connected_20260725.json`——跨工作区（`handeye_ws`）耦合，且仅 legacy executor 用。应改为相对/参数化 |
| C3 | `smart_grasp_bringup/config/perception.yaml:12` | `yolo_model: /home/guest/best.pt` 绝对路径硬编码；`best.pt` 迁移即失效。建议改为包内相对路径或启动参数 |
| C4 | `smart_grasp/config/detector_hsv.yaml:21` | `table_height: -999.0` 哨兵，用于 standalone HSV launch（`smart_grasp.launch.py`）；该 launch 下 table 门恒关、检测不稳定，与 `perception.yaml`(-0.2298) 语义不一致。若 HSV 调试确有意为之，请在 README 注明 |

### D. 上一轮已确认清理项（本次复验仍无）
- `.bak_20260807` ×3、`grasp_sim_yesterday_box.yaml`、`d405_camera_params.yaml`、命名 `106x76x30` → 复验确认均不存在。
- 原 C1/C2（`grasp_params.yaml`→`detector_hsv.yaml`）改名已生效，当前仅剩 `detector_hsv.yaml`（HSV 后端）与 `perception.yaml`（yolo_seg）并存的**有意** backend 切换设计，但二者 `table_height` 语义不同（见 C4）。

### 结论与建议处理顺序
1. **[优先]** 解决 B1：确认真实物体形状，补齐 `perception_test_box_60x40x40.yaml` 接线或调整尺寸，消除 detector/pick_server 尺寸矛盾。
2. **[清理]** 删除/归档 A2/A3/A4 孤儿配置（先确认无人手引）。
3. **[去脆弱]** C1/C2/C3 硬编码路径参数化；C4 注明 HSV 哨兵意图。
4. **[低优先]** A1 pyc 清理（`find . -name __pycache__ -exec rm -rf {} +`）。

---

## [2026-08-09] 真机成功抓取参数快照

本条记录 2026-08-09 现场已成功完成一次 `MODE=execute` 真机抓取时的可复现基线。后续调参先以本条为准。

### 成功启动命令

```bash
# 终端 1：机械臂驱动
source ~/grasp_ws/env.sh
bash ~/grasp_ws/scripts/start_arm_driver.sh

# 终端 2：相机驱动
source ~/grasp_ws/env.sh
bash ~/grasp_ws/scripts/start_camera_driver.sh

# 终端 3：MoveIt、手眼 TF、感知和抓取服务
source ~/grasp_ws/env.sh
bash ~/grasp_ws/scripts/start_grasp_system.sh

# 终端 4：使能、回 home、执行抓取
source ~/grasp_ws/env.sh
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /move_home std_srvs/srv/Empty "{}"
MODE=execute bash ~/grasp_ws/scripts/pick_object.sh
```

### 进程 / launch 参数

```yaml
arm_driver:
  command: bash ~/grasp_ws/scripts/start_arm_driver.sh
  can_port: can0
  arm_type: piper_x
  effector_type: agx_gripper
  speed_percent: 10
  auto_enable: false
  control_enabled: false
  allow_remote_disable: false
  gripper_default_effort: 0.5
  home_joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

camera_driver:
  command: bash ~/grasp_ws/scripts/start_camera_driver.sh
  camera_serial_no: "260322272696"
  camera_namespace: camera
  camera_name: camera
  rgb_camera.color_profile: 848x480x30
  depth_module.depth_profile: 848x480x30
  align_depth.enable: true
  enable_color: true
  enable_depth: true
  enable_infra: false
  pointcloud.enable: false

smart_grasp_system:
  command: bash ~/grasp_ws/scripts/start_grasp_system.sh
  use_driver: false
  use_camera: false
  execute: true
  calibration_validated: true
  detector_backend: yolo_seg
  yolo_model: /home/guest/best.pt
  yolo_class: "1"
  yolo_confidence: 0.7
  grasp_config: /home/guest/grasp_ws/src/smart_grasp_bringup/config/grasp_test_box_60x40x40.yaml
  perception_config: /home/guest/grasp_ws/src/smart_grasp_bringup/config/perception_test_box_60x40x40.yaml
  use_rviz: false
```

### `/smart_grasp_pick_server` 参数

```yaml
execution_allowed: true
calibration_validated: true
simulation_mode: false
planning_group: arm
base_frame: base_link
end_effector_link: tcp_link
planner_id: RRTConnectkConfigDefault
planning_time: 5.0
planning_attempts: 5
velocity_scaling: 0.02
acceleration_scaling: 0.02
pregrasp_distance: 0.120
grasp_depth: 0.020
lift_height: 0.050
tcp_to_grasp_xyz: [0.0, 0.0, 0.1425]
cartesian_step: 0.005
cartesian_min_fraction: 1.0
time_parameterize_cartesian: false
cartesian_velocity_scaling: 0.02
cartesian_acceleration_scaling: 0.02
validate_all_candidate_approaches: true
pregrasp_reobserve_mode: validate_only
allow_reobserve_fallback: true
reobserve_max_xy_shift: 0.04
reobserve_max_z_shift: 0.02
reobserve_max_axis_yaw_deg: 15.0
target_max_age: 0.5
detection_wait_timeout: 1.0
stable_detection_wait_timeout: 3.0
execution_joint_tolerance: 0.03
execution_settle_timeout: 60.0
observation_joint_positions: [-1.560708324, 1.875757707, -1.251889766, 0.776078105, 0.0, -0.005742133]
observation_joint_tolerance: 0.02
post_pick_joint_positions: [-1.583554684, 0.186139365, -0.379190233, 0.550424486, -0.055798176, 0.0]
post_pick_joint_tolerance: 0.05
post_pick_final_joint_positions: [-0.034924038, 0.366536596, -0.541017162, 1.152074386, 0.019024089, 0.0]
post_pick_final_joint_tolerance: 0.05
gripper_open: 0.090
gripper_close: 0.035
gripper_force: 0.5
gripper_motion_time: 1.0
gripper_timeout: 3.0
contact_width_min: 0.036
contact_width_max: 0.066
table_height: -0.2298
table_size: [0.30, 0.30, 0.05]
table_center_xy: [0.103, -0.464]
target_table_clearance: 0.001
static_obstacles: ""
workspace: [-0.20, -0.70, -0.20, 0.90, 0.70, 1.20]
arm_action: /arm_controller/follow_joint_trajectory
gripper_action: /gripper_controller/follow_joint_trajectory
joint_state_topic: /feedback/joint_states
target_class: "1"
```

### `/smart_grasp_detector` 参数

```yaml
detector_backend: yolo_seg
yolo_model: /home/guest/best.pt
yolo_class: "1"
yolo_confidence: 0.7
detection_rate: 8.0
fixed_object_size: [0.06, 0.04, 0.04]
color_topic: /camera/camera/color/image_rect_raw
depth_topic: /camera/camera/aligned_depth_to_color/image_raw
info_topic: /camera/camera/color/camera_info
camera_frame: camera_color_optical_frame
base_frame: base_link
depth_scale: 0.001
min_depth: 0.08
max_depth: 0.60
min_depth_points: 500
min_depth_valid_ratio: 0.6
mask_erode_pixels: 3
point_outlier_radius: 0.08
position_outlier_radius: 0.02
orientation_surface_band: 0.02
stability_frames: 10
max_position_span: 0.015
max_yaw_span_deg: 20.0
yaw_outlier_radius_deg: 30.0
table_height: -0.2298
table_ransac_threshold: 0.004
tf_lookup_timeout: 0.25
trust_yolo: false
hsv_lower: [95, 110, 70]
hsv_upper: [130, 255, 255]
min_contour_area: 800.0
min_rectangularity: 0.65
min_solidity: 0.85
```

### 控制器 / 门控状态

```text
arm_controller: active
gripper_controller: active
joint_state_broadcaster: active
arm_action: /arm_controller/follow_joint_trajectory
gripper_action: /gripper_controller/follow_joint_trajectory
joint_state_topic: /feedback/joint_states
```

`gripper_controller` 当前只支持 `position` command interface。`pick_server` 给夹爪 action 只发送 `positions: [width]`，不能在 `FollowJointTrajectory` 点里填 `effort`，否则控制器报 `Trajectories with effort fields are currently not supported` 并拒绝 goal。夹爪力由 `start_arm_driver.sh` 传给驱动的 `gripper_default_effort: 0.5` 提供。

2026-08-09 已更新 `pick_server` 成功路径：抓取、抬升并到达配置的 post-pick sequence
后，会自动发送 `gripper_open` 夹爪轨迹，在最终收尾点位开爪放下物块。该释放动作
只在 `execute:true` 且 post-pick sequence 成功到位后执行；plan-only 不发夹爪命令。

### 启动链路修复记录

- `agx_arm_moveit/launch/demo.launch.py` 需要显式声明 `feedback_topic` / `control_topic`，否则 `arm_controller` spawner 退出后触发 `agx_arm_state_sync` 时 launch 会报 `launch configuration 'control_topic' does not exist`。
- `arm_controller` spawner 成功退出后再启动 `agx_arm_state_sync`；同步成功后再启动 `agx_arm_control_gate`。短启动验证曾看到 `GenericSystem synchronized to live feedback; max_error=0.0000 rad` 和 `You can start planning now!`。
- 启动前确认只有一个 `/agx_arm_ctrl_single_node`，重复机械臂驱动会同时发布 `/feedback/joint_states` 并订阅 `/control/joint_states`，导致行为混乱。

## [2026-08-09] 抓取后到最终点位自动开爪释放快照

本条记录在 `[2026-08-09] 真机成功抓取参数快照` 的可复现基线上，追加“放置点位开夹爪放物块”动作。

### 行为快照

- 成功抓取并完成 `lift_height: 0.050` 抬升后，仍先通过 MoveIt 回到
  `post_pick_joint_positions` 中间点。
- 如果配置了 `post_pick_final_joint_positions`，继续从中间点移动到最终放置点位：
  `[-0.034924038, 0.366536596, -0.541017162, 1.152074386, 0.019024089, 0.0]`。
- 只有在 post-pick sequence 成功到位后，`pick_server` 才发送
  `gripper_open: 0.090` 到 `/gripper_controller/follow_joint_trajectory`，
  反馈阶段名为 `OPEN_GRIPPER_AT_PLACE`。
- 该释放动作只在 `execute:true` 成功路径执行；`execute:false` 只规划，不发夹爪命令。
- 若最终到位后开爪 action 失败，action 结果为 `GRIPPER_FAULT`，不会报告成功。

### 代码 / 文档变更

- `src/smart_grasp_moveit/src/pick_server.cpp`：成功路径中 `moveToPostPickSequence(...)`
  后追加 `commandGripper(gripper_open)`，并把成功结果文案更新为
  `pick completed and released at configured post-pick sequence`。
- `README.md`：真机流程说明已补充“成功到达最终收尾点位后自动打开夹爪放下物块”。

### 验证

```bash
source /opt/ros/humble/setup.bash
source /home/guest/agx_arm_ws/install/setup.bash
cd /home/guest/grasp_ws
colcon build --symlink-install --packages-select smart_grasp_moveit
```

结果：`smart_grasp_moveit` 构建通过，耗时约 `2min 2s`。

### 下次现场复现命令

```bash
# 终端 1：机械臂驱动
source ~/grasp_ws/env.sh
bash ~/grasp_ws/scripts/start_arm_driver.sh

# 终端 2：相机驱动
source ~/grasp_ws/env.sh
bash ~/grasp_ws/scripts/start_camera_driver.sh

# 终端 3：MoveIt、手眼 TF、感知和抓取服务
source ~/grasp_ws/env.sh
bash ~/grasp_ws/scripts/start_grasp_system.sh

# 终端 4：使能、回 home、执行抓取
source ~/grasp_ws/env.sh
ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"
ros2 service call /move_home std_srvs/srv/Empty "{}"
MODE=execute bash ~/grasp_ws/scripts/pick_object.sh
```

预期成功链路：观察位识别 -> 预抓取 -> 笛卡尔接近 -> 闭爪接触验证 ->
附着 -> 50 mm 抬升 -> post-pick 中间点 -> final 放置点 -> 自动开爪释放。

<!-- 新问题时，在此行上方追加，格式参考上面 -->

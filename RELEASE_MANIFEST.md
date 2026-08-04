# Smart Grasp 发布清单

## 版本身份

| 项目 | 值 |
|---|---|
| 发布版本 | `0.2.0` |
| 发布日期 | `2026-07-26` |
| 抓取工作区 | `/home/mdz/grasp_ws` |
| 抓取实现基线提交 | `e3204f5405253afebfeb025dacd6e3063226f87a` |
| 抓取发布标签 | `smart-grasp-v0.2.0` |
| 官方源码本地克隆 | `/home/mdz/agx_arm_ws/src/agx_arm_ros` |
| AgileX 上游基点 | `91e6b2e5eb2d9880e85230d0add9945e27387d87` |
| 本地 MoveIt 集成提交 | `af427ea373b92194bccd60c2ad0edfcec5eb1011` |
| 本地集成标签 | `smart-grasp-integration-v0.2.0` |
| 本地提交身份 | `Codex <codex@local>` |

`smart-grasp-v0.2.0` 指向包含本清单的发布提交；可使用
`git -C ~/grasp_ws rev-parse smart-grasp-v0.2.0^{commit}` 获取它的完整提交号。

## 远程仓库状态

- 没有执行 `git push`。
- `grasp_ws` 当前没有配置远程仓库，仅建立本地 Git 历史。
- `agx_arm_ros` 的 `origin` 仍指向 AgileX 官方仓库；本地 `ros2` 分支比
  `origin/ros2` 领先 1 个提交，该提交只保存在本机。
- 除非项目所有者以后明确指定自己的远程仓库，否则不得推送这两个标签或
  本地集成提交到 AgileX 官方仓库。

## 本版本包含的内容

抓取工作区基线提交包含：

- `smart_grasp_interfaces`：目标检测消息和抓取动作接口。
- `smart_grasp`：HSV/YOLO-Seg 后端、RGB-D 三维几何、稳定性、手眼 TF 和
  验证工具。
- `smart_grasp_moveit`：MoveIt 抓取动作服务器与安全状态机。
- `smart_grasp_bringup`：统一启动、目标/场景/外参配置、模型占位和 RViz 文档。
- 根目录 README、版本文件、变更日志、回归测试和忽略规则。

官方源码本地集成提交只包含以下 4 个文件：

```text
src/agx_arm_ctrl/launch/start_single_agx_arm_moveit.launch.py
src/agx_arm_moveit/config/ompl_planning.yaml
src/agx_arm_moveit/launch/_moveit_config_builder.py
src/agx_arm_moveit/launch/demo.launch.py
```

它增加显式 OMPL/RRTConnect 配置、RViz 参数透传，以及机械臂和夹爪两个
FollowJointTrajectory 控制器的门控监听。

## 明确排除的本地状态

官方源码克隆中下列变化在本任务开始前已经存在，没有纳入集成提交，也没有
被回退：

```text
src/agx_arm_moveit/scripts/agx_arm_control_gate
mode 100644 -> 100755
```

因此检出集成标签后，工作树是否仍显示该权限变化取决于本机文件系统状态；
它不属于 `af427ea373b92194bccd60c2ad0edfcec5eb1011`。

以下运行时或现场数据不进入 Git：

- `build/`、`install/`、`log/` 和 Python/pytest 缓存。
- ROS bag、D405 录制数据和现场日志。
- `.pt`、`.onnx`、TensorRT engine 等模型权重。
- 尚未实测填写的桌面尺寸和高度。

## 关键文件校验值

以下 SHA-256 对应抓取实现基线提交：

```text
d14d1ca84d4aa3771c657bba66df252fbb0cf546ab452f10ee24e5577b5840d0  src/smart_grasp_interfaces/msg/DetectedObject.msg
6242235cff80af377d1332d35bcf7ba7bab9938af504c7c288dfba66d4b2b1a7  src/smart_grasp_interfaces/action/PickObject.action
76883bbfd083ec40663f1f9a0df9cb19dcc1c81593d292426231622e334c0cb4  src/smart_grasp_bringup/config/perception.yaml
80bb4feb14f3e3c031901c8cb4cd2d8283c31a969a2541aeae1e2f0ebaa25d12  src/smart_grasp_bringup/config/grasp.yaml
42849b212e603ed29434e6fd69671e0a518e94bd9e9735c3eccccea665fe3bb5  src/smart_grasp_bringup/config/handeye_20260725.yaml
583e182727e16dd184d0125d21a966f3d765110e5fcb12be5802dca4332fc6c5  src/smart_grasp_bringup/config/target_blue_block.yaml
```

## 验证记录

- 四个抓取包和下层 MoveIt 修改已完成构建。
- 纯算法回归测试：`9 passed`。
- 无 CAN 的 MoveIt 冒烟测试成功，动作服务器、KDL 和 RRTConnect 可加载。
- 未进行现场 D405 精度验收、5-8 姿态外参验收或真机抓取验收。
- `handeye_20260725.yaml` 保持 `validated: false`。
- 桌面碰撞参数保持无效哨兵值，真机执行门默认关闭。

## 未发布修复记录（2026-07-27）

本次“只做颜色识别、删除视觉尺寸匹配”实现提交：
`9d32c83f53fdaf43f2fe2292801ca5e14444bb91`。

- 纯相机模式默认分辨率由 `424x240x30` 提升到 `640x480x30`，并增加
  `color_profile` 启动参数；虚拟机USB不稳定时可临时降级而无需修改源码。

- 经项目所有者明确批准，已删除视觉物理尺寸计算、尺寸匹配、尺寸稳定窗口和
  `SIZE_MISMATCH` 结果；HSV 是目标身份判断，深度仅提供中心和水平朝向。
- `fixed_object_size` 只向 PlanningScene 和抓取几何提供固定配置，不参与检测
  是否通过。该设计取代本次未发布阶段中曾实现的尺寸门控方案，可通过本次
  提交的父提交回看或回档旧实现。
- 测试盒使用 15 帧位姿窗口。精确图像时间 TF 等待参数化，VMware 测试配置
  使用 250 ms，仍禁止回退到非图像时间戳 TF。
- 新增实测 `106.5 x 76.5 x 30 mm` 测试盒的独立感知和夹爪配置，并将目标
  点云离群半径参数化为该配置下的 80 mm；默认 60 x 40 x 40 mm 配置不变。
- 测试盒夹爪参数固定为张开 90 mm、闭合目标 0 mm、力 0.5、有效接触宽度
  68.5-84.5 mm；桌面、标定和执行安全门保持关闭。
- 修复 `param_tuner` 重复初始化、参数枚举、数组类型赋值、不存在参数识别和
  临时节点清理问题。
- 校正包内 README 的后端、TF 校验服务、MoveIt 状态机、RViz 话题、动态参数
  生效范围和无机械臂感知说明，并补充 HSV 像素轮廓与固定抓取几何的边界。
- 新增 `test_param_tuner.py` 回归覆盖；
  `smart_grasp` 包构建成功，临时 ROS 2 参数节点上的 `list`、浮点、整数数组和
  不存在参数路径均通过端到端验证。
- 删除尺寸逻辑后的直接算法回归为 `13 passed`，四个抓取包重新构建成功。
- 2026-07-28现场静止测试显示位置波动已达标，但水平角波动为16-33度；方向
  PCA改为只使用顶部8 mm点云带，并增加180度对称角圆周过滤和最多20%的离群
  帧剔除。新增3项回归后直接算法测试为 `16 passed`。实现提交为
  `554c326f74f5c55a6a7ab72ae3ef013eda72786c`。
- 现场分段TF检查确认 `base_link -> tcp_link` 和相机内部TF正常，但构造阶段的
  `tcp_link -> camera_link` 静态样本在VMware DDS发现期间丢失。手眼节点现于
  启动窗口每秒有限重发，增加安装矩阵回归后直接测试为 `17 passed`。实现
  提交为 `2dfedfaa76cfab7cecb29ee1c50f153cf7642c67`。
- 现场完整TF恢复后，检测深度有效率约0.95且位置仅毫米级波动，但固定哨兵
  协方差显示测试盒位姿窗口未形成足够角度内点。仅将测试盒调试配置放宽为
  8帧、30度角度内点半径和20度单姿态角度极差；位置门仍为15 mm，最终
  5-8姿态外参验收仍为20 mm / 3度。实现提交为
  `773170b85ee795eee894e6d867f42dadebefb847`。
- 本记录仍属于 `Unreleased`，随本次修复提交保存但尚未建立发布标签；没有修改
  或推送官方源码仓库。

## 未发布真机验收记录（2026-08-02）

- `agx_arm_ctrl` 新增默认兼容的动态参数 `allow_remote_disable`；普通驱动默认
  为 true，智能夹取 launch 显式设为 false。保护开启时远程失能请求被拒绝，
  `control_enable=false` 仍只关闭外部轨迹命令，不取消伺服使能。
- 远程失能保护单元测试 `3 passed`；`agx_arm_ctrl`、`agx_arm_moveit` 和
  `smart_grasp_bringup` 定向构建通过。
- 实机动作从 Home 开始，依次完成观察位、稳定检测、双候选 RRTConnect、
  预抓取、笛卡尔下降、夹紧、接触验证、目标附着和 50 mm 抬升。
- 动作结果为 `success=true`、`error_code=0`，接触宽度 `0.0766 m`；动作后
  `arm_status=0`、`err_status=0`、`motion_status=0`，夹爪反馈宽度约
  `0.0764 m`，机械臂和夹爪保持使能且所有夹爪故障字段为 false。
- 现场人员托稳物体后，夹爪打开至 `0.090 m`（控制器成功，实际约 `0.0897 m`），
  临时解锁 `allow_remote_disable` 并调用失能服务，驱动返回
  `success=true`、`Agx_arm disabled`；随后停止完整 launch，相关节点均已退出。
- 停止阶段记录到 MoveIt 退出段错误及驱动超时强制结束，均发生在机械臂失能之后，
  归类为待修复的 ROS 进程清理问题，不影响本次真机验收。
- 尚未完成 10 次至少 9 次成功的重复性验收，本记录仍属于 `Unreleased`。

## 查看与无破坏回档

```bash
git -C ~/grasp_ws show smart-grasp-v0.2.0 --stat
git -C ~/agx_arm_ws/src/agx_arm_ros \
  show smart-grasp-integration-v0.2.0 --stat
```

推荐从标签建立新分支，不覆盖当前工作：

```bash
git -C ~/grasp_ws switch -c rollback/smart-grasp-v0.2.0 \
  smart-grasp-v0.2.0
git -C ~/agx_arm_ws/src/agx_arm_ros switch \
  -c rollback/smart-grasp-integration-v0.2.0 \
  smart-grasp-integration-v0.2.0
```

若要把两个仓库一起恢复到本版本，必须同时检出两个标签。只恢复
`grasp_ws` 而不恢复对应的 MoveIt 集成，可能导致规划管线或控制门行为不一致。

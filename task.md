# Task List — 已知但未解决问题

> 本文件记录项目推进过程中**已发现、但当前无法在代码层面直接解决**的问题。
> 这些问题大多依赖现场实测、硬件语义确认或实机验证，需要人工介入。
> 最后更新：2026-07-31

---

## 阻塞项（真实抓取前必须解决）

### [B1] 桌面实测值仍为占位 `-999`，桌子碰撞体未加载
- **位置**：`smart_grasp_bringup/config/grasp.yaml`
- `table_height: -999.0`、`table_size: [0,0,0]`、`table_center_xy: [0,0]`
- 触发 `TABLE_UNCONFIGURED`，`pick_server` 拒绝执行。
- **当前测试值**：`grasp_test_box_60x40x40.yaml` 按现场高度假设更新为 `-0.15 / [1.2,1.2,0.10] / [0.20,0.0]`；仍需用 `base_link` 原点和桌面实测复核。
- **待办**：现场测量后回填 `grasp.yaml`，并同步 `perception.yaml` 的 `table_height`。

### [B2] 手眼标定 `validated: false` 未置位
- **位置**：`smart_grasp_bringup/config/handeye_20260725.yaml`
- 外参已填实，但 `validated` 仍为 `false`。
- **门控链路**：`reconcile_calibration` 只向下压制（yaml=false 时强制 CLI 覆写为 false），仍需 yaml=true **且** launch 传 `calibration_validated:=true` 双 true。
- **待办**：按 `/smart_grasp/validation/record`（5–8 姿态、散布 <20mm / 3°）验证通过后，yaml 置 `true` + launch 传 flag。

### [B3] 需实机验证手眼标定精度
- 标定散布是否达标（位置 <20mm、姿态 <3°）只能在真实硬件上确认，当前无法在代码中判定。

---

## 工程风险（不影响"能否实现"，影响"是否稳妥"）

### [R1] `gripperHealthy()` 故障位极性未确认
- **位置**：`smart_grasp_moveit/src/pick_server.cpp`
- `sensor_status` / `driver_error_status` 按 "true=故障" 处理，需对照 AgileX 驱动实际语义确认，否则健康检查可能永久失败或失效。

### [R2] 长耗时阶段缺取消检查 + attach 失败无回滚
- **位置**：`pick_server.cpp` 笛卡尔逼近 / 夹取 / 抬升阶段
- 没有 `canceled()` 检查；attach 后若抬升失败不会清理 ACM / 碰撞体。
- 建议动真机前补上。

### [R3] 代码层可改进项（非阻塞）
- `pick_server.cpp`：`gripper_base/link1/link2`/EE 硬编码（当前 `agx_gripper` 下无碍，切机型失效）；`gripper_force` 参数声明未用；`MOVE_TO_OBSERVE` 空操作。
- 接口 `PickObject.action` 错误码缺 `3`（历史空洞），`BUSY`/`INTERNAL_ERROR` 从未赋值。
- 两包零 C++/lint 测试。

---

## 设计 / 现场待确认项（本次对话提出，尚未定论）

### [Q1] 船体摇晃导致相机无法稳定锁定抓取目标
- **现象**：机械臂 + tron2 组合后，船体持续摇晃保持平衡，感知端 `PoseStabilityWindow`（base_link 下 10 帧、max_pos_span=0.015m、max_yaw_span=5°）永远判 `stable=False`，物体相对 base_link 坐标被"假运动"污染。
- **根因**：相机与目标同在船上，`base_link` 随船晃，物体相对 base_link 坐标随之变。
- **候选方案**（未实施）：
  - **A（推荐）**：把稳定性判据从 `base_link` 改到 `camera_color_optical_frame`/`tcp_link`（相机不动时物体在此系近静止，抵消船摇），判稳后再一次性转 base_link。改动集中在 `detector_node.py` + `stability.py` 使用处。
  - **B**：底座 IMU/里程对 `base_link` 做动态补偿（需可靠姿态源，频率/延迟达标）。
  - **C**：抓取前让 tron2 静默/锁定再拍抓（牺牲动态抓取）。
- **待办**：用户确认"相机与目标是否同船"后定方案。已确认二者相对 `base_link` 静止。

### [Q2] 雷达/主控机静态障碍物坐标需实测替换
- **位置**：`grasp.yaml` 的 `static_obstacles` / `workspace`（本次已加，但为**保守占位**）
- 当前占位：`radar_mast` 在 `y=-0.45,z=0.45`；`main_controller` 在 `x=0.35,z=0.15`（假设，非实测）。
- **待办**：现场测量雷达/主控机相对 `base_link` 的真实中心 xyz 与包围盒尺寸，替换占位值；`workspace` 边界亦按需调整。
- **已实现机制**：`applyStaticObstacles()` 解析 CSV 多行 → BOX 碰撞体 `ADD` 进规划场景；`setWorkspace()` 硬性规划边界。雷达/主控机与机械臂相对 `base_link` 静止（已确认），静态盒建模正确。

---

## 说明
- **已完成项不在此列**：Python 18 单测全过、C++ 编译通过、模块导入正常、README 已同步 `target_blue_block.yaml` 合并 / `reconcile_calibration` 描述 / 静态障碍物章节。
- 本文件仅跟踪"发现但暂无法解决"的问题，解决后请移至对应 changelog 或删除条目。

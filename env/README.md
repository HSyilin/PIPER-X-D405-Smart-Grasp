# Smart Grasp 离线环境

此目录按 `agx_arm_ws/env` 的结构保存智能抓取工作区依赖，可在无 PyPI 网络
的环境中重新创建虚拟环境并编译 ROS 2 工作区。

## 兼容范围

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- x86_64
- 同级 `agx_arm_ws`，或由 `AGX_ARM_WS` 指定的 AGX 工作区

目标电脑需预装 `system-apt-packages.txt` 中的 apt 包。NumPy、SciPy 与 OpenCV
使用 Ubuntu/ROS 的系统包，其中 NumPy 保持在 1.x，以兼容 Humble 的 `cv_bridge`。
`wheels/` 预留给后续非 ROS Python 依赖。

## 重新编译

先编译 AGX underlay，再编译本工作区：

```bash
cd /新的父目录/agx_arm_ws
./env/build.sh

cd /新的父目录/grasp_ws
./env/check-system.sh
./env/build.sh
source env.sh
```

如果 AGX 不在同级目录：

```bash
export AGX_ARM_WS=/实际路径/agx_arm_ws
./env/build.sh
```

两边的构建输出都按当前路径哈希写入 `.portable`。因此整体搬迁后，只需依次
重新运行两个 `build.sh`，不会误用旧路径中的 CMake、Python shebang 或 colcon
缓存。

默认 HSV 检测流程已包含完整依赖。YOLO-Seg 接口完整保留，包括
`detector_backend:=yolo_seg`、`yolo_model` 启动参数和 `YoloSegBackend` 实现。
仓库没有训练模型，且 Ultralytics/PyTorch 与 CPU、CUDA 平台强相关，因此核心
离线环境不内置模型和推理依赖；接入时提供模型及目标机器匹配的推理环境即可。

## 文件说明

- `python-requirements.lock`: 固定的 Python 依赖版本
- `wheels/`: Python 3.10 / x86_64 离线 wheel
- `SHA256SUMS`: wheel 完整性校验值
- `system-apt-packages.txt`: ROS、RealSense 与系统依赖
- `check-system.sh`: 检查目标系统及 AGX underlay
- `build.sh`: 创建虚拟环境并重新编译
- `activate.sh`: 依次加载 AGX 与当前工作区

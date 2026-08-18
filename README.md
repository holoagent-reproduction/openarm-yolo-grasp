# OpenArm 双相机 YOLOE 语义抓取系统

这是一个基于 OpenArm 双臂机械臂、RealSense 双相机、YOLOE 开放词汇实例分割和 MoveIt 2 的视觉语义抓取项目。

项目当前针对左臂和固定桌面场景，支持通过中文指令识别水杯等物品，生成抓取规划，经人工确认后执行真实抓取。

## 一、仓库总体架构

```text
openarm/
├── windows/                         Windows 与 WSL2 侧
│   ├── 一键启动脚本/                 一键启动、远程SSH、端口转发
│   ├── windows_scripts/              原始启动脚本兼容目录
│   ├── yoloe_inference_service/     YOLOE HTTP 推理服务
│   └── 标定板/                       AprilTag 标定板文件
│
├── linux/                           Linux NUC / ROS 2 侧
│   ├── openarm_vision_grasp/        视觉抓取核心包
│   ├── openarm_semantic_grasp_interfaces/  ROS 2 Action与消息
│   ├── openarm_calibration_jog/      手眼标定和关节微调
│   ├── ros2_joint_gui/               关节显示与手动控制界面
│   ├── nuc/                          NUC源码、MoveIt、标定和运行备份
│   ├── semantic-pick-skill/          Skill说明和辅助脚本
│   └── 新建文件夹/                   项目文档与验收资料
│
└── README.md                         本项目总说明
```

## 二、运行环境分工

### Windows / WSL2

Windows 端负责：

- 启动 WSL2 中的 YOLOE 推理服务；
- 配置 Windows 到 WSL2 的 8765 端口转发；
- 通过 SSH 连接 Linux NUC；
- 启动和清理 NUC 上的 ROS 2 程序；
- 保存项目代码和备份。

YOLOE 服务只返回检测结果，不直接控制机械臂。

### Linux NUC

Linux 端负责：

- OpenArm 硬件和 CAN 控制；
- MoveIt 2 运动规划与轨迹执行；
- D435i 头部相机和 D415 左腕相机；
- YOLOE 结果接收和三维定位；
- 语义抓取状态机；
- 夹爪控制和抓取结果验证；
- RViz、相机画面和语义确认界面显示。

## 三、核心数据流

```text
中文指令
  ↓
语义命令节点
  ↓
SemanticPick Action
  ↓
YOLOE HTTP 推理服务
  ↓
类别、置信度、边框、掩码
  ↓
D435i 彩色图 + 对齐深度 + CameraInfo
  ↓
三维点云和目标位姿
  ↓
TF 转换到 world 坐标系
  ↓
抓取候选与 MoveIt 规划
  ↓
人工确认
  ↓
左臂执行轨迹
  ↓
夹爪张开、接近、闭合、抬升
  ↓
夹持证据验证
```

## 四、Linux ROS 2 软件包

### `openarm_vision_grasp`

当前视觉抓取核心包，主要节点包括：

- `semantic_command_node`：将中文指令解析为标准类别并发送抓取任务；
- `yoloe_client_node`：将相机图像发送到 YOLOE 服务；
- `semantic_3d_node`：掩码深度融合、点云过滤、三维目标定位；
- `object_tracker_node`：维护目标实例和跟踪状态；
- `semantic_grasp_task_node`：抓取规划、MoveIt执行、夹爪控制和状态机；
- `semantic_confirm_ui`：Linux 主机上的识别、规划、确认界面；
- `semantic_skill_bridge`：对外提供 HoloAgent 风格的 Skill 接口；
- `tag_detector`：AprilTag 检测和标定辅助；
- `wrist_depth_range_node`：左腕相机距离测量辅助。

主要配置：

- `config/cameras.yaml`：相机序列号、话题、外参和坐标系；
- `config/semantic_grasp.yaml`：抓取策略、姿态、补偿、速度、夹爪参数；
- `config/tag_objects.yaml`：AprilTag 物品和收纳区配置；
- `config/drop_zone.yaml`：放置区域配置；
- `launch/semantic_grasp.launch.py`：语义抓取后端启动文件；
- `launch/dual_realsense_grasp.launch.py`：双相机启动文件。

### `openarm_semantic_grasp_interfaces`

定义 ROS 2 接口：

- `SemanticPick.action`：语义抓取目标、反馈和结果；
- `SemanticDetection.msg`：YOLOE 检测结果；
- `SemanticObject3D.msg`：三维目标位姿、尺寸和稳定性。

### `openarm_calibration_jog`

用于真实机械臂手眼标定：

- 关节微调节点和界面；
- 同步采样机器人 TF 与 AprilTag 位姿；
- 标定数据清洗和异常样本剔除；
- 手眼外参求解和误差评估。

### OpenArm 与 MoveIt 相关代码

主要位于 `linux/nuc/ros2_ws/src/`：

- `openarm_description`：URDF、网格模型、末端和 ros2_control 描述；
- `openarm_bimanual_moveit_config`：双臂 MoveIt 配置、规划组和 RViz；
- `openarm_bringup`：硬件和控制器启动；
- `openarm_hardware`：真实硬件接口；
- `openarm_can`：CAN 总线控制；
- `realsense_camera_launch`：D435i/D415 相机启动；
- `ros2_joint_gui`：关节状态和手动调节界面。

## 五、当前抓取状态机

```text
IDLE
  ↓
DETECTING
  ↓
LOCALIZED
  ↓
PLANNING
  ↓
WAIT_CONFIRM
  ↓ 人工确认
OPEN
  ↓
APPROACH
  ↓
CLOSE
  ↓
LIFT
  ↓
VERIFYING
  ↓
SUCCEEDED / FAILED / CANCELLED
```

当前主要流程：

1. 接收“拿起桌子上的水杯”等中文指令；
2. YOLOE 识别目标类别和掩码；
3. D435i 深度生成目标三维位置；
4. 根据当前 TCP 姿态和斜侧方向生成抓取位姿；
5. MoveIt 规划到预抓取位置和最终抓取位置；
6. 界面等待人工确认；
7. 先张开夹爪，再执行最终接近轨迹；
8. 闭合夹爪并垂直抬升约 10 cm；
9. 根据夹爪间隙、视觉目标状态和目标高度变化判断是否夹持成功。

## 六、当前抓取策略

当前默认策略为：

```yaml
grasp_strategy: slanted_side
```

含义是从桌面内的对角方向接近目标，当前接近轴为：

```yaml
slanted_side_approach_axis_xyz: [0.7071, -0.7071, 0.0]
```

该方向保持水平，不主动向桌面下压。末端姿态使用规划时读取的实际 TCP 姿态，避免使用未经标定的猜测四元数。

## 七、夹爪控制和证据

当前水杯配置为全闭合目标：

```yaml
gripper_close_m: 0.000
```

抓取验证主要使用：

- 夹爪实际位置是否保留夹持间隙；
- 抬升后目标是否消失或视觉数据是否过期；
- 目标三维高度是否明显上升。

满足至少两项才报告抓取成功。夹爪电机温度、急停和现场安全优先级高于软件判断。

## 八、手眼标定

标定使用 AprilTag 36h11：

- 物品标签 ID：`0–9`；
- 收纳区标签 ID：`100`；
- D435i：眼在手外标定，负责全局三维定位；
- D415：左腕相机手眼标定，负责近距离辅助测距和复核。

标定数据和复测结果位于：

```text
linux/nuc/ros2_ws/calibration_samples/
```

## 九、Windows 一键启动

推荐从以下目录启动：

```text
windows/一键启动脚本/
```

- `start_openarm_preview.cmd`：预览模式，只规划不执行；
- `start_openarm_real_grasp.cmd`：真实抓取模式；
- `README_使用说明.md`：详细启动说明和故障排查。

MoveIt、RViz、相机画面和语义界面显示在 Linux NUC 屏幕，不显示在 Windows 桌面。

## 十、当前已实现功能

- 双 RealSense 相机独立启动；
- AprilTag 手眼标定和 TF 外参管理；
- YOLOE 开放词汇实例分割；
- 中文目标语义解析；
- 目标掩码与深度融合；
- world 坐标系三维目标定位；
- 左臂 MoveIt 预抓取规划；
- 斜侧水平接近策略；
- 夹爪张开、接近、闭合和抬升；
- 人工确认和任务取消；
- 任务状态反馈和错误码；
- 抓取证据验证；
- Windows 一键远程启动；
- Linux 主机本地显示 MoveIt 和相机画面。

## 十一、当前边界

- 目前主要验证左臂；
- 物品需要是 YOLOE 可识别的常见桌面物体；
- 不保证透明杯、玻璃、软袋和强反光物体；
- 不支持任意六自由度通用抓取；
- 暂未完成自动放置到收纳区的完整流程；
- YOLOE 和视觉结果不是安全信号；
- 真实动作必须人工确认，急停和现场安全措施优先。

## 十二、从零复现当前项目

下面流程用于在一台已经安装 Ubuntu 22.04、ROS 2 Humble 和 OpenArm 驱动的 NUC 上复现当前系统。Windows 电脑只作为代码工作区、YOLOE 推理主机和远程启动端。

### 1. 检查 Windows 工作区

确认以下目录存在：

```text
windows/一键启动脚本/
windows/yoloe_inference_service/
linux/openarm_vision_grasp/
linux/openarm_semantic_grasp_interfaces/
linux/openarm_calibration_jog/
```

进入一键启动目录：

```powershell
cd "E:\Users\lenovo\Desktop\openarm\windows\一键启动脚本"
```

该命令切换到启动脚本所在目录，保证脚本能够找到同目录下的 Python 和 PowerShell 文件。

### 2. 检查 NUC 网络和 SSH

在 PowerShell 中测试 NUC：

```powershell
Test-Connection 172.16.13.202 -Count 1
ssh nuc@172.16.13.202
```

第二条命令通过 SSH 登录 NUC。当前用户名是 `nuc`，密码由使用者输入，不写入代码仓库。

登录 NUC 后检查 ROS 2：

```bash
source /opt/ros/humble/setup.bash
source /home/nuc/ros2_ws/install/setup.bash
ros2 topic list
```

这些命令加载 ROS 2 和工作区环境，并确认 ROS 2 图正常工作。

### 3. 确认 NUC 机械臂和相机

确认机械臂已上电、CAN 已连接、急停可用。检查相机设备：

```bash
lsusb | grep -i realsense
```

该命令确认 D435i 和 D415 已被 Linux 识别。不要在机械臂未上电或急停状态不明确时启动真实执行。

### 4. 启动预览模式

回到 Windows，在以下目录双击：

```text
windows\一键启动脚本\start_openarm_preview.cmd
```

预览脚本会依次完成：

1. 在 WSL2 启动 YOLOE 服务；
2. 检查 `8765` 健康接口；
3. 配置 Windows 到 WSL2 的端口转发；
4. 通过 SSH 清理旧的语义节点；
5. 在 NUC 启动 MoveIt、双相机和语义抓取后端；
6. 在 NUC 屏幕显示 RViz、相机画面和语义界面。

预览模式不会执行机械臂真实运动，只用于验证识别和规划。

### 5. 检查启动结果

在 NUC 上执行：

```bash
source /opt/ros/humble/setup.bash
source /home/nuc/ros2_ws/install/setup.bash
ros2 node list | grep -E 'move_group|semantic|yoloe|realsense'
```

该命令检查 MoveIt、语义节点、YOLOE 客户端和相机节点是否存在。

检查 YOLOE 服务：

```bash
curl --max-time 3 http://172.16.13.1:8765/health
```

返回 `{"ok":true}` 表示 NUC 可以访问 Windows/WSL2 中的 YOLOE 服务。

检查实时图像：

```bash
ros2 topic hz /openarm_vision/head_d435i/color/image_raw
ros2 topic hz /openarm_vision/head_yoloe_overlay
```

第一条检查头部原始图像，第二条检查 YOLOE 叠加图频率。

### 6. 执行预览规划

在 Linux NUC 屏幕的语义抓取界面中：

1. 输入“拿起桌子上的水杯”；
2. 保持“仅规划预览”勾选；
3. 点击“开始识别与规划”；
4. 检查 YOLOE 掩码、三维坐标、夹爪方向和 MoveIt 轨迹；
5. 确认界面显示“规划完成，等待人工确认”。

也可以在 NUC 终端检查状态：

```bash
ros2 topic echo --once --full-length /openarm_vision/semantic_status
```

该命令显示当前语义抓取状态和错误码。

### 7. 标定验证

检查头部相机到世界坐标的 TF：

```bash
timeout 5 ros2 run tf2_ros tf2_echo world head_d435i_color_optical_frame
```

检查左腕相机到末端的 TF：

```bash
timeout 5 ros2 run tf2_ros tf2_echo openarm_left_hand_tcp left_wrist_d415_color_optical_frame
```

如果提示坐标树不连通，不要执行真实抓取，应先检查 `cameras.yaml` 和 MoveIt 的 TF 树。

### 8. 真实抓取

预览规划确认无误后，关闭预览界面和旧启动进程，再双击：

```text
windows\一键启动脚本\start_openarm_real_grasp.cmd
```

该脚本以 `allow_motion=true` 启动真实执行。真实任务仍需在 Linux 语义界面中人工确认。执行前必须确认：

- 机械臂工作区无人；
- 夹爪没有卡住或过热；
- 杯子位置没有变化；
- RViz 中规划轨迹不穿过桌面；
- 急停按钮可立即使用。

### 9. 真实抓取后的状态检查

检查关节状态：

```bash
timeout 5 ros2 topic hz /joint_states
```

检查夹爪关节反馈：

```bash
ros2 topic echo --once /joint_states
```

检查最终语义结果：

```bash
ros2 topic echo --once --full-length /openarm_vision/semantic_status
```

如果结果为 `succeeded`，表示状态机完成了抬升和夹持证据验证；如果为 `failed`，优先查看 `error_code` 和 NUC 上的 `semantic_backend.log`。

### 10. 重新部署 Linux 源码

Windows 端源码位于 `linux/`，NUC 端实际运行源码位于 `/home/nuc/ros2_ws/src/`。修改 Linux 侧代码后，应重新编译：

```bash
cd /home/nuc/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openarm_vision_grasp openarm_semantic_grasp_interfaces --symlink-install
source /home/nuc/ros2_ws/install/setup.bash
```

这些命令只重新编译视觉抓取和接口包，不会修改原始 OpenArm 或 MoveIt 配置。

### 11. 故障恢复顺序

如果任务卡住或出现重复节点：

1. 在语义界面点击“取消任务”；
2. 关闭当前启动窗口；
3. 确认机械臂停止并检查温度；
4. 重新运行预览脚本；
5. 先完成预览规划，再恢复真实抓取。

不要在任务状态未知时重复点击真实执行，也不要同时运行多个一键启动脚本。

### 12. YOLOE 环境和模型准备

YOLOE 不在 NUC 上推理，而是在 Windows 的 WSL2 Ubuntu-22.04 中运行。完整说明位于：

```text
windows/yoloe_inference_service/README.md
windows/yoloe_inference_service/models/README_模型权重.md
```

最低环境要求：

- Windows 11 + WSL2 Ubuntu-22.04；
- 支持 WSL2 的 NVIDIA 驱动；
- WSL2 中 `nvidia-smi` 可见 GPU；
- Python 3.10；
- PyTorch 2.2+、Ultralytics 8.3+；
- 建议 GPU 显存至少 8 GB。

创建 YOLOE 环境并安装依赖：

```bash
python3 -m venv ~/venvs/openarm-yoloe
source ~/venvs/openarm-yoloe/bin/activate
python -m pip install --upgrade pip
pip install -r windows/yoloe_inference_service/requirements.txt
```

这些命令创建独立虚拟环境并安装服务依赖。模型权重 `yoloe-11s-seg.pt` 需要单独放在 WSL2 的：

```text
/home/lenovo/openarm_yoloe_service/models/yoloe-11s-seg.pt
```

模型权重没有提交到 GitHub，原因是文件较大且许可证独立于本项目。准备权重后，一键启动脚本会检查模型文件是否存在；缺失时会停止并提示先准备模型。

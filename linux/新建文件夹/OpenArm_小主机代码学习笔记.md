# OpenArm 小主机代码学习笔记

> 本笔记基于 2026-08-07 对 `172.16.20.130` 的只读 SSH 检查。没有启动 ROS、没有发送 CAN 指令、没有修改远端代码。

## 1. 先建立全局地图

这台机器并不是只有一个 OpenArm 程序，而是几条独立链路共存：

```text
A. OpenArm ROS 2 真机链路
ROS 指令 -> ros2_control -> openarm_hardware -> SocketCAN/CAN-FD -> 达妙电机

B. OpenArm MuJoCo 仿真链路
ROS 指令 -> ros2_control -> openarm_mujoco_hardware -> WebSocket/JSON -> MuJoCo

C. OpenArm VR 遥操链路
WebXR 手柄 -> 逆运动学（IK）-> ROS 控制器 -> A 的硬件插件

D. 天机/Wuji 遥操作链路（不同设备）
VR/手套 TF -> IK -> TianjiChestDriver -> 天机双臂

E. 移动底盘链路
键盘或 VR 摇杆 -> base_control.py -> SocketCAN -> 安捷轮底盘
```

注意：C 可以与 A 配合；D 不是 A 的替代实现；E 控制的是轮式底盘而不是 OpenArm 关节。

## 2. 真实 OpenArm 的源码位置

| 目录 | 作用 | 初学时的阅读优先级 |
|---|---|---:|
| `/home/nuc/ros2_ws/src/openarm_ros2/openarm_hardware` | ROS 2 到真实电机的适配层 | 最高 |
| `/home/nuc/ros2_ws/src/openarm_ros2/openarm_bringup` | 单臂/双臂启动与控制器参数 | 最高 |
| `/home/nuc/ros2_ws/src/openarm_can` | 独立的底层 CAN 库和诊断工具 | 高 |
| `/home/nuc/ros2_ws/src/openarm_description` | URDF 模型、关节限位、坐标和 ros2_control 声明 | 高 |
| `/home/nuc/ros2_ws/src/openarm_mujoco_hardware` | MuJoCo 仿真插件 | 中 |
| `/home/nuc/telepo_openarm` | 现用的 VR 双臂遥操和底盘遥控 | 高 |
| `/home/nuc/.local/openarm_vr_teleop` | 较早的 VR 双臂遥操副本 | 中 |
| `/home/nuc/ros2_ws/src/ros2_joint_gui` | 自定义双臂 GUI | 中 |

`build/`、`install/`、`log/` 是编译输出和日志；`.cache/`、`.cursor/` 是缓存或编辑器索引，均不是应当学习的主源码。

## 3. ROS 2 真机控制链路

### 3.1 谁负责什么

1. `openarm_bringup/launch/openarm.bimanual.launch.py` 创建 `robot_state_publisher` 和 `ros2_control_node`。
2. `openarm_description` 展开 Xacro，得到机器人模型和每个关节的命令/状态接口。
3. `controller_manager` 按 YAML 加载轨迹控制器、位置控制器、夹爪控制器。
4. `openarm_hardware::OpenArmHW` 作为插件提供 `read()` 和 `write()`。
5. `MotorControl` 把控制量编码成 CAN-FD 帧；`CANBus` 用 Linux SocketCAN 将帧发到电机。

控制频率来自 `openarm_v10_controllers.yaml`：`update_rate: 100`，即每 10 毫秒运行一轮。

### 3.2 `OpenArmHW` 的核心数据

文件：`/home/nuc/ros2_ws/src/openarm_ros2/openarm_hardware/include/openarm_hardware/openarm_hardware.hpp`

- 7 个机械臂自由度，加上 1 个夹爪自由度。
- CAN 从站 ID 是 `0x01` 到 `0x07`；夹爪使用 `0x08`。
- `pos_states_`、`vel_states_`、`tau_states_` 保存“刚从电机读到的实际值”。
- `pos_commands_`、`vel_commands_`、`tau_ff_commands_` 保存“ROS 控制器希望发送的目标”。

真实硬件代码的刚度和阻尼是硬编码数组：

```text
Kp = [80, 80, 20, 55, 5, 5, 5, 0.5]
Kd = [2.75, 2.5, 0.7, 0.4, 0.7, 0.6, 0.5, 0.1]
```

可把每个关节的控制理解为：

```text
输出 = Kp ×（目标位置 - 实际位置）
     + Kd ×（目标速度 - 实际速度）
     + 前馈力矩
```

Kp 越大，关节越努力回到目标位置；Kd 越大，运动越不容易振荡。二者都不应盲目增大。

### 3.3 真实硬件的安全动作

文件：`openarm_hardware/src/openarm_hardware.cpp`

- 激活前先读电机位置，然后把位置目标设为当前位置。这避免了命令缓冲区默认值 `0` 使机械臂突然回零。
- 激活过程中用小步靠近目标，单步最大差值约为 `π/2` 弧度。
- 正常写入时，若某关节目标和实际位置相差超过 `π/2`，函数返回错误而不发该轮控制命令。
- 停用时先发一个全零 MIT 命令刷新状态，再发送电机禁用命令。

这些措施降低风险，但不等于完整安全系统：仍需要急停、真实限位、碰撞检查与现场监护。

### 3.4 CAN-FD 报文的含义

文件：`openarm_hardware/src/motor_control.cpp`

代码使用 MIT 控制格式，把以下 5 个量压缩到 8 字节：

```text
q（位置，16 位） | dq（速度，12 位） | Kp（12 位） | Kd（12 位） | τ（力矩，12 位）
```

`double_to_uint()` 先把实际物理量按允许范围夹紧，再映射到整数；`uint_to_double()` 则在反馈方向还原为弧度、弧度每秒和牛米。`0xFC` 是使能、`0xFD` 是禁用、`0xFE` 是设零位命令。

## 4. 模型与限制

真实关节限位在：

`/home/nuc/ros2_ws/src/openarm_description/config/arm/v10/joint_limits.yaml`

例如：J1 约 `-80° 到 200°`，J4 约 `-17° 到 140°`，并非每个关节都是 ±170°。

自定义 GUI 文件 `ros2_joint_gui/openarm_joint_gui/gui_node.py` 当前把所有关节统一限制为 ±170°。这只是保守的通用范围，和真实模型并不一致；在连接真实机器前必须改为上述 YAML 的逐关节范围。

## 5. 双臂配置中的本地修改

`openarm_description` 有本地未提交修改，位于：

`urdf/ros2_control/openarm.bimanual.ros2_control.xacro`

修改将：

```text
插件：OpenArm_v10HW -> OpenArmHW
参数：can_interface -> can_device
参数：arm_prefix -> prefix
```

这与实际插件登记文件 `openarm_hardware.xml` 和 `OpenArmHW::on_init()` 相匹配，因此是正确方向。不要用上游版本直接覆盖这个文件。

## 6. MuJoCo 为什么能复用控制器

文件：`/home/nuc/ros2_ws/src/openarm_mujoco_hardware/src/openarm_mujoco_hardware.cpp`

MuJoCo 插件对 ROS 2 暴露与真机相同的“位置、速度、力矩状态”和“位置、速度、力矩命令”接口。区别只在最后一步：

- 真机插件将命令编码为 CAN-FD。
- MuJoCo 插件计算力矩 `Kp×位置误差 + Kd×速度误差 + 前馈力矩`，以 JSON `{ "cmd": {关节名: 力矩} }` 通过 WebSocket 发给仿真。
- 仿真反馈 JSON 中的 `qpos`、`qvel`、`qtau` 会回写到 ROS 状态。

默认 WebSocket 端口是 `1337`。该插件目前把最大力矩限幅代码注释掉了，所以即便在仿真里也应谨慎调整 Kp/Kd。

## 7. VR 遥操作的实际算法

主版本：`/home/nuc/telepo_openarm/main.py`

流程是：

```text
VR 控制器 gripPose
  -> squeeze 作为“离合器/死手开关”
  -> 记录接管瞬间的手柄姿态和机械臂末端姿态
  -> 使用后续的相对位移构造末端目标
  -> Pyroki 求逆运动学（IK）
  -> 仅发布仍按住 squeeze 的那一侧关节数组
  -> forward_position_controller
```

“只发布接管侧”是很重要的设计：松开左手不会把左臂算出的新值错误地写到右臂；未接管侧保留真实 `/joint_states` 反馈。

夹爪使用 `GripperCommand` Action；左、右手扳机分别控制左、右夹爪。VR 摇杆还可控制移动底盘，而不是机械臂关节。

## 8. 不能混淆的天机/Wuji 链路

`wujihandros2/.../tianji_arm_node.py` 也发布 `/left_arm/joint_commands` 和 `/right_arm/joint_commands`，但它调用的是 `TianjiChestDriver`，默认目标为 `192.168.1.190` 的天机设备。

因此 `openarm_joint_gui` 目前的 GUI 话题会进入天机遥操作链路，而不会直接进入 OpenArm 的 `openarm_hardware` CAN 控制器。启动 GUI 前必须先确认你想控制的究竟是“天机臂”还是“OpenArm”。

## 9. 已识别的总线冲突风险

`telepo_openarm/base_control.py` 把移动底盘固定为 `can0`；单臂 OpenArm 启动文件的默认 `can_interface` 也是 `can0`。如果这两个程序在同一 SocketCAN 接口上同时运行，会把底盘协议和关节电机协议混在同一总线中，风险很高。

在任何真机启动前，先确认每个物理设备对应的 CAN 接口；不明确时不要运行控制程序。

## 10. 推荐学习顺序

1. 读本笔记第 3 节，理解 ROS 控制器、硬件插件和 CAN 的职责边界。
2. 读 `openarm_hardware.cpp` 的 `on_activate()`、`read()`、`write()`。
3. 读 `motor_control.cpp` 的 `controlMIT()` 和 `processPacketFD()`。
4. 再读 `openarm.bimanual.launch.py` 与双臂控制器 YAML，理解“程序怎样被接起来”。
5. 在不连接真机的前提下阅读 MuJoCo 插件。
6. 最后阅读 VR 遥操的 `DualClutchIKController.step()` 与 `IkPublishWorker._tick()`。

后续将继续补充：`openarm_can` 独立库、MoveIt 配置、Xacro 模型装配层、底盘脚本以及每一份本地副本之间的关系。

## 11. `openarm_can`：独立的底层 CAN 库

目录：`/home/nuc/ros2_ws/src/openarm_can`

这是一套 C++17 库，不是普通 ROS 节点。它把“打开 SocketCAN、管理电机、编码命令、解析反馈”封装成可复用的对象。当前的 `openarm_hardware` 并未在 CMake 中链接它，而是复制并实现了相似的 SocketCAN 和 MIT 报文逻辑。因此两者不能同时拥有同一个 CAN 口。

对象关系如下：

```text
OpenArm
 ├─ CANSocket                 打开/绑定 Linux SocketCAN
 ├─ ArmComponent              保存多个关节 Motor
 ├─ GripperComponent          保存一个夹爪 Motor
 └─ CANDeviceCollection       按接收 CAN ID 分发反馈帧
       └─ DMCANDevice         把帧解析后更新对应 Motor
```

### 11.1 收发流程

1. `OpenArm("can0", true)` 创建 CAN-FD Socket。
2. `init_arm_motors()` 为每个关节记录“发送 ID、接收 ID、型号和控制模式”。
3. `mit_control_all()` 调用协议编码器，把每个 `MITParam{kp,kd,q,dq,tau}` 变成 8 字节。
4. `CANSocket` 用 Linux `write()` 写入 `canfd_frame`。
5. `recv_all()` 用 `select()` 等待第一帧（默认最多 500 微秒），连续读取其余已到达帧。
6. `CANDeviceCollection` 根据帧 ID 找到 `DMCANDevice`；后者按当前回调模式将反馈写入 `Motor` 状态或参数缓存。

### 11.2 支持的电机模式

| 模式 | API | 实际含义 |
|---|---|---|
| MIT | `mit_control_*` | 同时给 Kp、Kd、位置、速度、力矩；OpenArm ROS 真机使用此模式 |
| POS_VEL | `posvel_control_*` | 发送 32 位浮点位置和速度，CAN ID 加 `0x100` |
| POS_FORCE | `posforce_control_*` | 发送位置、速度上限和电流/力矩上限，CAN ID 加 `0x300` |

库会在调用前检查电机当前模式；例如电机不在 MIT 模式时会拒绝 `mit_control`。

### 11.3 帧格式与反馈

MIT 控制帧固定 8 字节，与 ROS 真机插件一致：位置 16 位、速度/Kp/Kd/力矩各 12 位。转换前会按电机型号的最大位置、速度、力矩范围夹紧，避免把超范围数值直接编码进帧。

反馈帧则解析为：位置（rad）、速度（rad/s）、力矩（Nm）、MOS 温度、转子温度。参数查询和写入统一发到 CAN ID `0x7FF`：读取命令使用 `0x33`，写入命令使用 `0x55`。

### 11.4 诊断工具的安全说明

- `openarm-can-diagnosis` 会查询 7 个关节和夹爪的 CAN ID、主 ID、波特率；属于硬件通信操作，不是纯离线检查。
- `openarm-can-motor-check` 会使能指定的真实电机，并以 10 Hz 刷新状态后关闭。
- `examples/demo.cpp` 会使能真实电机，发送位置和力矩命令。

因此不能在不清楚接线、供电、急停和 CAN 接口归属时运行这些工具。阅读源码或用 MuJoCo 学习不需要执行它们。

## 12. Xacro、双臂模型和 MoveIt

### 12.1 Xacro 是“生成机器人 XML 的模板语言”

入口文件是：`openarm_description/urdf/robot/v10.urdf.xacro`。它加载 YAML 中的关节限位、惯量、坐标、Kp/Kd 等参数，然后调用 `openarm_robot.xacro`。

当 `bimanual=true` 时，模型依次装配：

1. OpenArm 机身；
2. 左臂（固定前缀 `left_`）；
3. 右臂（固定前缀 `right_`）；
4. 两个 ros2_control 硬件系统；
5. 左右末端手和夹爪。

因此最终关节名会是 `openarm_left_joint1` 到 `openarm_left_joint7` 与 `openarm_right_joint1` 到 `openarm_right_joint7`。

### 12.2 双臂 ROS 控制器

`openarm_v10_bimanual_controllers.yaml` 定义两套独立控制器：

- 左/右 `forward_position_controller`：高频透传一组位置数组，适合 VR 遥操；
- 左/右 `joint_trajectory_controller`：接收带时间的轨迹，适合 MoveIt 规划；
- 左/右 `gripper_controller`：接收夹爪 Action。

MoveIt 配置中的 `moveit_controllers.yaml` 将规划出的 `FollowJointTrajectory` 交给左右轨迹控制器；它不会直接访问 CAN。

### 12.3 已验证的仿真启动问题

`openarm_bimanual_moveit_config/launch/teleop.launch.py` 中的 `mode=sim` 会把 `hardware_type=mujoco` 传给 Xacro，但当前 `v10.urdf.xacro` 和其调用的宏没有使用 `hardware_type`，也没有选择 `openarm_mujoco_hardware/MujocoHardware` 的分支。

已在主机上只展开 Xacro（未启动控制器、未访问 CAN）验证：即使传入 `hardware_type:=mujoco`，生成结果仍是两个：

```xml
<plugin>openarm_hardware/OpenArmHW</plugin>
<param name="can_device">can1</param>
<plugin>openarm_hardware/OpenArmHW</plugin>
<param name="can_device">can0</param>
```

结论：**当前 `mode=sim` 不能证明会启用 MuJoCo；按静态代码它仍会选择真实硬件插件。** 在真机连接时不要把 `mode=sim` 当作安全开关。

### 12.4 配置文件的使用状态

`teleop.launch.py` 实际加载的是 `openarm_bringup/config/v10_controllers/openarm_v10_bimanual_controllers.yaml`。`openarm_bimanual_moveit_config/config/ros2_controllers.yaml` 有重复控制器块和可疑缩进，当前不应作为真机启动配置；除非先清理并验证该文件。

### 12.5 Kp/Kd YAML 与真实运行值并不一致

`control_gains.yaml` 为 J1~J7 设定 `Kp=[70,70,70,60,10,10,10]`、`Kd=[2.75,2.5,2.0,2.0,0.7,0.6,0.5]`，Xacro 确实会把它们写为 `kp1...kp7`、`kd1...kd7` 硬件参数。

但当前 `OpenArmHW::on_init()` 只读取 `can_device`、`prefix` 和 `disable_torque`；它没有读取上述 Kp/Kd 参数或 `can_fd`。同时它固定以 CAN-FD 创建总线，并使用 C++ 头文件内硬编码的 Kp/Kd。

所以目前：

```text
改 control_gains.yaml  ≠  改变真实电机刚度/阻尼
```

在确认插件已改为读取参数、重新构建并验证前，不要以为 YAML 调参已经生效。

## 13. 其余文件怎样理解

### 13.1 模型数据文件不是控制程序

`openarm_description/config/` 中的 `joint_limits.yaml`、`inertials.yaml`、`kinematics.yaml`、`kinematics_link.yaml`、`kinematics_offset.yaml` 是机器人模型数据：

- `joint_limits`：关节可动范围、最大速度、最大力矩；
- `inertials`：质量、质心和惯量，供动力学和仿真使用；
- `kinematics`：关节之间的坐标变换；
- `kinematics_link`：网格相对连杆的坐标变换；
- `kinematics_offset`：装配/标定的额外偏移。

它们通过 Xacro 生成 URDF。改动这些数据可能改变 RViz、IK、碰撞检测和规划结果，不能与“电机 MIT 增益”混为一谈。

### 13.2 可以安全学习模型的入口

`openarm_description/launch/display_openarm.launch.py` 仅启动：

```text
joint_state_publisher_gui + robot_state_publisher + RViz
```

它没有创建 `ros2_control_node`、没有加载 OpenArm 真机插件、不会打开 CAN 接口。因此它是理解关节名称、坐标系与限位的优先入口。

### 13.3 `openarm_can` 的 Python 绑定

文件 `openarm_can/python/src/openarm_can.cpp` 用 nanobind 把几乎整套 C++ API 暴露给 Python：CAN Socket、CAN 帧、Motor、MIT/PosVel/PosForce 参数、Arm/Gripper 和 `OpenArm` 总对象。

这不是高层安全接口；Python 调用 `enable_all()`、`set_zero_all()` 或 `mit_control_all()` 与 C++ 调用一样会操作真实电机。README 也明确标为实验性、不稳定 API。

### 13.4 MoveIt 启动文件的职责

- `display_openarm.launch.py`：纯可视化，安全。
- `move_group.launch.py`：只启动 MoveIt 规划服务，是否连接真机取决于别处是否有 ROS 控制系统。
- `spawn_controllers.launch.py`：只请求 controller_manager 加载控制器；若已有真机硬件系统则可能影响真机。
- `demo.launch.py`、`teleop.launch.py`：会创建 `ros2_control_node`，因此可能打开 CAN；其 `hardware_type`/仿真选择存在第 12.3 节已验证的问题。

## 14. 文件级最终归类

| 类别 | 主要文件/目录 | 是否可能动真机 |
|---|---|---:|
| 真机主控制 | `openarm_hardware/src/*.cpp` | 是 |
| 真机启动 | `openarm_bringup/launch/*.py` | 是 |
| CAN 驱动库/工具 | `openarm_can` | 工具和示例会动真机 |
| VR 到 OpenArm | `telepo_openarm/main.py` | 是，取决于 ROS 下游 |
| OpenArm 模型 | `openarm_description/urdf` 与 `config` | 否，仅展开/显示时 |
| MuJoCo 插件 | `openarm_mujoco_hardware` | 不应动 CAN，但当前启动配置没有正确选中它 |
| 天机/Wuji | `wujihandros2` | 控制的是另一套设备 |
| 关节 GUI | `ros2_joint_gui` | 会发布天机话题，非直接 OpenArm CAN |
| 移动底盘 | `base_openarm`、`telepo_openarm/base_control.py` | 是，控制轮式底盘 |
| `build`/`install`/`log`/`.cache`/`.cursor` | 生成物、日志、缓存 | 否 |

## 附录 A. 最后一轮源码核对：测试、示例与一个待修复点

### A.1 单元测试不会操作真实电机

`openarm_hardware/test/test_openarm_hardware.cpp` 是 ROS 2 插件加载测试：它只构造一段最小 URDF，然后让 `ResourceManager` 尝试加载硬件插件。它没有提供 `can_device`，也没有调用激活、读取或写入循环，因此不能当成真机测试程序，更不会在正常的测试路径中发送 CAN 控制帧。

### A.2 三类“读起来很有用、运行起来有风险”的示例

以下文件适合阅读协议和 API 用法，但不要在未确认急停、供电、机械活动范围及 CAN 接口归属前运行：

| 文件 | 运行时会做什么 |
|---|---|
| `openarm_can/examples/demo.cpp` | 使能电机并发送位置、力矩控制命令 |
| `openarm_can/examples/python/example.py`、`test_posvel.py`、`test_gripper_posforce.py` | 打开 SocketCAN、使能指定电机，部分程序会无限循环发命令 |
| `.local/openarm_vr_teleop/openarm_vr_teleop/gripper_test.py` | 通过 ROS 夹爪 Action 发送开合目标，属于真机标定/测试脚本 |

它们不是“仿真样例”。学习时先看其创建 CAN 总线、创建 Motor、`enable`、发送控制和关闭的顺序即可；不要照抄执行。

### A.3 `disable_torque` 分支的代码审阅结论

在 `openarm_hardware.cpp` 的 `OpenArmHW::write()` 中，`disable_torque_` 为真时，`return` 写在 `for` 循环内部：第一台电机收到一次全零 MIT 命令后，函数就立刻返回，后续关节和夹爪不会在该次调用中刷新。按代码意图，`return` 很可能应位于循环之后。

这是**静态代码审阅发现**，尚未在真机上验证；不要直接在连接机械臂时修改或测试。若以后要修复，应先在正确接入 MuJoCo 的仿真链路或断开功率的台架环境中验证，再构建部署。

### A.4 这次“所有 OpenArm 文件”指的范围

我已按用途检查了小主机上所有项目自身的 OpenArm 源码、启动文件、Xacro/URDF、控制器 YAML、CAN 库、VR 遥操副本、GUI 和以 OpenArm 命名的底盘脚本；并把 `build`、`install`、`log`、Python 虚拟环境、第三方依赖、编辑器缓存和回收站排除为非源代码。

学习顺序请固定为：**模型显示（第 13.2 节）→ ROS 控制器配置（第 4、12 节）→ `openarm_hardware`（第 5～8 节）→ `openarm_can`（第 9～11 节）→ VR/MoveIt（第 12、13 节）**。先建立这条链路，再阅读底盘和 Tianji/Wuji 这两套独立系统，最不容易混淆。

## 15. 移动底盘文件的最终判断

这些文件带有 `openarm` 路径名，但控制对象是轮式底盘，和 7 轴机械臂是不同硬件。

| 文件 | 当前性质 |
|---|---|
| `/home/nuc/telepo_openarm/base_control.py` | 较新的 SocketCAN 底盘控制模块；供 VR 主程序导入 |
| `/home/nuc/base_openarm/base_tcp_server.py` | 较早的本地 TCP 控制服务；使用 `can2` 和扩展 CAN ID |
| `/home/nuc/base_openarm/base_keyboard_input.py` | 蓝牙转 CAN 模块控制脚本，直接写 BLE GATT 特征值 |
| `/home/nuc/base_openarm/systemd/openarm-chassis-tcp.service` | 安装旧 TCP 服务的 systemd 单元 |

新版 `base_control.py` 的每个动作每 20 ms 重发 4 个 CAN 帧：两个帧给左轮、两个帧给右轮。`command_token` 的作用是防止旧动作在操作员换键或停止后继续发送。

已只读检查到 `openarm-chassis-tcp.service` 当前为 `disabled`、`inactive`。它的说明仍写着“蓝牙 + TCP”，但实际 `ExecStart` 是旧版 `base_tcp_server.py`；这份说明与实现已经不完全一致。

更重要的是：旧版使用 `can2` 且使用扩展帧；新版使用 `can0` 且使用标准帧。二者不能简单互换，也不应在未确认硬件适配器/总线参数时启动。

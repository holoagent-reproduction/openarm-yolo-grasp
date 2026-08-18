# OpenArm ROS 2 关节控制界面

这是一个基于 Tkinter 的双臂关节控制界面，面向当前主机上的 OpenArm 摇操链路。

它订阅以下实际反馈话题：

- `/left_arm/joint_states`
- `/right_arm/joint_states`

它只在用户明确启用并点击“发送轨迹”后，向以下命令话题发布单次 `sensor_msgs/JointState`：

- `/left_arm/joint_commands`
- `/right_arm/joint_commands`

每条手臂包含 7 个关节，界面以“度”为单位显示和输入，发布前自动转换成弧度。默认关闭输出，避免启动界面时意外移动机械臂。

## 安全约束

- 先清空机械臂工作范围，确保急停有效。
- 默认软限位为 ±170°，启动前请按真实机械臂的关节限位修改 `openarm_joint_gui/gui_node.py` 中的 `JOINT_LIMIT_DEGREES`。
- 本工具发布目标位置，不做碰撞检测、路径规划或力控；每次仅建议小幅调整。

## 安装与运行

将本包放到主机工作区的 `src` 目录后，在主机执行：

```bash
cd ~/ros2_ws
colcon build --packages-select openarm_joint_gui
source install/setup.bash
ros2 run openarm_joint_gui joint_gui
```

第一条进入 ROS 工作区；第二条仅构建本界面包；第三条加载刚构建的包；第四条启动图形界面。

在此之前，必须先启动 OpenArm 的硬件与 ROS 摇操节点，否则界面会显示“未收到反馈”，且命令不会被底层控制器消费。

## 界面操作

1. 点击“从反馈同步”将滑块对齐到实际关节角度。
2. 小幅移动所需关节滑块或输入框。
3. 勾选“启用真实机械臂输出”。
4. 选择左臂、右臂或双臂后点击“发送轨迹”。

“双臂发送”会在同一个 ROS 时刻发布左右两个关节目标。

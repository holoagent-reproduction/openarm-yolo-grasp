# OpenArm 双相机 YOLOE 语义抓取

本包在不修改 OpenArm、MoveIt 和原相机启动文件的前提下，提供以下两条独立流程：

- AprilTag：相机标定与定位真值验证；
- YOLOE：普通无标签物品的中文语义识别、三维定位、规划预览和人工确认抓取。

## 安全状态

`config/semantic_grasp.yaml` 中三个标定开关默认为 `false`，启动参数 `allow_motion` 也默认为 `false`。任一门控未通过时，系统最多完成识别和规划，不会发送真实轨迹。

## 构建

```bash
cd /home/nuc/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select openarm_semantic_grasp_interfaces openarm_vision_grasp --symlink-install
source install/setup.bash
```

第一条命令进入 NUC 的 ROS 工作区；第二条命令加载 ROS 2 Humble；第三条命令只构建新增接口包和视觉包；第四条命令加载构建结果。

## 启动安全预览

启动文件会自动忽略用户目录里的 NumPy 2.x，使用与 ROS Humble OpenCV 兼容的系统 NumPy 1.x。

```bash
ros2 launch openarm_vision_grasp semantic_grasp.launch.py allow_motion:=false show_ui:=false
```

该命令启动双相机、YOLOE 客户端、三维融合、目标跟踪、抓取 Action 和 HTTP Skill，且禁止真实运动。

确认界面需要在 NUC 本地桌面打开：

```bash
DISPLAY=:0 XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.6AD0T3 QT_QPA_PLATFORM=xcb ros2 run openarm_vision_grasp semantic_confirm_ui
```

该命令通过 SSH 发起程序，但窗口显示在 NUC 的显示器上，不显示到 Windows。

## 发送预览任务

```bash
ros2 topic pub --once /openarm_vision/semantic_goal std_msgs/msg/String "{data: '{\"instruction\":\"拿起桌子上的水杯\",\"preview_only\":true}'}"
```

该命令只发送一次中文语义任务，并要求仅规划预览。

## 真实运动启用条件

必须全部满足后才能把 `allow_motion` 设为 `true`：

1. 头部外参验收通过；
2. 左腕手眼平移 RMS 小于 10 mm、旋转 RMS 小于 3°；
3. `openarm_left_hand_tcp` 向下抓取姿态完成空载验证；
4. MoveIt、控制器、碰撞环境和急停正常；
5. 完成至少 20 次无碰撞空载预演。

即使启用真实运动，每个任务仍必须在界面人工确认。

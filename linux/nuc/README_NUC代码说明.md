# NUC 当前视觉抓取代码备份

本目录从 `nuc@172.16.13.202:/home/nuc/ros2_ws` 下载，保存当前正在使用的视觉抓取相关源码和配置。

## 目录

- `ros2_ws/src/openarm_vision_grasp/`：双相机、YOLOE、三维定位、语义抓取状态机、确认界面、启动文件和配置。
- `ros2_ws/src/openarm_semantic_grasp_interfaces/`：`SemanticPick` Action 以及语义检测/目标消息接口。
- `ros2_ws/src/openarm_calibration_jog/`：机械臂微调和同步手眼标定采样工具。
- `ros2_ws/src/openarm_description/`：OpenArm URDF、网格模型、末端和 ros2_control 描述。
- `ros2_ws/src/openarm_ros2/openarm_bimanual_moveit_config/`：双臂 MoveIt 配置、规划组、控制器和 RViz 启动文件。
- `ros2_ws/src/openarm_ros2/openarm_bringup/`：OpenArm 启动和硬件控制配置。
- `ros2_ws/src/openarm_ros2/openarm_hardware/`：OpenArm 硬件接口源码。
- `ros2_ws/src/openarm_ros2/openarm/`：OpenArm 基础 ROS 包。
- `ros2_ws/src/openarm_can/`：真实 OpenArm CAN 总线接口和控制示例。
- `ros2_ws/src/realsense_camera_launch/`：D435i/D415 相机启动和配置。
- `ros2_ws/src/ros2_joint_gui/`：关节状态显示和手动控制界面。
- `linux_scripts/`：NUC 上实际使用过的机器人、相机和遥操作启动脚本。
- `ros2_ws/calibration_samples/`：NUC 上已有的标定样本和复测结果。
- `ros2_ws/semantic_backend.log`：最近一次语义后端运行日志。

## NUC 原始位置

```text
/home/nuc/ros2_ws/src/openarm_vision_grasp
/home/nuc/ros2_ws/src/openarm_semantic_grasp_interfaces
/home/nuc/ros2_ws/src/openarm_calibration_jog
/home/nuc/ros2_ws/src/openarm_description
/home/nuc/ros2_ws/src/openarm_ros2/openarm_bimanual_moveit_config
/home/nuc/ros2_ws/src/openarm_ros2/openarm_bringup
/home/nuc/ros2_ws/src/openarm_ros2/openarm_hardware
/home/nuc/ros2_ws/src/openarm_ros2/openarm
/home/nuc/ros2_ws/src/openarm_can
/home/nuc/ros2_ws/src/realsense_camera_launch
/home/nuc/ros2_ws/src/ros2_joint_gui
/home/nuc/ros2_ws/calibration_samples
/home/nuc/ros2_ws/semantic_backend.log
```

本备份不包含 `build/`、`install/`、`log/` 和 Python `__pycache__`，这些目录属于编译产物，可以在 NUC 上重新生成。

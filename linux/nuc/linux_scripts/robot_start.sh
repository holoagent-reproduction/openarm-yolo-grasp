#!/bin/bash

# --- 1. 环境初始化 ---
source /opt/ros/humble/setup.bash
source /home/nuc/ros2_ws/install/setup.bash

# --- 2. 启动 RealSense 相机 (后台) ---
echo "Step 1: Launching RealSense..."
ros2 launch realsense_camera_launch multi_realsense.launch.py > /tmp/realsense.log 2>&1 &

# 等待相机硬件稳定
sleep 8

# --- 3. 启动 OpenArm MoveIt 配置 (后台) ---
echo "Step 2: Launching MoveIt Config..."
ros2 launch openarm_bimanual_moveit_config teleop.launch.py > /tmp/teleop.log 2>&1 &

# MoveIt 涉及多个控制器和 TF 树构建，给更长的时间
sleep 15

# --- 4. 启动 VR 摇操程序 (后台) ---
echo "Step 3: Starting VR Teleop main program..."
/home/nuc/telepo_openarm/teleop_openarm/bin/python /home/nuc/telepo_openarm/openarm_vr_teleop/openarm_vr_teleop/main.py > /tmp/vr_main.log 2>&1 &

# 等待摇操逻辑初始化
sleep 5

# --- 5. 启动 RoboDriver (主进程) ---
# 作为脚本的最后一行，不加 &，让 systemd 监控这个进程
echo "Step 4: Starting RoboDriver..."
/home/nuc/RoboDriver/.venv/bin/robodriver-run --robot.type openarm-teleop-ros2


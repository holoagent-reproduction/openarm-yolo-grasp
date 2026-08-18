source ~/ros2_ws/install/setup.bash

ros2 launch realsense_camera_launch multi_realsense.launch.py

ros2 launch openarm_bimanual_moveit_config teleop.launch.py

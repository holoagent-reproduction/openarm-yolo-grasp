"""启动独立双相机、AprilTag 检测、抓取任务与确认界面。"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("openarm_vision_grasp"))
    cameras = LaunchConfiguration("cameras_file")
    objects = LaunchConfiguration("objects_file")
    drop = LaunchConfiguration("drop_zone_file")
    allow_motion = LaunchConfiguration("allow_motion")
    dual = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / "launch" / "dual_realsense_grasp.launch.py")), launch_arguments={"cameras_file": cameras}.items())
    return LaunchDescription([
        DeclareLaunchArgument("cameras_file", default_value=str(share / "config" / "cameras.yaml")),
        DeclareLaunchArgument("objects_file", default_value=str(share / "config" / "tag_objects.yaml")),
        DeclareLaunchArgument("drop_zone_file", default_value=str(share / "config" / "drop_zone.yaml")),
        DeclareLaunchArgument("allow_motion", default_value="false", description="仅在完成所有标定和空载验证后才设为 true。"),
        dual,
        Node(package="openarm_vision_grasp", executable="tag_detector", output="screen", additional_env={"PYTHONNOUSERSITE": "1"}, parameters=[{"cameras_file": cameras, "objects_file": objects}]),
        Node(package="openarm_vision_grasp", executable="grasp_task_node", output="screen", parameters=[{"cameras_file": cameras, "objects_file": objects, "drop_zone_file": drop, "allow_motion": allow_motion}]),
        Node(package="openarm_vision_grasp", executable="confirm_ui", output="screen"),
    ])

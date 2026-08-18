"""独立启动 D435i 头部相机与左腕 D415；不会改动原三相机启动文件。"""
from pathlib import Path
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load(path):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _build(context):
    config = _load(LaunchConfiguration("cameras_file").perform(context))
    actions = []
    for key, camera in config["cameras"].items():
        serial = str(camera.get("serial", "")).strip()
        if not serial:
            raise RuntimeError(f"请先在 cameras.yaml 填写 {key} 的 RealSense 序列号。")
        parameters = {
            "camera_name": camera["camera_name"],
            "serial_no": "_" + serial,
            "enable_color": True,
            "enable_depth": bool(camera.get("enable_depth", True)),
            "enable_infra": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_accel": False,
            "enable_gyro": False,
            "enable_motion": False,
            "align_depth.enable": bool(camera.get("enable_depth", True)),
            "pointcloud.enable": False,
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
            "publish_tf": True,
            "tf_publish_rate": 10.0,
            "wait_for_device_timeout": 15.0,
        }
        actions.append(Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            namespace="openarm_vision",
            name=camera["camera_name"],
            parameters=[parameters],
            output="screen",
        ))
        if camera.get("calibrated", False):
            t, q = camera["translation"], camera["rotation_xyzw"]
            actions.append(Node(package="tf2_ros", executable="static_transform_publisher",
                arguments=["--x", str(t[0]), "--y", str(t[1]), "--z", str(t[2]),
                           "--qx", str(q[0]), "--qy", str(q[1]),
                           "--qz", str(q[2]), "--qw", str(q[3]),
                           "--frame-id", camera["parent_frame"],
                           "--child-frame-id", camera["mount_frame"]], output="screen"))
    return actions


def generate_launch_description():
    default = str(Path(get_package_share_directory("openarm_vision_grasp")) / "config" / "cameras.yaml")
    return LaunchDescription([DeclareLaunchArgument("cameras_file", default_value=default), OpaqueFunction(function=_build)])

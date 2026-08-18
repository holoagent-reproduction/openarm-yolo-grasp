#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch 3 RealSense cameras with fixed ROS2 topic mapping."""

from pathlib import Path
from typing import Dict

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


def _load_yaml_file(file_path: str) -> Dict[str, str]:
    """Load mapping yaml from given path."""
    config_path = Path(file_path).expanduser()
    if not config_path.exists():
        raise RuntimeError(f"配置文件不存在: {config_path}")

    with config_path.open("r", encoding="utf-8") as file_obj:
        data = yaml.safe_load(file_obj) or {}

    if not isinstance(data, dict):
        raise RuntimeError("配置文件格式错误，根节点必须是 key-value 字典。")

    return data


def _discover_d435_serial() -> str:
    """Find D435 serial from connected RealSense devices."""
    if rs is None:
        raise RuntimeError(
            "未安装 pyrealsense2，无法自动识别 D435。"
            "请先安装 pyrealsense2，或扩展配置为手动指定 D435 序列号。"
        )

    context = rs.context()
    devices = context.query_devices()

    d435_serial_list = []
    for device in devices:
        name = device.get_info(rs.camera_info.name)
        serial = device.get_info(rs.camera_info.serial_number)
        if "D435" in name:
            d435_serial_list.append(serial)

    if len(d435_serial_list) != 1:
        raise RuntimeError(
            "自动识别 D435 失败，请确保仅连接 1 台 D435。"
            f"当前识别到 D435 数量: {len(d435_serial_list)}"
        )

    return d435_serial_list[0]


def _build_launch(context, *_args, **_kwargs):
    """Build launch actions dynamically from config file."""
    config_path = LaunchConfiguration("mapping_file").perform(context)
    config = _load_yaml_file(config_path)

    left_serial = str(config.get("d415_left_serial", "")).strip()
    right_serial = str(config.get("d415_right_serial", "")).strip()
    if not left_serial or not right_serial:
        raise RuntimeError(
            "配置文件必须包含 d415_left_serial 和 d415_right_serial 两个字段。"
        )
    if left_serial == right_serial:
        raise RuntimeError("d415_left_serial 与 d415_right_serial 不能相同。")

    d435_serial = _discover_d435_serial()

    rs_launch = str(Path(get_package_share_directory("realsense2_camera")) / "launch" / "rs_launch.py")

    def camera_group(camera_name: str, serial_no: str):
        base = f"/openarm/{camera_name}"
        is_top_camera = camera_name == "D435_top"
        launch_args = {
            "camera_namespace": "openarm",
            "camera_name": camera_name,
            # realsense2_camera expects numeric serial as string; prefix '_' avoids YAML int coercion.
            "serial_no": f"_{serial_no}",
            "enable_color": "true",
            "enable_depth": "true",
            "rgb_camera.color_profile": "640x480x30",
            "depth_module.depth_profile": "640x480x30",
            "depth_module.infra_profile": "640x480x30",
            "align_depth.enable": "false",
            "publish_tf": "false",
            "tf_publish_rate": "0.0",
            # Allow device re-enumeration during startup to avoid transient misses.
            "wait_for_device_timeout": "15.0",
        }
        if is_top_camera:
            # D435_top occasionally enters "node up but no frame" state;
            # initial_reset is equivalent to an automatic hw_reset at startup.
            launch_args["initial_reset"] = "true"

        return GroupAction(
            actions=[
                SetRemap(
                    src=f"{base}/depth/image_rect_raw",
                    dst=f"{base}/color/image_raw/depth",
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rs_launch),
                    launch_arguments=launch_args.items(),
                ),
            ]
        )

    return [
        LogInfo(msg=f"[realsense_camera_launch] D435 serial: {d435_serial}"),
        LogInfo(msg=f"[realsense_camera_launch] D415 left serial: {left_serial}"),
        LogInfo(msg=f"[realsense_camera_launch] D415 right serial: {right_serial}"),
        camera_group("D435_top", d435_serial),
        camera_group("D415_wrist_left", left_serial),
        camera_group("D415_wrist_right", right_serial),
    ]


def generate_launch_description():
    """Generate ROS2 launch description."""
    default_config = str(
        Path(get_package_share_directory("realsense_camera_launch"))
        / "config"
        / "realsense_sn.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mapping_file",
                default_value=default_config,
                description="D415 left/right serial mapping yaml file",
            ),
            OpaqueFunction(function=_build_launch),
        ]
    )

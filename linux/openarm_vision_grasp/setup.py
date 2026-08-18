from setuptools import find_packages, setup

package_name = "openarm_vision_grasp"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "launch/dual_realsense_grasp.launch.py",
            "launch/vision_grasp.launch.py",
            "launch/semantic_grasp.launch.py",
        ]),
        ("share/" + package_name + "/config", [
            "config/cameras.yaml", "config/tag_objects.yaml", "config/drop_zone.yaml",
            "config/semantic_grasp.yaml",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenArm",
    maintainer_email="nuc@example.com",
    description="双相机 AprilTag 视觉抓取",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "tag_detector = openarm_vision_grasp.tag_detector:main",
        "grasp_task_node = openarm_vision_grasp.grasp_task_node:main",
        "confirm_ui = openarm_vision_grasp.confirm_ui:main",
        "semantic_command_node = openarm_vision_grasp.semantic_command_node:main",
        "yoloe_client_node = openarm_vision_grasp.yoloe_client_node:main",
        "semantic_3d_node = openarm_vision_grasp.semantic_3d_node:main",
        "object_tracker_node = openarm_vision_grasp.object_tracker_node:main",
        "semantic_grasp_task_node = openarm_vision_grasp.semantic_grasp_task_node:main",
        "semantic_confirm_ui = openarm_vision_grasp.semantic_confirm_ui:main",
        "semantic_skill_bridge = openarm_vision_grasp.semantic_skill_bridge:main",
        "semantic_step_grasp_node = openarm_vision_grasp.semantic_step_grasp_node:main",
        "wrist_depth_range_node = openarm_vision_grasp.wrist_depth_range_node:main",
    ]},
)

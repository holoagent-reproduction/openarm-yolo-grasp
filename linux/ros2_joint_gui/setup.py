from setuptools import find_packages, setup

package_name = "openarm_joint_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/openarm_joint_gui"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenArm",
    maintainer_email="nuc@example.com",
    description="OpenArm 双臂关节可视化控制界面。",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "joint_gui = openarm_joint_gui.gui_node:main",
        ],
    },
)

import os
import xacro
from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def resolve_hardware_type_str(context, hardware_type, use_fake_hardware,
                               use_sim_hardware):
    """
    v10.urdf.xacro 只识别 hardware_type（real / mock / mujoco）。
    use_fake_hardware / use_sim_hardware 仅用于在本 launch 中推导 hardware_type。
    显式传入的 hardware_type 优先于仍为默认值 real 时的推导。
    """
    hardware_type_str = context.perform_substitution(hardware_type)
    use_fake_str = context.perform_substitution(use_fake_hardware)
    use_sim_str = context.perform_substitution(use_sim_hardware)
    if hardware_type_str != "real":
        return hardware_type_str
    if use_sim_str.lower() == "true":
        return "mujoco"
    if use_fake_str.lower() == "true":
        return "mock"
    return "real"


def generate_robot_description(
    context: LaunchContext,
    description_package,
    description_file,
    arm_type,
    hardware_type,
    use_fake_hardware,
    use_sim_hardware,
    right_can_interface,
    left_can_interface,
):
    """Render Xacro and return XML string."""
    description_package_str = context.perform_substitution(description_package)
    description_file_str = context.perform_substitution(description_file)
    arm_type_str = context.perform_substitution(arm_type)
    right_can_interface_str = context.perform_substitution(right_can_interface)
    left_can_interface_str = context.perform_substitution(left_can_interface)
    hardware_type_resolved = resolve_hardware_type_str(
        context, hardware_type, use_fake_hardware, use_sim_hardware)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "urdf",
        "robot",
        description_file_str,
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "hardware_type": hardware_type_resolved,
            "ros2_control": "true",
            "left_can_interface": left_can_interface_str,
            "right_can_interface": right_can_interface_str,
            # arm_prefix unused inside xacro but kept for completeness
        },
    ).toprettyxml(indent="  ")

    return robot_description


def robot_nodes_spawner(
    context: LaunchContext,
    description_package,
    description_file,
    arm_type,
    hardware_type,
    use_fake_hardware,
    use_sim_hardware,
    controllers_file,
    right_can_interface,
    left_can_interface,
    arm_prefix,
):
    robot_description = generate_robot_description(
        context,
        description_package,
        description_file,
        arm_type,
        hardware_type,
        use_fake_hardware,
        use_sim_hardware,
        right_can_interface,
        left_can_interface,
    )

    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    # MoveIt/RViz 使用本次按启动参数生成的同一份双臂 URDF。
    # 否则 RViz 会因缺少 robot_description 而无法加载模型。
    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config"
    ).to_moveit_configs()
    moveit_params = moveit_config.to_dict()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params, robot_description_param],
    )

    rviz_cfg = os.path.join(
        get_package_share_directory(
            "openarm_bimanual_moveit_config"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_cfg],
        parameters=[moveit_params, robot_description_param],
    )

    return [robot_state_pub_node, control_node, move_group_node, rviz_node]


def controller_spawner(context: LaunchContext, robot_controller):
    robot_controller_str = context.perform_substitution(robot_controller)

    if robot_controller_str == "forward_position_controller":
        left = "left_forward_position_controller"
        right = "right_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        left = "left_joint_trajectory_controller"
        right = "right_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[left, right, "-c", "/controller_manager"],
        )
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="openarm_description",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="v10.urdf.xacro",
        ),
        DeclareLaunchArgument("arm_type", default_value="v10"),
        DeclareLaunchArgument(
            "hardware_type",
            default_value="real",
            choices=["real", "mock", "mujoco"],
            description=(
                "ros2_control 硬件类型；保持默认 real 时可用 "
                "use_sim_hardware:=true 选 MuJoCo WebSocket"
            ),
        ),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument("use_sim_hardware", default_value="false"),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
        ),
        DeclareLaunchArgument(
            "runtime_config_package", default_value="openarm_bringup"
        ),
        DeclareLaunchArgument("arm_prefix", default_value=""),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("left_can_interface", default_value="can1"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_v10_bimanual_controllers.yaml",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    arm_type = LaunchConfiguration("arm_type")
    hardware_type = LaunchConfiguration("hardware_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_sim_hardware = LaunchConfiguration("use_sim_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    right_can_interface = LaunchConfiguration("right_can_interface")
    left_can_interface = LaunchConfiguration("left_can_interface")
    arm_prefix = LaunchConfiguration("arm_prefix")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         "v10_controllers", controllers_file]
    )

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[
            description_package,
            description_file,
            arm_type,
            hardware_type,
            use_fake_hardware,
            use_sim_hardware,
            controllers_file,
            right_can_interface,
            left_can_interface,
            arm_prefix,
        ],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    controller_spawner_func = OpaqueFunction(
        function=controller_spawner, args=[robot_controller])

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller",
                   "right_gripper_controller", "-c", "/controller_manager"],
    )

    delayed_jsb = TimerAction(period=2.0, actions=[jsb_spawner])
    delayed_arm_ctrl = TimerAction(
        period=1.0, actions=[controller_spawner_func])
    delayed_gripper = TimerAction(period=1.0, actions=[gripper_spawner])


    return LaunchDescription(
        declared_arguments
        + [
            robot_nodes_spawner_func,
            delayed_jsb,
            delayed_arm_ctrl,
            delayed_gripper,
        ]
    )

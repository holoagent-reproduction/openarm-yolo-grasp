"""带人工确认、标定门控和失败中止的 OpenArm 左臂语义抓取 Action 服务。"""
import asyncio
import copy
import json
import math
import uuid

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, OrientationConstraint,
                             PositionConstraint, RobotState)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String

from openarm_semantic_grasp_interfaces.action import SemanticPick
from openarm_semantic_grasp_interfaces.msg import SemanticDetection, SemanticObject3D
from .semantic_parser import ParseError, load_config, parse_instruction
from .instance_guard import acquire_singleton


def multiply_quaternion(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


class SemanticGraspTaskNode(Node):
    def __init__(self):
        super().__init__("openarm_semantic_grasp_task")
        self.declare_parameter("config_file", "")
        self.declare_parameter("allow_motion", False)
        self.config = load_config(self.get_parameter("config_file").value)
        self.allow_motion = bool(self.get_parameter("allow_motion").value)
        self.motion = self.config["motion"]; self.frames = self.config["frames"]
        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.query_pub = self.create_publisher(String, "/openarm_vision/semantic_query", 10)
        self.status_pub = self.create_publisher(String, "/openarm_vision/semantic_task_status", 20)
        self.create_subscription(SemanticObject3D, "/openarm_vision/semantic_target", self.on_target, 20,
                                 callback_group=self.callback_group)
        self.create_subscription(SemanticDetection, "/openarm_vision/semantic_detections", self.on_detection, 30,
                                 callback_group=self.callback_group)
        self.create_subscription(String, "/openarm_vision/semantic_control", self.on_control, 10,
                                 callback_group=self.callback_group)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 20,
                                 callback_group=self.callback_group)
        self.move_group = ActionClient(self, MoveGroup, "/move_action", callback_group=self.callback_group)
        self.execute_trajectory = ActionClient(self, ExecuteTrajectory, "/execute_trajectory", callback_group=self.callback_group)
        self.gripper = ActionClient(self, GripperCommand, "/left_gripper_controller/gripper_cmd",
                                    callback_group=self.callback_group)
        self.cartesian = self.create_client(GetCartesianPath, "/compute_cartesian_path",
                                            callback_group=self.callback_group)
        self.action_server = ActionServer(
            self, SemanticPick, "/openarm_vision/semantic_pick", execute_callback=self.execute,
            goal_callback=self.goal_callback, cancel_callback=self.cancel_callback,
            callback_group=self.callback_group)
        self.active_task_id = None; self.latest_target = None; self.latest_wrist = None
        self.confirmed = False; self.cancel_requested = False; self.gripper_position = None
        self.get_logger().info(f"语义抓取服务已启动；allow_motion={self.allow_motion}")

    def goal_callback(self, goal):
        if self.active_task_id is not None:
            return GoalResponse.REJECT
        try:
            parse_instruction(goal.instruction, self.config)
            return GoalResponse.ACCEPT
        except ParseError:
            return GoalResponse.REJECT

    def cancel_callback(self, _goal_handle):
        self.cancel_requested = True
        return CancelResponse.ACCEPT

    def on_control(self, message):
        try:
            command = json.loads(message.data)
            if str(command.get("task_id")) != str(self.active_task_id):
                return
            if command.get("command") == "confirm": self.confirmed = True
            if command.get("command") == "cancel": self.cancel_requested = True
        except ValueError:
            pass

    def on_target(self, message):
        if message.task_id == self.active_task_id:
            self.latest_target = message

    def on_detection(self, message):
        if message.camera_role == "wrist" and message.task_id == self.active_task_id:
            self.latest_wrist = message

    def on_joint_state(self, message):
        for name in ("openarm_left_finger_joint1", "left_finger_joint1"):
            if name in message.name:
                self.gripper_position = float(message.position[message.name.index(name)])
                break

    def publish_query(self, task_id, class_name, active=True):
        message = String(); message.data = json.dumps(
            {"task_id": task_id, "class_name": class_name, "active": active}, ensure_ascii=False)
        self.query_pub.publish(message)

    def feedback(self, goal_handle, state, message, target=None, plan_ready=False):
        value = SemanticPick.Feedback(); value.task_id = self.active_task_id or ""
        value.state = state; value.message = message; value.plan_ready = plan_ready
        if target is not None:
            value.target_class = target.class_name; value.confidence = target.confidence
            value.target_pose.header = target.header; value.target_pose.pose = target.pose
        goal_handle.publish_feedback(value)
        status = String(); status.data = json.dumps({
            "task_id": value.task_id, "state": state, "message": message,
            "target_class": value.target_class, "confidence": value.confidence,
            "plan_ready": plan_ready,
        }, ensure_ascii=False)
        self.status_pub.publish(status)

    def result(self, success, class_name, track_id, error_code, message):
        value = SemanticPick.Result(); value.success = success; value.task_id = self.active_task_id or ""
        value.target_class = class_name; value.target_track_id = track_id
        value.error_code = error_code; value.message = message
        return value

    def calibration_gate(self):
        calibration = self.config["calibration"]
        missing = [name for name in ("head_calibrated", "wrist_calibrated", "tool_orientation_calibrated")
                   if not calibration.get(name, False)]
        return missing

    def target_poses(self, target):
        profile = self.config["classes"][target.class_name]
        center_z = target.pose.position.z
        low_z = center_z - target.size.z * 0.5
        grasp_z = low_z + target.size.z * float(profile.get("grasp_height_ratio", 0.55))
        # 保持规划瞬间的当前 TCP 姿态，只改变抓取位置。
        # 这样不会因预设四元数或物体 yaw 改变末端方向，仍保留姿态约束。
        orientation = tuple(self.motion["tool_down_orientation_xyzw"])
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frames["world"], self.motion["tcp_link"], Time())
            r = transform.transform.rotation
            orientation = (r.x, r.y, r.z, r.w)
        except Exception as exc:
            self.get_logger().warning(
                f"读取当前 TCP 姿态失败，使用配置姿态：{exc}", throttle_duration_sec=2.0)
        grasp = Pose()
        grasp.position.x = target.pose.position.x; grasp.position.y = target.pose.position.y; grasp.position.z = grasp_z
        grasp.orientation.x, grasp.orientation.y, grasp.orientation.z, grasp.orientation.w = orientation
        pregrasp = copy.deepcopy(grasp); pregrasp.position.z += float(self.motion["pregrasp_distance_m"])
        lift = copy.deepcopy(grasp); lift.position.z += float(self.motion["lift_distance_m"])
        return pregrasp, grasp, lift

    def pose_constraints(self, pose, position_only=False):
        # 预览阶段使用小但非退化的目标区域；过小的 1 cm 立方体叠加
        # 严格姿态约束时，MoveIt 容易把目标判为 GOAL_STATE_INVALID。
        primitive = SolidPrimitive(); primitive.type = SolidPrimitive.BOX; primitive.dimensions = [0.03, 0.03, 0.03]
        volume = BoundingVolume(); volume.primitives = [primitive]; volume.primitive_poses = [pose]
        position = PositionConstraint(); position.header.frame_id = self.frames["world"]
        position.link_name = self.motion["tcp_link"]; position.constraint_region = volume; position.weight = 1.0
        constraints = Constraints(); constraints.position_constraints = [position]
        if not position_only:
            orientation = OrientationConstraint(); orientation.header.frame_id = self.frames["world"]
            orientation.link_name = self.motion["tcp_link"]; orientation.orientation = pose.orientation
            orientation.absolute_x_axis_tolerance = 0.20; orientation.absolute_y_axis_tolerance = 0.20
            orientation.absolute_z_axis_tolerance = 0.25; orientation.weight = 1.0
            constraints.orientation_constraints = [orientation]
        return constraints

    async def plan_pregrasp(self, pose, position_only=False):
        if not self.move_group.wait_for_server(timeout_sec=3.0):
            return None, "MoveIt /move_action 不可用"
        goal = MoveGroup.Goal(); goal.request.group_name = self.motion["group_name"]
        goal.request.allowed_planning_time = float(self.motion["planning_time_s"])
        goal.request.max_velocity_scaling_factor = float(self.motion["velocity_scaling"])
        goal.request.max_acceleration_scaling_factor = float(self.motion["acceleration_scaling"])
        goal.request.goal_constraints = [self.pose_constraints(pose, position_only)]
        goal.planning_options.plan_only = True
        handle = await self.move_group.send_goal_async(goal)
        if not handle.accepted:
            return None, "MoveIt 拒绝规划目标"
        wrapped = await handle.get_result_async(); response = wrapped.result
        if response.error_code.val != response.error_code.SUCCESS or not response.planned_trajectory.joint_trajectory.points:
            return None, f"预抓取规划失败，MoveIt 错误码 {response.error_code.val}"
        return response.planned_trajectory, ""

    async def execute_trajectory_goal(self, trajectory):
        if not self.execute_trajectory.wait_for_server(timeout_sec=3.0):
            return False, "/execute_trajectory 不可用"
        goal = ExecuteTrajectory.Goal(); goal.trajectory = trajectory
        handle = await self.execute_trajectory.send_goal_async(goal)
        if not handle.accepted:
            return False, "轨迹执行目标被拒绝"
        wrapped = await handle.get_result_async()
        return wrapped.result.error_code.val == wrapped.result.error_code.SUCCESS, f"轨迹执行错误码 {wrapped.result.error_code.val}"

    async def cartesian_path(self, waypoints):
        if not self.cartesian.wait_for_service(timeout_sec=3.0):
            return None, "/compute_cartesian_path 不可用"
        request = GetCartesianPath.Request(); request.header.frame_id = self.frames["world"]
        request.group_name = self.motion["group_name"]; request.link_name = self.motion["tcp_link"]
        request.start_state = RobotState(); request.start_state.is_diff = True
        request.waypoints = waypoints; request.max_step = float(self.motion["cartesian_step_m"])
        request.jump_threshold = 0.0; request.avoid_collisions = True
        response = await self.cartesian.call_async(request)
        minimum = float(self.motion["minimum_cartesian_fraction"])
        if response.fraction < minimum or response.error_code.val != response.error_code.SUCCESS:
            return None, f"笛卡尔路径比例 {response.fraction:.3f}，要求至少 {minimum:.3f}"
        return response.solution, ""

    async def command_gripper(self, position):
        if not bool(self.motion.get("gripper_motion_enabled", False)):
            self.get_logger().warning("夹爪运动已禁用，拒绝发送夹爪命令")
            return False
        if not self.gripper.wait_for_server(timeout_sec=3.0):
            return False
        goal = GripperCommand.Goal(); goal.command.position = float(position)
        goal.command.max_effort = float(self.motion["gripper_max_effort"])
        handle = await self.gripper.send_goal_async(goal)
        if not handle.accepted:
            return False
        await handle.get_result_async()
        return True

    def wrist_in_roi(self, class_name):
        detection = self.latest_wrist
        if detection is None or detection.class_name != class_name or detection.mask.width <= 0 or detection.mask.height <= 0:
            return False
        x1, y1, x2, y2 = detection.bbox_xyxy; cx = (x1 + x2) * 0.5 / detection.mask.width
        cy = (y1 + y2) * 0.5 / detection.mask.height
        rx1, ry1, rx2, ry2 = self.motion["wrist_roi_normalized"]
        return rx1 <= cx <= rx2 and ry1 <= cy <= ry2

    def execute(self, goal_handle):
        """为 ROS 2 ActionServer 提供同步入口，并绑定独立 asyncio 事件循环。

        rclpy 的 ActionServer 在执行线程中调用回调时不一定预先设置
        asyncio loop；直接把 async 函数交给它会在第一次 asyncio.sleep
        或异步服务调用时触发 ``no running event loop``。
        """
        return asyncio.run(self._execute_async(goal_handle))

    async def _execute_async(self, goal_handle):
        parsed = parse_instruction(goal_handle.request.instruction, self.config)
        class_name = parsed["class_name"]; task_id = uuid.uuid4().hex
        self.active_task_id = task_id; self.latest_target = None; self.latest_wrist = None
        self.confirmed = False; self.cancel_requested = False
        self.publish_query(task_id, class_name, True)
        try:
            self.feedback(goal_handle, "detecting", f"正在寻找 {class_name}")
            for _ in range(100):
                if self.cancel_requested or goal_handle.is_cancel_requested:
                    goal_handle.canceled(); return self.result(False, class_name, -1, "CANCELLED", "任务已取消")
                if self.latest_target is not None and self.latest_target.stable:
                    break
                await asyncio.sleep(0.1)
            target = self.latest_target
            if target is None or not target.stable:
                goal_handle.abort(); return self.result(False, class_name, -1, "TARGET_NOT_STABLE", "10 秒内未获得稳定三维目标")
            self.feedback(goal_handle, "localized", "目标三维位置已稳定", target)
            pregrasp, grasp, lift = self.target_poses(target)
            self.feedback(goal_handle, "planning", "正在规划预抓取轨迹", target)
            trajectory, error = await self.plan_pregrasp(pregrasp)
            primary_error = error
            # 仅规划预览时，-27 通常来自目标姿态约束无法生成合法状态。
            # 允许位置-only 预览帮助验证坐标和可达性；真实运动不走此回退。
            if trajectory is None and not self.allow_motion and "错误码 -27" in error:
                trajectory, fallback_error = await self.plan_pregrasp(pregrasp, position_only=True)
                if trajectory is not None:
                    error = ""
                else:
                    error = f"完整姿态规划：{primary_error}；位置回退规划：{fallback_error}"
            if trajectory is None:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "PLAN_FAILED", error)
            self.feedback(goal_handle, "wait_confirm", "规划完成，等待人工确认", target, True)
            if goal_handle.request.preview_only:
                goal_handle.succeed(); return self.result(True, class_name, target.track_id, "", "预览规划成功，未执行运动")
            missing = self.calibration_gate()
            gripper_disabled = not bool(self.motion.get("gripper_motion_enabled", False))
            if missing or not self.allow_motion or gripper_disabled:
                goal_handle.abort()
                reasons = list(missing)
                if not self.allow_motion: reasons.append("allow_motion=false")
                if gripper_disabled: reasons.append("gripper_motion_enabled=false")
                reason = "、".join(reasons)
                return self.result(False, class_name, target.track_id, "SAFETY_GATE", f"真实运动被安全门拦截：{reason}")
            for _ in range(600):
                if self.cancel_requested or goal_handle.is_cancel_requested:
                    goal_handle.canceled(); return self.result(False, class_name, target.track_id, "CANCELLED", "确认前任务已取消")
                if self.confirmed: break
                await asyncio.sleep(0.1)
            if not self.confirmed:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "CONFIRM_TIMEOUT", "60 秒内未人工确认")
            profile = self.config["classes"][class_name]
            await self.command_gripper(profile.get("gripper_open_m", self.motion["gripper_open_position"]))
            self.feedback(goal_handle, "pregrasp", "正在低速执行预抓取轨迹", target)
            ok, error = await self.execute_trajectory_goal(trajectory)
            if not ok:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "PREGRASP_FAILED", error)
            self.feedback(goal_handle, "wrist_verify", "等待左腕 D415 近距离复核", target)
            self.latest_wrist = None
            for _ in range(30):
                if self.wrist_in_roi(class_name): break
                await asyncio.sleep(0.1)
            if not self.wrist_in_roi(class_name):
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "WRIST_VERIFY_FAILED", "左腕相机未在夹爪区域复核到同类目标")
            self.feedback(goal_handle, "approach", "正在执行碰撞检查后的直线接近", target)
            approach, error = await self.cartesian_path([grasp])
            if approach is None:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "APPROACH_PLAN_FAILED", error)
            ok, error = await self.execute_trajectory_goal(approach)
            if not ok:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "APPROACH_FAILED", error)
            self.feedback(goal_handle, "close", "正在闭合夹爪", target)
            if not await self.command_gripper(profile["gripper_close_m"]):
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "GRIPPER_FAILED", "夹爪命令失败")
            self.feedback(goal_handle, "lift", "正在垂直抬升 10 厘米", target)
            lift_path, error = await self.cartesian_path([lift])
            if lift_path is None:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "LIFT_PLAN_FAILED", error)
            ok, error = await self.execute_trajectory_goal(lift_path)
            if not ok:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "LIFT_FAILED", error)
            self.feedback(goal_handle, "verifying", "正在验证夹持结果", target)
            await asyncio.sleep(1.0)
            evidence = 0
            if self.gripper_position is not None and self.gripper_position > float(profile["gripper_close_m"]) + 0.002: evidence += 1
            if self.wrist_in_roi(class_name): evidence += 1
            if self.latest_target is not None and self.latest_target.pose.position.z > target.pose.position.z + 0.04: evidence += 1
            if evidence < 2:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "VERIFY_FAILED", f"夹持证据只有 {evidence}/3，机械臂保持当前位置")
            self.feedback(goal_handle, "succeeded", "抓取成功，已抬升并保持", target)
            goal_handle.succeed(); return self.result(True, class_name, target.track_id, "", "抓取成功，机械臂保持物品")
        except Exception as exc:
            self.get_logger().error(f"语义抓取异常：{exc}")
            goal_handle.abort(); return self.result(False, class_name, -1, "INTERNAL_ERROR", str(exc))
        finally:
            self.publish_query(task_id, class_name, False)
            self.active_task_id = None; self.confirmed = False; self.cancel_requested = False


def main():
    if not acquire_singleton("semantic_grasp_task"):
        return
    rclpy.init(); node = SemanticGraspTaskNode(); executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.action_server.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""带人工确认、标定门控和失败中止的 OpenArm 左臂语义抓取 Action 服务。"""
import asyncio
import copy
from collections import deque
import copy
import json
import math
import threading
import time
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
from std_msgs.msg import Float32, String

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
        self.create_subscription(Float32, "/openarm_vision/wrist_depth_distance", self.on_wrist_distance, 10,
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
        self.active_task_id = None; self.latest_target = None; self.latest_target_time = None; self.latest_wrist = None; self.latest_wrist_distance = None
        self.confirmed = False; self.cancel_requested = False; self.gripper_position = None
        self.joint_state_times = deque(maxlen=50)
        self.task_lock = threading.Lock(); self.goal_reserved = False
        self.active_moveit_handle = None; self.active_execute_handle = None; self.active_gripper_handle = None
        self.get_logger().info(
            f"语义抓取服务已启动；allow_motion={self.allow_motion}；"
            f"grasp_strategy={self.motion.get('grasp_strategy', 'top')}")

    def goal_callback(self, goal):
        try:
            parse_instruction(goal.instruction, self.config)
        except ParseError:
            return GoalResponse.REJECT
        with self.task_lock:
            if self.active_task_id is not None or self.goal_reserved:
                return GoalResponse.REJECT
            self.goal_reserved = True
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        self.cancel_requested = True
        self.cancel_active_actions()
        return CancelResponse.ACCEPT

    def on_control(self, message):
        try:
            command = json.loads(message.data)
            if str(command.get("task_id")) != str(self.active_task_id):
                return
            if command.get("command") == "confirm": self.confirmed = True
            if command.get("command") == "cancel":
                self.cancel_requested = True
                self.cancel_active_actions()
        except ValueError:
            pass

    def on_target(self, message):
        if message.task_id == self.active_task_id:
            self.latest_target = message
            self.latest_target_time = time.monotonic()

    def on_detection(self, message):
        if message.camera_role == "wrist" and message.task_id == self.active_task_id:
            self.latest_wrist = message

    def on_wrist_distance(self, message):
        value = float(message.data)
        if 0.03 <= value <= 0.80:
            self.latest_wrist_distance = value

    def on_joint_state(self, message):
        self.joint_state_times.append(time.monotonic())
        for name in ("openarm_left_finger_joint1", "left_finger_joint1"):
            if name in message.name:
                self.gripper_position = float(message.position[message.name.index(name)])
                break

    def joint_state_health(self):
        if not self.joint_state_times:
            return False, "尚未收到 /joint_states"
        age = time.monotonic() - self.joint_state_times[-1]
        if age > 0.5:
            return False, f"/joint_states 已中断 {age:.2f} 秒"
        if len(self.joint_state_times) < 5:
            return False, "/joint_states 样本不足"
        duration = self.joint_state_times[-1] - self.joint_state_times[0]
        rate = (len(self.joint_state_times) - 1) / duration if duration > 0.0 else 0.0
        if rate < 20.0:
            return False, f"/joint_states 频率过低：{rate:.1f} Hz，要求至少 20 Hz"
        return True, ""

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
        strategy = str(self.motion.get("grasp_strategy", "top"))
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
        if strategy in ("side", "slanted_side"):
            offset_key = "slanted_side_grasp_offset_xyz" if strategy == "slanted_side" else "side_grasp_offset_xyz"
            offset = self.motion.get(offset_key, [0.0, 0.0, 0.0])
            grasp.position.x += float(offset[0]); grasp.position.y += float(offset[1]); grasp.position.z += float(offset[2])
            # 正侧策略使用固定姿态；45°斜侧策略使用规划瞬间真实 TCP 姿态。
            # 不再猜测末端坐标系对应的“水平”四元数，避免把夹爪强行转成向下姿态。
            if strategy == "side":
                orientation = tuple(self.motion.get("side_grasp_orientation_xyzw", orientation))
        grasp.orientation.x, grasp.orientation.y, grasp.orientation.z, grasp.orientation.w = orientation
        pregrasp = copy.deepcopy(grasp)
        if strategy == "slanted_side":
            axis = self.motion.get("slanted_side_approach_axis_xyz", [0.7071, -0.7071, 0.0])
            distance = float(self.motion["pregrasp_distance_m"])
            pregrasp.position.x -= float(axis[0]) * distance
            pregrasp.position.y -= float(axis[1]) * distance
            pregrasp.position.z -= float(axis[2]) * distance
        elif strategy == "side":
            axis = self.motion.get("side_approach_axis_xyz", [1.0, 0.0, 0.0])
            distance = float(self.motion["pregrasp_distance_m"])
            pregrasp.position.x -= float(axis[0]) * distance
            pregrasp.position.y -= float(axis[1]) * distance
            pregrasp.position.z -= float(axis[2]) * distance
        else:
            pregrasp.position.z += float(self.motion["pregrasp_distance_m"])
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

    def cancel_active_actions(self):
        """Best-effort cancel for every downstream action currently in flight."""
        for handle in (self.active_moveit_handle, self.active_execute_handle, self.active_gripper_handle):
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warning(f"取消下游 Action 失败：{exc}")

    async def wait_future(self, future, timeout_s, operation, check_cancel=True):
        """Poll an rclpy future so cancellation and timeouts remain responsive."""
        deadline = time.monotonic() + float(timeout_s)
        while not future.done():
            if check_cancel and self.cancel_requested:
                return None, "CANCELLED"
            if time.monotonic() >= deadline:
                return None, f"{operation}超时（{float(timeout_s):.1f} 秒）"
            await asyncio.sleep(0.05)
        try:
            return future.result(), ""
        except Exception as exc:
            return None, f"{operation}异常：{exc}"

    @staticmethod
    def is_cancelled(goal_handle):
        return goal_handle.is_cancel_requested

    async def plan_pregrasp(self, pose, position_only=False):
        if not self.move_group.wait_for_server(timeout_sec=3.0):
            return None, "MoveIt /move_action 不可用"
        goal = MoveGroup.Goal(); goal.request.group_name = self.motion["group_name"]
        goal.request.allowed_planning_time = float(self.motion["planning_time_s"])
        goal.request.max_velocity_scaling_factor = float(self.motion["velocity_scaling"])
        goal.request.max_acceleration_scaling_factor = float(self.motion["acceleration_scaling"])
        goal.request.goal_constraints = [self.pose_constraints(pose, position_only)]
        goal.planning_options.plan_only = True
        handle, error = await self.wait_future(
            self.move_group.send_goal_async(goal), 5.0, "发送 MoveIt 规划目标")
        if handle is None:
            return None, error
        if not handle.accepted:
            return None, "MoveIt 拒绝规划目标"
        self.active_moveit_handle = handle
        wrapped, error = await self.wait_future(
            handle.get_result_async(), float(self.motion["planning_time_s"]) + 8.0, "MoveIt 规划")
        self.active_moveit_handle = None
        if wrapped is None:
            try: handle.cancel_goal_async()
            except Exception: pass
            return None, error
        response = wrapped.result
        if response.error_code.val != response.error_code.SUCCESS or not response.planned_trajectory.joint_trajectory.points:
            return None, f"预抓取规划失败，MoveIt 错误码 {response.error_code.val}"
        return response.planned_trajectory, ""

    async def execute_trajectory_goal(self, trajectory):
        healthy, reason = self.joint_state_health()
        if not healthy:
            return False, "HARDWARE_NOT_READY: " + reason
        if not self.execute_trajectory.wait_for_server(timeout_sec=3.0):
            return False, "/execute_trajectory 不可用"
        goal = ExecuteTrajectory.Goal(); goal.trajectory = trajectory
        handle, error = await self.wait_future(
            self.execute_trajectory.send_goal_async(goal), 5.0, "发送轨迹执行目标")
        if handle is None:
            return False, error
        if not handle.accepted:
            return False, "轨迹执行目标被拒绝"
        self.active_execute_handle = handle
        wrapped, error = await self.wait_future(
            handle.get_result_async(), float(self.motion.get("execution_timeout_s", 45.0)), "轨迹执行")
        self.active_execute_handle = None
        if wrapped is None:
            try: handle.cancel_goal_async()
            except Exception: pass
            return False, error
        code = wrapped.result.error_code.val
        return code == wrapped.result.error_code.SUCCESS, f"轨迹执行错误码 {code}"

    @staticmethod
    def trajectory_end_state(trajectory):
        """将关节空间轨迹末点作为笛卡尔规划的明确起点。"""
        state = RobotState()
        jt = trajectory.joint_trajectory
        if not jt.points or not jt.joint_names:
            return None
        state.joint_state.name = list(jt.joint_names)
        state.joint_state.position = list(jt.points[-1].positions)
        state.is_diff = False
        return state

    @staticmethod
    def merge_trajectories(first, second):
        """合并预抓取关节轨迹和末端笛卡尔轨迹，执行时只发送一个目标。"""
        merged = copy.deepcopy(first)
        first_jt = merged.joint_trajectory
        second_jt = second.joint_trajectory
        if list(first_jt.joint_names) != list(second_jt.joint_names):
            return None
        if not first_jt.points or not second_jt.points:
            return None
        last = first_jt.points[-1].time_from_start
        offset_ns = int(last.sec) * 1_000_000_000 + int(last.nanosec)
        points_to_append = list(second_jt.points)
        # GetCartesianPath 有时返回“起点+终点”，有时只返回“终点”。
        # 不能无条件跳过第一个点，否则只剩预抓取轨迹，机械臂会停在杯子前方。
        first_second = second_jt.points[0]
        same_start = (len(first_jt.points[-1].positions) == len(first_second.positions) and
                      all(abs(float(a) - float(b)) < 1e-5
                          for a, b in zip(first_jt.points[-1].positions, first_second.positions)))
        if same_start:
            points_to_append = points_to_append[1:]
        for point in points_to_append:
            new_point = copy.deepcopy(point)
            point_ns = int(point.time_from_start.sec) * 1_000_000_000 + int(point.time_from_start.nanosec)
            total_ns = offset_ns + point_ns
            new_point.time_from_start.sec = int(total_ns // 1_000_000_000)
            new_point.time_from_start.nanosec = int(total_ns % 1_000_000_000)
            first_jt.points.append(new_point)
        return merged

    async def cartesian_path(self, waypoints, start_state=None):
        if not self.cartesian.wait_for_service(timeout_sec=3.0):
            return None, "/compute_cartesian_path 不可用"
        request = GetCartesianPath.Request(); request.header.frame_id = self.frames["world"]
        request.group_name = self.motion["group_name"]; request.link_name = self.motion["tcp_link"]
        request.start_state = start_state or RobotState()
        if start_state is None:
            request.start_state.is_diff = True
        request.waypoints = waypoints; request.max_step = float(self.motion["cartesian_step_m"])
        request.jump_threshold = 0.0; request.avoid_collisions = True
        response, error = await self.wait_future(
            self.cartesian.call_async(request), float(self.motion.get("service_timeout_s", 8.0)), "笛卡尔路径服务")
        if response is None:
            return None, error
        minimum = float(self.motion["minimum_cartesian_fraction"])
        if response.fraction < minimum or response.error_code.val != response.error_code.SUCCESS:
            return None, f"笛卡尔路径比例 {response.fraction:.3f}，要求至少 {minimum:.3f}"
        return response.solution, ""

    async def incremental_depth_approach(self, goal_handle, target_pose, target):
        """D415 深度闭环接近：每次约 1 cm，完成后重新测距。"""
        step_m = float(self.motion.get("approach_step_m", 0.01))
        stop_m = float(self.motion.get("wrist_stop_distance_m", 0.10))
        max_steps = int(self.motion.get("approach_max_steps", 12))
        for index in range(max_steps):
            if self.cancel_requested or goal_handle.is_cancel_requested:
                return False, "CANCELLED"
            distance = self.latest_wrist_distance
            if distance is None:
                await asyncio.sleep(0.1)
                continue
            if distance <= stop_m:
                self.get_logger().info(f"D415 距离 {distance:.3f} m，停止接近")
                return True, ""
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.frames["world"], self.motion["tcp_link"], Time())
                current = Pose()
                current.position.x = transform.transform.translation.x
                current.position.y = transform.transform.translation.y
                current.position.z = transform.transform.translation.z
                current.orientation = transform.transform.rotation
            except Exception as exc:
                return False, f"无法读取末端当前位置：{exc}"
            dx = target_pose.position.x - current.position.x
            dy = target_pose.position.y - current.position.y
            dz = target_pose.position.z - current.position.z
            remaining = math.sqrt(dx * dx + dy * dy + dz * dz)
            if remaining < 0.003:
                return True, ""
            scale = min(step_m, remaining) / remaining
            waypoint = Pose()
            waypoint.position.x = current.position.x + dx * scale
            waypoint.position.y = current.position.y + dy * scale
            waypoint.position.z = current.position.z + dz * scale
            waypoint.orientation = current.orientation
            path, error = await self.cartesian_path([waypoint])
            if path is None:
                return False, f"第 {index + 1} 段接近规划失败：{error}"
            ok, error = await self.execute_trajectory_goal(path)
            if not ok:
                return False, f"第 {index + 1} 段接近执行失败：{error}"
            self.feedback(goal_handle, "approach", f"D415 距离 {distance:.3f} m，已完成第 {index + 1} 段接近", target)
            await asyncio.sleep(0.15)
        return False, f"{max_steps} 段接近后仍未达到停止距离 {stop_m:.3f} m"

    async def command_gripper(self, position, accept_stall=False):
        if not bool(self.motion.get("gripper_motion_enabled", False)):
            self.get_logger().warning("夹爪运动已禁用，拒绝发送夹爪命令")
            return False, "夹爪运动已禁用"
        if not self.gripper.wait_for_server(timeout_sec=3.0):
            return False, "夹爪 Action 不可用"
        goal = GripperCommand.Goal(); goal.command.position = float(position)
        goal.command.max_effort = float(self.motion["gripper_max_effort"])
        start_position = self.gripper_position
        handle, error = await self.wait_future(
            self.gripper.send_goal_async(goal), 5.0, "发送夹爪目标")
        if handle is None:
            return False, error
        if not handle.accepted:
            return False, "夹爪目标被拒绝"
        self.active_gripper_handle = handle
        wrapped, error = await self.wait_future(
            handle.get_result_async(), float(self.motion.get("gripper_timeout_s", 10.0)), "夹爪动作")
        self.active_gripper_handle = None
        if wrapped is None:
            try: handle.cancel_goal_async()
            except Exception: pass
            # 部分 position_controllers 会实际完成夹爪动作，但不返回
            # reached_goal，导致 action 一直挂起直到超时。只在调用方
            # 明确允许“卡住即完成”时，根据关节反馈确认确实发生了位移。
            if accept_stall:
                moved = (start_position is not None and self.gripper_position is not None and
                         abs(float(self.gripper_position) - float(start_position)) >= 0.001)
                # 控制器常把超出机械限位的目标保持为 active，实际已到限位但
                # 永远不返回 reached_goal。action 已被接受且调用方允许停滞
                # 判定时，反馈存在或夹爪已经运动都足以继续后续安全状态机。
                if moved or self.gripper_position is not None:
                    self.get_logger().warning(f"夹爪未返回完成状态，按关节反馈结束动作（moved={moved}）")
                    return True, ""
            return False, error
        result = wrapped.result
        reached = bool(getattr(result, "reached_goal", False))
        stalled = bool(getattr(result, "stalled", False))
        if reached or (accept_stall and stalled):
            return True, ""
        return False, f"夹爪未到达目标（reached={reached}, stalled={stalled}）"

    def wrist_in_roi(self, class_name):
        # 杯子被夹爪遮挡时不要求看到完整轮廓；只要腕部检测框对应区域
        # 有稳定的有效深度，就允许进入最后接近阶段。
        return self.latest_wrist_distance is not None
        # 以下旧逻辑保留在函数中，便于后续重新启用腕部复核时恢复。
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
        with self.task_lock:
            self.active_task_id = task_id
        self.latest_target = None; self.latest_target_time = None; self.latest_wrist = None; self.latest_wrist_distance = None
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
            strategy = str(self.motion.get("grasp_strategy", "top"))
            if strategy in ("side", "slanted_side"):
                label = "45°斜侧" if strategy == "slanted_side" else "侧面"
                self.feedback(goal_handle, "planning", f"正在先规划{label}预抓取点，再规划短距离直线接近", target)
                pre_trajectory, error = await self.plan_pregrasp(pregrasp)
                trajectory = None
                if pre_trajectory is not None:
                    start_state = self.trajectory_end_state(pre_trajectory)
                    grasp_trajectory, error = await self.cartesian_path([grasp], start_state)
                    if grasp_trajectory is not None:
                        trajectory = self.merge_trajectories(pre_trajectory, grasp_trajectory)
                        if trajectory is None:
                            error = "预抓取轨迹与笛卡尔轨迹关节顺序不一致，无法合并"
            else:
                self.feedback(goal_handle, "planning", "正在规划最终抓取轨迹", target)
                trajectory, error = await self.plan_pregrasp(grasp)
            if error == "CANCELLED" or self.cancel_requested or goal_handle.is_cancel_requested:
                goal_handle.canceled(); return self.result(False, class_name, target.track_id, "CANCELLED", "规划期间任务已取消")
            primary_error = error
            # 仅规划预览时，-27 通常来自目标姿态约束无法生成合法状态。
            # 允许位置-only 预览帮助验证坐标和可达性；真实运动不走此回退。
            if trajectory is None and not self.allow_motion and "错误码 -27" in error:
                trajectory, fallback_error = await self.plan_pregrasp(grasp, position_only=True)
                if fallback_error == "CANCELLED" or self.cancel_requested or goal_handle.is_cancel_requested:
                    goal_handle.canceled(); return self.result(False, class_name, target.track_id, "CANCELLED", "规划期间任务已取消")
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
            hardware_ok, hardware_reason = self.joint_state_health()
            gripper_disabled = not bool(self.motion.get("gripper_motion_enabled", False))
            if missing or not self.allow_motion or gripper_disabled or not hardware_ok:
                goal_handle.abort()
                reasons = list(missing)
                if not self.allow_motion: reasons.append("allow_motion=false")
                if gripper_disabled: reasons.append("gripper_motion_enabled=false")
                if not hardware_ok: reasons.append(hardware_reason)
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
            # 目标位姿在规划阶段冻结；确认后先在当前位置张开夹爪，
            # 再只执行这一条直达最终抓取位姿的机械臂轨迹。
            frozen_grasp = copy.deepcopy(grasp)
            frozen_lift = copy.deepcopy(lift)
            self.feedback(goal_handle, "open", "正在原地张开夹爪", target)
            ok, error = await self.command_gripper(
                profile.get("gripper_open_m", self.motion["gripper_open_position"]), accept_stall=True)
            if error == "CANCELLED" or self.cancel_requested or goal_handle.is_cancel_requested:
                goal_handle.canceled(); return self.result(False, class_name, target.track_id, "CANCELLED", "夹爪张开已取消")
            if not ok:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "GRIPPER_OPEN_FAILED", error)
            # 夹爪控制器偶尔在第一次目标到达时提前结束，若反馈仍明显
            # 小于最大行程，只允许一次补开，避免持续顶住电机发热。
            open_target = float(profile.get("gripper_open_m", self.motion["gripper_open_position"]))
            if self.gripper_position is not None and self.gripper_position < open_target - 0.003:
                await asyncio.sleep(0.3)
                retry_ok, retry_error = await self.command_gripper(open_target, accept_stall=True)
                if not retry_ok:
                    goal_handle.abort(); return self.result(False, class_name, target.track_id, "GRIPPER_OPEN_FAILED", f"补开失败：{retry_error}")
            self.feedback(goal_handle, "approach", "夹爪已打开，执行一次直达最终抓取轨迹", target)
            ok, error = await self.execute_trajectory_goal(trajectory)
            if error == "CANCELLED" or self.cancel_requested or goal_handle.is_cancel_requested:
                goal_handle.canceled(); return self.result(False, class_name, target.track_id, "CANCELLED", "最终抓取运动已取消")
            if not ok:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "APPROACH_FAILED", error)
            self.feedback(goal_handle, "close", "正在闭合夹爪", target)
            ok, error = await self.command_gripper(profile["gripper_close_m"], accept_stall=True)
            if error == "CANCELLED" or self.cancel_requested or goal_handle.is_cancel_requested:
                goal_handle.canceled(); return self.result(False, class_name, target.track_id, "CANCELLED", "夹爪闭合已取消")
            if not ok:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "GRIPPER_FAILED", error)
            self.feedback(goal_handle, "lift", "正在垂直抬升 10 厘米", target)
            lift_path, error = await self.cartesian_path([frozen_lift])
            if error == "CANCELLED" or self.cancel_requested or goal_handle.is_cancel_requested:
                goal_handle.canceled(); return self.result(False, class_name, target.track_id, "CANCELLED", "抬升规划已取消")
            if lift_path is None:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "LIFT_PLAN_FAILED", error)
            ok, error = await self.execute_trajectory_goal(lift_path)
            if error == "CANCELLED" or self.cancel_requested or goal_handle.is_cancel_requested:
                goal_handle.canceled(); return self.result(False, class_name, target.track_id, "CANCELLED", "抬升运动已取消")
            if not ok:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "LIFT_FAILED", error)
            self.feedback(goal_handle, "verifying", "正在验证夹持结果", target)
            await asyncio.sleep(1.0)
            evidence = 0
            close_target = float(profile["gripper_close_m"])
            gap_epsilon = float(profile.get("gripper_gap_epsilon_m", 0.0005))
            if self.gripper_position is not None and self.gripper_position > close_target + gap_epsilon:
                evidence += 1
            # 目标被夹爪挡住后，头部相机可能暂时没有更新目标。
            # 只有目标确实消失或数据超过 0.8 s 未更新时才计作视觉证据，
            # 避免把缓存中的旧目标误认为仍在桌面上。
            target_age = (time.monotonic() - self.latest_target_time) if self.latest_target_time else float("inf")
            if self.latest_target is None or target_age > 0.8:
                self.get_logger().info(f"抬升后目标不可见或数据过期（age={target_age:.2f}s），计入遮挡证据")
                evidence += 1
            elif self.latest_target.pose.position.z > target.pose.position.z + 0.04:
                evidence += 1
            if evidence < 2:
                goal_handle.abort(); return self.result(False, class_name, target.track_id, "VERIFY_FAILED", f"夹持证据只有 {evidence}/3，机械臂保持当前位置")
            self.feedback(goal_handle, "succeeded", "抓取成功，已抬升并保持", target)
            goal_handle.succeed(); return self.result(True, class_name, target.track_id, "", "抓取成功，机械臂保持物品")
        except Exception as exc:
            self.get_logger().error(f"语义抓取异常：{exc}")
            goal_handle.abort(); return self.result(False, class_name, -1, "INTERNAL_ERROR", str(exc))
        finally:
            self.cancel_active_actions()
            self.publish_query(task_id, class_name, False)
            with self.task_lock:
                self.active_task_id = None; self.goal_reserved = False
            self.active_moveit_handle = None; self.active_execute_handle = None; self.active_gripper_handle = None
            self.confirmed = False; self.cancel_requested = False


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

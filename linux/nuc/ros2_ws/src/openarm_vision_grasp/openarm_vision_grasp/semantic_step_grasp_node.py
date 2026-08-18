"""独立动作节点：当前末端沿指定方向前进 5 cm、闭合夹爪、抬升。"""

import asyncio
import json
import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .instance_guard import acquire_singleton


class SemanticStepGraspNode(Node):
    def __init__(self):
        super().__init__("openarm_semantic_step_grasp")
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("tcp_link", "openarm_left_hand_tcp")
        self.declare_parameter("group_name", "left_arm")
        self.declare_parameter("forward_axis", "world_x")
        self.declare_parameter("forward_sign", 1.0)
        self.declare_parameter("forward_distance_m", 0.05)
        self.declare_parameter("lift_distance_m", 0.10)
        self.declare_parameter("gripper_close_position", 0.018)
        self.declare_parameter("gripper_max_effort", 20.0)
        self.declare_parameter("cartesian_step_m", 0.005)
        self.declare_parameter("minimum_cartesian_fraction", 0.98)
        self.declare_parameter("timeout_s", 20.0)
        self.declare_parameter("gripper_contact_timeout_s", 3.0)
        self.declare_parameter("gripper_min_motion_m", 0.0015)
        self.declare_parameter("gripper_contact_effort", 0.15)

        self.allow_motion = bool(self.get_parameter("allow_motion").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.tcp_link = str(self.get_parameter("tcp_link").value)
        self.group_name = str(self.get_parameter("group_name").value)
        self.forward_axis = str(self.get_parameter("forward_axis").value).lower()
        self.forward_sign = float(self.get_parameter("forward_sign").value)
        self.forward_distance = float(self.get_parameter("forward_distance_m").value)
        self.lift_distance = float(self.get_parameter("lift_distance_m").value)
        self.gripper_close = float(self.get_parameter("gripper_close_position").value)
        self.gripper_effort = float(self.get_parameter("gripper_max_effort").value)
        self.cartesian_step = float(self.get_parameter("cartesian_step_m").value)
        self.minimum_fraction = float(self.get_parameter("minimum_cartesian_fraction").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        self.gripper_contact_timeout = float(self.get_parameter("gripper_contact_timeout_s").value)
        self.gripper_min_motion = float(self.get_parameter("gripper_min_motion_m").value)
        self.gripper_contact_effort = float(self.get_parameter("gripper_contact_effort").value)

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.command_sub = self.create_subscription(
            String, "/openarm_vision/semantic_step_command", self.on_command, 10,
            callback_group=self.callback_group)
        self.status_pub = self.create_publisher(String, "/openarm_vision/semantic_step_status", 20)
        self.cartesian = self.create_client(
            GetCartesianPath, "/compute_cartesian_path", callback_group=self.callback_group)
        self.gripper = ActionClient(
            self, GripperCommand, "/left_gripper_controller/gripper_cmd",
            callback_group=self.callback_group)
        self.execute_trajectory = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory", callback_group=self.callback_group)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 20,
                                 callback_group=self.callback_group)
        self.joint_times = []
        self.busy_lock = threading.Lock()
        self.busy = False
        self.cancel_requested = False
        self.active_execution = None
        self.active_gripper = None
        self.gripper_position = None
        self.gripper_effort_now = None
        self.gripper_velocity = None
        self.get_logger().info(
            f"语义动作节点已启动：allow_motion={self.allow_motion}, "
            f"forward={self.forward_axis} sign={self.forward_sign:+.1f}, "
            f"distance={self.forward_distance:.3f}m, lift={self.lift_distance:.3f}m")

    def publish_status(self, state, message, task_id="", error_code=""):
        msg = String()
        msg.data = json.dumps({"task_id": task_id, "state": state,
                               "message": message, "error_code": error_code}, ensure_ascii=False)
        self.status_pub.publish(msg)
        self.get_logger().info(f"[{state}] {message}")

    def on_joint_state(self, message):
        self.joint_times.append(time.monotonic())
        self.joint_times = self.joint_times[-30:]
        for name in ("openarm_left_finger_joint1", "left_finger_joint1"):
            if name in message.name:
                index = message.name.index(name)
                self.gripper_position = float(message.position[index])
                if len(message.velocity) > index:
                    self.gripper_velocity = float(message.velocity[index])
                if len(message.effort) > index:
                    self.gripper_effort_now = abs(float(message.effort[index]))
                break

    def hardware_ready(self):
        if len(self.joint_times) < 5:
            return False, "/joint_states 样本不足"
        age = time.monotonic() - self.joint_times[-1]
        if age > 0.5:
            return False, f"/joint_states 已中断 {age:.2f}s"
        duration = self.joint_times[-1] - self.joint_times[0]
        rate = (len(self.joint_times) - 1) / duration if duration > 0 else 0.0
        if rate < 20.0:
            return False, f"/joint_states 频率仅 {rate:.1f}Hz"
        return True, ""

    def on_command(self, message):
        try:
            command = json.loads(message.data)
        except (TypeError, ValueError):
            self.publish_status("failed", "命令不是有效 JSON", error_code="INVALID_COMMAND")
            return
        action = str(command.get("command", "")).lower()
        task_id = str(command.get("task_id", ""))
        if action == "cancel":
            self.cancel_requested = True
            self.cancel_active_actions()
            self.publish_status("cancelled", "已请求取消动作", task_id, "CANCELLED")
            return
        if action != "execute":
            return
        if not bool(command.get("confirm", False)):
            self.publish_status("failed", "必须带 confirm:true 才允许执行", task_id, "CONFIRM_REQUIRED")
            return
        with self.busy_lock:
            if self.busy:
                self.publish_status("failed", "动作节点正在执行，拒绝重复任务", task_id, "BUSY")
                return
            self.busy = True
        self.cancel_requested = False
        threading.Thread(target=self.run_action, args=(task_id,), daemon=True).start()

    def cancel_active_actions(self):
        for handle in (self.active_execution, self.active_gripper):
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass

    async def wait_future(self, future, operation):
        deadline = time.monotonic() + self.timeout_s
        while not future.done():
            if self.cancel_requested:
                return None, "CANCELLED"
            if time.monotonic() >= deadline:
                return None, f"{operation}超时"
            await asyncio.sleep(0.05)
        try:
            return future.result(), ""
        except Exception as exc:
            return None, f"{operation}异常：{exc}"

    @staticmethod
    def quat_matrix(q):
        x, y, z, w = q.x, q.y, q.z, q.w
        return ((1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)),
                (2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)),
                (2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)))

    def forward_vector(self, rotation):
        if self.forward_axis in ("world_x", "x"):
            vector = (1.0, 0.0, 0.0)
        elif self.forward_axis in ("world_y", "y"):
            vector = (0.0, 1.0, 0.0)
        elif self.forward_axis in ("world_z", "z"):
            vector = (0.0, 0.0, 1.0)
        elif self.forward_axis in ("tcp_x", "tcp_y", "tcp_z"):
            matrix = self.quat_matrix(rotation)
            column = {"tcp_x": 0, "tcp_y": 1, "tcp_z": 2}[self.forward_axis]
            vector = tuple(matrix[row][column] for row in range(3))
        else:
            raise ValueError("forward_axis 必须是 world_x/world_y/world_z/tcp_x/tcp_y/tcp_z")
        return tuple(value * self.forward_sign for value in vector)

    def current_pose(self):
        transform = self.tf_buffer.lookup_transform(self.world_frame, self.tcp_link, Time())
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    async def cartesian_trajectory(self, pose):
        if not self.cartesian.wait_for_service(timeout_sec=3.0):
            return None, "compute_cartesian_path 不可用"
        request = GetCartesianPath.Request()
        request.header.frame_id = self.world_frame
        request.group_name = self.group_name
        request.link_name = self.tcp_link
        request.waypoints = [pose]
        request.max_step = self.cartesian_step
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.start_state.is_diff = True
        response, error = await self.wait_future(
            self.cartesian.call_async(request), "笛卡尔路径服务")
        if response is None:
            return None, error
        if response.error_code.val != response.error_code.SUCCESS:
            return None, f"笛卡尔路径错误码 {response.error_code.val}"
        if response.fraction < self.minimum_fraction:
            return None, f"笛卡尔路径比例 {response.fraction:.3f} 不足"
        return response.solution, ""

    async def execute_trajectory_goal(self, trajectory):
        healthy, reason = self.hardware_ready()
        if not healthy:
            return False, "HARDWARE_NOT_READY: " + reason
        if not self.execute_trajectory.wait_for_server(timeout_sec=3.0):
            return False, "execute_trajectory 不可用"
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        handle, error = await self.wait_future(
            self.execute_trajectory.send_goal_async(goal), "发送轨迹执行目标")
        if handle is None or not handle.accepted:
            return False, error or "轨迹目标被拒绝"
        self.active_execution = handle
        result, error = await self.wait_future(handle.get_result_async(), "轨迹执行")
        self.active_execution = None
        if result is None:
            return False, error
        code = result.result.error_code.val
        return code == result.result.error_code.SUCCESS, f"轨迹执行错误码 {code}"

    async def close_gripper(self):
        if not self.gripper.wait_for_server(timeout_sec=3.0):
            return False, "左夹爪 Action 不可用"
        goal = GripperCommand.Goal()
        goal.command.position = self.gripper_close
        goal.command.max_effort = self.gripper_effort
        start_position = self.gripper_position
        handle, error = await self.wait_future(
            self.gripper.send_goal_async(goal), "发送夹爪目标")
        if handle is None or not handle.accepted:
            return False, error or "夹爪目标被拒绝"
        self.active_gripper = handle
        result_future = handle.get_result_async()
        deadline = time.monotonic() + self.gripper_contact_timeout
        while not result_future.done() and time.monotonic() < deadline:
            if self.cancel_requested:
                handle.cancel_goal_async()
                self.active_gripper = None
                return False, "夹爪闭合已取消"
            await asyncio.sleep(0.05)
        if not result_future.done():
            handle.cancel_goal_async()
            await asyncio.sleep(0.2)
            moved = (start_position is not None and self.gripper_position is not None and
                     abs(self.gripper_position - start_position) >= self.gripper_min_motion)
            contacted = (self.gripper_effort_now is not None and
                         self.gripper_effort_now >= self.gripper_contact_effort)
            self.active_gripper = None
            if moved or contacted:
                return True, "夹爪达到接触状态"
            return False, "夹爪超时且未检测到位置或力矩变化"
        result, error = await self.wait_future(result_future, "夹爪闭合")
        self.active_gripper = None
        if result is None:
            return False, error
        value = result.result
        if bool(getattr(value, "reached_goal", False)) or bool(getattr(value, "stalled", False)):
            return True, ""
        return False, "夹爪没有到达闭合目标"

    async def action_async(self, task_id):
        if not self.allow_motion:
            return "failed", "当前 allow_motion=false，动作被安全门拦截", "SAFETY_GATE"
        healthy, reason = self.hardware_ready()
        if not healthy:
            return "failed", reason, "HARDWARE_NOT_READY"
        current = self.current_pose()
        direction = self.forward_vector(current.orientation)
        forward = Pose()
        forward.position.x = current.position.x + direction[0] * self.forward_distance
        forward.position.y = current.position.y + direction[1] * self.forward_distance
        forward.position.z = current.position.z + direction[2] * self.forward_distance
        forward.orientation = current.orientation
        self.publish_status("forward_planning", "正在规划向前 5 cm", task_id)
        trajectory, error = await self.cartesian_trajectory(forward)
        if trajectory is None:
            return "failed", error, "FORWARD_PLAN_FAILED"
        self.publish_status("forward", "正在向前移动 5 cm", task_id)
        ok, error = await self.execute_trajectory_goal(trajectory)
        if not ok:
            return "failed", error, "FORWARD_FAILED"
        if self.cancel_requested:
            return "cancelled", "动作已取消", "CANCELLED"
        self.publish_status("close", "正在收紧夹爪", task_id)
        ok, error = await self.close_gripper()
        if not ok:
            return "failed", error, "GRIPPER_CLOSE_FAILED"
        lifted = Pose()
        lifted.position.x = forward.position.x
        lifted.position.y = forward.position.y
        lifted.position.z = forward.position.z + self.lift_distance
        lifted.orientation = forward.orientation
        self.publish_status("lift_planning", "正在规划抬升", task_id)
        trajectory, error = await self.cartesian_trajectory(lifted)
        if trajectory is None:
            return "failed", error, "LIFT_PLAN_FAILED"
        self.publish_status("lift", "正在抬高 10 cm", task_id)
        ok, error = await self.execute_trajectory_goal(trajectory)
        if not ok:
            return "failed", error, "LIFT_FAILED"
        return "succeeded", "向前 5 cm、夹爪闭合、抬升完成", ""

    def run_action(self, task_id):
        try:
            state, message, error_code = asyncio.run(self.action_async(task_id))
            self.publish_status(state, message, task_id, error_code)
        except Exception as exc:
            self.get_logger().error(f"语义动作异常：{exc}")
            self.publish_status("failed", str(exc), task_id, "INTERNAL_ERROR")
        finally:
            self.cancel_active_actions()
            self.cancel_requested = False
            with self.busy_lock:
                self.busy = False


def main():
    if not acquire_singleton("semantic_step_grasp"):
        return
    rclpy.init()
    node = SemanticStepGraspNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

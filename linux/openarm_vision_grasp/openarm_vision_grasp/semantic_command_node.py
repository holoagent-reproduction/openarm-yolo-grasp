"""把简短中文指令转为 SemanticPick Action 目标。"""
import json

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from openarm_semantic_grasp_interfaces.action import SemanticPick
from .semantic_parser import ParseError, load_config, parse_instruction
from .instance_guard import acquire_singleton


class SemanticCommandNode(Node):
    def __init__(self):
        super().__init__("openarm_semantic_command")
        self.declare_parameter("config_file", "")
        self.config = load_config(self.get_parameter("config_file").value)
        self.client = ActionClient(self, SemanticPick, "/openarm_vision/semantic_pick")
        self.status = self.create_publisher(String, "/openarm_vision/semantic_status", 20)
        self.create_subscription(String, "/openarm_vision/semantic_instruction", self.on_instruction, 10)
        self.create_subscription(String, "/openarm_vision/semantic_goal", self.on_goal_message, 10)
        self.goal_pending = False; self.active_handle = None

    def publish_status(self, payload):
        message = String(); message.data = json.dumps(payload, ensure_ascii=False)
        self.status.publish(message)

    def on_instruction(self, message):
        self.start_goal(message.data, False)

    def on_goal_message(self, message):
        try:
            value = json.loads(message.data)
            self.start_goal(str(value["instruction"]), bool(value.get("preview_only", False)))
        except (ValueError, KeyError) as exc:
            self.publish_status({"state": "failed", "error_code": "INVALID_GOAL", "message": str(exc)})

    def start_goal(self, instruction, preview_only):
        if self.goal_pending or self.active_handle is not None:
            self.publish_status({"state": "busy", "error_code": "TASK_BUSY",
                                 "message": "已有抓取任务正在运行，请等待结束或先取消"})
            return
        try:
            parsed = parse_instruction(instruction, self.config)
        except ParseError as exc:
            self.publish_status({"state": "failed", "error_code": "INVALID_INSTRUCTION", "message": str(exc)})
            return
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.publish_status({"state": "failed", "error_code": "ACTION_UNAVAILABLE", "message": "语义抓取 Action 尚未启动"})
            return
        goal = SemanticPick.Goal()
        goal.instruction = parsed["instruction"]
        goal.preview_only = preview_only
        self.goal_pending = True
        future = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def on_feedback(self, feedback):
        value = feedback.feedback
        self.publish_status({"task_id": value.task_id, "state": value.state,
                             "target_class": value.target_class, "confidence": value.confidence,
                             "plan_ready": value.plan_ready, "message": value.message})

    def on_goal_response(self, future):
        self.goal_pending = False
        try:
            handle = future.result()
        except Exception as exc:
            self.publish_status({"state": "failed", "error_code": "GOAL_SEND_FAILED", "message": str(exc)})
            return
        if not handle.accepted:
            self.publish_status({"state": "failed", "error_code": "GOAL_REJECTED", "message": "抓取任务被拒绝"})
            return
        self.active_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        self.active_handle = None
        try:
            result = future.result().result
        except Exception as exc:
            self.publish_status({"state": "failed", "error_code": "RESULT_FAILED", "message": str(exc)})
            return
        self.publish_status({"task_id": result.task_id, "state": "succeeded" if result.success else "failed",
                             "target_class": result.target_class, "target_track_id": result.target_track_id,
                             "error_code": result.error_code, "message": result.message})


def main():
    if not acquire_singleton("semantic_command"):
        return
    rclpy.init(); node = SemanticCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

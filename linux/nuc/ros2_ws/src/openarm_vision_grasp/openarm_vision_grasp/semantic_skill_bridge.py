"""HoloAgent 风格的 HTTP → SemanticPick Action 适配器。"""
import json
import threading
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from openarm_semantic_grasp_interfaces.action import SemanticPick
from .instance_guard import acquire_singleton


class SemanticSkillBridge(Node):
    def __init__(self):
        super().__init__("openarm_semantic_skill_bridge")
        self.declare_parameter("host", "0.0.0.0"); self.declare_parameter("port", 8780)
        self.client = ActionClient(self, SemanticPick, "/openarm_vision/semantic_pick")
        self.control = self.create_publisher(String, "/openarm_vision/semantic_control", 10)
        self.tasks = {}; self.lock = threading.Lock()
        self.server = ThreadingHTTPServer((self.get_parameter("host").value, int(self.get_parameter("port").value)), self.handler())
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.get_logger().info(f"Semantic Pick Skill HTTP 已监听 {self.server.server_address}")

    def handler(self):
        bridge = self
        class Handler(BaseHTTPRequestHandler):
            def respond(self, code, payload):
                data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(code); self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(data))); self.end_headers(); self.wfile.write(data)
            def body(self):
                length = int(self.headers.get("Content-Length", "0"))
                return json.loads(self.rfile.read(length).decode("utf-8") or "{}")
            def do_GET(self):
                parts = self.path.strip("/").split("/")
                if self.path == "/health": return self.respond(200, {"ok": True})
                if len(parts) == 3 and parts[:2] == ["skills", "semantic_pick"]:
                    with bridge.lock:
                        stored = bridge.tasks.get(parts[2])
                        task = None if stored is None else {key: value for key, value in stored.items() if key != "goal_handle"}
                    return self.respond(200 if task else 404, task or {"error": "任务不存在"})
                self.respond(404, {"error": "接口不存在"})
            def do_POST(self):
                parts = self.path.strip("/").split("/")
                try:
                    payload = self.body()
                    if parts == ["skills", "semantic_pick"]:
                        task_id = bridge.start_task(str(payload["instruction"]), bool(payload.get("preview_only", False)))
                        return self.respond(202, {"task_id": task_id, "state": "submitted"})
                    if len(parts) == 4 and parts[:2] == ["skills", "semantic_pick"] and parts[3] in ("confirm", "cancel"):
                        ok, message = bridge.control_task(parts[2], parts[3])
                        return self.respond(200 if ok else 409, {"ok": ok, "message": message})
                    self.respond(404, {"error": "接口不存在"})
                except (ValueError, KeyError, json.JSONDecodeError) as exc:
                    self.respond(400, {"error": str(exc)})
            def log_message(self, _format, *_args):
                return
        return Handler

    def start_task(self, instruction, preview_only):
        client_id = uuid.uuid4().hex
        with self.lock:
            self.tasks[client_id] = {"task_id": client_id, "robot_task_id": "", "state": "submitted",
                                     "instruction": instruction, "preview_only": preview_only, "message": "已提交"}
        if not self.client.wait_for_server(timeout_sec=2.0):
            with self.lock: self.tasks[client_id].update(state="failed", message="SemanticPick Action 不可用")
            return client_id
        goal = SemanticPick.Goal(); goal.instruction = instruction; goal.preview_only = preview_only
        future = self.client.send_goal_async(goal, feedback_callback=lambda value: self.on_feedback(client_id, value.feedback))
        future.add_done_callback(lambda value: self.on_goal(client_id, value))
        return client_id

    def on_goal(self, client_id, future):
        handle = future.result()
        with self.lock:
            self.tasks[client_id]["goal_handle"] = handle
            if not handle.accepted:
                self.tasks[client_id].update(state="failed", message="任务被 Action 服务拒绝")
                return
            self.tasks[client_id]["state"] = "accepted"
        result = handle.get_result_async(); result.add_done_callback(lambda value: self.on_result(client_id, value))

    def on_feedback(self, client_id, feedback):
        with self.lock:
            self.tasks[client_id].update(robot_task_id=feedback.task_id, state=feedback.state,
                                         target_class=feedback.target_class, confidence=feedback.confidence,
                                         plan_ready=feedback.plan_ready, message=feedback.message)

    def on_result(self, client_id, future):
        result = future.result().result
        with self.lock:
            self.tasks[client_id].update(robot_task_id=result.task_id,
                                         state="succeeded" if result.success else "failed",
                                         target_class=result.target_class, target_track_id=result.target_track_id,
                                         error_code=result.error_code, message=result.message)
            self.tasks[client_id].pop("goal_handle", None)

    def control_task(self, client_id, command):
        with self.lock: task = self.tasks.get(client_id)
        if not task: return False, "任务不存在"
        if command == "cancel":
            handle = task.get("goal_handle")
            if handle: handle.cancel_goal_async()
        robot_id = task.get("robot_task_id")
        if not robot_id: return False, "机器人任务尚未进入可控制状态"
        message = String(); message.data = json.dumps({"task_id": robot_id, "command": command}, ensure_ascii=False)
        self.control.publish(message)
        return True, "命令已发送"

    def destroy_node(self):
        self.server.shutdown(); self.server.server_close(); super().destroy_node()


def main():
    if not acquire_singleton("semantic_skill_bridge"):
        return
    rclpy.init(); node = SemanticSkillBridge(); executor = MultiThreadedExecutor(num_threads=3); executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

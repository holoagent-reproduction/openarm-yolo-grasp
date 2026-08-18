"""安全优先的抓取任务状态机；未显式启用 allow_motion 时绝不执行运动。"""
import json
import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener


class GraspTaskNode(Node):
    def __init__(self):
        super().__init__("openarm_grasp_task")
        for name, default in (("cameras_file", ""), ("objects_file", ""), ("drop_zone_file", ""), ("allow_motion", False)):
            self.declare_parameter(name, default)
        self.cameras = self._load(self.get_parameter("cameras_file").value)
        self.objects_config = self._load(self.get_parameter("objects_file").value)
        self.drop = self._load(self.get_parameter("drop_zone_file").value)
        self.allow_motion = bool(self.get_parameter("allow_motion").value)
        self.base_frame = self.cameras.get("base_frame", "base_link")
        self.detections, self.selected_id, self.ready = {}, None, False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(PoseStamped, "/openarm_vision/pregrasp_pose", 10)
        self.status_pub = self.create_publisher(String, "/openarm_vision/status", 10)
        self.create_subscription(String, "/openarm_vision/tag_detections", self._on_detection, 30)
        self.create_subscription(String, "/openarm_vision/command", self._on_command, 10)
        self._status("就绪：仅预览模式。请完成相机标定、物品配置和空载验证。")

    @staticmethod
    def _load(path):
        with Path(path).expanduser().open(encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def _status(self, text):
        message = String(); message.data = text; self.status_pub.publish(message); self.get_logger().info(text)

    def _on_detection(self, message):
        try:
            data = json.loads(message.data)
            marker_id = str(data["id"])
            if marker_id not in self.objects_config.get("objects", {}) and int(data["id"]) != int(self.drop.get("tag_id", -1)):
                return
            base_pose = self._to_base(data)
            if base_pose is None:
                return
            self.detections[(data["role"], marker_id)] = (data, base_pose)
            if data["role"] == "head" and marker_id == self.selected_id:
                self._publish_preview(data, base_pose)
        except (ValueError, KeyError) as exc:
            self._status(f"忽略无效标签数据：{exc}")

    def _on_command(self, message):
        try:
            command = json.loads(message.data)
            kind = command.get("command")
            if kind == "select":
                marker_id = str(command["tag_id"])
                profile = self.objects_config.get("objects", {}).get(marker_id)
                if not profile or not profile.get("enabled", False):
                    return self._status("该标签未配置或尚未启用，拒绝抓取。")
                self.selected_id, self.ready = marker_id, False
                head = self.detections.get(("head", marker_id))
                if head:
                    self._publish_preview(head[0], head[1])
                else:
                    self._status("已选择目标；等待头部相机检测到该标签。")
            elif kind == "confirm":
                self._confirm()
            elif kind == "cancel":
                self.selected_id, self.ready = None, False
                self._status("任务已取消；未发送运动命令。")
        except (ValueError, KeyError) as exc:
            self._status(f"无效任务命令：{exc}")

    def _to_base(self, detection):
        pose = PoseStamped(); pose.header.frame_id = detection["frame_id"]
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = detection["position"]
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = detection["rotation_xyzw"]
        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, pose.header.frame_id, rclpy.time.Time())
            return do_transform_pose_stamped(pose, transform)
        except Exception as exc:
            self._status(f"等待相机到 {self.base_frame} 的标定坐标变换：{exc}")
            return None

    def _publish_preview(self, detection, base_tag_pose):
        if not self._calibrated():
            return self._status("相机外参尚未标定，当前仅显示标签检测，不生成抓取位姿。")
        pose = self._grasp_pose(base_tag_pose, self.objects_config["objects"][self.selected_id])
        self.pose_pub.publish(pose)
        wrist = self.detections.get(("wrist", self.selected_id))
        self.ready = wrist is not None and self._agree(base_tag_pose, wrist[1])
        suffix = "腕部复核通过，可确认执行。" if self.ready else "等待左腕相机复核，或检查两相机标定。"
        self._status(f"目标 {self.selected_id} 已生成预览；{suffix}")

    @staticmethod
    def _grasp_pose(tag_pose, profile):
        pose = PoseStamped(); pose.header = tag_pose.header; pose.pose = tag_pose.pose
        offset = profile.get("tag_to_grasp", {}).get("translation", [0.0, 0.0, 0.0])
        q = pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = [[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)], [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)], [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]]
        pose.pose.position.x += sum(rotation[0][i] * offset[i] for i in range(3))
        pose.pose.position.y += sum(rotation[1][i] * offset[i] for i in range(3))
        pose.pose.position.z += sum(rotation[2][i] * offset[i] for i in range(3))
        return pose

    def _agree(self, head, wrist):
        dx = head.pose.position.x - wrist.pose.position.x
        dy = head.pose.position.y - wrist.pose.position.y
        dz = head.pose.position.z - wrist.pose.position.z
        limit = float(self.objects_config["left_arm"].get("max_head_wrist_position_error_m", 0.03))
        return math.sqrt(dx * dx + dy * dy + dz * dz) <= limit

    def _calibrated(self):
        return all(camera.get("calibrated", False) for camera in self.cameras.get("cameras", {}).values())

    def _confirm(self):
        if not self.selected_id or not self.ready:
            return self._status("尚未完成头部定位与腕部复核，拒绝执行。")
        if not self.allow_motion:
            return self._status("allow_motion=false：已拦截真实运动。此时仅可验证视觉与规划预览。")
        # 真实动作接口故意独立于检测逻辑；只有显式打开 allow_motion 后才进入此分支。
        # 首次真机使用前必须先接入经验证的 MoveIt 任务执行器并完成空载测试。
        self._status("运动已获确认，但本首版仅提供安全任务门控与位姿发布；请先完成 MoveIt 执行器集成测试。")


def main():
    rclpy.init(); node = GraspTaskNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.shutdown()

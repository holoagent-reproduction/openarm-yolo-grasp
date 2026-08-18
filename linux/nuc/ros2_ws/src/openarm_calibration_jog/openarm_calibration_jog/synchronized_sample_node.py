"""按相机检测时间戳读取机械臂 TF，避免手眼样本的时间错位。"""
import json
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class SynchronizedSampleNode(Node):
    def __init__(self):
        super().__init__("openarm_synchronized_calibration_sampler")
        self.declare_parameter("camera_role", "wrist")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("hand_frame", "openarm_left_hand_tcp")
        self.declare_parameter("max_detection_age_s", 0.30)
        self.declare_parameter("sample_file", "/home/nuc/ros2_ws/calibration_samples/synchronized_hand_eye.json")
        self.role = self.get_parameter("camera_role").value; self.latest_tag = None
        self.tf = Buffer(cache_time=Duration(seconds=30.0)); self.listener = TransformListener(self.tf, self)
        self.status = self.create_publisher(String, "/openarm_calibration_jog/status", 10)
        self.create_subscription(String, "/openarm_vision/tag_detections", self.on_tag, 30)
        self.create_subscription(String, "/openarm_calibration_jog/command", self.on_command, 10)
        self.say(f"同步标定采样已启动：camera_role={self.role}")

    def say(self, text):
        message = String(); message.data = text; self.status.publish(message); self.get_logger().info(text)

    def on_tag(self, message):
        try:
            value = json.loads(message.data)
            if value.get("role") == self.role: self.latest_tag = value
        except ValueError: pass

    def on_command(self, message):
        try:
            if json.loads(message.data).get("command") == "record": self.record()
        except ValueError:
            self.say("记录命令不是有效 JSON")
        except Exception as exc:
            # 单个坏样本不应导致整个同步采样节点退出。
            self.say(f"同步记录异常，已拒绝本组但节点继续运行：{exc}")

    def record(self):
        if not self.latest_tag:
            return self.say(f"{self.role} 相机尚未检测到 AprilTag")
        try:
            stamp_value = self.latest_tag["stamp"]
            stamp_nanoseconds = (
                int(stamp_value["sec"]) * 1_000_000_000
                + int(stamp_value["nanosec"])
            )
            # Image.header.stamp 不携带时钟类型。显式采用节点时钟类型，
            # 避免 ROS_TIME 与 SYSTEM_TIME 相减导致首次记录时节点退出。
            stamp = Time(
                nanoseconds=stamp_nanoseconds,
                clock_type=self.get_clock().clock_type,
            )
            age = abs(self.get_clock().now().nanoseconds - stamp_nanoseconds) * 1e-9
            if age > float(self.get_parameter("max_detection_age_s").value):
                return self.say(f"AprilTag 检测已过期 {age:.3f} 秒，未记录")
            transform = self.tf.lookup_transform(
                self.get_parameter("world_frame").value,
                self.get_parameter("hand_frame").value,
                stamp, timeout=Duration(seconds=0.5))
            t = transform.transform.translation; q = transform.transform.rotation
            sample = {
                "camera_role": self.role,
                "stamp": stamp_value,
                "hand_in_world": {"translation": [t.x, t.y, t.z], "rotation_xyzw": [q.x, q.y, q.z, q.w]},
                "tag_in_camera": self.latest_tag,
            }
            path = Path(self.get_parameter("sample_file").value); path.parent.mkdir(parents=True, exist_ok=True)
            data = json.loads(path.read_text(encoding="utf-8")) if path.exists() else []
            data.append(sample); path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
            self.say(f"已同步记录第 {len(data)} 组 {self.role} 标定样本，时间差 {age:.3f} 秒")
        except Exception as exc:
            self.say(f"同步记录失败，未写入本组：{exc}")


def main():
    rclpy.init(); node = SynchronizedSampleNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

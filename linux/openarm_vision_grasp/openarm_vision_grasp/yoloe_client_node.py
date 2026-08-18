"""向独立 GPU YOLOE 服务发送双相机彩色帧并发布类型化分割结果。"""
import base64
import json
import threading
import time
import urllib.request
import uuid

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from openarm_semantic_grasp_interfaces.msg import SemanticDetection
from .semantic_parser import load_config
from .instance_guard import acquire_singleton


class YoloeClientNode(Node):
    def __init__(self):
        super().__init__("openarm_yoloe_client")
        self.declare_parameter("config_file", "")
        self.config = load_config(self.get_parameter("config_file").value)
        cfg = self.config["yoloe"]
        self.server_url = cfg["server_url"]
        self.timeout = float(cfg.get("request_timeout_s", 1.5))
        self.period = 1.0 / float(cfg.get("max_request_rate_hz", 5.0))
        self.jpeg_quality = int(cfg.get("jpeg_quality", 85))
        self.confidence = float(cfg.get("confidence_threshold", 0.4))
        self.iou = float(cfg.get("iou_threshold", 0.6))
        self.image_size = int(cfg.get("image_size", 640))
        self.bridge = CvBridge()
        self.query = None
        self.last_sent = {"head": 0.0, "wrist": 0.0}
        self.busy = {"head": False, "wrist": False}
        self.lock = threading.Lock()
        self.publisher = self.create_publisher(SemanticDetection, "/openarm_vision/semantic_detections", 20)
        self.overlay_pub = {
            "head": self.create_publisher(Image, "/openarm_vision/head_yoloe_overlay", 3),
            "wrist": self.create_publisher(Image, "/openarm_vision/wrist_yoloe_overlay", 3),
        }
        self.status = self.create_publisher(String, "/openarm_vision/yoloe_status", 10)
        self.create_subscription(String, "/openarm_vision/semantic_query", self.on_query, 10)
        self.create_subscription(Image, cfg["head_image_topic"], lambda msg: self.on_image("head", msg), 5)
        self.create_subscription(Image, cfg["wrist_image_topic"], lambda msg: self.on_image("wrist", msg), 5)

    def on_query(self, message):
        try:
            query = json.loads(message.data)
            if query.get("active", True):
                self.query = {"task_id": str(query["task_id"]), "class_name": str(query["class_name"])}
            else:
                self.query = None
        except (ValueError, KeyError) as exc:
            self.say(f"忽略无效语义查询：{exc}")

    def say(self, text):
        msg = String(); msg.data = text; self.status.publish(msg)
        self.get_logger().info(text)

    def on_image(self, role, message):
        query = self.query
        now = time.monotonic()
        with self.lock:
            if query is None or self.busy[role] or now - self.last_sent[role] < self.period:
                return
            self.busy[role] = True
            self.last_sent[role] = now
        threading.Thread(target=self.request, args=(role, message, dict(query)), daemon=True).start()

    def request(self, role, message, query):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if not ok:
                raise RuntimeError("JPEG 编码失败")
            boundary = "----openarm" + uuid.uuid4().hex
            fields = {
                "prompts": json.dumps([query["class_name"]]),
                "confidence": str(self.confidence), "iou": str(self.iou), "image_size": str(self.image_size),
            }
            body = bytearray()
            for name, value in fields.items():
                body.extend(f"--{boundary}\r\nContent-Disposition: form-data; name=\"{name}\"\r\n\r\n{value}\r\n".encode())
            body.extend(f"--{boundary}\r\nContent-Disposition: form-data; name=\"image\"; filename=\"frame.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n".encode())
            body.extend(encoded.tobytes()); body.extend(f"\r\n--{boundary}--\r\n".encode())
            request = urllib.request.Request(self.server_url, data=bytes(body), method="POST",
                                             headers={"Content-Type": f"multipart/form-data; boundary={boundary}"})
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                result = json.loads(response.read().decode("utf-8"))
            overlay = image.copy()
            for item in result.get("detections", []):
                if item.get("class_name") != query["class_name"] or not item.get("mask_png_base64"):
                    continue
                mask_bytes = base64.b64decode(item["mask_png_base64"])
                mask = cv2.imdecode(np.frombuffer(mask_bytes, np.uint8), cv2.IMREAD_GRAYSCALE)
                if mask is None:
                    continue
                color = np.zeros_like(overlay); color[:, :, 1] = mask
                overlay = cv2.addWeighted(overlay, 1.0, color, 0.35, 0.0)
                x1, y1, x2, y2 = [int(v) for v in item["bbox_xyxy"]]
                cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(overlay, f'{item["class_name"]} {float(item["confidence"]):.2f}',
                            (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                output = SemanticDetection()
                output.header = message.header
                output.task_id = query["task_id"]; output.camera_role = role
                output.class_name = item["class_name"]; output.confidence = float(item["confidence"])
                output.track_id = int(item.get("track_id", -1)); output.bbox_xyxy = [int(v) for v in item["bbox_xyxy"]]
                output.mask = self.bridge.cv2_to_imgmsg(mask, encoding="mono8"); output.mask.header = message.header
                self.publisher.publish(output)
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8"); overlay_msg.header = message.header
            self.overlay_pub[role].publish(overlay_msg)
        except Exception as exc:
            self.get_logger().warning(f"{role} YOLOE 请求失败：{exc}", throttle_duration_sec=2.0)
        finally:
            with self.lock:
                self.busy[role] = False


def main():
    if not acquire_singleton("yoloe_client"):
        return
    rclpy.init(); node = YoloeClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""将两台 RealSense 的 AprilTag 检测结果发布为 JSON，供抓取状态机消费。"""
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class TagDetector(Node):
    def __init__(self):
        super().__init__("openarm_apriltag_detector")
        self.declare_parameter("cameras_file", "")
        self.declare_parameter("objects_file", "")
        cameras_path = self.get_parameter("cameras_file").value
        objects_path = self.get_parameter("objects_file").value
        self.cameras = self._load(cameras_path)["cameras"]
        object_config = self._load(objects_path)
        self.tag_size = float(object_config.get("tag_size_m", 0.05))
        family = object_config.get("tag_family", "DICT_APRILTAG_36h11")
        if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, family):
            raise RuntimeError("当前 OpenCV 不含 AprilTag aruco 模块；请安装带 aruco 的 python3-opencv。")
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, family))
        self.parameters = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, "DetectorParameters")
                           else cv2.aruco.DetectorParameters_create())
        self.detector = (cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
                         if hasattr(cv2.aruco, "ArucoDetector") else None)
        self.bridge = CvBridge()
        self.camera_matrix, self.distortion = {}, {}
        self.publisher = self.create_publisher(String, "/openarm_vision/tag_detections", 30)
        for name, camera in self.cameras.items():
            self.create_subscription(CameraInfo, camera["camera_info_topic"], lambda msg, n=name: self._on_info(n, msg), 10)
            self.create_subscription(Image, camera["image_topic"], lambda msg, n=name: self._on_image(n, msg), 10)
        self.get_logger().info("AprilTag 检测已启动；仅发布检测结果，不控制机械臂。")

    @staticmethod
    def _load(path):
        with Path(path).expanduser().open(encoding="utf-8") as stream:
            return yaml.safe_load(stream)

    def _on_info(self, name, msg):
        self.camera_matrix[name] = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.distortion[name] = np.array(msg.d, dtype=np.float64)

    def _on_image(self, name, msg):
        if name not in self.camera_matrix:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.detector is not None:
                corners, ids, _ = self.detector.detectMarkers(image)
            else:
                corners, ids, _ = cv2.aruco.detectMarkers(image, self.dictionary, parameters=self.parameters)
            if ids is None:
                return
            half = self.tag_size / 2.0
            object_points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
            for corners_one, marker_id in zip(corners, ids.flatten()):
                ok, rvec, tvec = cv2.solvePnP(object_points, corners_one.reshape((4, 2)), self.camera_matrix[name], self.distortion[name], flags=cv2.SOLVEPNP_IPPE_SQUARE)
                if not ok:
                    continue
                rotation, _ = cv2.Rodrigues(rvec)
                quaternion = self._matrix_to_quaternion(rotation)
                payload = {"camera": name, "role": self.cameras[name]["role"], "id": int(marker_id),
                           "frame_id": msg.header.frame_id, "stamp": {"sec": msg.header.stamp.sec, "nanosec": msg.header.stamp.nanosec},
                           "position": [float(v) for v in tvec.flatten()], "rotation_xyzw": quaternion}
                out = String()
                out.data = json.dumps(payload, ensure_ascii=False)
                self.publisher.publish(out)
        except Exception as exc:
            self.get_logger().error(f"标签检测失败：{exc}", throttle_duration_sec=2.0)

    @staticmethod
    def _matrix_to_quaternion(matrix):
        trace = np.trace(matrix)
        if trace > 0:
            s = np.sqrt(trace + 1.0) * 2.0
            return [(matrix[2, 1] - matrix[1, 2]) / s, (matrix[0, 2] - matrix[2, 0]) / s, (matrix[1, 0] - matrix[0, 1]) / s, 0.25 * s]
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            s = np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            return [0.25 * s, (matrix[0, 1] + matrix[1, 0]) / s, (matrix[0, 2] + matrix[2, 0]) / s, (matrix[2, 1] - matrix[1, 2]) / s]
        if index == 1:
            s = np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            return [(matrix[0, 1] + matrix[1, 0]) / s, 0.25 * s, (matrix[1, 2] + matrix[2, 1]) / s, (matrix[0, 2] - matrix[2, 0]) / s]
        s = np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        return [(matrix[0, 2] + matrix[2, 0]) / s, (matrix[1, 2] + matrix[2, 1]) / s, 0.25 * s, (matrix[1, 0] - matrix[0, 1]) / s]


def main():
    rclpy.init()
    node = TagDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""将 YOLOE 分割掩码与 D435i 对齐深度融合为 world 坐标系目标点云统计。"""
from collections import deque
import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.clock import ClockType
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from openarm_semantic_grasp_interfaces.msg import SemanticDetection, SemanticObject3D
from .semantic_parser import load_config
from .instance_guard import acquire_singleton


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quaternion_matrix(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ], dtype=np.float64)


class Semantic3DNode(Node):
    def __init__(self):
        super().__init__("openarm_semantic_3d")
        self.declare_parameter("config_file", "")
        self.config = load_config(self.get_parameter("config_file").value)
        cfg = self.config["depth"]
        self.world = self.config["frames"]["world"]
        self.bridge = CvBridge(); self.depth_buffer = deque(maxlen=20); self.camera_info = None
        self.sync_error = float(cfg["max_sync_error_s"]); self.scale = float(cfg["depth_scale_16uc1"])
        self.min_depth = float(cfg["min_depth_m"]); self.max_depth = float(cfg["max_depth_m"])
        self.min_points = int(cfg["min_points"]); self.max_std = float(cfg["max_depth_std_m"])
        self.mad_scale = float(cfg["outlier_mad_scale"])
        self.table_z = float(self.config["workspace"].get("table_z", 0.0))
        self.tf = Buffer(); self.tf_listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(SemanticObject3D, "/openarm_vision/semantic_objects_3d_raw", 20)
        self.create_subscription(Image, cfg["topic"], self.on_depth, 10)
        self.create_subscription(CameraInfo, cfg["camera_info_topic"], self.on_info, 10)
        self.create_subscription(SemanticDetection, "/openarm_vision/semantic_detections", self.on_detection, 20)

    def on_info(self, message):
        self.camera_info = message

    def on_depth(self, message):
        try:
            depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            if message.encoding in ("16UC1", "mono16"):
                depth = depth.astype(np.float32) * self.scale
            else:
                depth = depth.astype(np.float32)
            self.depth_buffer.append((stamp_seconds(message.header.stamp), message.header.frame_id, depth))
        except Exception as exc:
            self.get_logger().warning(f"深度图转换失败：{exc}", throttle_duration_sec=2.0)

    def on_detection(self, detection):
        if detection.camera_role != "head" or self.camera_info is None or not self.depth_buffer:
            return
        target_stamp = stamp_seconds(detection.header.stamp)
        depth_stamp, depth_frame, depth = min(self.depth_buffer, key=lambda item: abs(item[0] - target_stamp))
        if abs(depth_stamp - target_stamp) > self.sync_error:
            self.get_logger().warning("YOLOE 掩码与深度图时间差超限", throttle_duration_sec=2.0)
            return
        try:
            mask = self.bridge.imgmsg_to_cv2(detection.mask, desired_encoding="mono8")
            if mask.shape != depth.shape:
                mask = cv2.resize(mask, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_NEAREST)
            valid = (mask > 0) & np.isfinite(depth) & (depth >= self.min_depth) & (depth <= self.max_depth)
            rows, cols = np.where(valid)
            if len(rows) < self.min_points:
                return
            # 限制计算量但保持掩码空间分布。
            if len(rows) > 8000:
                indices = np.linspace(0, len(rows) - 1, 8000, dtype=np.int32)
                rows, cols = rows[indices], cols[indices]
            z = depth[rows, cols]
            median = float(np.median(z)); mad = float(np.median(np.abs(z - median)))
            if mad > 1e-6:
                keep = np.abs(z - median) <= self.mad_scale * 1.4826 * mad
                rows, cols, z = rows[keep], cols[keep], z[keep]
            if len(z) < self.min_points or float(np.std(z)) > self.max_std:
                return
            k = np.asarray(self.camera_info.k, dtype=np.float64).reshape(3, 3)
            x = (cols - k[0, 2]) * z / k[0, 0]
            y = (rows - k[1, 2]) * z / k[1, 1]
            camera_points = np.column_stack((x, y, z))
            source_frame = detection.header.frame_id or depth_frame
            try:
                # 头部相机外参是静态变换。YOLOE经过网络推理后，检测时间戳可能
                # 略晚于 TF 缓存的最新时间；使用最新可用 TF 可避免“未来外推”。
                latest_time = Time(seconds=0, clock_type=ClockType.ROS_TIME)
                transform = self.tf.lookup_transform(self.world, source_frame, latest_time)
            except Exception:
                # 兼容仍提供精确时间戳的动态 TF；失败时再尝试检测时间。
                transform = self.tf.lookup_transform(self.world, source_frame,
                                                     Time.from_msg(detection.header.stamp))
            q = transform.transform.rotation
            rotation = quaternion_matrix((q.x, q.y, q.z, q.w))
            translation = np.array([transform.transform.translation.x, transform.transform.translation.y,
                                    transform.transform.translation.z])
            world_points = camera_points @ rotation.T + translation
            world_points = world_points[world_points[:, 2] >= self.table_z - 0.01]
            if len(world_points) < self.min_points:
                return
            low, high = np.percentile(world_points, [2, 98], axis=0)
            center = np.median(world_points, axis=0)
            xy = world_points[:, :2] - center[:2]
            eigenvalues, eigenvectors = np.linalg.eigh(np.cov(xy.T))
            major = eigenvectors[:, int(np.argmax(eigenvalues))]
            yaw = math.atan2(float(major[1]), float(major[0]))
            output = SemanticObject3D()
            output.header = detection.header; output.header.frame_id = self.world
            output.task_id = detection.task_id; output.class_name = detection.class_name
            output.confidence = detection.confidence
            output.track_id = detection.track_id if detection.track_id >= 0 else int(abs(hash((detection.class_name, round(center[0], 2), round(center[1], 2)))) % 2147483647)
            output.pose.position.x, output.pose.position.y, output.pose.position.z = [float(v) for v in center]
            output.pose.orientation.z = math.sin(yaw / 2.0); output.pose.orientation.w = math.cos(yaw / 2.0)
            output.size.x, output.size.y, output.size.z = [float(v) for v in high - low]
            output.grasp_yaw = float(yaw); output.point_count = len(world_points)
            output.depth_std = float(np.std(z)); output.stable = False
            self.publisher.publish(output)
        except Exception as exc:
            self.get_logger().warning(f"三维语义定位失败：{exc}", throttle_duration_sec=2.0)


def main():
    if not acquire_singleton("semantic_3d"):
        return
    rclpy.init(); node = Semantic3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

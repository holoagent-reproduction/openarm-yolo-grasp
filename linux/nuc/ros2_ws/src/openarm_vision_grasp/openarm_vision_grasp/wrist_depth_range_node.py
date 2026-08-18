"""根据左腕 D415 深度图和腕部目标框发布目标距离。"""
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from openarm_semantic_grasp_interfaces.msg import SemanticDetection


class WristDepthRangeNode(Node):
    def __init__(self):
        super().__init__('openarm_wrist_depth_range')
        self.declare_parameter('depth_topic', '/openarm_vision/left_wrist_d415/depth/image_rect_raw')
        self.declare_parameter('min_depth_m', 0.03)
        self.declare_parameter('max_depth_m', 0.80)
        self.bridge = CvBridge(); self.depth = None; self.detection = None
        self.distance_pub = self.create_publisher(Float32, '/openarm_vision/wrist_depth_distance', 10)
        self.status_pub = self.create_publisher(String, '/openarm_vision/wrist_depth_status', 10)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self.on_depth, 10)
        self.create_subscription(SemanticDetection, '/openarm_vision/semantic_detections', self.on_detection, 20)
        self.create_timer(0.05, self.compute)

    def on_depth(self, msg):
        try:
            self.depth = (self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), msg.header.stamp)
        except Exception as exc:
            self.get_logger().warning(f'D415 深度转换失败：{exc}')

    def on_detection(self, msg):
        if msg.camera_role == 'wrist':
            self.detection = msg

    def compute(self):
        if self.depth is None:
            return
        image, stamp = self.depth
        if self.detection is not None:
            x1, y1, x2, y2 = [int(v) for v in self.detection.bbox_xyxy]
            source = 'wrist_bbox'
        else:
            # D415 近距离只需看到杯子局部；使用夹爪前方的固定中心区域，
            # 避免 YOLOE 因遮挡没有输出完整腕部目标框时阻塞抓取。
            x1, x2 = int(image.shape[1] * 0.25), int(image.shape[1] * 0.75)
            y1, y2 = int(image.shape[0] * 0.30), int(image.shape[0] * 0.82)
            source = 'fixed_roi'
        h, w = image.shape[:2]; x1=max(0,min(w-1,x1)); x2=max(x1+1,min(w,x2)); y1=max(0,min(h-1,y1)); y2=max(y1+1,min(h,y2))
        roi = image[y1:y2, x1:x2].astype(np.float32)
        if image.dtype == np.uint16:
            roi *= 0.001
        values = roi[np.isfinite(roi)]
        lo=float(self.get_parameter('min_depth_m').value); hi=float(self.get_parameter('max_depth_m').value)
        values = values[(values >= lo) & (values <= hi)]
        if values.size < 30:
            return
        distance = float(np.median(values))
        self.distance_pub.publish(Float32(data=distance))
        self.status_pub.publish(String(data=f'距离={distance:.3f}m，有效深度点={values.size}，来源={source}'))


def main(args=None):
    rclpy.init(args=args); node=WristDepthRangeNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()

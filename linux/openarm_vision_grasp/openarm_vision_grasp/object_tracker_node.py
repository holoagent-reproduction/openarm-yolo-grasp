"""对三维语义目标做短时稳定跟踪，并发布当前任务的唯一候选。"""
from collections import defaultdict, deque
import math

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from openarm_semantic_grasp_interfaces.msg import SemanticObject3D
from .semantic_parser import load_config
from .instance_guard import acquire_singleton


class ObjectTrackerNode(Node):
    def __init__(self):
        super().__init__("openarm_semantic_object_tracker")
        self.declare_parameter("config_file", "")
        self.config = load_config(self.get_parameter("config_file").value)
        cfg = self.config["tracking"]
        self.required = int(cfg["stable_observations"]); self.max_std = float(cfg["max_position_std_m"])
        self.stale = float(cfg["stale_after_s"]); self.reach_center = np.array(cfg["left_reach_center"], dtype=float)
        self.workspace = self.config["workspace"]
        self.query = None; self.history = defaultdict(lambda: deque(maxlen=max(8, self.required * 2)))
        self.publisher = self.create_publisher(SemanticObject3D, "/openarm_vision/semantic_target", 10)
        self.create_subscription(String, "/openarm_vision/semantic_query", self.on_query, 10)
        self.create_subscription(SemanticObject3D, "/openarm_vision/semantic_objects_3d_raw", self.on_object, 30)

    def on_query(self, message):
        import json
        try:
            value = json.loads(message.data)
            self.query = value if value.get("active", True) else None
            if self.query:
                self.history.clear()
        except ValueError:
            self.query = None

    def in_workspace(self, position):
        return all(float(self.workspace[axis][0]) <= value <= float(self.workspace[axis][1])
                   for axis, value in zip(("x", "y", "z"), position))

    def on_object(self, message):
        if not self.query or message.task_id != str(self.query.get("task_id")) or message.class_name != self.query.get("class_name"):
            return
        position = np.array([message.pose.position.x, message.pose.position.y, message.pose.position.z])
        if not self.in_workspace(position):
            return
        profile = self.config["classes"].get(message.class_name, {})
        size = np.array([message.size.x, message.size.y, message.size.z])
        if np.any(size < np.array(profile.get("size_min_m", [0, 0, 0]))) or np.any(size > np.array(profile.get("size_max_m", [99, 99, 99]))):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        key = (message.task_id, message.class_name, message.track_id)
        self.history[key].append((now, position, message))
        for old_key in list(self.history):
            if not self.history[old_key] or now - self.history[old_key][-1][0] > self.stale:
                del self.history[old_key]
        candidates = []
        for values in self.history.values():
            recent = list(values)[-self.required:]
            if len(recent) < self.required:
                continue
            positions = np.array([item[1] for item in recent])
            position_std = float(np.max(np.std(positions, axis=0)))
            if position_std > self.max_std:
                continue
            latest = recent[-1][2]
            score = float(latest.confidence) - 0.15 * float(np.linalg.norm(positions.mean(axis=0) - self.reach_center))
            candidates.append((score, latest, positions.mean(axis=0)))
        if not candidates:
            return
        _, selected, mean = max(candidates, key=lambda item: item[0])
        selected.pose.position.x, selected.pose.position.y, selected.pose.position.z = [float(v) for v in mean]
        selected.stable = True
        self.publisher.publish(selected)


def main():
    if not acquire_singleton("object_tracker"):
        return
    rclpy.init(); node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

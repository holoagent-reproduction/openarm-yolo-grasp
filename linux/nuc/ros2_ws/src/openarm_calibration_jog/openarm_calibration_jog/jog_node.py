import json
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from tf2_ros import Buffer, TransformListener

JOINTS = [f"openarm_left_joint{i}" for i in range(1, 8)]

class JogNode(Node):
    def __init__(self):
        super().__init__("openarm_calibration_jog")
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("sample_file", "/home/nuc/ros2_ws/calibration_samples/left_hand_eye.json")
        self.allow_motion = self.get_parameter("allow_motion").value
        self.current, self.wrist_tag = {}, None
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.status = self.create_publisher(String, "/openarm_calibration_jog/status", 10)
        self.create_subscription(JointState, "/joint_states", self.on_state, 20)
        self.create_subscription(String, "/openarm_vision/tag_detections", self.on_tag, 20)
        self.create_subscription(String, "/openarm_calibration_jog/command", self.on_command, 10)
        self.say("安全模式：allow_motion=false，不会发送机械臂运动。")
    def say(self, text):
        m=String(); m.data=text; self.status.publish(m); self.get_logger().info(text)
    def on_state(self, msg):
        self.current.update(dict(zip(msg.name, msg.position)))
    def on_tag(self, msg):
        try:
            tag=json.loads(msg.data)
            if tag.get("role")=="wrist": self.wrist_tag=tag
        except ValueError: pass
    def on_command(self, msg):
        try:
            cmd=json.loads(msg.data)
            if cmd["command"]=="jog": self.jog(int(cmd["joint"]), float(cmd["delta_deg"]))
            elif cmd["command"]=="record": self.record()
        except (ValueError, KeyError) as e: self.say(f"命令无效：{e}")
    def jog(self, joint, delta):
        if not 1 <= joint <= 7 or len(self.current)<7: return self.say("等待完整左臂关节状态。")
        if abs(delta)>5: return self.say("单次调整不得超过 5°。")
        target={name:self.current[name] for name in JOINTS}
        target[JOINTS[joint-1]] += delta*3.141592653589793/180
        if not self.allow_motion: return self.say(f"预览 J{joint} {delta:+.1f}°；安全模式未发送轨迹。")
        if not self.move.wait_for_server(timeout_sec=2): return self.say("MoveIt move_action 未启动。")
        goal=MoveGroup.Goal(); goal.request.group_name="left_arm"; goal.request.allowed_planning_time=5.0
        goal.request.max_velocity_scaling_factor=0.05; goal.request.max_acceleration_scaling_factor=0.05
        c=Constraints(); c.joint_constraints=[JointConstraint(joint_name=n, position=p, tolerance_above=0.01, tolerance_below=0.01, weight=1.0) for n,p in target.items()]
        goal.request.goal_constraints=[c]; goal.planning_options.plan_only=False
        self.move.send_goal_async(goal); self.say(f"已请求 MoveIt 低速执行 J{joint} {delta:+.1f}°。")
    def record(self):
        if not self.wrist_tag: return self.say("未检测到左腕相机 AprilTag，未记录样本。")
        try:
            # OpenArm 的 URDF 根坐标系为 world，而不是 base_link。
            hand=self.tf.lookup_transform("world", "openarm_left_hand", rclpy.time.Time())
            path=Path(self.get_parameter("sample_file").value); path.parent.mkdir(parents=True, exist_ok=True)
            data=json.loads(path.read_text()) if path.exists() else []
            data.append({"hand_in_base": {"translation": [hand.transform.translation.x,hand.transform.translation.y,hand.transform.translation.z], "rotation_xyzw": [hand.transform.rotation.x,hand.transform.rotation.y,hand.transform.rotation.z,hand.transform.rotation.w]}, "tag_in_wrist_camera": self.wrist_tag})
            path.write_text(json.dumps(data, ensure_ascii=False, indent=2)); self.say(f"已记录第 {len(data)} 个手眼标定样本。")
        except Exception as e: self.say(f"记录失败：{e}")
def main():
    rclpy.init(); n=JogNode()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()

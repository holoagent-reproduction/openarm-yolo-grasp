"""OpenArm 双臂关节控制界面。

默认不发出机械臂命令。用户需启用输出并主动点击发送按钮。
"""

import math
import queue
import threading
import tkinter as tk
from dataclasses import dataclass
from tkinter import messagebox, ttk
from typing import Dict, List, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_SIDES = ("left", "right")
JOINT_COUNT = 7
# 这是保守的软件限位；正式使用前应替换为该机械臂 URDF 的真实限位。
JOINT_LIMIT_DEGREES = [(-170.0, 170.0)] * JOINT_COUNT


@dataclass
class ArmWidgets:
    values: List[tk.DoubleVar]
    feedback: List[tk.StringVar]


class OpenArmJointGuiNode(Node):
    """ROS 节点：收关节反馈，按需发布关节目标。"""

    def __init__(self, feedback_queue: queue.Queue):
        super().__init__("openarm_joint_gui")
        self._feedback_queue = feedback_queue
        self._publishers = {
            side: self.create_publisher(JointState, f"/{side}_arm/joint_commands", 10)
            for side in ARM_SIDES
        }
        self._subscriptions = [
            self.create_subscription(
                JointState,
                f"/{side}_arm/joint_states",
                lambda msg, arm=side: self._on_joint_state(arm, msg),
                10,
            )
            for side in ARM_SIDES
        ]

    def _on_joint_state(self, arm: str, msg: JointState) -> None:
        if len(msg.position) < JOINT_COUNT:
            return
        positions = [math.degrees(value) for value in msg.position[:JOINT_COUNT]]
        self._feedback_queue.put((arm, positions))

    def send(self, arm: str, values_in_degrees: List[float]) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f"{arm}_joint_{index}" for index in range(1, JOINT_COUNT + 1)]
        message.position = [math.radians(value) for value in values_in_degrees]
        self._publishers[arm].publish(message)
        self.get_logger().info(f"已发送 {arm} 臂关节目标：{values_in_degrees}")


class JointGui:
    """Tkinter 主界面。ROS 回调不直接触碰 UI，避免跨线程访问。"""

    def __init__(self, root: tk.Tk, node: OpenArmJointGuiNode, feedback_queue: queue.Queue):
        self.root = root
        self.node = node
        self.feedback_queue = feedback_queue
        self.arms: Dict[str, ArmWidgets] = {}
        self.last_feedback: Dict[str, Optional[List[float]]] = {side: None for side in ARM_SIDES}
        self.output_enabled = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value="等待 ROS 关节反馈… 输出已锁定")

        self.root.title("OpenArm 双臂关节控制")
        self.root.minsize(940, 540)
        self._build()
        self._drain_feedback_queue()

    def _build(self) -> None:
        main = ttk.Frame(self.root, padding=14)
        main.grid(sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)

        for column, (side, title) in enumerate((("left", "左臂"), ("right", "右臂"))):
            frame = ttk.LabelFrame(main, text=title, padding=10)
            frame.grid(row=0, column=column, padx=6, pady=4, sticky="nsew")
            main.columnconfigure(column, weight=1)
            self.arms[side] = self._build_arm(frame)

        controls = ttk.Frame(main)
        controls.grid(row=1, column=0, columnspan=2, pady=(14, 0), sticky="ew")
        controls.columnconfigure(0, weight=1)

        ttk.Checkbutton(
            controls,
            text="启用真实机械臂输出（确认周边安全后再勾选）",
            variable=self.output_enabled,
            command=self._on_output_switch,
        ).grid(row=0, column=0, sticky="w")
        ttk.Button(controls, text="从反馈同步", command=self._sync_from_feedback).grid(row=0, column=1, padx=4)
        ttk.Button(controls, text="发送左臂轨迹", command=lambda: self._send(("left",))).grid(row=0, column=2, padx=4)
        ttk.Button(controls, text="发送右臂轨迹", command=lambda: self._send(("right",))).grid(row=0, column=3, padx=4)
        ttk.Button(controls, text="发送双臂轨迹", command=lambda: self._send(ARM_SIDES)).grid(row=0, column=4, padx=4)
        ttk.Label(main, textvariable=self.status, foreground="#1f5f8b").grid(
            row=2, column=0, columnspan=2, pady=(10, 0), sticky="w"
        )

    def _build_arm(self, parent: ttk.LabelFrame) -> ArmWidgets:
        values, feedback = [], []
        ttk.Label(parent, text="关节").grid(row=0, column=0, sticky="w")
        ttk.Label(parent, text="目标（°）").grid(row=0, column=1, sticky="w")
        ttk.Label(parent, text="实际（°）").grid(row=0, column=3, sticky="w")
        for index, (minimum, maximum) in enumerate(JOINT_LIMIT_DEGREES):
            target = tk.DoubleVar(value=0.0)
            actual = tk.StringVar(value="--")
            values.append(target)
            feedback.append(actual)
            row = index + 1
            ttk.Label(parent, text=f"J{index + 1}").grid(row=row, column=0, sticky="w", pady=3)
            slider = ttk.Scale(parent, from_=minimum, to=maximum, variable=target, length=220)
            slider.grid(row=row, column=1, columnspan=2, padx=8, sticky="ew")
            spin = ttk.Spinbox(parent, from_=minimum, to=maximum, increment=0.1, textvariable=target, width=8)
            spin.grid(row=row, column=3, padx=(0, 8))
            ttk.Label(parent, textvariable=actual, width=8).grid(row=row, column=4, sticky="e")
        parent.columnconfigure(1, weight=1)
        return ArmWidgets(values=values, feedback=feedback)

    def _on_output_switch(self) -> None:
        state = "已启用真实输出" if self.output_enabled.get() else "输出已锁定"
        self.status.set(state)

    def _drain_feedback_queue(self) -> None:
        while True:
            try:
                arm, positions = self.feedback_queue.get_nowait()
            except queue.Empty:
                break
            self.last_feedback[arm] = positions
            widgets = self.arms.get(arm)
            if widgets:
                for feedback, value in zip(widgets.feedback, positions):
                    feedback.set(f"{value:.1f}")
                self.status.set(f"已收到{('左' if arm == 'left' else '右')}臂反馈")
        self.root.after(100, self._drain_feedback_queue)

    def _sync_from_feedback(self) -> None:
        for arm, positions in self.last_feedback.items():
            if positions is None:
                continue
            for variable, value in zip(self.arms[arm].values, positions):
                variable.set(value)
        self.status.set("已将滑块同步到最新反馈")

    def _send(self, arms: tuple) -> None:
        if not self.output_enabled.get():
            messagebox.showwarning("输出已锁定", "请先确认现场安全，再勾选“启用真实机械臂输出”。")
            return
        for arm in arms:
            values = [variable.get() for variable in self.arms[arm].values]
            for index, (value, limits) in enumerate(zip(values, JOINT_LIMIT_DEGREES), start=1):
                if not limits[0] <= value <= limits[1]:
                    messagebox.showerror("目标超限", f"{arm} 臂 J{index} 超出软件限位。")
                    return
            self.node.send(arm, values)
        self.status.set("已发送目标；请观察机械臂运动与急停状态")


def main() -> None:
    rclpy.init()
    feedback_queue: queue.Queue = queue.Queue()
    node = OpenArmJointGuiNode(feedback_queue)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    root = tk.Tk()
    try:
        JointGui(root, node, feedback_queue)
        root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

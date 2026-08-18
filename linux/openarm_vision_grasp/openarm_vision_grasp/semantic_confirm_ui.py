"""YOLOE 语义抓取人工确认界面。"""

import base64
import json
import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .instance_guard import acquire_singleton


class SemanticConfirmUiNode(Node):
    def __init__(self, events):
        super().__init__("openarm_semantic_confirm_ui")
        self.events = events
        self.bridge = CvBridge()
        self.goal_pub = self.create_publisher(String, "/openarm_vision/semantic_goal", 10)
        self.control_pub = self.create_publisher(String, "/openarm_vision/semantic_control", 10)
        self.step_pub = self.create_publisher(String, "/openarm_vision/semantic_step_command", 10)
        self.create_subscription(String, "/openarm_vision/semantic_status", self.on_status, 20)
        self.create_subscription(String, "/openarm_vision/semantic_task_status", self.on_status, 20)
        self.create_subscription(String, "/openarm_vision/semantic_step_status", self.on_step_status, 20)
        self.create_subscription(Image, "/openarm_vision/head_yoloe_overlay", self.on_image, 3)

    def on_status(self, message):
        try:
            self.events.put(("status", json.loads(message.data)))
        except ValueError:
            self.events.put(("status", {"message": message.data}))

    def on_step_status(self, message):
        try:
            self.events.put(("step_status", json.loads(message.data)))
        except ValueError:
            self.events.put(("step_status", {"message": message.data}))

    def on_image(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            height, width = image.shape[:2]
            scale = min(1.0, 640.0 / width, 420.0 / height)
            if scale < 1.0:
                image = cv2.resize(image, (int(width * scale), int(height * scale)))
            ok, encoded = cv2.imencode(".png", image)
            if ok:
                self.events.put(("image", base64.b64encode(encoded.tobytes()).decode("ascii")))
        except Exception:
            pass

    @staticmethod
    def send(publisher, payload):
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(message)


def main():
    if not acquire_singleton("semantic_confirm_ui"):
        return
    rclpy.init()
    events = queue.Queue()
    node = SemanticConfirmUiNode(events)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    root = tk.Tk()
    root.title("OpenArm YOLOE 语义抓取确认")
    root.minsize(900, 680)
    frame = ttk.Frame(root, padding=12)
    frame.grid(sticky="nsew")
    frame.columnconfigure(1, weight=1)

    ttk.Label(frame, text="中文指令：").grid(row=0, column=0, sticky="w")
    instruction = tk.StringVar(value="拿起桌子上的水杯")
    ttk.Entry(frame, textvariable=instruction, width=48).grid(row=0, column=1, columnspan=3, sticky="ew")
    preview = tk.BooleanVar(value=True)
    ttk.Checkbutton(frame, text="仅规划预览（真实抓取前请取消）", variable=preview).grid(
        row=1, column=0, columnspan=2, sticky="w")

    task_id = tk.StringVar(value="")
    state = tk.StringVar(value="idle")
    step_state = tk.StringVar(value="idle")
    detail = tk.StringVar(value="等待任务")
    active_states = {"submitting", "busy", "detecting", "localized", "planning", "wait_confirm",
                     "pregrasp", "open", "wrist_verify", "approach", "close", "lift",
                     "verifying", "cancelling"}

    def update_buttons():
        current = state.get()
        start_button.configure(state="disabled" if current in active_states else "normal")
        confirm_button.configure(state="normal" if current == "wait_confirm" and task_id.get() else "disabled")
        cancel_button.configure(state="normal" if current in active_states and task_id.get() else "disabled")
        step_button.configure(
            state="normal" if current in {"idle", "wait_confirm"} and not preview.get()
            and step_state.get() not in {"submitting", "forward", "close", "lift"} else "disabled")

    def start():
        if state.get() in active_states:
            detail.set("当前任务尚未结束，请先等待或取消")
            return
        task_id.set("")
        state.set("submitting")
        detail.set("正在提交任务")
        update_buttons()
        # ROS 2 的一次性发布可能发生在订阅者尚未匹配时，导致界面永久停在“提交任务”。
        # 在收到状态前短暂重发，并设置超时反馈，避免静默卡死。
        payload = {"instruction": instruction.get(), "preview_only": preview.get()}
        attempts = {"count": 0}

        def publish_until_seen():
            if state.get() != "submitting":
                return
            attempts["count"] += 1
            node.send(node.goal_pub, payload)
            if attempts["count"] < 5:
                root.after(300, publish_until_seen)
            else:
                state.set("failed")
                detail.set("任务提交超时：未收到语义命令节点响应，请检查语义后端")
                update_buttons()

        publish_until_seen()

    def control(command):
        if not task_id.get():
            detail.set("当前没有可控制的活动任务")
            return
        node.send(node.control_pub, {"task_id": task_id.get(), "command": command})
        if command == "cancel":
            state.set("cancelling")
            detail.set("正在取消任务并停止动作")
            update_buttons()

    def step_grasp():
        if state.get() not in {"idle", "wait_confirm"} or preview.get():
            detail.set("请取消仅规划预览，并确认机械臂已经位于杯子前方")
            return
        if not messagebox.askyesno("确认真实动作", "即将执行：前进 5 cm、收紧夹爪、抬升 10 cm。确认继续吗？"):
            return
        step_task = task_id.get()
        step_state.set("submitting")
        detail.set("3 秒后执行真实动作，请准备急停")

        def countdown(seconds):
            if seconds <= 0:
                if step_task:
                    node.send(node.control_pub, {"task_id": step_task, "command": "cancel"})
                root.after(300 if step_task else 0, lambda: node.send(
                    node.step_pub, {"task_id": step_task, "command": "execute", "confirm": True}))
                return
            detail.set(f"{seconds} 秒后执行真实动作，请准备急停")
            root.after(1000, lambda: countdown(seconds - 1))

        countdown(3)
        update_buttons()

    start_button = ttk.Button(frame, text="开始识别与规划", command=start)
    confirm_button = ttk.Button(frame, text="确认完整抓取", command=lambda: control("confirm"))
    step_button = ttk.Button(frame, text="执行抓取", command=step_grasp)
    cancel_button = ttk.Button(frame, text="取消任务", command=lambda: control("cancel"))
    start_button.grid(row=2, column=0, pady=8)
    confirm_button.grid(row=2, column=1, pady=8)
    step_button.grid(row=2, column=2, pady=8)
    cancel_button.grid(row=2, column=3, pady=8)

    ttk.Label(frame, text="状态：").grid(row=3, column=0, sticky="nw")
    ttk.Label(frame, textvariable=state, foreground="#075985").grid(row=3, column=1, sticky="w")
    ttk.Label(frame, textvariable=step_state, foreground="#92400e").grid(row=3, column=2, sticky="w")
    ttk.Label(frame, textvariable=detail, wraplength=820).grid(row=4, column=0, columnspan=4, sticky="w")
    image_label = ttk.Label(frame, text="等待 D435i YOLOE 叠加图")
    image_label.grid(row=5, column=0, columnspan=4, pady=10)
    image_ref = {"value": None}

    def tick():
        while not events.empty():
            kind, value = events.get_nowait()
            if kind == "status":
                if value.get("task_id"):
                    task_id.set(str(value["task_id"]))
                if value.get("state"):
                    state.set(str(value["state"]))
                detail.set(f'{value.get("message", "")}  类别={value.get("target_class", "")}  置信度={float(value.get("confidence", 0.0)):.2f}')
                if state.get() in {"succeeded", "failed", "cancelled", "idle"}:
                    task_id.set("")
                update_buttons()
            elif kind == "step_status":
                step_state.set(str(value.get("state", "idle")))
                detail.set(f'独立动作：{value.get("message", "")}')
                update_buttons()
            elif kind == "image":
                image_ref["value"] = tk.PhotoImage(data=value)
                image_label.configure(image=image_ref["value"], text="")
        root.after(100, tick)

    preview.trace_add("write", lambda *_: update_buttons())
    update_buttons()
    tick()
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

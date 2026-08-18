"""人工确认界面：选择标签、确认或取消。不会绕过任务节点的安全门控。"""
import json
import queue
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ConfirmUiNode(Node):
    def __init__(self, events):
        super().__init__("openarm_grasp_confirm_ui")
        self.events = events
        self.publisher = self.create_publisher(String, "/openarm_vision/command", 10)
        self.create_subscription(String, "/openarm_vision/status", lambda msg: events.put(msg.data), 10)

    def command(self, data):
        message = String(); message.data = json.dumps(data, ensure_ascii=False); self.publisher.publish(message)


def main():
    rclpy.init(); events = queue.Queue(); node = ConfirmUiNode(events)
    import threading
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    root = tk.Tk(); root.title("OpenArm 视觉抓取确认（默认不执行）"); root.minsize(520, 230)
    frame = ttk.Frame(root, padding=14); frame.grid(sticky="nsew")
    ttk.Label(frame, text="物品 AprilTag ID：").grid(row=0, column=0, sticky="w")
    tag_id = tk.StringVar(value="0")
    ttk.Entry(frame, textvariable=tag_id, width=10).grid(row=0, column=1, sticky="w")
    status = tk.StringVar(value="等待视觉节点状态……")
    def send(kind):
        data = {"command": kind}
        if kind == "select": data["tag_id"] = int(tag_id.get())
        node.command(data)
    ttk.Button(frame, text="选择并预览", command=lambda: send("select")).grid(row=1, column=0, pady=12)
    ttk.Button(frame, text="确认执行", command=lambda: send("confirm")).grid(row=1, column=1, pady=12)
    ttk.Button(frame, text="取消任务", command=lambda: send("cancel")).grid(row=1, column=2, pady=12)
    ttk.Label(frame, textvariable=status, wraplength=480, foreground="#1f5f8b").grid(row=2, column=0, columnspan=3, sticky="w")
    def update():
        while not events.empty(): status.set(events.get_nowait())
        root.after(100, update)
    update()
    try: root.mainloop()
    finally: node.destroy_node(); rclpy.shutdown()

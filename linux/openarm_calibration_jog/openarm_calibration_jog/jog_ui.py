import json, queue, threading, tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Ui(Node):
 def __init__(self,q):
  super().__init__("openarm_calibration_jog_ui"); self.q=q; self.p=self.create_publisher(String,"/openarm_calibration_jog/command",10); self.create_subscription(String,"/openarm_calibration_jog/status",lambda m:q.put(m.data),10)
 def send(self,x): m=String();m.data=json.dumps(x);self.p.publish(m)
def main():
 rclpy.init();q=queue.Queue();n=Ui(q);threading.Thread(target=rclpy.spin,args=(n,),daemon=True).start();root=tk.Tk();root.title("OpenArm 左臂标定关节工具"); f=ttk.Frame(root,padding=12);f.grid();status=tk.StringVar(value="安全模式，默认不会运动")
 for i in range(7):
  ttk.Label(f,text=f"J{i+1}").grid(row=i,column=0); ttk.Button(f,text="−5°",command=lambda x=i:n.send({"command":"jog","joint":x+1,"delta_deg":-5})).grid(row=i,column=1); ttk.Button(f,text="+5°",command=lambda x=i:n.send({"command":"jog","joint":x+1,"delta_deg":5})).grid(row=i,column=2)
 ttk.Button(f,text="记录手眼样本",command=lambda:n.send({"command":"record"})).grid(row=7,column=0,columnspan=3,pady=8);ttk.Label(f,textvariable=status,wraplength=360).grid(row=8,column=0,columnspan=3)
 def tick():
  while not q.empty():status.set(q.get_nowait())
  root.after(100,tick)
 tick();root.mainloop();n.destroy_node();rclpy.shutdown()

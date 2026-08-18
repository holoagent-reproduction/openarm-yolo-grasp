# YOLOE 语义抓取部署与验收指南

## 一、WSL2 GPU 推理服务

在 WSL2 中进入 `yoloe_inference_service` 目录后创建隔离环境并安装依赖：

```bash
python3 -m venv ~/venvs/openarm-yoloe
source ~/venvs/openarm-yoloe/bin/activate
pip install -r requirements.txt
```

三条命令分别创建 Python 环境、启用环境和安装 YOLOE HTTP 服务依赖。

启动服务：

```bash
python server.py --model yoloe-11s-seg.pt --device cuda:0 --port 8765
```

该命令使用 NVIDIA GPU 加载文本提示实例分割模型，并监听 WSL2 的 8765 端口。

在管理员 Windows PowerShell 中运行已生成的精确范围脚本：

```powershell
cd E:\Users\lenovo\Desktop\openarm
.\windows_scripts\配置YOLOE端口转发_管理员.ps1
```

第一条命令进入项目目录；第二条命令自动读取 WSL2 地址并更新转发。它只监听 Windows 的 `172.16.20.148:8765`，防火墙只允许 NUC `172.16.20.130` 访问，不会向其他设备开放。

普通 PowerShell 中启动服务：

```powershell
.\windows_scripts\启动YOLOE服务.ps1
```

该命令在 WSL2 中使用 RTX GPU 启动服务。NUC 配置已指向 `http://172.16.20.148:8765/infer`。WSL2 地址在重启后可能变化，此时重新执行管理员端口转发脚本即可。

## 二、NUC ROS 2 安全预览

NUC 用户目录中装有 NumPy 2.x，而 ROS Humble 的 OpenCV 使用系统 NumPy 1.x。新启动文件会自动设置 `PYTHONNOUSERSITE=1`，避免相机节点发生 NumPy 二进制接口冲突；手工运行单个节点时也应带上这个环境变量。

构建和启动命令见 `openarm_vision_grasp/README.md`。第一次必须使用：

```bash
ros2 launch openarm_vision_grasp semantic_grasp.launch.py allow_motion:=false
```

该命令允许识别与规划，但禁止发送真实运动。

## 三、分阶段验收

1. 离线识别：六类物品各 50 张图，单类召回率至少 90%。
2. 三维定位：10 个已测点，中位误差小于 15 mm。
3. 规划预演：六类各 20 次，轨迹不穿桌面。
4. 空载运动：至少 20 次无碰撞。
5. 水杯抓取：20 次中至少成功 18 次。
6. 扩展物品：其余每类至少 10 次，成功率至少 80%。

每一阶段不合格时停止进入下一阶段。

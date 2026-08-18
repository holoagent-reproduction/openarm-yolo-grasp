# OpenArm 一键启动脚本

本文件夹集中保存 Windows 端一键启动 OpenArm 视觉抓取所需脚本。脚本通过 SSH 连接 Linux 小主机 `172.16.13.202`，在 Linux 主机上启动 MoveIt、双相机、语义抓取后端、RViz 和语义抓取界面；YOLOE 推理服务运行在 Windows 的 WSL2 中。

## 文件说明

- `start_openarm_preview.cmd`：预览模式，只识别和规划，不执行机械臂运动。
- `start_openarm_real_grasp.cmd`：真实抓取模式，启动时 `allow_motion=true`。
- `openarm_one_click.ps1`：主启动脚本，负责 YOLOE 检查、端口转发和 SSH 启动 Linux 端程序。
- `openarm_remote_launcher.py`：通过 SSH 在 Linux 主机启动和清理 ROS 2 节点。
- `start_yoloe_service.ps1`：在 WSL2 中启动 YOLOE 服务。
- `configure_yoloe_portproxy.ps1`：配置 Windows 到 WSL2 的 8765 端口转发，需要管理员权限。

## 使用前检查

1. Windows 已安装并启动 WSL2 的 `Ubuntu-22.04`。
2. WSL2 中已经配置 YOLOE 虚拟环境和模型。
3. Windows 与 NUC 通过网线连接，NUC 地址为 `172.16.13.202`。
4. NUC 已上电，机械臂处于安全状态，急停可用。
5. NUC 的 Linux 桌面显示器已登录，因为 MoveIt、RViz、相机画面和语义界面会显示在 Linux 主机屏幕，不显示在 Windows 桌面。
6. 启动时会在终端安全提示输入 NUC 的 SSH 密码，密码不会写入脚本。

## 推荐启动顺序

### 预览模式

双击：

```text
start_openarm_preview.cmd
```

预览模式会启动：

- YOLOE 推理服务；
- Windows 到 WSL2 的 8765 端口转发；
- NUC 上的 OpenArm MoveIt/RViz；
- D435i 头部相机和 D415 左腕相机；
- YOLOE 语义检测和三维定位；
- Linux 主机上的语义抓取确认界面。

此模式只规划，不发送真实机械臂运动命令，适合先确认目标位置、夹爪姿态和轨迹。

### 真实抓取模式

确认预览轨迹正确、机械臂和夹爪无异常后，双击：

```text
start_openarm_real_grasp.cmd
```

该模式会以 `allow_motion=true` 启动真实动作。界面仍然要求人工确认，不能把手放入机械臂工作区，并且必须随时准备急停。

## PowerShell 手动启动

在本文件夹打开 PowerShell：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\start_openarm_preview.cmd
```

第一行只对当前 PowerShell 窗口临时允许脚本运行；第二行启动预览模式。

真实模式命令：

```powershell
Set-ExecutionPolicy -Scope Process Bypass
.\start_openarm_real_grasp.cmd
```

## 常用参数

直接运行主脚本时可以覆盖默认参数：

```powershell
.\openarm_one_click.ps1 -NucIp 172.16.13.202 -RealMotion
```

- `-NucIp`：Linux 主机 IP，默认 `172.16.13.202`。
- `-RealMotion`：启用真实运动；不加时为预览模式。
- `-SkipYoloE`：跳过启动 YOLOE，适用于 YOLOE 已经运行的情况。
- `-SkipPortForward`：跳过端口转发，适用于端口已经配置好的情况。

## 画面位置

- MoveIt/RViz：Linux NUC 屏幕。
- 相机画面：Linux NUC 屏幕上的相机或语义界面。
- YOLOE 服务窗口：Windows WSL2 对应的 PowerShell 窗口。
- Windows 本机不会显示 MoveIt 主界面。

## 常见问题

### 窗口提示找不到脚本

必须从本文件夹内运行 `.cmd` 文件，不要把脚本复制到其他目录单独运行，因为主脚本会调用同目录下的 Python 和 PowerShell 文件。

### SSH 密码提示

启动时会显示密码输入提示。输入时终端不会回显字符，这是正常的；密码只用于当前 SSH 连接，不会保存到文件。

### 提示 YOLOE 不健康

先确认 WSL2 中的 YOLOE 服务已经启动，或者重新运行 `start_openarm_preview.cmd`。服务健康地址是：

```text
http://127.0.0.1:8765/health
```

### MoveIt 界面没有出现

检查 NUC 是否登录图形桌面，以及 Linux 主机是否仍在运行 MoveIt。脚本不会把 RViz 窗口转发到 Windows。

### 任务卡在“正在提交任务”

当前界面已加入任务消息重发和提交超时处理。先等待几秒；若仍无响应，关闭旧的语义界面后重新运行启动脚本。

### 真实抓取前的最低检查

- MoveIt/RViz 已正常显示机器人；
- 两台相机有实时画面；
- YOLOE 能识别杯子；
- 预览规划成功；
- 夹爪张开和闭合方向正确；
- 机械臂周围无人、无障碍物，急停可用。

# YOLOE 推理服务（WSL2）

该服务只接收图像并返回 YOLOE 实例分割结果，不具备机械臂控制权限。

## 运行环境

YOLOE 服务运行在 Windows 的 WSL2 Ubuntu-22.04 中，NUC 不需要安装 YOLOE。推荐环境：

- Windows 11 + WSL2；
- Ubuntu 22.04；
- NVIDIA Windows 驱动支持 WSL CUDA；
- CUDA 可在 WSL2 中被 `nvidia-smi` 识别；
- Python 3.10；
- PyTorch 2.2 或更高版本；
- Ultralytics 8.3 或更高版本；
- NVIDIA GPU 显存建议至少 8 GB。

检查 WSL2 GPU：

```bash
nvidia-smi
```

该命令应显示 NVIDIA GPU 和驱动版本。如果找不到命令，应先安装支持 WSL2 的 NVIDIA 驱动。

## 安装

```bash
python3 -m venv ~/venvs/openarm-yoloe
source ~/venvs/openarm-yoloe/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

前三条命令依次创建虚拟环境、启用环境并升级 pip；最后一条安装 FastAPI、OpenCV、PyTorch、Ultralytics 和 YOLOE 依赖。PyTorch 应根据 NVIDIA 驱动和 CUDA 版本选择官方安装命令。

确认 PyTorch 能使用 GPU：

```bash
python -c "import torch; print('torch=',torch.__version__); print('cuda=',torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'NO GPU')"
```

必须显示 `cuda=True`，否则服务只能使用 CPU，无法满足实时抓取延迟要求。

## 模型权重

本仓库不直接提交几十到数百 MB 的模型二进制文件，仓库中提供模型目录和下载检查脚本。模型默认文件名为：

```text
models/yoloe-11s-seg.pt
```

在 WSL2 中执行：

```bash
mkdir -p ~/openarm_yoloe_service/models
source ~/venvs/openarm-yoloe/bin/activate
python -c "from ultralytics import YOLOE; YOLOE('/home/lenovo/openarm_yoloe_service/models/yoloe-11s-seg.pt')"
```

该命令让 Ultralytics 下载或加载 YOLOE 权重。首次运行需要网络，下载后可离线启动。若模型下载地址或许可证发生变化，应以 Ultralytics/YOLOE 官方发布页为准。

也可以把已有的 `yoloe-11s-seg.pt` 手动复制到 `~/openarm_yoloe_service/models/`。

## 启动

```bash
source ~/venvs/openarm-yoloe/bin/activate
python server.py --model ~/openarm_yoloe_service/models/yoloe-11s-seg.pt --device cuda:0 --port 8765
```

第一条命令启用推理环境；第二条命令加载本地 YOLOE 文本提示分割模型并监听 8765 端口。

## 检查

```bash
curl http://127.0.0.1:8765/health
```

该命令只检查服务和 CUDA 设备状态，不执行图像推理。

WSL2 服务需要通过 Windows 防火墙和端口转发暴露给 NUC。配置完成后，把 `semantic_grasp.yaml` 中的 `server_url` 改为 Windows 局域网地址，例如 `http://172.16.20.10:8765/infer`。

YOLOE/Ultralytics 的模型和代码许可证与本仓库不同，部署或分发前需要单独核对其许可证要求。

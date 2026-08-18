# YOLOE 模型权重

当前服务默认使用：

```text
yoloe-11s-seg.pt
```

请将权重放在：

```text
~/openarm_yoloe_service/models/yoloe-11s-seg.pt
```

权重文件没有直接提交到 Git 仓库，因为文件较大且许可证由 YOLOE/Ultralytics 单独规定。首次部署时，在 WSL2 中执行：

```bash
source ~/venvs/openarm-yoloe/bin/activate
mkdir -p ~/openarm_yoloe_service/models
python -c "from ultralytics import YOLOE; YOLOE('/home/lenovo/openarm_yoloe_service/models/yoloe-11s-seg.pt')"
```

执行完成后检查：

```bash
ls -lh ~/openarm_yoloe_service/models/yoloe-11s-seg.pt
```

如果已经从官方渠道取得权重，也可以直接复制到上述路径。不要把未经确认许可证的权重提交到公开 GitHub 仓库。

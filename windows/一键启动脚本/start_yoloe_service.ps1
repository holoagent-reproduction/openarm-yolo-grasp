$ErrorActionPreference = 'Stop'

$healthCommand = 'curl -fsS --max-time 2 http://127.0.0.1:8765/health >/dev/null'
& wsl.exe -d Ubuntu-22.04 -- bash -lc $healthCommand
if ($LASTEXITCODE -eq 0) {
    Write-Host 'YOLOE is already healthy on port 8765.'
    exit 0
}

$serviceDir = '/home/lenovo/openarm_yoloe_service'
$activateFile = '/home/lenovo/venvs/openarm-yoloe/bin/activate'
$serverFile = $serviceDir + '/server.py'
$modelFile = $serviceDir + '/models/yoloe-11s-seg.pt'

$checkCommand = "test -d '$serviceDir' && test -f '$activateFile' && test -f '$serverFile' && test -f '$modelFile'"
& wsl.exe -d Ubuntu-22.04 -- bash -lc $checkCommand
if ($LASTEXITCODE -ne 0) {
    throw 'YOLOE 服务、虚拟环境、server.py 或 models/yoloe-11s-seg.pt 缺失。请先准备模型权重。'
}

Write-Host 'Starting YOLOE. Keep this window open while using visual grasp.'
$command = "cd '$serviceDir' && source '$activateFile' && exec python '$serverFile' --model '$modelFile' --device cuda:0 --host 0.0.0.0 --port 8765"
& wsl.exe -d Ubuntu-22.04 -- bash -lc $command
if ($LASTEXITCODE -ne 0) {
    throw ('YOLOE exited unexpectedly, code ' + $LASTEXITCODE)
}

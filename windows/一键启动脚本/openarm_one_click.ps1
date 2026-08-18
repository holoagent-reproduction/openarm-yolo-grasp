param(
    [string]$NucIp = '172.16.13.202',
    [string]$WindowsLanIp = '172.16.13.1',
    [switch]$RealMotion,
    [switch]$SkipYoloE,
    [switch]$SkipPortForward
)

$ErrorActionPreference = 'Stop'
$helper = Join-Path $PSScriptRoot 'openarm_remote_launcher.py'
$yoloScript = Join-Path $PSScriptRoot 'start_yoloe_service.ps1'
$portScript = Join-Path $PSScriptRoot 'configure_yoloe_portproxy.ps1'

if (-not (Test-Path -LiteralPath $helper)) {
    throw 'openarm_remote_launcher.py is missing.'
}
if (-not (Test-Connection -ComputerName $NucIp -Count 1 -Quiet)) {
    throw ('Cannot ping NUC ' + $NucIp)
}

if (-not $SkipYoloE) {
    Start-Process powershell.exe -ArgumentList @('-NoExit', '-ExecutionPolicy', 'Bypass', '-File', $yoloScript) | Out-Null
}

Write-Host 'Waiting for YOLOE model service (up to 120 seconds)...'
$yoloReady = $false
for ($attempt = 1; $attempt -le 120; $attempt++) {
    & wsl.exe -d Ubuntu-22.04 -- bash -lc 'curl -fsS --max-time 2 http://127.0.0.1:8765/health >/dev/null 2>&1'
    if ($LASTEXITCODE -eq 0) {
        $yoloReady = $true
        break
    }
    Start-Sleep -Seconds 1
}
if (-not $yoloReady) {
    throw 'YOLOE did not become healthy. Check the YOLOE PowerShell window; NUC startup was not attempted.'
}
Write-Host 'YOLOE model service is healthy.'

if (-not $SkipPortForward) {
    $portArgs = '-NoProfile -ExecutionPolicy Bypass -File "' + $portScript + '" -WindowsLanIp "' + $WindowsLanIp + '" -NucIp "' + $NucIp + '"'
    Start-Process powershell.exe -Verb RunAs -Wait -ArgumentList $portArgs | Out-Null
}

$forwardReady = Test-NetConnection -ComputerName $WindowsLanIp -Port 8765 -InformationLevel Quiet
if (-not $forwardReady) {
    throw ('Windows port forward is not reachable at ' + $WindowsLanIp + ':8765.')
}

$helperFull = [IO.Path]::GetFullPath($helper)
$driveLetter = $helperFull.Substring(0, 1).ToLowerInvariant()
$helperTail = $helperFull.Substring(2).Replace('\', '/')
$wslHelper = '/mnt/' + $driveLetter + $helperTail

$launcherArgs = @(
    '-d', 'Ubuntu-22.04', '--', 'python3', '-u', $wslHelper,
    '--host', $NucIp,
    '--user', 'nuc',
    '--windows-ip', $WindowsLanIp
)
if ($RealMotion) {
    $launcherArgs += '--real-motion'
}

Write-Host 'Starting OpenArm remote processes. The SSH password will be requested securely.'
& wsl.exe @launcherArgs

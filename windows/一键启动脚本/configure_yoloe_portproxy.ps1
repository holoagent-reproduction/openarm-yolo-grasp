param(
    [string]$WindowsLanIp = '172.16.13.1',
    [string]$NucIp = '172.16.13.202',
    [int]$Port = 8765
)

$ErrorActionPreference = 'Stop'
$identity = [Security.Principal.WindowsIdentity]::GetCurrent()
$principal = [Security.Principal.WindowsPrincipal]::new($identity)
if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw 'Administrator privileges are required.'
}

$raw = & wsl.exe -d Ubuntu-22.04 -- hostname -I
$wslIp = (($raw -replace ([char]0), '').Trim() -split '\s+')[0]
if (-not $wslIp) {
    throw 'Cannot determine the WSL2 address.'
}

$ruleName = 'OpenArm YOLOE 8765 NUC Only'
& netsh interface portproxy delete v4tov4 listenaddress=$WindowsLanIp listenport=$Port | Out-Null
& netsh interface portproxy add v4tov4 listenaddress=$WindowsLanIp listenport=$Port connectaddress=$wslIp connectport=$Port | Out-Null

Get-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue | Remove-NetFirewallRule
New-NetFirewallRule -DisplayName $ruleName -Direction Inbound -Action Allow -Protocol TCP -LocalAddress $WindowsLanIp -LocalPort $Port -RemoteAddress $NucIp -Profile Any | Out-Null

Write-Host ('Port proxy ready: ' + $WindowsLanIp + ':' + $Port + ' -> ' + $wslIp + ':' + $Port)

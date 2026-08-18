#!/usr/bin/env bash
# 初始化 ZLG 双通道 CAN-FD：右臂 can0、左臂 can1。
# 此脚本不启动 ROS、相机、VR、底盘或 RoboDriver。
set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DRIVER_SCRIPT="${SCRIPT_DIR}/usbcanfd200_400u_2.10/driver_load.sh"
readonly CAN_INTERFACES=(can0 can1)
readonly BITRATE=1000000
readonly DATA_BITRATE=5000000

if [[ ! -f "${DRIVER_SCRIPT}" ]]; then
    echo "错误：未找到 ZLG 驱动加载脚本：${DRIVER_SCRIPT}" >&2
    exit 1
fi

if ! command -v openarm-can-configure-socketcan >/dev/null 2>&1; then
    echo "错误：未找到 openarm-can-configure-socketcan，请先安装 OpenArm CAN 工具。" >&2
    exit 1
fi

echo "加载 ZLG 双通道驱动（can0=右臂，can1=左臂）..."
bash "${DRIVER_SCRIPT}"

for can_interface in "${CAN_INTERFACES[@]}"; do
    if ! ip link show dev "${can_interface}" >/dev/null 2>&1; then
        echo "错误：未发现 ${can_interface}。请检查 ZLG 转接盒、USB 连接和驱动加载日志。" >&2
        exit 1
    fi

    echo "配置 ${can_interface} 为 CAN-FD：仲裁域 ${BITRATE}、数据域 ${DATA_BITRATE}..."
    openarm-can-configure-socketcan "${can_interface}" -fd -b "${BITRATE}" -d "${DATA_BITRATE}"

    can_details="$(ip -details link show dev "${can_interface}")"
    if [[ "${can_details}" != *"bitrate ${BITRATE}"* || "${can_details}" != *"dbitrate ${DATA_BITRATE}"* ]]; then
        echo "错误：${can_interface} 未按预期配置为 CAN-FD ${BITRATE}/${DATA_BITRATE}。" >&2
        echo "${can_details}" >&2
        exit 1
    fi
done

echo "双臂 CAN-FD 已就绪：右臂 can0，左臂 can1。"


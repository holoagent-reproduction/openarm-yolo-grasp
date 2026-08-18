#!/usr/bin/env python3
"""向本机 YOLOE 服务发送一张图片，验证完整 HTTP 推理链路。"""
import argparse
import json
from pathlib import Path

import requests


def main():
    parser = argparse.ArgumentParser(description="YOLOE HTTP 推理冒烟测试")
    parser.add_argument("image", type=Path)
    parser.add_argument("--url", default="http://127.0.0.1:8765/infer")
    parser.add_argument("--prompt", action="append", default=[])
    args = parser.parse_args()
    prompts = args.prompt or ["cup"]
    with args.image.open("rb") as stream:
        response = requests.post(
            args.url,
            files={"image": (args.image.name, stream, "image/jpeg")},
            data={
                "prompts": json.dumps(prompts),
                "confidence": "0.40",
                "iou": "0.60",
                "image_size": "640",
            },
            timeout=120,
        )
    response.raise_for_status()
    result = response.json()
    print(json.dumps({
        "图像宽度": result.get("width"),
        "图像高度": result.get("height"),
        "推理耗时毫秒": result.get("elapsed_ms"),
        "检测数量": len(result.get("detections", [])),
        "检测类别": [item.get("class_name") for item in result.get("detections", [])],
        "掩码非空": [bool(item.get("mask_png_base64")) for item in result.get("detections", [])],
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

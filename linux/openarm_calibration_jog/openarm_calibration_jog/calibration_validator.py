#!/usr/bin/env python3
"""使用独立样本验证 eye-in-hand 外参，不重新求解外参。"""
import argparse
import json
from pathlib import Path

import numpy as np

from .calibration_solver import inverse, matrix, quaternion, residuals


def validate(samples, result):
    calibrated_transform = matrix({
        "translation": result["translation_m"],
        "rotation_xyzw": result["rotation_xyzw"],
    })
    mode = result.get("mode", "eye_in_hand")
    invariant_poses = []
    details = []
    for number, sample in enumerate(samples, start=1):
        world_to_hand = matrix(sample["hand_in_world"])
        camera_to_tag = matrix(sample["tag_in_camera"])
        if mode == "eye_in_hand":
            invariant = world_to_hand @ calibrated_transform @ camera_to_tag
            invariant_name = "tag_in_world"
        elif mode == "eye_to_hand":
            invariant = inverse(world_to_hand) @ calibrated_transform @ camera_to_tag
            invariant_name = "tag_in_hand"
        else:
            raise ValueError(f"不支持的标定模式：{mode}")
        invariant_poses.append(invariant)
        details.append({
            "sample_number": number,
            f"{invariant_name}_translation_m": invariant[:3, 3].tolist(),
            f"{invariant_name}_rotation_xyzw": quaternion(invariant[:3, :3]),
        })
    center, translation_error, rotation_error = residuals(invariant_poses)
    for detail, translation_mm, rotation_deg in zip(details, translation_error, rotation_error):
        detail["translation_error_mm"] = float(translation_mm)
        detail["rotation_error_deg"] = float(rotation_deg)
    return center, translation_error, rotation_error, details


def main():
    parser = argparse.ArgumentParser(description="验证 OpenArm 左腕 D415 手眼标定外参")
    parser.add_argument("samples", type=Path, help="独立复测的新格式同步样本")
    parser.add_argument("result", type=Path, help="清洗后求解结果 JSON")
    parser.add_argument("--output", type=Path, required=True, help="验证报告 JSON")
    parser.add_argument("--min-samples", type=int, default=5)
    parser.add_argument("--max-translation-rms-mm", type=float, default=10.0)
    parser.add_argument("--max-rotation-rms-deg", type=float, default=3.0)
    args = parser.parse_args()

    samples = json.loads(args.samples.read_text(encoding="utf-8"))
    result = json.loads(args.result.read_text(encoding="utf-8"))
    expected_role = "head" if result.get("mode") == "eye_to_hand" else "wrist"
    valid = [
        value for value in samples
        if "hand_in_world" in value
        and "tag_in_camera" in value
        and value.get("camera_role") == expected_role
        and value["tag_in_camera"].get("frame_id") == result.get("child_frame")
    ]
    if len(valid) < args.min_samples:
        raise SystemExit(f"有效独立复测样本只有 {len(valid)} 组，至少需要 {args.min_samples} 组")

    center, translation_error, rotation_error, details = validate(valid, result)
    translation_rms = float(np.sqrt(np.mean(translation_error ** 2)))
    rotation_rms = float(np.sqrt(np.mean(rotation_error ** 2)))
    accepted = (
        translation_rms <= args.max_translation_rms_mm
        and rotation_rms <= args.max_rotation_rms_deg
    )
    output = {
        "accepted": accepted,
        "mode": result.get("mode", "eye_in_hand"),
        "camera_role": expected_role,
        "sample_count": len(valid),
        "source_samples": str(args.samples),
        "source_calibration_result": str(args.result),
        "invariant": "tag_in_hand" if result.get("mode") == "eye_to_hand" else "tag_in_world",
        "validated_camera_frame": result.get("child_frame"),
        "invariant_mean_translation_m": center[:3, 3].tolist(),
        "invariant_mean_rotation_xyzw": quaternion(center[:3, :3]),
        "quality": {
            "translation_rms_mm": translation_rms,
            "translation_max_mm": float(np.max(translation_error)),
            "rotation_rms_deg": rotation_rms,
            "rotation_max_deg": float(np.max(rotation_error)),
        },
        "thresholds": {
            "max_translation_rms_mm": args.max_translation_rms_mm,
            "max_rotation_rms_deg": args.max_rotation_rms_deg,
        },
        "samples": details,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(output, ensure_ascii=False, indent=2))
    if not accepted:
        raise SystemExit("独立静态复测未通过，外参不可写入正式抓取配置")


if __name__ == "__main__":
    main()

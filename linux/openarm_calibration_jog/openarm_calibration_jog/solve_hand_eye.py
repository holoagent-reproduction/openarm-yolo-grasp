#!/usr/bin/env python3
"""由记录的 OpenArm 左臂样本计算左手到 D415 的手眼外参。"""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np


def transform(item):
    """将记录的 position + xyzw 四元数转换为旋转矩阵与平移向量。"""
    q = np.asarray(item["rotation_xyzw"], dtype=np.float64)
    q /= np.linalg.norm(q)
    rotation = np.array([
        [1 - 2*(q[1]**2 + q[2]**2), 2*(q[0]*q[1] - q[2]*q[3]), 2*(q[0]*q[2] + q[1]*q[3])],
        [2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[0]**2 + q[2]**2), 2*(q[1]*q[2] - q[0]*q[3])],
        [2*(q[0]*q[2] - q[1]*q[3]), 2*(q[1]*q[2] + q[0]*q[3]), 1 - 2*(q[0]**2 + q[1]**2)],
    ], dtype=np.float64)
    return rotation, np.asarray(item["translation" if "translation" in item else "position"], dtype=np.float64).reshape(3, 1)


def quaternion_from_matrix(rotation):
    """将 3×3 旋转矩阵转换为 xyzw 四元数。"""
    k = np.array([
        [rotation[0, 0] - rotation[1, 1] - rotation[2, 2], rotation[1, 0] + rotation[0, 1], rotation[2, 0] + rotation[0, 2], rotation[1, 2] - rotation[2, 1]],
        [rotation[1, 0] + rotation[0, 1], rotation[1, 1] - rotation[0, 0] - rotation[2, 2], rotation[2, 1] + rotation[1, 2], rotation[2, 0] - rotation[0, 2]],
        [rotation[2, 0] + rotation[0, 2], rotation[2, 1] + rotation[1, 2], rotation[2, 2] - rotation[0, 0] - rotation[1, 1], rotation[0, 1] - rotation[1, 0]],
        [rotation[1, 2] - rotation[2, 1], rotation[2, 0] - rotation[0, 2], rotation[0, 1] - rotation[1, 0], rotation[0, 0] + rotation[1, 1] + rotation[2, 2]],
    ]) / 3.0
    values, vectors = np.linalg.eigh(k)
    q = vectors[:, np.argmax(values)]
    if q[3] < 0:
        q = -q
    return q.tolist()


def score(samples, hand_to_camera_r, hand_to_camera_t):
    """标签在 world 中应为常量；以其离散程度评价解的质量。"""
    poses = []
    for hand, tag in samples:
        r, t = transform(hand)
        rt, tt = transform(tag)
        poses.append((r @ hand_to_camera_r @ rt, r @ (hand_to_camera_r @ tt + hand_to_camera_t) + t))
    positions = np.array([p[1].ravel() for p in poses])
    center = positions.mean(axis=0)
    translation_rms_mm = float(np.sqrt(np.mean(np.sum((positions - center) ** 2, axis=1))) * 1000)
    ref = poses[0][0]
    angles = []
    for rotation, _ in poses:
        delta = ref.T @ rotation
        angles.append(math.degrees(math.acos(np.clip((np.trace(delta) - 1) / 2, -1, 1))))
    return translation_rms_mm, float(np.sqrt(np.mean(np.square(angles)))), poses


def main():
    parser = argparse.ArgumentParser(description="计算 OpenArm 左腕 D415 手眼标定")
    parser.add_argument("samples", type=Path)
    parser.add_argument("--output", type=Path, default=Path("left_hand_to_d415.json"))
    args = parser.parse_args()
    raw = json.loads(args.samples.read_text(encoding="utf-8"))
    samples = [(s["hand_in_base"], s["tag_in_wrist_camera"]) for s in raw if s.get("tag_in_wrist_camera", {}).get("role") == "wrist"]
    if len(samples) < 8:
        raise SystemExit(f"有效左腕样本只有 {len(samples)} 组，至少需要 8 组。")
    r_gripper, t_gripper, r_target, t_target = [], [], [], []
    for hand, tag in samples:
        rh, th = transform(hand)
        rt, tt = transform(tag)
        r_gripper.append(rh); t_gripper.append(th)
        r_target.append(rt); t_target.append(tt)
    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    results = []
    for name, method in methods.items():
        try:
            rotation, translation = cv2.calibrateHandEye(r_gripper, t_gripper, r_target, t_target, method=method)
            mm, deg, poses = score(samples, rotation, translation)
            results.append((mm + 2 * deg, name, rotation, translation, mm, deg, poses))
        except cv2.error as exc:
            print(f"{name} 求解失败：{exc}")
    if not results:
        raise SystemExit("所有手眼标定算法都求解失败。")
    _, method, rotation, translation, mm, deg, poses = min(results, key=lambda x: x[0])
    output = {
        "说明": "left_hand 到 left_wrist_d415_color_optical_frame 的标定结果；尚未自动写入相机配置。",
        "parent_frame": "openarm_left_hand",
        "child_frame": "left_wrist_d415_color_optical_frame",
        "method": method,
        "sample_count": len(samples),
        "translation_m": translation.ravel().tolist(),
        "rotation_xyzw": quaternion_from_matrix(rotation),
        "quality": {"tag_world_translation_rms_mm": mm, "tag_world_rotation_rms_deg": deg},
        "tag_in_world_mean_translation_m": np.mean([p[1].ravel() for p in poses], axis=0).tolist(),
    }
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(output, ensure_ascii=False, indent=2))
    print("质量建议：平移 RMS 小于 10 mm、旋转 RMS 小于 3° 时，可进入独立复核。")


if __name__ == "__main__":
    main()

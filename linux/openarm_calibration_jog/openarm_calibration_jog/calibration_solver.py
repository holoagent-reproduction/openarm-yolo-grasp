#!/usr/bin/env python3
"""用 AX=XB 求解腕部 eye-in-hand 或头部 eye-to-hand，并输出质量门控结果。"""
import argparse
import json
import math
from pathlib import Path

import numpy as np


def matrix(value):
    q = np.asarray(value["rotation_xyzw"], dtype=float); q /= np.linalg.norm(q)
    x, y, z, w = q
    transform = np.eye(4)
    transform[:3, :3] = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    transform[:3, 3] = value.get("translation", value.get("position"))
    return transform


def inverse(value):
    result = np.eye(4); result[:3, :3] = value[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ value[:3, 3]
    return result


def rotation_angle(rotation):
    return math.degrees(math.acos(float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))))


def project_rotation(value):
    u, _, vt = np.linalg.svd(value); rotation = u @ vt
    if np.linalg.det(rotation) < 0: u[:, -1] *= -1; rotation = u @ vt
    return rotation


def solve_ax_xb(a_list, b_list):
    equations = [np.kron(np.eye(3), a[:3, :3]) - np.kron(b[:3, :3].T, np.eye(3))
                 for a, b in zip(a_list, b_list)]
    _, _, vt = np.linalg.svd(np.vstack(equations))
    rotation = project_rotation(vt[-1].reshape((3, 3), order="F"))
    lhs, rhs = [], []
    for a, b in zip(a_list, b_list):
        lhs.append(a[:3, :3] - np.eye(3)); rhs.append(rotation @ b[:3, 3] - a[:3, 3])
    translation = np.linalg.lstsq(np.vstack(lhs), np.hstack(rhs), rcond=None)[0]
    result = np.eye(4); result[:3, :3] = rotation; result[:3, 3] = translation
    return result


def average_pose(poses):
    result = np.eye(4); result[:3, :3] = project_rotation(sum(p[:3, :3] for p in poses))
    result[:3, 3] = np.median(np.array([p[:3, 3] for p in poses]), axis=0)
    return result


def residuals(poses):
    center = average_pose(poses)
    translation = np.array([np.linalg.norm(p[:3, 3] - center[:3, 3]) * 1000.0 for p in poses])
    rotation = np.array([rotation_angle(center[:3, :3].T @ p[:3, :3]) for p in poses])
    return center, translation, rotation


def quaternion(rotation):
    """把主动旋转矩阵转换为 ROS 使用的 xyzw 四元数。"""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = np.array([
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
            0.25 * s,
        ])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            q = np.array([0.25 * s, (rotation[0, 1] + rotation[1, 0]) / s,
                          (rotation[0, 2] + rotation[2, 0]) / s,
                          (rotation[2, 1] - rotation[1, 2]) / s])
        elif index == 1:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            q = np.array([(rotation[0, 1] + rotation[1, 0]) / s, 0.25 * s,
                          (rotation[1, 2] + rotation[2, 1]) / s,
                          (rotation[0, 2] - rotation[2, 0]) / s])
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            q = np.array([(rotation[0, 2] + rotation[2, 0]) / s,
                          (rotation[1, 2] + rotation[2, 1]) / s, 0.25 * s,
                          (rotation[1, 0] - rotation[0, 1]) / s])
    q /= np.linalg.norm(q)
    if q[3] < 0.0:
        q = -q
    return q.tolist()


def solve(samples, mode):
    hands = [matrix(s["hand_in_world"]) for s in samples]
    tags = [matrix(s["tag_in_camera"]) for s in samples]
    a_list, b_list = [], []
    for i in range(len(samples)):
        for j in range(i + 1, len(samples)):
            a = inverse(hands[j]) @ hands[i]
            b = (tags[j] @ inverse(tags[i])) if mode == "eye_in_hand" else (inverse(tags[j]) @ tags[i])
            if rotation_angle(a[:3, :3]) >= 2.0 or np.linalg.norm(a[:3, 3]) >= 0.02:
                a_list.append(a); b_list.append(b)
    if len(a_list) < 8: raise ValueError("有效姿态变化不足，至少需要 8 个运动对")
    x = solve_ax_xb(a_list, b_list)
    invariants = ([h @ x @ t for h, t in zip(hands, tags)] if mode == "eye_in_hand"
                  else [h @ x @ inverse(t) for h, t in zip(hands, tags)])
    center, t_error, r_error = residuals(invariants)
    return x, center, t_error, r_error


def main():
    parser = argparse.ArgumentParser(description="OpenArm 鲁棒手眼/眼在手外标定求解")
    parser.add_argument("samples", type=Path); parser.add_argument("--mode", choices=("eye_in_hand", "eye_to_hand"), required=True)
    parser.add_argument("--output", type=Path, required=True); parser.add_argument("--max-translation-rms-mm", type=float, default=10.0)
    parser.add_argument("--max-rotation-rms-deg", type=float, default=3.0); args = parser.parse_args()
    raw = json.loads(args.samples.read_text(encoding="utf-8"))
    selected = [(index, value) for index, value in enumerate(raw) if "hand_in_world" in value and "tag_in_camera" in value]
    if len(selected) < 12: raise SystemExit("有效同步样本少于 12 组")
    rejected = []
    while True:
        x, fixed, t_error, r_error = solve([v for _, v in selected], args.mode)
        t_rms = float(np.sqrt(np.mean(t_error ** 2))); r_rms = float(np.sqrt(np.mean(r_error ** 2)))
        if (t_rms <= args.max_translation_rms_mm and r_rms <= args.max_rotation_rms_deg) or len(selected) <= 12:
            break
        combined = t_error / args.max_translation_rms_mm + r_error / args.max_rotation_rms_deg
        worst = int(np.argmax(combined)); median = float(np.median(combined)); mad = float(np.median(np.abs(combined - median)))
        if combined[worst] <= median + max(0.5, 3.0 * 1.4826 * mad) or len(rejected) >= max(1, len(raw) // 4):
            break
        rejected.append(selected[worst][0]); selected.pop(worst)
    accepted = t_rms <= args.max_translation_rms_mm and r_rms <= args.max_rotation_rms_deg
    transform = x if args.mode == "eye_in_hand" else fixed
    output = {
        "accepted": accepted, "mode": args.mode, "sample_count": len(selected), "rejected_sample_indices": rejected,
        "parent_frame": "openarm_left_hand_tcp" if args.mode == "eye_in_hand" else "world",
        "child_frame": "left_wrist_d415_color_optical_frame" if args.mode == "eye_in_hand" else "head_d435i_color_optical_frame",
        "translation_m": transform[:3, 3].tolist(), "rotation_xyzw": quaternion(transform[:3, :3]),
        "quality": {"translation_rms_mm": t_rms, "rotation_rms_deg": r_rms},
    }
    if args.mode == "eye_to_hand":
        output["hand_to_tag"] = {"translation_m": x[:3, 3].tolist(), "rotation_xyzw": quaternion(x[:3, :3])}
    args.output.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(output, ensure_ascii=False, indent=2))
    if not accepted: raise SystemExit("标定质量未达到门控标准，结果不可写入正式配置")


if __name__ == "__main__": main()

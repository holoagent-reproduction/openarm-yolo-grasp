import math
import unittest

import numpy as np

from openarm_calibration_jog.calibration_solver import inverse, matrix, quaternion, rotation_angle, solve
from openarm_calibration_jog.calibration_validator import validate


def pose(rx, ry, rz, translation):
    """根据 XYZ 欧拉角生成齐次变换，仅用于无噪声合成测试。"""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    rxm = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rym = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rzm = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3] = rzm @ rym @ rxm
    result[:3, 3] = translation
    return result


def as_record(value):
    rotation = value[:3, :3]
    trace = np.trace(rotation)
    w = math.sqrt(max(0.0, 1.0 + trace)) / 2.0
    x = (rotation[2, 1] - rotation[1, 2]) / (4.0 * w)
    y = (rotation[0, 2] - rotation[2, 0]) / (4.0 * w)
    z = (rotation[1, 0] - rotation[0, 1]) / (4.0 * w)
    return {"translation": value[:3, 3].tolist(), "rotation_xyzw": [x, y, z, w]}


class CalibrationSolverTest(unittest.TestCase):
    def setUp(self):
        self.hands = [
            pose(0.10 * i, -0.08 * (i % 4), 0.07 * (i % 5),
                 [0.30 + 0.025 * i, -0.18 + 0.018 * (i % 3), 0.42 + 0.012 * (i % 4)])
            for i in range(12)
        ]

    def assert_transform_close(self, actual, expected):
        self.assertLess(np.linalg.norm(actual[:3, 3] - expected[:3, 3]), 1e-6)
        self.assertLess(rotation_angle(expected[:3, :3].T @ actual[:3, :3]), 1e-4)

    def test_quaternion_round_trip(self):
        expected = pose(0.58, -0.31, 1.27, [0.0, 0.0, 0.0])
        actual = matrix({
            "translation": [0.0, 0.0, 0.0],
            "rotation_xyzw": quaternion(expected[:3, :3]),
        })
        self.assert_transform_close(actual, expected)

    def test_eye_in_hand(self):
        hand_to_camera = pose(0.12, -0.18, 0.07, [0.035, -0.022, 0.085])
        world_to_tag = pose(-0.05, 0.03, 0.22, [0.58, 0.12, 0.16])
        samples = []
        for hand in self.hands:
            camera_to_tag = inverse(hand_to_camera) @ inverse(hand) @ world_to_tag
            samples.append({"hand_in_world": as_record(hand), "tag_in_camera": as_record(camera_to_tag)})
        result, fixed, translation_error, rotation_error = solve(samples, "eye_in_hand")
        self.assert_transform_close(result, hand_to_camera)
        self.assert_transform_close(fixed, world_to_tag)
        self.assertLess(max(translation_error), 1e-5)
        self.assertLess(max(rotation_error), 1e-5)
        _, validation_translation, validation_rotation, _ = validate(samples, {
            "mode": "eye_in_hand",
            "translation_m": result[:3, 3].tolist(),
            "rotation_xyzw": quaternion(result[:3, :3]),
        })
        self.assertLess(max(validation_translation), 1e-5)
        self.assertLess(max(validation_rotation), 1e-5)

    def test_eye_to_hand(self):
        world_to_camera = pose(-0.09, 0.14, -0.20, [0.74, -0.31, 0.83])
        hand_to_tag = pose(0.06, -0.11, 0.15, [0.02, 0.01, 0.12])
        samples = []
        for hand in self.hands:
            camera_to_tag = inverse(world_to_camera) @ hand @ hand_to_tag
            samples.append({"hand_in_world": as_record(hand), "tag_in_camera": as_record(camera_to_tag)})
        result, fixed, translation_error, rotation_error = solve(samples, "eye_to_hand")
        self.assert_transform_close(result, hand_to_tag)
        self.assert_transform_close(fixed, world_to_camera)
        self.assertLess(max(translation_error), 1e-5)
        self.assertLess(max(rotation_error), 1e-5)
        _, validation_translation, validation_rotation, _ = validate(samples, {
            "mode": "eye_to_hand",
            "translation_m": fixed[:3, 3].tolist(),
            "rotation_xyzw": quaternion(fixed[:3, :3]),
        })
        self.assertLess(max(validation_translation), 1e-5)
        self.assertLess(max(validation_rotation), 1e-5)


if __name__ == "__main__":
    unittest.main()

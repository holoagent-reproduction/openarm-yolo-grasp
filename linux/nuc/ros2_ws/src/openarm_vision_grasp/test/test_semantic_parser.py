import unittest
from openarm_vision_grasp.semantic_parser import ParseError, parse_instruction


class SemanticParserTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = {
            "frames": {"world": "world"},
            "calibration": {"head_calibrated": False, "wrist_calibrated": False,
                            "tool_orientation_calibrated": False},
            "classes": {
                "cup": {"enabled": True, "中文": ["水杯", "杯子", "茶杯"]},
                "mug": {"enabled": True, "中文": ["马克杯", "带把杯"]},
                "plastic bottle": {"enabled": True, "中文": ["水瓶", "塑料瓶", "饮料瓶"]},
                "can": {"enabled": True, "中文": ["易拉罐", "罐子"]},
                "carton": {"enabled": True, "中文": ["纸盒", "饮料盒"]},
                "small box": {"enabled": True, "中文": ["小盒子", "包装盒"]},
            },
        }

    def test_supported_chinese_classes(self):
        cases = {
            "拿起桌子上的水杯": "cup",
            "抓取马克杯": "mug",
            "帮我拿住塑料瓶": "plastic bottle",
            "夹起易拉罐": "can",
            "拿起饮料盒": "carton",
            "捡起包装盒": "small box",
        }
        for instruction, expected in cases.items():
            self.assertEqual(parse_instruction(instruction, self.config)["class_name"], expected)

    def test_rejects_unsupported_action(self):
        with self.assertRaises(ParseError):
            parse_instruction("把水杯倒过来", self.config)

    def test_rejects_unknown_object(self):
        with self.assertRaises(ParseError):
            parse_instruction("拿起桌上的勺子", self.config)

    def test_real_motion_defaults_are_closed(self):
        calibration = self.config["calibration"]
        self.assertFalse(calibration["head_calibrated"])
        self.assertFalse(calibration["wrist_calibrated"])
        self.assertFalse(calibration["tool_orientation_calibrated"])
        self.assertEqual(self.config["frames"]["world"], "world")


if __name__ == "__main__":
    unittest.main()

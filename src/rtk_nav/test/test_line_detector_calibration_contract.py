# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import ast
import unittest
from pathlib import Path


SOURCE_PATH = Path(__file__).parents[1] / "rtk_nav" / "line_detector_node.py"


class LineDetectorCalibrationContractTest(unittest.TestCase):
    def test_default_effective_focal_length_matches_empirical_lateral_scale(self):
        tree = ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))
        focal_defaults = []

        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            if not isinstance(node.func, ast.Attribute):
                continue
            if node.func.attr != "declare_parameter" or len(node.args) < 2:
                continue
            if isinstance(node.args[0], ast.Constant) and node.args[0].value == "focal_length_px":
                focal_defaults.append(ast.literal_eval(node.args[1]))

        self.assertEqual(focal_defaults, [132.0])


if __name__ == "__main__":
    unittest.main()

"""Verify ROS2 parameter defaults used by the MQTT bridge."""

import ast
from pathlib import Path


def test_ca_cert_uses_a_string_default_for_ros2_parameter_typing():
    path = Path(__file__).parents[1] / "mqtt_ros2" / "mqtt_ros2_bridge.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))

    ca_cert_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "declare_parameter"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "ca_cert"
    ]

    assert len(ca_cert_calls) == 1
    assert isinstance(ca_cert_calls[0].args[1], ast.Constant)
    assert ca_cert_calls[0].args[1].value == ""

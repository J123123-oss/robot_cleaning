"""回归验证路径切换仅在 DISABLE 状态下被拒绝。"""

import ast
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import Mock


SOURCE = Path(__file__).parents[1] / "motor_control" / "motor_control.py"


def _load_method():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    node_class = next(
        item for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "MotorControlNode"
    )
    method = next(
        item for item in node_class.body
        if isinstance(item, ast.FunctionDef) and item.name == "handle_route_change"
    )
    namespace = {
        "os": __import__("os"),
        "String": SimpleNamespace,
    }
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), "exec"), namespace)
    return namespace["handle_route_change"]


def _make_controller(route_file, status):
    route_file.write_text("111.0,35.0,0\n", encoding="utf-8")
    publisher = Mock()
    logger = Mock()
    return SimpleNamespace(
        current_status=status,
        rtk_path_file=str(route_file),
        route_id="000",
        geofence_fault_latched=True,
        geofence_recovery_ready=True,
        nav_status="PAUSE",
        route_change_pub=publisher,
        get_logger=Mock(return_value=logger),
    ), publisher, logger


def test_enable_allows_route_change():
    handle_route_change = _load_method()
    with TemporaryDirectory() as temp_dir:
        controller, publisher, logger = _make_controller(
            Path(temp_dir) / "001.txt", "ENABLE"
        )

        handle_route_change(controller, "001")

        publisher.publish.assert_called_once()
        assert controller.route_id == "001"
        assert controller.nav_status == "IDLE"
        logger.warn.assert_not_called()


def test_disable_rejects_route_change_with_accurate_log():
    handle_route_change = _load_method()
    with TemporaryDirectory() as temp_dir:
        controller, publisher, logger = _make_controller(
            Path(temp_dir) / "001.txt", "DISABLE"
        )

        handle_route_change(controller, "001")

        publisher.publish.assert_not_called()
        logger.warn.assert_called_once_with(
            "[ROSNode] 当前状态是DISABLE，忽略路径切换指令"
        )

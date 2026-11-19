import ast
from pathlib import Path
from types import SimpleNamespace

import pytest


def make_driver():
    path = Path(__file__).parents[1] / "rtk_nav" / "wtrtk_serial_driver.py"
    tree = ast.parse(path.read_text(encoding="utf-8"))
    driver_class = next(node for node in tree.body if isinstance(node, ast.ClassDef))
    methods = [
        node for node in driver_class.body
        if isinstance(node, ast.FunctionDef)
        and node.name in {"parse_gga", "dms_to_decimal"}
    ]
    namespace = {
        "Header": lambda: SimpleNamespace(stamp=None, frame_id=""),
        "NavSatFix": type("NavSatFix", (), {
            "COVARIANCE_TYPE_APPROXIMATED": 1,
            "__init__": lambda self: setattr(self, "status", SimpleNamespace()),
        }),
        "NavSatStatus": SimpleNamespace(SERVICE_GPS=1),
    }
    body = [ast.ClassDef(name="Driver", bases=[], keywords=[], decorator_list=[], body=methods)]
    exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(path), "exec"), namespace)
    driver = namespace["Driver"]()
    warnings = []
    driver.get_logger = lambda: SimpleNamespace(warn=warnings.append)
    driver.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))
    return driver, warnings


@pytest.mark.parametrize("frame", [
    "$GNGGA,,,,,,0,0,283.17,0.00*7ETK,0.000,0.000,0.0,,,,,*78",
    "$GNGGA,,,,,,0,0,283.17,0.00*7Egarbage",
    "$GNGGA,,,,,,0,0,283.17,0.00*7",
    "$GNGGA,,,,,,0,0,283.17,0.00*ZZ",
])
def test_invalid_gga_frame_is_discarded_before_field_parsing(frame):
    driver, warnings = make_driver()

    assert driver.parse_gga(frame) is None
    assert not any("Failed to parse GGA fields" in warning for warning in warnings)


def test_valid_gga_frame_is_still_parsed():
    driver, warnings = make_driver()

    fix = driver.parse_gga("$GNGGA,,,,,,0,0,283.17,0.00*7E")

    assert fix is not None
    assert fix.status.status == 0
    assert fix.altitude == 0.0
    assert not warnings

import math

import pytest

from plot_rtk_points import apply_affine, fit_affine


CONTROL_POINTS = [
    {
        "name": "25",
        "px": 1372.5,
        "py": 813.4,
        "utm_e": 542680.6740828673,
        "utm_n": 3948871.7446467485,
    },
    {
        "name": "26",
        "px": 1347.7,
        "py": 258.9,
        "utm_e": 542685.4065246811,
        "utm_n": 3948929.8657571017,
    },
    {
        "name": "27",
        "px": 1175.7,
        "py": 271.9,
        "utm_e": 542664.0768334133,
        "utm_n": 3948931.623088899,
    },
]


def test_affine_fit_recovers_three_clicked_control_points():
    model = fit_affine(CONTROL_POINTS)

    assert model is not None
    assert model["count"] == 3
    assert model["rmse"] < 1e-6

    for point in CONTROL_POINTS:
        pixel = apply_affine(point, model)
        assert math.hypot(pixel["u"] - point["px"], pixel["v"] - point["py"]) < 1e-6


def test_affine_fit_rejects_collinear_controls():
    controls = [
        dict(CONTROL_POINTS[0], px=100.0, py=200.0),
        dict(CONTROL_POINTS[1], px=100.0, py=300.0),
        dict(CONTROL_POINTS[2], px=100.0, py=400.0),
    ]

    assert fit_affine(controls) is None


def test_affine_fit_reports_residual_for_inconsistent_control():
    controls = [dict(point) for point in CONTROL_POINTS]
    controls.append({
        "name": "28",
        "px": 1164.1 + 100.0,
        "py": 273.0,
        "utm_e": 542662.6348543823,
        "utm_n": 3948931.723012953,
    })

    model = fit_affine(controls)

    assert model is not None
    assert model["rmse"] > 20.0

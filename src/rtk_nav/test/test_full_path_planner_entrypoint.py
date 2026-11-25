from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]


def test_full_path_planner_entrypoint_targets_dense_implementation():
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    dense_source = PACKAGE_ROOT / "rtk_nav" / "full_path_planner_dense.py"
    legacy_source = PACKAGE_ROOT / "rtk_nav" / "full_path_planner.py"

    assert dense_source.is_file()
    assert "full_path_planner = rtk_nav.full_path_planner_dense:main" in setup_source
    assert not legacy_source.exists()

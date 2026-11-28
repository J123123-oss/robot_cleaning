#!/usr/bin/env python3
"""Generate the shared first-entry geofence from the dense full-area planner."""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path

import yaml

try:
    from .full_path_planner_dense import (
        DEFAULT_DENSE_SPACING,
        generate_cleaning_path_with_rotation_3points,
        generate_dense_from_utm,
    )
    from .geofence_geometry import build_geofence_payload, load_geofence_payload
except ImportError:
    from full_path_planner_dense import (
        DEFAULT_DENSE_SPACING,
        generate_cleaning_path_with_rotation_3points,
        generate_dense_from_utm,
    )
    from geofence_geometry import build_geofence_payload, load_geofence_payload


PACKAGE_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = PACKAGE_DIR / "config" / "full.yaml"
DEFAULT_OUTPUT_PATH = PACKAGE_DIR / "cleaning_path" / "common_entry_area.fence.json"
VALID_CORNERS = {"top_left", "top_right", "bottom_right", "bottom_left"}


def _load_areas(config_path: Path) -> list[dict]:
    with config_path.open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    if not isinstance(config, dict) or not isinstance(config.get("areas"), list):
        raise ValueError(f"配置文件缺少有效的 areas 列表: {config_path}")

    default_params = config.get("default") or {}
    if not isinstance(default_params, dict):
        raise ValueError("配置文件中的 default 必须是对象")

    areas = []
    for index, area in enumerate(config["areas"]):
        if not isinstance(area, dict):
            raise ValueError(f"区域 {index} 必须是对象")

        try:
            calib_a = (
                area["calib_point_a"]["lon"],
                area["calib_point_a"]["lat"],
            )
            calib_b = (
                area["calib_point_b"]["lon"],
                area["calib_point_b"]["lat"],
            )
            calib_c = (
                area["calib_point_c"]["lon"],
                area["calib_point_c"]["lat"],
            )
        except (KeyError, TypeError) as exc:
            raise ValueError(f"区域 {index} 缺少有效的三点标定数据") from exc

        if calib_a[0] == 0.0 or calib_a[1] == 0.0:
            raise ValueError(f"区域 {index} 的 calib_point_a 未配置")

        params = dict(default_params)
        for key in (
            "interval",
            "start_corner",
            "swap_wh_select",
            "edge_distance_lon",
            "edge_distance_lat",
            "end_corner_mode",
            "reverse_end_point",
        ):
            if key in area:
                params[key] = area[key]

        params["dense_spacing"] = area.get(
            "dense_spacing",
            DEFAULT_DENSE_SPACING,
        )
        start_corner = params.get("start_corner")
        if start_corner not in VALID_CORNERS:
            raise ValueError(
                f"区域 {index} 的 start_corner 无效: {start_corner!r}"
            )

        areas.append(
            {
                "index": index,
                "name": area.get("name", f"area_{index}"),
                "calib_point_a": calib_a,
                "calib_point_b": calib_b,
                "calib_point_c": calib_c,
                "param": params,
            }
        )

    if not areas:
        raise ValueError(f"配置文件没有区域: {config_path}")
    return areas


def generate_payload(config_path: str | os.PathLike[str]) -> dict:
    """Generate a fence payload using the dense planner's area geometry."""
    areas = _load_areas(Path(config_path).expanduser())
    area_names = []
    inner_corners = []
    area_paths = []
    utm_zone = None

    for area in areas:
        path_latlon, path_utm, _, area_inner_corners, area_zone = (
            generate_cleaning_path_with_rotation_3points(
                area["calib_point_a"],
                area["calib_point_b"],
                area["calib_point_c"],
                area["param"]["start_corner"],
                area["param"],
            )
        )
        del path_latlon

        if not path_utm:
            raise ValueError(f"区域 {area['name']} 未生成路径")
        if utm_zone is None:
            utm_zone = tuple(area_zone)
        elif tuple(area_zone) != utm_zone:
            raise ValueError(
                f"区域 {area['name']} 的 UTM 分带 {area_zone} "
                f"与前面区域的 {utm_zone} 不一致"
            )

        area_names.append(area["name"])
        inner_corners.append(area_inner_corners)
        area_paths.append((area["name"], area["param"], path_utm))

    bridge_tracks = []
    for previous, current in zip(area_paths, area_paths[1:]):
        previous_name, _, previous_path = previous
        current_name, current_params, current_path = current
        bridge_tracks.append(
            {
                "name": f"{previous_name}_to_{current_name}",
                "from_area": previous_name,
                "to_area": current_name,
                "centerline_utm": generate_dense_from_utm(
                    [previous_path[-1], current_path[0]],
                    current_params["dense_spacing"],
                ),
            }
        )

    return build_geofence_payload(
        area_names,
        inner_corners,
        bridge_tracks,
        utm_zone,
        area_expansion_radius_m=0.6,
        bridge_expansion_radius_m=0.3,
    )


def write_payload(payload: dict, output_path: str | os.PathLike[str]) -> None:
    """Write JSON atomically so an interrupted update keeps the old file."""
    destination = Path(output_path).expanduser()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=destination.parent,
            prefix=f".{destination.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_path = Path(temporary_file.name)
            json.dump(payload, temporary_file, ensure_ascii=False, indent=2)
            temporary_file.write("\n")
        os.replace(temporary_path, destination)
    finally:
        if temporary_path is not None and temporary_path.exists():
            temporary_path.unlink()


def ensure_generated(
    config_path: str | os.PathLike[str] | None = None,
    output_path: str | os.PathLike[str] | None = None,
) -> bool:
    """Refresh the generated fence when the source configuration is newer."""
    source = Path(config_path or DEFAULT_CONFIG_PATH).expanduser()
    destination = Path(output_path or DEFAULT_OUTPUT_PATH).expanduser()
    if not source.is_file():
        return False
    if destination.is_file() and destination.stat().st_mtime >= source.stat().st_mtime:
        return False

    payload = generate_payload(source)
    write_payload(payload, destination)
    return True


def main(args: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="根据 full.yaml 的现有稠密规划器几何生成公共首点接入围栏"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=f"输入 YAML，默认 {DEFAULT_CONFIG_PATH}",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT_PATH,
        help=f"输出 JSON，默认 {DEFAULT_OUTPUT_PATH}",
    )
    options = parser.parse_args(args)

    payload = generate_payload(options.config)
    write_payload(payload, options.output)
    print(
        f"已生成 {options.output}，"
        f"区域 {len(payload.get('areas', []))} 个，"
        f"衔接走廊 {len(payload.get('bridges', []))} 条，"
        f"UTM {payload['utm_zone'][0]}{payload['utm_zone'][1]}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

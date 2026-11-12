#!/usr/bin/env python3
"""Generate an interactive drawing view for recorded RTK points.

When a background image is supplied the drawing is used as the coordinate
frame.  Two or more control points (image pixel + WGS84 position) are used to
fit a similarity transform, so every recorded point is placed on the drawing
at its true pixel position.  The generated page can also calibrate itself:
pick a recorded point, click its position on the drawing, and export the
result as JSON for the next run.
"""
import argparse
import base64
import json
import math
import mimetypes
import re
import struct
import sys
import tempfile
from pathlib import Path

RECORD_RE = re.compile(r"- seq:\s*(?P<seq>\d+)\s+(?P<body>.*?)(?=\n- seq:|\Z)", re.DOTALL)
FIELD_RE = re.compile(
    r"point:\s*\{lon:\s*(?P<lon>[-+0-9.eE]+),\s*lat:\s*(?P<lat>[-+0-9.eE]+)\}.*?"
    r"valid:\s*(?P<valid>true|false).*?position_status:\s*(?P<position>[-+0-9]+).*?"
    r"orientation_status:\s*(?P<orientation>[-+0-9]+).*?time:\s*\"(?P<time>[^\"]*)\"",
    re.DOTALL,
)
AREA_RE = re.compile(r'(?m)^\s*area:\s*"([^"]*)"')
LABEL_RE = re.compile(r"(?m)^\s*label:\s*([^\s#]+)")
_UTM_TRANSFORMERS = {}


def lonlat_to_utm(lon, lat):
    """Return UTM easting, northing and zone label for a WGS84 coordinate."""
    zone = int((lon + 180.0) / 6.0) + 1
    hemisphere = "N" if lat >= 0 else "S"
    key = (zone, hemisphere)
    transformer = _UTM_TRANSFORMERS.get(key)
    if transformer is None:
        try:
            from pyproj import Transformer
            epsg = (32600 if lat >= 0 else 32700) + zone
            transformer = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True)
        except Exception:
            transformer = False
        _UTM_TRANSFORMERS[key] = transformer
    if transformer:
        easting, northing = transformer.transform(lon, lat)
        return easting, northing, f"{zone}{hemisphere}"
    return (*_utm_fallback(lon, lat), f"{zone}{hemisphere}")


def _utm_fallback(lon, lat):
    """Pure-Python WGS84 UTM forward projection (used when pyproj is absent)."""
    a = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity = flattening * (2.0 - flattening)
    k0 = 0.9996
    zone = int((lon + 180.0) / 6.0) + 1
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    n = a / math.sqrt(1.0 - eccentricity * math.sin(lat_rad) ** 2)
    t = math.tan(lat_rad) ** 2
    c = eccentricity / (1.0 - eccentricity) * math.cos(lat_rad) ** 2
    aa = math.cos(lat_rad) * (lon_rad - lon0)
    m = a * ((1 - eccentricity / 4 - 3 * eccentricity ** 2 / 64 - 5 * eccentricity ** 3 / 256) * lat_rad
             - (3 * eccentricity / 8 + 3 * eccentricity ** 2 / 32 + 45 * eccentricity ** 3 / 1024) * math.sin(2 * lat_rad)
             + (15 * eccentricity ** 2 / 256 + 45 * eccentricity ** 3 / 1024) * math.sin(4 * lat_rad)
             - (35 * eccentricity ** 3 / 3072) * math.sin(6 * lat_rad))
    easting = k0 * n * (aa + (1 - t + c) * aa ** 3 / 6
                        + (5 - 18 * t + t ** 2 + 72 * c - 58 * eccentricity) * aa ** 5 / 120) + 500000.0
    northing = k0 * (m + n * math.tan(lat_rad) * (aa ** 2 / 2
                     + (5 - t + 9 * c + 4 * c ** 2) * aa ** 4 / 24
                     + (61 - 58 * t + t ** 2 + 600 * c - 330 * eccentricity) * aa ** 6 / 720))
    if lat < 0:
        northing += 10000000.0
    return easting, northing


def with_utm(point):
    if point.get("utm_e") is None or point.get("utm_n") is None:
        easting, northing, zone = lonlat_to_utm(float(point["lon"]), float(point["lat"]))
        point["utm_e"] = easting
        point["utm_n"] = northing
        point["utm_zone"] = zone
    else:
        point["utm_e"] = float(point["utm_e"])
        point["utm_n"] = float(point["utm_n"])
    return point


def load_points(path):
    text = Path(path).read_text(encoding="utf-8")
    points = []
    for record in RECORD_RE.finditer(text):
        body = record.group("body")
        fields = FIELD_RE.search(body)
        if not fields:
            continue
        area = AREA_RE.search(body)
        label = LABEL_RE.search(body)
        points.append(with_utm({
            "seq": int(record.group("seq")),
            "lon": fields.group("lon"),
            "lat": fields.group("lat"),
            "valid": fields.group("valid") == "true",
            "position": int(fields.group("position")),
            "orientation": int(fields.group("orientation")),
            "time": fields.group("time"),
            "area": area.group(1) if area else "",
            "label": label.group(1) if label else "",
        }))
    return points


def solve_linear(matrix, vector):
    """Solve a small dense system with Gauss-Jordan elimination."""
    size = len(vector)
    rows = [list(matrix[i]) + [vector[i]] for i in range(size)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(rows[row][column]))
        if abs(rows[pivot][column]) < 1e-12:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]
        divisor = rows[column][column]
        for index in range(column, size + 1):
            rows[column][index] /= divisor
        for row in range(size):
            if row == column:
                continue
            factor = rows[row][column]
            if abs(factor) < 1e-15:
                continue
            for index in range(column, size + 1):
                rows[row][index] -= factor * rows[column][index]
    return [rows[i][size] for i in range(size)]


def fit_similarity(control_points):
    """Fit pixel -> UTM: x = a*u - b*v + tx, y = b*u + a*v + ty."""
    if len(control_points) < 2:
        return None
    control_points = [with_utm(point) for point in control_points]
    east0 = sum(point["utm_e"] for point in control_points) / len(control_points)
    north0 = sum(point["utm_n"] for point in control_points) / len(control_points)
    ata = [[0.0] * 4 for _ in range(4)]
    atb = [0.0] * 4
    for point in control_points:
        x = point["utm_e"] - east0
        y = point["utm_n"] - north0
        u, v = point["px"], point["py"]
        rows = ((u, -v, 1.0, 0.0, x), (v, u, 0.0, 1.0, y))
        for row in rows:
            for i in range(4):
                for j in range(4):
                    ata[i][j] += row[i] * row[j]
                atb[i] += row[i] * row[4]
    solution = solve_linear(ata, atb)
    if solution is None:
        return None
    a, b, tx, ty = solution
    determinant = a * a + b * b
    if determinant < 1e-12:
        return None
    residuals = []
    for point in control_points:
        x = point["utm_e"] - east0
        y = point["utm_n"] - north0
        dx, dy = x - tx, y - ty
        u = (a * dx + b * dy) / determinant
        v = (-b * dx + a * dy) / determinant
        residuals.append(math.hypot(u - point["px"], v - point["py"]))
    return {
        "a": a,
        "b": b,
        "tx": tx,
        "ty": ty,
        "det": determinant,
        "e0": east0,
        "n0": north0,
        "pixels_per_meter": 1.0 / math.sqrt(determinant),
        "rotation_deg": math.degrees(math.atan2(b, a)),
        "residuals": residuals,
        "rmse": math.sqrt(sum(value * value for value in residuals) / len(residuals)),
        "count": len(control_points),
    }


def parse_control_point(value):
    parts = value.split(",")
    if len(parts) != 5:
        raise ValueError(f"--control-point 需要 NAME,PX,PY,LON,LAT: {value}")
    name, px, py, lon, lat = parts
    return with_utm({
        "name": name.strip(),
        "px": float(px),
        "py": float(py),
        "lon": float(lon),
        "lat": float(lat),
    })


def load_calibration_metadata(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return data if isinstance(data, dict) else {}


def load_calibration(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if isinstance(data, dict):
        data = data.get("control_points") or data.get("points") or []
    if not isinstance(data, list):
        raise ValueError("标定文件需要 control_points 数组")
    controls = []
    for index, item in enumerate(data, 1):
        controls.append(with_utm({
            "name": str(item.get("name") or f"CP{index}"),
            "px": float(item["px"]),
            "py": float(item["py"]),
            "lon": float(item["lon"]),
            "lat": float(item["lat"]),
            "utm_e": item.get("utm_e"),
            "utm_n": item.get("utm_n"),
            "utm_zone": item.get("utm_zone"),
        }))
    return controls


def _svg_size(path):
    head = Path(path).read_text(encoding="utf-8", errors="ignore")[:4096]
    tag = re.search(r"<svg\b[^>]*>", head)
    if not tag:
        raise ValueError(f"无法解析 SVG 尺寸: {path}")
    attrs = dict(re.findall(r'(width|height|viewBox)\s*=\s*"([^"]*)"', tag.group(0)))
    def number(value):
        match = re.match(r"[-+0-9.eE]+", value.strip())
        if not match:
            raise ValueError(f"无法解析 SVG 尺寸: {path}")
        return float(match.group(0))
    if "width" in attrs and "height" in attrs:
        return round(number(attrs["width"])), round(number(attrs["height"]))
    if "viewBox" in attrs:
        parts = attrs["viewBox"].replace(",", " ").split()
        if len(parts) == 4:
            return round(float(parts[2])), round(float(parts[3]))
    raise ValueError(f"无法解析 SVG 尺寸: {path}")


def image_size(path):
    if Path(path).suffix.lower() == ".svg":
        return _svg_size(path)
    with open(path, "rb") as handle:
        head = handle.read(32)
    if head[:8] == b"\x89PNG\r\n\x1a\n":
        return struct.unpack(">II", head[16:24])
    if head[:2] == b"\xff\xd8":
        data = Path(path).read_bytes()
        index = 2
        while index + 9 < len(data):
            if data[index] != 0xFF:
                index += 1
                continue
            marker = data[index + 1]
            index += 2
            if marker in (0xD8, 0xD9):
                continue
            if index + 2 > len(data):
                break
            length = int.from_bytes(data[index:index + 2], "big")
            if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
                height = int.from_bytes(data[index + 3:index + 5], "big")
                width = int.from_bytes(data[index + 5:index + 7], "big")
                return width, height
            index += length
    raise ValueError(f"无法读取图片尺寸: {path}")


def image_content_bbox(path, size):
    if Path(path).suffix.lower() == ".svg":
        return [0, 0, size[0], size[1]]
    try:
        from PIL import Image, ImageChops
        with Image.open(path) as image:
            rgb = image.convert("RGB")
            background = Image.new("RGB", rgb.size, (255, 255, 255))
            bbox = ImageChops.difference(rgb, background).getbbox()
            if bbox:
                x0, y0, x1, y1 = bbox
                if x1 > x0 and y1 > y0:
                    return [x0, y0, x1, y1]
    except Exception:
        pass
    return [0, 0, size[0], size[1]]


SHEET_TEMPLATE = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RTK 点位 · 图纸</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
html,body{height:100%;margin:0}
body{font-family:"Microsoft YaHei",system-ui,sans-serif;color:#1f2933}
#app{display:flex;height:100%}
#sidebar{width:340px;flex:0 0 340px;box-sizing:border-box;padding:14px 16px;overflow-y:auto;background:#f8fafc;border-right:1px solid #d9e2ec}
#sidebar.collapsed{display:none}
.sidebar-header,.vc-header{display:flex;align-items:center;justify-content:space-between;gap:8px}
.sidebar-header{margin-bottom:8px}
#sidebar h1{font-size:16px;margin:0}
.panel-toggle{flex:0 0 auto;margin:0;padding:3px 7px;font-size:12px}
#sidebar-reopen{display:none;position:fixed;z-index:1200;top:10px;left:56px;margin:0;box-shadow:0 2px 10px rgba(0,0,0,.25)}
#sidebar-reopen.visible{display:block}
#status{font-size:12px;line-height:1.6;color:#334e68;margin-bottom:8px;white-space:pre-line}
#banner{font-size:12px;line-height:1.5;padding:8px;border-radius:4px;margin-bottom:8px}
#banner.warn{background:#fff7e6;border:1px solid #f0b429;color:#8d2b0b}
#banner.ok{background:#e6f6ec;border:1px solid #2f9e44;color:#1b5e20}
#banner.error{background:#fdecea;border:1px solid #d64545;color:#8a1c1c}
#holdout{margin:0 0 8px}
#holdout.holdout-ok{color:#1b5e20;font-weight:700}
#holdout.holdout-warn{color:#8d2b0b;font-weight:700}
#holdout.holdout-bad{color:#8a1c1c;font-weight:700}
button{font:13px inherit;padding:6px 10px;margin:4px 4px 4px 0;border:1px solid #9fb3c8;border-radius:4px;background:#fff;color:#102a43;cursor:pointer}
button:hover{background:#eef2f7}
button:disabled{color:#9aa5b1;background:#f0f4f8;cursor:not-allowed}
#point-select{width:100%;margin:6px 0;font:12px monospace;box-sizing:border-box}
#pairs{margin:8px 0}
.pair{display:grid;grid-template-columns:minmax(0,1fr) auto;gap:6px;align-items:center;font-size:12px;padding:5px 0;border-bottom:1px dashed #d9e2ec}
.pair small{color:#52606d;display:block;font-family:monospace}
.pair .res{font-family:monospace;margin-right:4px}
.pair .res.bad{color:#b91c1c;font-weight:700}
.hint{font-size:12px;line-height:1.6;color:#52606d}
#map{flex:1 1 auto;min-width:0;background:#e5e7eb}
.rtk-image-wrap{position:absolute;left:0;top:0}
.seq-label{background:#fff;border:1px solid #334e68;border-radius:3px;color:#102a43;font:700 13px/1.1 monospace;padding:2px 4px;white-space:nowrap;box-shadow:0 1px 2px rgba(0,0,0,.22)}
.seq-label.approx{border-style:dashed;color:#627d98}
.seq-label.label-hidden{display:none}
.cp-label{background:#1565c0;color:#fff;border-radius:3px;font:700 12px/1 monospace;padding:3px 4px;white-space:nowrap;box-shadow:0 1px 2px rgba(0,0,0,.3)}
.leaflet-popup-content{font:12px/1.55 monospace;margin:10px 12px}
.popup-title{font-weight:700;font-size:13px}
.popup-record + .popup-record{margin-top:8px;padding-top:8px;border-top:1px dashed #bcccdc}
#view-controls{position:fixed;z-index:1200;top:10px;right:10px;width:236px;box-sizing:border-box;background:#fff;border:1px solid #d9e2ec;border-radius:6px;box-shadow:0 2px 10px rgba(0,0,0,.25);padding:10px 12px;font:12px/1.5 "Microsoft YaHei",system-ui,sans-serif;color:#102a43}
#view-controls .vc-title{font-weight:700}
#view-controls .vc-header{margin-bottom:6px}
#view-controls.collapsed{width:auto;padding:6px 8px}
#view-controls.collapsed .vc-header{margin-bottom:0}
#view-controls.collapsed .vc-body{display:none}
#view-controls label{display:block;margin:5px 0}
#view-controls input[type=range]{width:100%;margin:2px 0}
#view-controls .vc-buttons{display:flex;flex-wrap:wrap;gap:4px;margin-top:8px}
#view-controls button{margin:0;padding:4px 6px;font-size:12px}
#view-controls button.active{background:#1565c0;border-color:#1565c0;color:#fff}
#view-controls .vc-hint{font-size:11px;color:#627d98;margin-top:6px}
#coord-panel{position:fixed;z-index:1200;left:356px;bottom:16px;width:330px;box-sizing:border-box;background:#fff;border:1px solid #d9e2ec;border-radius:6px;box-shadow:0 2px 10px rgba(0,0,0,.25);padding:10px 12px;font:12px/1.6 "Microsoft YaHei",system-ui,sans-serif;color:#102a43;display:none}
#coord-panel.visible{display:block}
#coord-panel .coord-head{display:flex;align-items:center;justify-content:space-between;font-weight:700;margin-bottom:4px}
#coord-panel .coord-row{display:flex;align-items:center;justify-content:space-between;gap:8px}
#coord-panel .coord-value{font:700 13px monospace}
#coord-panel .coord-hint{font-size:11px;color:#627d98;margin-top:4px}
.dataset-row{display:block;font-size:12px;color:#334e68;margin:0 0 8px}
#dataset-select{width:100%;margin-top:4px;font:12px monospace;box-sizing:border-box}
</style></head><body>
<div id="app">
<aside id="sidebar">
<div class="sidebar-header">
  <h1>RTK 点位 · 图纸</h1>
  <button id="sidebar-collapse" class="panel-toggle" type="button" aria-controls="sidebar" aria-expanded="true" title="收起标定菜单">收起</button>
</div>
<label class="dataset-row">点位序列 <select id="dataset-select"></select></label>
<div class="hint" id="dataset-hint"></div>
<div id="status">加载中…</div>
<div id="banner" class="warn"></div>
<div id="holdout" class="hint"></div>
<div id="pairs"></div>
<div class="hint">标定方法：在下方选择一条采点，点击“在图纸上拾取”，再点击图纸上对应的位置。至少 2 个控制点即可对齐。</div>
<select id="point-select" size="10"></select>
<button id="pick-btn">在图纸上拾取所选点位</button>
<button id="export-btn">导出标定 JSON</button>
<button id="locate-btn">定位到点位</button>
<button id="coord-pick-btn">拾取坐标（估算经纬度）</button>
<button id="clear-btn">清空标定</button>
<div class="hint" id="pick-hint">图纸标签只显示序号；下拉列表附带区域/标签和经纬度。</div>
</aside>
<button id="sidebar-reopen" type="button" aria-controls="sidebar" title="展开标定菜单">显示标定</button>
<div id="map"></div>
<div id="coord-panel">
  <div class="coord-head"><span>点坐标（估算）</span><button id="coord-close" type="button">×</button></div>
  <div class="coord-row"><span class="coord-value" id="coord-value">—</span><button id="coord-copy" type="button">复制</button></div>
  <div class="coord-hint" id="coord-utm"></div>
  <div class="coord-hint">由当前标定反算，精度取决于控制点。</div>
</div>
<div id="view-controls">
  <div class="vc-header">
    <div class="vc-title">底图对齐</div>
    <button id="view-collapse" class="panel-toggle" type="button" aria-controls="view-controls" aria-expanded="true" title="收起或展开底图对齐菜单">收起</button>
  </div>
  <div class="vc-body">
  <label><input id="image-visible" type="checkbox" checked> 显示底图</label>
  <label>透明度 <input id="image-opacity" type="range" min="0" max="1" step="0.05" value="1"></label>
  <label>缩放 <input id="image-scale" type="range" min="0.25" max="3" step="0.05" value="1"> <span id="image-scale-value">1.00×</span></label>
  <label>旋转 <input id="image-rotation" type="range" min="-360" max="360" step="1" value="0"> <span id="image-angle">0°</span></label>
  <label>平移X <input id="image-offset-x" type="range" min="-10000" max="10000" step="5" value="0"> <span id="image-offset-x-value">0</span></label>
  <label>平移Y <input id="image-offset-y" type="range" min="-10000" max="10000" step="5" value="0"> <span id="image-offset-y-value">0</span></label>
  <div class="vc-buttons">
    <button id="flip-x">水平镜像</button>
    <button id="flip-y">垂直镜像</button>
    <button id="flip-180">旋转180°</button>
    <button id="reset-adjust">重置</button>
  </div>
  <div class="vc-hint" id="view-state"></div>
  <div class="vc-hint">滑动条和镜像按钮只调整底图显示，点位保持不动；不写入标定 JSON。</div>
  </div>
</div>
</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const DATASETS = __DATASETS__;
let datasetIndex = 0;
let POINTS = DATASETS[datasetIndex].points;
const IMAGE = __IMAGE__;
const INITIAL_PAIRS = __PAIRS__;
const DEFAULT_VIEW = __DEFAULT_VIEW__;
const IMAGE_W = IMAGE.width, IMAGE_H = IMAGE.height;
const DEFAULT_HINT = '图纸标签只显示序号；下拉列表附带区域/标签和经纬度。';
const map = L.map('map', {
  crs: L.CRS.Simple,
  minZoom: -8,
  maxZoom: 5,
  zoomSnap: 0.25,
  zoomDelta: 0.5,
  attributionControl: false
});
var rtkViewReady = false;
const AdjustableImageOverlay = L.ImageOverlay.extend({
  onAdd: function (map) {
    if (!this._image) this._initImage();
    if (!this._wrap) {
      this._wrap = L.DomUtil.create('div', 'rtk-image-wrap');
      this._image.style.width = '100%';
      this._image.style.height = '100%';
      this._image.style.display = 'block';
    }
    this._wrap.appendChild(this._image);
    this.getPane().appendChild(this._wrap);
    if (this.options.opacity < 1) this._updateOpacity();
    this._reset();
  },
  onRemove: function () {
    if (this._wrap && this._wrap.parentNode) L.DomUtil.remove(this._wrap);
  },
  _reset: function () {
    if (!this._map || !this._wrap) return;
    const bounds = L.bounds(
      this._map.latLngToLayerPoint(this._bounds.getNorthWest()),
      this._map.latLngToLayerPoint(this._bounds.getSouthEast())
    );
    const size = bounds.getSize();
    L.DomUtil.setPosition(this._wrap, bounds.min);
    this._wrap.style.width = size.x + 'px';
    this._wrap.style.height = size.y + 'px';
    this._image.style.transform = '';
    this._image.style.width = '100%';
    this._image.style.height = '100%';
    if (rtkViewReady) applyImageTransform();
  },
  getWrapper: function () {
    return this._wrap;
  }
});
const imageOverlay = new AdjustableImageOverlay(IMAGE.src, [[0, 0], [IMAGE_H, IMAGE_W]]).addTo(map);
const pointLayer = L.layerGroup().addTo(map);
const cpLayer = L.layerGroup().addTo(map);
const coordLayer = L.layerGroup().addTo(map);
const statusEl = document.getElementById('status');
const bannerEl = document.getElementById('banner');
const holdoutEl = document.getElementById('holdout');
const pairsEl = document.getElementById('pairs');
const selectEl = document.getElementById('point-select');
const pickBtn = document.getElementById('pick-btn');
const exportBtn = document.getElementById('export-btn');
const locateBtn = document.getElementById('locate-btn');
const clearBtn = document.getElementById('clear-btn');
const coordPickBtn = document.getElementById('coord-pick-btn');
const coordPanelEl = document.getElementById('coord-panel');
const coordValueEl = document.getElementById('coord-value');
const coordUtmEl = document.getElementById('coord-utm');
const coordCopyBtn = document.getElementById('coord-copy');
const coordCloseBtn = document.getElementById('coord-close');
const hintEl = document.getElementById('pick-hint');
const imageVisibleEl = document.getElementById('image-visible');
const imageOpacityEl = document.getElementById('image-opacity');
const imageScaleEl = document.getElementById('image-scale');
const imageScaleValueEl = document.getElementById('image-scale-value');
const imageRotationEl = document.getElementById('image-rotation');
const imageAngleEl = document.getElementById('image-angle');
const imageOffsetXEl = document.getElementById('image-offset-x');
const imageOffsetYEl = document.getElementById('image-offset-y');
const imageOffsetXValueEl = document.getElementById('image-offset-x-value');
const imageOffsetYValueEl = document.getElementById('image-offset-y-value');
const flipXBtn = document.getElementById('flip-x');
const flipYBtn = document.getElementById('flip-y');
const flip180Btn = document.getElementById('flip-180');
const resetAdjustBtn = document.getElementById('reset-adjust');
const viewStateEl = document.getElementById('view-state');
const view = Object.assign({
  scale: 1, rotation: 0, mirrorX: false, mirrorY: false, offsetX: 0, offsetY: 0
}, DEFAULT_VIEW);
rtkViewReady = true;
let pairs = INITIAL_PAIRS.map(pair => {
  const matches = dataset => dataset.points.find(item =>
    Math.abs(Number(item.lon) - Number(pair.lon)) < 1e-9 &&
    Math.abs(Number(item.lat) - Number(pair.lat)) < 1e-9);
  const point = matches(DATASETS[datasetIndex]) || DATASETS.map(matches).find(Boolean);
  return Object.assign({
    seq: point ? point.seq : null,
    area: point ? point.area : '',
    label: point ? point.label : '',
    utm_e: point ? point.utm_e : null,
    utm_n: point ? point.utm_n : null,
    utm_zone: point ? point.utm_zone : ''
  }, pair);
});
let transform = null;
let picking = false;
let coordPicking = false;
let labelEntries = [];
let lastHoldout = null;
const datasetSelectEl = document.getElementById('dataset-select');
const datasetHintEl = document.getElementById('dataset-hint');
function updateDatasetHint() {
  const dataset = DATASETS[datasetIndex];
  datasetHintEl.textContent = '数据源：' + dataset.file + ' · ' + dataset.points.length + ' 点';
}
function switchDataset(index) {
  if (!DATASETS[index]) return;
  datasetIndex = index;
  POINTS = DATASETS[index].points;
  approx = computeApprox();
  renderPoints();
  renderSelect();
  updateDatasetHint();
  requestAnimationFrame(refreshLabels);
}
DATASETS.forEach((dataset, index) => {
  const option = document.createElement('option');
  option.value = String(index);
  option.textContent = dataset.label + '（' + dataset.points.length + ' 点）';
  datasetSelectEl.appendChild(option);
});
datasetSelectEl.value = '0';
datasetSelectEl.onchange = () => switchDataset(Number(datasetSelectEl.value));
updateDatasetHint();

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, ch => ({'&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'}[ch]));
}
function mean(values) {
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}
function utmLocal(point, east0, north0) {
  return [Number(point.utm_e) - east0, Number(point.utm_n) - north0];
}
function solve4(matrix, vector) {
  const size = 4;
  const rows = matrix.map((row, index) => row.concat([vector[index]]));
  for (let column = 0; column < size; column++) {
    let pivot = column;
    for (let row = column + 1; row < size; row++) {
      if (Math.abs(rows[row][column]) > Math.abs(rows[pivot][column])) pivot = row;
    }
    if (Math.abs(rows[pivot][column]) < 1e-12) return null;
    [rows[column], rows[pivot]] = [rows[pivot], rows[column]];
    const divisor = rows[column][column];
    for (let index = column; index <= size; index++) rows[column][index] /= divisor;
    for (let row = 0; row < size; row++) {
      if (row === column) continue;
      const factor = rows[row][column];
      if (Math.abs(factor) < 1e-15) continue;
      for (let index = column; index <= size; index++) rows[row][index] -= factor * rows[column][index];
    }
  }
  return rows.map(row => row[size]);
}
function fitSimilarity(controls) {
  if (controls.length < 2) return null;
  const east0 = mean(controls.map(point => Number(point.utm_e)));
  const north0 = mean(controls.map(point => Number(point.utm_n)));
  const ata = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]];
  const atb = [0,0,0,0];
  controls.forEach(point => {
    const local = utmLocal(point, east0, north0);
    const u = Number(point.px), v = Number(point.py);
    const rows = [[u, -v, 1, 0, local[0]], [v, u, 0, 1, local[1]]];
    rows.forEach(row => {
      for (let i = 0; i < 4; i++) {
        for (let j = 0; j < 4; j++) ata[i][j] += row[i] * row[j];
        atb[i] += row[i] * row[4];
      }
    });
  });
  const solution = solve4(ata, atb);
  if (!solution) return null;
  const [a, b, tx, ty] = solution;
  const det = a * a + b * b;
  if (det < 1e-12) return null;
  const residuals = controls.map(point => {
    const local = utmLocal(point, east0, north0);
    const dx = local[0] - tx, dy = local[1] - ty;
    const u = (a * dx + b * dy) / det;
    const v = (-b * dx + a * dy) / det;
    return Math.hypot(u - Number(point.px), v - Number(point.py));
  });
  return {
    a, b, tx, ty, det, east0, north0,
    zone: controls[0].utm_zone || '',
    pixelsPerMeter: 1 / Math.sqrt(det),
    rotation: Math.atan2(b, a) * 180 / Math.PI,
    residuals,
    rmse: Math.sqrt(residuals.reduce((sum, value) => sum + value * value, 0) / residuals.length),
    count: controls.length
  };
}
function pointPixel(point, model) {
  const local = utmLocal(point, model.east0, model.north0);
  const dx = local[0] - model.tx, dy = local[1] - model.ty;
  return {
    u: (model.a * dx + model.b * dy) / model.det,
    v: (-model.b * dx + model.a * dy) / model.det
  };
}
function pixelToUtm(u, v, model) {
  const x = model.a * u - model.b * v + model.tx;
  const y = model.b * u + model.a * v + model.ty;
  return {e: x + model.east0, n: y + model.north0};
}
function approxPixelToUtm(u, v, model) {
  return {
    e: model.east0 + model.xmid + (u - model.cx) / model.scale,
    n: model.north0 + model.ymid - (v - model.cy) / model.scale
  };
}
function utmToLonLat(easting, northing, zone, hemisphere) {
  const a = 6378137.0, f = 1 / 298.257223563;
  const e = f * (2 - f), ep = e / (1 - e);
  const e1 = (1 - Math.sqrt(1 - e)) / (1 + Math.sqrt(1 - e));
  const k0 = 0.9996;
  const x = easting - 500000.0;
  const y = hemisphere === 'S' ? northing - 10000000.0 : northing;
  const lonOrigin = (zone - 1) * 6 - 180 + 3;
  const m = y / k0;
  const mu = m / (a * (1 - e / 4 - 3 * e * e / 64 - 5 * e * e * e / 256));
  const phi1 = mu + (3 * e1 / 2 - 27 * Math.pow(e1, 3) / 32) * Math.sin(2 * mu)
    + (21 * e1 * e1 / 16 - 55 * Math.pow(e1, 4) / 32) * Math.sin(4 * mu)
    + (151 * Math.pow(e1, 3) / 96) * Math.sin(6 * mu)
    + (1097 * Math.pow(e1, 4) / 512) * Math.sin(8 * mu);
  const sinPhi = Math.sin(phi1), cosPhi = Math.cos(phi1), tanPhi = Math.tan(phi1);
  const n1 = a / Math.sqrt(1 - e * sinPhi * sinPhi);
  const t1 = tanPhi * tanPhi;
  const c1 = ep * cosPhi * cosPhi;
  const r1 = a * (1 - e) / Math.pow(1 - e * sinPhi * sinPhi, 1.5);
  const d = x / (n1 * k0);
  const lat = phi1 - (n1 * tanPhi / r1) * (d * d / 2
    - (5 + 3 * t1 + 10 * c1 - 4 * c1 * c1 - 9 * ep) * Math.pow(d, 4) / 24
    + (61 + 90 * t1 + 298 * c1 + 45 * t1 * t1 - 252 * ep - 3 * c1 * c1) * Math.pow(d, 6) / 720);
  const lon = (d - (1 + 2 * t1 + c1) * Math.pow(d, 3) / 6
    + (5 - 2 * c1 + 28 * t1 - 3 * c1 * c1 + 8 * ep + 24 * t1 * t1) * Math.pow(d, 5) / 120) / cosPhi;
  return {
    lat: lat * 180 / Math.PI,
    lon: lonOrigin + lon * 180 / Math.PI
  };
}
function computeApprox() {
  if (!POINTS.length || !IMAGE_W || !IMAGE_H) return null;
  const east0 = mean(POINTS.map(point => Number(point.utm_e)));
  const north0 = mean(POINTS.map(point => Number(point.utm_n)));
  const local = POINTS.map(point => utmLocal(point, east0, north0));
  const xs = local.map(value => value[0]), ys = local.map(value => value[1]);
  const xmin = Math.min.apply(null, xs), xmax = Math.max.apply(null, xs);
  const ymin = Math.min.apply(null, ys), ymax = Math.max.apply(null, ys);
  const content = Array.isArray(IMAGE.content) && IMAGE.content.length === 4 ? IMAGE.content : [0,0,IMAGE_W,IMAGE_H];
  const width = Math.max(1, xmax - xmin), height = Math.max(1, ymax - ymin);
  const scale = Math.min((content[2] - content[0]) * 0.9 / width, (content[3] - content[1]) * 0.9 / height);
  return {
    east0, north0,
    xmid: (xmin + xmax) / 2,
    ymid: (ymin + ymax) / 2,
    scale,
    cx: (content[0] + content[2]) / 2,
    cy: (content[1] + content[3]) / 2
  };
}
function approxPixel(point, model) {
  const local = utmLocal(point, model.east0, model.north0);
  return {
    u: model.cx + (local[0] - model.xmid) * model.scale,
    v: model.cy - (local[1] - model.ymid) * model.scale
  };
}
let approx = computeApprox();
function popupHtml(records) {
  return records.map(record => {
    const area = record.area ? ' · ' + escapeHtml(record.area) : '';
    const label = record.label ? '-' + escapeHtml(record.label) : '';
    return '<div class="popup-record"><span class="popup-title">' + escapeHtml(record.seq) + '</span>' + area + label +
      '<br>UTM: E ' + Number(record.utm_e).toFixed(2) + ' N ' + Number(record.utm_n).toFixed(2) + ' (' + escapeHtml(record.utm_zone) + ')' +
      '<br>lon: ' + escapeHtml(record.lon) + '<br>lat: ' + escapeHtml(record.lat) +
      '<br>valid: ' + escapeHtml(record.valid) +
      '<br>position_status: ' + escapeHtml(record.position) +
      '<br>orientation_status: ' + escapeHtml(record.orientation) +
      '<br>time: ' + escapeHtml(record.time) + '</div>';
  }).join('');
}
function renderPoints() {
  pointLayer.clearLayers();
  labelEntries = [];
  const groups = new Map();
  POINTS.forEach(point => {
    const base = transform ? pointPixel(point, transform) : (approx ? approxPixel(point, approx) : null);
    if (!base || !Number.isFinite(base.u) || !Number.isFinite(base.v)) return;
    const pixel = base;
    const key = pixel.u.toFixed(2) + ',' + pixel.v.toFixed(2);
    if (!groups.has(key)) groups.set(key, {pixel, records: []});
    groups.get(key).records.push(point);
  });
  groups.forEach(group => {
    const latlng = [IMAGE_H - group.pixel.v, group.pixel.u];
    const invalid = group.records.some(record => !record.valid);
    const marker = L.circleMarker(latlng, {
      radius: 6,
      color: invalid ? '#c62828' : '#16803c',
      fillColor: invalid ? '#ef9a9a' : '#8ce99a',
      fillOpacity: 0.9,
      weight: 2,
      dashArray: transform ? null : '4,4',
      interactive: !picking,
      zIndexOffset: 1000
    });
    marker.bindPopup(popupHtml(group.records));
    marker.addTo(pointLayer);
    const text = group.records.map(record => record.seq).join(',');
    const label = L.marker(latlng, {
      icon: L.divIcon({
        className: 'seq-label' + (transform ? '' : ' approx'),
        html: escapeHtml(text),
        iconSize: null,
        iconAnchor: [-9, 9]
      }),
      interactive: false,
      keyboard: false,
      zIndexOffset: 1500
    });
    label.addTo(pointLayer);
    labelEntries.push({marker: label, latlng, text});
  });
  requestAnimationFrame(refreshLabels);
}
function refreshLabels() {
  if (!labelEntries.length) return;
  const items = labelEntries.map(entry => {
    const point = map.latLngToContainerPoint(entry.latlng);
    const width = Math.max(20, entry.text.length * 8 + 10);
    return {
      entry,
      point,
      box: {x: point.x + 9, y: point.y - 9, w: width, h: 20}
    };
  });
  items.forEach(item => {
    item.density = items.filter(other =>
      Math.abs(other.point.x - item.point.x) < 48 &&
      Math.abs(other.point.y - item.point.y) < 48).length;
  });
  items.sort((left, right) => left.density - right.density || left.entry.text.localeCompare(right.entry.text));
  const placed = [];
  items.forEach(item => {
    const element = item.entry.marker.getElement();
    if (!element) return;
    const overlaps = placed.some(box =>
      !(item.box.x + item.box.w < box.x || item.box.x > box.x + box.w ||
        item.box.y + item.box.h < box.y || item.box.y > box.y + box.h));
    if (overlaps) {
      element.classList.add('label-hidden');
    } else {
      element.classList.remove('label-hidden');
      placed.push(item.box);
    }
  });
}
function renderPairs() {
  pairsEl.innerHTML = '';
  pairs.forEach((pair, index) => {
    const row = document.createElement('div');
    row.className = 'pair';
    const info = document.createElement('div');
    info.innerHTML = '<b>' + escapeHtml(pair.name || pair.seq) + '</b>' +
      '<small>px ' + Number(pair.px).toFixed(1) + ', ' + Number(pair.py).toFixed(1) + '</small>';
    const right = document.createElement('div');
    if (transform) {
      const residual = transform.residuals[index];
      const value = document.createElement('span');
      value.className = 'res' + (residual > 2 ? ' bad' : '');
      value.textContent = residual.toFixed(2) + ' px';
      right.appendChild(value);
    }
    const remove = document.createElement('button');
    remove.textContent = '删除';
    remove.onclick = () => {
      lastHoldout = null;
      pairs.splice(index, 1);
      refresh();
    };
    right.appendChild(remove);
    row.appendChild(info);
    row.appendChild(right);
    pairsEl.appendChild(row);
  });
}
function renderSelect() {
  const bound = new Set(pairs.map(pair => pair.seq));
  selectEl.innerHTML = '';
  POINTS.forEach(point => {
    if (bound.has(point.seq)) return;
    const option = document.createElement('option');
    option.value = String(point.seq);
    option.textContent = point.seq +
      (point.area ? ' · ' + point.area : '') +
      (point.label ? '-' + point.label : '') +
      ' · E ' + Number(point.utm_e).toFixed(2) + ' N ' + Number(point.utm_n).toFixed(2);
    selectEl.appendChild(option);
  });
  const disabled = !selectEl.options.length;
  selectEl.disabled = disabled;
  pickBtn.disabled = disabled;
}
function updateStatus() {
  if (transform) {
    bannerEl.className = 'ok';
    bannerEl.textContent = '已标定：' + transform.count + ' 个控制点，所有点位按图纸像素精确摆放。';
    statusEl.textContent = '比例 ' + transform.pixelsPerMeter.toFixed(2) + ' px/m' +
      ' · 旋转 ' + transform.rotation.toFixed(1) + '°' +
      ' · RMSE ' + transform.rmse.toFixed(2) + ' px';
  } else if (pairs.length >= 2) {
    bannerEl.className = 'error';
    bannerEl.textContent = '控制点无法求解（可能重合或像素/经纬度顺序有误），请删除后重试。';
    statusEl.textContent = '标定失败';
  } else {
    bannerEl.className = 'warn';
    bannerEl.textContent = '未标定：当前为近似摆放，请选择点位并在图纸上拾取至少 2 个已知位置。';
    statusEl.textContent = '未标定 · 近似摆放';
  }
}
function renderCalibrationPoints() {
  cpLayer.clearLayers();
  pairs.forEach((pair, index) => {
    const latlng = [IMAGE_H - Number(pair.py), Number(pair.px)];
    const marker = L.circleMarker(latlng, {
      radius: 9,
      color: '#1565c0',
      fillColor: '#e3f2fd',
      fillOpacity: 1,
      weight: 3,
      interactive: !picking,
      zIndexOffset: 2000
    });
    marker.bindPopup('<b>控制点 ' + (index + 1) + '</b><br>' + escapeHtml(pair.name || pair.seq) +
      '<br>px: ' + Number(pair.px).toFixed(2) + '<br>py: ' + Number(pair.py).toFixed(2) +
      '<br>UTM: E ' + Number(pair.utm_e).toFixed(2) + ' N ' + Number(pair.utm_n).toFixed(2) +
      '<br>lon: ' + pair.lon + '<br>lat: ' + pair.lat);
    marker.addTo(cpLayer);
    const label = L.marker(latlng, {
      icon: L.divIcon({
        className: 'cp-label',
        html: escapeHtml('CP' + (index + 1)),
        iconSize: null,
        iconAnchor: [-11, -11]
      }),
      interactive: false,
      keyboard: false,
      zIndexOffset: 2100
    });
    label.addTo(cpLayer);
  });
}
function renderHoldout() {
  if (!lastHoldout) {
    holdoutEl.textContent = '';
    holdoutEl.className = 'hint';
    return;
  }
  const level = lastHoldout.errorPx < 1 ? 'holdout-ok' : lastHoldout.errorPx < 3 ? 'holdout-warn' : 'holdout-bad';
  holdoutEl.className = 'hint ' + level;
  holdoutEl.textContent = '交叉验证 ' + lastHoldout.name + '：点击位置与原标定预测相差 ' +
    lastHoldout.errorPx.toFixed(2) + ' px（' + lastHoldout.errorMeters.toFixed(2) + ' m）';
}
function syncViewControls() {
  imageScaleEl.value = String(view.scale);
  imageRotationEl.value = String(view.rotation);
  imageOffsetXEl.value = String(view.offsetX);
  imageOffsetYEl.value = String(view.offsetY);
  imageScaleValueEl.textContent = view.scale.toFixed(2) + '×';
  imageAngleEl.textContent = view.rotation.toFixed(0) + '°';
  imageOffsetXValueEl.textContent = view.offsetX.toFixed(0);
  imageOffsetYValueEl.textContent = view.offsetY.toFixed(0);
  flipXBtn.classList.toggle('active', view.mirrorX);
  flipYBtn.classList.toggle('active', view.mirrorY);
}
function updateViewState() {
  const mirrors = [];
  if (view.mirrorX) mirrors.push('水平');
  if (view.mirrorY) mirrors.push('垂直');
  viewStateEl.textContent = '当前：缩放 ' + view.scale.toFixed(2) + '× · 旋转 ' + view.rotation.toFixed(0) +
    '° · 镜像 ' + (mirrors.length ? mirrors.join('+') : '无') +
    ' · 平移 ' + view.offsetX.toFixed(0) + ',' + view.offsetY.toFixed(0);
}
function applyImageTransform() {
  const wrapper = imageOverlay.getWrapper();
  if (!wrapper) return;
  const position = L.DomUtil.getPosition(wrapper);
  const x = position ? position.x : 0;
  const y = position ? position.y : 0;
  const factor = (wrapper.offsetWidth || IMAGE_W) / IMAGE_W;
  const scaleX = view.scale * (view.mirrorX ? -1 : 1);
  const scaleY = view.scale * (view.mirrorY ? -1 : 1);
  wrapper.style.transformOrigin = '50% 50%';
  wrapper.style.transform = 'translate3d(' + (x + view.offsetX * factor) + 'px,' +
    (y + view.offsetY * factor) + 'px,0) rotate(' + view.rotation +
    'deg) scale(' + scaleX + ',' + scaleY + ')';
}
function refresh() {
  transform = pairs.length >= 2 ? fitSimilarity(pairs) : null;
  renderPoints();
  renderCalibrationPoints();
  renderPairs();
  renderSelect();
  updateStatus();
  renderHoldout();
  syncViewControls();
  updateViewState();
  updateCoordState();
  applyImageTransform();
}
function setPicking(value) {
  if (value && coordPicking) setCoordPicking(false);
  picking = value;
  pickBtn.textContent = picking ? '取消拾取' : '在图纸上拾取所选点位';
  map.getContainer().style.cursor = picking ? 'crosshair' : '';
  hintEl.textContent = picking ? '点位标记已暂时隐藏，请点击图纸上的实际特征位置。' : DEFAULT_HINT;
  if (picking) {
    map.removeLayer(pointLayer);
  } else {
    pointLayer.addTo(map);
  }
  renderPoints();
  renderCalibrationPoints();
}
pickBtn.onclick = () => {
  if (!picking && !selectEl.value) return;
  setPicking(!picking);
};
map.on('click', event => {
  if (!picking) return;
  const point = POINTS.find(item => item.seq === Number(selectEl.value));
  if (!point) return;
  const u = event.latlng.lng;
  const v = IMAGE_H - event.latlng.lat;
  const name = (point.area ? point.area + '-' : '') + (point.label || point.seq);
  let holdout = null;
  if (transform && pairs.length >= 2) {
    const predicted = pointPixel(point, transform);
    const errorPx = Math.hypot(predicted.u - u, predicted.v - v);
    holdout = {
      name,
      seq: point.seq,
      errorPx,
      errorMeters: errorPx / transform.pixelsPerMeter
    };
  }
  const existing = pairs.findIndex(pair => pair.seq === point.seq);
  if (existing >= 0) pairs.splice(existing, 1);
  pairs.push({
    name,
    seq: point.seq,
    area: point.area,
    label: point.label,
    px: u,
    py: v,
    lon: Number(point.lon),
    lat: Number(point.lat),
    utm_e: Number(point.utm_e),
    utm_n: Number(point.utm_n),
    utm_zone: point.utm_zone
  });
  lastHoldout = holdout;
  setPicking(false);
  refresh();
});
exportBtn.onclick = () => {
  const payload = {
    image: IMAGE.name,
    image_width: IMAGE_W,
    image_height: IMAGE_H,
    control_points: pairs.map(pair => ({
      name: pair.name,
      seq: pair.seq,
      area: pair.area,
      label: pair.label,
      px: Number(pair.px.toFixed(2)),
      py: Number(pair.py.toFixed(2)),
      lon: Number(pair.lon),
      lat: Number(pair.lat),
      utm_e: Number(pair.utm_e),
      utm_n: Number(pair.utm_n),
      utm_zone: pair.utm_zone
    }))
  };
  const blob = new Blob([JSON.stringify(payload, null, 2)], {type: 'application/json'});
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob);
  link.download = 'rtk_calibration.json';
  link.click();
  URL.revokeObjectURL(link.href);
};
imageVisibleEl.onchange = () => {
  if (imageVisibleEl.checked) {
    imageOverlay.addTo(map);
    requestAnimationFrame(applyImageTransform);
  } else {
    map.removeLayer(imageOverlay);
  }
};
imageOpacityEl.oninput = () => imageOverlay.setOpacity(Number(imageOpacityEl.value));
imageScaleEl.oninput = () => {
  view.scale = Number(imageScaleEl.value);
  imageScaleValueEl.textContent = view.scale.toFixed(2) + '×';
  applyImageTransform();
  updateViewState();
};
imageRotationEl.oninput = () => {
  view.rotation = Number(imageRotationEl.value);
  imageAngleEl.textContent = view.rotation.toFixed(0) + '°';
  applyImageTransform();
  updateViewState();
};
imageOffsetXEl.oninput = () => {
  view.offsetX = Number(imageOffsetXEl.value);
  imageOffsetXValueEl.textContent = view.offsetX.toFixed(0);
  applyImageTransform();
  updateViewState();
};
imageOffsetYEl.oninput = () => {
  view.offsetY = Number(imageOffsetYEl.value);
  imageOffsetYValueEl.textContent = view.offsetY.toFixed(0);
  applyImageTransform();
  updateViewState();
};
flipXBtn.onclick = () => {
  view.mirrorX = !view.mirrorX;
  flipXBtn.classList.toggle('active', view.mirrorX);
  applyImageTransform();
  updateViewState();
};
flipYBtn.onclick = () => {
  view.mirrorY = !view.mirrorY;
  flipYBtn.classList.toggle('active', view.mirrorY);
  applyImageTransform();
  updateViewState();
};
flip180Btn.onclick = () => {
  let rotation = view.rotation + 180;
  if (rotation > 360) rotation -= 720;
  if (rotation < -360) rotation += 720;
  view.rotation = rotation;
  imageRotationEl.value = String(rotation);
  imageAngleEl.textContent = rotation.toFixed(0) + '°';
  applyImageTransform();
  updateViewState();
};
resetAdjustBtn.onclick = () => {
  Object.assign(view, {
    scale: 1, rotation: 0, mirrorX: false, mirrorY: false, offsetX: 0, offsetY: 0
  }, DEFAULT_VIEW);
  syncViewControls();
  applyImageTransform();
  updateViewState();
};
const sidebarEl = document.getElementById('sidebar');
const sidebarCollapseBtn = document.getElementById('sidebar-collapse');
const sidebarReopenBtn = document.getElementById('sidebar-reopen');
const viewControlsEl = document.getElementById('view-controls');
const viewCollapseBtn = document.getElementById('view-collapse');
function refreshMapSize() {
  requestAnimationFrame(() => {
    map.invalidateSize({pan: false});
    applyImageTransform();
    refreshLabels();
  });
}
function setSidebarCollapsed(collapsed) {
  sidebarEl.classList.toggle('collapsed', collapsed);
  sidebarReopenBtn.classList.toggle('visible', collapsed);
  sidebarCollapseBtn.setAttribute('aria-expanded', String(!collapsed));
  refreshMapSize();
}
sidebarCollapseBtn.onclick = () => setSidebarCollapsed(true);
sidebarReopenBtn.onclick = () => setSidebarCollapsed(false);
viewCollapseBtn.onclick = () => {
  const collapsed = viewControlsEl.classList.toggle('collapsed');
  viewCollapseBtn.textContent = collapsed ? '展开' : '收起';
  viewCollapseBtn.setAttribute('aria-expanded', String(!collapsed));
  refreshMapSize();
};
locateBtn.onclick = () => {
  const latlngs = [];
  POINTS.forEach(point => {
    const pixel = transform ? pointPixel(point, transform) : (approx ? approxPixel(point, approx) : null);
    if (pixel && Number.isFinite(pixel.u) && Number.isFinite(pixel.v)) {
      latlngs.push([IMAGE_H - pixel.v, pixel.u]);
    }
  });
  if (latlngs.length) map.fitBounds(L.latLngBounds(latlngs), {padding: [40, 40], maxZoom: 2});
};
clearBtn.onclick = () => {
  if (!pairs.length || confirm('清空所有标定控制点？')) {
    pairs = [];
    lastHoldout = null;
    setPicking(false);
    refresh();
  }
};
function showCoordMessage(message) {
  coordValueEl.textContent = '—';
  coordUtmEl.textContent = message;
  coordPanelEl.classList.add('visible');
}
function setCoordPicking(value) {
  coordPicking = value;
  if (value) setPicking(false);
  coordPickBtn.textContent = coordPicking ? '退出坐标拾取' : '拾取坐标（估算经纬度）';
  map.getContainer().style.cursor = coordPicking ? 'crosshair' : '';
  if (!coordPicking) coordPanelEl.classList.remove('visible');
  if (coordPicking && !transform && !approx) showCoordMessage('没有可用的坐标基准');
}
function updateCoordState() {
  coordPickBtn.disabled = !(transform || approx);
  coordPickBtn.title = transform ? '' : (approx ? '未标定，将使用近似摆放估算' : '没有可用的坐标基准');
  if (!transform && !approx && coordPicking) setCoordPicking(false);
}
coordPickBtn.onclick = () => setCoordPicking(!coordPicking);
coordCloseBtn.onclick = () => {
  setCoordPicking(false);
  coordLayer.clearLayers();
};
coordCopyBtn.onclick = () => {
  const text = coordValueEl.textContent;
  if (!text || text === '—') return;
  if (navigator.clipboard && navigator.clipboard.writeText) {
    navigator.clipboard.writeText(text).catch(() => {});
  }
  coordCopyBtn.textContent = '已复制';
  setTimeout(() => { coordCopyBtn.textContent = '复制'; }, 1200);
};
map.on('click', event => {
  if (!coordPicking || (!transform && !approx)) return;
  const u = event.latlng.lng;
  const v = IMAGE_H - event.latlng.lat;
  const utm = transform ? pixelToUtm(u, v, transform) : approxPixelToUtm(u, v, approx);
  const zoneText = String((transform && transform.zone) || (POINTS[0] && POINTS[0].utm_zone) || '49N');
  const match = zoneText.match(/(\d+)\s*([NS])/);
  const zone = match ? Number(match[1]) : 49;
  const hemisphere = match && match[2] === 'S' ? 'S' : 'N';
  const geo = utmToLonLat(utm.e, utm.n, zone, hemisphere);
  coordValueEl.textContent = geo.lat.toFixed(6) + ',' + geo.lon.toFixed(6);
  coordUtmEl.textContent = 'UTM: E ' + utm.e.toFixed(2) + ' N ' + utm.n.toFixed(2) + ' (' + zoneText + ')' +
    (transform ? '' : ' · 近似摆放估算，标定后更准确');
  coordPanelEl.classList.add('visible');
  coordLayer.clearLayers();
  L.circleMarker(event.latlng, {
    radius: 6,
    color: '#b45309',
    fillColor: '#fbbf24',
    fillOpacity: 0.95,
    weight: 2,
    zIndexOffset: 2500
  }).addTo(coordLayer);
});
map.on('zoomend moveend viewreset', applyImageTransform);
map.on('zoomend moveend', refreshLabels);
imageOverlay.on('load', applyImageTransform);
const content = Array.isArray(IMAGE.content) && IMAGE.content.length === 4 ? IMAGE.content : [0,0,IMAGE_W,IMAGE_H];
map.fitBounds(L.latLngBounds([[IMAGE_H - content[3], content[0]], [IMAGE_H - content[1], content[2]]]), {padding: [24, 24]});
refresh();
</script></body></html>"""


GEO_TEMPLATE = r"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><title>RTK points</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>html,body,#map{height:100%;margin:0} #map{background:#d7d7d7} .seq-label{background:#fff;border:1px solid #333;border-radius:3px;color:#111;font:bold 12px monospace;padding:1px 3px;white-space:nowrap;} .popup{font:14px monospace;line-height:1.6}</style></head><body><div id="map"></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script><script>
const points=__POINTS__; const referencePoints=__REFERENCE__; const map=L.map('map').setView(__CENTER__,18);
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:22,attribution:'© OpenStreetMap'}).addTo(map);
const bounds=[];
points.forEach(p=>{const marker=L.circleMarker([Number(p.lat),Number(p.lon)],{radius:6,color:p.valid?'#16803c':'#c62828',fillOpacity:.8,zIndexOffset:1000});
L.marker([Number(p.lat),Number(p.lon)],{icon:L.divIcon({className:'seq-label',html:String(p.seq),iconSize:null,iconAnchor:[-8,8]}),interactive:false,zIndexOffset:1500}).addTo(map);
marker.bindPopup('<div class="popup"><b>'+p.seq+'</b>'+(p.area?'<br>area: '+p.area:'')+(p.label?'<br>label: '+p.label:'')+'<br>UTM: E '+Number(p.utm_e).toFixed(2)+' N '+Number(p.utm_n).toFixed(2)+' ('+p.utm_zone+')<br>lon: '+p.lon+'<br>lat: '+p.lat+'<br>valid: '+p.valid+'<br>position_status: '+p.position+'<br>orientation_status: '+p.orientation+'<br>time: '+p.time+'</div>'); marker.addTo(map); bounds.push([Number(p.lat),Number(p.lon)]);});
if(bounds.length) map.fitBounds(bounds,{padding:[30,30]});
referencePoints.forEach(p=>{const marker=L.circleMarker([p.lat,p.lon],{radius:8,color:'#00bcd4',fillColor:'#fff',fillOpacity:1,weight:3,zIndexOffset:2000}).addTo(map); marker.bindPopup('<div class="popup"><b>参考点 '+p.name+'</b><br>lon: '+p.lon+'<br>lat: '+p.lat+'</div>');});
</script></body></html>"""


def pdf_to_png(pdf_path, output_path, dpi=150):
    """Render the first PDF page to PNG with pypdfium2 (no poppler needed)."""
    try:
        import pypdfium2 as pdfium
    except ImportError as error:
        raise RuntimeError("转换 PDF 需要 pypdfium2：pip install pypdfium2") from error
    document = pdfium.PdfDocument(str(pdf_path))
    try:
        document[0].render(scale=dpi / 72).to_pil().save(str(output_path))
    finally:
        document.close()


def pdf_to_svg(pdf_path, output_path, dpi=150):
    """Export the first PDF page to a vector SVG scaled to dpi pixels per inch."""
    try:
        import pymupdf
    except ImportError as error:
        raise RuntimeError("转换 PDF 为 SVG 需要 PyMuPDF：pip install pymupdf") from error
    document = pymupdf.open(str(pdf_path))
    try:
        page = document[0]
        svg = page.get_svg_image()
        width = round(page.rect.width / 72 * dpi)
        height = round(page.rect.height / 72 * dpi)
        root = re.search(r"<svg\b[^>]*>", svg)
        if root:
            tag = root.group(0)
            if 'width="' in tag and 'height="' in tag:
                tag = re.sub(r'\bwidth="[^"]*"', f'width="{width}"', tag, count=1)
                tag = re.sub(r'\bheight="[^"]*"', f'height="{height}"', tag, count=1)
            else:
                tag = tag.replace("<svg", f'<svg width="{width}" height="{height}"', 1)
            svg = svg[:root.start()] + tag + svg[root.end():]
    finally:
        document.close()
    Path(output_path).write_text(svg, encoding="utf-8")


def image_data_uri(path):
    mime = mimetypes.guess_type(str(path))[0] or "image/png"
    encoded = base64.b64encode(Path(path).read_bytes()).decode("ascii")
    return f"data:{mime};base64,{encoded}"


def render_sheet(points, background_image, control_points=None, default_view=None, datasets=None):
    control_points = control_points or []
    datasets = datasets or [{"label": "当前点位", "file": "", "points": points}]
    default_view = dict({
        "scale": 1,
        "rotation": 0,
        "mirrorX": False,
        "mirrorY": False,
        "offsetX": 0,
        "offsetY": 0,
    }, **(default_view or {}))
    width, height = image_size(background_image)
    content = image_content_bbox(background_image, (width, height))
    image_payload = json.dumps({
        "src": image_data_uri(background_image),
        "name": Path(background_image).name,
        "width": width,
        "height": height,
        "content": content,
    }, ensure_ascii=False)
    datasets_payload = json.dumps(datasets, ensure_ascii=False)
    pairs_payload = json.dumps(control_points, ensure_ascii=False)
    print(f"底图: {background_image} ({width}×{height}), 内容区: {content}")
    for dataset in datasets:
        print(f"点位序列: {dataset['label']} ({len(dataset['points'])} 点)")
    transform = fit_similarity(control_points)
    if transform:
        print(f"标定: {transform['count']} 点, {transform['pixels_per_meter']:.2f} px/m, "
              f"旋转 {transform['rotation_deg']:.1f}°, RMSE {transform['rmse']:.2f} px")
        for index, point in enumerate(control_points):
            print(f"  {point['name']}: 残差 {transform['residuals'][index]:.2f} px")
    elif len(control_points) >= 2:
        print("警告: 控制点无法求解，页面将显示近似摆放", file=sys.stderr)
    return (SHEET_TEMPLATE
            .replace("__DATASETS__", datasets_payload.replace("</", "<\\/"))
            .replace("__IMAGE__", image_payload)
            .replace("__PAIRS__", pairs_payload.replace("</", "<\\/"))
            .replace("__DEFAULT_VIEW__", json.dumps(default_view)))


def render_geo(points, reference_points=None):
    reference_points = reference_points or []
    points_payload = json.dumps(points, ensure_ascii=False)
    reference_payload = json.dumps(reference_points, ensure_ascii=False)
    center = [float(points[0]["lat"]), float(points[0]["lon"])] if points else [0, 0]
    return (GEO_TEMPLATE
            .replace("__POINTS__", points_payload.replace("</", "<\\/"))
            .replace("__REFERENCE__", reference_payload)
            .replace("__CENTER__", json.dumps(center)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--background-pdf")
    parser.add_argument("--pdf-dpi", type=int, default=150,
                        help="PDF 渲染 DPI，默认 150；放大查看可提高到 300")
    parser.add_argument("--pdf-format", choices=("png", "svg"), default="png",
                        help="PDF 底图格式：png 位图（默认）或 svg 矢量，svg 放大不模糊")
    parser.add_argument("--background-image")
    parser.add_argument("--reference-point", action="append", default=[], metavar="NAME,LON,LAT")
    parser.add_argument("--control-point", action="append", default=[], metavar="NAME,PX,PY,LON,LAT")
    parser.add_argument("--dataset", action="append", default=[], metavar="LABEL=PATH",
                        help="额外点位序列，可在页面左侧“点位序列”中切换；可重复使用")
    parser.add_argument("--calibration", help="JSON file with a control_points array")
    parser.add_argument("--view-mirror-x", action="store_true", help="flip the background image horizontally")
    parser.add_argument("--view-mirror-y", action="store_true", help="flip the background image vertically")
    args = parser.parse_args()
    calibration_meta = load_calibration_metadata(args.calibration) if args.calibration else {}
    default_view = {
        "mirrorX": args.view_mirror_x or bool(calibration_meta.get("image_mirror_x")),
        "mirrorY": args.view_mirror_y or bool(calibration_meta.get("image_mirror_y")),
    }
    points = load_points(args.input)
    references = []
    for value in args.reference_point:
        name, lon, lat = value.split(",", 2)
        references.append({"name": name, "lon": float(lon), "lat": float(lat)})
    controls = []
    if args.calibration:
        controls.extend(load_calibration(args.calibration))
    for value in args.control_point:
        controls.append(parse_control_point(value))
    datasets = [{"label": Path(args.input).name, "file": Path(args.input).name, "points": points}]
    for value in args.dataset:
        label, separator, path = value.partition("=")
        if not separator:
            label, path = Path(label).name, label
        datasets.append({"label": label.strip(), "file": Path(path).name, "points": load_points(path)})
    print(f"点位: {len(points)}")
    if args.background_pdf:
        with tempfile.TemporaryDirectory() as temp_dir:
            suffix = ".svg" if args.pdf_format == "svg" else ".png"
            image = Path(temp_dir) / ("page" + suffix)
            if args.pdf_format == "svg":
                pdf_to_svg(args.background_pdf, image, args.pdf_dpi)
            else:
                pdf_to_png(args.background_pdf, image, args.pdf_dpi)
            output = render_sheet(points, image, controls, default_view, datasets)
            Path(args.output).write_text(output, encoding="utf-8")
        return
    if args.background_image:
        output = render_sheet(points, args.background_image, controls, default_view, datasets)
        Path(args.output).write_text(output, encoding="utf-8")
        return
    if controls:
        print("警告: 未提供底图，忽略标定参数", file=sys.stderr)
    Path(args.output).write_text(render_geo(points, references), encoding="utf-8")


if __name__ == "__main__":
    main()

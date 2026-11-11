#!/usr/bin/env python3
"""Generate a Leaflet map showing every recorded RTK point independently."""
import argparse
import base64
import json
import re
import subprocess
import tempfile
from pathlib import Path

RECORD_RE = re.compile(r"- seq:\s*(?P<seq>\d+)\s+(?P<body>.*?)(?=\n- seq:|\Z)", re.DOTALL)
FIELD_RE = re.compile(
    r"point:\s*\{lon:\s*(?P<lon>[-+0-9.eE]+),\s*lat:\s*(?P<lat>[-+0-9.eE]+)\}.*?"
    r"valid:\s*(?P<valid>true|false).*?position_status:\s*(?P<position>[-+0-9]+).*?"
    r"orientation_status:\s*(?P<orientation>[-+0-9]+).*?time:\s*\"(?P<time>[^\"]*)\"",
    re.DOTALL,
)


def load_points(path):
    text = Path(path).read_text(encoding="utf-8")
    points = []
    for record in RECORD_RE.finditer(text):
        fields = FIELD_RE.search(record.group("body"))
        if fields:
            points.append({"seq": int(record.group("seq")), "lon": fields.group("lon"),
                           "lat": fields.group("lat"), "valid": fields.group("valid") == "true",
                           "position": int(fields.group("position")),
                           "orientation": int(fields.group("orientation")), "time": fields.group("time")})
    return points


def render(points, background_image=None, reference_points=None):
    payload = json.dumps(points, ensure_ascii=False)
    reference_points = reference_points or []
    reference_payload = json.dumps(reference_points, ensure_ascii=False)
    center = [float(points[0]["lat"]), float(points[0]["lon"])] if points else [0, 0]
    background = ""
    if background_image:
        encoded = base64.b64encode(Path(background_image).read_bytes()).decode("ascii")
        svg_data = encoded
        background = f'<div id="pdf-controls"><label><input id="pdf-visible" type="checkbox" checked>显示底图</label><label>透明度 <input id="pdf-opacity" type="range" min="0" max="1" step="0.05" value="0.45"></label><label>底图缩放 <input id="pdf-scale" type="range" min="0.25" max="3" step="0.05" value="1"> <span id="pdf-scale-value">1.00x</span></label><label>旋转 <input id="pdf-rotation" type="range" min="-180" max="180" step="1" value="0"> <span id="pdf-angle">0°</span></label><label><input id="online-map" type="checkbox">显示在线地图</label></div><div id="pdf-data" data-image="data:image/png;base64,{svg_data}"></div>'
    else:
        background = ''
    if points:
        lats = [float(point["lat"]) for point in points]
        lons = [float(point["lon"]) for point in points]
        bounds = [[min(lats), min(lons)], [max(lats), max(lons)]]
    else:
        bounds = [[center[0] - 0.001, center[1] - 0.001], [center[0] + 0.001, center[1] + 0.001]]
    return f'''<!doctype html><html><head><meta charset="utf-8"><title>RTK points</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>html,body,#map{{height:100%;margin:0}} #map{{background:#d7d7d7}} #map .leaflet-image-layer{{transform-origin:50% 50%;}} #pdf-controls{{position:fixed;z-index:1000;top:10px;right:10px;background:#fff;padding:8px;border-radius:4px;box-shadow:0 1px 5px #555;font:13px sans-serif}} #pdf-controls label{{display:block;margin:3px}} .popup{{font:14px monospace;line-height:1.6}}</style></head><body><div id="map"></div>{background}
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script><script>
const points={payload}; const referencePoints={reference_payload}; const map=L.map('map').setView({json.dumps(center)},18);
const onlineTiles=L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',{{maxZoom:22,attribution:'© OpenStreetMap'}});
const onlineToggle=document.getElementById('online-map'); if(onlineToggle) onlineToggle.onchange=()=>onlineToggle.checked?onlineTiles.addTo(map):map.removeLayer(onlineTiles);
const bounds=[]; const pdfBounds={json.dumps(bounds)}; let pdfOverlay=null;
if(document.getElementById('pdf-data')){{const image=document.getElementById('pdf-data').dataset.image; const anchor=L.latLngBounds(pdfBounds).getCenter(); pdfOverlay=L.marker(anchor,{{draggable:true,icon:L.divIcon({{className:'background-anchor',html:'<img id="background-image" src="'+image+'" style="position:absolute;max-width:none;transform-origin:center;pointer-events:none">',iconSize:[0,0],iconAnchor:[0,0]}}),zIndexOffset:-10000,opacity:.45}}).addTo(map);}}

points.forEach(p=>{{const marker=L.circleMarker([Number(p.lat),Number(p.lon)],{{radius:6,color:p.valid?'#16803c':'#c62828',fillOpacity:.8,zIndexOffset:1000}});
marker.bindPopup('<div class="popup"><b>seq '+p.seq+'</b><br>lon: '+p.lon+'<br>lat: '+p.lat+'<br>valid: '+p.valid+'<br>position_status: '+p.position+'<br>orientation_status: '+p.orientation+'<br>time: '+{json.dumps('')}+' '+p.time+'</div>'); marker.addTo(map); bounds.push([Number(p.lat),Number(p.lon)]);}});
if(bounds.length) map.fitBounds(bounds,{{padding:[30,30]}});
referencePoints.forEach(p=>{{const marker=L.circleMarker([p.lat,p.lon],{{radius:8,color:'#00bcd4',fillColor:'#fff',fillOpacity:1,weight:3,zIndexOffset:2000}}).addTo(map); marker.bindPopup('<div class="popup"><b>参考点 '+p.name+'</b><br>lon: '+p.lon+'<br>lat: '+p.lat+'</div>');}});
const visible=document.getElementById('pdf-visible'); const opacity=document.getElementById('pdf-opacity'); const scale=document.getElementById('pdf-scale'); const rotation=document.getElementById('pdf-rotation');
if(visible) visible.onchange=()=>{{if(visible.checked) pdfOverlay.addTo(map); else map.removeLayer(pdfOverlay);}};
if(opacity) opacity.oninput=()=>pdfOverlay.setOpacity(Number(opacity.value));
const baseZoom=map.getZoom();
function updatePdfTransform(){{const img=document.getElementById('background-image'); if(img){{img.style.width=(600*Math.pow(2,map.getZoom()-baseZoom)*Number(scale.value))+'px'; img.style.height='auto'; img.style.transform=`translate(-50%,-50%) rotate(${{rotation.value}}deg)`;}}}}
map.on('zoomend',updatePdfTransform); updatePdfTransform();
if(scale) scale.oninput=()=>{{document.getElementById('pdf-scale-value').textContent=Number(scale.value).toFixed(2)+'x'; updatePdfTransform();}};
if(rotation) rotation.oninput=()=>{{document.getElementById('pdf-angle').textContent=rotation.value+'°'; updatePdfTransform();}};
</script></body></html>'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--background-pdf")
    parser.add_argument("--background-image")
    parser.add_argument("--reference-point", action="append", default=[], metavar="NAME,LON,LAT")
    args = parser.parse_args()
    references = []
    for value in args.reference_point:
        name, lon, lat = value.split(",", 2)
        references.append({"name": name, "lon": float(lon), "lat": float(lat)})
    background = None
    if args.background_pdf:
        with tempfile.TemporaryDirectory() as temp_dir:
            prefix = str(Path(temp_dir) / "page")
            subprocess.run(["pdftoppm", "-f", "1", "-singlefile", "-png", "-r", "150", args.background_pdf, prefix], check=True)
            background = Path(prefix + ".png")
            Path(args.output).write_text(render(load_points(args.input), background, references), encoding="utf-8")
        return
    Path(args.output).write_text(render(load_points(args.input), args.background_image, references), encoding="utf-8")


if __name__ == "__main__":
    main()

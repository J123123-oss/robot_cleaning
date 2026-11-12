# RTK 点位图纸对齐使用教程

本教程说明如何用 `tools/plot_rtk_points.py` 生成“现场 RTK 点位 + CAD 底图”的对齐页面，以及更换底图或点位后如何一步步完成对准。

## 0. 从零到对准速查

1. 准备底图：PDF 直接用（工具内置 pypdfium2 转图），或放 PNG/JPEG。
2. 准备点位：确认 `rtk_points.md` 或派生点位文件，混合场地先拆分。
3. 生成页面：`--input` + `--output` + `--background-pdf/--background-image`，可加 `--dataset` 序列。
4. 打开页面：双击启动 bat，或 `http://localhost:8766/页面名`。
5. 初判方向：先用镜像 / 旋转180° / 旋转滑动条让点位大致贴合。
6. 精确标定：选 2 个远距离点拾取 → 查看 RMSE → 第 3 点交叉验证。
7. 导出 `rtk_calibration.json`，放回配置目录。
8. 重新生成页面并固化默认视图（必要时补 `image_mirror_x/y`）。

## 1. 启动入口

- 双击仓库根目录的 `启动RTK图纸预览.bat`：自动在 8766 端口启动本地 HTTP 服务并打开 `rtk_points_006-E22-W24_with_pdf.html`。
- 双击 `启动RTK布置图1112预览.bat`：打开更换底图后的 `rtk_points_006-E22-W24_布置图1112.html`。
- 也可以给 `启动RTK图纸预览.bat` 传入页面名：`启动RTK图纸预览.bat rtk_points_006-E22-W24_布置图1112.html`。
- 手动启动：

```bash
python -m http.server 8766 --bind 127.0.0.1
# 浏览器打开 http://localhost:8766/rtk_points_006-E22-W24_with_pdf.html
```

页面依赖 unpkg 上的 Leaflet 1.9.4，首次打开需要联网；`file://` 直接打开可能因 CDN 加载失败卡在“加载中”。

## 2. 文件角色

| 文件 | 说明 |
|------|------|
| `src/rtk_nav/rtk_nav/config/rtk_points.md` | 现场原始采点（机器人 `WAIT_RTK` 追加写入，勿手改） |
| `src/rtk_nav/rtk_nav/config/rtk_points_*.md` | 按场地/区域派生的点位序列，可多份并存 |
| `src/rtk_nav/rtk_nav/config/rtk_calibration_*.json` | 控制点标定（像素 px/py + 经纬度/UTM） |
| `tmp/pdfs/*.png` | 由 PDF 图纸转换出的底图 |
| `rtk_points_*_with_pdf.html` | 生成的对齐页面（底图内嵌，单文件） |

点位文件格式（每条记录）：

```yaml
- seq: 1
  area: "bridge_5B-6B"        # 可选
  label: a                    # 可选
  point: {lon: 110.6474, lat: 35.6059}
  valid: true
  position_status: 4
  orientation_status: 4
  time: "2026-11-12 10:00:00"
```

## 3. 更换底图的完整步骤

1. 把新图纸放到仓库根目录或 `tmp/pdfs/`。PDF 无需安装 poppler：工具内置 `pypdfium2`，按 150 dpi 渲染首页。可直接用 `--background-pdf` 一步生成页面：

```bash
python tools/plot_rtk_points.py --input src/rtk_nav/rtk_nav/config/rtk_points.md --output rtk_points_预览.html --background-pdf 图纸.pdf
```

若想先单独转成 PNG 再检查，也可以：

```bash
python -c "import pypdfium2 as pdfium; pdfium.PdfDocument('图纸.pdf')[0].render(scale=150/72).to_pil().save('tmp/pdfs/新底图.png')"
```

2. 记录底图分辨率。控制点的 `px/py` 都是**该分辨率下的整图像素坐标**，换图或改分辨率后必须重新拾取，不能沿用旧值。

3. 生成页面（不带标定，先用近似摆放确认底图和点位大致方向）：

```bash
python tools/plot_rtk_points.py ^
  --input src/rtk_nav/rtk_nav/config/rtk_points.md ^
  --output rtk_points_预览.html ^
  --background-image tmp/pdfs/新底图.png
```

4. 打开页面，按第 5 节完成标定，导出 `rtk_calibration.json`。

5. 把导出的 JSON 放到 `src/rtk_nav/rtk_nav/config/` 并按场地命名，重新生成（示例为 006 场地）：

```bash
python tools/plot_rtk_points.py ^
  --input src/rtk_nav/rtk_nav/config/rtk_points_006-E22-W24.md ^
  --output rtk_points_006-E22-W24_with_pdf.html ^
  --background-image tmp/pdfs/001-E1-E8-base.png ^
  --calibration src/rtk_nav/rtk_nav/config/rtk_calibration_006-E22-W24.json ^
  --dataset "原始采点 rtk_points.md=src/rtk_nav/rtk_nav/config/rtk_points.md" ^
  --dataset "E1-E8 派生点=src/rtk_nav/rtk_nav/config/rtk_points_001-E1-E8.md"
```

`--dataset LABEL=PATH` 可重复，页面左侧“点位序列”下拉框会列出所有序列，默认选中 `--input`。切换序列只换点位，不改变标定。

### 3.1 布置图 1112 页面

- 底图：`布置图1112.pdf`（A3，150 dpi → 2481×1754），已转为 `tmp/pdfs/布置图1112-base.png`。
- 页面：`rtk_points_006-E22-W24_布置图1112.html`，替换底图后的新界面，尚未标定，打开后按第 5 节拾取控制点。
- 原已对准页面 `rtk_points_006-E22-W24_with_pdf.html` 保持不动，两者共用同一套点位序列与标定逻辑，但标定 JSON 不能混用（像素坐标随底图变化）。

生成命令（不带 `--calibration`，先做近似摆放）：

```bash
python tools/plot_rtk_points.py ^
  --input src/rtk_nav/rtk_nav/config/rtk_points_006-E22-W24.md ^
  --output rtk_points_006-E22-W24_布置图1112.html ^
  --background-image tmp/pdfs/布置图1112-base.png ^
  --dataset "原始采点 rtk_points.md=src/rtk_nav/rtk_nav/config/rtk_points.md" ^
  --dataset "E1-E8 派生点=src/rtk_nav/rtk_nav/config/rtk_points_001-E1-E8.md"
```

完成标定并导出 JSON 后，把 `rtk_calibration_006-E22-W24.json` 换成新底图的标定文件再重新生成即可固化。

## 4. 更换点位数据的步骤

1. 现场采点后，`rtk_points.md` 会自动追加新记录；先备份原始文件。
2. 若混有多个相距很远的场地，按 `seq` 拆成独立文件（参考 `rtk_points_001-017.md`、`rtk_points_018-020.md`），避免一张底图无法覆盖。
3. 若点位来自区域 YAML（如 `006-E22-W24.yaml`），按 `area, calib_point_a/b/c` 顺序派生成 `rtk_points_006-E22-W24.md`，每条保留 `area/label`。
4. 用第 3 节的生成命令加上 `--dataset`，把新序列加入页面。
5. 点位换了但底图没换时，已有标定可继续使用；点位换了场地、底图也换了，则必须重新标定。

## 5. 页面操作与对准流程

### 5.1 初判方向（底图对齐面板）

- 只调整底图，点位保持不动。
- 如果点位相对底图整体呈镜像关系（怎么旋转都对不上），点“垂直镜像”或“水平镜像”。
- 如果整体差 180°，点“旋转180°”。
- 再用“旋转/平移X/平移Y/缩放”滑动条做粗对齐；“重置”恢复到标定文件的默认视图（含默认镜像）。

### 5.2 精确标定（左侧标定面板）

1. 在“点位序列”里确认当前序列。
2. 在下拉列表选择一条采点，点“在图纸上拾取所选点位”，再点击底图上对应特征位置（建议选相距远、特征清晰的 2 个点，如对角）。
3. 至少 2 个控制点后显示比例、旋转、RMSE；每个控制点的残差应小于 2 px。
4. 再加第 3 个点做交叉验证：页面先按旧标定预测位置，再与点击位置比较。误差在 1 px 内说明标定可信。
5. 点“导出标定 JSON”，得到 `rtk_calibration.json`。

### 5.3 保存默认方向

页面导出的 JSON 只包含控制点。若该场地需要默认镜像，在 JSON 顶层加：

```json
{
  "image": "底图.png",
  "image_width": 7021,
  "image_height": 4967,
  "image_mirror_y": true,
  "control_points": [ ... ]
}
```

生成时读取到 `image_mirror_x/image_mirror_y` 会自动设置页面默认视图；也可以显式加 `--view-mirror-x` / `--view-mirror-y`。

## 6. 常见问题

- **缩放时底图翻转/变形**：已修复。镜像和旋转加在 Leaflet 不会改写的容器上，缩放结束后会清除图片自身的动画变换。若仍出现，请重新生成页面。
- **点位整体歪 90° 且 RMSE 很大（几十到上百 px）**：通常是底图镜像方向不对，先试垂直/水平镜像，再重新拾取控制点。
- **点位跑到图外**：该序列属于另一个场地，或尚未标定（虚线标记为近似摆放）。
- **控制点残差大**：检查是否点错特征、`px/py` 是否被缩放过的截图坐标、经纬度是否取错点。
- **标定 JSON 里 `py` 与点击值不同**：006 标定保存了 `source_py`，`py` 是垂直镜像后的值，属正常。
- **页面卡在“加载中”**：确认通过 `http://localhost:8766` 访问且能联网加载 Leaflet。

## 7. 参考命令

```bash
# 生成 006 图纸页（当前使用）
python tools/plot_rtk_points.py --input src/rtk_nav/rtk_nav/config/rtk_points_006-E22-W24.md --output rtk_points_006-E22-W24_with_pdf.html --background-image tmp/pdfs/001-E1-E8-base.png --calibration src/rtk_nav/rtk_nav/config/rtk_calibration_006-E22-W24.json --dataset "原始采点 rtk_points.md=src/rtk_nav/rtk_nav/config/rtk_points.md" --dataset "E1-E8 派生点=src/rtk_nav/rtk_nav/config/rtk_points_001-E1-E8.md"

# 生成 E1-E8 图纸页
python tools/plot_rtk_points.py --input src/rtk_nav/rtk_nav/config/rtk_points_001-E1-E8.md --output rtk_points_001-E1-E8_with_pdf.html --background-image tmp/pdfs/001-E1-E8-base.png --calibration src/rtk_nav/rtk_nav/config/rtk_calibration_001-E1-E8.json --dataset "原始采点 rtk_points.md=src/rtk_nav/rtk_nav/config/rtk_points.md" --dataset "006 派生点=src/rtk_nav/rtk_nav/config/rtk_points_006-E22-W24.md"

# 无底图地图模式
python tools/plot_rtk_points.py --input src/rtk_nav/rtk_nav/config/rtk_points.md --output rtk_points.html
```

# RTK 点位图纸对齐教程

## 1. 启动

- 双击 `启动RTK图纸预览.bat`：默认打开 SVG 布置图页面。
- 双击 `启动RTK布置图1112预览.bat`：同样打开布置图 1112 页面。
- 打开原 006 对准页面：

```bash
start_rtk_preview.bat rtk_points_006-E22-W24_with_pdf.html
```

- 手动启动：`python -m http.server 8766 --bind 127.0.0.1`，访问 `http://localhost:8766/页面名`。
- 页面需要联网加载 Leaflet 1.9.4；`file://` 直接打开可能一直显示“加载中”。

## 2. 文件位置

| 内容 | 位置 |
|------|------|
| 点位序列 | `src/rtk_nav/rtk_nav/config/rtk_points*.md` |
| 标定文件 | `src/rtk_nav/rtk_nav/config/rtk_calibration_*.json` |
| 矢量底图 | 仓库根目录 `布置图1112.svg` |
| 对齐页面 | `rtk_points_*.html`（底图内嵌，单文件） |

`rtk_points.md` 由机器人 `WAIT_RTK` 自动追加，不要手改；混合场地按 `seq` 拆成独立文件。

## 3. 更换底图并生成页面

命令均为单行，可直接粘贴到 PowerShell 或 cmd。`^` 是 cmd 的续行符，PowerShell 不识别，请勿使用。

仓库当前只保留 SVG，已有底图直接用 `--background-image`（下面第二个命令）。新 PDF 可用 `--background-pdf ... --pdf-format svg` 生成（需 `pymupdf`）。

```bash
python tools/plot_rtk_points.py --input src/rtk_nav/rtk_nav/config/rtk_points_1112.md --output rtk_points_006-E22-W24_layout1112.html --background-pdf 图纸.pdf --pdf-format svg --dataset "原始采点 rtk_points.md=src/rtk_nav/rtk_nav/config/rtk_points.md"
```

已有 SVG 或图片时，用 `--background-image`；布置图 1112 的默认视图保存在 `rtk_calibration_1112.json`，点位文件为 BD09，用 `--coord-system bd09` 声明、`--calibration` 恢复视图：

```bash
python tools/plot_rtk_points.py --input src/rtk_nav/rtk_nav/config/rtk_points_1112.md --output rtk_points_006-E22-W24_layout1112.html --background-image 布置图1112.svg --coord-system bd09 --calibration src/rtk_nav/rtk_nav/config/rtk_calibration_1112.json --dataset "006 派生点=src/rtk_nav/rtk_nav/config/rtk_points_006-E22-W24.md" --dataset "原始采点 rtk_points.md=src/rtk_nav/rtk_nav/config/rtk_points.md" --dataset "E1-E8 派生点=src/rtk_nav/rtk_nav/config/rtk_points_001-E1-E8.md"
```

页面左侧“源坐标系”可切换 WGS84 / GCJ02 / BD09：选中的坐标系会先转换为 WGS84，再计算 UTM、标定和显示。BD09/GCJ02 数据选错会导致点位整体偏移数百米到 1 km。

- `--input`：默认点位序列。
- `--coord-system`：点位文件坐标系（`wgs84`/`gcj02`/`bd09`），页面会转换为 WGS84。
- `--dataset "名称=路径"`：增加到页面“点位序列”下拉框，可重复。
- 换底图或改分辨率后，控制点 `px/py` 会变化，必须重新标定。

## 4. 对准步骤

1. **初判方向**：用“水平镜像 / 垂直镜像 / 旋转180° / 旋转 / 平移 / 缩放”调整底图；点位始终不动。
2. **精确标定**：左侧选择一条采点 → “在图纸上拾取” → 点击图上对应位置。至少选 2 个相距远、特征清晰的控制点，残差应小于 2 px。
3. **交叉验证**：加第 3 个点，页面会先按旧标定预测再比较，误差小于 1 px 说明可信。
4. **导出**：点“导出标定 JSON”，把文件放回 `src/rtk_nav/rtk_nav/config/`。
5. **固化**：导出的 JSON 现在包含 `view`（缩放/旋转/镜像/平移），重新生成时加 `--calibration 标定.json`，页面会自动恢复旋转和平移。若 `control_points` 为空，只会恢复视图，点位仍是近似摆放。

辅助功能：点“拾取坐标（估算经纬度）”，再点图纸任意位置，左下角显示该点的 `lat,lon` 与 UTM，可复制。已完成标定时用标定反算；未标定时用近似摆放估算（结果仅供参考）。

## 5. 常见问题

- **标定不可信**：RMSE 超过 20 px 时页面会提示“控制点与图上位置不匹配”，并自动回退近似摆放，不会再用错误参数移动整页点位。删掉偏差大的控制点，换更远、位置明确的点重试。
- **点位整体歪 90°、RMSE 很大**：底图镜像方向不对，先试垂直/水平镜像，再重新拾取。
- **点位跑到图外**：选错点位序列，或该场地尚未标定（虚线为近似摆放）。
- **缩放时底图翻转**：旧页面问题，重新生成即可；镜像已放在 Leaflet 不会改写的容器上。
- **标定 JSON 的 `py` 与点击值不同**：006 标定保存了 `source_py`，`py` 是垂直镜像后的值，属正常。
- **页面卡在“加载中”**：确认通过 `http://localhost:8766` 访问且能联网加载 Leaflet。

## 6. 参考：当前页面命令

```bash
# 布置图 1112（SVG 矢量底图，默认 1112 序列）
python tools/plot_rtk_points.py --input src/rtk_nav/rtk_nav/config/rtk_points_1112.md --output rtk_points_006-E22-W24_layout1112.html --background-image 布置图1112.svg --dataset "006 派生点=src/rtk_nav/rtk_nav/config/rtk_points_006-E22-W24.md" --dataset "原始采点 rtk_points.md=src/rtk_nav/rtk_nav/config/rtk_points.md" --dataset "E1-E8 派生点=src/rtk_nav/rtk_nav/config/rtk_points_001-E1-E8.md"

# 006 对准页面（保留不动）
python tools/plot_rtk_points.py --input src/rtk_nav/rtk_nav/config/rtk_points_006-E22-W24.md --output rtk_points_006-E22-W24_with_pdf.html --background-image tmp/pdfs/001-E1-E8-base.png --calibration src/rtk_nav/rtk_nav/config/rtk_calibration_006-E22-W24.json --dataset "原始采点 rtk_points.md=src/rtk_nav/rtk_nav/config/rtk_points.md" --dataset "E1-E8 派生点=src/rtk_nav/rtk_nav/config/rtk_points_001-E1-E8.md"
```

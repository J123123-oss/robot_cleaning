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

新 PDF 可直接生成 SVG 页面（需 `pymupdf`）：

```bash
python tools/plot_rtk_points.py ^
  --input src/rtk_nav/rtk_nav/config/rtk_points_1112.md ^
  --output rtk_points_006-E22-W24_layout1112.html ^
  --background-pdf 图纸.pdf --pdf-format svg ^
  --dataset "原始采点 rtk_points.md=src/rtk_nav/rtk_nav/config/rtk_points.md"
```

已有 SVG 或图片时，用 `--background-image`：

```bash
python tools/plot_rtk_points.py ^
  --input src/rtk_nav/rtk_nav/config/rtk_points_1112.md ^
  --output rtk_points_006-E22-W24_layout1112.html ^
  --background-image 布置图1112.svg ^
  --dataset "006 派生点=src/rtk_nav/rtk_nav/config/rtk_points_006-E22-W24.md" ^
  --dataset "原始采点 rtk_points.md=src/rtk_nav/rtk_nav/config/rtk_points.md" ^
  --dataset "E1-E8 派生点=src/rtk_nav/rtk_nav/config/rtk_points_001-E1-E8.md"
```

- `--input`：默认点位序列。
- `--dataset "名称=路径"`：增加到页面“点位序列”下拉框，可重复。
- 换底图或改分辨率后，控制点 `px/py` 会变化，必须重新标定。

## 4. 对准步骤

1. **初判方向**：用“水平镜像 / 垂直镜像 / 旋转180° / 旋转 / 平移 / 缩放”调整底图；点位始终不动。
2. **精确标定**：左侧选择一条采点 → “在图纸上拾取” → 点击图上对应位置。至少选 2 个相距远、特征清晰的控制点，残差应小于 2 px。
3. **交叉验证**：加第 3 个点，页面会先按旧标定预测再比较，误差小于 1 px 说明可信。
4. **导出**：点“导出标定 JSON”，把文件放回 `src/rtk_nav/rtk_nav/config/`。
5. **固化**：重新生成时加 `--calibration 标定.json`。需要默认镜像时，在 JSON 顶层加 `"image_mirror_y": true`（或 `image_mirror_x`）。

## 5. 常见问题

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

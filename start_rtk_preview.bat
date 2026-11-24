@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"

set "PORT=8766"
set "PAGE=%~1"
if "%PAGE%"=="" set "PAGE=tools/rtk_preview/rtk_points_1112.html"

where python >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到 Python，请先安装 Python 并加入 PATH。
    pause
    exit /b 1
)

if "%~1"=="" (
    python tools/plot_rtk_points.py --input "src/rtk_nav/rtk_nav/config/rtk_points.md" --output "%PAGE%" --background-image "布置图1112.svg" --calibration "src/rtk_nav/rtk_nav/config/rtk_calibration.json"
    if errorlevel 1 (
        echo [错误] 生成默认标定页面失败，请检查采点、底图和标定文件。
        pause
        exit /b 1
    )
)

if not exist "%PAGE%" (
    echo [错误] 找不到页面文件：%PAGE%
    pause
    exit /b 1
)

python -c "import socket,sys; s=socket.socket(); s.settimeout(0.5); sys.exit(0 if s.connect_ex(('127.0.0.1',%PORT%))==0 else 1)"
if errorlevel 1 (
    start "RTK 图纸预览服务 - 8766" cmd /k python -m http.server %PORT% --bind 127.0.0.1
    ping -n 2 127.0.0.1 >nul
)
start "" "http://localhost:%PORT%/%PAGE%"

echo 已打开 http://localhost:%PORT%/%PAGE%
echo 关闭“RTK 图纸预览服务 - 8766”窗口即可停止服务。
ping -n 3 127.0.0.1 >nul

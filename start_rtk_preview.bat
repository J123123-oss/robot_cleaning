@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0"

set "PORT=8766"
set "PAGE=%~1"
if "%PAGE%"=="" set "PAGE=rtk_points_006-E22-W24_with_pdf.html"

if not exist "%PAGE%" (
    echo [错误] 找不到页面文件：%PAGE%
    pause
    exit /b 1
)

where python >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到 Python，请先安装 Python 并加入 PATH。
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

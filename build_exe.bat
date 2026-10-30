@echo off
chcp 65001 >nul
rem สร้างไฟล์ AI_XO.exe ใหม่ (ใช้หลังแก้โค้ด) · ผลลัพธ์อยู่ในโฟลเดอร์ exe
rem ต้องติดตั้ง PyInstaller ก่อน 1 ครั้ง:  python -m pip install --user pyinstaller
cd /d "%~dp0"
python make_icon.py
python -m PyInstaller --noconfirm --onefile --windowed --name AI_XO --distpath exe ^
    --icon "%~dp0app_icon.ico" --add-data "%~dp0app_icon.ico;." ^
    --workpath "%TEMP%\ai_xo_build" --specpath "%TEMP%\ai_xo_build" ui_app.py
if errorlevel 1 (
    echo.
    echo สร้างไม่สำเร็จ ดูข้อความด้านบน
    pause
    exit /b 1
)
rem คัดลอกค่าตั้งปัจจุบันไปไว้ข้าง .exe (ถ้ายังไม่มี)
if not exist "exe\robot_config.json" copy "robot_config.json" "exe\robot_config.json" >nul
echo.
echo เสร็จแล้ว: exe\AI_XO.exe
pause

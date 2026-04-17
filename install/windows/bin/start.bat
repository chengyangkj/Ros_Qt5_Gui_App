@echo off
setlocal
cd /d "%~dp0"
set "APP_DIR=%~dp0"
set "PATH=%APP_DIR%;%APP_DIR%..\lib;%PATH%"
set "QT_QPA_PLATFORM_PLUGIN_PATH=%APP_DIR%platforms"
start "" "%APP_DIR%ros_qt5_gui_app.exe"
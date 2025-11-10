@echo off
cd /d "%~dp0"
set PATH=%~dp0..\lib;%PATH%

REM 获取 build 目录的相对路径（假设 build 和 install 在同一级目录）
REM install\bin\start.bat -> ..\..\build\ros_qt5_gui_app.exe
set INSTALL_DIR=%~dp0..
set PROJECT_ROOT=%INSTALL_DIR%\..
set BUILD_DIR=%PROJECT_ROOT%\build
set EXECUTABLE=%BUILD_DIR%\ros_qt5_gui_app.exe

REM 如果 build 目录中的可执行程序存在，使用它；否则使用当前目录的
if exist "%EXECUTABLE%" (
    set PATH=%BUILD_DIR%\lib;%PATH%
    "%EXECUTABLE%"
) else (
    ros_qt5_gui_app.exe
)


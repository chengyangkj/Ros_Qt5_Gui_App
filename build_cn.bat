@echo off
setlocal

@REM # 设置清华镜像源
@REM $env:VCPKG_DOWNLOAD_MIRROR="https://mirrors.tuna.tsinghua.edu.cn/github-release/ninja-build/ninja/"
@REM $env:X_VCPKG_ASSET_SOURCES="x-azurl,https://mirrors.tuna.tsinghua.edu.cn/vcpkg/assets/"

@REM # 或者使用中科大镜像源
@REM $env:VCPKG_DOWNLOAD_MIRROR="https://mirrors.ustc.edu.cn/github-release/ninja-build/ninja/"
@REM $env:X_VCPKG_ASSET_SOURCES="x-azurl,https://mirrors.ustc.edu.cn/vcpkg/assets/"


set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"

set "CMAKE_EXTRA_ARGS=-Ddockwidget_GIT_REPOSITORY=https://gitee.com/kqz2007/qt-advanced-docking-system_github.git -Dnlohmann_json_GIT_REPOSITORY=https://gitee.com/athtan/json.git -Dyaml-cpp_GIT_REPOSITORY=https://gitee.com/dragonet_220/yaml-cpp.git -Dwebsocketpp_GIT_REPOSITORY=https://gitee.com/open-source-software_1/websocketpp.git"

call "%SCRIPT_DIR%build.bat" %*
exit /b %errorlevel%
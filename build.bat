@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%"

set "BUILD_MODE=full"
if /I "%~1"=="make" set "BUILD_MODE=make"

set "BUILD_DIR=build"
set "INSTALL_DIR=%BUILD_DIR%\install"
set "TRIPLET=x64-windows"
set "CONFIG=Release"
set "PREFERRED_VCPKG_ROOT=C:\Users\chengyangkj\vcpkg"
set "SELECTED_VCPKG_ROOT="

if exist "%PREFERRED_VCPKG_ROOT%\vcpkg.exe" (
  if defined VCPKG_ROOT (
    echo [INFO] Override existing VCPKG_ROOT.
  )
  echo [INFO] Use preferred vcpkg path: "%PREFERRED_VCPKG_ROOT%"
  set "SELECTED_VCPKG_ROOT=%PREFERRED_VCPKG_ROOT%"
) else if defined VCPKG_ROOT if exist "%VCPKG_ROOT%\vcpkg.exe" (
  echo [INFO] Use VCPKG_ROOT from environment: "%VCPKG_ROOT%"
  set "SELECTED_VCPKG_ROOT=%VCPKG_ROOT%"
) else if exist "%USERPROFILE%\vcpkg\vcpkg.exe" (
  echo [INFO] Use vcpkg from user profile: "%USERPROFILE%\vcpkg"
  set "SELECTED_VCPKG_ROOT=%USERPROFILE%\vcpkg"
) else (
  echo [ERROR] No usable vcpkg root found.
  echo [ERROR] Checked:
  echo [ERROR]   1. "%PREFERRED_VCPKG_ROOT%"
  echo [ERROR]   2. VCPKG_ROOT environment variable
  echo [ERROR]   3. "%USERPROFILE%\vcpkg"
  exit /b 1
)

set "VCPKG_ROOT=%SELECTED_VCPKG_ROOT%"

call :IsVcpkgWritable "%VCPKG_ROOT%"
if errorlevel 1 (
  echo [ERROR] No write permission for vcpkg path: "%VCPKG_ROOT%"
  exit /b 1
)

call :TrySetupVsDevEnv

set "VCPKG_ROOT=%SELECTED_VCPKG_ROOT%"
echo [INFO] Re-apply selected vcpkg path after VS env setup: "%VCPKG_ROOT%"

set "INSTALL_ROOT=%VCPKG_ROOT%\installed"

echo [INFO] Use vcpkg root: "%VCPKG_ROOT%"
echo [INFO] MSVC config: %CONFIG%
if /I "%BUILD_MODE%"=="make" (
  echo [INFO] mode=make: skip vcpkg install and cmake configure
  goto AfterConfigure
)

echo [1/4] Install vcpkg dependencies...
set "VCPKG_DISABLE_METRICS=1"
"%VCPKG_ROOT%\vcpkg.exe" install --triplet %TRIPLET% --x-install-root="%INSTALL_ROOT%"
if errorlevel 1 exit /b 1

echo [2/4] Configure CMake...
cmake -B "%BUILD_DIR%" -S . ^
  -DCMAKE_TOOLCHAIN_FILE="%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake" ^
  -DCMAKE_INSTALL_PREFIX="%INSTALL_DIR%" ^
  -DCMAKE_PREFIX_PATH="%VCPKG_ROOT%\installed\%TRIPLET%" ^
  -DQt5_DIR="%VCPKG_ROOT%\installed\%TRIPLET%\share\cmake\Qt5" ^
  -DQT_DIR="%VCPKG_ROOT%\installed\%TRIPLET%\share\cmake\Qt5" ^
  -DCMAKE_BUILD_TYPE=%CONFIG% ^
  -DBUILD_WITH_TEST=OFF ^
  %CMAKE_EXTRA_ARGS%
if errorlevel 1 exit /b 1

:AfterConfigure
echo [3/4] Build...
if not defined NUMBER_OF_PROCESSORS set NUMBER_OF_PROCESSORS=8
cmake --build "%BUILD_DIR%" --config %CONFIG% --parallel %NUMBER_OF_PROCESSORS%
if errorlevel 1 exit /b 1

echo [4/4] Install...
cmake --install "%BUILD_DIR%" --config %CONFIG% --prefix "%INSTALL_DIR%"
if errorlevel 1 exit /b 1

call :CopyRuntimeDlls
if errorlevel 1 exit /b 1

echo Build completed successfully.
exit /b 0

:TrySetupVsDevEnv
where cl >nul 2>nul
if not errorlevel 1 (
  where nmake >nul 2>nul
  if not errorlevel 1 (
    echo [INFO] MSVC build tools already available in current shell.
    exit /b 0
  )
)

set "VS_DEV_CMD=C:\Program Files\Microsoft Visual Studio\18\Community\Common7\Tools\VsDevCmd.bat"
if not exist "%VS_DEV_CMD%" (
  echo [WARNING] VS developer environment script not found:
  echo [WARNING]   "%VS_DEV_CMD%"
  echo [WARNING] Please install Visual Studio C++ tools or update VS_DEV_CMD path in build.bat.
  exit /b 0
)

echo [INFO] Try loading VS developer environment...
call "%VS_DEV_CMD%" -arch=x64 >nul 2>nul
if errorlevel 1 (
  echo [WARNING] Failed to load VS developer environment from:
  echo [WARNING]   "%VS_DEV_CMD%"
  echo [WARNING] Please open a Developer Command Prompt or update VS_DEV_CMD path in build.bat.
  exit /b 0
)

where cl >nul 2>nul
if errorlevel 1 (
  echo [WARNING] VS developer environment loaded, but cl was not found in PATH.
  echo [WARNING] Please verify Visual Studio C++ workload and local setup.
  exit /b 0
)

where nmake >nul 2>nul
if errorlevel 1 (
  echo [WARNING] VS developer environment loaded, but nmake was not found in PATH.
  echo [WARNING] Please verify Visual Studio C++ workload and local setup.
  exit /b 0
)

echo [INFO] VS developer environment is ready.
exit /b 0

:IsVcpkgWritable
set "WRITE_TEST_ROOT=%~1\installed\vcpkg"
set "WRITE_TEST_DIR=%WRITE_TEST_ROOT%\_write_test_%RANDOM%%RANDOM%"
2>nul mkdir "%WRITE_TEST_DIR%"
if errorlevel 1 exit /b 1
2>nul rmdir "%WRITE_TEST_DIR%"
exit /b 0

:CopyRuntimeDlls
set "INSTALL_BIN_DIR=%INSTALL_DIR%\bin"
if /I "%CONFIG%"=="Debug" (
  set "RUNTIME_BIN_DIR=%VCPKG_ROOT%\installed\%TRIPLET%\debug\bin"
  set "QT_PLUGIN_DIR=%VCPKG_ROOT%\installed\%TRIPLET%\debug\plugins"
) else (
  set "RUNTIME_BIN_DIR=%VCPKG_ROOT%\installed\%TRIPLET%\bin"
  set "QT_PLUGIN_DIR=%VCPKG_ROOT%\installed\%TRIPLET%\plugins"
)
if not exist "%INSTALL_BIN_DIR%" (
  echo [ERROR] Install bin directory not found: "%INSTALL_BIN_DIR%"
  exit /b 1
)
if not exist "%RUNTIME_BIN_DIR%" (
  echo [ERROR] Runtime bin directory not found: "%RUNTIME_BIN_DIR%"
  echo [ERROR] Debug builds need vcpkg debug DLLs under installed\%TRIPLET%\debug\bin .
  exit /b 1
)
echo [INFO] Copy runtime DLLs from "%RUNTIME_BIN_DIR%" to "%INSTALL_BIN_DIR%"...
xcopy "%RUNTIME_BIN_DIR%\*.dll" "%INSTALL_BIN_DIR%\" /Y /I >nul
if errorlevel 1 (
  echo [ERROR] Failed to copy runtime DLLs from "%RUNTIME_BIN_DIR%".
  exit /b 1
)
if exist "%QT_PLUGIN_DIR%" (
  echo [INFO] Copy Qt plugin directories...
  if exist "%QT_PLUGIN_DIR%\platforms" xcopy "%QT_PLUGIN_DIR%\platforms\*.dll" "%INSTALL_BIN_DIR%\platforms\" /Y /I >nul
  if exist "%QT_PLUGIN_DIR%\imageformats" xcopy "%QT_PLUGIN_DIR%\imageformats\*.dll" "%INSTALL_BIN_DIR%\imageformats\" /Y /I >nul
  if exist "%QT_PLUGIN_DIR%\iconengines" xcopy "%QT_PLUGIN_DIR%\iconengines\*.dll" "%INSTALL_BIN_DIR%\iconengines\" /Y /I >nul
  if exist "%QT_PLUGIN_DIR%\styles" xcopy "%QT_PLUGIN_DIR%\styles\*.dll" "%INSTALL_BIN_DIR%\styles\" /Y /I >nul
  if exist "%QT_PLUGIN_DIR%\platformthemes" xcopy "%QT_PLUGIN_DIR%\platformthemes\*.dll" "%INSTALL_BIN_DIR%\platformthemes\" /Y /I >nul
)
exit /b 0
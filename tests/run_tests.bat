@echo off
setlocal enabledelayedexpansion

echo ====================================================================
echo  Running ps3xpad Test Suites on Windows
echo ====================================================================

where cl >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo Initializing MSVC environment...
    if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" (
        call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
    ) else if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat" (
        call "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
    )
)

cd /d "%~dp0"

echo [1/3] Compiling and running test_switch_pro...
cl /nologo /W3 /O2 test_switch_pro.c /Fe:test_switch_pro.exe >nul
if %ERRORLEVEL% neq 0 (
    echo Failed to build test_switch_pro
    exit /b 1
)
test_switch_pro.exe
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%

echo.
echo [2/3] Compiling and running test_bt_dongle...
cl /nologo /W3 /O2 test_bt_dongle.c /Fe:test_bt_dongle.exe >nul
if %ERRORLEVEL% neq 0 (
    echo Failed to build test_bt_dongle
    exit /b 1
)
test_bt_dongle.exe
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%

echo.
echo [3/4] Compiling and running test_hen_integration...
cl /nologo /W3 /O2 test_hen_integration.c /Fe:test_hen_integration.exe >nul
if %ERRORLEVEL% neq 0 (
    echo Failed to build test_hen_integration
    exit /b 1
)
test_hen_integration.exe
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%

echo.
echo [4/4] Compiling and running test_xmb_ui...
cl /nologo /W3 /O2 test_xmb_ui.c /Fe:test_xmb_ui.exe >nul
if %ERRORLEVEL% neq 0 (
    echo Failed to build test_xmb_ui
    exit /b 1
)
test_xmb_ui.exe
if %ERRORLEVEL% neq 0 exit /b %ERRORLEVEL%

echo.
echo ====================================================================
echo  ALL TEST SUITES PASSED CLEANLY
echo ====================================================================

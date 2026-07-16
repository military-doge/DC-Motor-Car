@echo off
setlocal enabledelayedexpansion

:: Read sdk_config.ini from project root (two levels up from tools/keil/)
set CONFIG_FILE=%~dp0..\..\sdk_config.ini

:: Validate config file exists
if not exist "%CONFIG_FILE%" (
    echo.
    echo Couldn't find sdk_config.ini at %CONFIG_FILE%
    echo.
    exit /b 1
)

:: Read SYSCFG path from [SYSCFG] section
set SYSCFG_PATH=
for /f "delims=" %%a in ('type "%CONFIG_FILE%"') do (
    set "line=%%a"
    if "!line!"=="[SYSCFG]" set "READ_PATH=1"
    if "!line!"=="[DEVICE]" set "READ_PATH=0"
    if "!READ_PATH!"=="1" (
        for /f "tokens=1,* delims== " %%x in ("!line!") do (
            if "%%x"=="path" set "SYSCFG_PATH=%%y"
        )
    )
)

set "SYSCFG_CLI=%SYSCFG_PATH%\sysconfig_cli.bat"

:: Read SDK path from [MSPM0_SDK] section
set SDK_PATH=
set READ_PATH=0
for /f "delims=" %%a in ('type "%CONFIG_FILE%"') do (
    set "line=%%a"
    if "!line!"=="[MSPM0_SDK]" set "READ_PATH=1"
    if "!line!"=="[SYSCFG]" set "READ_PATH=0"
    if "!READ_PATH!"=="1" (
        for /f "tokens=1,* delims== " %%x in ("!line!") do (
            if "%%x"=="path" set "SDK_PATH=%%y"
        )
    )
)

:: Validate SYSCFG tool
if not exist "%SYSCFG_CLI%" (
    echo.
    echo Couldn't find Sysconfig Tool at %SYSCFG_CLI%
    echo Please update the path in sdk_config.ini [SYSCFG] section.
    echo.
    exit /b 1
)

:: Validate SDK root
if not exist "%SDK_PATH%\.metadata\product.json" (
    echo.
    echo Couldn't find SDK at %SDK_PATH%
    echo Please update the path in sdk_config.ini [MSPM0_SDK] section.
    echo.
    exit /b 1
)

echo Using Sysconfig Tool from %SYSCFG_CLI%
echo Using SDK from %SDK_PATH%

set PROJ_DIR=%~1
set PROJ_DIR=%PROJ_DIR:'=%

set SYSCFG_FILE=%~2
set SYSCFG_FILE=%SYSCFG_FILE:'=%

:: Search for the directory containing the project's syscfg file
:: First try the exact file path relative to project dir
if exist "%PROJ_DIR%%SYSCFG_FILE%" (
    for %%f in ("%PROJ_DIR%%SYSCFG_FILE%") do set SYSCFG_DIR=%%~dpf
    IF "!SYSCFG_DIR:~-1!"=="\" SET "SYSCFG_DIR=!SYSCFG_DIR:~0,-1!"
    goto syscfg_search_exit
)
:: Fall back to recursive search from project dir
for /r "%PROJ_DIR%" %%f in (*.syscfg) do (
    set "SYSCFG_DIR=%%~dpf"
    IF "!SYSCFG_DIR:~-1!"=="\" SET "SYSCFG_DIR=!SYSCFG_DIR:~0,-1!"
    goto syscfg_search_exit
)
echo "Couldn't find syscfg file"
exit /b 1
:syscfg_search_exit

%SYSCFG_CLI% -o "%SYSCFG_DIR%" -s "%SDK_PATH%\.metadata\product.json" --compiler keil "%PROJ_DIR%%SYSCFG_FILE%"

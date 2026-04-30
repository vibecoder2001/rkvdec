@echo off
setlocal EnableDelayedExpansion

REM Reinstalls rkiommu, rkmpp_ccu, rkmpp from .\driver\<name>\ARM64\Debug\<name>.inf
REM Uninstall order: rkmpp -> rkmpp_ccu -> rkiommu  (children before parent)
REM Install order:   rkiommu -> rkmpp_ccu -> rkmpp

if not "%~1"=="elevated" (
    net session >nul 2>&1
    if errorlevel 1 (
        echo Requesting admin...
        powershell -NoProfile -Command "Start-Process -Verb RunAs -FilePath '%~f0' -ArgumentList 'elevated'"
        exit /b
    )
)

pushd "%~dp0"

echo.
echo === Uninstalling existing drivers ===
call :uninstall rkmpp.inf
call :uninstall rkmpp_ccu.inf
call :uninstall rkiommu.inf

echo.
echo === Installing drivers ===
call :install driver\rkiommu\ARM64\Debug\rkiommu.inf
call :install driver\rkmpp_ccu\ARM64\Debug\rkmpp_ccu.inf
call :install driver\rkmpp\ARM64\Debug\rkmpp.inf

echo.
echo === Done ===
popd
pause
exit /b

:uninstall
set "TARGET=%~1"
echo.
echo --- Removing %TARGET% ---
for /f "tokens=*" %%A in ('pnputil /enum-drivers ^| findstr /I /C:"Published Name" /C:"Original Name"') do (
    set "LINE=%%A"
    echo !LINE! | findstr /I /C:"Published Name" >nul && (
        for /f "tokens=2 delims=:" %%B in ("!LINE!") do set "PUB=%%B"
        set "PUB=!PUB: =!"
    )
    echo !LINE! | findstr /I /C:"Original Name" >nul && (
        for /f "tokens=2 delims=:" %%B in ("!LINE!") do set "ORIG=%%B"
        set "ORIG=!ORIG: =!"
        if /I "!ORIG!"=="%TARGET%" (
            echo   pnputil /delete-driver !PUB! /uninstall /force
            pnputil /delete-driver !PUB! /uninstall /force
        )
    )
)
exit /b

:install
set "INF=%~1"
echo.
echo --- Installing %INF% ---
if not exist "%INF%" (
    echo   ERROR: %INF% not found
    exit /b 1
)
pnputil /add-driver "%INF%" /install
exit /b

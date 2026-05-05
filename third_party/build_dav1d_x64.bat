@echo off
REM Build dav1d for x64-Windows (host arch on the dev machine).
REM Output: third_party/dav1d/build-x64/src/libdav1d.a
REM Allows running the AV1 regbuilder + parser harness on the dev box
REM without an ARM64 board.  Performance is irrelevant — dav1d is used
REM as a header parser, not for full decode.

setlocal
set "PATH=C:\Program Files (x86)\Microsoft Visual Studio\Installer;C:\Program Files\Meson;%PATH%"
call "D:\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 >nul || (echo vcvarsall failed & exit /b 1)

cd /d "%~dp0dav1d" || exit /b 1

if not exist build-x64 (
  meson setup build-x64 ^
    --default-library=static ^
    --buildtype=release ^
    -Denable_tools=false ^
    -Denable_tests=false ^
    -Denable_asm=false || exit /b 1
)

meson compile -C build-x64 || exit /b 1

echo Built: %~dp0dav1d\build-x64\src\libdav1d.a
endlocal

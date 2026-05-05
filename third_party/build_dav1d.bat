@echo off
REM Build the dav1d static library for ARM64-Windows (MSVC).
REM Output: third_party/dav1d/build/src/libdav1d.a
REM We disable asm (no gas-preprocessor.pl on the host) and tools/tests.
REM Performance is irrelevant — dav1d is used only for AV1 OBU parsing
REM (frame/sequence header extraction); the actual decode runs on RK3588 hardware.

setlocal
set "PATH=C:\Program Files (x86)\Microsoft Visual Studio\Installer;C:\Program Files\Meson;%PATH%"
call "D:\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64_arm64 >nul || (echo vcvarsall failed & exit /b 1)

cd /d "%~dp0dav1d" || exit /b 1

if not exist build (
  meson setup build ^
    --cross-file ../arm64-windows-msvc.txt ^
    --default-library=static ^
    --buildtype=release ^
    -Denable_tools=false ^
    -Denable_tests=false ^
    -Denable_asm=false || exit /b 1
)

meson compile -C build || exit /b 1

echo Built: %~dp0dav1d\build\src\libdav1d.a
endlocal

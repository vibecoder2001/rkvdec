@echo off
setlocal
cd /d C:\Users\vibecoder\rkvdec

REM av1_parse_smoke removed from the default build set — it requires
REM the real dav1d static lib (third_party/build_dav1d.bat).  The
REM clean-room variant is enough for routine builds.
cmake --build build\harness-arm64 --config Release --target rkmpp_smoke rkmpp_caps_dump rkmpp_iommu_fault rkmpp_decode rkmpp_decoder_mft mft_decode mft_play av1_parse_clean_smoke || exit /b 1

set "DST=Z:\drivers-arm\rkvdec\tools"
copy /Y "build\harness-arm64\rkmpp_smoke\Release\rkmpp_smoke.exe"                       "%DST%\" >nul || exit /b 1
copy /Y "build\harness-arm64\rkmpp_caps_dump\Release\rkmpp_caps_dump.exe"               "%DST%\" >nul || exit /b 1
copy /Y "build\harness-arm64\rkmpp_iommu_fault\Release\rkmpp_iommu_fault.exe"           "%DST%\" >nul || exit /b 1
copy /Y "build\harness-arm64\rkmpp_decode\Release\rkmpp_decode.exe"                     "%DST%\" >nul || exit /b 1
copy /Y "build\harness-arm64\mft_dll\Release\rkmpp_decoder_mft.dll"                    "%DST%\" >nul || exit /b 1
copy /Y "build\harness-arm64\mft_decode\Release\mft_decode.exe"                        "%DST%\" >nul || exit /b 1
copy /Y "build\harness-arm64\mft_play\Release\mft_play.exe"                            "%DST%\" >nul || exit /b 1
REM av1_parse_smoke.exe only emitted when build_dav1d.bat was run; copy
REM if present, otherwise skip silently.
if exist "build\harness-arm64\av1_parse_smoke\Release\av1_parse_smoke.exe" copy /Y "build\harness-arm64\av1_parse_smoke\Release\av1_parse_smoke.exe" "%DST%\" >nul
copy /Y "build\harness-arm64\av1_parse_smoke\Release\av1_parse_clean_smoke.exe"        "%DST%\" >nul || exit /b 1
echo Deployed Release harness tools to %DST%
echo.
echo To register the AV1 decoder on the ARM target:
echo   regsvr32 rkmpp_decoder_mft.dll

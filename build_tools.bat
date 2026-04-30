@echo off
setlocal
cd /d C:\Users\vibecoder\rkvdec

cmake --build build\harness-arm64 --config Release --target rkmpp_smoke rkmpp_caps_dump rkmpp_iommu_fault rkmpp_decode || exit /b 1

set "DST=Z:\drivers-arm\rkvdec\tools"
copy /Y "build\harness-arm64\rkmpp_smoke\Release\rkmpp_smoke.exe"             "%DST%\" >nul || exit /b 1
copy /Y "build\harness-arm64\rkmpp_caps_dump\Release\rkmpp_caps_dump.exe"     "%DST%\" >nul || exit /b 1
copy /Y "build\harness-arm64\rkmpp_iommu_fault\Release\rkmpp_iommu_fault.exe" "%DST%\" >nul || exit /b 1
copy /Y "build\harness-arm64\rkmpp_decode\Release\rkmpp_decode.exe"           "%DST%\" >nul || exit /b 1
echo Deployed Release harness tools to %DST%

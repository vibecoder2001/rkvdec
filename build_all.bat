@echo off
setlocal
call "D:\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64_arm64
set "WDKContentRoot=D:\Windows Kits\10\"
cd /d C:\Users\vibecoder\rkvdec

set "MSBARGS=/p:Configuration=Debug /p:Platform=ARM64 /p:WindowsTargetPlatformVersion=10.0.26100.0 /v:minimal"

for %%P in (rkmpp_ccu rkmpp rkiommu) do (
  echo === Building %%P ===
  msbuild driver\%%P\%%P.vcxproj %MSBARGS% || exit /b 1
  copy /Y "driver\%%P\ARM64\Debug\%%P\%%P.sys" "Z:\drivers-arm\rkvdec\%%P\" >nul || exit /b 1
  copy /Y "driver\%%P\ARM64\Debug\%%P\%%P.cat" "Z:\drivers-arm\rkvdec\%%P\" >nul || exit /b 1
  copy /Y "driver\%%P\ARM64\Debug\%%P\%%P.inf" "Z:\drivers-arm\rkvdec\%%P\" >nul || exit /b 1
  copy /Y "driver\%%P\ARM64\Debug\%%P.pdb"     "Z:\drivers-arm\rkvdec\%%P\" >nul || exit /b 1
  echo Deployed %%P sys/cat/inf/pdb
)

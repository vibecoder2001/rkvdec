@echo off
setlocal
call "D:\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64_arm64
set "WDKContentRoot=D:\Windows Kits\10\"
cd /d C:\Users\vibecoder\rkvdec

set "MSBARGS=/p:Configuration=Release /p:Platform=ARM64 /p:WindowsTargetPlatformVersion=10.0.26100.0 /v:minimal"

for %%P in (rkmpp_ccu rkiommu_vdec rkiommu_av1d rkvdec rkav1d) do (
  echo === Building %%P ^(Release^) ===
  msbuild driver\%%P\%%P.vcxproj %MSBARGS% || exit /b 1
  if not exist "Z:\drivers-arm\rkvdec\%%P\" mkdir "Z:\drivers-arm\rkvdec\%%P\"
  copy /Y "driver\%%P\ARM64\Release\%%P\%%P.sys" "Z:\drivers-arm\rkvdec\%%P\" >nul
  copy /Y "driver\%%P\ARM64\Release\%%P\%%P.cat" "Z:\drivers-arm\rkvdec\%%P\" >nul
  copy /Y "driver\%%P\ARM64\Release\%%P\%%P.inf" "Z:\drivers-arm\rkvdec\%%P\" >nul
  copy /Y "driver\%%P\ARM64\Release\%%P.pdb"     "Z:\drivers-arm\rkvdec\%%P\" >nul
  echo Deployed %%P sys/cat/inf/pdb
)

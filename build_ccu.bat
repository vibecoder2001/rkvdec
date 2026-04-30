@echo off
setlocal
call "D:\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64_arm64
set "WDKContentRoot=D:\Windows Kits\10\"
cd /d C:\Users\vibecoder\rkvdec
msbuild driver\rkmpp_ccu\rkmpp_ccu.vcxproj /p:Configuration=Debug /p:Platform=ARM64 /p:WindowsTargetPlatformVersion=10.0.26100.0 /v:minimal
if errorlevel 1 exit /b %errorlevel%

set "SRC=driver\rkmpp_ccu\ARM64\Debug"
set "DST=Z:\drivers-arm\rkvdec\rkmpp_ccu"
copy /Y "%SRC%\rkmpp_ccu\rkmpp_ccu.sys" "%DST%\" >nul || exit /b 1
copy /Y "%SRC%\rkmpp_ccu\rkmpp_ccu.cat" "%DST%\" >nul || exit /b 1
copy /Y "%SRC%\rkmpp_ccu\rkmpp_ccu.inf" "%DST%\" >nul || exit /b 1
copy /Y "%SRC%\rkmpp_ccu.pdb"           "%DST%\" >nul || exit /b 1
echo Deployed sys/cat/inf/pdb to %DST%

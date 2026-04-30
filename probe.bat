@echo off
call "D:\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64_arm64
echo === MicrosoftKitRoot=%MicrosoftKitRoot%
echo === WDKContentRoot=%WDKContentRoot%
echo === Reading registry...
reg query "HKLM\SOFTWARE\Microsoft\Windows Kits\Installed Roots" /v KitsRoot10

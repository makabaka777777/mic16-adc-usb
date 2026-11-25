@echo off
chcp 936 >nul
setlocal
set GIT="D:\tools\PortableGit\cmd\git.exe"
set PROJ=D:\stm32p\MIC16_ADC123_USB

%GIT% config --global user.name  "makabaka777777"
%GIT% config --global user.email "makabaka777777@users.noreply.github.com"
%GIT% config --global init.defaultBranch main
%GIT% config --global core.quotepath false
%GIT% config --global core.autocrlf false

echo ==================================================
echo   推送  MIC16_ADC123_USB  --^>  GitHub
echo ==================================================
cd /d "%PROJ%"
echo.
echo [当前改动]
%GIT% status --short
echo.
set "MSG="
set /p "MSG=请输入本次修改说明（直接回车 = 用默认）: "
if "%MSG%"=="" set "MSG=更新代码"

%GIT% add -A
%GIT% commit -m "%MSG%"
echo.
echo [上传中，第一次会弹浏览器让你登录 GitHub，点 Authorize 即可]
%GIT% push -u origin main

if errorlevel 1 (
  echo.
  echo ##########################################
  echo   推送失败：把上面的提示截图发给助手
  echo ##########################################
) else (
  echo.
  echo ==================================================
  echo   推送成功！正在打开仓库页面...
  echo ==================================================
  start https://github.com/makabaka777777/mic16-adc-usb
)
echo.
pause
@echo off

echo *****************************************
echo ***          jacky.li tools           ***
echo *****************************************
set CURRENT_TIME=%time:~0,2%:%time:~3,2%:%time:~6,2%
Echo %CURRENT_TIME%


echo current patch %~dp0%
start  chrome.exe "file://%~dp0%_html\index.html"

echo start  chrome.exe "file://%~dp0%_html\index.html"
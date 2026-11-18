@echo off
rem ------------------------------------------------------------------
rem  Starship tower-catch simulation launcher
rem  Opens index.html in a clean app window (Chrome or Edge), falling
rem  back to the default browser. three.js loads from a CDN, so the
rem  first launch needs an internet connection.
rem ------------------------------------------------------------------
setlocal
set "HTML=%~dp0index.html"
set "URL=file:///%HTML:\=/%"
set "CHROME=%ProgramFiles%\Google\Chrome\Application\chrome.exe"
set "CHROME86=%ProgramFiles(x86)%\Google\Chrome\Application\chrome.exe"
set "EDGE=%ProgramFiles(x86)%\Microsoft\Edge\Application\msedge.exe"

if exist "%CHROME%"   goto chrome
if exist "%CHROME86%" goto chrome86
if exist "%EDGE%"     goto edge
start "" "%HTML%"
goto :eof

:chrome
start "" "%CHROME%" --app="%URL%" --start-maximized
goto :eof
:chrome86
start "" "%CHROME86%" --app="%URL%" --start-maximized
goto :eof
:edge
start "" "%EDGE%" --app="%URL%" --start-maximized
goto :eof

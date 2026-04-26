@echo off
setlocal enabledelayedexpansion

:: ============================================================
:: G Pro Compat Mode Capture Battery (autocenter investigation)
:: ============================================================
:: The RS50's "G Pro compatibility mode" (toggle in the wheel's
:: OLED menu) makes the firmware enumerate as a Logitech G Pro
:: Racing Wheel for Xbox/PC (VID 046d, PID c272). In that mode
:: the firmware applies a default auto-center spring until GHUB
:: explicitly disables it. Native RS50 (PID c276) does NOT do
:: this, so our existing captures do not contain the disable
:: command.
::
:: This script captures three short scenarios in compat mode so
:: we can find the disable-autocenter command and add it to the
:: Linux driver's compat-mode probe path.
::
:: Output: tools\<DATE>_compat_*.pcapng
::
:: Run from an Administrator CMD prompt. Wireshark + USBPcap
:: must be installed.
:: ============================================================

set TSHARK=C:\Program Files\Wireshark\tshark.exe
set INTERFACE=\\.\USBPcap1

:: Output to repo-convention dev\captures\ (sibling of tools\). Resolve
:: against the script's own location so it works regardless of where
:: the user invokes it from.
set CAPTURE_DIR=%~dp0..\dev\captures
if "%CAPTURE_DIR:~-1%"=="\" set CAPTURE_DIR=%CAPTURE_DIR:~0,-1%
if not exist "%CAPTURE_DIR%" mkdir "%CAPTURE_DIR%"

if not exist "%TSHARK%" (
    echo ERROR: tshark not found at %TSHARK%
    echo Install Wireshark with the tshark and USBPcap components.
    pause
    exit /b 1
)

for /f %%i in ('powershell -command "Get-Date -Format yyyy-MM-dd"') do set DATE=%%i

if /i "%~1"=="" goto :show_help
if /i "%~1"=="help" goto :show_help
if /i "%~1"=="all" goto :run_all
if /i "%~1"=="firmware" goto :run_firmware
if /i "%~1"=="ghub-init" goto :run_ghub_init
if /i "%~1"=="autocenter-toggle" goto :run_autocenter_toggle

echo Unknown command: %~1
echo.
goto :show_help

:show_help
cls
echo ==================================================================
echo   G Pro Compat Mode Capture Battery
echo ==================================================================
echo.
echo Usage: windows_gpro_compat_capture.bat ^<command^>
echo.
echo   all                  Run all three captures back-to-back
echo   firmware             Wheel boot, NO GHUB running (firmware default)
echo   ghub-init            Wheel boot WITH GHUB running (the key one)
echo   autocenter-toggle    Toggle every centering knob GHUB exposes
echo.
echo PREREQUISITES (read carefully):
echo   1. Wheel switched to "G Pro compatibility mode" via the
echo      wheel's OLED menu. Confirm Device Manager shows
echo      "Logitech G Pro Racing Wheel" with VID_046D and PID_C272.
echo      (NOT C276 - that is native RS50 mode.)
echo   2. USBPcap1 is the USB root hub the wheel is plugged into.
echo      If unsure, run Wireshark once and look at the available
echo      USBPcap interfaces. Edit INTERFACE at the top of this
echo      script if needed.
echo   3. Each capture is short (~30 seconds). Follow the prompts
echo      exactly, especially the "wait" pauses - the autocenter
echo      command may be sent late in the init sequence.
echo.
echo Output: %CAPTURE_DIR%\%%DATE%%_compat_^<name^>.pcapng
echo.
pause
exit /b 0

:: ============================================================
:run_all
cls
echo ==================================================================
echo   ALL COMPAT MODE CAPTURES
echo ==================================================================
echo.
echo Sequence:
echo   1. firmware        (wheel boot, NO GHUB)
echo   2. ghub-init       (wheel boot WITH GHUB - the important one)
echo   3. autocenter-toggle  (toggle every centering knob in GHUB)
echo.
echo Press ENTER to begin...
pause >nul
call :run_firmware
call :run_ghub_init
call :run_autocenter_toggle

cls
echo ##################################################################
echo #  COMPAT MODE CAPTURE SEQUENCE COMPLETE                         #
echo ##################################################################
echo.
dir /b "%CAPTURE_DIR%\%DATE%_compat_*.pcapng" 2>nul
echo.
echo Files are in this tools\ folder. Copy them back to the Linux
echo side (e.g. shared folder) for analysis.
echo.
pause
goto :eof

:: ============================================================
:run_firmware
cls
echo ==================================================================
echo   1/3: Wheel boot, NO GHUB (firmware default state)
echo ==================================================================
echo.
echo PURPOSE: Capture exactly what the wheel does on its own when
echo          GHUB is NOT running. Establishes the firmware default
echo          baseline - we expect the auto-center spring to be on
echo          throughout this capture.
echo.
echo PREPARATION:
echo   1. EXIT GHUB completely. Right-click the GHUB tray icon
echo      and select Quit. Confirm in Task Manager that no
echo      "lghub*" processes are running.
echo   2. UNPLUG the wheel from USB. Wait 5 seconds. (Do not just
echo      power off - the firmware may keep state.)
echo   3. Press ENTER when ready - capture will start, then prompt
echo      you to plug the wheel back in.
echo.
pause >nul
call :start_capture "compat_firmware"
echo.
echo NOW:
echo   1. Plug the wheel back into USB.
echo   2. Wait 15 seconds for the wheel to fully boot. (Look at
echo      the OLED - it should finish its splash screen.)
echo   3. Try turning the wheel by hand briefly - does it spring
echo      back to center? Note your observation in the issue.
echo   4. Press ENTER below.
echo.
pause >nul
call :stop_capture "compat_firmware"
goto :eof

:: ============================================================
:run_ghub_init
cls
echo ==================================================================
echo   2/3: Wheel boot WITH GHUB (the important capture)
echo ==================================================================
echo.
echo PURPOSE: This is the one we actually need. Capture the full
echo          init handshake when GHUB is running and the wheel
echo          comes online. Somewhere in here is the command that
echo          disables the firmware's default auto-center spring.
echo.
echo PREPARATION:
echo   1. UNPLUG the wheel from USB. Wait 5 seconds.
echo   2. Open GHUB. Wait until its main window is shown and it
echo      has finished any startup work (no spinners).
echo   3. Press ENTER below. Capture will start, then prompt you
echo      to plug the wheel in.
echo.
pause >nul
call :start_capture "compat_ghub_init"
echo.
echo NOW:
echo   1. Plug the wheel back into USB.
echo   2. Wait until GHUB shows the wheel as connected and the
echo      OLED has finished booting (~15-20 seconds).
echo   3. Try turning the wheel by hand. The auto-center should
echo      now be GONE (or much weaker). Note your observation.
echo   4. Wait an additional 5 seconds for any late commands.
echo   5. Press ENTER below.
echo.
pause >nul
call :stop_capture "compat_ghub_init"
goto :eof

:: ============================================================
:run_autocenter_toggle
cls
echo ==================================================================
echo   3/3: Toggle every centering knob in GHUB
echo ==================================================================
echo.
echo PURPOSE: With GHUB connected to the wheel, toggle every
echo          setting that affects the centering spring (auto-
echo          center strength, dampening, etc.) so we see each
echo          enable/disable command isolated.
echo.
echo PREPARATION:
echo   1. GHUB is running and the wheel is connected.
echo   2. Open the wheel's settings page in GHUB.
echo   3. Identify any setting that influences how strongly the
echo      wheel returns to center. On a G Pro Wheel this might
echo      be called "Center Spring", "Auto-center", or similar.
echo      It might be hidden under an "Advanced" tab.
echo   4. Press ENTER below.
echo.
pause >nul
call :start_capture "compat_autocenter_toggle"
echo.
echo NOW in GHUB:
echo   1. If there is a Center Spring / Auto-center toggle:
echo        a. Turn it ON.  Wait 3 seconds.
echo        b. Turn it OFF. Wait 3 seconds.
echo        c. Turn it ON.  Wait 3 seconds.
echo   2. If there is a Center Spring strength slider:
echo        a. Move it to 100%%. Wait 2 seconds.
echo        b. Move it to 50%%.  Wait 2 seconds.
echo        c. Move it to 0%%.   Wait 2 seconds.
echo   3. If GHUB has FFB Strength / Master Gain:
echo        a. Move it to 100%%. Wait 2 seconds.
echo        b. Move it to 0%%.   Wait 2 seconds.
echo   4. Wait 3 seconds final pause for any commit traffic.
echo   5. Press ENTER below.
echo.
echo If GHUB does NOT expose any centering knob for this wheel,
echo just press ENTER - the absence of such a knob is itself
echo useful information (note it in the GitHub issue).
echo.
pause >nul
call :stop_capture "compat_autocenter_toggle"
goto :eof

:: ============================================================
:: HELPERS
:: ============================================================

:start_capture
set CURRENT_CAPTURE=%~1
set OUTPUT_FILE=%CAPTURE_DIR%\%DATE%_%CURRENT_CAPTURE%.pcapng
echo.
echo   Starting capture: %CURRENT_CAPTURE%
echo   Output: %OUTPUT_FILE%
echo.
start "" /b "%TSHARK%" -i "%INTERFACE%" -w "%OUTPUT_FILE%" -q
timeout /t 2 /nobreak >nul
tasklist /fi "imagename eq tshark.exe" 2>nul | findstr /i "tshark.exe" >nul
if errorlevel 1 (
    echo   ERROR: tshark failed to start.
    echo   Run this script as Administrator and check USBPcap install.
    pause
    goto :eof
)
echo   [RECORDING STARTED]
echo.
goto :eof

:stop_capture
taskkill /f /im tshark.exe >nul 2>&1
timeout /t 1 /nobreak >nul
echo.
echo   [RECORDING STOPPED]
echo   Saved: %DATE%_%~1.pcapng
if exist "%CAPTURE_DIR%\%DATE%_%~1.pcapng" (
    for %%A in ("%CAPTURE_DIR%\%DATE%_%~1.pcapng") do echo   Size: %%~zA bytes
)
echo.
timeout /t 2 /nobreak >nul
goto :eof

@echo off
setlocal enabledelayedexpansion

:: ============================================================
:: Logitech RS50 Windows Wheel/G-Hub Captures (community edition)
:: ============================================================
:: This is the no-game subset of the RE capture battery.
:: These captures only need an RS50 attached to Windows with G Hub
:: running - no games required. Contributors who don't own BeamNG
:: or Assetto Corsa can still help a lot by running these.
::
:: Each capture exercises a specific setting or wheel interaction
:: (calibration, FFB filter, D-pad, profile button, LIGHTSYNC etc.)
:: so we can see exactly what G Hub writes to the device over USB.
::
:: See docs/WINDOWS_RE_CAPTURE_GUIDE.md for full context.
::
:: Output files land next to this script, named so each one is
:: self-describing when attached to the issue tracker. Example:
::   2026-04-18_calibrate.pcapng
::   2026-04-18_ffb_filter.pcapng
::   2026-04-18_dpad.pcapng
::   2026-04-18_commit_0x8127.pcapng
::   2026-04-18_persist_before.pcapng
::   2026-04-18_persist_after.pcapng
::   2026-04-18_profile_button.pcapng
::   2026-04-18_lightsync_effects.pcapng
:: ============================================================

:: Configuration - edit if your install paths differ
set TSHARK=C:\Program Files\Wireshark\tshark.exe
set CAPTURE_DIR=%~dp0
set INTERFACE=\\.\USBPcap1

:: Trim trailing backslash from CAPTURE_DIR for tidy paths
if "%CAPTURE_DIR:~-1%"=="\" set CAPTURE_DIR=%CAPTURE_DIR:~0,-1%

:: Verify tshark
if not exist "%TSHARK%" (
    echo ERROR: tshark not found at %TSHARK%
    echo Install Wireshark with the tshark and USBPcap components.
    pause
    exit /b 1
)

:: Date prefix from PowerShell
for /f %%i in ('powershell -command "Get-Date -Format yyyy-MM-dd"') do set DATE=%%i

:: Menu dispatch
if /i "%~1"=="" goto :show_help
if /i "%~1"=="help" goto :show_help
if /i "%~1"=="all" goto :run_all

if /i "%~1"=="calibrate" goto :run_calibrate
if /i "%~1"=="filter" goto :run_filter
if /i "%~1"=="dpad" goto :run_dpad
if /i "%~1"=="commit" goto :run_commit
if /i "%~1"=="persist-before" goto :run_persist_before
if /i "%~1"=="persist-after" goto :run_persist_after
if /i "%~1"=="profile" goto :run_profile
if /i "%~1"=="lightsync" goto :run_lightsync

echo Unknown command: %~1
echo.
goto :show_help

:: ============================================================
:show_help
cls
echo ==================================================================
echo   RS50 Windows Wheel/G-Hub Captures (community edition)
echo ==================================================================
echo.
echo Usage: windows_wheel_captures.bat ^<command^>
echo.
echo   all              Run every capture in sequence (~20-30 min)
echo   calibrate        G Hub calibration routine
echo   filter           FFB filter slider + auto toggle
echo   dpad             D-pad directions (all 8 / 4)
echo   commit           Single settings change with surrounding traffic
echo   persist-before   Set Damping, unplug the wheel
echo   persist-after    Replug, read back Damping (run after persist-before)
echo   profile          Press profile/mode button on the wheel rim
echo   lightsync        Try each LIGHTSYNC effect type on a custom slot
echo.
echo Prerequisites:
echo   - Windows with Wireshark + USBPcap installed
echo     - Wireshark: https://www.wireshark.org/download.html
echo     - USBPcap (if missing): https://desowin.org/usbpcap/
echo   - Run this script from an Administrator CMD prompt
echo   - RS50 wheel attached and recognized by G Hub
echo.
echo Output: %CAPTURE_DIR%\%%DATE%%_^<name^>.pcapng
echo         (files land next to this script in the tools\ folder)
echo.
echo After capture: attach the pcapng files to a GitHub issue.
echo.
pause
exit /b 0

:: ============================================================
:run_all
cls
echo ==================================================================
echo   FULL WHEEL/G-HUB CAPTURE SEQUENCE
echo ==================================================================
echo.
echo Runs every no-game capture in sequence. Estimated time: 20-30 min.
echo Ctrl+C to exit; resume with individual commands.
echo.
echo Order:
echo   1. calibrate
echo   2. filter
echo   3. dpad
echo   4. commit
echo   5. persist-before + persist-after
echo   6. profile
echo   7. lightsync
echo.
echo Press ENTER to continue, or close window to abort.
pause >nul

call :run_calibrate
call :run_filter
call :run_dpad
call :run_commit
call :run_persist_before
call :run_persist_after
call :run_profile
call :run_lightsync

cls
echo ##################################################################
echo #                                                                #
echo #   WHEEL/G-HUB CAPTURE SEQUENCE COMPLETE                        #
echo #                                                                #
echo ##################################################################
echo.
dir /b "%CAPTURE_DIR%\%DATE%_*.pcapng" 2>nul
echo.
echo Please attach the %DATE%_*.pcapng files to a new GitHub issue
echo titled "Wheel capture contribution %DATE%".
echo All files are in this tools\ folder, next to the script.
echo.
pause
goto :eof

:: ============================================================
:run_calibrate
cls
echo ==================================================================
echo   Wheel calibration routine
echo ==================================================================
echo.
echo PURPOSE: Capture whatever commands G Hub sends when you run
echo          Calibrate. We want to know if the wheel exposes a
echo          dedicated calibration command or if G Hub emulates it
echo          in software.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub running
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "calibrate"
echo.
echo ================================================================
echo   [RECORDING] calibrate
echo ================================================================
echo.
echo NOW in G Hub:
echo   1. Navigate to the RS50 device settings
echo   2. Find the Calibrate option
echo      (Usually labelled "Calibrate" / "Calibrate Center" /
echo       "Wheel Alignment" in the wheel settings)
echo   3. Click Calibrate
echo   4. Follow any on-screen prompts (turn wheel, centre, etc.)
echo   5. Complete the calibration
echo   6. Wait 5 seconds after completion
echo.
echo If G Hub does NOT offer a Calibrate option, note that in the
echo issue comment - that's a useful data point on its own.
echo.
echo Press ENTER when calibration has completed...
pause >nul
call :stop_capture "calibrate"
goto :eof

:: ============================================================
:run_filter
cls
echo ==================================================================
echo   FFB filter slider + auto toggle
echo ==================================================================
echo.
echo PURPOSE: Capture the FFB filter setting commands as you move the
echo          slider through discrete values and toggle Auto on/off.
echo          Gives us clean samples we can decode byte-by-byte.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub open to the RS50 settings page
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "ffb_filter"
echo.
echo ================================================================
echo   [RECORDING] ffb_filter
echo ================================================================
echo.
echo NOW in G Hub:
echo   1. Find the FFB Filter setting
echo   2. If Auto is ON, turn it OFF first
echo   3. Set filter to 1
echo   4. Set filter to 5
echo   5. Set filter to 10
echo   6. Set filter to 15
echo   7. Turn Auto ON
echo   8. Turn Auto OFF
echo.
echo Wait ~2 seconds between each change so commands don't merge.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "ffb_filter"
goto :eof

:: ============================================================
:run_dpad
cls
echo ==================================================================
echo   D-pad directions
echo ==================================================================
echo.
echo PURPOSE: Confirm how the wheel encodes D-pad directions -
echo          whether it reports 4 cardinal directions only or all 8
echo          including diagonals.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - Wheel has a D-pad (usually on the rim)
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "dpad"
echo.
echo ================================================================
echo   [RECORDING] dpad
echo ================================================================
echo.
echo NOW press and HOLD (1-2 seconds each) in sequence:
echo   1. Up
echo   2. Up-Right
echo   3. Right
echo   4. Down-Right
echo   5. Down
echo   6. Down-Left
echo   7. Left
echo   8. Up-Left
echo   9. Release all
echo.
echo Wait 1 second between each direction so the report clearly changes.
echo If a diagonal feels like it's just "fighting" between two cardinals,
echo hold it firmly in the diagonal position and note that in the issue.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "dpad"
goto :eof

:: ============================================================
:run_commit
cls
echo ==================================================================
echo   Settings commit sequence
echo ==================================================================
echo.
echo PURPOSE: Capture a single settings change with surrounding quiet
echo          time. G Hub appears to send a commit/save command after
echo          each setting update; this isolates it so we can document
echo          the exact timing and bytes.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub open to RS50 settings
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "commit_0x8127"
echo.
echo ================================================================
echo   [RECORDING] commit_0x8127
echo ================================================================
echo.
echo NOW in G Hub:
echo   1. Change the FFB Strength slider by any amount
echo   2. Wait 3 seconds
echo   3. Change the Damping slider by any amount
echo   4. Wait 3 seconds
echo.
echo Two discrete settings changes separated by quiet time, so the
echo commit commands are easy to read in the capture.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "commit_0x8127"
goto :eof

:: ============================================================
:run_persist_before
cls
echo ==================================================================
echo   Settings persistence: before unplug (part 1 of 2)
echo ==================================================================
echo.
echo PURPOSE: Part 1 of a two-part test. Change Damping to a unique
echo          value and unplug the wheel. After that, run persist-after
echo          to check whether the value survived the power cycle.
echo          Tells us whether settings persist in the device or not.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub open to RS50 settings
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "persist_before"
echo.
echo ================================================================
echo   [RECORDING] persist_before
echo ================================================================
echo.
echo NOW in G Hub:
echo   1. Set Damping to a unique value (e.g. 47%%)
echo   2. Wait 3 seconds
echo   3. Unplug the RS50 USB cable
echo   4. Wait 5 seconds
echo.
echo Press ENTER when unplugged...
pause >nul
call :stop_capture "persist_before"
echo.
echo Next: plug the wheel back in, then run:
echo   windows_wheel_captures.bat persist-after
echo Press ENTER to continue...
pause >nul
goto :eof

:: ============================================================
:run_persist_after
cls
echo ==================================================================
echo   Settings persistence: after replug (part 2 of 2)
echo ==================================================================
echo.
echo PURPOSE: Part 2 of the persistence test. Replug the wheel and
echo          capture G Hub reading the settings back.
echo.
echo PREREQUISITES:
echo   - RS50 unplugged (from persist-before step)
echo   - G Hub running
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "persist_after"
echo.
echo ================================================================
echo   [RECORDING] persist_after
echo ================================================================
echo.
echo NOW:
echo   1. Plug the RS50 back in
echo   2. Wait for G Hub to fully re-initialise it
echo   3. Note the Damping value G Hub shows and include it in the
echo      issue comment (e.g. "Damping read back as 47%%" or
echo      "Damping read back as 10%% / default")
echo   4. Wait 5 seconds
echo.
echo Press ENTER when G Hub has read back the settings...
pause >nul
call :stop_capture "persist_after"
goto :eof

:: ============================================================
:run_profile
cls
echo ==================================================================
echo   Profile switch via hardware button
echo ==================================================================
echo.
echo PURPOSE: Some wheels have a profile/mode button on the rim that
echo          switches presets without going through G Hub. Capture
echo          the unsolicited notification the wheel sends when the
echo          user does that, so Linux can react to it.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub running
echo   - Wheel has a profile / mode button on the rim
echo     (consult the wheel manual for the exact button)
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "profile_button"
echo.
echo ================================================================
echo   [RECORDING] profile_button
echo ================================================================
echo.
echo NOW:
echo   1. Press and hold the profile / mode button on the wheel
echo      (typically a dedicated button near the center, or a combo)
echo   2. Release when the wheel LED changes color / pattern
echo   3. Wait 3 seconds
echo   4. Press it again to switch to a different profile
echo   5. Wait 3 seconds
echo   6. Switch back to the original profile
echo   7. Wait 3 seconds
echo.
echo If your wheel does NOT have a profile/mode button, skip this
echo capture and note that in the issue comment.
echo.
echo Press ENTER when done cycling profiles...
pause >nul
call :stop_capture "profile_button"
goto :eof

:: ============================================================
:run_lightsync
cls
echo ==================================================================
echo   LIGHTSYNC effect types
echo ==================================================================
echo.
echo PURPOSE: Configure a different LIGHTSYNC effect (solid, cycle,
echo          gradient, sweep, pulse) on each custom slot and capture
echo          the commands G Hub sends. Lets us decode which byte
echo          selects which effect.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub open to the LIGHTSYNC settings
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "lightsync_effects"
echo.
echo ================================================================
echo   [RECORDING] lightsync_effects
echo ================================================================
echo.
echo NOW in G Hub's LIGHTSYNC panel:
echo   1. Select CUSTOM 1. Set it to a SOLID color (any color).
echo      Click Apply / Save.
echo   2. Wait 2 seconds
echo   3. Select CUSTOM 2. Set it to a RAINBOW CYCLE or equivalent
echo      color-cycling effect. Apply / Save.
echo   4. Wait 2 seconds
echo   5. Select CUSTOM 3. Set it to a GRADIENT across the LEDs.
echo      Apply / Save.
echo   6. Wait 2 seconds
echo   7. Select CUSTOM 4. Set it to a SIDE-TO-SIDE SWEEP effect.
echo      Apply / Save.
echo   8. Wait 2 seconds
echo   9. Select CUSTOM 5. Set it to PULSE / BREATHE.
echo      Apply / Save.
echo.
echo If the Logitech UI doesn't expose all these, just use whatever
echo visibly distinct effects are available - the goal is to cover
echo different effect types, not exact names.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "lightsync_effects"
goto :eof

:: ============================================================
:: HELPER FUNCTIONS
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
    echo   Run this script as Administrator and check USBPcap installation.
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

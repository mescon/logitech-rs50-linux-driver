@echo off
setlocal enabledelayedexpansion

:: ============================================================
:: G Pro Compat Mode Range / Onboard Profile Captures
:: ============================================================
:: The RS50's G Pro compat firmware applies its own internal
:: "steering angle" clamp. In Linux we cannot change it via
:: HID++ feature 0x812F (returns EOPNOTSUPP), and the only known
:: way to change it is via the wheel's own OLED menu.
::
:: This battery captures any traffic GHUB generates that touches
:: the angle setting, so we can confirm whether GHUB knows a
:: command for it (and add the same command to our Linux driver)
:: or whether GHUB also relies on the OLED.
::
:: Run from an Administrator CMD prompt. Wireshark + USBPcap
:: must be installed. Wheel must be in G Pro compatibility mode
:: (PID c272) before starting.
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
    pause
    exit /b 1
)

for /f %%i in ('powershell -command "Get-Date -Format yyyy-MM-dd"') do set DATE=%%i

:: Mode tag (desktop / onboard) - included in every output filename
set MODE_TAG=
if /i "%~2"=="desktop" set MODE_TAG=desktop
if /i "%~2"=="onboard" set MODE_TAG=onboard

if /i "%~1"=="" goto :show_help
if /i "%~1"=="help" goto :show_help

:: Subcommands all need a mode tag
if "%MODE_TAG%"=="" goto :prompt_mode
goto :dispatch

:prompt_mode
echo.
echo You must specify which wheel mode you are in (so the capture
echo files are tagged correctly):
echo.
echo   windows_gpro_compat_range_capture.bat ^<command^> desktop
echo   windows_gpro_compat_range_capture.bat ^<command^> onboard
echo.
echo Switch the wheel to the mode you want to test via its OLED
echo menu first, then re-run with the matching tag.
echo.
pause
exit /b 1

:dispatch
if /i "%~1"=="all" goto :run_all
if /i "%~1"=="ghub-angle-slider" goto :run_ghub_angle_slider
if /i "%~1"=="onboard-profile-edit" goto :run_onboard_profile_edit
if /i "%~1"=="oled-angle-change" goto :run_oled_angle_change
if /i "%~1"=="ghub-strength-slider" goto :run_ghub_strength_slider
if /i "%~1"=="ghub-other-sliders" goto :run_ghub_other_sliders
if /i "%~1"=="ghub-damping-only" goto :run_ghub_damping_only
if /i "%~1"=="ghub-filter-only" goto :run_ghub_filter_only

echo Unknown command: %~1
echo.
goto :show_help

:show_help
cls
echo ==================================================================
echo   G Pro Compat Mode Range / Onboard Profile Captures
echo ==================================================================
echo.
echo Usage: windows_gpro_compat_range_capture.bat ^<command^> ^<mode^>
echo.
echo COMMAND:
echo   all                   Run all five captures in sequence
echo   ghub-angle-slider     If GHUB has any "wheel rotation" / angle
echo                         slider in the device settings, sweep it
echo                         through several values
echo   onboard-profile-edit  If GHUB has an "Onboard Memory" or
echo                         "Onboard Profile" editor, change the
echo                         steering angle in a profile and save
echo   oled-angle-change     Change the angle from the wheel's OLED
echo                         menu while a capture runs - confirms
echo                         whether the wheel sends an unsolicited
echo                         notification to the host on OLED change
echo   ghub-strength-slider  Sweep the FFB strength / overall force
echo                         slider through several values - tells us
echo                         which feature byte encodes strength so
echo                         the driver can wire wheel_strength_store
echo                         to it in compat mode
echo   ghub-other-sliders    Toggle damping / TRUEFORCE / brake force
echo                         / FFB filter sliders one at a time so we
echo                         can decode each independently
echo   ghub-damping-only     Sweep ONLY the damping slider, no other
echo                         GHUB control touched. Resolves whether
echo                         the index seen in our previous sweep is
echo                         really damping or aliases another setting.
echo   ghub-filter-only      Sweep ONLY the FFB filter slider. The
echo                         filter and mode-switch share feature 0x1a
echo                         in our captures; this isolates the filter
echo                         param byte unambiguously.
echo.
echo MODE (required - whatever the wheel is currently in via OLED):
echo   desktop               Wheel uses host-pushed live settings
echo   onboard               Wheel uses its own stored profile
echo.
echo The mode is embedded in every output filename so two passes
echo (one per mode) do not collide.
echo.
echo Recommended order:
echo   1) Switch wheel to desktop mode via OLED, then:
echo        windows_gpro_compat_range_capture.bat all desktop
echo   2) Switch wheel to onboard mode via OLED, then:
echo        windows_gpro_compat_range_capture.bat all onboard
echo.
echo PREREQUISITES:
echo   - Wheel in G Pro compatibility mode (PID c272 in Device Mgr)
echo   - Wireshark + USBPcap installed
echo   - GHUB running and showing the wheel
echo   - Capture interface USBPcap1 (edit at top of script if not)
echo.
pause
exit /b 0

:run_all
cls
echo ==================================================================
echo   ALL RANGE CAPTURES
echo ==================================================================
echo.
echo Sequence:
echo   1. ghub-angle-slider
echo   2. onboard-profile-edit
echo   3. oled-angle-change
echo   4. ghub-strength-slider
echo   5. ghub-other-sliders
echo   6. ghub-damping-only
echo   7. ghub-filter-only
echo.
echo If a step's GHUB control does not exist for this wheel, just
echo press ENTER through it (an empty pcap is itself useful data:
echo it confirms GHUB has no such control either).
echo.
pause
call :run_ghub_angle_slider
call :run_onboard_profile_edit
call :run_oled_angle_change
call :run_ghub_strength_slider
call :run_ghub_other_sliders
call :run_ghub_damping_only
call :run_ghub_filter_only
cls
echo ##################################################################
echo #  COMPAT-MODE RANGE CAPTURE SEQUENCE COMPLETE (%MODE_TAG%)       #
echo ##################################################################
echo.
dir /b "%CAPTURE_DIR%\%DATE%_compat_range_*_%MODE_TAG%.pcapng" 2>nul
echo.
echo If you have not yet captured the OTHER mode, switch the wheel
echo via OLED and re-run with the other tag.
pause
goto :eof

:run_ghub_angle_slider
cls
echo ==================================================================
echo   1/3: GHUB angle / wheel rotation slider
echo ==================================================================
echo.
echo PURPOSE: Some Logitech wheels expose a "Steering Angle" or
echo          "Wheel Rotation" or "Saturation" slider in GHUB's
echo          per-device settings. If yours does, sweeping it gives
echo          us the command bytes to set angle in compat mode.
echo.
echo PREPARATION:
echo   1. Open GHUB. Click the wheel device.
echo   2. Look in: Device Settings, Advanced, Sensitivity, or any
echo      tab that mentions wheel rotation, angle, range, or
echo      saturation.
echo   3. If you cannot find any such control, press ENTER and the
echo      empty pcap will document its absence.
echo   4. Press ENTER to start capture.
echo.
pause >nul
call :start_capture "compat_range_ghub_slider"
echo.
echo NOW in GHUB (skip if no slider exists):
echo   1. Set the angle slider to 90 degrees.   Wait 2 sec.
echo   2. Set it to 270 degrees.                Wait 2 sec.
echo   3. Set it to 540 degrees.                Wait 2 sec.
echo   4. Set it to 900 degrees.                Wait 2 sec.
echo   5. Set it to 1080 degrees.               Wait 2 sec.
echo.
echo (If GHUB only accepts certain steps, just hit each available
echo step value; the goal is several distinct angles so we can
echo decode the byte that encodes them.)
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "compat_range_ghub_slider"
goto :eof

:run_onboard_profile_edit
cls
echo ==================================================================
echo   2/3: GHUB onboard profile edit
echo ==================================================================
echo.
echo PURPOSE: G Pro Racing Wheels have onboard memory holding
echo          per-profile settings (steering angle, FFB, etc.). GHUB
echo          may have an "Onboard Memory" editor that writes profile
echo          fields to the wheel. Capture that.
echo.
echo PREPARATION:
echo   1. Open GHUB. Click the wheel device.
echo   2. Look for: Onboard Memory, Onboard Profile, Profile Editor,
echo      Save to Wheel, or similar wording.
echo   3. If found, open one of the onboard profiles for editing.
echo   4. If not found, press ENTER through to record the absence.
echo   5. Press ENTER to start capture.
echo.
pause >nul
call :start_capture "compat_range_onboard_profile"
echo.
echo NOW in GHUB (skip if no onboard editor exists):
echo   1. Set steering angle to 540 degrees. Wait 2 sec.
echo   2. Click Save / Apply to push to wheel. Wait 2 sec.
echo   3. Set steering angle to 1080 degrees. Wait 2 sec.
echo   4. Save / Apply again. Wait 2 sec.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "compat_range_onboard_profile"
goto :eof

:run_oled_angle_change
cls
echo ==================================================================
echo   3/3: OLED menu angle change
echo ==================================================================
echo.
echo PURPOSE: Confirm whether the wheel emits any notification on the
echo          USB bus when you change the steering angle from the
echo          OLED menu directly. If it does, our Linux driver could
echo          listen for it and update its cached value.
echo.
echo PREPARATION:
echo   1. GHUB running. Wheel connected.
echo   2. Press ENTER to start capture.
echo.
pause >nul
call :start_capture "compat_range_oled_change"
echo.
echo NOW on the wheel itself (NOT in GHUB):
echo   1. Open the OLED menu.
echo   2. Navigate to the steering angle setting.
echo   3. Change the angle to a different value.
echo   4. Confirm / save.
echo   5. Wait 5 seconds for any unsolicited notifications.
echo   6. Press ENTER below.
echo.
pause >nul
call :stop_capture "compat_range_oled_change"
goto :eof

:: ============================================================
:run_ghub_strength_slider
cls
echo ==================================================================
echo   4/5: GHUB FFB strength slider
echo ==================================================================
echo.
echo PURPOSE: Sweep GHUB's FFB strength / overall force / master gain
echo          slider through several discrete values. We have observed
echo          a candidate command (feature index 0x16 fn 2 with a
echo          16-bit param) but only as a side effect of the angle
echo          sweep; this isolates the strength control so we can
echo          confirm which command really sets it and add a compat-
echo          mode path to wheel_strength_store.
echo.
echo PREPARATION:
echo   1. GHUB running, wheel connected.
echo   2. Open the wheel's settings page in GHUB.
echo   3. Locate the FFB strength / overall force / master gain
echo      slider (the one expressed in percent or 0..100).
echo   4. Press ENTER to start capture.
echo.
pause >nul
call :start_capture "compat_range_strength_slider"
echo.
echo NOW in GHUB (skip if no slider exists):
echo   1. Set strength to   0%%. Wait 2 sec.
echo   2. Set strength to  25%%. Wait 2 sec.
echo   3. Set strength to  50%%. Wait 2 sec.
echo   4. Set strength to  75%%. Wait 2 sec.
echo   5. Set strength to 100%%. Wait 2 sec.
echo.
echo Wait between steps so each command is isolated in the trace.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "compat_range_strength_slider"
goto :eof

:: ============================================================
:run_ghub_other_sliders
cls
echo ==================================================================
echo   5/5: GHUB other sliders (damping, TRUEFORCE, brake, filter)
echo ==================================================================
echo.
echo PURPOSE: Cover the remaining wheel-config sliders that are
echo          inert in compat mode on Linux today. Toggling each
echo          one separately gives us a per-slider command we can
echo          add to the driver later.
echo.
echo PREPARATION:
echo   1. GHUB running, wheel connected, settings page open.
echo   2. Press ENTER to start capture.
echo.
pause >nul
call :start_capture "compat_range_other_sliders"
echo.
echo NOW in GHUB (skip any slider that does not exist):
echo   1. DAMPING: move from 0%% -^> 50%% -^> 100%%. Wait 2 sec each.
echo   2. TRUEFORCE strength: same sweep.
echo   3. BRAKE FORCE: same sweep.
echo   4. FFB FILTER: try a couple of distinct values (0, 5, 10).
echo   5. Pause 5 sec at the end for any commit traffic.
echo.
echo If GHUB has more wheel-config sliders we did not list,
echo toggle them too - the more captures the better.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "compat_range_other_sliders"
goto :eof

:: ============================================================
:run_ghub_damping_only
cls
echo ==================================================================
echo   6/7: GHUB damping slider, ISOLATED
echo ==================================================================
echo.
echo PURPOSE: Our previous "other sliders" capture suggested damping
echo          uses feature index 0x15 fn 2. The audit's speculative
echo          feature-ID table claimed the same index aliases the
echo          steering range feature (0x8138). This capture isolates
echo          ONLY the damping slider so we can verify the command
echo          unambiguously and add a wheel_damping compat path.
echo.
echo PREPARATION:
echo   1. GHUB running, wheel connected, settings page open.
echo   2. Press ENTER to start capture.
echo   3. DO NOT touch any slider other than the damping one. Even
echo      a small mouse-over on another control may emit traffic.
echo.
pause >nul
call :start_capture "compat_range_damping_only"
echo.
echo NOW in GHUB - touch ONLY the damping slider:
echo   1. Set damping to   0%%.  Wait 3 sec.
echo   2. Set damping to  20%%.  Wait 3 sec.
echo   3. Set damping to  50%%.  Wait 3 sec.
echo   4. Set damping to  80%%.  Wait 3 sec.
echo   5. Set damping to 100%%.  Wait 3 sec.
echo.
echo Wider value spread + longer pauses make the SET commands easy
echo to isolate from background traffic.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "compat_range_damping_only"
goto :eof

:: ============================================================
:run_ghub_filter_only
cls
echo ==================================================================
echo   7/7: GHUB FFB filter slider, ISOLATED
echo ==================================================================
echo.
echo PURPOSE: Our captures suggest the FFB filter shares feature
echo          0x1a with the mode-switch command. This makes the byte
echo          encoding ambiguous. Sweeping ONLY the filter slider
echo          (no mode switches, no other sliders) lets us see the
echo          unambiguous filter byte values.
echo.
echo PREPARATION:
echo   1. GHUB running, wheel connected, settings page open.
echo   2. The wheel must be in DESKTOP mode (not onboard) so live
echo      filter SETs from GHUB take effect.
echo   3. Make sure "Auto FFB filter" is OFF before starting (if on,
echo      GHUB may push values continuously and pollute the trace).
echo   4. Press ENTER to start capture.
echo.
pause >nul
call :start_capture "compat_range_filter_only"
echo.
echo NOW in GHUB - touch ONLY the FFB filter slider:
echo   1. Set filter to  0. Wait 3 sec.
echo   2. Set filter to  3. Wait 3 sec.
echo   3. Set filter to  7. Wait 3 sec.
echo   4. Set filter to 10. Wait 3 sec.
echo   5. Set filter to 15. Wait 3 sec.
echo.
echo Each set should produce one or two short HID++ writes. We will
echo decode the filter byte from those.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "compat_range_filter_only"
goto :eof

:: ============================================================
:start_capture
set CURRENT_CAPTURE=%~1
set OUTPUT_FILE=%CAPTURE_DIR%\%DATE%_%CURRENT_CAPTURE%_%MODE_TAG%.pcapng
echo.
echo   Starting capture: %CURRENT_CAPTURE% (%MODE_TAG% mode)
echo   Output: %OUTPUT_FILE%
echo.
start "" /b "%TSHARK%" -i "%INTERFACE%" -w "%OUTPUT_FILE%" -q
timeout /t 2 /nobreak >nul
tasklist /fi "imagename eq tshark.exe" 2>nul | findstr /i "tshark.exe" >nul
if errorlevel 1 (
    echo   ERROR: tshark failed to start. Run as Administrator.
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
echo   Saved: %DATE%_%~1_%MODE_TAG%.pcapng
if exist "%CAPTURE_DIR%\%DATE%_%~1_%MODE_TAG%.pcapng" (
    for %%A in ("%CAPTURE_DIR%\%DATE%_%~1_%MODE_TAG%.pcapng") do echo   Size: %%~zA bytes
)
echo.
timeout /t 2 /nobreak >nul
goto :eof

@echo off
setlocal enabledelayedexpansion

:: ============================================================
:: Windows Reverse-Engineering Captures (Phase C preparation)
:: ============================================================
:: Runs on bare-metal Windows with the RS50 attached and G Hub
:: installed. Each capture targets a specific open question from
:: docs/plans/2026-04-16-windows-gap-analysis.md.
::
:: Filename convention: <DATE>_re_<name>.pcapng
:: ============================================================

:: Configuration
set TSHARK=C:\Program Files\Wireshark\tshark.exe
set CAPTURE_DIR=P:\logitech-rs50-linux-driver\dev\captures
set INTERFACE=\\.\USBPcap1

:: Verify tshark
if not exist "%TSHARK%" (
    echo ERROR: tshark not found at %TSHARK%
    echo Install Wireshark with tshark + USBPcap components.
    pause
    exit /b 1
)

:: Verify capture dir
if not exist "%CAPTURE_DIR%" (
    echo ERROR: Capture dir not found: %CAPTURE_DIR%
    echo Map the repo drive or edit CAPTURE_DIR in this file.
    pause
    exit /b 1
)

:: Date prefix from PowerShell
for /f %%i in ('powershell -command "Get-Date -Format yyyy-MM-dd"') do set DATE=%%i

:: Menu dispatch
if /i "%~1"=="" goto :show_help
if /i "%~1"=="help" goto :show_help
if /i "%~1"=="all" goto :run_all

:: Trueforce
if /i "%~1"=="tf-beamng" goto :run_tf_beamng
if /i "%~1"=="tf-ace" goto :run_tf_ace
if /i "%~1"=="tf-abort" goto :run_tf_abort

:: Calibration
if /i "%~1"=="calibrate" goto :run_calibrate

:: Protocol disambiguation
if /i "%~1"=="filter-fn" goto :run_filter_fn
if /i "%~1"=="dpad-8way" goto :run_dpad_8way
if /i "%~1"=="commit-0x8127" goto :run_commit_8127

:: Persistence
if /i "%~1"=="persist-unplug" goto :run_persist_unplug
if /i "%~1"=="persist-readback" goto :run_persist_readback

:: Broadcasts
if /i "%~1"=="profile-hw" goto :run_profile_hw

:: LIGHTSYNC
if /i "%~1"=="ls-types" goto :run_lightsync_types

:: DLL survey (ProcMon, not tshark)
if /i "%~1"=="dll-survey" goto :run_dll_survey

echo Unknown command: %~1
echo.
goto :show_help

:: ============================================================
:show_help
cls
echo ==================================================================
echo   RS50 Windows Reverse-Engineering Captures
echo ==================================================================
echo.
echo Usage: windows_re.bat ^<command^>
echo.
echo   all                Run the full sequence (long, ~60-90 min)
echo.
echo Trueforce RE (highest priority):
echo   dll-survey         Identify Logitech DLLs loaded by games (ProcMon)
echo   tf-beamng          BeamNG.drive cold start to on-track (with TF)
echo   tf-ace              Assetto Corsa / AC Evo with TF (second game)
echo   tf-abort           Gameplay with ALT-F4 abort (graceful stop test)
echo.
echo Issue #13 calibration:
echo   calibrate          G Hub calibration routine
echo.
echo Protocol disambiguation:
echo   filter-fn          FFB filter SET function byte (fn2 vs fn3)
echo   dpad-8way          D-pad 8-way directions
echo   commit-0x8127      Feature 0x8127 commit sequence
echo.
echo Persistence tests:
echo   persist-unplug     Set damping=50%%, unplug wheel (with capture)
echo   persist-readback   Replug wheel, read back damping (continuation)
echo.
echo Broadcasts:
echo   profile-hw         Switch profile via hardware button (not G Hub)
echo.
echo LIGHTSYNC:
echo   ls-types           Each slot effect type captured separately
echo.
echo Output: %CAPTURE_DIR%\%%DATE%%_re_^<name^>.pcapng
echo.
pause
exit /b 0

:: ============================================================
:run_all
cls
echo ==================================================================
echo   FULL RE CAPTURE SEQUENCE
echo ==================================================================
echo.
echo This will run every RE capture in order. Estimated time: 60-90 min.
echo You can exit anytime (Ctrl+C) and resume with individual commands.
echo.
echo Recommended order:
echo   1. dll-survey       (before first game launch)
echo   2. tf-beamng
echo   3. tf-ace
echo   4. tf-abort
echo   5. calibrate
echo   6. filter-fn
echo   7. dpad-8way
echo   8. commit-0x8127
echo   9. persist-unplug + persist-readback
echo   10. profile-hw
echo   11. ls-types
echo.
echo Press ENTER to continue, or close window to abort.
pause >nul

call :run_dll_survey
call :run_tf_beamng
call :run_tf_ace
call :run_tf_abort
call :run_calibrate
call :run_filter_fn
call :run_dpad_8way
call :run_commit_8127
call :run_persist_unplug
call :run_persist_readback
call :run_profile_hw
call :run_lightsync_types

cls
echo ##################################################################
echo #                                                                #
echo #   FULL RE SEQUENCE COMPLETE                                    #
echo #                                                                #
echo ##################################################################
echo.
dir /b "%CAPTURE_DIR%\%DATE%_re_*.pcapng"
echo.
echo Return to Linux and run: git add dev/captures ^&^& git commit
echo Then share the %DATE%_re_*.pcapng files with Claude for analysis.
echo.
pause
goto :eof

:: ============================================================
:run_dll_survey
cls
echo ==================================================================
echo   DLL SURVEY (ProcMon, not tshark)
echo ==================================================================
echo.
echo PURPOSE: Identify which Logitech DLLs games load for FFB/TF.
echo.
echo This capture is NOT tshark. It uses Sysinternals Process Monitor
echo (ProcMon) to record file and registry activity during a game launch.
echo.
echo SETUP:
echo   1. Download ProcMon if not installed:
echo        https://learn.microsoft.com/sysinternals/downloads/procmon
echo   2. Launch ProcMon as Administrator
echo   3. Filter ^> Filter...
echo        Process Name contains BeamNG.drive    -^> Include
echo        Process Name contains AssettoCorsa    -^> Include
echo        Path contains Logi                    -^> Include
echo      (Add a line per game you plan to test.)
echo   4. Enable capture (the magnifier button)
echo.
echo CAPTURE:
echo   1. Start ProcMon capture (clear existing if any: File ^> Clear)
echo   2. Launch BeamNG.drive. Wait until main menu is visible.
echo   3. Close BeamNG.
echo   4. Stop ProcMon capture.
echo   5. File ^> Save ^> "All events"
echo      Save as: %CAPTURE_DIR%\%DATE%_re_dll_survey_beamng.csv
echo   6. Clear, repeat for AC Evo / AC classic if available.
echo      Save as: %CAPTURE_DIR%\%DATE%_re_dll_survey_ace.csv
echo.
echo ADDITIONAL: list the Logitech files on disk:
echo   powershell -c "Get-ChildItem -Path 'C:\Program Files\LGHUB','C:\Program Files (x86)\Logitech' -Filter '*.dll' -Recurse -ErrorAction SilentlyContinue ^| Select-Object FullName,Length,LastWriteTime ^| Export-Csv -NoType '%CAPTURE_DIR%\%DATE%_re_logi_dlls.csv'"
echo.
echo Press ENTER when you have completed the ProcMon captures...
pause >nul

:: Run the powershell DLL inventory automatically
echo.
echo Inventorying Logitech DLLs on disk...
powershell -c "Get-ChildItem -Path 'C:\Program Files\LGHUB','C:\Program Files (x86)\Logitech','C:\Program Files\Logitech' -Filter '*.dll' -Recurse -ErrorAction SilentlyContinue | Select-Object FullName,Length,LastWriteTime | Export-Csv -NoType '%CAPTURE_DIR%\%DATE%_re_logi_dlls.csv'"
echo Saved: %DATE%_re_logi_dlls.csv
echo.
pause
goto :eof

:: ============================================================
:run_tf_beamng
cls
echo ==================================================================
echo   Trueforce: BeamNG.drive full session
echo ==================================================================
echo.
echo PURPOSE: Capture the complete Trueforce timeline in BeamNG.drive.
echo          Cold start -^> menus -^> on track -^> in-game exit.
echo.
echo This re-verifies our analysis of the issue #5 capture with a known
echo start/end timeline. Allows us to check:
echo   - Whether the init sequence is sent once or truly twice
echo   - Whether the 48 init parameters differ per game (vs AC)
echo   - Whether there is any explicit stop sequence on game exit
echo   - Whether the G SDK DLL is loaded during cold start
echo.
echo PREREQUISITES:
echo   - RS50 plugged in and initialized
echo   - G Hub running and showing the wheel
echo   - BeamNG installed
echo   - "Use Logitech Features" enabled in BeamNG FFB settings
echo   - BeamNG not yet running
echo.
echo Press ENTER when ready to start capture...
pause >nul
call :start_capture "re_tf_beamng"
echo.
echo ================================================================
echo   [RECORDING] re_tf_beamng
echo ================================================================
echo.
echo NOW:
echo   1. Launch BeamNG.drive
echo   2. Wait until the main menu loads
echo   3. Start a Free Roam drive with any car
echo   4. Drive on track for 30-60 seconds
echo      - Accelerate, brake, turn both directions
echo      - Hit a curb or gentle obstacle if available
echo   5. Return to main menu (in-game exit, not ALT-F4)
echo   6. Quit BeamNG cleanly (File ^> Exit, or the main-menu Quit)
echo.
echo Press ENTER when BeamNG is FULLY CLOSED...
pause >nul
call :stop_capture "re_tf_beamng"
goto :eof

:: ============================================================
:run_tf_ace
cls
echo ==================================================================
echo   Trueforce: Assetto Corsa / AC Evo full session
echo ==================================================================
echo.
echo PURPOSE: Capture Trueforce in a second game to verify whether
echo          the 48 init parameters are game-independent.
echo.
echo If G Hub init parameters differ between BeamNG and AC, they are
echo game-tunable. If identical, they are device-hardcoded (matches our
echo current analysis) and we can hardcode them in the Linux driver.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - Either Assetto Corsa Competizione, AC Evo, or AC classic
echo   - Trueforce enabled in game settings
echo   - Wheel calibrated in the game
echo   - Game not yet running
echo.
echo Press ENTER to start capture (then launch the game)...
pause >nul
call :start_capture "re_tf_ace"
echo.
echo ================================================================
echo   [RECORDING] re_tf_ace
echo ================================================================
echo.
echo NOW:
echo   1. Launch the game
echo   2. Wait until the main menu loads
echo   3. Start a Practice or Free Drive session
echo   4. Drive for 30-60 seconds
echo   5. Return to main menu cleanly
echo   6. Quit the game normally
echo.
echo Press ENTER when the game is FULLY CLOSED...
pause >nul
call :stop_capture "re_tf_ace"
goto :eof

:: ============================================================
:run_tf_abort
cls
echo ==================================================================
echo   Trueforce: ALT-F4 abort mid-stream
echo ==================================================================
echo.
echo PURPOSE: Capture what happens on ep 0x03 when a TF-streaming game
echo          is killed abruptly (no graceful shutdown).
echo.
echo We already know that in the BeamNG issue #5 capture the file ends
echo mid-stream with no type-0x04 stop. This capture confirms that with
echo a known-forced termination. Tells us whether the device recovers
echo cleanly or holds force.
echo.
echo PREREQUISITES:
echo   - Same as tf-beamng. Any TF-enabled game works.
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "re_tf_abort"
echo.
echo ================================================================
echo   [RECORDING] re_tf_abort
echo ================================================================
echo.
echo NOW:
echo   1. Launch BeamNG (or another TF-enabled game)
echo   2. Start a drive session
echo   3. Drive for 15-30 seconds
echo   4. Press ALT-F4 (or kill the process in Task Manager)
echo   5. Wait 10 seconds for any aftermath on ep 0x03
echo.
echo Press ENTER when the game is killed and 10s have passed...
pause >nul
call :stop_capture "re_tf_abort"
goto :eof

:: ============================================================
:run_calibrate
cls
echo ==================================================================
echo   Wheel calibration routine (Issue #13)
echo ==================================================================
echo.
echo PURPOSE: Find the HID++ command sequence G Hub sends when the
echo          user runs Calibrate.
echo.
echo None of our existing captures exercise the Calibrate function.
echo This is the only way to learn whether RS50 exposes a dedicated
echo calibration HID++ feature. If yes, issue #13 becomes easy.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub running
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "re_calibrate"
echo.
echo ================================================================
echo   [RECORDING] re_calibrate
echo ================================================================
echo.
echo NOW in G Hub:
echo   1. Navigate to the RS50 device settings
echo   2. Find the Calibrate option
echo      (Usually under "Calibrate" / "Calibrate Center" /
echo       "Wheel Alignment" in wheel settings)
echo   3. Click Calibrate
echo   4. Follow any on-screen prompts (turn wheel, center, etc.)
echo   5. Complete the calibration
echo   6. Wait 5 seconds after completion
echo.
echo If G Hub does NOT offer a Calibrate button, just quit and note that
echo in the capture naming.
echo.
echo Press ENTER when calibration has completed...
pause >nul
call :stop_capture "re_calibrate"
goto :eof

:: ============================================================
:run_filter_fn
cls
echo ==================================================================
echo   FFB filter SET function byte (fn2 vs fn3)
echo ==================================================================
echo.
echo PURPOSE: Resolve the contradiction between our Phase B analysis
echo          (which read fn2 from captures) and an earlier TODO fix
echo          (which set fn3 on both RS50 and G Pro).
echo.
echo Move the FFB filter slider through at least 4 positions:
echo   1 (min), 5, 10, 15 (max).
echo Then toggle auto on and off twice.
echo.
echo We will decode the exact fn byte from this capture.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub open to RS50 settings page
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "re_filter_fn"
echo.
echo ================================================================
echo   [RECORDING] re_filter_fn
echo ================================================================
echo.
echo NOW in G Hub:
echo   1. Find FFB Filter setting
echo   2. If Auto is ON, turn it OFF first
echo   3. Set filter to 1
echo   4. Set filter to 5
echo   5. Set filter to 10
echo   6. Set filter to 15
echo   7. Turn Auto ON
echo   8. Turn Auto OFF
echo.
echo Wait 2 seconds between each change so commands don't merge.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "re_filter_fn"
goto :eof

:: ============================================================
:run_dpad_8way
cls
echo ==================================================================
echo   D-pad: 8-way vs 4-way directions
echo ==================================================================
echo.
echo PURPOSE: Resolve the contradiction in our protocol spec.
echo          The code decodes 8 directions (with diagonals), the spec
echo          Section 3 lists only 4. Capture actual device output.
echo.
echo Hold each of 8 D-pad directions deliberately and check the raw
echo reports on interface 0 (endpoint 0x81). Specifically the 3-bit
echo direction field in byte[0].
echo.
echo PREREQUISITES:
echo   - RS50 plugged in with a D-pad (usually on the wheel rim)
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "re_dpad_8way"
echo.
echo ================================================================
echo   [RECORDING] re_dpad_8way
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
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "re_dpad_8way"
goto :eof

:: ============================================================
:run_commit_8127
cls
echo ==================================================================
echo   Feature 0x8127 commit sequence
echo ==================================================================
echo.
echo PURPOSE: G Hub sends a "11 FF 11 2D <16 zeros>" command after
echo          every settings change. We have never sent this. Capture
echo          a single settings change to document the exact sequence
echo          including 0x8127 before/after timing.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub open to RS50 settings
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "re_commit_8127"
echo.
echo ================================================================
echo   [RECORDING] re_commit_8127
echo ================================================================
echo.
echo NOW in G Hub:
echo   1. Change FFB Strength slider by any amount
echo   2. Wait 3 seconds
echo   3. Change Damping slider by any amount
echo   4. Wait 3 seconds
echo.
echo This gives two discrete settings changes separated by quiet time
echo so the 0x8127 timing is easy to read.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "re_commit_8127"
goto :eof

:: ============================================================
:run_persist_unplug
cls
echo ==================================================================
echo   Persistence test: setting change then unplug
echo ==================================================================
echo.
echo PURPOSE: Half 1 of the persistence test. Set damping to 50%%, then
echo          unplug the wheel. Then run persist-readback after replug.
echo.
echo If damping is 50%% after replug, settings ARE persisted. If reverts
echo to default or 10%%, our omission of 0x8127 may matter for persistence.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub open to RS50 settings
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "re_persist_unplug"
echo.
echo ================================================================
echo   [RECORDING] re_persist_unplug
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
call :stop_capture "re_persist_unplug"
echo.
echo Next: run "windows_re.bat persist-readback" after replug.
echo Press ENTER to continue...
pause >nul
goto :eof

:: ============================================================
:run_persist_readback
cls
echo ==================================================================
echo   Persistence test: replug and readback
echo ==================================================================
echo.
echo PURPOSE: Half 2 of the persistence test. Replug the wheel and
echo          check whether damping persisted.
echo.
echo PREREQUISITES:
echo   - RS50 unplugged (from persist-unplug step)
echo   - G Hub running
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "re_persist_readback"
echo.
echo ================================================================
echo   [RECORDING] re_persist_readback
echo ================================================================
echo.
echo NOW:
echo   1. Plug RS50 back in
echo   2. Wait for G Hub to fully re-initialize it
echo   3. Note the Damping value G Hub shows
echo      (Record it here: ________________________)
echo   4. Wait 5 seconds
echo.
echo Press ENTER when G Hub has read back the settings...
pause >nul
call :stop_capture "re_persist_readback"
goto :eof

:: ============================================================
:run_profile_hw
cls
echo ==================================================================
echo   Profile switch via hardware button (device broadcast)
echo ==================================================================
echo.
echo PURPOSE: Capture the unsolicited notification the device sends when
echo          the user switches profiles via a wheel button (not G Hub).
echo          Format: 11 02 0E 10 [new_idx] 01 00 00...
echo.
echo Our Linux driver doesn't listen for these and gets stale state.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub running
echo   - Wheel has a profile / mode button on the rim
echo     (consult wheel manual for the exact button)
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "re_profile_hw"
echo.
echo ================================================================
echo   [RECORDING] re_profile_hw
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
echo Press ENTER when done cycling profiles...
pause >nul
call :stop_capture "re_profile_hw"
goto :eof

:: ============================================================
:run_lightsync_types
cls
echo ==================================================================
echo   LIGHTSYNC slot type byte
echo ==================================================================
echo.
echo PURPOSE: Decode what the LIGHTSYNC slot type byte (params[1] in
echo          SET_CONFIG) means. Our analysis saw 0x01-0x04 per-slot
echo          but the driver hardcodes 0x03.
echo.
echo Create a distinct LIGHTSYNC configuration per slot (static, rainbow
echo cycle, rainbow slide, etc.) and capture SET_CONFIG for each.
echo.
echo PREREQUISITES:
echo   - RS50 plugged in
echo   - G Hub open to LIGHTSYNC settings
echo.
echo Press ENTER to start capture...
pause >nul
call :start_capture "re_lightsync_types"
echo.
echo ================================================================
echo   [RECORDING] re_lightsync_types
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
echo If the Logitech UI doesn't expose all these, do whatever visibly
echo distinct effects you can. The goal is DISTINCT effect types.
echo.
echo Press ENTER when done...
pause >nul
call :stop_capture "re_lightsync_types"
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

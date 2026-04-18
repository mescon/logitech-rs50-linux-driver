@echo off
setlocal enabledelayedexpansion

:: ============================================================
:: Logitech RS50 Trueforce Windows Captures (community edition)
:: ============================================================
:: This is the game-requiring subset of the RE capture battery.
:: We are looking for contributors who own a TF-enabled game
:: (BeamNG.drive, Assetto Corsa / AC Evo / ACC) AND a Logitech
:: RS50 wheel on Windows, to run these captures and share the
:: resulting pcapng files via a GitHub issue or PR.
::
:: Each capture targets a specific open question about how
:: G Hub drives the Trueforce audio-haptic channel during real
:: gameplay. The non-game captures (settings, calibration,
:: profile switches, etc.) are handled separately by the project
:: maintainers and are not part of this script.
::
:: Output files land next to this script, named so each one is
:: self-describing when attached to the issue tracker. Example:
::   2026-04-18_trueforce_beamng.pcapng
::   2026-04-18_trueforce_ace.pcapng
::   2026-04-18_trueforce_abort.pcapng
::   2026-04-18_dll_survey_beamng.csv
::   2026-04-18_logitech_dlls.csv
::
:: See docs/WINDOWS_RE_CAPTURE_GUIDE.md for full context.
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

if /i "%~1"=="dll-survey" goto :run_dll_survey
if /i "%~1"=="tf-beamng" goto :run_tf_beamng
if /i "%~1"=="tf-ace" goto :run_tf_ace
if /i "%~1"=="tf-abort" goto :run_tf_abort

echo Unknown command: %~1
echo.
goto :show_help

:: ============================================================
:show_help
cls
echo ==================================================================
echo   RS50 Trueforce Windows Captures (community edition)
echo ==================================================================
echo.
echo Usage: windows_tf_captures.bat ^<command^>
echo.
echo   all             Run all four captures in sequence (~30-45 min)
echo   dll-survey      Identify Logitech DLLs loaded by games (ProcMon)
echo   tf-beamng       BeamNG.drive cold start to on-track (with TF)
echo   tf-ace          Assetto Corsa / AC Evo / ACC with TF (second game)
echo   tf-abort        Gameplay with ALT-F4 abort (graceful stop test)
echo.
echo Prerequisites:
echo   - Windows with Wireshark + USBPcap installed
echo   - Run this script from an Administrator CMD prompt
echo   - RS50 wheel attached and recognized by G Hub
echo   - At least one TF-enabled game (BeamNG and/or AC family)
echo   - Sysinternals ProcMon for the dll-survey capture:
echo       https://learn.microsoft.com/sysinternals/downloads/procmon
echo.
echo Output: %CAPTURE_DIR%\%%DATE%%_trueforce_^<name^>.pcapng
echo         (files land next to this script in the tools\ folder)
echo.
echo After capture: attach the pcapng (and ProcMon CSV for dll-survey)
echo to a GitHub issue on the logitech-rs50-linux-driver repo.
echo.
pause
exit /b 0

:: ============================================================
:run_all
cls
echo ==================================================================
echo   FULL TF CAPTURE SEQUENCE
echo ==================================================================
echo.
echo This runs all four game-requiring captures in order.
echo Estimated time: 30-45 minutes.
echo You can exit anytime (Ctrl+C) and resume with individual commands.
echo.
echo Order:
echo   1. dll-survey       (before first game launch)
echo   2. tf-beamng
echo   3. tf-ace
echo   4. tf-abort
echo.
echo Press ENTER to continue, or close window to abort.
pause >nul

call :run_dll_survey
call :run_tf_beamng
call :run_tf_ace
call :run_tf_abort

cls
echo ##################################################################
echo #                                                                #
echo #   TF CAPTURE SEQUENCE COMPLETE                                 #
echo #                                                                #
echo ##################################################################
echo.
dir /b "%CAPTURE_DIR%\%DATE%_trueforce_*.pcapng" 2>nul
dir /b "%CAPTURE_DIR%\%DATE%_dll_survey_*.csv" 2>nul
dir /b "%CAPTURE_DIR%\%DATE%_logitech_dlls.csv" 2>nul
echo.
echo Please attach the %DATE%_trueforce_*.pcapng files (and the
echo %DATE%_dll_survey_*.csv + %DATE%_logitech_dlls.csv from
echo dll-survey) to a new GitHub issue titled
echo "Trueforce capture contribution %DATE%".
echo All files are in this tools\ folder, next to the script.
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
echo      Save as: %CAPTURE_DIR%\%DATE%_dll_survey_beamng.csv
echo   6. Clear, repeat for AC Evo / AC classic if available.
echo      Save as: %CAPTURE_DIR%\%DATE%_dll_survey_ace.csv
echo.
echo This script will ALSO run a PowerShell inventory of Logitech DLLs
echo installed on disk, saving the output as
echo   %CAPTURE_DIR%\%DATE%_logitech_dlls.csv
echo.
echo Press ENTER when you have completed the ProcMon captures...
pause >nul

:: Run the powershell DLL inventory automatically
echo.
echo Inventorying Logitech DLLs on disk...
powershell -c "Get-ChildItem -Path 'C:\Program Files\LGHUB','C:\Program Files (x86)\Logitech','C:\Program Files\Logitech' -Filter '*.dll' -Recurse -ErrorAction SilentlyContinue | Select-Object FullName,Length,LastWriteTime | Export-Csv -NoType '%CAPTURE_DIR%\%DATE%_logitech_dlls.csv'"
echo Saved: %DATE%_logitech_dlls.csv
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
call :start_capture "trueforce_beamng"
echo.
echo ================================================================
echo   [RECORDING] trueforce_beamng
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
call :stop_capture "trueforce_beamng"
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
call :start_capture "trueforce_ace"
echo.
echo ================================================================
echo   [RECORDING] trueforce_ace
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
call :stop_capture "trueforce_ace"
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
call :start_capture "trueforce_abort"
echo.
echo ================================================================
echo   [RECORDING] trueforce_abort
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
call :stop_capture "trueforce_abort"
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

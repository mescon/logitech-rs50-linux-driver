# USB Capture Guide for Logitech Wheel Protocol Analysis

This guide explains how to capture USB traffic between G Hub and your Logitech wheel on Windows. These captures help us reverse-engineer FFB protocols for Linux driver development.

**Target devices:** Logitech wheels beyond the RS50 (`046d:c276`) and G PRO Racing Wheel (`046d:c272` Xbox/PC, `046d:c268` PS/PC), both of which are already supported. New targets typically include any Logitech direct-drive or belt-drive racing wheel that exposes a Windows G Hub entry.

For the two supported wheels we already have a ready-to-run capture battery at:

- `tools/windows_tf_captures.bat` (TRUEFORCE / game-session captures)
- `tools/windows_wheel_captures.bat` (settings / input captures, no game required)

If you're submitting captures for an already-supported wheel, prefer those scripts: they produce deterministically-named files that our analysis scripts already expect.

## What We Need

To add FFB support for a new wheel, we need captures showing:

1. **Device initialization** - What happens when G Hub starts
2. **FFB during gameplay** - The actual force feedback commands
3. **Settings changes** - Rotation range, FFB strength, damping, etc.

## Requirements

- Windows 10/11 with Logitech G Hub installed
- Wireshark with USBPcap (https://www.wireshark.org/)
- Your Logitech wheel connected via USB
- A game with force feedback (any racing game works)

## Installation

1. Download Wireshark from https://www.wireshark.org/download.html
2. During installation, check the box to install **USBPcap**
3. Reboot after installation

## Finding Your Device

1. Open Device Manager (Win+X -> Device Manager)
2. Expand "Human Interface Devices"
3. Find your wheel (e.g., "Logitech G Pro Racing Wheel")
4. Note which USB root hub it's connected to

Or use USBPcap's command line:
```
USBPcapCMD.exe
```
This lists all USB devices and their bus numbers.

## Capture Procedure

### Capture 1: G Hub Initialization

This shows feature discovery and initial device setup.

1. Close G Hub completely (system tray -> right-click -> Quit)
2. Start Wireshark
3. Select the USBPcap interface for your wheel's USB bus
4. Start capture
5. Launch G Hub
6. Wait for G Hub to fully connect to the wheel (settings page loads)
7. Stop capture
8. Save as: `ghub_init_[device].pcapng`

### Capture 2: FFB Gameplay

This shows the actual force feedback protocol.

1. Start Wireshark capture
2. Launch a racing game with FFB support
3. Play for 30-60 seconds with varied inputs:
   - Drive straight
   - Turn left and right
   - Hit curbs/rumble strips
   - Collide with something
4. Stop capture
5. Save as: `ffb_gameplay_[device].pcapng`

### Capture 3: Settings Sweep

This shows the HID++ commands for each setting.

For each setting below, do one capture:
1. Start capture
2. In G Hub, slowly drag the slider from min to max
3. Stop capture
4. Save with descriptive name

**Settings to capture:**
- `rotation_range_[device].pcapng` - Rotation range (e.g., 360 -> 900 -> 1080)
- `ffb_strength_[device].pcapng` - FFB strength slider
- `damping_[device].pcapng` - Damping slider (if present)
- `trueforce_[device].pcapng` - TRUEFORCE slider (if present)

### Capture 4: Cold Boot (Optional but Valuable)

Shows what the wheel does before G Hub loads.

1. Unplug wheel
2. Close G Hub
3. Start Wireshark capture
4. Plug in wheel
5. Wait 10 seconds (capture raw USB enumeration)
6. Start G Hub
7. Wait for connection
8. Stop capture
9. Save as: `cold_boot_[device].pcapng`

## Wireshark Filter Tips

To isolate your wheel's traffic:
```
usb.idVendor == 0x046d && usb.idProduct == 0xc272
```
(Replace `0xc272` with your wheel's product ID)

To see only outgoing data (host to device):
```
usb.dst == "X.Y.Z"
```
(Replace with your device address from the capture)

## What Makes a Good Capture

- **Start clean**: Close G Hub before starting capture, then launch it
- **Isolate actions**: One setting change per capture file
- **Label clearly**: Include device name and action in filename
- **Include context**: Note what game you used, what actions you performed

## Submitting Captures

1. Create a GitHub issue at: https://github.com/mescon/logitech-rs50-linux-driver/issues
2. Title: "USB captures for [Device Name]"
3. Attach your capture files (or link to cloud storage if too large)
4. Include:
   - Device name and USB ID (from Device Manager)
   - G Hub version
   - Brief description of each capture file

## Privacy Note

USB captures may contain:
- Your Windows username (in USB device paths)
- Timestamps

They do NOT contain:
- Network traffic
- File contents
- Passwords

If concerned, you can scrub captures with Wireshark's "Edit -> Preferences -> Name Resolution" settings, or just let us know and we'll treat the files as confidential.

## Quick Capture Script

For convenience, save this as `capture_ffb.bat` and run from an admin command prompt:

```batch
@echo off
setlocal

REM === Configuration ===
set DEVICE_NAME=gpro
set USB_BUS=2
set CAPTURE_DIR=%USERPROFILE%\Desktop\wheel_captures

REM === Setup ===
mkdir "%CAPTURE_DIR%" 2>nul
cd /d "%ProgramFiles%\USBPcap"

echo.
echo Logitech Wheel USB Capture Script
echo ==================================
echo Device: %DEVICE_NAME%
echo Output: %CAPTURE_DIR%
echo.

:menu
echo.
echo Select capture type:
echo   1. G Hub initialization
echo   2. FFB gameplay (60 seconds)
echo   3. Rotation range sweep
echo   4. FFB strength sweep
echo   5. Damping sweep
echo   6. TRUEFORCE sweep
echo   7. Cold boot sequence
echo   8. Custom (manual stop)
echo   9. Exit
echo.
set /p choice="Enter choice (1-9): "

if "%choice%"=="1" goto capture_init
if "%choice%"=="2" goto capture_ffb
if "%choice%"=="3" goto capture_rotation
if "%choice%"=="4" goto capture_strength
if "%choice%"=="5" goto capture_damping
if "%choice%"=="6" goto capture_trueforce
if "%choice%"=="7" goto capture_coldboot
if "%choice%"=="8" goto capture_custom
if "%choice%"=="9" goto end

echo Invalid choice
goto menu

:capture_init
echo.
echo INSTRUCTIONS:
echo   1. Close G Hub completely (quit from system tray)
echo   2. Press any key to start capture
echo   3. Launch G Hub
echo   4. Wait for wheel to connect
echo   5. Press Ctrl+C to stop
echo.
pause
USBPcapCMD.exe -d \\.\USBPcap%USB_BUS% -o "%CAPTURE_DIR%\ghub_init_%DEVICE_NAME%.pcapng"
goto menu

:capture_ffb
echo.
echo INSTRUCTIONS:
echo   1. Launch your racing game
echo   2. Press any key when ready to capture
echo   3. Drive for 60 seconds with varied inputs
echo   4. Capture will auto-stop after 60 seconds
echo.
pause
USBPcapCMD.exe -d \\.\USBPcap%USB_BUS% -o "%CAPTURE_DIR%\ffb_gameplay_%DEVICE_NAME%.pcapng" -s 60
goto menu

:capture_rotation
echo.
echo INSTRUCTIONS:
echo   1. Open G Hub wheel settings
echo   2. Press any key to start capture
echo   3. Slowly drag Rotation Range from min to max
echo   4. Press Ctrl+C when done
echo.
pause
USBPcapCMD.exe -d \\.\USBPcap%USB_BUS% -o "%CAPTURE_DIR%\rotation_range_%DEVICE_NAME%.pcapng"
goto menu

:capture_strength
echo.
echo INSTRUCTIONS:
echo   1. Open G Hub wheel settings
echo   2. Press any key to start capture
echo   3. Slowly drag FFB Strength from min to max
echo   4. Press Ctrl+C when done
echo.
pause
USBPcapCMD.exe -d \\.\USBPcap%USB_BUS% -o "%CAPTURE_DIR%\ffb_strength_%DEVICE_NAME%.pcapng"
goto menu

:capture_damping
echo.
echo INSTRUCTIONS:
echo   1. Open G Hub wheel settings
echo   2. Press any key to start capture
echo   3. Slowly drag Damping from min to max
echo   4. Press Ctrl+C when done
echo.
pause
USBPcapCMD.exe -d \\.\USBPcap%USB_BUS% -o "%CAPTURE_DIR%\damping_%DEVICE_NAME%.pcapng"
goto menu

:capture_trueforce
echo.
echo INSTRUCTIONS:
echo   1. Open G Hub wheel settings
echo   2. Press any key to start capture
echo   3. Slowly drag TRUEFORCE from min to max
echo   4. Press Ctrl+C when done
echo.
pause
USBPcapCMD.exe -d \\.\USBPcap%USB_BUS% -o "%CAPTURE_DIR%\trueforce_%DEVICE_NAME%.pcapng"
goto menu

:capture_coldboot
echo.
echo INSTRUCTIONS:
echo   1. Unplug your wheel
echo   2. Close G Hub completely
echo   3. Press any key to start capture
echo   4. Plug in your wheel
echo   5. Wait 10 seconds
echo   6. Launch G Hub
echo   7. Wait for connection
echo   8. Press Ctrl+C when done
echo.
pause
USBPcapCMD.exe -d \\.\USBPcap%USB_BUS% -o "%CAPTURE_DIR%\cold_boot_%DEVICE_NAME%.pcapng"
goto menu

:capture_custom
echo.
echo Starting custom capture. Press Ctrl+C to stop.
echo.
set /p custom_name="Enter filename (without extension): "
USBPcapCMD.exe -d \\.\USBPcap%USB_BUS% -o "%CAPTURE_DIR%\%custom_name%.pcapng"
goto menu

:end
echo.
echo Captures saved to: %CAPTURE_DIR%
echo.
pause
```

## Technical Background

### What we're looking for in a new wheel

For each new target, we need to determine:

1. **USB Interface Structure**
   - How many interfaces does it present?
   - Which interface handles FFB?
   - Does it expose HID++ sub-devices (like the G Pro's sub-device 0x05 for calibration)?

2. **FFB Command Format**
   - Does it use HID++ Feature 0x8123 (G920/G923 style, now shared by G Pro's PID FFB)?
   - Or dedicated USB endpoints + raw 64-byte reports (RS50 / TRUEFORCE style, shared by G Pro for TF streaming)?
   - What's the report format?

3. **Force Value Encoding**
   - Signed vs unsigned
   - Offset binary vs two's complement
   - Resolution (8-bit, 16-bit?)

### Reference: RS50 / G Pro FFB Architecture

The RS50 uses dedicated endpoints for FFB, not HID++:

- Interface 2, Endpoint 0x03 OUT
- 64-byte reports, report ID 0x01
- Constant force: `01 00 00 00 01 <seq> <force_lo> <force_hi> <force_lo> <force_hi> ...`
- Force value: little-endian offset-binary u16, `0x8000` = neutral

Both games and the G Hub TRUEFORCE stream speak the same 64-byte format on interface 2. See `docs/TRUEFORCE_PROTOCOL.md` for the full streaming layout including the 68-packet two-pass init.

The G PRO Racing Wheel uses the same interface 2 / ep 0x03 TRUEFORCE transport as the RS50 (byte-for-byte identical init, verified 2026-04-21). Its regular PID FFB goes through HID++ Feature 0x8123 on interface 1 instead, following the G920 / G923 pattern.

A direct-drive wheel from a different vendor or family most likely follows one of the two patterns above, but only a capture will tell you which.

## Questions?

Open an issue on GitHub or check existing discussions. Your captures are valuable - even partial data helps us understand the protocol.

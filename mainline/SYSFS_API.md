# RS50 Linux Driver - Sysfs API Reference

**Driver**: `hid-logitech-hidpp`
**Device**: Logitech RS50 Direct Drive Wheel Base (USB 046d:c276)
**Version**: 2026-01-30

---

## Overview

The RS50 driver exposes wheel configuration through sysfs attributes located at:
```
/sys/bus/hid/devices/<DEVICE_ID>/
```

Where `<DEVICE_ID>` is typically `0003:046D:C276.XXXX`.

To find your device path:
```bash
find /sys/bus/hid/devices -name "*046D*C276*" 2>/dev/null
```

---

## Mode and Profile Control

### rs50_mode
**Access**: Read/Write
**Values**: `desktop`, `onboard`

Controls the operating mode of the wheel base.

- **Desktop mode** (profile 0): Settings controlled by host software. Sensitivity available.
- **Onboard mode** (profiles 1-5): Settings stored in wheel profiles. Brake force available.

```bash
# Read current mode
cat rs50_mode
# Output: "desktop" or "onboard"

# Switch to onboard mode
echo "onboard" > rs50_mode

# Switch to desktop mode
echo "desktop" > rs50_mode
```

### rs50_profile
**Access**: Read/Write
**Values**: `0` (desktop), `1-5` (onboard profiles)

Controls the active profile. Profile 0 is desktop mode; profiles 1-5 are onboard profiles.

```bash
# Read current profile
cat rs50_profile
# Output: 0-5

# Switch to onboard profile 3
echo 3 > rs50_profile

# Switch to desktop mode (profile 0)
echo 0 > rs50_profile
```

---

## Force Feedback Settings

### rs50_range
**Access**: Read/Write
**Values**: `90` to `2700` (degrees)

Sets the steering wheel rotation range.

```bash
# Read current range
cat rs50_range
# Output: degrees (e.g., "900")

# Set to 540 degrees
echo 540 > rs50_range
```

### rs50_strength
**Access**: Read/Write
**Values**: `0` to `100` (percentage)

Sets the force feedback strength.

**Internal encoding**: The driver converts percentage to a 16-bit value where:
- 0% = 0x0000
- 100% = 0xFFFF (corresponds to 8.0 Nm max torque)

```bash
# Read current strength
cat rs50_strength
# Output: percentage (e.g., "75")

# Set to 50%
echo 50 > rs50_strength
```

### rs50_damping
**Access**: Read/Write
**Values**: `0` to `100` (percentage)

Sets the wheel damping (resistance when turning).

```bash
# Read current damping
cat rs50_damping

# Set to 25%
echo 25 > rs50_damping
```

### rs50_trueforce
**Access**: Read/Write
**Values**: `0` to `100` (percentage)

Sets the TRUEFORCE bass shaker intensity.

```bash
# Read current TRUEFORCE level
cat rs50_trueforce

# Enable at 80%
echo 80 > rs50_trueforce

# Disable
echo 0 > rs50_trueforce
```

### rs50_brake_force
**Access**: Read/Write
**Values**: `0` to `100` (percentage)
**Mode Restriction**: **Onboard mode only**

Sets the brake pedal force threshold (load cell sensitivity).

**Note**: Returns `-EPERM` (Permission denied) in desktop mode.

```bash
# Set brake force to 75% (must be in onboard mode)
echo "onboard" > rs50_mode
echo 75 > rs50_brake_force
```

### rs50_sensitivity
**Access**: Read/Write
**Values**: `0` to `100` (percentage)
**Mode Restriction**: **Desktop mode only**

Sets the wheel sensitivity/responsiveness.

**Note**: Returns `N/A (onboard mode)` when reading in onboard mode, and `-EPERM` when writing.

```bash
# Set sensitivity to 50% (must be in desktop mode)
echo "desktop" > rs50_mode
echo 50 > rs50_sensitivity
```

### rs50_ffb_filter
**Access**: Read/Write
**Values**: `0` to `5` (filter level)

Sets the force feedback smoothing/filtering level.

```bash
# Read current filter level
cat rs50_ffb_filter

# Set to level 3
echo 3 > rs50_ffb_filter
```

### rs50_ffb_filter_auto
**Access**: Read/Write
**Values**: `0` (manual), `1` (auto)

Enables automatic FFB filter adjustment based on game output.

```bash
# Enable auto filter
echo 1 > rs50_ffb_filter_auto

# Disable (use manual filter setting)
echo 0 > rs50_ffb_filter_auto
```

---

## LIGHTSYNC LED Control

The RS50 wheel base has 10 RGB LEDs arranged in a strip. The driver provides per-slot configuration with 5 custom slots (0-4).

### LED Control Workflow

1. **Select a slot**: `echo 2 > rs50_led_slot`
2. **Set direction** (optional): `echo 1 > rs50_led_direction`
3. **Set colors**: `echo "FF0000 FF0000 ... (10 colors)" > rs50_led_colors`
4. **Apply changes**: `echo 1 > rs50_led_apply`

Alternatively, use built-in effects via `rs50_led_effect`.

### rs50_led_slot
**Access**: Read/Write
**Values**: `0` to `4` (custom slot index)

Selects the active custom LED slot for configuration.

```bash
# Select slot 2
echo 2 > rs50_led_slot
```

### rs50_led_direction
**Access**: Read/Write
**Values**: `0` to `3`

Sets the LED animation direction for the current slot:
- `0` = Left to Right
- `1` = Right to Left
- `2` = Inside Out (center outward)
- `3` = Outside In (edges inward)

```bash
# Set direction to Right-to-Left
echo 1 > rs50_led_direction
```

### rs50_led_colors
**Access**: Read/Write
**Format**: 10 space-separated hex RGB values (`RRGGBB`)

Sets all 10 LED colors for the current slot. LED1 is leftmost.

```bash
# Set all LEDs to red
echo "FF0000 FF0000 FF0000 FF0000 FF0000 FF0000 FF0000 FF0000 FF0000 FF0000" > rs50_led_colors

# Rainbow effect (example)
echo "FF0000 FF7F00 FFFF00 7FFF00 00FF00 00FF7F 00FFFF 007FFF 0000FF 7F00FF" > rs50_led_colors

# Read current colors
cat rs50_led_colors
# Output: "RRGGBB RRGGBB RRGGBB RRGGBB RRGGBB RRGGBB RRGGBB RRGGBB RRGGBB RRGGBB"
```

### rs50_led_brightness
**Access**: Read/Write
**Values**: `0` to `100` (percentage)

Sets the global LED brightness.

```bash
# Set brightness to 50%
echo 50 > rs50_led_brightness
```

### rs50_led_effect
**Access**: Read/Write
**Values**: Effect index (device-specific)

Selects a built-in LED effect. Known values:
- `5` = Static/Custom (use custom slot colors)
- `6` = Built-in effect 1
- `9` = Built-in effect 2

```bash
# Use custom colors
echo 5 > rs50_led_effect

# Use built-in effect
echo 6 > rs50_led_effect
```

### rs50_led_apply
**Access**: Write-only
**Values**: `1` (apply)

Applies the current slot configuration to the device.

```bash
# Apply current slot settings
echo 1 > rs50_led_apply
```

---

## Pedal Configuration

### rs50_combined_pedals
**Access**: Read/Write
**Values**: `0` (separate), `1` (combined)

Enables combined pedal axis mode (throttle and brake on single axis).

```bash
# Enable combined pedals
echo 1 > rs50_combined_pedals

# Disable (separate axes)
echo 0 > rs50_combined_pedals
```

### rs50_throttle_curve / rs50_brake_curve / rs50_clutch_curve
**Access**: Read/Write
**Values**: `0` (linear), `1` (low sensitivity), `2` (high sensitivity)

Sets the response curve for each pedal:
- `0` = Linear (1:1 mapping)
- `1` = Low sensitivity (less sensitive at start)
- `2` = High sensitivity (more sensitive at start)

```bash
# Set brake to low sensitivity curve
echo 1 > rs50_brake_curve
```

### rs50_throttle_deadzone / rs50_brake_deadzone / rs50_clutch_deadzone
**Access**: Read/Write
**Values**: `0` to `100` (percentage)

Sets the deadzone for each pedal axis.

```bash
# Set 5% brake deadzone
echo 5 > rs50_brake_deadzone
```

---

## Compatibility Attributes

These attributes provide compatibility with existing wheel management tools (e.g., Oversteer).

### rs50_compat_range
**Access**: Read/Write
**Values**: `90` to `2700` (degrees)

Alias for `rs50_range` for Oversteer compatibility.

### rs50_compat_gain
**Access**: Read/Write
**Values**: `0` to `100` (percentage)

Alias for `rs50_strength` for Oversteer compatibility.

### rs50_compat_autocenter
**Access**: Read/Write
**Values**: `0` to `100` (percentage)

Sets auto-centering spring strength.

### rs50_compat_damper_level
**Access**: Read/Write
**Values**: `0` to `100` (percentage)

Alias for `rs50_damping` for Oversteer compatibility.

### rs50_compat_combine_pedals
**Access**: Read/Write
**Values**: `0`, `1`

Alias for `rs50_combined_pedals` for Oversteer compatibility.

---

## Debug Attributes

### rs50_hidpp_debug
**Access**: Read/Write
**Values**: `0` (off), `1` (on)

Enables verbose HID++ protocol debug logging in dmesg.

```bash
# Enable debug logging
echo 1 > rs50_hidpp_debug

# View logs
dmesg | grep RS50
```

---

## Error Codes

| Error | Meaning |
|-------|---------|
| `-ENODEV` | Device not found or driver not ready |
| `-EPERM` | Operation not permitted in current mode |
| `-EINVAL` | Invalid value provided |
| `-EOPNOTSUPP` | Feature not supported by device |
| `-EIO` | Communication error with device |

---

## Example Scripts

### Quick Setup Script
```bash
#!/bin/bash
# Set up RS50 for racing

DEVICE=$(find /sys/bus/hid/devices -name "*046D*C276*" | head -1)
cd "$DEVICE" || exit 1

# Force feedback settings
echo 900 > rs50_range        # 900 degrees
echo 75 > rs50_strength      # 75% force
echo 20 > rs50_damping       # 20% damping
echo 50 > rs50_trueforce     # 50% TRUEFORCE

# LED: Red theme
echo 0 > rs50_led_slot
echo "FF0000 FF0000 FF0000 FF0000 FF0000 FF0000 FF0000 FF0000 FF0000 FF0000" > rs50_led_colors
echo 1 > rs50_led_apply

echo "RS50 configured!"
```

### Mode Switch Script
```bash
#!/bin/bash
# Toggle between desktop and onboard mode

DEVICE=$(find /sys/bus/hid/devices -name "*046D*C276*" | head -1)
MODE=$(cat "$DEVICE/rs50_mode")

if [ "$MODE" = "desktop" ]; then
    echo "onboard" > "$DEVICE/rs50_mode"
    echo "Switched to onboard mode"
else
    echo "desktop" > "$DEVICE/rs50_mode"
    echo "Switched to desktop mode"
fi
```

---

## Protocol Details

For developers interested in the HID++ protocol details, see:
- `RS50_PROTOCOL_SPECIFICATION.md` - Full protocol documentation
- `dev/docs/CAPTURE_ANALYSIS_*.md` - USB capture analysis

### Feature Pages Used

| Page | Index Var | Description |
|------|-----------|-------------|
| 0x8040 | idx_brightness | LED Brightness / Sensitivity |
| 0x807A | idx_lightsync | LIGHTSYNC Effects |
| 0x807B | idx_rgb_config | RGB Zone Configuration |
| 0x8133 | idx_damping | Wheel Damping |
| 0x8134 | idx_brakeforce | Brake Force |
| 0x8136 | idx_strength | FFB Strength |
| 0x8137 | idx_profile | Profile/Mode Switching |
| 0x8138 | idx_range | Rotation Range |
| 0x8139 | idx_trueforce | TRUEFORCE |
| 0x8140 | idx_filter | FFB Filter |

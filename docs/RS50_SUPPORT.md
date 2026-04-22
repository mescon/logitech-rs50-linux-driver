# Logitech RS50 Linux Driver Support

## Overview

This patch adds Linux kernel driver support for the **Logitech RS50 Direct Drive Wheel Base**
(USB Product ID: `0xc276`, marketed as "RS50 Base for PlayStation/PC").

## Device Information

| Property | Value |
|----------|-------|
| Vendor ID | `0x046d` (Logitech) |
| Product ID | `0xc276` |
| Device Name | Logitech RS50 Base for PlayStation/PC |
| Type | Direct Drive Racing Wheel Base |
| Platform | PlayStation 5/4, PC |

## Technical Background

### Key Differences from G920/G923

| Feature | G920/G923 | RS50 |
|---------|-----------|------|
| FFB Protocol | HID++ Feature 0x8123 | Dedicated endpoint 0x03 |
| Report ID | 0x11/0x12 | 0x01 (custom) |
| Sequence Field | 2 bytes | 1 byte |
| Max Rotation | 900° | 2700° |
| Motor Type | Belt/Gear | Direct drive |

### Protocol Analysis

The RS50 uses a **custom FFB protocol** on endpoint 0x03, NOT the standard HID++ Feature 0x8123
that other Logitech wheels use. This was discovered through USB capture analysis.

**FFB Report Format** (64 bytes, endpoint 0x03 OUT):
```
Offset  Size  Description
------  ----  -----------
0       1     Report ID (0x01)
1-3     3     Reserved (zeros)
4       1     Effect type (0x01 = constant)
5       1     Sequence counter (0x00-0xFF)
6-7     2     Force value (little-endian)
8-9     2     Force value duplicate
10-63   54    Padding (zeros)
```

**Force Values** (offset binary):
- `0x0000` = Maximum LEFT
- `0x8000` = NEUTRAL
- `0xFFFF` = Maximum RIGHT

### Why HIDPP_QUIRK_CLASS_G920 Is Not Sufficient

The existing `HIDPP_QUIRK_CLASS_G920` quirk attempts to use HID++ Feature 0x8123 for FFB.
While the RS50 does expose Feature 0x8123, it doesn't actually use it for force feedback.
The RS50 requires custom handling to send FFB via the dedicated endpoint.

## Implementation Requirements

1. **Claim Interface 2**: FFB endpoint 0x03 is on interface 2
2. **Custom FFB Handler**: Send 64-byte reports with 1-byte sequence counter
3. **Periodic Refresh**: Send `05 07` command every ~30 seconds during gameplay
4. **Settings via HID++**: Use proprietary features (0x8133-0x8140) for G Hub settings

## USB Endpoints

| Endpoint | Direction | Interface | Purpose |
|----------|-----------|-----------|---------|
| 0x81 | IN | 0 | Joystick (30-byte reports) |
| 0x82 | IN | 1 | HID++ responses |
| 0x00 | OUT | 1 | HID++ commands (control) |
| 0x83 | IN | 2 | FFB status |
| 0x03 | OUT | 2 | FFB commands |

## Testing

```bash
# Compile driver
cd mainline && make

# Load driver
sudo rmmod hid-logitech-hidpp 2>/dev/null
sudo insmod ./hid-logitech-hidpp.ko

# Check detection
dmesg | grep -i rs50

# Test FFB
fftest /dev/input/eventX
```

## References

- [`RS50_PROTOCOL_SPECIFICATION.md`](RS50_PROTOCOL_SPECIFICATION.md) - Complete protocol documentation
- [`SYSFS_API.md`](SYSFS_API.md) - Sysfs API reference for configuration
- [JacKeTUs/hid-logitech-hidpp](https://github.com/JacKeTUs/hid-logitech-hidpp) - Base driver fork

## Status

**Current**: Under active development - may contain bugs or incomplete features
**Wheels covered**: RS50 (`046d:c276`) and, reusing the same settings code path, the Logitech G Pro Racing Wheel (`046d:c272` Xbox/PC, `046d:c268` PS/PC).
**Features**:
- Force feedback (FF_CONSTANT) on RS50 via dedicated endpoint 0x03. G Pro uses the G920 HID++ FFB path (FF_CONSTANT plus the rest of the standard effects).
- All buttons mapped, 8-way D-pad.
- sysfs settings: rotation range, FFB strength, damping, TRUEFORCE, brake force, sensitivity, FFB filter + auto, profile/mode.
- Centre calibration (`wheel_calibrate`, G Pro only) targeting sub-device 0x05 page `0x812C`.
- LIGHTSYNC LED control (RS50 only; G Pro LIGHTSYNC not yet confirmed byte-for-byte).
- TRUEFORCE: out-of-kernel userspace (`userspace/libtrueforce/`, Wine PE shim in `userspace/tf_wine_shim/`), both wheels.

**Kernel Compatibility**: Linux 5.15+ (tested on 5.15, 6.1, 6.8, 6.12, 6.18)
**Date**: 2026-04-21

### Architecture Note

The RS50 uses a **fundamentally different FFB architecture** than the G920/G923:
- G920/G923: FFB via HID++ Feature 0x8123 (FAP messages)
- RS50: FFB via dedicated USB endpoint with raw HID output reports

This means code for G920 FFB cannot be directly reused for RS50. The driver uses `HIDPP_QUIRK_RS50_FFB` to select the appropriate code path.

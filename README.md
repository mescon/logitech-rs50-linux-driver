# Logitech RS50 Linux Driver

Linux kernel driver for the **Logitech RS50 Direct Drive Wheel Base** (USB ID `046d:c276`).

This is a patched version of the `hid-logitech-hidpp` driver that adds RS50 support with full force feedback, including condition effects, and exposes all G Hub settings via sysfs for runtime configuration.

**Note:** This driver replaces the in-kernel `hid-logitech-hidpp` module and continues to support all other Logitech HID++ devices (mice, keyboards, other racing wheels like the G29, G920, G923, G Pro, etc.).

## Features

- **Force Feedback**
  - FF_CONSTANT: Standard constant force
  - FF_SPRING: Centering spring effect
  - FF_DAMPER: Velocity-based resistance
  - FF_FRICTION: Constant directional resistance
  - FF_INERTIA: Acceleration-based resistance

- **Complete Input Support**
  - All 17 buttons mapped
  - 8-direction D-pad
  - High-resolution wheel axis (up to 2700°)
  - 16-bit pedal axes (throttle, brake, clutch)

- **G Hub Settings via sysfs**
  - Rotation range (90-2700°)
  - FFB strength, damping, TRUEFORCE
  - Brake force (load cell threshold)
  - FFB filter level and auto mode
  - LED effect and brightness

- **Pedal Customization**
  - Response curves (linear, low sensitivity, high sensitivity)
  - Configurable deadzones
  - Combined pedals mode for older games

## Button Mapping

![RS50 Button Layout](docs/images/rs-wheel-hub-button-layout.png)

| # | Button | Linux Input Code |
|---|--------|------------------|
| 1 | X | BTN_THUMB |
| 2 | Y | BTN_TOP |
| 3 | A | BTN_TRIGGER |
| 4 | B | BTN_THUMB2 |
| 5 | RT (Right Trigger) | BTN_BASE |
| 6 | RSB (Right Stick Button) | BTN_BASE5 |
| 7 | GR (Gear Right) | BTN_TRIGGER_HAPPY14 |
| 8₁ | Right Encoder Twist ↑ (CW) | BTN_TRIGGER_HAPPY6 |
| 8₂ | Right Encoder Twist ↓ (CCW) | BTN_TRIGGER_HAPPY7 |
| 8₃ | Right Encoder Push | BTN_TRIGGER_HAPPY8 |
| 9 | Menu | BTN_BASE4 |
| 10 | G1 | BTN_TRIGGER_HAPPY12 |
| 11 | Camera | BTN_BASE3 |
| 12₁ | Left Encoder Twist ↑ (CW) | BTN_TRIGGER_HAPPY9 |
| 12₂ | Left Encoder Twist ↓ (CCW) | BTN_TRIGGER_HAPPY10 |
| 12₃ | Left Encoder Push | BTN_TRIGGER_HAPPY11 |
| 13 | GL (Gear Left) | BTN_TRIGGER_HAPPY13 |
| 14 | LSB (Left Stick Button) | BTN_BASE6 |
| 15 | LT (Left Trigger) | BTN_BASE2 |
| 16 | Left Paddle (behind LT) | BTN_PINKIE |
| 17 | Right Paddle (behind RT) | BTN_TOP2 |
| D | D-pad (right of LT/LSB) | ABS_HAT0X / ABS_HAT0Y |

## Requirements

- Linux kernel 6.6+ (tested on 6.12 and 6.18)
- Kernel headers for your running kernel
- Build tools: `make`, `clang` or `gcc`

## Installation

### Step 1: Build the Driver

```bash
git clone https://github.com/mescon/logitech-rs50-linux-driver.git
cd logitech-rs50-linux-driver/mainline
make
```

### Step 2: Install with DKMS (Recommended)

DKMS automatically rebuilds the driver when you update your kernel.

```bash
# Install DKMS if not already installed
# Arch: sudo pacman -S dkms
# Ubuntu/Debian: sudo apt install dkms
# Fedora: sudo dnf install dkms

# Copy source to DKMS directory
sudo mkdir -p /usr/src/hid-logitech-hidpp-1.0/build
sudo cp mainline/* /usr/src/hid-logitech-hidpp-1.0/build/
sudo cp dkms.conf /usr/src/hid-logitech-hidpp-1.0/

# Register and build with DKMS
sudo dkms add -m hid-logitech-hidpp -v 1.0
sudo dkms build -m hid-logitech-hidpp -v 1.0
sudo dkms install -m hid-logitech-hidpp -v 1.0
```

### Step 3: Blacklist the In-Kernel Driver

The kernel includes an older `hid-logitech-hidpp` driver without RS50 support. You must blacklist it:

```bash
echo "blacklist hid-logitech-hidpp" | sudo tee /etc/modprobe.d/blacklist-hid-logitech-hidpp.conf
sudo depmod -a
```

### Step 4: Load the Driver

```bash
# Unload old driver if loaded
sudo rmmod hid-logitech-hidpp 2>/dev/null

# Load new driver
sudo modprobe hid-logitech-hidpp

# Verify RS50 is detected
dmesg | grep -i "rs50"
```

You should see: `RS50 force feedback initialized with condition effects support`

### Quick Test (Without DKMS)

For testing without permanent installation:

```bash
cd mainline
make
sudo rmmod hid-logitech-hidpp 2>/dev/null
sudo insmod ./hid-logitech-hidpp.ko
dmesg | grep -i rs50
```

## Usage

### Test Force Feedback

```bash
# Find your device
ls /dev/input/by-id/ | grep -i logi

# Test FFB (requires linuxconsole package)
fftest /dev/input/by-id/usb-Logitech_RS50*-event-joystick
```

### Configure Settings via sysfs

Settings are exposed at `/sys/class/hidraw/hidrawX/device/` (where X varies by system).

```bash
# Find your RS50's hidraw device
RS50_DEV=$(ls -d /sys/class/hidraw/*/device/rs50_range 2>/dev/null | head -1 | xargs dirname)
echo "RS50 found at: $RS50_DEV"

# Example: Set rotation to 900 degrees
echo 900 | sudo tee $RS50_DEV/rs50_range

# Example: Set FFB strength to 80%
echo 80 | sudo tee $RS50_DEV/rs50_strength

# Example: Set LED to static effect
echo 5 | sudo tee $RS50_DEV/rs50_led_effect
```

### Available sysfs Attributes

| Attribute | Range | Description |
|-----------|-------|-------------|
| `rs50_range` | 90-2700 | Rotation range in degrees |
| `rs50_strength` | 0-100 | FFB strength percentage |
| `rs50_damping` | 0-100 | Damping percentage |
| `rs50_trueforce` | 0-100 | TRUEFORCE audio-haptic level |
| `rs50_brake_force` | 0-100 | Brake pedal load cell threshold |
| `rs50_ffb_filter` | 1-15 | FFB smoothing level |
| `rs50_ffb_filter_auto` | 0-1 | Auto FFB filter (0=off, 1=on) |
| `rs50_led_effect` | 1-5 | LED effect (1-4=animated, 5=static) |
| `rs50_led_brightness` | 0-100 | LED brightness percentage |
| `rs50_combined_pedals` | 0-1 | Combined pedals mode |
| `rs50_throttle_curve` | 0-2 | Throttle response curve |
| `rs50_brake_curve` | 0-2 | Brake response curve |
| `rs50_clutch_curve` | 0-2 | Clutch response curve |
| `rs50_throttle_deadzone` | "L U" | Throttle deadzone (lower% upper%) |
| `rs50_brake_deadzone` | "L U" | Brake deadzone |
| `rs50_clutch_deadzone` | "L U" | Clutch deadzone |

### Oversteer Compatibility

The driver exposes standard wheel attributes for [Oversteer](https://github.com/berarma/oversteer) compatibility:
- `range` - Rotation range (up to 2700°)
- `gain` - FFB strength
- `autocenter` - Autocenter strength (stub)
- `combine_pedals` - Combined pedals mode
- `damper_level` - Damping level

**Note:** Oversteer requires a patch for RS50 support. This patch has been submitted upstream; until merged, you can apply it manually.

The patch (`oversteer-rs50-support.patch`) adds:
- RS50 device detection (USB ID `046d:c276`)
- 2700° rotation range support
- Correct pedal axis mapping
- udev permissions for non-root access

#### Applying the Patch

**Option 1: System package / pip install**

```bash
# Find where Oversteer is installed
python3 -c "import oversteer; print(oversteer.__file__)"
# Usually: /usr/lib/python3.x/site-packages/oversteer/__init__.py

# Apply patch (adjust path as needed)
cd /usr/lib/python3.x/site-packages/
sudo patch -p1 < /path/to/oversteer-rs50-support.patch
```

**Option 2: From git source**

```bash
git clone https://github.com/berarma/oversteer.git
cd oversteer
git apply /path/to/oversteer-rs50-support.patch
sudo pip install .
```

**Option 3: Flatpak**

Flatpak apps are sandboxed, so you need to extract, patch, and reinstall:

```bash
# Export the installed Flatpak to a bundle
flatpak build-bundle ~/.local/share/flatpak/repo oversteer.flatpak \
  io.github.berarma.Oversteer

# Unfortunately, Flatpak bundles can't be easily patched.
# For Flatpak users, the recommended approach is to:
# 1. Uninstall the Flatpak version
flatpak uninstall io.github.berarma.Oversteer

# 2. Install from source with the patch applied (Option 2 above)

# 3. Or wait for the upstream patch to be merged and Flatpak updated
```

#### udev Rule (Required for non-root access)

Create `/etc/udev/rules.d/99-oversteer-rs50.rules`:

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c276", MODE="0666", TAG+="uaccess"
```

Then reload:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Game Compatibility

The driver works with any game that supports Linux force feedback:

| Game | Status | Notes |
|------|--------|-------|
| **Native Linux** | ✓ | F1, Dirt Rally 2.0, Euro Truck Simulator 2 |
| **Proton/Steam** | ✓ | Assetto Corsa, ACC, iRacing, etc. |
| **Wine** | ✓ | Most racing games via Proton |

Games detect the wheel as a standard Linux joystick with FF support. No special configuration needed beyond setting up controls in-game.

### Proton Tips

- Enable "Steam Input" → "Gamepad with Joystick Trackpad" for some games
- Some games may need `SDL_JOYSTICK_DEVICE=/dev/input/eventX` environment variable

## Technical Details

The RS50 is a multi-interface USB device:
- Interface 0: Joystick input (30-byte reports)
- Interface 1: HID++ 4.2 protocol (configuration)
- Interface 2: Force feedback (64-byte reports on endpoint 0x03)

Unlike the G920/G923 which use HID++ Feature 0x8123 for FFB, the RS50 uses a dedicated endpoint with a custom protocol. See `docs/RS50_PROTOCOL_SPECIFICATION.md` for complete protocol documentation.

## Troubleshooting

### "Invalid code 768" messages during boot

These are harmless warnings from the HID descriptor declaring more buttons than physically exist. The driver patches the descriptor to prevent them during normal operation.

### FFB not working

1. Verify the driver loaded: `lsmod | grep hidpp`
2. Check dmesg for errors: `dmesg | grep -i rs50`
3. Ensure you're testing with a game/app that supports FFB

### Settings not persisting

sysfs settings are volatile and reset on driver reload. For persistent settings, add commands to a udev rule or startup script.

### Wine/Proton claiming the device (no FFB or double input)

Wine can access USB devices directly via its HID backend, bypassing the Linux driver. This causes:
- No force feedback (Wine doesn't understand the RS50 FFB protocol)
- Double/ghost inputs (both Wine and Linux see the device)
- Wheel not detected at all in some games

**Solution 1: Disable Wine's direct HID access (Recommended)**

Create a file to hide the RS50 from Wine's HID layer:

```bash
# For Steam/Proton games, add to your game's launch options:
PROTON_ENABLE_HIDRAW=0 %command%

# Or disable globally by creating:
echo 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="c276", MODE="0000"' | \
  sudo tee /etc/udev/rules.d/99-hide-rs50-from-wine.rules
sudo udevadm control --reload-rules
```

**Solution 2: Use SDL instead of Wine's dinput**

Some games work better with SDL's joystick handling:

```bash
# Steam launch options:
SDL_JOYSTICK_HIDAPI=0 %command%
```

**Solution 3: Check hidraw permissions**

If Oversteer or sysfs settings don't work, Wine may have grabbed the hidraw device:

```bash
# Find your RS50's hidraw device number
ls -la /sys/class/hidraw/*/device/rs50_range 2>/dev/null

# Check who has the device open (replace X with your hidraw number)
sudo lsof /dev/hidrawX

# If wine processes are listed, close them or use Solution 1
```

## Contributing

Contributions are welcome! This driver is forked from [JacKeTUs/hid-logitech-hidpp](https://github.com/JacKeTUs/hid-logitech-hidpp) with RS50-specific additions. If your changes apply to other Logitech devices, please consider contributing upstream as well.

## License

GPL-2.0-only (same as the Linux kernel)

## Acknowledgments

- RS50 USB protocol reverse-engineered using Wireshark captures from G Hub on Windows
- Based on [JacKeTUs/hid-logitech-hidpp](https://github.com/JacKeTUs/hid-logitech-hidpp) which adds G Pro wheel support and improved FFB
- Upstream Linux kernel [hid-logitech-hidpp driver](https://github.com/torvalds/linux/blob/master/drivers/hid/hid-logitech-hidpp.c) by Benjamin Tissoires and contributors
- [Oversteer](https://github.com/berarma/oversteer) by Bernat Arlandis for the wheel configuration GUI

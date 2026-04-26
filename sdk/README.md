# Logitech SDK Inventory

This directory holds copies of Logitech Windows SDK artifacts we use to reason about the driver's Linux-side API shape and the Wine bridge design.

## What's here

| Path | Contents | Origin |
|---|---|---|
| `Include/LogitechSteeringWheelLib.h` | Public C header for the legacy Steering Wheel SDK | Published 2015 (C# + C++) |
| `Include/LogitechGSDK.cs` | C# binding for the same legacy SDK | Same |
| `Lib/` | Prebuilt .lib files for the legacy SDK | Same |
| `Doc/`, `Demo/`, `Samples/` | Reference material and sample projects | Same |
| `trueforce_1_3_11/trueforce_sdk_{x64,x86}.dll` | **Trueforce SDK 1.3.11** (post-2020, private to partnered developers) | Extracted from a game install |
| `trueforce_1_3_11/exports_x64.txt` | Full export listing (75 symbols) | `winedump -j export` |
| `wheel_9_1_0/logi_steering_wheel_{x64,x86}.dll` | **Wheel SDK 9.1.0** (successor to the 2015 public SDK) | Extracted from a game install |
| `wheel_9_1_0/exports_x64.txt` | Full export listing (58 symbols) | `winedump -j export` |

## Legacy public SDK vs newer SDKs — what changed

### Public Steering Wheel SDK (2015)

Models supported top out at G920 / G29. The header enumerates constants like `LOGI_FORCE_SPRING`, `LOGI_FORCE_DIRT_ROAD`, and up to `LOGI_NUMBER_DEVICE_TYPES=4`. 39 functions. No Trueforce concept.

### Wheel SDK 9.1.0 (newer, post-2015)

58 exports. Adds versus the legacy SDK:

- `LogiGetDiState`, `LogiGetDiStateENGINES` — direct DirectInput state passthrough (bypasses internal bookkeeping)
- `LogiFreeStateENGINES` — explicit state struct cleanup (paired with GetStateENGINES)
- `LogiGetLedCaps`, `LogiGetLedCapsDInput` — runtime LED capability discovery
- `LogiSetRpmLedsDirect`, `LogiSetRpmLedsDirectDInput` — direct RPM LED control (vs `LogiPlayLeds` which takes RPM/first/redline)

Everything else is the same API. No Trueforce.

### Trueforce SDK 1.3.11

This is the SDK that Trueforce-aware games (BeamNG, some AC titles, iRacing) link against. 75 total exports; 59 have readable names, 16 are obfuscated C++ symbols that we don't need to shim.

**API groups (readable exports):**

Device lifecycle:
- `logiWheelOpenByDirectInputA/W`, `logiWheelClose`, `logiWheelSdkHasControl`
- `logiTrueForceAvailable`, `logiTrueForceSupported`, `logiTrueForceSupportedByDirectInputA/W`
- `logiWheelGetVersion`, `logiWheelGetCoreLibraryVersion`

Wheel state:
- `logiTrueForceGetAngleDegrees`, `logiTrueForceGetAngleRadians`
- `logiTrueForceGetAngularVelocityDegrees`, `logiTrueForceGetAngularVelocityRadians`

Force mode and range (mirrors Wheel SDK 9.1.0):
- `logiWheelGetForceMode`, `logiWheelSetForceMode`
- `logiWheelGetOperatingRangeDegrees/Radians`, `logiWheelSetOperatingRangeDegrees/Radians`
- `logiWheelGetOperatingRangeBoundsDegrees/Radians`
- `logiWheelGetRpmLedCaps`, `logiWheelPlayLeds`

Kinetic Force (KF — the classic constant-force channel):
- `logiTrueForceSetTorqueKF`, `logiTrueForceSetTorqueKFPiecewise`, `logiTrueForceGetTorqueKF`, `logiTrueForceClearKF`
- `logiTrueForceGetGainKF`, `logiTrueForceSetGainKF`
- `logiTrueForceGetMaxContinuousTorqueKF`, `logiTrueForceGetMaxPeakTorqueKF`
- `logiTrueForceGetReconstructionFilterKF`, `logiTrueForceSetReconstructionFilterKF`

Trueforce audio stream (TF):
- `logiTrueForceSetStreamTF` — set a stream of samples (the ~1kHz bulk API)
- `logiTrueForceSetTorqueTFfloat`, `logiTrueForceSetTorqueTFdouble`, `logiTrueForceSetTorqueTFint8/16/32` — per-sample or small-buffer setters, numeric type variants
- `logiTrueForceGetTorqueTF`, `logiTrueForceClearTF`
- `logiTrueForceGetGainTF`, `logiTrueForceSetGainTF`
- `logiTrueForceGetHapticRate`, `logiTrueForceGetHapticThreadStatus`
- `logiTrueForcePause`, `logiTrueForceResume`, `logiTrueForceIsPaused`
- `logiTrueForceSync` — stream synchronization
- `logiTrueForceGetTorqueTFRateBounds`

Damping / viscosity (shared between KF and TF):
- `logiTrueForceGetDamping`, `logiTrueForceSetDamping`, `logiTrueForceGetDampingMax`
- (viscosity appears in string table; not currently an export, likely deprecated/unused from this version)

Advanced / diagnostics:
- `logiAdvancedGetThreadHandles` — exposes SDK's internal thread handles to the host (for affinity / priority control)

DllRegisterServer, DllUnregisterServer, dllOpen, dllClose — standard DLL boilerplate.

**Architecture revealed by strings:**

The SDK does NOT talk to USB directly. String table includes:

- `local_connection::Connection`, `local_connection::CodecConnection` — the SDK uses a "local connection" abstraction
- `logi.trueforce.connect` — almost certainly the IPC endpoint name (named pipe, local socket, or similar)
- `Packet::Header`, `Packet::Gains`, `Packet::Aperture`, `Packet::HeloContainerId`, `Packet::HeloProtocolVersion` — packet types serialized over the IPC
- `"TrueForce message pump"` — SDK runs a background thread processing incoming packets
- `trueforce_features.cfg`, `trueforce_data.bin` — device-specific config files (likely under `C:\Program Files\LGHUB\` or similar)

**Implication:** on Windows the flow is:

```
Game ── links ──▶ trueforce_sdk_x64.dll ── IPC(logi.trueforce.connect) ──▶ G HUB Agent ──▶ USB ep 0x03 ──▶ RS50
```

The USB wire protocol we see in captures is generated by the G HUB Agent, not by the SDK. The SDK only serializes high-level "packets" and hands them off.

### "KF" vs "TF" inside the SDK

- **KF — Kinetic Force** — classic constant-force style torque. Single value per call, or piecewise curve. Maps to the existing PID FFB path on the wheel (feature 0x8123 via HID++, or on the RS50 the dedicated endpoint 0x03 with a DC force value).
- **TF — Trueforce** — the audio haptic stream. ~1000 samples/sec (per captures). Multiple numeric types accepted (int8/int16/int32/float/double) — the SDK does the conversion before serializing.

Both channels coexist at runtime: the SDK sets both KF (slow, steering feel) and TF (fast, vibration/texture) simultaneously.

### Implications for Phase C

- We no longer need to reverse-engineer the TF API shape from captures; the exports table is the API.
- We do need to observe the IPC wire format (named pipe between `trueforce_sdk_x64.dll` and G HUB Agent) if we want a drop-in Wine DLL override. Alternative: write our own DLL that **replaces** `trueforce_sdk_x64.dll` and talks directly to our Linux kernel driver (skipping IPC).
- The "multiple numeric types" surface (int8 through double) tells us our Linux API should accept at least one common format (16-bit signed LE is the natural choice given the USB wire format).
- `logiTrueForceSetStreamTF` suggests a bulk-upload path in addition to per-sample. Our Linux API should support streaming writes via `write(2)` on a character device rather than a single ioctl per sample.

## How to re-dump the exports

```bash
# From the repo root:
winedump -j export sdk/trueforce_1_3_11/trueforce_sdk_x64.dll > sdk/trueforce_1_3_11/exports_x64.txt
winedump -j export sdk/wheel_9_1_0/logi_steering_wheel_x64.dll > sdk/wheel_9_1_0/exports_x64.txt
```

On Arch/Fedora, `winedump` ships with `wine-core`. On Debian/Ubuntu, install `wine`.

## DLL layout consumed by `tools/install-tf-shim.sh`

The shim installer expects the four real Logitech-signed SDK DLLs under
`sdk/Logi/`, mirroring the exact directory tree they install into on
Windows. These files are gitignored and **must be supplied by the user**.

Required paths:

```
sdk/Logi/Trueforce/1_3_11/trueforce_sdk_x64.dll
sdk/Logi/Trueforce/1_3_11/trueforce_sdk_x86.dll
sdk/Logi/wheel_sdk/9_1_0/logi_steering_wheel_x64.dll
sdk/Logi/wheel_sdk/9_1_0/logi_steering_wheel_x86.dll
```

How to obtain them: install Logitech G HUB on Windows (or in a throwaway
wine prefix on Linux) and copy the contents of
`C:\Program Files\Logi\Trueforce\1_3_11\` and
`C:\Program Files\Logi\wheel_sdk\9_1_0\` into the matching paths above.
File names and directory casing must match.

`tools/install-tf-shim.sh` runs `require_sources` first; if any of the
four files are missing it prints the same expected paths and exits with
status 2 without touching any wine prefix. So you can re-run the
installer safely after populating `sdk/Logi/`.

## Licensing note

The DLL files in `trueforce_1_3_11/`, `wheel_9_1_0/`, and `Logi/` are
Logitech's copyrighted binaries. They are kept locally for reference /
interoperability purposes only. We do not redistribute them publicly;
all three trees are listed in `sdk/.gitignore`. The export listings we
produce alongside the binaries are derived research data and are
tracked.

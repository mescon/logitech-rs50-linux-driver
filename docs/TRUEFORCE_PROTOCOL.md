# Trueforce Protocol Research

> **Status:** Research only — not yet implemented in the driver.
>
> Based on USB capture analysis from [issue #5](https://github.com/mescon/logitech-rs50-linux-driver/issues/5):
> BeamNG.drive gameplay with and without Trueforce enabled, captured by [@SandSeppel](https://github.com/SandSeppel).

## Overview

Trueforce is a high-frequency audio haptic stream that supplements traditional PID force feedback. Rather than low-rate constant-force updates (~50-100 Hz via HID SET_REPORT), Trueforce sends a ~6,500 Hz audio waveform directly to the wheel's DSP, which drives the motor with much higher fidelity.

The protocol runs entirely on **USB Interface 2** (endpoints 0x03 OUT / 0x83 IN), which the driver currently delegates to hid-generic. No HID++ feature activation is required — the game simply starts writing to endpoint 0x03.

## Traffic Comparison

| Metric | Without Trueforce | With Trueforce |
|--------|-------------------|----------------|
| Interface 2 data packets | **0** | **~41,000** (20K each direction) |
| PID constant force updates (intf 1) | **1,295** | **11** |
| Endpoint 0x03 packet rate | idle | ~500 Hz (2ms interval) |
| Effective audio sample rate | N/A | ~6,500 Hz |

When Trueforce is active, traditional PID FFB is used only for initial setup and occasional parameter changes. The high-frequency force data moves entirely to the audio stream.

## HID Descriptor (Interface 2)

```
Usage Page: 0xFFFD (vendor-defined)
Usage:      0xFD01
Report ID:  0x01
Size:       63 bytes IN + 63 bytes OUT (64 bytes total with report ID)
```

## Packet Format

All packets share a common header:

```
byte[0]:    0x01              Report ID
byte[1-3]:  0x00 0x00 0x00    Padding
byte[4]:    COMMAND_TYPE       See command table
byte[5]:    SEQUENCE           Rolling counter (0x00-0xFF), shared across all types
byte[6..]:  PAYLOAD            Type-specific
```

## Command Types (Host → Device, endpoint 0x03)

| Type | Purpose | Count in capture |
|------|---------|-----------------|
| 0x01 | **Audio data stream** (dominant) | 20,531 |
| 0x05 | Parameter initialization (48 floats) | 96 |
| 0x06 | Effect slot configuration | 12 |
| 0x0e | Frequency / sample rate config | 2 |
| 0x07 | Query / handshake | 2 |
| 0x09 | Runtime parameter update | 2 |
| 0x04 | Stop / clear | 1 |
| 0x03 | Start / play | 1 |

## Initialization Sequence

The game sends the following sequence before starting the audio stream. The entire init sequence was observed twice in the capture (possible retry/re-sync mechanism).

### Step 1: Parameter Upload (Type 0x05)

48 parameters (indices 0x00–0x1d and 0x2b–0x3c) are sent as IEEE 754 little-endian floats.

```
Host:   01 00 00 00 05 <seq> <param_id> 00 ...
Device: 01 00 00 00 05 <seq> <param_id> 09 <param_id> <float32_LE> <checksum> ...
```

Key decoded parameters:

| Index | Value | Possible meaning |
|-------|-------|------------------|
| 0x00 | 2.0 | Channel count or mode |
| 0x01 | 1.0 | — |
| 0x02 | 32768.0 | Max amplitude (0x8000) |
| 0x03 | 65535.0 | Max range (0xFFFF) |
| 0x04 | 65535.0 | Max range |
| 0x07 | 5.4054 | Damping coefficient? |
| 0x09 | 0.3 | Gain? |
| 0x0c | 47.1239 (15π) | Angular rate limit? |
| 0x0d | 1.5708 (π/2) | Phase offset? |
| 0x0e | −9.4248 (−3π) | Filter parameter? |
| 0x0f | 9.4248 (3π) | Filter parameter? |
| 0x10 | 13.0 | Samples per packet |
| 0x12 | 4000.0 | Max frequency? |
| 0x14 | 2000.0 | Crossover frequency? |
| 0x15 | 450.0 | — |
| 0x18 | 40.0 | — |
| 0x1d | 4.0 | — |
| 0x33 | 350.0 | Crossover frequency? |

### Step 2: Frequency Configuration (Type 0x0e)

```
01 00 00 00 0e <seq> 00 c0 28 45 00 ...
```

Contains IEEE 754 float: **2700.0** (likely filter cutoff or Nyquist frequency).

### Step 3: Handshake (Type 0x07)

Query/reset command. Device responds with status.

### Step 4: Effect Slot Setup (Type 0x06)

6 effect slots configured (slots 1–6).

### Step 5: Runtime Parameters (Type 0x09)

Contains additional IEEE 754 float values for runtime configuration.

### Step 6: Stop → Start (Types 0x04, 0x03)

Stop clears any previous state, Start begins the audio stream.

### Step 7: Audio Stream Begins (Type 0x01)

~500 Hz packet rate with 13 samples per packet.

## Audio Data Stream (Type 0x01)

```
byte[0-3]:   01 00 00 00           Report header
byte[4]:     01                    Command type
byte[5]:     sequence              Rolling counter
byte[6-7]:   sample_0 (LE16)      Left channel (unsigned, center = 0x8000)
byte[8-9]:   sample_0 (LE16)      Right channel (identical — mono duplicated)
byte[10-11]: 0x040D                Flag word (rarely varies: 0x050D seen)
byte[12-13]: sample_1 L
byte[14-15]: sample_1 R
byte[16-17]: sample_2 L
byte[18-19]: sample_2 R
...
byte[60-61]: sample_12 L
byte[62-63]: sample_12 R
```

- **13 unique samples per packet**, each duplicated as a stereo pair (L=R always, mono)
- **Unsigned 16-bit little-endian**, center at 0x8000 (signed zero)
- Values oscillate with engine/road vibration
- Packet rate: ~500 Hz → **effective sample rate: ~6,500 Hz**
- The RS50 is a single-motor device, so the stereo duplication is expected

## Device Response (Type 0x02, endpoint 0x83 → host)

```
byte[0-3]:   01 00 00 00           Report header
byte[4]:     02                    Response type
byte[5]:     sequence              Echoes command sequence
byte[6-7]:   LE16 value            Motor current or temperature?
byte[8]:     0x03                  Status byte?
byte[9-10]:  wheel_position (LE16) Matches joystick axis data
byte[11-12]: wheel_position2       Slightly delayed (~1 sample behind)
byte[13-16]: 32-bit counter        Timestamp or sample counter
byte[17]:    varying                Checksum-like
byte[18-32]: status/counters
byte[33-63]: zeros
```

Responses arrive at the same ~500 Hz rate, providing real-time wheel position feedback for synchronization.

## PID FFB Commands (Interface 1, for reference)

Traditional FFB uses HID SET_REPORT to interface 1 with two report sizes:

### Report 0x10 (7 bytes)

Format: `[10 FF <cmd> <param1> <param2> <param3>]`

| Command | Description |
|---------|-------------|
| ff 10 | Constant force update |
| ff 00 | Effect stop/reset |
| ff 01 | Effect start |
| ff 0f | Effect create/allocate |
| ff 17 | Set envelope |
| ff 08 | Set condition |
| ff 02 | Set effect type |
| ff 0d | Set periodic |
| ff 0a | Set constant force params |
| ff 09 | Set ramp |

### Report 0x11 (20 bytes)

Extended command format: `[11 FF 10 2e 01 80 00 00 00 00 XX XX ...]`

Used for constant force values with extended precision.

## Implementation Considerations

### What's Needed

1. **Claim interface 2** instead of delegating to hid-generic
2. **Expose a userspace API** for games to push audio data (character device, ALSA device, or similar)
3. **Handle the init sequence** (parameter upload, frequency config, slot setup, start)
4. **Stream audio at 500 Hz** (13 samples per 64-byte packet, 2ms interval)
5. **Read type 0x02 responses** for synchronization / diagnostics

### Open Questions

- How does the Logitech SDK on Windows initiate the Trueforce stream? Does the game call an SDK function that handles the init, or does it push raw audio and the SDK handles framing?
- Can the init parameters be hardcoded (they appear game-independent) or do they vary per game/title?
- What is the flag word at bytes[10-11] (0x040D) — a frame type indicator, audio format descriptor, or something else?
- What are the decoded meanings of the device response fields (type 0x02)?
- Is the init sequence being sent twice intentional (re-sync) or a BeamNG quirk?

### Userspace API Options

| Approach | Pros | Cons |
|----------|------|------|
| Character device (`/dev/rs50_trueforce`) | Simple, direct, low latency | Custom API, no existing tooling |
| ALSA PCM device | Standard audio API, tools work out of the box | Complex kernel implementation, may not map cleanly |
| hidraw passthrough | Zero kernel changes, game writes directly | Requires userspace to handle framing and timing |

### Proton/Wine Integration

On Windows, games use the Logitech Steering Wheel SDK (or newer G SDK) which internally opens the USB interface and pushes audio data. On Linux via Proton, this SDK would need to be redirected to the Linux driver's API. Options include:
- A Wine DLL override that translates SDK calls to the Linux driver API
- Proton patches to intercept the Logitech SDK
- A userspace daemon that bridges the Windows SDK interface to the Linux driver

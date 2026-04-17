# Phase B: Windows G Hub Gap Analysis

**Date:** 2026-04-16
**Scope:** Feature-by-feature inventory of what G Hub does on Windows vs what our Linux driver supports, backed by USB captures.
**Sources:** 33 RS50 captures in `dev/captures/`, 11 G Pro captures in `/tmp/gpro_captures/`, issue #5 Trueforce captures (referenced only — files not available on this host).

Each row uses the columns: **Feature | HID++/wire | Windows behavior | Linux status | Linux impl | Evidence | Gaps/Issues**.
Status values: `OK` (driver matches G Hub), `Partial` (implemented but incomplete/incorrect), `Missing`, `Unknown`.

---

## 1. FFB Gameplay Wire Protocol (Interface 2, endpoint 0x03)

| Feature | HID++/wire | Windows behavior | Linux status | Linux impl | Evidence | Gaps/Issues |
|---|---|---|---|---|---|---|
| FFB command layout | 64-byte report on ep 0x03 OUT, ID 0x01: `01 00 00 00 01 [seq:1] [force:u16LE] [force:u16LE] 00...(54 zeros)` | Byte 4 = 0x01 (cmd type / axis mask). Byte 5 = 8-bit incrementing sequence. Bytes 6-7 AND 8-9 each carry the **same u16 force**, offset-binary (0x8000 = center). | Partial | `rs50_ff_report` struct + `rs50_ff_send_force` | `rs50_ffb_game3.pcapng` @ 50s: `0100000001271f801f80` | **Verify** driver writes force into both u16 slots. **Verify** driver uses incrementing sequence (firmware may dedupe if stuck). |
| Sequence byte | Byte 5 = monotonic u8, wraps 0x00→0xFF | Increments every packet at 1 kHz | Unknown | Presence/counter handling not confirmed | game3: `b4, b5, b6, b7, b8...` sequential | **Gap if constant.** Firmware may dedupe stuck-sequence packets. |
| Packet rate | ~1000 Hz (1 ms interval) during active FFB | 1 kHz always, even at center force | Partial | 500 Hz timer (`RS50_FF_TIMER_INTERVAL_MS = 2`) | 160,451 OUT packets in 160s ≈ 1003/s | **Driver undershoots** G Hub's rate. Firmware tolerates 500 Hz today; unknown whether 1 kHz matters for feel. |
| Effect types sent | Only FF_CONSTANT-equivalent (one u16 per axis) | No FF_SPRING/DAMPER/RAMP/PERIODIC patterns seen; spring/damper handled onboard via feature 0x8133 | OK | Driver implements FF_CONSTANT only | All 160K packets identical shape; only force differs | Confirmed: we don't need other effect types. |
| Keepalive / "refresh" | 32-byte packet `05 07 00 00 00 00 00 FF FF 00...` on ep 0x03 OUT | **Ad-hoc, not periodic.** game3: 3 times at 3.6s, 117.8s, 142.9s. ffb_new: 3.2s, 95.6s, 149.2s. | **Mismatch** | Driver sends every 20s on a timer | `rs50_ffb_game3.pcapng`: 3 refresh packets in 160s | **Gap:** Our 20s cadence is not what G Hub does. Unclear semantics — possibly "open channel" or "motor reset" marker, not a keepalive. |
| FFB init handshake | None observed | No command precedes gameplay FFB. Wheel accepts immediately. | OK | Driver just writes | Cold-start captures have zero ep 0x03 traffic | Confirmed — no handshake needed. |
| FFB stop / disable | None observed | Final packets stream `0080 0080` (center) without explicit disable | OK | Driver sends force=0 on FF_STOP | Last 20 frames of game3 all center | — |
| FFB status IN reports (ep 0x83) | ~1000 Hz IN at 64 bytes, report ID 0x02 | Wheel continuously reports position/pedals/buttons + FFB state | Partial | Driver reads joystick HID via ep 0x81, parsing ep 0x83 report ID 0x02 is unclear | 320,904 ep 0x83 packets in game3 | **Gap:** Any "FFB active / clipping" status bytes we're ignoring? |

## 2. FFB Settings — HID++ (Interface 1)

| Feature | HID++/wire | Windows behavior | Linux status | Linux impl | Evidence | Gaps/Issues |
|---|---|---|---|---|---|---|
| Rotation range (0x8138, idx 0x18) | SET: `10 FF 18 2D [hi lo 00]` (fn2). GET: fn1. INFO: fn0 returns `005a 0a8c 000a` | u16 BE degrees. Info: min 90°, max 2700°, step 10° | OK | `fn_set_range = 0x20` | `2026-01-26_rotation_sweep.pcapng` | — |
| FFB Strength (0x8136, idx 0x16) | SET: `10 FF 16 2D [hi lo 00]` (fn2) | u16 BE, full 0x0001-0xFFFF range. Info `08 00` = 8.0 Nm max, 0.1 Nm step | OK | `fn_set_strength = 0x20` | `ffb_strength_sweep.pcapng` | **Verify** linear mapping (val * 65535) / 100 matches G Hub's specific values (G Hub sends 0x1FFF at 12.5% — may be non-linear). |
| Damping (0x8133, idx 0x14) | SET: `10 FF 14 1D [hi lo 00]` (**fn1!**). INFO: fn0 returns `19 99` | u16 BE. "Default" is 0x1999 (~10%). | OK | `fn_set_damping = 0x10` | `damping_sweep.pcapng` | Driver correctly uses fn1. |
| FFB Filter (0x8140, idx 0x1A) | SET: `10 FF 1A 2D 01 00 [level]` (**fn2**). INFO returns min 1, max 15, step 1 | Manual mode: `[0x01, 0x00, level]`. Level 0x01-0x0F. | **Mismatch** | Driver's `fn_set_filter = 0x30` (fn3) | `ffb_filter_sweep.pcapng`: `10 ff 1a 2d ...` | **Contradiction with earlier TODO fix.** Subagent A read captures as fn2, Subagent Phase A/earlier sessions determined fn3 from G Pro captures. Possible that **G Pro and RS50 differ** on this fn. Re-verify both with a precise fn-byte check. |
| FFB Filter AUTO (0x8140) | SET: `10 FF 1A 2[C/D] 04 00 [level]` (auto ON) vs `00 00 [level]` (auto OFF) | Byte4 = 0x04 auto on, 0x00 auto off. Level preserved in both. | Partial | `wheel_ffb_filter_auto` sysfs | `auto_ffb_filter.pcapng`: `10 ff 1a 2c 04 00 0b` | GET decoder uses 0x05/0x01 (documented earlier) while SET sends 0x04/0x00. Bit 0 in GET may be a separate "dirty" flag. **Verify read/write are consistent.** |
| Brake Force (0x8134, idx 0x15) ONBOARD | SET: `10 FF 15 2D [hi lo 00]` (fn2) | u16 BE 0x0001-0xFFFF (% load cell) | OK | `fn_set_brakeforce = 0x20` | `brake_force_sweep.pcapng` | — |
| **Desktop Sensitivity** | **feature 0x80A4 fn4 — axis-curve upload** (NOT feature 0x8040 fn2 that driver writes) | G Hub uploads a 4-point curve via 88 × `11 ff 0d 4d 03 [x1 y1 x2 y2] ...` long packets per slider change | **MAJOR GAP** | Driver's `wheel_sensitivity_store` writes feature 0x8040 — likely no effect or touches LED brightness instead | `2026-01-30_desktop_sensitivity.pcapng`: 88 × `11 ff 0d 4d`, 0 × `10 ff 0a ...` | **Major gap.** Our `wheel_sensitivity` writes to the wrong feature. To match G Hub we need to upload axis response curves to feature 0x80A4 (see also AxisCalibration gap below). |
| TRUEFORCE sensitivity (0x8139, idx 0x19) | SET: `10 FF 19 3D [hi lo 00]` (**fn3**!). u16 BE. | 0x0001-0xFFFF, u16 BE | OK | `fn_set_trueforce = 0x30`, sysfs `wheel_trueforce` | `trueforce_sweep.pcapng` | **Subagent's table says "GAP — driver has no sysfs"**. Actually we do (`wheel_trueforce`). Likely subagent grepped for `idx_trueforce` too narrowly. Take as OK. |
| Profile query (0x8137, idx 0x17) | `10 FF 17 1D 00 00 00` (fn1 GET) sent **before every setting change** | G Hub bookkeeping | N/A | Driver doesn't auto-query | Every sweep starts with profile GET | Not needed for correctness. |
| Sync feature (0x1BC0, idx 0x09) | `10 FF 09 2D 00 00 00` (short fn2) and `11 FF 09 1D ...` (long fn1 enumerating indices 0x09-0x15) | G Hub sends before/around setting changes — possibly "batch begin/commit" | **Missing** | Not called | All sweep captures have many fn2/fn1 on 0x1BC0 | **Unknown impact.** May affect persistence across power cycles. |
| Read-modify-write pattern | G Hub always queries current value then re-writes ALL wheel settings on each slider tick | Redundant but consistent | N/A | Driver writes directly | Every sweep capture | Not required for correctness; hints that wheel may need redundant writes for persistence. |
| Autocenter | No dedicated HID++ feature observed | Emergent from damping + FFB filter | N/A | Driver doesn't try | No feature ID | **Issue #13 calibration cannot use a dedicated autocenter feature.** Need software emulation. |
| Gain (separate from strength) | None — 0x8136 "FFBStrength" is the master gain | — | N/A | `wheel_strength` is the gain | — | — |
| Unknown fn2 on 0x11 (0x8127) | `11 FF 11 2D 00 00 00...` (16 zeros), response `01 00 64` | Sent after every profile/mode change and every settings cycle | **Missing** | Not sent | `switch_all.log`, `mode_desktop_to_onboard.log` | **Unknown.** Possibly "commit settings" ack. Omission's effect: unknown. |
| Feature 0x0E (0x80D0) fn0 | `10 FF 0E 0D 00 00 00` queried in every post-change refresh | G Hub always queries | **Missing** | Not used | `ghub_init_wheel_on.log` | **Unknown function.** |
| Feature 0x8120 (FFBExtended, idx 0x0F) fn2 | `11 FF 0F 2D 0A 01 02 03 04 05 06 07 08 09 0A` — push axis-ID list | At startup only | Missing | Driver queries but doesn't push | `ghub_startup.log:1347` | Device defaults permissive — works without, but we're diverging from G Hub. |

## 3. Axis Calibration / Response Curves (0x80A4, idx 0x0D)

| Feature | HID++/wire | Windows behavior | Linux status | Linux impl | Evidence | Gaps/Issues |
|---|---|---|---|---|---|---|
| Curve upload (fn4 SET_CURVE) | Long writes: `11 FF 0D 4D 03 [x1 y1 x2 y2] ...` (16 bytes payload per packet) + commit (fn5) | On desktop-profile activation G Hub writes 24+ fn4 long packets (~768 B curve data) defining axis curves, then fn5 commit | **Missing** | Nothing for feature 0x80A4 | `mode_onboard_to_desktop.log:225-345`, `profile_desktop.log:107-193`, `desktop_sensitivity.pcapng` | **Major gap.** Desktop profile on Linux is fundamentally less configurable than Windows. Blocks true "desktop mode" parity AND blocks the sensitivity feature we're supposed to expose. |
| Curve list (fn1) | `10 FF 0D 1D ...` | G Hub queries existing curves at boot | Missing | Not called | Init captures | — |
| Per-axis calibration (fn6) | `11 FF 0D 6D [axis] 00 00` | Side-effect of brake_force sweep; axes 1,2,3 | Missing | Not sent | `brake_force_sweep.pcapng` | — |

## 4. LIGHTSYNC LEDs (0x807A, 0x8071, 0x1BC0)

| Feature | HID++/wire | Windows behavior | Linux status | Linux impl | Evidence | Gaps/Issues |
|---|---|---|---|---|---|---|
| LIGHTSYNC init (enable) | fn7 on 0x0B | Sent once on cold start + on "wheel attached" event | OK | `rs50_lightsync_enable` | `ghub_startup:1339` | — |
| fn0/fn1/fn2 probes | fn0 GET_INFO, fn1 GET_CAPS, fn2 GET_STATE | Probed ONCE at cold start | Over-queries | Driver re-runs every probe | hidpp.c:5467-5489 | Minor waste. Responses ignored (dead code per PROBE.F10). |
| Per-LED color set | Very-long `12 FF 0C 2D [slot] [type] <30B RGB>` | **Always preceded by fn4 SET_NAME** even if name unchanged | **Partial** | Driver writes SET_CONFIG only, no SET_NAME prepend | `lightsync_save.log:5-8`, `desktop_led_colors.log:437-443` | **Gap:** Driver never sends fn4 SET_NAME; G Hub always does. Device may require SET_NAME to "dirty" the slot before SET_CONFIG persists. |
| Slot type byte (params[1]) | `12 FF 0C 2D [slot] [type] ...` | **type varies per slot:** slot0=03, slot1=02, slot2=01, slot3=03, slot4=03/04 | **Partial** | Driver hardcodes `0x03` | `hidpp_settings.txt`; stored slots show `0x01..0x04` | **Gap:** Type byte is slot-specific, not a constant. Meaning of type 0x01/0x02/0x04 vs 0x03 unknown — gradient vs static vs rainbow? |
| Pre-config & commit | `11 FF 0B 6C ...` pre then commit | Done before each change + at end of session | OK | Driver sends both | `lightsync_save.log:1-4` | — |
| Sync "begin" writes | `11 FF 09 1D 01 00 09 00 0D..12, 15 0000...` (7 long writes) | **Only in DESKTOP mode** on LED change; **absent in onboard** | **Missing** | Not implemented | `desktop_led_colors.log:33-67` vs `onboard_led_effect.log` | **Gap:** Driver never sends the 7-write sync sequence. May explain any desktop LED reliability issues. Should gate on `current_mode`. |
| Feature 0x09 fn2 state query | `10 FF 09 2D 00 00 00` | Queried before every LED change (returns zeros) | Minor | Probed once at init | `lightsync.log:27` | Non-critical. |
| Effect set (fn3 on 0x0B) | `10 FF 0B 3D [effect] 00 00` | Per effect change, both modes | OK | `wheel_led_effect` sysfs | `lightsync.log:29` | But: **PROBE.F1** (docs claim range 5-9, code uses 1-5). |
| Brightness (0x8040 fn2) | `10 FF 0A 2D 00 [0-64] 00` | Every LED interaction sends it even if unchanged | OK | `wheel_led_brightness` | `lightsync.log:39` | — |
| Current slot "activate" (fn3 on 0x0C) | `10 FF 0C 3D [slot] 00 00` | Returns the slot NAME (fn3 = GET_NAME). Actual activation is implicit via fn6 commit on 0x0B. | **Misnamed** | Driver calls fn3 expecting ACTIVATE | Captures show fn3 returns name | **Gap:** `RS50_RGB_FN_ACTIVATE` is misnamed. The "activate" call is a no-op; real activation happens via fn6 on 0x0B. Harmless but misleading. |
| fn4 SET_NAME (0x0C) | `11 FF 0C 4D [slot] [len] <name bytes>` | Sent on save, also before every color change | **Missing write-path** | Driver has `wheel_led_slot_name` read-only | `lightsync_save.log:5` | **Gap:** No write-path via fn4. Combined with "Per-LED color" gap above — we never send names, device may refuse to persist. |
| Read-back slots after save | fn1 GET_CONFIG + fn3 GET_NAME for all 5 slots | G Hub always verifies after write | N/A | Driver assumes sysfs is stateful | `lightsync_save.log:29-44` | Informational. But also: **PROBE.F4 gap** — driver never queries stored slot configs on init, so user's G Hub-saved colors get clobbered on next slot switch. |
| Slot count / LED count | GET_INFO returns num_slots=5, num_leds=10 | Runtime discoverable | **Hardcoded** | `RS50_LIGHTSYNC_NUM_SLOTS=5`, `RS50_LIGHTSYNC_NUM_LEDS=10` compile-time | Spec §9.3.2 | **Future-proofing gap.** G Pro may differ. |

## 5. Profiles / Modes (0x8137, idx 0x17)

| Feature | HID++/wire | Windows behavior | Linux status | Linux impl | Evidence | Gaps/Issues |
|---|---|---|---|---|---|---|
| Profile get current | `10 FF 17 1D 00 00 00` → `XX 01 00` | Heavily polled | OK | `wheel_profile` read | All captures | — |
| Profile set | `10 FF 17 2D [idx] 00 00` | Desktop=0, onboard=1..5 | OK | `wheel_profile` write | `mode_desktop_to_onboard.log:17` | — |
| Profile-changed broadcast | `11 02 0E 10 [new_idx] 01 00 00...` | Device pushes after profile change | **Missing** | No interrupt handler for feature 0x0E event | `mode_desktop_to_onboard.log:61` | **Gap:** Driver doesn't observe user-initiated (wheel button) profile changes → stale `current_profile`. |
| Rotation-changed broadcast | `12 FF 18 00 [rot_hi rot_lo] 00...` | Device pushes when range changes (e.g., triggered by profile switch) | **Missing** | No broadcast handler | `mode_onboard_to_desktop` line 23 | **Gap:** Stale `wheel_range`. |
| Re-query after profile change | Full sequence: profile GET, rotation GET, strength GET, damping GET, filter GET, trueforce GET, brake_force GET, LED GET, sensitivity GET | G Hub re-queries EVERYTHING after profile change | **Missing** | Driver never refreshes | `switch_all.log`, `mode_desktop_to_onboard.log:527-725` | **Gap (matches Phase A SYS.F42).** sysfs values become stale until next manual read. |
| Profile names (fn3 on 0x17) | `10 FF 17 3D [0..5] 00 00` | G Hub reads names for all 6 slots at startup | **Missing** | No API exposes names | `ghub_startup.log:1411-1431` | **Gap:** No way to see "AC EVO", "iRF", etc. via driver. |
| Profile info (fn0) | `10 FF 17 0D` → `06 09 02 03 03 03 03 03` | Read at startup | Not used | Discovery only | `ghub_startup.log:1399` | — |

## 6. Initialization

| Feature | HID++/wire | Windows behavior | Linux status | Linux impl | Evidence | Gaps/Issues |
|---|---|---|---|---|---|---|
| Feature enumeration | `10 FF 00 1D` (IRoot) + `10 FF 01 1D [idx]` (IFeatureSet) | Full walk of 37 features | OK | `hidpp_root_get_feature` per-feature | `init_sequence.txt` | — |
| DeviceInfo (0x0003) | fn2 | Reads name, firmware, serial | Partial | Driver discovers but doesn't log serial | `ghub_startup.log:51` | Minor. |
| **Multi-device enumeration** | Commands to device indices 1, 2, and 5 (e.g., `10 02 00 1D`, `10 05 02 0D`) | **G Hub enumerates THREE HID++ sub-devices** over one USB interface: wheel-base (idx 2), idx 5, idx 1 | **Missing** | `hidpp->device_index` is hard-coded to 0xFF | `ghub_startup.log:477,593,1631` | **Gap matches Phase A PROBE.F17.** On G Pro, G Hub only sends `feat=0x00 fn1` (presence probe) to these sub-devices and then never uses them — so possibly safely ignorable on G Pro. **RS50 may be different** — need to re-check. |
| FFB enable before gameplay | NONE — G Hub just streams ep 0x03 | — | OK | — | Init captures have zero ep 0x03 | Confirmed. |
| Minimum init for input/FFB | NONE (`boot_no_ghub.pcapng` has 0 HID++ OUTs) | Joystick input + FFB work with zero HID++ init; wheel pushes broadcasts unsolicited | — | Driver's extensive init isn't required for input/FFB | `boot_no_ghub.log` | **Observation:** Much of our init exists to expose sysfs knobs, not to get the wheel working. A "minimal mode" is possible. |
| Background polling | `10 FF 17 1C` (profile), `10 FF 0F 1C` (FFB state) at ~10 Hz | During active session | Different cadence | Driver has `refresh_work` but semantics differ | `ghub_init_wheel_on tail` | — |
| Temperature polling | None observed for RS50 | No temp/status polling | — | — | — | — |

## 7. Wheel/Pedal Input (Interface 0)

| Feature | HID++/wire | Windows behavior | Linux status | Linux impl | Evidence | Gaps/Issues |
|---|---|---|---|---|---|---|
| **30-byte report layout (RS50)** | byte[0] low nibble=D-pad (bits 0-2 = 8 directions, bit 3 = released), high nibble=btn 1-4; byte[1]=btn 5-12; byte[2]=reserved 0x00; byte[3]=btn 13-20; bytes[4-5]=wheel X LE (center 0x8000); bytes[6-7]=throttle LE; bytes[8-9]=brake LE; bytes[10-11]=clutch LE; bytes[12-19]=0x00; byte[20]=small rotary (0-2); bytes[21-29]=constant/zero | Decoded from `wheel_input`, `pedal_throttle`, `pedal_brake`, `button_mapping` captures | OK | `rs50_process_pedals` offsets 4/6/8/10 match | Confirmed in all input captures | **PROBE.F3 open question:** D-pad diagonals (0x01/0x03/0x05/0x07) — spec says 4-way, code decodes 8-way. Need hold-diagonal capture. |
| Input report rate | Fixed 1 kHz (1.000-1.002 ms) | Not activity-dependent | OK | — | 29,166 samples with 1000-1050 µs intervals | — |
| Physical button count | ~19-20 real buttons (byte[0] high nibble + byte[1] + byte[3]) | Bit 0x01 of byte[3] never set | OK | `RS50_MAX_BUTTON_USAGE = 0x50` filters phantom | `button_mapping.pcapng` | — |
| **G Pro report layout** | **Unknown** (USBPcap captures don't include interrupt IN traffic) | — | Unknown | Driver shares RS50 decoder | — | **Gap:** Need Linux-side capture on G Pro to verify layout + button count. HID descriptor size is 205 B (RS50 is 141 B) — materially different. |
| G Pro phantom button threshold | Unknown (same reason) | — | Unknown | `rs50_input_mapping` threshold 0x50 applied | — | **Gap matches Phase A PROBE.F8.** |

## 8. Trueforce Audio Stream (RS50) — verified from issue #5 captures

Analyzed from `/tmp/trueforce_captures/woTF.pcapng` (baseline, no TF) and `/tmp/trueforce_captures/wTF_starts111132.pcapng` (BeamNG with TF, 20.68s gameplay, 20,647 OUT packets). Prior research in `docs/TRUEFORCE_PROTOCOL.md` has been corrected in several places; findings below are the ground truth.

| Feature | HID++/wire | Windows behavior | Linux status | Linux impl | Evidence | Gaps/Issues |
|---|---|---|---|---|---|---|
| Stream transport | Interface 2 ep 0x03 OUT + 0x83 IN, 64-byte packets | **1000 Hz** (1 ms/packet) — not 500 Hz as prior doc claimed | **Missing** | Interface 2 delegated to hid-generic | 19,000 packets in 18.999 s | Driver must maintain 1 ms URB cadence. |
| Sample format | LE 16-bit offset-binary, center 0x8000 | Verified with mid-stream packet decode showing monotonic descent through 0x8000 zero-crossing | Missing | — | Packet seq 0x96: `2480 2480 040d 4e80 4e80 3f80 3f80 2f80 2f80 1c80 1c80 0a80 0a80 f97f ... aa7f` | Signed-vs-unsigned ambiguity in prior doc resolved: offset-binary u16. |
| Samples per packet | 13 samples, but **only 4 are new each packet**; 9 overlap with previous (13-sample sliding window advancing by 4) | Effective sample rate = 4 × 1000 Hz = **4000 Hz** (not 6500 Hz as prior doc claimed) | Missing | — | Verified 9,898/9,898 shift-4 matches across consecutive flag=0x040d packets | Major correction to prior doc. |
| Amplitude during gameplay | Observed range 0x7DF5..0x81F5 (±523 from center in offset-binary) | ~1.5% of full scale — TF is small high-frequency vibration on top of PID baseline, not full-amplitude audio | Missing | — | 258,687 samples analyzed | — |
| Mono/stereo | L == R byte-for-byte across all 266,903 pairs | Confirmed mono, duplicated as stereo pair | Missing | — | — | Single-motor device. |
| Flag word (bytes 10-11) | 99.62% `0x040D`, occasional `0x050D`/`0x0000`/others only in first ~200 ms (warmup) | Steady-state is uniformly `0x040D` | Missing | — | 20,452 of 20,531 audio packets | Prior doc listed it as "usually 0x040D" — now verified. |
| Init — parameter query (type 0x05) | **Type 0x05 sent host→device carries only param_id (1 byte)**; float value is returned in device response on ep 0x83 (bytes 9-12) | **Direction corrected** — prior doc implied floats flow host→device | Missing | — | Init frames 96617-96825 | Prior doc wrong about direction of float data. |
| Init parameter set | 48 IDs: `{0x00..0x1D} ∪ {0x2B..0x3C} ∪ {0x30}` = 48 distinct values | All values canonical math constants (π/2, ±3π, 15π) or small fractions (0.014, 0.028, 0.30, 0.74, 0.97, 200/37 = 5.4054) — **device/firmware-defined, NOT game-tunable** | Missing | — | Both init batches in wTF capture have identical values | Driver can hardcode all 48. |
| Init — frequency config (type 0x0e) | `00 c0 28 45` = float32 LE 2700.0 | 2700 matches observed ~1080° × 2.5 wheel resolution | Missing | — | Seq 0x32, frame 96833 | — |
| Init — handshake (type 0x07) | Empty payload | — | Missing | — | Seq 0x34 | — |
| Init — effect slot setup (type 0x06) | **Slots 1-3 use 2-byte payload** (`0101`, `0102`, `0103`); **slots 4-6 use 3-byte payload** (`010401`, `010501`, `010601`) | **Correction to prior doc** — asymmetry between first and second triplet | Missing | — | Seq 0x36, 0x38, 0x3A, 0x3E, 0x40, 0x42 | — |
| Init — runtime params (type 0x09) | Payload: `02 02 00 00 80 3f 00 00 af 43` = two u8 (`02 02`) + float32 `1.0` + float32 `350.0` | — | Missing | — | Seq 0x3c, frame 96883 | **Correction:** Prior doc just said "runtime parameters" without layout. |
| Init — stop/start (types 0x04 / 0x03) | Empty payloads; 0x04 clears, 0x03 begins | Both empty | Missing | — | Seq 0x43, 0x44 | — |
| Init runs twice | **First init ~ms 0-93 of stream; second init ~ms 140-234** while audio already flowing. Identical content. | **Not a retry** — deliberate re-init with audio continuing. Purpose unclear; possibly BeamNG-specific. | Missing | — | Frames 96617+, 96953+ | Driver probably can skip the second batch. |
| Audio stream begins | Type 0x01 starts at seq 0x31, between param-batch end (seq 0x30) and frequency-config (seq 0x32) | Audio streams silence (0x8000) during the init "kickoff" sequence; the `0x03` Start effectively unmutes already-flowing data | Missing | — | Frame 96829 | — |
| Sequence counter | Single counter shared across all types on ep 0x03. Rolls continuously from 0x01. | u8 mod 256 | Missing | — | Seq 0x30 → 0x31 → 0x32 observed | Not reset between init phases. |
| Response (ep 0x83) rate | 1.12 ms average interval (20,554 responses in 23 s) | Slightly offset from command cadence; device emits more than one per ms occasionally | Missing | — | — | Not a 1:1 bidirectional sync. |
| Response (type 0x02) sequence echo | **Response seq = command seq − 1** (verified in 1,995 of 2,000 cmd/resp pairs) | **Prior doc's "echoes sequence exactly" is wrong** — responses lag by 1 | Missing | — | — | — |
| Response layout (type 0x02) | Bytes 0-3 header `01 00 00 00`; b4=0x02; b5=seq(echoed lagging); **b6-7 signed LE16 motor sense**; b8=0x03 const; **b9-10 wheel pos #1** (u16 offset-binary); **b11-12 wheel pos #2** (usually == #1, differs by ±1 — likely next-sample pre-read); **b13-16 LE32 microsecond counter** (increments 1000/ms); b17 pseudo-random (LRC/CRC?); b18-21 4-byte status word (two dominant patterns `0000 8027` and `FFFF 7F27`); b22-29 **8-byte device constant `00 80 07 00 00 1D 95 00`**; b30-32 `00 1D 95`; b33 pseudo-random; b34-63 zeros | — | Missing | — | 19,900 mid-stream packets | **Correction:** Prior doc's 32-bit counter at bytes 13-16 is actually a **µs timer** (increments by 1,000,000/sec, verified over 20s). Bytes 22-29 are a device/firmware constant, not counters. |
| Graceful stop | **None — capture ends mid-stream** with normal audio flowing. No type-0x04 Stop. No trailing response. | — | Missing | — | Final frame 211520 t=117.04s | Protocol doesn't require shutdown; driver can just stop submitting URBs. |
| Init params game-specific? | **No — device-hardcoded.** Both init batches have identical values. All values are canonical math constants or small fractions. | — | — | — | — | Driver can hardcode all 48 values; no per-game SDK lookup needed. |
| Keepalive / heartbeat | **None — 100% of ep 0x03 traffic during steady-state is type 0x01 audio** | Measured over 10s window, all 10,000 packets are type 0x01 | — | — | — | No alternative cadence, no optional path — driver must sustain 1 ms audio submissions. |
| Trueforce "activation" on HID++ | **None — no HID++ feature SET needed.** Feature 0x8139 appears in device feature table in BOTH woTF and wTF captures. Neither capture writes a setter to 0x8139 during TF activation. | TF is activated purely by writing to ep 0x03 | — | Driver's `wheel_trueforce` sensitivity is a separate firmware setting, not an enable | — | **Correction to prior doc.** TF is not gated by a HID++ command. |
| PID FFB traffic with TF enabled | Const-force cmd 0x10 (7-byte) drops from 1,295 (woTF) to 11 (wTF). Extended const-force cmd 0x10 (20-byte) drops from 1,295 to 11 too. Effect-allocate/envelope/etc unchanged. | The game hands off high-frequency force to TF audio; PID is now used only for slow DC force (centering, bumps). | — | — | — | Validates the design assumption: FF_CONSTANT still needed for low-freq, TF audio for high-freq. |

## 9. G Pro-Specific Protocol Deltas

| Feature | HID++/wire | Windows behavior | Linux status | Linux impl | Evidence | Gaps/Issues |
|---|---|---|---|---|---|---|
| FFB gameplay | HID++ **feature 0x8123 on resolved index 0x09**: fn6 long (9,686 updates in gameplay capture) + matching fn2 short ack | Uses G920/G923-style HID++ FFB stream (NOT ep 0x03 like RS50) | OK | Driver routes G Pro through G920 FFB path via `HIDPP_QUIRK_CLASS_G920` | `ffb_gameplay_gpro_pc.pcapng` | Confirmed. |
| FFB settings SET functions | Damping fn1, strength fn2, range fn2, Trueforce fn3, filter **fn2** (the G Pro capture actually shows fn2) | — | Mixed | Driver: damping=fn1, strength=fn2, range=fn2, trueforce=fn3, **filter=fn3** | Captures | **Possible mismatch on filter** — Phase A had us at fn3 (from an earlier session fix); subagent B1 and subagent B3 both read fn2 from captures. **Needs fresh hardware test.** |
| Sub-devices 0x01, 0x02, 0x05 | G Hub sends `feat=0x00 fn1 00 00` (root::ping) only — **no functional commands** | Presence probe only; never addressed for settings or FFB | OK-to-skip | Driver uses device_index 0xFF always | All G Pro captures | **Phase A PROBE.F17 partially debunked:** G Hub itself doesn't functionally use these on G Pro. May be legacy receiver-style slots. Can safely stay un-addressed for now. |
| G923 compat mode | Switch via feature 0x06 fn1 long writes. PID likely changes from 0xc272 to 0xc26d on re-enumeration. Reduced feature set (only 0x00-0x0E touched) | — | Unknown | Driver unaware of mode switch | `cold_boot_gpro_to_g923.pcapng` | **Unknown behavior** if user initiates switch from hardware. Driver likely continues with stale state. |
| G Pro Trueforce audio transport | **Unknown.** Interface 2 descriptor is 94 B vs RS50's 30 B (3× larger). No ep 0x03/0x83 traffic in any G Pro capture available. | — | Unknown | Driver delegates iface 2 to hid-generic | `ffb_gameplay_gpro_pc.pcapng` has zero iface 2 traffic | **Major unknown.** G Pro's Trueforce transport may be completely different. Our TRUEFORCE_PROTOCOL.md findings may not apply to G Pro. |
| G Pro HID descriptor | Interface 0 = 205 B (RS50: 141), Interface 1 = 148 B (RS50: 84), Interface 2 = 94 B (RS50: 30) | — | Unknown | Driver assumes RS50 layout | G Pro USB descriptor responses | **Unknown:** G Pro input may have extra axes (dual clutch paddles?) or reordered fields. |
| G Pro LIGHTSYNC | Feature 0x15 fn3 enumerates slots 0-5 in captures — different resolved index than RS50 | Same feature pages, different runtime indices | OK | `idx_lightsync` resolved dynamically via page ID | All G Pro captures | — |

## 10. Windows SDK surface (for Wine bridge design)

The repo contains the **legacy Logitech Gaming Steering Wheel SDK (2011-2015)** at `sdk/Include/LogitechSteeringWheelLib.h`. Models listed max out at G920 / G29 — no RS50, G Pro, or Trueforce APIs. This SDK is what older games (pre-2019 sims) call into on Windows.

**The newer "Logitech G SDK"** (circa 2019-2020) is what Trueforce-aware games like BeamNG.drive call into. We don't have a copy of that SDK's headers. Capturing which DLL a Trueforce-aware game loads on Windows (via VM passthrough) would tell us the DLL name and method table to shim.

| SDK API group | Function(s) | Matches Linux feature | Wine bridge action |
|---|---|---|---|
| Init | `LogiSteeringInitialize(ignoreXInput)`, `LogiUpdate()`, `LogiSteeringShutdown()` | No direct mapping — bridge maintains its own state | Shim opens evdev device, maintains state |
| Device discovery | `LogiIsConnected(idx)`, `LogiIsModelConnected(idx, model)`, `LogiGetFriendlyProductName(...)`, `LogiGetDevicePath(...)`, up to 4 controllers | Map Linux /dev/input/by-id/*Logitech*event-joystick to virtual index | Shim enumerates evdev devices, maps to controller index |
| Buttons / input | `LogiButtonTriggered/Released/IsPressed(idx, btn)`, `LogiGetState(idx)` returning DIJOYSTATE2 | Linux evdev input | Shim reads evdev, translates to DIJOYSTATE2 |
| Constant force | `LogiPlayConstantForce(idx, magPct)`, `LogiStopConstantForce(idx)`, `LogiIsPlaying(idx, LOGI_FORCE_CONSTANT)` | Linux FF_CONSTANT upload/playback | Shim calls `EVIOCSFF` |
| Spring | `LogiPlaySpringForce(idx, offset, sat, coef)`, `LogiStopSpringForce(idx)` | FF_SPRING, or feature 0x8133 damping | Shim uploads FF_SPRING; driver may need to implement |
| Damper | `LogiPlayDamperForce(idx, coef)`, `LogiStopDamperForce(idx)` | FF_DAMPER, or feature 0x8133 | — |
| Periodic surface | `LogiPlaySurfaceEffect(idx, type, mag, period)` with SINE/SQUARE/TRIANGLE | FF_PERIODIC | — |
| Named effects | `LogiPlaySideCollisionForce`, `LogiPlayFrontalCollisionForce`, `LogiPlayDirtRoadEffect`, `LogiPlayBumpyRoadEffect`, `LogiPlaySlipperyRoadEffect`, `LogiPlayCarAirborne`, `LogiPlaySoftstopForce` | No direct Linux FF equivalent — these are "canned" effects baked into the SDK | Shim synthesizes from constant+periodic+ramp combinations |
| Range | `LogiSetOperatingRange(idx, range)`, `LogiGetOperatingRange(idx, &range)`, `LogiSetOperatingRangeDInput`, `LogiGetOperatingRangeDInput` | Write to `wheel_range` sysfs | — |
| Properties | `LogiSetPreferredControllerProperties({forceEnable, overallGain, springGain, damperGain, defaultSpringEnabled, defaultSpringGain, combinePedals, wheelRange, ...})` | Maps to multiple sysfs attrs | — |
| LEDs | `LogiPlayLeds(idx, currentRPM, rpmFirstLed, redLine)`, `LogiPlayLedsDInput(...)` | Write to `wheel_led_*` sysfs | Shim picks appropriate LED slot/effect |
| Shifter | `LogiGetShifterMode(idx)` returning gated/sequential | No direct mapping (we don't expose this) | — |

**Implications for Phase C Trueforce design:**
- The Wine bridge is actually **two DLL overrides**: one for the legacy Logitech SDK (above API), one for the newer G SDK (Trueforce + whatever else was added).
- Most of the legacy API maps trivially to evdev + sysfs. The "named effects" (DirtRoad, BumpyRoad, SideCollision, etc.) have no Linux equivalent — they're SDK-synthesized combinations of primitive forces. The shim owns this mapping.
- The `LogiControllerPropertiesData` struct is a clean model for exposing wheel state via sysfs. Worth checking our attribute coverage against it.

**TODO in Phase C:** identify the newer G SDK's DLL name (via VM+Trueforce-game capture) and sketch its API surface.

## 11. Wheel Calibration (Issue #13)

| Feature | HID++/wire | Windows behavior | Linux status | Linux impl | Evidence | Gaps/Issues |
|---|---|---|---|---|---|---|
| Calibration command | **No dedicated command observed in any capture** | Rotation range change may implicitly re-zero mechanically | Missing | — | All 43+ captures checked | **Open question.** Need a dedicated capture of the "Calibrate" button in G Hub UI. Candidates to try: feat 0x02/0x03 reset, feat page 0x8130. |

---

## Cross-Cutting Themes

1. **We're missing G Hub's whole-wheel re-configuration pattern.** G Hub always (a) queries current state before any change, (b) after profile switches re-queries everything, (c) after LED changes on desktop sends a 7-write sync sequence, and (d) sends `0x11 fn2` (16 zeros, response `01 00 64`) after every change as an apparent commit/ack. Our driver does atomic one-shot writes and never refreshes. See items in §2, §4, §5.

2. **Desktop mode on Linux is fundamentally incomplete.** Two big things are Windows-only:
   - **Axis calibration curves (feature 0x80A4)** — G Hub uploads 4-point Bezier curves for steering and pedal response.
   - **Desktop LED sync** — the 7-write sequence on feature 0x09 only fires in desktop mode.
   Our `wheel_sensitivity` sysfs is writing to the wrong feature entirely.

3. **Device-pushed broadcasts are ignored.** Feature 0x0E profile-change and `0x12 FF 18 00 ...` rotation-change events come unsolicited. Driver doesn't subscribe. Matches Phase A SYS.F42.

4. **LIGHTSYNC has multiple silent gaps** — no SET_NAME before SET_CONFIG, hardcoded slot type byte, no reading of saved slot configs on init (PROBE.F4), no desktop sync writes, and fn3 on 0x0C is misnamed.

5. **G Pro looks simpler than feared.** Sub-devices aren't functionally addressed by G Hub either. FFB uses the G920 HID++ path we already have. Main open question: Trueforce transport (interface 2 is different from RS50).

6. **Trueforce audio for the RS50 is well-documented but unimplemented.** For the G Pro, the transport is an open question.

7. **Three contradictions to resolve before building roadmap:**
   - **FFB filter SET function** — captures analyzed this session show fn2 for RS50; prior TODO fix set fn3 from G Pro analysis. Need one authoritative answer per device.
   - **D-pad 4-way vs 8-way** — code claims 8, spec claims 4. PROBE.F3 open.
   - **Multi-device enumeration on RS50 vs G Pro** — subagent B2 saw 3 sub-devices on RS50 ghub_startup captures; subagent B3 saw sub-devices on G Pro are presence-probed but unused. Need to re-verify RS50's are unused too.

---

## Surprises

- **FFB filter SET appears to be fn2, not fn3** (at least on RS50). An earlier session moved it to fn3 based on G Pro captures. This may be device-dependent.
- **Desktop sensitivity is axis-curves, not a simple slider.** Our current sysfs is likely inert.
- **No autocenter feature exists.** Calibration can't use a dedicated HID++ command.
- **G Hub's refresh packet is NOT periodic.** It fires 2-3 times in a 160-second session, at irregular intervals. Our 20-second periodic keepalive is misaligned.
- **FFB packets duplicate the u16 force into two slots.** Unusual. Driver should write both.
- **G Hub always re-writes ALL settings on every slider tick.** Suggests wheel firmware may require redundant writes to persist individual settings.
- **Every setting change triggers a `11 FF 11 2D <16 zeros>` "commit" call** we're not making.
- **Device emits spontaneous profile/rotation change broadcasts** we're ignoring.
- **fn3 on 0x0C is GET_NAME, not ACTIVATE.** Driver's naming is wrong; the "activate" call is a no-op.
- **LIGHTSYNC always sends SET_NAME before SET_CONFIG** even when the name is unchanged. Possibly required by firmware for persistence.
- **LIGHTSYNC slot `type` byte varies per slot** (0x01-0x04), driver hardcodes 0x03.
- **The whole wheel works with ZERO HID++ init.** Cold boot without G Hub still produces input and broadcasts. Our extensive init exists only to populate sysfs.
- **On G Pro, G Hub probes sub-devices 1/2/5 at boot but never sends functional commands to them.** Confirms we can skip these for now.

## Unknowns (Requiring Fresh Captures or Research)

1. Byte 4 (=`0x01`) of FFB OUT — is it cmd type, axis mask, or channel ID? Never observed non-0x01.
2. Redundant second u16 force slot (bytes 8-9) — required mirror or ignorable?
3. `05 07 00 00 00 00 00 FF FF 00...` refresh packet semantics — not periodic. Not a keepalive. Possibly "open channel".
4. Feature 0x1BC0 (idx 0x09) sync semantics — does its absence affect persistence?
5. ep 0x83 IN report ID 0x02 full byte-level decode — any FFB clipping/active flags?
6. Auto FFB filter "level" byte when auto=ON — is it a seed or firmware-overridden?
7. Feature 0x11 (0x8127) fn2 16-zero "commit" — required, optional, or hinting?
8. Feature 0x0E (0x80D0) fn0 — always queried, never SET. What is it?
9. LIGHTSYNC slot type byte meanings (0x01/0x02/0x03/0x04) — rainbow/gradient/static?
10. Wheel calibration command — does one exist? Needs dedicated G Hub "Calibrate" button capture.
11. G Pro input report layout — needs Linux-side capture.
12. G Pro Trueforce audio transport (interface 2 is 94 B, different from RS50's 30 B) — needs dedicated capture.
13. Trueforce init parameters game-independent? — second-game capture needed.
14. D-pad diagonals encoding (4-way vs 8-way) — PROBE.F3 capture needed.
15. FFB filter SET function number — fn2 vs fn3, RS50 vs G Pro — needs authoritative verification.

---

## Status Summary

Items by status:

- **OK:** FFB constant/sequence (wire protocol), rotation, strength, damping, brake force, Trueforce sensitivity, LIGHTSYNC enable, brightness, effect set, profile get/set, feature enumeration, G Pro FFB path, G Pro LIGHTSYNC feature resolution
- **Partial:** FFB packet rate (500 Hz vs 1 kHz), keepalive cadence (20s vs ad-hoc), FFB filter auto encoding inconsistency, FFB status IN parsing, LIGHTSYNC SET_NAME, LIGHTSYNC slot type byte, Multi-device enumeration
- **Missing:** Desktop sensitivity (axis curves), Response curve upload (0x80A4 fn4/5), LIGHTSYNC desktop sync (7-write sequence), LIGHTSYNC SET_NAME write path, Profile-changed broadcast, Rotation-changed broadcast, Re-query after profile change, Profile names, Feature 0x8127 "commit", Trueforce audio stream
- **Unknown:** G Pro input layout, G Pro Trueforce transport, G Pro phantom button threshold, D-pad diagonals, wheel calibration command, Feature 0x0E purpose, numerous protocol details (see Unknowns list)

**Ratio of feature parity:** Rough estimate — roughly 60% of G Hub's feature surface is implemented. The missing 40% is concentrated in:
- Desktop profile completeness (sensitivity curves, LED sync)
- Dynamic refresh after profile/mode changes
- Trueforce audio
- G Pro deep support (input decoder, sub-device probing)
- LIGHTSYNC persistence details (SET_NAME, slot type byte)
- Wheel calibration

**Next:** Phase C synthesizes findings from Phase A (94 audit findings) and this gap analysis into a prioritized roadmap + full-stack Trueforce design.

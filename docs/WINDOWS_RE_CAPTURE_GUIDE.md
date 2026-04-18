# Windows Reverse-Engineering Capture Guide

**Purpose:** Capture specific USB exchanges and artifacts on bare-metal Windows that answer open questions from `docs/plans/2026-04-16-windows-gap-analysis.md`. The resulting pcapng files and artifact dumps feed into Phase C of the ultrareview.

**Companion batch scripts:**
- `dev/windows_re.bat` — maintainer-local, no-game captures (settings, calibration, D-pad, profile switches, LIGHTSYNC). Lives under `dev/` (gitignored).
- `tools/windows_tf_captures.bat` — tracked, community-facing. Covers the four game-requiring Trueforce captures. Runs against BeamNG.drive and/or the Assetto Corsa family.

---

## Background — why these captures exist

The Phase B gap analysis compared our Linux driver against Windows G Hub's behavior, using existing captures in `dev/captures/` plus the Trueforce captures from GitHub issue #5. That analysis produced:

- **15 open unknowns** requiring fresh captures or hardware access
- **3 protocol contradictions** (FFB filter fn byte, D-pad 4-vs-8-way, sub-device usage)
- An unclear picture of how the Trueforce audio SDK reaches USB from Windows games

Since Phase B, two material discoveries changed the scope:

1. **The Windows Trueforce SDK DLL is now in-repo** at `sdk/trueforce_1_3_11/trueforce_sdk_{x64,x86}.dll`. Its exports are in `sdk/trueforce_1_3_11/exports_x64.txt` (75 exports). This gives us the complete C API surface without needing to capture games linking against it.
2. **The newer public Wheel SDK DLL is also in-repo** at `sdk/wheel_9_1_0/` — a post-2015 evolution of the public Steering Wheel SDK.

Because of (1), several RE targets originally required to reverse-engineer the TF SDK from games are no longer needed. This guide focuses on what the DLLs can't tell us.

---

## Prerequisites

- Windows machine with the RS50 attached.
- Logitech G HUB installed and working (wheel shows up, settings apply).
- Wireshark with USBPcap installed (Wireshark installer offers USBPcap).
- The repo cloned / mapped locally. The batch script expects `H:\Projects\logitech-rs50-linux-driver\dev\captures\` (our Windows-guest view of the Linux host's shared folder). Edit `CAPTURE_DIR` at the top of the script if your layout differs.
- (Optional) Microsoft Sysinternals Process Monitor (ProcMon) for the DLL survey capture.
- Run CMD as Administrator — USBPcap + tshark need it.

---

## Capture categories

Categories A–D need a running game (BeamNG / Assetto Corsa). Community contributors are welcome to run them via `tools/windows_tf_captures.bat` — see "Contributing Trueforce captures" below. Categories E–K need only the wheel and G Hub and live in the maintainer-local `dev/windows_re.bat`.

### A. Trueforce SDK calls in practice

With the SDK DLL in hand we know the API surface. What we still don't know:

- **What IPC does the SDK use** to talk to G HUB Agent? Named pipe? Local socket? Shared memory?
- **Do games call `SetStreamTF` (bulk audio upload) or per-sample `SetTorqueTF*` (rate-driven)?** This determines our Linux kernel API shape.
- **Does G HUB Agent keep the TF audio stream running when the game is suspended?**

These are answered by:

- Running **ProcMon** while a TF-enabled game launches → identify which DLLs are loaded and where (confirms `trueforce_sdk_x64.dll` is in fact used) and what named pipes / files / registry paths the SDK opens.
- **Optional:** Running API Monitor (or a hook) to log actual SDK calls.

The pure tshark captures from Phase B already tell us the USB wire side. Combined with the DLL exports and the ProcMon trace we have the full stack.

**Batch commands:** `windows_tf_captures.bat dll-survey`  *(game-required)*

### B. Game-independence of Trueforce init parameters

Phase B established that the 48 init parameters on ep 0x03 look device-hardcoded (canonical math constants and small fractions, identical across both init batches within a single BeamNG session). To confirm they truly don't vary per game, capture a second game:

**Batch commands:** `windows_tf_captures.bat tf-ace`  *(game-required)*

If the 48 values differ between BeamNG and AC(E) / AC Evo / AC Competizione, they're game-tunable. If identical, we can safely hardcode them in the Linux driver.

### C. Trueforce graceful shutdown

The issue #5 BeamNG capture ends mid-stream with no shutdown command. Was the capture truncated, or does the SDK genuinely not send a stop? Running a game and deliberately ALT-F4'ing it while capturing resolves this.

**Batch commands:** `windows_tf_captures.bat tf-abort`  *(game-required)*

Also useful: does the wheel hold the last force after the host stops talking, or does it return to center by itself? Observe physically during the capture.

### D. Full BeamNG session (gold standard)

A complete "cold start → menus → on track → in-game exit → process ends" capture gives us a reference we can diff against anything future. Not strictly blocking, but cheap to take.

**Batch commands:** `windows_tf_captures.bat tf-beamng`  *(game-required)*

### E. Wheel calibration routine (issue #13)

None of our captures exercise G Hub's Calibrate button. That routine may use a dedicated HID++ feature we haven't seen. If it does, issue #13 becomes trivial to implement; if it doesn't, we know calibration must be emulated in software.

**Batch commands:** `windows_re.bat calibrate`

### F. FFB filter SET function byte

Phase B surfaced a contradiction: our earlier session change set the G Pro's FFB filter SET to fn3, but the Phase B subagent read fn2 from the RS50 capture. Resolve by making a fresh, deliberate RS50 capture where you move the slider through specific values and toggle auto twice. We then decode the fn byte explicitly.

**Batch commands:** `windows_re.bat filter-fn`

### G. D-pad 8-way vs 4-way

The driver decodes 8 D-pad directions (diagonals included) but the protocol spec Section 3 lists only 4. Press all 8 directions with clear 1-second holds between each, then decode the byte[0] bits.

**Batch commands:** `windows_re.bat dpad-8way`

### H. Feature 0x8127 "commit"

After every settings change, G Hub sends `11 FF 11 2D <16 zeros>` to feature 0x8127. We don't. This capture isolates that command so we can document its exact position in the sequence (before or after the SET?) and its response.

**Batch commands:** `windows_re.bat commit-0x8127`

### I. Settings persistence test

Does the device persist settings across power cycles without the 0x8127 "commit"? Two-step test:

1. Set damping to a distinctive value (e.g. 47%), unplug the wheel, capture.
2. Replug the wheel, read back damping via G Hub, capture.

If the value survives the unplug, 0x8127 is not required for persistence. If it reverts, 0x8127 likely is the commit. Either way, we learn something actionable.

**Batch commands:** `windows_re.bat persist-unplug` then `windows_re.bat persist-readback`

### J. Profile switch via hardware button

When the user switches profiles via a wheel-mounted button (not G Hub), the device pushes an unsolicited notification on interface 1. Our driver doesn't listen for these, which causes stale `wheel_profile` state. Capture the broadcast format:

**Batch commands:** `windows_re.bat profile-hw`

### K. LIGHTSYNC slot type byte

The RS50's LIGHTSYNC `SET_CONFIG` has a type byte that G Hub sets to 0x01, 0x02, 0x03, or 0x04 depending on effect type. Our driver always sends 0x03. Configure each of the 5 custom slots to a visually different effect and capture the sequence — we can then match type bytes to visible effects.

**Batch commands:** `windows_re.bat ls-types`

---

## Running the captures

Both scripts run from an Administrator CMD, both write to `dev\captures\`, and both use the same interactive step-by-step pattern (the script prints instructions, you perform them on the device, press ENTER to advance). Captures are written to `dev/captures/<date>_re_<name>.pcapng`.

### No-game captures (maintainer)

```cmd
cd H:\Projects\logitech-rs50-linux-driver\dev
windows_re.bat help
windows_re.bat all          :: runs the full no-game battery (~20-30 min)
```

Recommended order for the no-game subset:

1. `calibrate` — issue #13. Low effort, potentially high reward.
2. `filter-fn` — resolves fn-byte contradiction.
3. `dpad-8way` — resolves 4-way/8-way contradiction.
4. `commit-0x8127` — captures the "commit" sequence we don't send.
5. `persist-unplug` followed by `persist-readback`.
6. `profile-hw` — broadcast capture.
7. `ls-types` — LIGHTSYNC type byte.

### Game-required captures (community contributors welcome)

```cmd
cd %REPO%\tools
windows_tf_captures.bat help
windows_tf_captures.bat all   :: runs all four game captures (~30-45 min)
```

Recommended order:

1. `dll-survey` — ProcMon-based, records the DLL load sequence for BeamNG and AC. Do this before any game captures.
2. `tf-beamng` — full TF session, the reference capture for Phase C Trueforce design.
3. `tf-ace` — second game for game-independence verification.
4. `tf-abort` — graceful shutdown test.

See "Contributing Trueforce captures" below for how to share the resulting files.

---

## What happens after capture

1. Reboot into Linux.
2. From the repo root: `git status` should show new files in `dev/captures/<date>_re_*.pcapng`.
3. `git add dev/captures && git commit -m "Windows RE captures YYYY-MM-DD"`.
4. Share the commit SHA with Claude — Phase C analysis will decode the captures and update the gap analysis / roadmap accordingly.

---

## Contributing Trueforce captures (community)

The four categories in `tools/windows_tf_captures.bat` (`dll-survey`, `tf-beamng`, `tf-ace`, `tf-abort`) need somebody with a Windows gaming rig plus at least one Trueforce-enabled game. The project maintainer does not currently own BeamNG or the AC family — a contribution here directly unblocks Phase C Trueforce work.

What you need:

- A Windows box with the RS50 wheel attached and working in G Hub.
- Wireshark with the USBPcap component enabled (install with the default Wireshark setup).
- Optional for `dll-survey`: Sysinternals ProcMon (<https://learn.microsoft.com/sysinternals/downloads/procmon>).
- At least one TF-enabled game: BeamNG.drive, Assetto Corsa (classic), AC Evo, or Assetto Corsa Competizione.

What to do:

1. Clone the repo and open `tools/windows_tf_captures.bat`.
2. Confirm the `TSHARK` and `INTERFACE` variables at the top match your machine (default USBPcap interface is `\\.\USBPcap1`).
3. Open an Administrator CMD prompt in `tools\`.
4. `windows_tf_captures.bat all` — runs all four captures in sequence. Each one is interactive, with step-by-step on-screen instructions.
5. When finished, attach the resulting `*.pcapng` files (and the ProcMon CSVs from `dll-survey`) to a GitHub issue.

Captures will not contain any personally identifying data beyond the USB device descriptors of the devices attached to that machine during the capture window. Best practice: close unrelated USB devices (other game controllers, smartphones, etc.) before starting.

---

## DLL survey — manual instructions (no batch automation)

ProcMon isn't tshark, so the batch script can't automate it. Do this once before any game captures:

1. Install Sysinternals ProcMon: <https://learn.microsoft.com/sysinternals/downloads/procmon>
2. Launch ProcMon as Administrator.
3. Filter → Filter... (Ctrl+L). Set up these includes:
   - `Process Name` `contains` `BeamNG` → Include
   - `Process Name` `contains` `AssettoCorsa` → Include
   - `Path` `contains` `logi` → Include
   - `Path` `contains` `trueforce` → Include
4. File → Capture events (Ctrl+E) to enable.
5. Launch BeamNG. Wait to main menu. Close BeamNG.
6. File → Save (select "All events", filter was already set).
7. Save as `H:\Projects\logitech-rs50-linux-driver\dev\captures\<date>_re_dll_survey_beamng.pml` (ProcMon native format).
8. Optionally File → Save also as CSV for easier grep: `<date>_re_dll_survey_beamng.csv`.
9. Repeat for AC (classic) or AC Competizione if available.

The batch script also runs a PowerShell inventory of Logitech DLLs on disk, saving to `<date>_re_logi_dlls.csv`.

---

## What the captures don't cover (known limitations)

- **G Pro** — this capture set targets RS50. G Pro-specific captures require a G Pro owner. See `docs/USB_CAPTURE_GUIDE.md` for the general contributor workflow.
- **API Monitor hooks** on the TF SDK — useful for observing per-call sequences but require user-mode hooking. Not in this guide. If the ProcMon + USB capture combination doesn't answer a question, a hooked DLL replacement is a fallback.
- **G Pro Trueforce transport** — G Pro's interface 2 descriptor is 3× larger than RS50's; the TF transport may differ. Out of scope here.

# Ultrareview Design

**Date:** 2026-04-16
**Goal:** Comprehensive review of the RS50/G Pro driver targeting feature parity with the Windows G Hub driver, with full-stack Trueforce support as the long-term endpoint.

---

## Structure

Three phases, each producing a committed document before moving to the next. Each phase can be course-corrected before the next starts. The terminal step is invoking `superpowers:writing-plans` on the roadmap's top priority item to produce a concrete implementation plan.

| Phase | Output | Purpose |
|-------|--------|---------|
| A. Code Audit | `docs/plans/2026-04-16-code-audit.md` | Tactical + strategic quality findings in current driver |
| B. Windows Gap Analysis | `docs/plans/2026-04-16-windows-gap-analysis.md` | Feature-by-feature inventory vs G Hub, backed by captures |
| C. Roadmap + Trueforce Design | `docs/plans/2026-04-16-roadmap-and-trueforce-design.md` | Synthesis: prioritized roadmap + full-stack Trueforce design |

---

## Phase A: Code Audit

**Scope:** Tactical + strategic quality review of the driver as it is on master (commit `1c75a5b` at time of planning).

**Files in scope:**
- `mainline/hid-logitech-hidpp.c` — focus only on RS50/G Pro additions and our modifications, not the upstream baseline
- `mainline/hid-ids.h` — device ID defines
- `mainline/dkms.conf` + `Makefile` + `Kbuild` — build system
- `README.md`, `docs/*.md` — technical accuracy, staleness

**Tactical findings to surface:**
- Dead code, unused variables, stale comments
- Inconsistent patterns (e.g., inconsistent use of `atomic_read(&ff->stopping)` guards)
- Missing error handling on HID++ calls that silently swallow failures
- Sysfs attributes where the cached state and device state can diverge
- Race conditions on `hidpp->private_data` access
- Memory ownership of `rs50_ff_data` across the two init paths (RS50 FFB vs G Pro sysfs-only)
- Docs claiming behavior that doesn't match code

**Strategic findings to surface:**
- Is `HIDPP_QUIRK_RS50_FFB` + product-ID checks sprinkled throughout sustainable?
- Should RS50-specific FFB, LIGHTSYNC, and sysfs be split into separate `.c` files?
- Is `rs50_ff_data` doing too many things (FFB state + settings state + LED state + debug state)?
- Per-feature `fn_set_*` fields: right abstraction or should it be a table?
- Could multi-interface wheel support be generalized into a reusable helper?
- Is the G Pro path (reusing `rs50_ff_data` without FFB) confusing enough to warrant its own struct?

**Output format — numbered findings (F1, F2, ...):**

```
## F1: <short title>
Severity:    Blocker | Important | Nice-to-have
Category:    Tactical | Strategic
Location:    mainline/hid-logitech-hidpp.c:NNNN (function_name)
Description: <what is wrong and why>
Evidence:    <code quote or behavior observed>
Fix direction: <approach, not full code>
```

**Execution:** Dispatch 2-3 parallel subagents, each scoped to a different area (FFB code, sysfs/settings, multi-interface/probe). Each returns findings in the above format. Consolidate and number.

---

## Phase B: Windows Gap Analysis

**Scope:** Feature-by-feature inventory of what G Hub does on Windows vs what our Linux driver supports, backed by USB captures.

**Sources:**

| Source | Location | Contents |
|--------|----------|----------|
| Local RS50 captures | `dev/captures/*.pcapng` (43 files) | GHub startup, settings sweeps, gameplay, LIGHTSYNC, profiles, Trueforce, boot, button mapping, pedal captures |
| Local RS50 notes | `dev/captures/*.{md,txt}` | Prior analysis (`init_sequence.txt`, `button_states.txt`, `hidpp_settings.txt`, `GHUB_INTERFACE.md`, `rs50_features.md`) |
| G Pro captures | `/tmp/gpro_captures/` + issue #8 zips | GHub init, rotation/strength/damping/trueforce sweeps, gameplay, G923 compat mode |
| Trueforce captures | `/tmp/woTF.pcapng`, `/tmp/wTF_*.pcapng` | Issue #5 (BeamNG with/without Trueforce) |

**Output: Feature matrix** with one row per discrete feature:

| Column | Example |
|--------|---------|
| **Feature** | "Rotation range (90-2700°)" |
| **HID++ page/fn** | `0x8138 / fn0=info, fn1=get, fn2=set` |
| **Windows behavior** | "Sends `10 FF 18 2D HH LL 00` on slider change; firmware persists" |
| **Linux status** | `Done` / `Partial` / `Missing` / `Unknown` |
| **Linux implementation** | "`wheel_range` sysfs, verified working on hardware" |
| **Evidence** | `rotation_sweep.pcapng frame 1200-1250` |
| **Gaps / Issues** | "None" or "Writes fine but read returns capability min — see F12 in audit" |

**Feature categories to cover:**

1. **Device identification** — descriptors, firmware version query, feature discovery
2. **Wheel input** — axes, buttons, D-pad, hat, button count, encoder modes
3. **Pedal input** — axes, combined pedals, curves, deadzones
4. **Force feedback** — PID FFB commands used by games, effect types, refresh/keepalive
5. **FFB settings** — range, strength, damping, filter (+auto), brake force, sensitivity
6. **Trueforce** — sensitivity level (HID++), audio stream (endpoint 0x03)
7. **LIGHTSYNC** — slot selection, per-slot color/brightness, effects, global brightness
8. **Profiles / modes** — desktop vs onboard, profile switching, per-profile storage
9. **Initialization** — cold boot sequence, FFB enable, refresh/keepalive commands
10. **G Pro-specific** — differences from RS50 (multi-device HID++ structure, G923 compat mode)

**Also in the output:**
- **Unknowns list** — features seen in captures but not yet understood (mystery feature indices, opaque long reports)
- **Surprises list** — things G Hub does that we didn't expect (e.g., Trueforce init sequence sent twice)

**Execution:** Dispatch subagents per category, each given a specific capture subset, each returns matrix rows. Consolidate into the final document.

---

## Phase C: Roadmap + Trueforce Design

**Scope:** Synthesize Phase A findings and Phase B gaps into a prioritized roadmap, plus the detailed full-stack Trueforce design.

### Part 1: Prioritized Roadmap

**Input:** Phase A findings + Phase B gaps/unknowns + open GitHub issues.

**Output — single ordered work-item list:**

| Column | Example |
|--------|---------|
| **Rank** | 1, 2, 3... |
| **Item** | "Implement hardware wheel calibration (center offset)" |
| **Source** | `issue #13` / `F7+F12` / `GapB.LightSync` |
| **Type** | Bugfix / Feature / Refactor / Docs |
| **Size** | S / M / L / XL |
| **Dependency** | "Requires G Hub calibration capture" |
| **User impact** | High / Medium / Low |
| **Readiness** | Ready / Needs-research / Needs-hardware |

**Prioritization principles:**
1. Blockers (F1/Blocker-severity audit findings) come first regardless of size
2. User-visible bugs on open issues > feature work
3. Parity gaps with confirmed Windows behavior > Trueforce (Trueforce is a huge XL)
4. Refactors only when they unblock something specific — don't refactor for its own sake

### Part 2: Trueforce Full-Stack Design

Its own section with subsections:

**2.1 Kernel layer**
- Claim interface 2 (currently delegated to hid-generic on RS50)
- Expose via: character device (`/dev/rs50_trueforce`) or `input_ff` extension
- Protocol handling: init sequence (48 param floats, frequency config, slot setup, start) → 500Hz audio streaming
- Response handling (type 0x02 with wheel position)
- Buffering, backpressure, error recovery

**2.2 Userspace daemon / SDK shim**
- Minimal library vs daemon with IPC
- API surface mimicking Logitech's Windows SDK
- How it talks to the kernel
- G Pro variant: HID++ long reports instead of endpoint 0x03 — abstracted in the shim

**2.3 Wine/Proton bridge**
- Wine DLL override replacing `logitech_wheel_sdk.dll`
- Alternative: Proton patch
- How the bridge talks to the userspace shim (unix socket? shared lib?)

**2.4 Phased delivery**
- Phase 1: kernel only — games don't use it yet, but enables experimentation
- Phase 2: userspace shim + test client
- Phase 3: Wine DLL override, tested with BeamNG.drive

**2.5 Open questions / unknowns**
- 48 init parameters game-independent or per-game?
- BeamNG Trueforce data via Proton env var or deeper integration?
- Latency budget for the userspace path

### Part 3: Next step

- Top priority item → detailed implementation plan via `writing-plans`
- Subsequent items → their own plans as we pick them up
- Trueforce is the exception: its full-stack design IS the plan-of-plans

**Execution:** Written directly (no subagent), drawing on Phase A + Phase B outputs. Trueforce design pulls from existing `docs/TRUEFORCE_PROTOCOL.md`.

---

## Flow

```
[Phase A design approved]
  -> Dispatch parallel subagents for code audit
  -> Consolidate -> commit docs/plans/2026-04-16-code-audit.md
  -> Checkpoint: review findings together, adjust if needed

[Phase B]
  -> Dispatch per-category subagents against capture files
  -> Consolidate matrix -> commit docs/plans/2026-04-16-windows-gap-analysis.md
  -> Checkpoint: review matrix, identify any missing areas

[Phase C]
  -> Write roadmap + Trueforce design directly
  -> Commit docs/plans/2026-04-16-roadmap-and-trueforce-design.md
  -> Checkpoint: review priorities, confirm top item

[Hand off]
  -> Invoke superpowers:writing-plans on top priority roadmap item
  -> That plan becomes the first concrete implementation task
```

# Phase A: Code Audit

**Date:** 2026-04-16
**Scope:** RS50/G Pro driver code + docs as of master commit `1c75a5b`
**Methodology:** Three parallel subagent reviews — FFB subsystem, sysfs settings, probe/input/docs

Findings are grouped by area (FFB.*, SYS.*, PROBE.*) and ordered by severity within each. This document is a **finding inventory**, not a plan. Phase C synthesizes the prioritized roadmap.

---

## Severity Legend

- **Blocker** — user-visible bug, data corruption risk, or security/UAF concern
- **Important** — protocol correctness, silent state divergence, or real user impact under realistic conditions
- **Nice-to-have** — cleanliness, consistency, maintainability, defense-in-depth, future-proofing

---

## FFB Subsystem (22 findings)

### FFB.F1: Advertised `FF_GAIN` capability has a no-op handler
- **Severity:** Important
- **Category:** Tactical
- **Location:** `mainline/hid-logitech-hidpp.c:4130` (`rs50_ff_set_gain`), `:4701` (`set_bit(FF_GAIN)`)
- **Description:** `FF_GAIN` is advertised on `input->ffbit`, but `rs50_ff_set_gain()` only emits `hid_dbg`. Apps using `EV_FF/FF_GAIN` (Wine, most games, `ffset`) believe gain adjustment works when it silently does nothing.
- **Fix direction:** Either implement real gain (store `u16 gain`, scale `constant_force` at send time) or drop the capability and handler.

### FFB.F2: Upload overwrites `constant_force` instead of summing across effects
- **Severity:** Important
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:4011-4018` (`rs50_ff_upload`)
- **Description:** When upload updates a playing FF_CONSTANT effect it does `ff->constant_force = level;` discarding contributions from other playing effects. Playback-stop correctly re-sums. Upload-while-playing is asymmetric.
- **Fix direction:** Recompute `constant_force` by iterating all playing FF_CONSTANT effects. Shared helper would unify the three sites (upload/playback-start/playback-stop).

### FFB.F3: Playback-start overwrites `constant_force` when other effects already play
- **Severity:** Important
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:4073-4093` (`rs50_ff_playback`)
- **Description:** Starting a new effect sets `constant_force = level` from just this one effect. Any previously-playing constant effect is silently dropped until next playback toggle. Most games use a single constant slot so it hasn't surfaced.
- **Fix direction:** Always recompute across playing effects (shared helper with F2).

### FFB.F4: Timer can be re-armed after `timer_delete_sync()` via playback/upload callbacks
- **Severity:** Important
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:4023-4025`, `:4119-4122`, `:7548`
- **Description:** `rs50_ff_playback` and `rs50_ff_upload` call `mod_timer()` without checking `ff->stopping`. Input FF dispatch happens under `ff->mutex` but not under our stopping flag. A callback in flight during destroy can re-arm the timer after `timer_delete_sync()` returned.
- **Fix direction:** Check `atomic_read_acquire(&ff->stopping)` before every `mod_timer` call. Consider a second `timer_delete_sync()` after `drain_workqueue()`.

### FFB.F5: `constant_force` summation can overflow s16 and wrap at offset-binary conversion
- **Severity:** Important
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:3975`, `:4107`
- **Description:** `constant_force` is s32 but `rs50_ff_send_force` does `(s16)force + 0x8000`, truncating. Multiple active FF_CONSTANT effects can sum beyond [-32768, 32767], silently wrapping a strong right force into a left force.
- **Fix direction:** `clamp(force, S16_MIN, S16_MAX)` before the s16 cast. See F8 for helper extraction.

### FFB.F6: Per-transfer `hid_err` floods kernel log at 500 Hz on persistent failure
- **Severity:** Important
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:4196-4197` (`rs50_ff_work_handler`)
- **Description:** 2ms timer + per-tick work items → 500 error log messages per second on unplugged wheel or USB fault. `refresh_work` is already throttled; per-tick path isn't.
- **Fix direction:** `time_after(jiffies, ff->last_err_log + HZ*N)` style ratelimit, or `printk_ratelimit()`. The unused `ff->err_count` field (F7) can drive this.

### FFB.F7: `err_count` field declared and initialized but never read or incremented
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:3810`, `:7352`
- **Description:** Dead field, probably intended for F6 throttling.
- **Fix direction:** Remove, or actually use it.

### FFB.F8: No `rs50_force_to_offset_binary` helper — conversion is inlined with comment repeated in three places
- **Severity:** Nice-to-have
- **Category:** Strategic
- **Location:** `hid-logitech-hidpp.c:3971-3975`
- **Description:** The offset-binary conversion is a 3-line inline. Extracting it gives one place to apply clamping (F5), lets the direction-projection math (F9) live next to the wire-format contract.
- **Fix direction:** `static u16 rs50_force_to_offset_binary(s32 force)` that clamps to s16 and returns u16.

### FFB.F9: Direction projection math duplicated in three places
- **Severity:** Nice-to-have
- **Category:** Strategic
- **Location:** `hid-logitech-hidpp.c:4016`, `:4092`, `:4106`
- **Description:** `level = (level * fixp_sin16((dir * 360) >> 16)) >> 15;` appears three times. Convention comment is only on one.
- **Fix direction:** Helper `static s32 rs50_project_constant(const struct ff_effect *e)` with the explanation comment on it.

### FFB.F10: Stale comments on refresh cadence (20s vs 30s contradict each other)
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:4210`, `:4721`, constant `:3649`
- **Description:** Constant is 20000 ms. One comment says "20-30 seconds", another says "30 seconds".
- **Fix direction:** Pick 20s and update both comments.

### FFB.F11: `RS50_FF_TIMER_INTERVAL_MS = 2` (500 Hz) is hardcoded with no tuning knob
- **Severity:** Nice-to-have
- **Category:** Strategic
- **Location:** `hid-logitech-hidpp.c:3749`, `:3900` (`MAX_PENDING_WORK`)
- **Description:** 500Hz is fine on fast USB but may saturate EP3 OUT on weak hosts / USB1.1. No module_param, no sysfs knob, no coalescing when force is unchanged.
- **Fix direction:** `module_param_named(force_update_hz, ..., S_IRUGO)` or make coalescing explicit (skip `rs50_ff_send_force` if `force == last_force`).

### FFB.F12: State machine tracks per-effect `playing` but derives `constant_force` inconsistently
- **Severity:** Nice-to-have
- **Category:** Strategic
- **Location:** `hid-logitech-hidpp.c:3760-3764` (struct), `:4095-4109` (stop re-sum)
- **Description:** `constant_force` is separately mutated by upload/playback-start/stop. Should be a pure projection of `{playing && uploaded && type==FF_CONSTANT}` slots, recomputed on any state change. That eliminates F2/F3 by construction.
- **Fix direction:** Introduce `rs50_ff_recompute_constant_force(ff)` callable under `effects_lock`. Call from upload, erase, playback-start, playback-stop.

### FFB.F13: `constant_force` read outside `effects_lock` in timer callback and post-unlock branches
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:3929`, `:4023`, `:4116-4119`
- **Description:** `s32` loads are single-instruction so no torn reads, but discipline is inconsistent. The `if (ff->constant_force != 0)` post-unlock arming decision can race with another thread setting it to zero.
- **Fix direction:** `READ_ONCE` cross-thread or snapshot inside the critical section.

### FFB.F14: Destroy ordering: `refresh_work` cancelled before `init_work`, but init schedules refresh
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:7541`, `:7555`, `:7562`
- **Description:** `init_work` can re-queue `refresh_work` after the cancel. `drain_workqueue` recovers, but ordering is logically backwards.
- **Fix direction:** Swap the two cancel calls; cancel `init_work` first.

### FFB.F15: `ff_hdev_open` check is dead — `ff_hdev` was NULL'd earlier
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:7539`, `:7612-7614`
- **Description:** Line 7539 does `WRITE_ONCE(ff->ff_hdev, NULL)`. Line 7612 checks `if (ff->ff_hdev_open && ff->ff_hdev)` — the second condition is always false, so `hid_hw_close` is never called.
- **Fix direction:** Cache `ff_hdev` in a local before the WRITE_ONCE.

### FFB.F16: Redundant NULL-assign after `kfree(ff)`
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:7619`
- **Description:** `ff->ff_hdev = NULL` right before `kfree(ff)`.
- **Fix direction:** Delete.

### FFB.F17: `init_work` uses `schedule_delayed_work` (system_wq) while everything else uses `ff->wq`
- **Severity:** Nice-to-have
- **Category:** Strategic
- **Location:** `hid-logitech-hidpp.c:4615`, `:7479`
- **Description:** `drain_workqueue(ff->wq)` doesn't cover `init_work`. Route through `ff->wq` to make teardown a single sync point.
- **Fix direction:** `queue_delayed_work(ff->wq, &ff->init_work, ...)`.

### FFB.F18: Playback accesses `ff->effects[id].effect.type` after releasing the lock
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:4113-4118`
- **Description:** Post-unlock re-read of effect.type/level races with `rs50_ff_erase`.
- **Fix direction:** Snapshot into locals inside the critical section.

### FFB.F19: `rs50_ff_send_force` silently truncates s32 -> s16 and silently drops on alloc failure
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:3954-3981`
- **Description:** No clamp. `kmalloc(GFP_ATOMIC)` failures silently dropped with no log/counter.
- **Fix direction:** Clamp to s16 range (tied to F5/F8). Consider a mempool for the bounded `RS50_FF_MAX_PENDING_WORK`.

### FFB.F20: Zero-force release may wait for a previously-scheduled timer tick
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:3929-3947`, `:4094-4122`
- **Description:** Playback-stop that transitions force non-zero → zero doesn't proactively arm the timer; the release command happens whenever the next already-scheduled tick runs. Usually prompt (~2ms) but not guaranteed.
- **Fix direction:** Proactively `mod_timer(..., jiffies)` on the zero-force transition, or call `rs50_ff_send_force(ff, 0)` directly.

### FFB.F21: `ff->last_force` vs `ff->constant_force` read/write discipline is mixed
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `hid-logitech-hidpp.c:3881`, `:3929`, `:3937`, `:3945`
- **Description:** `last_force` is timer-private, `constant_force` is lock-protected. No `READ_ONCE` or memory-order guarantees for the timer's snapshot of `constant_force`.
- **Fix direction:** Document that `last_force` is timer-private; use `READ_ONCE` when reading `constant_force`.

### FFB.F22: `rs50_find_ff_data` stopping-flag short-circuit defeats the interface-0-remove-first cleanup
- **Severity:** Nice-to-have
- **Category:** Strategic
- **Location:** `hid-logitech-hidpp.c:8150-8153`
- **Description:** If interface 1 has already set `stopping=1` while interface 0 is processing its remove path, `rs50_find_ff_data` returns NULL and interface 0 skips the `ff->input->ff->private = NULL` cleanup. `input_ff_destroy` then kfrees the shared ff_data while interface 1 still holds it.
- **Fix direction:** Add a "bypass stopping" variant for the remove path, since that's exactly when we need to find the shared ff_data.

---

## Sysfs Settings (48 findings)

### SYS.F1: `autocenter` store accepts negative inputs, silently clamps to 0
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:4973` (`wheel_autocenter_store`)
- **Fix direction:** Reject with `-EINVAL` or document as intentional.

### SYS.F2: `wheel_range_store` silently clamps out-of-range values; inconsistent with `wheel_throttle_curve_store` which rejects
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:4814` (and many other `_store` handlers with the same pattern)
- **Fix direction:** Pick one convention (kernel convention favors `-EINVAL`), apply uniformly.

### SYS.F3: `wheel_sensitivity_show` returns non-numeric string "N/A (onboard mode)"
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:5225`
- **Description:** Breaks sysfs convention that numeric attributes return a parseable number.
- **Fix direction:** Return the last known desktop value, return `-EPERM`/`-ENODATA`, or don't create the attribute in onboard mode.

### SYS.F4: `wheel_ffb_filter` show returns effective value; write uses user-preference semantics — no clear contract
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:4523-4524`, `:5302`, `:5405`
- **Fix direction:** Document contract, then keep show consistent with it (effective-value vs user-preference).

### SYS.F5: `fn_set_sensitivity = 0x20` is guessed; corrupts LED brightness if wrong
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:7014`, `:5258`, `:5269`
- **Description:** Sensitivity aliases on the brightness feature (0x8040). Since we haven't captured a sensitivity SET, the fn byte is a guess. Wrong fn would write into LED brightness.
- **Fix direction:** Refuse writes until validated via capture, or gate on a separate discovered feature.

### SYS.F6: `wheel_brake_force_show` returns numeric in desktop mode where store rejects with `-EPERM` — asymmetric
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:5132`, `:5170`
- **Fix direction:** Either allow writes in desktop mode (applied when switching to onboard) or also return error on show.

### SYS.F7: FFB filter auto encoding: SET uses 0x04/0x00 but GET decoder expects 0x05/0x01 (mismatch persists after earlier fix)
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:4523`, `:5334`, `:5403`
- **Description:** `params[0] = 0x04` in SET but `params[0] == 0x05` in GET decode. Bit 0 may be a separate flag we don't understand.
- **Fix direction:** Capture-verify the actual encoding and document, or unify both directions.

### SYS.F8: `wheel_led_slot_brightness_store` caches new value BEFORE attempting device write
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:5985-5999`
- **Fix direction:** Write first, cache on success (matches `wheel_strength_store` pattern).

### SYS.F9: Same cache-before-apply bug in `wheel_led_direction_store`
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:6059-6065`

### SYS.F10: Same cache-before-apply bug in `wheel_led_colors_store`
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:6166-6172`

### SYS.F11: No lock on `led_active_slot`; concurrent sysfs writes race
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:5844`, `:5896`, `:6096`, `:6166`
- **Fix direction:** Mutex around LED-slot state, or `READ_ONCE/WRITE_ONCE` with awareness that apply is non-atomic.

### SYS.F12: `wheel_mode_{show,store}` and `wheel_profile_{show,store}` skip `stopping`-flag check → UAF window
- **Severity:** Blocker
- **Category:** Tactical
- **Location:** `:6825`, `:6841`, `:6870`, `:6885`
- **Fix direction:** Add `READ_ONCE(hidpp->private_data)` + `atomic_read_acquire(&ff->stopping)` pattern matching other handlers.

### SYS.F13: `wheel_hidpp_debug_show` lacks the stopping-flag guard
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:6363-6374`
- **Fix direction:** Add guard matching the store path.

### SYS.F14: `wheel_hidpp_debug_store` accepts arbitrary HID++ feature/function with no validation
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:6400`, `:6434`
- **Description:** `%x` sscanf → implicit truncation to u8. Any feature 0-255 + any function 0-255 sent unchecked.
- **Fix direction:** See F19 — gate the whole interface behind Kconfig or move to debugfs.

### SYS.F15: `gpro_sysfs_init()` duplicates feature discovery, query, and defaults from `rs50_ff_*` — drift risk
- **Severity:** Important
- **Category:** Strategic
- **Location:** `:6962-7226`
- **Description:** Known silent-drift bug (sensitivity-not-queried) was caused by this pattern.
- **Fix direction:** Factor `rs50_ff_init_settings_state()` and `rs50_ff_discover_settings_features()` called from both paths.

### SYS.F16: `rs50_ff_data` is a god struct (settings + FFB runtime + LIGHTSYNC + pedal + debug)
- **Severity:** Important
- **Category:** Strategic
- **Location:** `:3795-3897`
- **Fix direction:** Split `struct rs50_settings` (shared) + `struct rs50_ffb_runtime *rt` (RS50-only, NULL on G Pro).

### SYS.F17: Seven `fn_set_*` fields with near-identical structure — table-driven refactor opportunity
- **Severity:** Important
- **Category:** Strategic
- **Location:** `:3835-3841`, `:4303-4347`, `:4460-4547`
- **Fix direction:** `struct rs50_setting_def { u16 page; u8 default_fn_set; bool (*parse_get)(...); ... }` table. Reduces 7 SET handlers to one parameterized template.

### SYS.F18: Oversteer compat attrs duplicate wheel_* attrs, conflict with `hidpp_ff`'s `range` on G Pro
- **Severity:** Important
- **Category:** Strategic
- **Location:** `:4845`, `:4925`, `:4979`, `:5058`, `:6510`
- **Fix direction:** For upstream, drop them. Out-of-tree DKMS can keep. Or keep only non-conflicting ones (`combine_pedals`, `damper_level`, `autocenter`).

### SYS.F19: `wheel_hidpp_debug` exposed unconditionally — no Kconfig gate
- **Severity:** Blocker
- **Category:** Strategic
- **Location:** `:6363`, `:7434` (register), `:7206` (G Pro register)
- **Description:** Root-only raw HID++ shell. Unacceptable for upstream.
- **Fix direction:** Convert to debugfs, or wrap in `CONFIG_HID_LOGITECH_HIDPP_DEBUG` defaulting to "n", or remove before upstream.

### SYS.F20: `rs50_ff_query_settings` hand-codes per-feature response parsing — no table
- **Severity:** Nice-to-have
- **Category:** Strategic
- **Location:** `:4460-4547`
- **Fix direction:** Per-feature parser callbacks in the table from F17.

### SYS.F21: `rs50_ff_discover_features` mixes settings + LIGHTSYNC feature discovery
- **Severity:** Nice-to-have
- **Category:** Strategic
- **Location:** `:4281-4349`
- **Fix direction:** Split into `rs50_discover_settings_features()` + `rs50_discover_lightsync_features()`.

### SYS.F22: Pedal curve/deadzone are software-only but not documented as such
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:6513`, `:6614`, etc., `:8357`, `:8389`
- **Fix direction:** File-level comment clarifying no HID++ feature involved.

### SYS.F23: `rs50_apply_deadzone` doesn't handle lower+upper > 100 sensibly
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:8396-8397`, store at `:6705-6706`
- **Fix direction:** Reject `lower + upper > 100` with `-EINVAL` in the store.

### SYS.F24: `wheel_led_slot_name_store` doesn't reject non-printable or embedded-newline names
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:5898-5911`

### SYS.F25: `scnprintf(buf, PAGE_SIZE, ...)` used in most show handlers; `sysfs_emit` is the modern API
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** Many (~20 handlers)
- **Fix direction:** Convert to `sysfs_emit()` consistently.

### SYS.F26: `wheel_hidpp_debug_show` prints multi-line output violating sysfs convention
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:6376-6389`
- **Fix direction:** Simplify (or move to debugfs per F19).

### SYS.F27: `hidpp_debug_store` truncates `%x` values > 0xFF without warning
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:6427-6428`

### SYS.F28: `gpro_sysfs_destroy()` lacks the `stopping`-flag guard used by RS50 destroy
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:7232-7263`
- **Description:** `device_remove_file` waits for in-flight handlers on *that specific attr*; another attr's in-flight handler can still deref `ff` during kfree. Also, `WRITE_ONCE(private_data, NULL)` should precede `kfree(ff)`.
- **Fix direction:** Set stopping=1 first; WRITE_ONCE before kfree.

### SYS.F29: Manual `device_remove_file` list in destroy — maintenance drift risk
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:7240-7255`
- **Fix direction:** Table of `device_attribute *attrs[]`, iterate for both create and destroy.

### SYS.F30: `gpro_sysfs_init()` doesn't initialize `led_slots[]` defaults (white/brightness=100); RS50 does
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:6976-7028` vs `:7317-7330`

### SYS.F31: `led_active_slot` index used without bounds recheck in show/store handlers
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:5868`, `:5896`, `:5951`, `:5984`, `:6028`, `:6059`, `:6096`, `:6166`, `:6201`
- **Fix direction:** `WARN_ON_ONCE` guard or clamp helper.

### SYS.F32: `rs50_get_current_mode()` silently assumes desktop when profile feature not found
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:4365-4370`
- **Fix direction:** Track `mode_known` flag, or return `-ENOTSUPP`.

### SYS.F33: `rs50_ff_query_settings()` doesn't check `stopping` — HID++ commands during teardown
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:4441-4573`
- **Fix direction:** Check at function entry and between feature groups.

### SYS.F34: `rs50_get_current_mode` error return ignored in `gpro_sysfs_init`
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:7096`

### SYS.F35: Sensitivity cached based on `current_mode == 0` which may be untrustworthy (see F32/F34)
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:4541-4545`

### SYS.F36: `wheel_led_brightness_store` hardcodes `RS50_HIDPP_FN_SET` — no `fn_set_brightness`
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:6340`, `:5994`
- **Fix direction:** Add `fn_set_brightness` (apply table approach from F17).

### SYS.F37: `rs50_ff_data` has no marker distinguishing G Pro (partial) vs RS50 (full) allocation
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:6976`
- **Fix direction:** `enum device_class` or `bool is_settings_only` field; assert in FFB-touching functions.

### SYS.F38: Default LED state divergence between RS50 (white/100%) and G Pro (zeroed) inits
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:6976` vs `:7317-7330`

### SYS.F39: Stale comment: sensitivity and brightness aliasing is subtle and undocumented
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:3846`

### SYS.F40: `params[2] = 0` reserved-byte assumptions not documented per feature
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** Many SET callers
- **Fix direction:** Comment each with the capture/source that confirms "reserved".

### SYS.F41: `hidpp_send_fap_command_sync` error handling inconsistent (some handlers distinguish positive ret, some don't)
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** Various error paths
- **Fix direction:** Extract `hidpp_errno(int ret, const char *op)` helper.

### SYS.F42: `rs50_set_mode` doesn't re-query dependent settings after switch
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:4393-4429`
- **Description:** After mode switch, range/strength/damping may change; sysfs still shows stale values.
- **Fix direction:** Call `rs50_ff_query_settings` (or a settings-only subset) after successful mode change.

### SYS.F43: `wheel_led_effect_store` sets cached effect but doesn't re-apply slot config — custom/non-custom transition incoherent
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:6281`

### SYS.F44: LIGHTSYNC init uses `hid_info` heavily instead of `hid_dbg` — dmesg noise on every module load
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:5457-5792` (many sites)
- **Fix direction:** Convert to `hid_dbg`; keep one summary `hid_info`.

### SYS.F45: `wheel_led_slot_name_store` implicit bound `RS50_SLOT_NAME_MAX_LEN + 2 <= sizeof(params)` not asserted
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:5880`, `:5913-5915`
- **Fix direction:** `BUILD_BUG_ON`.

### SYS.F46: `hidpp_send_fap_command_sync(..., 0)` with zero-length payload relies on implicit contract
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** Many GET callers

### SYS.F47: Concurrent `wheel_hidpp_debug` writes can interleave `debug_last_*` fields
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:6438-6441`

### SYS.F48: `wheel_sensitivity_store` doesn't update `ff->led_brightness` even though it writes the brightness feature
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:5278`
- **Description:** If sensitivity and LED brightness are the same feature (one byte), writes must keep both caches in sync. Currently the two fields diverge.
- **Fix direction:** Verify aliasing hypothesis via capture. If truly same, alias fields. If different sub-features, use the correct `params[0]` byte to discriminate.

---

## Probe / Input / Docs / Build (24 findings)

### PROBE.F1: `wheel_led_effect` range 1-5 but docs say 5-9
- **Severity:** Important
- **Category:** Tactical
- **Location:** `docs/SYSFS_API.md:295-310`, `README.md:226`, code `:6259-6260`
- **Fix direction:** Update both docs to match code range 1-5.

### PROBE.F2: README claims driver "patches the descriptor" — it doesn't
- **Severity:** Important
- **Category:** Tactical
- **Location:** `README.md:357-359`
- **Fix direction:** Reword to "filters these buttons during HID input mapping".

### PROBE.F3: Diagonal D-pad values (0x01/0x03/0x05/0x07) unverified
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:3637-3644`, `:8291-8324`, spec Section 3 vs Section 10 contradicting
- **Fix direction:** Recapture with 8-way press to confirm; update spec Section 3 or remove diagonal cases.

### PROBE.F4: `wheel_led_slot` switch clobbers device-stored RGB with driver defaults
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:5815-5847`, `:7317-7330`
- **Description:** Init sets all slots to white. Slot-name query on init doesn't fetch RGB. Any user-saved (or G Hub) slot colors get overwritten on next slot switch.
- **Fix direction:** Call `RS50_RGB_FN_GET_CONFIG` for all 5 slots during discovery to populate `led_slots[].colors`, or don't auto-apply on slot switch — require explicit `wheel_led_apply`.

### PROBE.F5: `rs50_setup_dpad` gated on RS50-only quirk — G Pro D-pad setup missing
- **Severity:** Important (Blocker if G Pro has D-pad)
- **Category:** Tactical
- **Location:** `:7993-7995`
- **Description:** `HIDPP_QUIRK_RS50_FFB` guard excludes G Pro. Whole raw_event D-pad path similarly gated.
- **Fix direction:** Verify G Pro HID descriptor. If it needs manual hat setup, add a shared quirk or key on product ID.

### PROBE.F6: G Pro interface-0 path runs full HID++ init infrastructure unnecessarily
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:8934-8948`
- **Description:** `rs50_continue_probe` bypasses the `HID_CONNECT_DEFAULT` early-exit. All subsequent probe infrastructure (battery group, send_mutex, work queues, hid_connect) runs on interface 0 where it's unused.
- **Fix direction:** Split a minimal "raw-event-only" probe path, or gate the full init on `supported_reports != 0`.

### PROBE.F7: `rs50_continue_probe` label + downstream `supported_reports` re-checks trending toward quirk-based dispatch
- **Severity:** Nice-to-have
- **Category:** Strategic
- **Location:** `:8912-9087`
- **Description:** 5+ defensive `supported_reports` re-checks + scattered product-ID checks in `hidpp_input_mapping` and `hidpp_ff_range_store`.
- **Fix direction:** `HIDPP_QUIRK_MULTI_IF_INPUT` bit flagged at probe, replace scattered product-ID checks.

### PROBE.F8: `RS50_MAX_BUTTON_USAGE = 0x50` threshold unverified for G Pro
- **Severity:** Important
- **Category:** Strategic
- **Location:** `:3674`, `:6925-6948`
- **Fix direction:** Dump G Pro HID descriptor via `/sys/kernel/debug/hid/<id>/rdesc`; confirm highest used button usage.

### PROBE.F9: LIGHTSYNC slot count (5) / LED count (10) hardcoded — not runtime-derived from fn0 GET_INFO
- **Severity:** Nice-to-have
- **Category:** Strategic
- **Location:** `:3686-3687`, `:5517`, `:5689`, `:5755`
- **Fix direction:** Parse fn0 response into runtime fields; dynamically size `led_slots[]`.

### PROBE.F10: `rs50_lightsync_enable` sends multiple queries that log but don't feed any runtime state
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:5466-5508`
- **Fix direction:** Delete the diagnostic probes, or use them to populate F9's runtime LED/slot counts.

### PROBE.F11: `rs50_lightsync_apply_slot` ignores fn3 ACTIVATE return; logged only
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:5741-5743`, `:5760-5771`

### PROBE.F12: `docs/RS50_SUPPORT.md` is obsolete — duplicates README/PROTOCOL_SPEC content, stale date
- **Severity:** Nice-to-have
- **Category:** Strategic
- **Location:** `docs/RS50_SUPPORT.md`
- **Fix direction:** Delete or add deprecation banner.

### PROBE.F13: `docs/USB_CAPTURE_GUIDE.md` still lists G Pro as "target unsupported wheel", has stale open questions
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `docs/USB_CAPTURE_GUIDE.md:5`, `:104`, `:153`, `:303-329`

### PROBE.F14: Protocol spec sysfs table incomplete (missing mode/profile, LED slot name/brightness/effect, debug, Oversteer compat)
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `docs/RS50_PROTOCOL_SPECIFICATION.md:572-602`
- **Fix direction:** Replace with pointer to `SYSFS_API.md`.

### PROBE.F15: G Pro Xbox-vs-PS PID assignment unverified (comment in probe table)
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:9287-9289`, `hid-ids.h:899-901`

### PROBE.F16: `hidpp_ff_init` hardcodes "input lives on interface 0" for multi-interface G920-class wheels
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:2934-2941`
- **Description:** `usb_ifnum_to_if(udev, 0)` assumption. Also baked into `rs50_ff_init_work` (:4606, :4630) and `hidpp_ff_range_store` (:2865).
- **Fix direction:** Iterate interfaces to find one with non-empty `inputs`, or document assumption.

### PROBE.F17: G Pro sub-device structure (device indices 0x01, 0x02, 0x05) entirely unaddressed
- **Severity:** Important (Blocker for full G Pro support)
- **Category:** Strategic
- **Location:** Driver-wide — all HID++ sends use `device_index = 0xFF`
- **Description:** Captures show G Pro exposes sub-devices. Our driver only addresses 0xFF. Pedal-base/paddle-shifter module features through sub-devices are invisible.
- **Fix direction:** Short-term: document the limitation clearly (README claim "Full support" is misleading). Long-term: enumerate sub-devices and route features.

### PROBE.F18: `hidpp_raw_event` doesn't verify interface number before 30-byte pedal processing
- **Severity:** Important
- **Category:** Tactical
- **Location:** `:8220-8228`, `:8425-8500`
- **Description:** Any 30-byte non-HID++ report on any interface would be parsed as pedal data, and `rs50_process_pedals` modifies the buffer in-place via `put_unaligned_le16`.
- **Fix direction:** Check interface number == 0 at entry.

### PROBE.F19: Protocol spec contradicts itself about whether fn6/fn7 exist on feature 0x0C
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `docs/RS50_PROTOCOL_SPECIFICATION.md:840` vs `:908`, `:1113-1123`
- **Fix direction:** Align spec examples with driver behavior (fn6/fn7 on 0x0B only).

### PROBE.F20: README DKMS install copies pre-built artifacts into `/usr/src`
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `README.md:107-115`
- **Fix direction:** Instruct `make clean` first, or copy only source files.

### PROBE.F21: `dkms.conf` at project root rather than alongside sources in `mainline/`
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `/dkms.conf`, `README.md:107-110`
- **Fix direction:** Move to `mainline/`, simplify install flow.

### PROBE.F22: `Kbuild` and `Makefile` both declare `obj-m` — Kbuild wins, Makefile's declaration is dead
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `mainline/Kbuild:1`, `mainline/Makefile:4`
- **Fix direction:** Delete one.

### PROBE.F23: `hidpp_usages[]` table undocumented — historical mouse-regression-fix fragility invisible
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:9342-9345`
- **Fix direction:** Add 3-4 line comment explaining purpose and the `HID_ANY_ID - 1` sentinel convention.

### PROBE.F24: `hidpp_remove` interface-0-first branch uses `rs50_find_ff_data` which may race with sibling teardown
- **Severity:** Nice-to-have
- **Category:** Tactical
- **Location:** `:9141-9162`
- **Description:** `atomic_read_acquire(&ff->stopping)` check in `rs50_find_ff_data` still leaves a timing gap: interface 1's stopping flag hasn't been set yet when interface 0 starts its remove.
- **Fix direction:** Serialize via module-level lock or document that USB disconnect is serialized per-device.

---

## Cross-Cutting Themes

Grouping findings by structural theme:

1. **Cached state divergence from device** — F4, F8, F9, F10, F42, F48 — Multiple store handlers mutate cache before device write, or never refresh after mode changes.
2. **Missing `stopping`-flag guards** — F12, F13, F28, F33 — UAF window between probe-path queries and remove-path teardown.
3. **Debug interface not Kconfig-gated** — F19, F14, F26 — Must resolve before upstream.
4. **`rs50_ff_data` as god struct + 7×7 SET duplication** — F16, F17, F20, F21 — Strategic refactor to unlock cleaner G Pro support and easier new-wheel additions.
5. **Silent divergence between RS50 and G Pro init paths** — F15, F30, F34, F35, F38 — Factor shared initialization.
6. **Hardcoded constants that should be runtime** — PROBE.F9 (LED/slot counts), PROBE.F16 (interface 0 hardcoded), FFB.F11 (500Hz).
7. **Product-ID checks sprinkled everywhere** — PROBE.F5, PROBE.F7, PROBE.F8, FFB scattering — Quirk-bit approach needed.
8. **Docs lag code** — PROBE.F1, PROBE.F2, PROBE.F12, PROBE.F13, PROBE.F14, PROBE.F19 — Several docs mismatches.
9. **G Pro support is thinner than README implies** — PROBE.F5, PROBE.F8, PROBE.F17 — D-pad, button threshold, sub-devices unaddressed.
10. **FFB state-machine inconsistency** — FFB.F2, F3, F12 — `constant_force` should be a pure projection.

---

## Summary

**94 findings total:**
- **2 Blockers** (SYS.F12, SYS.F19)
- **27 Important**
- **65 Nice-to-have**

Distribution:
- FFB subsystem: 22 findings (4 Important, 18 Nice-to-have)
- Sysfs settings: 48 findings (2 Blockers, 14 Important, 32 Nice-to-have)
- Probe/input/docs/build: 24 findings (7 Important, 17 Nice-to-have)

**Next:** Phase B (Windows Gap Analysis) will map these findings against actual G Hub feature coverage to produce the prioritized roadmap in Phase C.

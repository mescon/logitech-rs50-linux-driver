# G Pro Racing Wheel Support Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add Logitech G Pro Racing Wheel (046d:c272) support with working FFB and sysfs settings, reusing the existing G920 FFB code path and RS50 settings framework.

**Architecture:** The G Pro uses HID++ Feature 0x8123 FFB (like G920/G923), not dedicated-endpoint FFB (like RS50). FFB goes through `hidpp_ff_init` / `hidpp_ff_upload_effect` on interface 1. Settings (range, strength, damping, trueforce, LIGHTSYNC) use the same HID++ feature page IDs as RS50 but with per-feature function number differences. We add a new quirk `HIDPP_QUIRK_GPRO_WHEEL` that shares the G920 FFB path and layers RS50-style sysfs settings on top, with a function number mapping table.

**Tech Stack:** Linux kernel module (C), HID++ protocol, kernel-dev VM for build testing

---

## Background

- G Pro USB ID: `046d:c272` (Xbox/PC variant), `046d:c268` (PS/PC variant)
- JacKeTUs fork has Bronze-tier G Pro support with broken FFB in most games
- The JacKeTUs issues: FFB command queue overflow (2700+ queued commands), incorrect init sequence
- USB captures from issue #8 confirm: G920-style FFB on feature index 0x09, settings at same page IDs as RS50
- Key protocol difference: SET function number varies per feature (fn=1 for damping, fn=2 for range/strength, fn=3 for trueforce)

## Key Findings from USB Capture Analysis

**Feature indices (G Pro PC Pro mode, device 0xFF):**

| Index | Page ID | Feature | SET fn |
|-------|---------|---------|--------|
| 0x09 | 0x8123 | FFB | fn=2 (same as G920) |
| 0x0B | 0x807A | LIGHTSYNC | - |
| 0x0C | 0x80A0 | RGB Config | - |
| 0x11 | 0x8137 | Profile | fn=2 |
| 0x12 | 0x8133 | Damping | fn=1 |
| 0x14 | 0x8136 | FFB Strength | fn=2 |
| 0x16 | 0x8138 | Rotation Range | fn=2 |
| 0x17 | 0x8139 | Trueforce | fn=3 |

Note: Feature indices are discovered at runtime via IRoot. The table above is from captures; actual indices may vary.

**FFB architecture:**
- Uses HID++ feature 0x8123 (same as G920/G923)
- Short reports (`10 ff [idx] 2d ...`) + long reports (`11 ff [idx] 6d ...`) for force data
- No traffic on interface 2 / endpoint 0x03 during gameplay (unlike RS50)

---

### Task 1: Add G Pro Device IDs

**Files:**
- Modify: `mainline/hid-ids.h` - add device ID defines
- Modify: `mainline/hid-logitech-hidpp.c:~8874` - add device table entries

**Step 1: Add device ID defines to hid-ids.h**

Find the Logitech wheel section (near `USB_DEVICE_ID_LOGITECH_G920_WHEEL`) and add:

```c
#define USB_DEVICE_ID_LOGITECH_G_PRO_WHEEL         0xc272
#define USB_DEVICE_ID_LOGITECH_G_PRO_PS_WHEEL      0xc268
```

**Step 2: Add device table entries to hid-logitech-hidpp.c**

After the G923 entry (~line 8878), add:

```c
{ /* Logitech G Pro Racing Wheel (Xbox/PC) over USB */
  HID_USB_DEVICE(USB_VENDOR_ID_LOGITECH, USB_DEVICE_ID_LOGITECH_G_PRO_WHEEL),
    .driver_data = HIDPP_QUIRK_CLASS_G920 | HIDPP_QUIRK_FORCE_OUTPUT_REPORTS },
{ /* Logitech G Pro Racing Wheel (PlayStation/PC) over USB */
  HID_USB_DEVICE(USB_VENDOR_ID_LOGITECH, USB_DEVICE_ID_LOGITECH_G_PRO_PS_WHEEL),
    .driver_data = HIDPP_QUIRK_CLASS_G920 | HIDPP_QUIRK_FORCE_OUTPUT_REPORTS },
```

This uses `HIDPP_QUIRK_CLASS_G920 | HIDPP_QUIRK_FORCE_OUTPUT_REPORTS` (same as G920/G923), which routes FFB through the existing `g920_get_config` + `hidpp_ff_init` path. No RS50 quirk needed since G Pro FFB doesn't use dedicated endpoints.

**Step 3: Set default range to 1080 for G Pro**

In `g920_get_config()` (~line 3588), the fallback range for unknown wheels is 900. Add the G Pro to the 1080 default:

```c
if (hidpp->hid_dev->product == USB_DEVICE_ID_LOGITECH_RS50 ||
    hidpp->hid_dev->product == USB_DEVICE_ID_LOGITECH_G_PRO_WHEEL ||
    hidpp->hid_dev->product == USB_DEVICE_ID_LOGITECH_G_PRO_PS_WHEEL)
    data->range = 1080;
```

**Step 4: Build and verify compilation**

```bash
ssh -i /home/mescon/.ssh/id_ed25519 root@kernel-dev \
  'cp /home/mescon/Projects/logitech-rs50-linux-driver/mainline/hid-logitech-hidpp.c /tmp/driver/ && \
   cp /home/mescon/Projects/logitech-rs50-linux-driver/mainline/hid-ids.h /tmp/driver/ && \
   cd /tmp/driver && make 2>&1 | tail -5'
```

Expected: Clean build, no errors.

**Step 5: Commit**

```
feat: add G Pro Racing Wheel device IDs (046d:c272, 046d:c268)

Add basic support for the Logitech G Pro Racing Wheel using the
existing G920 FFB code path (HID++ feature 0x8123). This provides
force feedback with FF_CONSTANT, FF_PERIODIC, FF_SPRING, FF_DAMPER,
and other effects.

Both Xbox/PC (c272) and PlayStation/PC (c268) variants are included.
```

---

### Task 2: Add G Pro Sysfs Settings (Range, Strength, Damping)

This task adds the RS50-style sysfs settings framework for the G Pro. The challenge is that the SET function number varies per feature on the G Pro (fn=1 for damping, fn=2 for range/strength, fn=3 for trueforce), while the RS50 uses fn=2 for everything.

**Files:**
- Modify: `mainline/hid-logitech-hidpp.c` - add per-device function mapping and G Pro sysfs init

**Step 1: Add per-feature function number defines**

Near the existing `RS50_HIDPP_FN_*` defines (~line 3673), add a comment noting the per-device differences:

```c
/*
 * HID++ function IDs for settings features.
 * Most features follow: fn0=getInfo, fn1=getValue, fn2=setValue
 * But some devices differ per-feature:
 *   G Pro damping: fn0=getInfo, fn1=setValue (not fn2)
 *   G Pro trueforce: fn0=getInfo, fn1=getValue, fn3=setValue (not fn2)
 */
```

**Step 2: Add function number fields to rs50_ff_data**

The `rs50_ff_data` struct is reused for G Pro settings. Add per-feature SET function fields:

```c
u8 fn_set_range;        /* SET function for range (default 0x20) */
u8 fn_set_strength;     /* SET function for strength (default 0x20) */
u8 fn_set_damping;      /* SET function for damping (RS50: 0x20, G Pro: 0x10) */
u8 fn_set_trueforce;    /* SET function for trueforce (RS50: 0x20, G Pro: 0x30) */
```

**Step 3: Initialize function numbers based on device**

In the init function, after allocating ff_data, set defaults based on product ID:

```c
/* Default: fn2 for all SET operations (RS50 pattern) */
ff->fn_set_range = RS50_HIDPP_FN_SET;
ff->fn_set_strength = RS50_HIDPP_FN_SET;
ff->fn_set_damping = RS50_HIDPP_FN_SET;
ff->fn_set_trueforce = RS50_HIDPP_FN_SET;

/* G Pro overrides */
if (hid->product == USB_DEVICE_ID_LOGITECH_G_PRO_WHEEL ||
    hid->product == USB_DEVICE_ID_LOGITECH_G_PRO_PS_WHEEL) {
    ff->fn_set_damping = RS50_HIDPP_FN_GET;     /* fn1 = 0x10 */
    ff->fn_set_trueforce = 0x30;                 /* fn3 */
}
```

**Step 4: Update sysfs store functions to use per-feature fn**

Each `_store` function currently uses `RS50_HIDPP_FN_SET`. Update to use the per-feature field:

- `rs50_range_store`: use `ff->fn_set_range`
- `rs50_strength_store`: use `ff->fn_set_strength`
- `rs50_damping_store`: use `ff->fn_set_damping`
- `rs50_trueforce_store`: use `ff->fn_set_trueforce`

**Step 5: Create G Pro sysfs init function**

Create `gpro_sysfs_init()` that discovers features and creates sysfs attributes, similar to `rs50_ff_init` but without the dedicated-endpoint FFB setup. This runs after `hidpp_ff_init` completes for G Pro devices.

The function should:
1. Allocate `rs50_ff_data` (reuse the struct, it holds settings state)
2. Discover feature indices via `hidpp_root_get_feature` for each page ID
3. Query current values via fn1
4. Create sysfs attributes (reuse the RS50 `dev_attr_rs50_*` attributes)
5. Set per-device function numbers

**Step 6: Hook G Pro sysfs init into probe**

In `hidpp_probe`, after the G920 FFB init block (~line 8668), add:

```c
if (hidpp->hid_dev->product == USB_DEVICE_ID_LOGITECH_G_PRO_WHEEL ||
    hidpp->hid_dev->product == USB_DEVICE_ID_LOGITECH_G_PRO_PS_WHEEL) {
    gpro_sysfs_init(hidpp);
}
```

**Step 7: Build and test**

Build on kernel-dev VM. We cannot test with actual G Pro hardware, but verify:
- Clean compilation
- RS50 still works correctly (no regression)

**Step 8: Commit**

```
feat: add sysfs settings for G Pro Racing Wheel

Add rotation range, FFB strength, damping, and trueforce sysfs
controls for the G Pro wheel, reusing the RS50 sysfs framework.

The G Pro uses different HID++ SET function numbers per feature
(fn=1 for damping, fn=3 for trueforce), so the driver now stores
per-feature function numbers instead of using a single hardcoded
value.
```

---

### Task 3: Investigate and Fix FFB Command Queue Overflow

The JacKeTUs fork's G Pro FFB was broken because the HID++ FFB command queue overflowed with 2700+ commands, causing "substantial delays". This needs investigation using the gameplay capture.

**Files:**
- Analyze: `/tmp/gpro_captures/ffb_gameplay_gpro_pc.pcapng`
- Modify: `mainline/hid-logitech-hidpp.c` - FFB command handling

**Step 1: Analyze FFB command rate in gameplay capture**

Using tshark, determine:
- How many FFB commands per second does AC Evo send?
- Are there paired short+long reports per FFB tick?
- What is the idle/keepalive pattern?
- Is there a backpressure mechanism (device responses to FFB commands)?

Compare with the G920 driver's expected rate to identify the overflow cause.

**Step 2: Check the hidpp_ff work queue**

The existing `hidpp_ff_upload_effect` queues work items on `data->wq` (single-threaded). If the game uploads effects faster than the workqueue can send HID++ commands, the queue grows unbounded.

Investigate whether:
- The G Pro's FFB feature returns fewer effect slots than expected
- The command send rate needs throttling
- The workqueue needs draining/flushing on effect updates

**Step 3: Implement fix**

Based on findings, likely fixes:
- Rate-limit the FFB command queue
- Drop stale commands when the queue grows too large
- Ensure effect updates replace queued commands rather than appending

**Step 4: Build and verify**

**Step 5: Commit**

---

### Task 4: Update Documentation

**Files:**
- Modify: `README.md` - add G Pro to supported devices
- Create: `docs/GPRO_PROTOCOL.md` - document G Pro protocol findings
- Modify: `docs/RS50_PROTOCOL_SPECIFICATION.md` - note shared protocol elements

**Step 1: Update README**

Add G Pro to the supported devices list and note the Xbox/PC and PS/PC variants.

**Step 2: Write G Pro protocol documentation**

Document the capture analysis findings:
- Feature table with indices and function numbers
- FFB architecture (HID++ 0x8123, same as G920)
- Differences from RS50 (no dedicated FFB endpoint, different SET fn numbers)
- G923 compatibility mode observations
- Multi-device structure (devices 0xFF, 0x01, 0x02, 0x05)

**Step 3: Respond on issue #8**

Comment on the issue with findings, thank the contributor, and note what was implemented.

**Step 4: Commit**

```
docs: add G Pro Racing Wheel protocol documentation and README update
```

---

## Execution Notes

- **We have no G Pro hardware.** Tasks 1-2 can be built and verified for compilation + RS50 non-regression. Actual G Pro testing requires the issue #8 reporter or another community member.
- **Task 3 is speculative.** The FFB overflow may require G Pro hardware to reproduce and fix. The capture analysis in Step 1 may reveal the issue theoretically, but we may need to ship Tasks 1-2 first and get community testing feedback.
- **The sysfs attribute names stay as `rs50_*`.** This is not ideal long-term but avoids breaking existing users. A future rename to generic names (e.g., `wheel_range`) could be done as a separate task.
- **Blacklisting:** G Pro users will also need to blacklist `hid-logitech` (lg4ff) since lg4ff matches Logitech vendor ID broadly. The README already documents this.

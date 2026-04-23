# Changelog

This project follows a loose semver: major versions mark API-breaking
changes to the sysfs surface, minor versions add supported wheels or
new attributes, patch versions are bug fixes and documentation. Pre-1.0
the contract is "it works on RS50 and G Pro as listed here".

## Unreleased

Over one hundred commits since the `v0.9-pre-simplification` tag on
2026-02-02. Rather than enumerate all of them, this entry groups them
by theme. See `git log v0.9-pre-simplification..HEAD` for the full
chronology.

### Added

- **Full force feedback effect set** via software emulation on top
  of the RS50's constant-force endpoint (commit `d5b7cc0`). The
  driver now accepts and produces `FF_SPRING`, `FF_DAMPER`,
  `FF_FRICTION`, `FF_INERTIA`, `FF_RAMP`, and `FF_PERIODIC`
  (SINE/SQUARE/TRIANGLE/SAW_UP/SAW_DOWN) in addition to
  `FF_CONSTANT`. Condition effects read the live wheel position,
  velocity and acceleration sampled from interface-0 input reports
  at the 500 Hz timer cadence. Motivated by ACC which uploads
  thousands of DAMPER effects and essentially no constant forces,
  revealing the previous constant-only behaviour as a feel-killer.
- **G PRO Racing Wheel support**, both Xbox/PC (`046d:c272`) and PS/PC
  (`046d:c268`) variants. FFB via the G920-class HID++ 0x8123 path on
  interface 1, TRUEFORCE streaming via the same interface 2 endpoint 0x03
  that the RS50 uses. Every `wheel_*` sysfs attribute relevant to the
  G Pro's hardware is exposed. `gpro_sysfs_init` discovers the
  per-feature SET function numbers and any G Pro-specific sub-device
  features at init time.
- **Wheel calibration** via a new write-only sysfs attribute
  `wheel_calibrate`. Writes a 0..65535 raw encoder value that the wheel
  adopts as the new centre reference. Backed by sub-device `0x05`,
  feature page `0x812C`, function 3 (matching what G Hub does when the
  user clicks Calibrate). Originally only wired up on the G Pro;
  commit `1ed2d80` enabled the same path on RS50 once an RS50 G Hub
  capture (`2026-04-22_re_calibrate.pcapng`) confirmed the sub-device
  layout matches. Closes issue #13.
- **TRUEFORCE full-stack userspace support** in `userspace/libtrueforce/`.
  A shared library that speaks the 64-byte report ID 0x01 stream on
  interface 2 directly via hidraw. Handles the 68-packet two-pass init
  exactly as G Hub does (verified byte-for-byte against both wheels
  across multiple games). Exposes the full Logitech Steering Wheel SDK
  entry-point surface (discover / open / close, set / get torque, TF
  streaming, angle and angular velocity, operating range, damping,
  gain). Forwards range / damping / TF gain to the kernel's `wheel_*`
  sysfs knobs so the library and the driver never disagree.
- **Wine PE shim** at `userspace/tf_wine_shim/`. Builds a
  `trueforce_sdk.dll.so` via winegcc that implements the entire export
  surface of Logitech's Windows `trueforce_sdk_x64.dll` and forwards
  every named entry point into libtrueforce. Lets Proton games call
  the Logitech SDK with `WINEDLLOVERRIDES="trueforce_sdk=n,b"` and no
  Windows G Hub Agent. Build verified in CI; end-to-end game load still
  untested.
- **Profile / rotation broadcasts** on interface 1. The wheel emits
  unsolicited notifications on profile button press and rotation-range
  changes; the driver now consumes both and updates cached sysfs state,
  including re-querying dependent settings after a mode change.
- **Onboard and desktop profile/mode support** via `wheel_mode` and
  `wheel_profile`. Switching between `desktop` and onboard profile 1-5
  applies the correct active profile to the wheel and invalidates the
  settings cache so the next sysfs read reflects reality.
- **LIGHTSYNC custom slot control** on RS50. Five user-configurable
  slots with per-LED RGB, per-slot effect/direction, brightness, and
  slot-name write. LED configuration writes are transactional (apply +
  commit) to match G Hub's behaviour.
- **Community-facing capture scripts** under `tools/`:
  `windows_tf_captures.bat` for game-session captures and
  `windows_wheel_captures.bat` for settings / input captures. Tracked
  in the repo, referenced from `docs/WINDOWS_RE_CAPTURE_GUIDE.md` and
  the in-repo issues that ask contributors for captures (#14 and #15).
- **CI coverage for userspace**: GitHub Actions now builds libtrueforce
  and the Wine PE shim on every push and runs the wire-conversion unit
  tests (`make check`). Kernel driver continues to build against
  5.15, 6.8 and (locally verified) 6.18-debug.

### Fixed

- **rmmod regressions on live RS50**: two destroy-path crashes. The
  `ff_hdev` pointer cached on interface 1 became stale if interface 2's
  `hidpp_remove` ran first during rmmod, producing a null-ptr deref
  inside `hid_hw_close`. The thin-probe interfaces also left the
  `hidpp_device` work_structs uninitialised, tripping
  `WARN_ON_ONCE(!work->func)` in `cancel_work_sync`. Both resolved
  (995607f, simplified in 8ab5fc4).
- **FFB filter byte-0 bitfield**: earlier analysis modelled byte 0 as a
  single flag with a per-wheel offset. Cross-capture re-analysis
  decoded it as `bit 0 = user explicit, bit 2 = auto`, identical on
  RS50 and G Pro (63999d8).
- **RS50 damping and trueforce SET function numbers**: damping uses
  fn=1 and trueforce uses fn=3, not the default fn=2 both paths used
  to send. The G Pro init block already had the overrides; the RS50
  path was missing them (c2ee83e).
- **G Pro FFB filter SET**: corrected fn=3 to fn=2 and auto-flag
  encoding to `0x01 / 0x05` after capture analysis on a live G Pro
  (09e2a6c).
- **Profile broadcast handler** previously gated on the wrong nibble of
  the HID++ function byte; missed broadcasts meant the cached
  `wheel_profile` went stale on profile-button presses. Fixed to gate
  on `sw_id == 0` (46914ad).
- **G Pro interface-0 probe path** and the G Pro / RS50 hid_hw_init
  interface iteration (d1a1bd4, 8106b3a) address sixtysecondstosmash's
  "fftest shows 0 effects" report in issue #8. Retest on G Pro still
  pending user confirmation.
- **C90 compliance** on kernel 5.15 builds: three recent additions
  slipped through with C99 inline declarations that the Ubuntu-22.04
  build rejects under `-std=gnu89`. Rolled back to C90-clean
  declarations (7249eef).
- **Batch script line endings**: `tools/*.bat` scripts were LF-only,
  which broke `call :label` resolution in Windows `cmd.exe` past a
  certain file size. Forced CRLF via `.gitattributes` and `-text`
  (35d0eb4).
- **TRUEFORCE init sent twice, not once**: libtrueforce originally
  replayed the 68-packet init on session open but stopped after one
  pass. Live G Hub captures on both wheels show a duplicate pass with
  the sequence counter reset to 1; the library now matches that
  (0aebf70).
- Many smaller correctness fixes: FF_GAIN scaling in the constant-force
  path, constant_force accesses paired under `WRITE_ONCE`/`READ_ONCE`,
  timer re-arming on zero-force release, pedal deadzone overlap
  rejection, rate-limited FFB error counters, wheel_sensitivity numeric
  return in onboard mode, sysfs_emit for show handlers, LIGHTSYNC
  probe cleanup, LED stores that write the device before updating the
  cache.
- **Sensitivity cache aliasing** correctly gated on `mode_known` so a
  failed mode query no longer caches an LED-brightness value as wheel
  sensitivity (`a99847b`).
- **Out-of-tree build portability**: dropped the `usbhid/usbhid.h`
  include and inlined the one `hid_to_usb_dev` macro we used from it,
  so builds succeed on Fedora, CachyOS, Arch and similar distributions
  whose kernel-devel package does not ship that internal header
  (`f2d212c`).

### Changed

- **Phase A audit closed**. The remaining Phase A findings were all
  worked through in commits `0d8918a` (7 trivial findings closed),
  `cc3e46a` (SYS.F29: sysfs attributes moved behind a single
  `attribute_group`, -67 lines), `0cd9fc7` (SYS.F41: extract
  `hidpp_errno` helper, -21 lines across 14 call sites), `934efb7`
  (SYS.F40: document the `params[2] = 0` padding convention), and
  `25fb739` (SYS.F21: split `rs50_ff_discover_features` into settings
  and LIGHTSYNC halves). The remaining strategic items (god-struct
  split, table-drive the settings handlers) were explicitly deferred
  with rationale recorded in `dev/docs/plans/STATUS.md`.
- **Protocol spec (`docs/RS50_PROTOCOL_SPECIFICATION.md`) bumped to
  v6.1**, rescoped to cover both RS50 and G Pro, D-pad rewritten from
  4-way to 8-way, per-feature SET function numbers tabulated,
  centre-calibration section added.
- **TRUEFORCE doc** rewritten from "research only" to the current
  implementation state, including the library layout, the two-pass
  init, and the wheel-coverage table.
- **SYSFS API, README, RS50_SUPPORT** brought in sync with the code.
- **USB_CAPTURE_GUIDE** broadened from "G Pro-specific" to "any
  Logitech wheel beyond the two we already support", with references
  to the `tools/windows_*_captures.bat` scripts and updated protocol
  background.

### Documentation

- New `userspace/libtrueforce/tests/unit.c` covering the wire-format
  conversions with a 65536-sample monotonicity sweep.
- Phase B gap analysis (`dev/docs/plans/2026-04-16-windows-gap-analysis.md`)
  and the Phase A audit (`dev/docs/plans/2026-04-16-code-audit.md`)
  are archived; `dev/docs/plans/STATUS.md` maps each rank and finding
  ID to its current shipping state.

## v0.9-pre-simplification (2026-02-02)

Tagged snapshot before the simplification + audit sprint. RS50-only,
FFB constant force via the existing `rs50_ff_*` path, basic sysfs
settings, LIGHTSYNC per-slot writes. See `git log
v0.9-pre-simplification` for the full history up to that point.

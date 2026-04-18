# libtrueforce

Native Linux implementation of the Logitech Trueforce SDK
(`trueforce_sdk_x64.dll`, version 1.3.11). Built for the RS50 wheel
family; G Pro support is a separate workstream.

The library talks to the wheel's interface-2 hidraw node directly; no
custom kernel driver is required beyond what the in-tree
`hid-logitech-hidpp` fork already provides. See the parent repo's
`dev/docs/plans/2026-04-16-roadmap-and-trueforce-design.md` for the
overall architecture.

## Status

**Phase 22.1 (skeleton + discovery).** The 59 public entry points are
declared and scannable, but only discovery, pause/resume flags, and
the version getter return meaningful values. Torque / streaming /
damping calls return `LOGITF_ERR_NOT_SUPPORTED`. Subsequent phases
will fill those in.

## Build

```bash
make               # builds libtrueforce.so.1.3.11 and tests/discover
sudo make install  # installs library + header under PREFIX (default /usr/local)
sudo make udev-install   # installs the hidraw access rule
```

After the udev rule is installed and udev is reloaded, unplug and
replug the wheel so the rule applies.

```bash
sudo udevadm control --reload
sudo udevadm trigger
```

Verify discovery:

```bash
./tests/discover
```

Expected output when a single RS50 is connected:

```
libtrueforce 1.3.11
  [0] available: supported=yes, paused=no
```

## udev rule

`udev/99-logitech-rs50-trueforce.rules` grants read/write access on
`/dev/hidrawN` for interface 2 of VID:PID `046d:c276` to users in the
`input` group, and also tags it with `uaccess` so logind-managed
sessions get access automatically on login.

## API

`include/trueforce.h` mirrors the 59 named exports of the Windows
SDK. The Wine PE shim (Rank 23, coming next) translates Windows ABI
calls (HANDLE, GUID, wide strings) into calls on this library.

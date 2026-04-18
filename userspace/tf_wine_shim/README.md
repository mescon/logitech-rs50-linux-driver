# tf_wine_shim

Wine PE shim that replaces `trueforce_sdk_x64.dll` inside a Proton
prefix so a Windows game's Trueforce SDK calls reach our native
Linux `libtrueforce.so` instead of the real (Windows-only) DLL.

The genuine Windows SDK talks to the G HUB Agent over a named pipe.
There's no G HUB Agent on Linux; this shim forwards directly to our
library, which in turn writes to `/dev/hidrawN` and
`/dev/input/eventN`.

## Status

**Phase 23.1 (scaffolding)**. Builds cleanly with `winegcc`; exports
all 75 ordinals at the right names and signatures. End-to-end
gameplay verification (BeamNG / AC Evo under Proton) is pending:
that needs Phase 23.2 (DirectInput handle → controller-index
mapping refinement) and a real game run.

## Prerequisites

- Wine dev tools (`winegcc`, `winebuild`). Fedora: `wine-devel`;
  Debian/Ubuntu: `wine-dev`.
- `libtrueforce.so` built and on the Wine prefix's `LD_LIBRARY_PATH`
  (or at a system location like `/usr/local/lib`).

## Build

```bash
# Build the Linux library first.
(cd ../libtrueforce && make && sudo make install)

# Then the Wine shim.
make
```

Produces `trueforce_sdk.dll.so`.

## Deploy to a Proton prefix

Find your game's Proton prefix (usually
`~/.steam/steam/steamapps/compatdata/<appid>/pfx`) and drop the shim
into `system32/`:

```bash
PREFIX=~/.steam/steam/steamapps/compatdata/<appid>/pfx
cp trueforce_sdk.dll.so $PREFIX/drive_c/windows/system32/trueforce_sdk.dll
```

Then set the DLL override so Wine loads our shim instead of the
game's bundled DLL:

```bash
export WINEDLLOVERRIDES="trueforce_sdk=n,b"
```

For Steam games, add that to the game's "Launch Options" as:

```
WINEDLLOVERRIDES="trueforce_sdk=n,b" %command%
```

## Verify

Inside the Proton prefix, use `winedbg` or run the game with the
Wine log enabled (`WINEDEBUG=+loaddll,+module`). You should see:

```
trace:loaddll:... trueforce_sdk.dll.so (owned by Wine)
```

If your game's launch log shows it loaded the bundled `.dll` instead,
the override didn't take effect; check your env var or Steam launch
options.

## Safety

See `../libtrueforce/README.md#safety` - same rules apply: the wheel
may rotate on power-up / profile change / first TF-setter call.
Brace before launching a game the first time.

#!/usr/bin/env bash
#
# Update the DKMS-installed hid-logitech-hidpp module from the current
# repo checkout. Copies mainline/ into /usr/src/hid-logitech-hidpp-1.0/,
# removes any previous DKMS state for that version, and installs the
# freshly built module. Does NOT unload the running module — reload it
# manually (see the final message) once the wheel is free.
#
# Usage: sudo ./tools/dkms-update.sh
#
# Written for contributors iterating on fixes (in particular #8) who
# otherwise end up typing the full dkms-remove / rm -rf / cp / build /
# install dance every time.

set -euo pipefail

PKG="hid-logitech-hidpp"
VER="1.0"
SRC_DIR="/usr/src/${PKG}-${VER}"
REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
REPO_SRC="$REPO_ROOT/mainline"
UDEV_SRC="$REPO_ROOT/udev/70-logitech-rs50.rules"
UDEV_DST="/etc/udev/rules.d/70-logitech-rs50.rules"

if [ "$EUID" -ne 0 ]; then
	echo "error: run as root (sudo $0)" >&2
	exit 1
fi

if [ ! -d "$REPO_SRC" ]; then
	echo "error: cannot find mainline/ at $REPO_SRC" >&2
	exit 1
fi

echo "== updating $SRC_DIR from $REPO_SRC =="
rm -rf "$SRC_DIR"
mkdir -p "$SRC_DIR"
cp -r "$REPO_SRC/." "$SRC_DIR/"

# Drop previous DKMS state for this version. Ignore "not found".
dkms remove -m "$PKG" -v "$VER" --all >/dev/null 2>&1 || true

echo "== dkms install -m $PKG -v $VER =="
dkms install -m "$PKG" -v "$VER"

# Install / refresh udev rule so wheel_* sysfs attrs and hidraw nodes
# are writable by the logged-in session user (or members of "input"),
# not just root. Without this every Oversteer knob and every echo >
# wheel_* needs sudo.
if [ -f "$UDEV_SRC" ]; then
	if ! cmp -s "$UDEV_SRC" "$UDEV_DST" 2>/dev/null; then
		echo "== installing udev rule to $UDEV_DST =="
		install -m 0644 "$UDEV_SRC" "$UDEV_DST"
		udevadm control --reload
		udevadm trigger --subsystem-match=hidraw
	else
		echo "udev rule up to date ($UDEV_DST)"
	fi
fi

cat <<'EOF'

Module installed. To pick it up without a reboot:

  1) Unplug the wheel (or close anything holding the evdev / hidraw
     device open — e.g. fftest, games, browser tabs with Gamepad API)
  2) sudo modprobe -r hid-logitech-hidpp
  3) sudo modprobe hid-logitech-hidpp
  4) Plug the wheel back in

If modprobe -r reports "Module is in use", something still has the
device open. Find it with:  sudo fuser -v /dev/input/event* /dev/hidraw*

On UEFI Secure Boot systems, DKMS should re-sign the module with your
MOK key automatically. If load fails with "Key was rejected by
service", re-enroll the MOK and reboot once.
EOF

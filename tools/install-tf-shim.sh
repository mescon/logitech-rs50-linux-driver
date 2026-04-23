#!/usr/bin/env bash
#
# Install the Logitech TrueForce SDK shim so Proton / wine games that
# load the SDK via its COM CLSID (ACC, AMS2, iRacing, etc.) find our
# libtrueforce-backed replacement instead of failing silently.
#
# What this does:
#   1. Build + install libtrueforce.so.1 and trueforce_sdk_x64.dll.so
#      into /usr/lib/logi-tf-shim/ (plus an ld.so.conf.d entry so the
#      dynamic linker finds the native lib from inside the wine DLL).
#   2. For each target wine prefix, set the registry value
#        HKLM\SOFTWARE\Classes\CLSID\{e8dfb59f-...} default = Z:\usr\lib\logi-tf-shim\trueforce_sdk_x64.dll
#      in system.reg. When the game's embedded Logi SDK does its
#      LoadLibrary-by-CLSID-lookup dance, it finds our shim at that
#      path, GetProcAddress's the logi* + logiTrueForce* surface, and
#      the game's TrueForce code path starts sending torque packets
#      to our kernel driver via libtrueforce.
#
# Usage:
#   sudo ./tools/install-tf-shim.sh --all-steam           # install into every Steam prefix
#   sudo ./tools/install-tf-shim.sh --prefix <path>       # install into one prefix
#   sudo ./tools/install-tf-shim.sh --uninstall           # remove everything
#
# Safe to re-run; registry insert is idempotent.

set -euo pipefail

CLSID='{e8dfb59f-141f-40e4-8dd4-5526ead25a4c}'
LIB_DIR='/usr/lib/logi-tf-shim'
DLL_WINE_PATH='Z:\\usr\\lib\\logi-tf-shim\\trueforce_sdk_x64.dll'
LDCONF='/etc/ld.so.conf.d/logi-tf-shim.conf'

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SHIM_DIR="$REPO_ROOT/userspace/tf_wine_shim"
LIBTF_DIR="$REPO_ROOT/userspace/libtrueforce"
BUILT_SHIM="$SHIM_DIR/trueforce_sdk.dll.so"

usage() {
	cat <<EOF
Usage:
  $0 --all-steam               Install into every Steam wine prefix under ~/.local/share/Steam
  $0 --prefix <path>           Install into a single wine prefix (path to the .../pfx directory)
  $0 --uninstall               Remove registry entries from all Steam prefixes, delete $LIB_DIR
EOF
	exit 1
}

need_root() {
	if [ "$EUID" -ne 0 ]; then
		echo "error: this step needs root (rerun with sudo)" >&2
		exit 1
	fi
}

build_if_needed() {
	local need_libtf=0 need_shim=0

	# Glob can match multiple files (so.1, so.1.3.11), can't use [ -f ]
	compgen -G "$LIBTF_DIR/libtrueforce.so.1.*" >/dev/null || need_libtf=1
	[ -f "$BUILT_SHIM" ] || need_shim=1

	if [ "$need_libtf" = 1 ]; then
		echo "building libtrueforce..."
		make -C "$LIBTF_DIR" >&2
	fi
	if [ "$need_shim" = 1 ]; then
		command -v winegcc >/dev/null || {
			echo "error: winegcc not found. Install wine-devel (Fedora), wine-dev (Debian), or wine (Arch)." >&2
			exit 1
		}
		echo "building wine shim..."
		make -C "$SHIM_DIR" >&2
	fi
}

install_system_files() {
	need_root
	mkdir -p "$LIB_DIR"

	# Shim DLL. The .dll.so is the real file, the .dll symlink lets Wine's
	# loader find it when the registry path says "trueforce_sdk_x64.dll".
	install -m 0755 "$BUILT_SHIM" "$LIB_DIR/trueforce_sdk_x64.dll.so"
	ln -sf trueforce_sdk_x64.dll.so "$LIB_DIR/trueforce_sdk_x64.dll"

	# libtrueforce.so and its SONAMEs, as produced by make in libtrueforce/
	local full
	full=$(basename "$(ls "$LIBTF_DIR"/libtrueforce.so.1.*)")
	install -m 0755 "$LIBTF_DIR/$full" "$LIB_DIR/$full"
	ln -sf "$full" "$LIB_DIR/libtrueforce.so.1"
	ln -sf libtrueforce.so.1 "$LIB_DIR/libtrueforce.so"

	# Tell ld.so where to find libtrueforce.so.1 at runtime, then refresh the cache
	echo "$LIB_DIR" > "$LDCONF"
	ldconfig

	echo "installed files under $LIB_DIR"
}

# Append / replace the CLSID entry in a wine prefix's system.reg.
# Direct file edit avoids depending on the prefix's own wine binary.
register_in_prefix() {
	local prefix="$1"
	local sys_reg="$prefix/system.reg"

	if [ ! -f "$sys_reg" ]; then
		echo "  skip $prefix (no system.reg)" >&2
		return 0
	fi

	python3 - "$sys_reg" "$CLSID" "$DLL_WINE_PATH" <<'PY'
import os, sys, time
reg_path, clsid, dllpath = sys.argv[1], sys.argv[2], sys.argv[3]
# Wine stores keys as e.g. [Software\\Classes\\CLSID\\{guid}] with
# literal double backslashes; the file itself uses single backslashes
# at the binary level but ConfigParser-style text here has them doubled.
key_header_prefix = f"[Software\\\\Classes\\\\CLSID\\\\{clsid}]"

with open(reg_path, "r") as f:
    lines = f.readlines()

out = []
skip = False
for line in lines:
    if line.startswith(key_header_prefix):
        skip = True
        continue
    if skip:
        # block ends at the first blank line
        if line.strip() == "":
            skip = False
        continue
    out.append(line)

# Ensure trailing newline so the appended block separates cleanly
if out and not out[-1].endswith("\n"):
    out[-1] += "\n"
if out and out[-1].strip() != "":
    out.append("\n")

ts = int(time.time())
out.append(f"{key_header_prefix} {ts}\n")
out.append(f'@="{dllpath}"\n')
out.append("\n")

# atomic replace
tmp = reg_path + ".new"
with open(tmp, "w") as f:
    f.writelines(out)
os.replace(tmp, reg_path)
PY
	echo "  registered $prefix"
}

unregister_in_prefix() {
	local prefix="$1"
	local sys_reg="$prefix/system.reg"
	[ -f "$sys_reg" ] || return 0
	python3 - "$sys_reg" "$CLSID" <<'PY'
import os, sys
reg_path, clsid = sys.argv[1], sys.argv[2]
key_header_prefix = f"[Software\\\\Classes\\\\CLSID\\\\{clsid}]"
with open(reg_path, "r") as f:
    lines = f.readlines()
out = []; skip = False
for line in lines:
    if line.startswith(key_header_prefix):
        skip = True; continue
    if skip:
        if line.strip() == "":
            skip = False
        continue
    out.append(line)
tmp = reg_path + ".new"
with open(tmp, "w") as f: f.writelines(out)
os.replace(tmp, reg_path)
PY
	echo "  unregistered $prefix"
}

install_all_steam() {
	local count=0
	for pfx in "$HOME"/.local/share/Steam/steamapps/compatdata/*/pfx; do
		[ -d "$pfx" ] || continue
		register_in_prefix "$pfx"
		count=$((count+1))
	done
	# When invoked via sudo, $HOME is root's. Also try the invoking user.
	if [ -n "${SUDO_USER:-}" ]; then
		local user_home
		user_home=$(getent passwd "$SUDO_USER" | cut -d: -f6)
		for pfx in "$user_home"/.local/share/Steam/steamapps/compatdata/*/pfx; do
			[ -d "$pfx" ] || continue
			register_in_prefix "$pfx"
			count=$((count+1))
		done
	fi
	echo "registered shim in $count Steam prefix(es)"
}

uninstall_all_steam() {
	for pfx in "$HOME"/.local/share/Steam/steamapps/compatdata/*/pfx; do
		[ -d "$pfx" ] && unregister_in_prefix "$pfx"
	done
	if [ -n "${SUDO_USER:-}" ]; then
		local user_home
		user_home=$(getent passwd "$SUDO_USER" | cut -d: -f6)
		for pfx in "$user_home"/.local/share/Steam/steamapps/compatdata/*/pfx; do
			[ -d "$pfx" ] && unregister_in_prefix "$pfx"
		done
	fi
}

case "${1:-}" in
--all-steam)
	build_if_needed
	install_system_files
	install_all_steam
	;;
--prefix)
	[ -n "${2:-}" ] || usage
	build_if_needed
	install_system_files
	register_in_prefix "$2"
	;;
--uninstall)
	need_root
	uninstall_all_steam
	rm -rf "$LIB_DIR" "$LDCONF"
	ldconfig
	echo "removed $LIB_DIR and CLSID entries"
	;;
*) usage ;;
esac

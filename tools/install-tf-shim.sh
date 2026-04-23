#!/usr/bin/env bash
#
# Install the Logitech TrueForce SDK shim into wine prefixes so Proton
# games (ACC, AMS2, iRacing, ...) that load the SDK via its CLSID find
# our libtrueforce-backed replacement.
#
# What this does:
#   1. Build the wine PE shim (libtrueforce is linked statically in,
#      so the shim has no Linux shared-lib dependency).
#   2. For each target wine prefix, drop the shim DLL into
#         <prefix>/drive_c/logi-tf-shim/trueforce_sdk_x64.dll
#      and set the registry value
#         HKLM\SOFTWARE\Classes\CLSID\{e8dfb59f-...} default = C:\logi-tf-shim\trueforce_sdk_x64.dll
#      via direct edit of system.reg (no wine binary required).
#
# Per-prefix install is required because Proton's pressure-vessel
# sandbox doesn't expose the host's /usr/lib to the game, so anything
# under a system path is invisible to ACC and friends. Inside each
# prefix's drive_c, the file is plainly visible to the game.
#
# Usage:
#   ./tools/install-tf-shim.sh --all-steam       Install in every Steam prefix
#   ./tools/install-tf-shim.sh --prefix <path>   Install in one prefix
#   ./tools/install-tf-shim.sh --uninstall       Remove from all Steam prefixes
#
# Run as the user that owns the wine prefix (do not sudo). Idempotent;
# safe to re-run.

set -euo pipefail

CLSID='{e8dfb59f-141f-40e4-8dd4-5526ead25a4c}'
DLL_BASENAME='trueforce_sdk_x64.dll'
# Path inside the wine prefix's drive_c
PREFIX_REL_DIR='drive_c/logi-tf-shim'
DLL_WINE_PATH='C:\\logi-tf-shim\\trueforce_sdk_x64.dll'

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SHIM_DIR="$REPO_ROOT/userspace/tf_wine_shim"
LIBTF_DIR="$REPO_ROOT/userspace/libtrueforce"
BUILT_SHIM="$SHIM_DIR/trueforce_sdk.dll.so"

usage() {
	cat <<EOF
Usage:
  $0 --all-steam               Install into every Steam wine prefix under ~/.local/share/Steam
  $0 --prefix <path>           Install into a single wine prefix (the .../pfx directory)
  $0 --uninstall               Remove the shim + registry entry from all Steam prefixes
EOF
	exit 1
}

build_if_needed() {
	if [ ! -f "$BUILT_SHIM" ] || [ "$LIBTF_DIR/src/exports.c" -nt "$BUILT_SHIM" ] \
		|| [ "$SHIM_DIR/src/shim.c" -nt "$BUILT_SHIM" ]; then
		command -v winegcc >/dev/null || {
			echo "error: winegcc not found. Install wine-devel (Fedora) / wine-dev (Debian) / wine (Arch)." >&2
			exit 1
		}
		echo "building wine shim (libtrueforce.a + trueforce_sdk.dll.so)..."
		make -C "$SHIM_DIR" >&2
	fi
}

install_in_prefix() {
	local prefix="$1"
	local sys_reg="$prefix/system.reg"

	if [ ! -f "$sys_reg" ]; then
		echo "  skip $prefix (no system.reg)" >&2
		return 0
	fi

	# Drop the shim file inside drive_c
	local target_dir="$prefix/$PREFIX_REL_DIR"
	mkdir -p "$target_dir"
	# wine recognises the .dll.so ELF as a wine PE shim regardless of
	# extension; we install it under the .dll name the registry points at
	install -m 0644 "$BUILT_SHIM" "$target_dir/$DLL_BASENAME"

	# Registry: replace any existing CLSID block, then append the fresh one
	python3 - "$sys_reg" "$CLSID" "$DLL_WINE_PATH" <<'PY'
import os, sys, time
reg_path, clsid, dllpath = sys.argv[1], sys.argv[2], sys.argv[3]
key_header = f"[Software\\\\Classes\\\\CLSID\\\\{clsid}]"

with open(reg_path) as f:
    lines = f.readlines()

out = []
skip = False
for line in lines:
    if line.startswith(key_header):
        skip = True
        continue
    if skip:
        if line.strip() == "":
            skip = False
        continue
    out.append(line)

if out and not out[-1].endswith("\n"):
    out[-1] += "\n"
if out and out[-1].strip() != "":
    out.append("\n")

ts = int(time.time())
out.append(f"{key_header} {ts}\n")
out.append(f'@="{dllpath}"\n')
out.append("\n")

tmp = reg_path + ".new"
with open(tmp, "w") as f:
    f.writelines(out)
os.replace(tmp, reg_path)
PY
	echo "  installed $prefix"
}

uninstall_in_prefix() {
	local prefix="$1"
	local sys_reg="$prefix/system.reg"
	[ -d "$prefix/$PREFIX_REL_DIR" ] && rm -rf "$prefix/$PREFIX_REL_DIR"
	[ -f "$sys_reg" ] || return 0
	python3 - "$sys_reg" "$CLSID" <<'PY'
import os, sys
reg_path, clsid = sys.argv[1], sys.argv[2]
key_header = f"[Software\\\\Classes\\\\CLSID\\\\{clsid}]"
with open(reg_path) as f: lines = f.readlines()
out = []; skip = False
for line in lines:
    if line.startswith(key_header):
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
	echo "  uninstalled $prefix"
}

steam_prefixes() {
	echo "$HOME"/.local/share/Steam/steamapps/compatdata/*/pfx
}

case "${1:-}" in
--all-steam)
	build_if_needed
	count=0
	for pfx in $(steam_prefixes); do
		[ -d "$pfx" ] || continue
		install_in_prefix "$pfx"
		count=$((count+1))
	done
	echo "installed in $count Steam prefix(es)"
	;;
--prefix)
	[ -n "${2:-}" ] || usage
	build_if_needed
	install_in_prefix "$2"
	;;
--uninstall)
	for pfx in $(steam_prefixes); do
		[ -d "$pfx" ] && uninstall_in_prefix "$pfx"
	done
	;;
*) usage ;;
esac

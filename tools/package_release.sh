#!/usr/bin/env bash
# This Source Code Form is subject to the terms of the
# Mozilla Public License, v. 2.0. If a copy of the MPL was
# not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

# A release's downloads, in release/: the addon with all eight
# libraries, the patched double Godot editors (Linux, Windows)
# and their export templates as a .tpz. Run build_all.sh first.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
version="${1:?usage: package_release.sh VERSION [GODOT_SOURCE]}"
godot="${2:-$root/engine/godot}"
bin="$godot/bin"
gv="4.7.2.stable.double"
out="$root/release"
stage="$out/stage"

addon="$root/dist/addons/regolith"
[ -d "$root/../addons/regolith" ] && addon="$root/../addons/regolith"

need=(godot.linuxbsd.editor.double.x86_64 godot.linuxbsd.template_debug.double.x86_64
	godot.linuxbsd.template_release.double.x86_64 godot.windows.editor.double.x86_64.exe
	godot.windows.editor.double.x86_64.console.exe godot.windows.template_debug.double.x86_64.exe
	godot.windows.template_debug.double.x86_64.console.exe godot.windows.template_release.double.x86_64.exe
	godot.windows.template_release.double.x86_64.console.exe)
for f in "${need[@]}"; do
	[ -f "$bin/$f" ] || { echo "missing $bin/$f" >&2; exit 1; }
done
[ "$(ls "$addon/bin" | wc -l)" -eq 8 ] || { echo "not all 8 libraries in $addon/bin" >&2; exit 1; }

rm -rf "$stage"
mkdir -p "$stage/addon" "$stage/linux" "$stage/windows" "$stage/tpz/templates"

zipdir() { # zip the contents of $1 into $2, paths relative to $1
	(cd "$1" && python3 -m zipfile -c "$2" ./*)
}

# the addon, without editor leftovers
mkdir -p "$stage/addon/addons"
cp -r "$addon" "$stage/addon/addons/regolith"
find "$stage/addon" \( -name "*.uid" -o -name "*.import" \) -delete
zipdir "$stage/addon" "$out/regolith-$version-addon.zip"

# the editors, with Godot's licence (MIT)
for d in linux windows; do
	cp "$godot/LICENSE.txt" "$godot/COPYRIGHT.txt" "$stage/$d/"
done
strip -o "$stage/linux/godot.linuxbsd.editor.double.x86_64" "$bin/godot.linuxbsd.editor.double.x86_64"
cp "$bin"/godot.windows.editor.double.x86_64*.exe "$stage/windows/"
zipdir "$stage/linux" "$out/godot-$gv-regolith-linux-editor.zip"
zipdir "$stage/windows" "$out/godot-$gv-regolith-windows-editor.zip"

# export templates, named as the editor expects; symbols stripped
t="$stage/tpz/templates"
strip -o "$t/linux_debug.x86_64" "$bin/godot.linuxbsd.template_debug.double.x86_64"
strip -o "$t/linux_release.x86_64" "$bin/godot.linuxbsd.template_release.double.x86_64"
cp "$bin/godot.windows.template_debug.double.x86_64.exe" "$t/windows_debug_x86_64.exe"
cp "$bin/godot.windows.template_debug.double.x86_64.console.exe" "$t/windows_debug_x86_64_console.exe"
cp "$bin/godot.windows.template_release.double.x86_64.exe" "$t/windows_release_x86_64.exe"
cp "$bin/godot.windows.template_release.double.x86_64.console.exe" "$t/windows_release_x86_64_console.exe"
printf '%s\n' "$gv" > "$t/version.txt"
zipdir "$stage/tpz" "$out/godot-$gv-regolith-export-templates.tpz"

rm -rf "$stage"
ls -la "$out"

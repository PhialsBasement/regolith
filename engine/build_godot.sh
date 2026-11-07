#!/usr/bin/env bash
# Godot 4.7.2 with Regolith's renderer patches, double
# precision: the editor and this platform's export templates.
# Usage: engine/build_godot.sh [DEST]  (see engine/README.md)
set -euo pipefail

GODOT_TAG="4.7.2-stable"
GODOT_REPO="https://github.com/godotengine/godot.git"
here="$(cd "$(dirname "$0")" && pwd)"
dest="${1:-$here/godot}"
jobs="${JOBS:-$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)}"

die() { echo "build_godot: $*" >&2; exit 1; }

for tool in git python3 scons; do
	command -v "$tool" >/dev/null || die "$tool is required"
done

case "$(uname -s)" in
	Linux) platform=linuxbsd; tdir="${XDG_DATA_HOME:-$HOME/.local/share}/godot/export_templates" ;;
	Darwin) platform=macos; tdir="$HOME/Library/Application Support/Godot/export_templates" ;;
	*) die "use build_godot.ps1 on Windows" ;;
esac
case "$(uname -m)" in
	x86_64 | amd64) arch=x86_64 ;;
	aarch64 | arm64) arch=arm64 ;;
	*) die "unsupported architecture $(uname -m)" ;;
esac

if [ ! -d "$dest/.git" ]; then
	echo "== cloning Godot $GODOT_TAG into $dest"
	git clone --depth 1 --branch "$GODOT_TAG" "$GODOT_REPO" "$dest"
fi
cd "$dest"
# the patches are made for this one tag
[ "$(git describe --tags --exact-match HEAD 2>/dev/null)" = "$GODOT_TAG" ] ||
	die "$dest is at $(git describe --tags --always), expected $GODOT_TAG"

echo "== patching"
for p in "$here"/patches/*.patch; do
	name="$(basename "$p")"
	if git apply --reverse --check "$p" 2>/dev/null; then
		echo "   already applied: $name"
	elif git apply --check "$p"; then
		git apply "$p"
		echo "   applied: $name"
	else
		die "$name does not apply; start from a clean $GODOT_TAG checkout"
	fi
done

export BUILD_NAME=regolith
common=(platform="$platform" arch="$arch" precision=double -j"$jobs")
for target in editor template_debug template_release; do
	echo "== building $target"
	scons "${common[@]}" target="$target"
done

# where the editor looks: <templates>/4.7.2.stable.double/
version="${GODOT_TAG%-stable}.stable.double"
mkdir -p "$tdir/$version"
if [ "$platform" = linuxbsd ]; then
	cp "bin/godot.linuxbsd.template_debug.double.$arch" "$tdir/$version/linux_debug.$arch"
	cp "bin/godot.linuxbsd.template_release.double.$arch" "$tdir/$version/linux_release.$arch"
	editor="bin/godot.linuxbsd.editor.double.$arch"
else
	echo "   macOS templates: package bin/ as in Godot's docs"
	editor="bin/godot.macos.editor.double.$arch"
fi
printf '%s\n' "$version" > "$tdir/$version/version.txt"

echo
echo "Done."
echo "  editor:    $dest/$editor"
echo "  templates: $tdir/$version"
echo "Open your project with that editor; Regolith runs at full speed."

#!/usr/bin/env bash
# This Source Code Form is subject to the terms of the
# Mozilla Public License, v. 2.0. If a copy of the MPL was
# not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

# Every library regolith.gdextension lists: Linux and Windows,
# debug (editor) and release (exports), double and single.
# Windows cross-compiles with MinGW-w64 and needs zstd sources.
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
jobs="${JOBS:-$(nproc)}"
# zstd for Windows: Godot's own copy (build_godot.sh clones it)
zstd_src="${ZSTD_SRC:-$root/engine/godot/thirdparty/zstd}"
platforms="${PLATFORMS:-linux windows}"

configure_and_build() {
	local platform="$1" target="$2" precision="$3"
	local dir="$root/build-$platform-$target-$precision"
	local extra=()
	if [ "$platform" = windows ]; then
		if [ ! -d "$zstd_src" ]; then
			echo "no zstd sources at $zstd_src: set ZSTD_SRC" >&2
			exit 1
		fi
		extra=(-DCMAKE_TOOLCHAIN_FILE="$root/cmake/mingw-w64.cmake" -DREGOLITH_ZSTD_SOURCE="$zstd_src"
			-DREGOLITH_BUILD_TESTS=OFF -DREGOLITH_BUILD_BAKER=OFF)
	fi
	echo "== $platform $target $precision"
	cmake -S "$root" -B "$dir" -G Ninja -DCMAKE_BUILD_TYPE=Release \
		-DGODOTCPP_TARGET="$target" -DGODOTCPP_PRECISION="$precision" "${extra[@]}" > /dev/null
	cmake --build "$dir" -j "$jobs" --target regolith
}

for platform in $platforms; do
	for target in template_debug template_release; do
		for precision in double single; do
			configure_and_build "$platform" "$target" "$precision"
		done
	done
done
echo "done: $(ls "$root/../addons/regolith/bin" | wc -l) libraries in addons/regolith/bin"

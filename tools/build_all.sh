#!/usr/bin/env bash
# This Source Code Form is subject to the terms of the
# Mozilla Public License, v. 2.0. If a copy of the MPL was
# not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

# Every library regolith.gdextension lists: Linux and Windows,
# debug (editor) and release (exports), double and single.
# Windows cross-compiles with MinGW-w64 (zstd comes from its submodule).
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
jobs="${JOBS:-$(nproc)}"
platforms="${PLATFORMS:-linux windows}"

configure_and_build() {
	local platform="$1" target="$2" precision="$3"
	local dir="$root/build-$platform-$target-$precision"
	local extra=()
	if [ "$platform" = windows ]; then
		extra=(-DCMAKE_TOOLCHAIN_FILE="$root/cmake/mingw-w64.cmake"
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
addon="$root/dist/addons/regolith"
[ -d "$root/../addons/regolith" ] && addon="$root/../addons/regolith"
echo "done: $(ls "$addon/bin" | wc -l) libraries in $addon/bin"

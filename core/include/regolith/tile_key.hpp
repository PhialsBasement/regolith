// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cstdint>

namespace regolith {

// Packed as face:3 | level:5 | x:28 | y:28.
struct TileKey {
	static constexpr int MAX_LEVEL = 28;
	static constexpr uint64_t COORD_MASK = (uint64_t(1) << 28) - 1;

	uint64_t bits = 0;

	static constexpr TileKey make(int p_face, int p_level, uint32_t p_x, uint32_t p_y) {
		return { (uint64_t(p_face) << 61) | (uint64_t(p_level) << 56) | (uint64_t(p_x) << 28) | uint64_t(p_y) };
	}

	constexpr int face() const { return int(bits >> 61); }
	constexpr int level() const { return int((bits >> 56) & 31u); }
	constexpr uint32_t x() const { return uint32_t((bits >> 28) & COORD_MASK); }
	constexpr uint32_t y() const { return uint32_t(bits & COORD_MASK); }

	constexpr TileKey parent() const {
		return make(face(), level() - 1, x() >> 1, y() >> 1);
	}

	// p_quadrant: bit 0 picks the +x half, bit 1 the +y half.
	constexpr TileKey child(int p_quadrant) const {
		return make(face(), level() + 1, (x() << 1) | uint32_t(p_quadrant & 1), (y() << 1) | uint32_t(p_quadrant >> 1));
	}

	constexpr bool operator==(const TileKey &) const = default;
};

} // namespace regolith

#pragma once

#include <cstdint>
#include <vector>

#include "regolith/height_tile.hpp"
#include "regolith/tile_layout.hpp"
#include "regolith/vec.hpp"

namespace regolith {

// Horizon maps for terrain shadows (docs/shadows.md). Per tile,
// at every 4th sample (border 1, like the geology page), the
// horizon elevation angle in 8 grid directions:
// (1,0) (1,1) (0,1) (-1,1) (-1,0) (-1,-1) (0,-1) (1,-1).
// Two RGBA8 layers: directions 0-3, then 4-7.
constexpr int HORIZON_DIRS = 8;
constexpr double HORIZON_MIN = -0.25; // radians, byte 0
constexpr double HORIZON_SPAN = 1.25; // radians over 0..255
// Near march length in samples; each parent covers twice as far.
constexpr int HORIZON_PAD = 32;

inline int horizon_side(const TileLayout &p_lay) {
	return p_lay.intervals / 4 + 3;
}

// p_pos: body-fixed sample positions over [-pad, n + pad]^2,
// row-major, reaching into the neighbouring tiles. The far
// field comes from the parent's page (null at level 0).
void compute_horizon(const std::vector<DVec3> &p_pos, TileKey p_key, const TileLayout &p_lay,
		const std::vector<uint8_t> *p_parent, std::vector<uint8_t> &r_page);

} // namespace regolith

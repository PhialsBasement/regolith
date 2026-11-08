// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cmath>
#include <cstdio>

#include "regolith/horizon.hpp"
#include "regolith/synth.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

constexpr double MARS_RADIUS_M = 3389500.0;
const TileLayout LAYOUT{ 64, 2 };

constexpr double WALL_HEIGHT_M = 300.0;
// The wall stands this many samples past the tile's +i edge.
constexpr int WALL_PAST_EDGE = 10;

// Horizon directions stored in a page.
constexpr int TOWARD_WALL = 0;
constexpr int AWAY_FROM_WALL = 4;

constexpr double HORIZON_TOLERANCE_RAD = 0.006;

double decode_angle(uint8_t p_byte) {
	return HORIZON_MIN + double(p_byte) / 255.0 * HORIZON_SPAN;
}

// Horizon angle at page point (p_qi, p_qj) toward p_direction.
double page_angle(const std::vector<uint8_t> &p_page, int p_qi, int p_qj, int p_direction) {
	const size_t side = size_t(horizon_side(LAYOUT));
	const size_t plane = size_t(p_direction / 4);
	const size_t channel = size_t(p_direction % 4);
	const size_t texel = plane * side * side + size_t(p_qj) * side + size_t(p_qi);
	return decode_angle(p_page[texel * 4 + channel]);
}

// Positions over the tile and its horizon pad: a bare sphere
// with the wall beyond the +i edge, in the neighbouring tile.
std::vector<DVec3> sphere_with_wall(TileKey p_key, int p_wall_i) {
	const int first = -HORIZON_PAD;
	const int last = LAYOUT.intervals + HORIZON_PAD;
	const size_t width = size_t(last - first + 1);
	std::vector<DVec3> positions(width * width);
	for (int j = first; j <= last; ++j) {
		for (int i = first; i <= last; ++i) {
			const double height = i >= p_wall_i ? WALL_HEIGHT_M : 0.0;
			positions[size_t(j - first) * width + size_t(i - first)] =
					Synth::sample_dir(p_key, LAYOUT, i, j) * (MARS_RADIUS_M + height);
		}
	}
	return positions;
}

} // namespace

TEST_CASE(horizon_sees_a_wall_beyond_its_tile) {
	const TileKey key = TileKey::make(0, 9, 100, 100);
	const int wall_i = LAYOUT.intervals + WALL_PAST_EDGE;
	std::vector<uint8_t> page;
	compute_horizon(sphere_with_wall(key, wall_i), key, LAYOUT, nullptr, page);

	// Page point (qi, qj) is sample ((qi - 1) * 4, (qj - 1) * 4):
	// here sample (56, 32), 18 samples from the wall.
	const int qi = 15, qj = 9;
	const DVec3 viewer = Synth::sample_dir(key, LAYOUT, 56, 32) * MARS_RADIUS_M;
	const DVec3 wall_top = Synth::sample_dir(key, LAYOUT, wall_i, 32) * (MARS_RADIUS_M + WALL_HEIGHT_M);
	const DVec3 up = normalize(viewer);
	const double exact = std::asin(dot(wall_top - viewer, up) / length(wall_top - viewer));
	const double toward = page_angle(page, qi, qj, TOWARD_WALL);
	const double away = page_angle(page, qi, qj, AWAY_FROM_WALL);
	std::printf("  wall in the next tile: horizon %.4f rad, exact %.4f; away %.4f\n", toward, exact, away);
	CHECK(std::fabs(toward - exact) < HORIZON_TOLERANCE_RAD);
	// the curvature dip over a few km is under one step
	CHECK(away <= HORIZON_SPAN / 255.0);
}

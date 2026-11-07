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

constexpr double R = 3389500.0;
const TileLayout LAY{ 64, 2 };

double decode(uint8_t p_b) {
	return HORIZON_MIN + double(p_b) / 255.0 * HORIZON_SPAN;
}

// Horizon angle at page point (qi, qj) toward direction p_dir.
double page_at(const std::vector<uint8_t> &p_page, int p_qi, int p_qj, int p_dir) {
	const int side = horizon_side(LAY);
	return decode(p_page[(size_t(p_dir / 4) * side * side + size_t(p_qj) * side + size_t(p_qi)) * 4 + size_t(p_dir % 4)]);
}

} // namespace

TEST_CASE(horizon_sees_a_wall_beyond_its_tile) {
	// Bare sphere, with a 300 m wall 10 samples past the +i edge,
	// in the neighbouring tile.
	const TileKey key = TileKey::make(0, 9, 100, 100);
	const int n = LAY.intervals, lo = -HORIZON_PAD, hi = n + HORIZON_PAD, w = hi - lo + 1;
	const int wall = n + 10;
	std::vector<DVec3> pos(size_t(w) * w);
	for (int j = lo; j <= hi; ++j) {
		for (int i = lo; i <= hi; ++i) {
			pos[size_t(j - lo) * w + size_t(i - lo)] = Synth::sample_dir(key, LAY, i, j) * (R + (i >= wall ? 300.0 : 0.0));
		}
	}
	std::vector<uint8_t> page;
	compute_horizon(pos, key, LAY, nullptr, page);
	// Page point (qi, qj) = sample ((qi-1)*4, (qj-1)*4).
	const int qi = 15, qj = 9; // sample (56, 32), 18 samples from it
	const DVec3 p = Synth::sample_dir(key, LAY, 56, 32) * R;
	const DVec3 q = Synth::sample_dir(key, LAY, wall, 32) * (R + 300.0);
	const DVec3 up = normalize(p);
	const double want = std::asin(dot(q - p, up) / length(q - p));
	const double toward = page_at(page, qi, qj, 0);
	const double away = page_at(page, qi, qj, 4);
	std::printf("  wall in the next tile: horizon %.4f rad, exact %.4f; away %.4f\n", toward, want, away);
	CHECK(std::fabs(toward - want) < 0.006);
	// the curvature dip over a few km is under one step
	CHECK(away <= HORIZON_SPAN / 255.0);
}

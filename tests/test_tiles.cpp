// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cmath>

#include "regolith/cube.hpp"
#include "regolith/height_tile.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

const TileLayout LAYOUT{ 16, 2 };

// A tile filled from a smooth function of position.
HeightTile smooth_tile(TileKey p_key) {
	HeightTile tile;
	tile.key = p_key;
	tile.rho.resize(size_t(LAYOUT.count()));
	const int64_t edge_intervals = int64_t(LAYOUT.intervals) << p_key.level();
	const int64_t first_i = int64_t(p_key.x()) * LAYOUT.intervals;
	const int64_t first_j = int64_t(p_key.y()) * LAYOUT.intervals;
	for (int j = -LAYOUT.border; j <= LAYOUT.intervals + LAYOUT.border; ++j) {
		for (int i = -LAYOUT.border; i <= LAYOUT.intervals + LAYOUT.border; ++i) {
			const IVec3 point = fold_lattice(p_key.face(), first_i + i, first_j + j, edge_intervals);
			const DVec3 dir = lattice_dir(point, edge_intervals);
			tile.rho[size_t(LAYOUT.index(i, j))] = float(800.0 * std::sin(40.0 * dir.x) + 300.0 * dir.y * dir.z);
		}
	}
	return tile;
}

// A child's even samples are its parent's samples, unchanged.
void check_even_samples(const HeightTile &p_parent, const HeightTile &p_child, int p_quadrant) {
	const int first_parent_i = (p_quadrant & 1) * LAYOUT.intervals;
	const int first_parent_j = (p_quadrant >> 1) * LAYOUT.intervals;
	for (int j = 0; j <= LAYOUT.intervals; j += 2) {
		for (int i = 0; i <= LAYOUT.intervals; i += 2) {
			const int parent_i = (first_parent_i + i) / 2;
			const int parent_j = (first_parent_j + j) / 2;
			CHECK(p_child.at(LAYOUT, i, j) == p_parent.at(LAYOUT, parent_i, parent_j));
		}
	}
}

// Subdivides the two tiles meeting on the edge between face 0
// (+X) and face 2 (+Y) and compares the shared column.
int compare_subdivided_edge(const HeightTile &p_face0_tile, const HeightTile &p_face2_tile, int p_child_row) {
	HeightTile face0_child, face2_child;
	subdivide(p_face0_tile, LAYOUT, 1 | (p_child_row << 1), face0_child);
	subdivide(p_face2_tile, LAYOUT, 0 | (p_child_row << 1), face2_child);
	for (int j = 0; j <= LAYOUT.intervals; ++j) {
		CHECK(face0_child.at(LAYOUT, LAYOUT.intervals, j) == face2_child.at(LAYOUT, 0, j));
	}
	return LAYOUT.intervals + 1;
}

} // namespace

TEST_CASE(subdivision_keeps_even_samples) {
	const HeightTile parent = smooth_tile(TileKey::make(2, 3, 5, 6));
	for (int quadrant = 0; quadrant < 4; ++quadrant) {
		HeightTile child;
		subdivide(parent, LAYOUT, quadrant, child);
		CHECK(child.key == parent.key.child(quadrant));
		check_even_samples(parent, child, quadrant);
	}
}

TEST_CASE(subdivided_face_edges_are_bit_identical) {
	// Face 0's s = +1 column meets face 2's s = -1 column,
	// at the same t.
	const int level = 2;
	const uint32_t last = (1u << level) - 1;
	int compared = 0;
	for (uint32_t tile_y = 0; tile_y <= last; ++tile_y) {
		const HeightTile face0_tile = smooth_tile(TileKey::make(0, level, last, tile_y));
		const HeightTile face2_tile = smooth_tile(TileKey::make(2, level, 0, tile_y));
		for (int child_row = 0; child_row < 2; ++child_row) {
			compared += compare_subdivided_edge(face0_tile, face2_tile, child_row);
		}
	}
	std::printf("  %d face-edge samples compared after subdivision\n", compared);
}

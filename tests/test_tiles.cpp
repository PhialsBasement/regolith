#include <cmath>

#include "regolith/cube.hpp"
#include "regolith/height_tile.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

// A parent tile filled from a smooth function of position.
HeightTile smooth_tile(TileKey p_key, const TileLayout &p_layout) {
	HeightTile t;
	t.key = p_key;
	t.rho.resize(size_t(p_layout.count()));
	const int64_t m = int64_t(p_layout.intervals) << p_key.level();
	for (int j = -p_layout.border; j <= p_layout.intervals + p_layout.border; ++j) {
		for (int i = -p_layout.border; i <= p_layout.intervals + p_layout.border; ++i) {
			const IVec3 p = fold_lattice(p_key.face(), int64_t(p_key.x()) * p_layout.intervals + i,
					int64_t(p_key.y()) * p_layout.intervals + j, m);
			const DVec3 d = lattice_dir(p, m);
			t.rho[size_t(p_layout.index(i, j))] = float(800.0 * std::sin(40.0 * d.x) + 300.0 * d.y * d.z);
		}
	}
	return t;
}

} // namespace

TEST_CASE(subdivision_keeps_even_samples) {
	const TileLayout lay{ 16, 2 };
	const HeightTile parent = smooth_tile(TileKey::make(2, 3, 5, 6), lay);
	for (int q = 0; q < 4; ++q) {
		HeightTile child;
		subdivide(parent, lay, q, child);
		CHECK(child.key == parent.key.child(q));
		for (int j = 0; j <= lay.intervals; j += 2) {
			for (int i = 0; i <= lay.intervals; i += 2) {
				const int pi = ((q & 1) * lay.intervals + i) / 2;
				const int pj = ((q >> 1) * lay.intervals + j) / 2;
				CHECK(child.at(lay, i, j) == parent.at(lay, pi, pj));
			}
		}
	}
}

TEST_CASE(subdivided_face_edges_are_bit_identical) {
	// Face 0 (+X) and face 2 (+Y) share the edge x = y; face 0's
	// s = +1 column meets face 2's s = -1 column, same t.
	const TileLayout lay{ 16, 2 };
	const int level = 2;
	const uint32_t last = (1u << level) - 1;
	int compared = 0;
	for (uint32_t ty = 0; ty <= last; ++ty) {
		const HeightTile a = smooth_tile(TileKey::make(0, level, last, ty), lay);
		const HeightTile b = smooth_tile(TileKey::make(2, level, 0, ty), lay);
		for (int qy = 0; qy < 2; ++qy) {
			HeightTile ca, cb;
			subdivide(a, lay, 1 | (qy << 1), ca);
			subdivide(b, lay, 0 | (qy << 1), cb);
			for (int j = 0; j <= lay.intervals; ++j) {
				CHECK(ca.at(lay, lay.intervals, j) == cb.at(lay, 0, j));
				++compared;
			}
		}
	}
	std::printf("  %d face-edge samples compared after subdivision\n", compared);
}

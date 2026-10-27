#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include "regolith/codec.hpp"
#include "regolith/cube.hpp"
#include "regolith/pack.hpp"
#include "regolith/synth.hpp"
#include "regolith/tile_loader.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

constexpr double R = 3389500.0;
const TileLayout LAY{ 16, 2 };
constexpr int PACK_MAX = 2;

// A small pack whose levels are scaled down so every synthesis
// pass runs: detail at 2, roughness at 3, relief at 4.
std::string make_pack() {
	const char *path = "regolith_synth_test.rgp";
	PackHeader h;
	h.body = "test";
	h.intervals = uint32_t(LAY.intervals);
	h.border = uint32_t(LAY.border);
	h.step = 1.0 / 16.0;
	h.r_ref = R;
	h.codec = CODEC_MED;
	h.max_level = PACK_MAX;
	PackWriter w;
	w.open(path, h, "");
	std::vector<std::pair<TileKey, std::vector<int32_t>>> tiles;
	for (int face = 0; face < FACE_COUNT; ++face) {
		for (int level = 0; level <= PACK_MAX; ++level) {
			for (uint32_t x = 0; x < (1u << level); ++x) {
				for (uint32_t y = 0; y < (1u << level); ++y) {
					const TileKey k = TileKey::make(face, level, x, y);
					std::vector<int32_t> v(size_t(LAY.count()));
					for (int j = -LAY.border; j <= LAY.intervals + LAY.border; ++j) {
						for (int i = -LAY.border; i <= LAY.intervals + LAY.border; ++i) {
							const DVec3 d = Synth::sample_dir(k, LAY, i, j);
							const double rho = 3000.0 * std::sin(6.0 * d.x) * std::cos(5.0 * d.y + d.z);
							v[size_t(LAY.index(i, j))] = int32_t(std::llround(rho * 16.0));
						}
					}
					tiles.push_back({ k, v });
				}
			}
		}
	}
	std::sort(tiles.begin(), tiles.end(), [](auto &p_a, auto &p_b) { return p_a.first.bits < p_b.first.bits; });
	for (auto &t : tiles) {
		w.add(t.first, encode_tile(t.second.data(), LAY.side(), CODEC_MED, 1), 0, 0);
	}
	w.finish();
	return path;
}

struct EdgeTally {
	int compared = 0;
	int mismatched = 0;
	int cross_face = 0;
	int near_corner = 0;
	float worst = 0.0f;
	float worst_cross = 0.0f;
};

// Every interior edge sample of tile a against the tile that
// owns the same lattice point.
void compare_edges(TileLoader &p_loader, TileKey p_a, EdgeTally &r_t) {
	const HeightTilePtr a = p_loader.produce(p_a);
	const int n = LAY.intervals;
	const int64_t m = int64_t(n) << p_a.level();
	const uint32_t tiles = 1u << p_a.level();
	for (int e = 0; e <= n; ++e) {
		const int pts[4][2] = { { 0, e }, { n, e }, { e, 0 }, { e, n } };
		for (const auto &pt : pts) {
			// the sample one step outside a's edge, from both tiles
			const int oi = pt[0] == 0 ? -1 : (pt[0] == n ? n + 1 : pt[0]);
			const int oj = pt[1] == 0 ? -1 : (pt[1] == n ? n + 1 : pt[1]);
			const FaceSample c = canonical_sample(fold_lattice(p_a.face(), int64_t(p_a.x()) * n + oi,
														 int64_t(p_a.y()) * n + oj, m),
					m);
			const uint32_t tx = std::min(uint32_t(c.gi / n), tiles - 1), ty = std::min(uint32_t(c.gj / n), tiles - 1);
			const TileKey bk = TileKey::make(c.face, p_a.level(), tx, ty);
			if (bk == p_a) {
				continue;
			}
			const HeightTilePtr b = p_loader.produce(bk);
			const float va = a->at(LAY, oi, oj);
			const float vb = b->at(LAY, int(c.gi - int64_t(tx) * n), int(c.gj - int64_t(ty) * n));
			const DVec3 d = Synth::sample_dir(p_a, LAY, oi, oj);
			const bool corner = std::fabs(std::fabs(d.x) - std::fabs(d.y)) < 0.05 && std::fabs(std::fabs(d.y) - std::fabs(d.z)) < 0.05;
			++r_t.compared;
			if (va != vb) {
				if (corner) {
					++r_t.near_corner;
				} else if (c.face != p_a.face()) {
					++r_t.cross_face;
					r_t.worst_cross = std::max(r_t.worst_cross, std::fabs(va - vb));
				} else {
					++r_t.mismatched;
					r_t.worst = std::max(r_t.worst, std::fabs(va - vb));
				}
			}
		}
	}
}

} // namespace

TEST_CASE(synthesis_is_seamless) {
	const std::string path = make_pack();
	PackTileSource src;
	CHECK(src.open(path));
	SynthConfig cfg;
	cfg.profile = 1;
	cfg.r_ref = R;
	cfg.intervals = LAY.intervals;
	cfg.detail_level = 2;
	cfg.rough_level = 3;
	cfg.relief_level = 4;
	const Synth synth(cfg);
	TileCache cache(100000, 1);
	TileLoader loader(src, cache, 0, &synth);
	std::mt19937 rng(3);
	for (int level : { 2, 3, 4, 5 }) {
		EdgeTally t;
		std::uniform_int_distribution<uint32_t> pick(0, (1u << level) - 1);
		for (int k = 0; k < 10; ++k) {
			compare_edges(loader, TileKey::make(int(rng() % 6), level, pick(rng), pick(rng)), t);
		}
		// Face edges and a cube corner on purpose.
		compare_edges(loader, TileKey::make(0, level, (1u << level) - 1, 0), t);
		compare_edges(loader, TileKey::make(4, level, 0, (1u << level) - 1), t);
		std::printf("  level %d: %d shared samples; within a face %d differ (worst %.4f m); across faces %d (worst %.4f m); "
					"%d at cube corners\n",
				level, t.compared, t.mismatched, double(t.worst), t.cross_face, double(t.worst_cross), t.near_corner);
		CHECK(t.compared > 0);
		// Beyond a face edge the border is extrapolated, not
		// shared (it only feeds normals), so it is reported only.
		CHECK(t.worst < 0.01f);
	}
	const HeightTilePtr g = loader.produce(TileKey::make(1, 4, 3, 7));
	CHECK(g->geology.size() == size_t(Synth::geology_side(LAY) * Synth::geology_side(LAY) * 4));
	std::remove(path.c_str());
}

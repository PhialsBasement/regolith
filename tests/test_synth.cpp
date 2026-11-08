// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

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

constexpr double MARS_RADIUS_M = 3389500.0;
const TileLayout LAYOUT{ 16, 2 };

// A small pack, levels 0 to 2, with the synthesis levels
// scaled down to it so every pass runs: detail at 2,
// roughness at 3, relief at 4.
const char *const PACK_PATH = "regolith_synth_test.rgp";
constexpr int PACK_MAX_LEVEL = 2;
constexpr double PACK_STEP_M = 1.0 / 16.0;
constexpr int DETAIL_LEVEL = 2;
constexpr int ROUGH_LEVEL = 3;
constexpr int RELIEF_LEVEL = 4;

constexpr int RANDOM_TILES_PER_LEVEL = 10;
// Within a face, a shared sample must agree to this.
constexpr float SEAM_TOLERANCE_M = 0.01f;

// The pack's terrain: smooth swells, in pack steps.
std::vector<int32_t> pack_tile_samples(TileKey p_key) {
	std::vector<int32_t> samples(size_t(LAYOUT.count()));
	for (int j = -LAYOUT.border; j <= LAYOUT.intervals + LAYOUT.border; ++j) {
		for (int i = -LAYOUT.border; i <= LAYOUT.intervals + LAYOUT.border; ++i) {
			const DVec3 dir = Synth::sample_dir(p_key, LAYOUT, i, j);
			const double rho = 3000.0 * std::sin(6.0 * dir.x) * std::cos(5.0 * dir.y + dir.z);
			samples[size_t(LAYOUT.index(i, j))] = int32_t(std::llround(rho / PACK_STEP_M));
		}
	}
	return samples;
}

void add_level_keys(int p_face, int p_level, std::vector<TileKey> &r_keys) {
	const uint32_t tiles = 1u << p_level;
	for (uint32_t x = 0; x < tiles; ++x) {
		for (uint32_t y = 0; y < tiles; ++y) {
			r_keys.push_back(TileKey::make(p_face, p_level, x, y));
		}
	}
}

// Every tile of the pack, in the ascending order a pack needs.
std::vector<TileKey> pack_keys() {
	std::vector<TileKey> keys;
	for (int face = 0; face < FACE_COUNT; ++face) {
		for (int level = 0; level <= PACK_MAX_LEVEL; ++level) {
			add_level_keys(face, level, keys);
		}
	}
	std::sort(keys.begin(), keys.end(), [](TileKey p_a, TileKey p_b) { return p_a.bits < p_b.bits; });
	return keys;
}

void write_pack() {
	PackHeader header;
	header.body = "test";
	header.intervals = uint32_t(LAYOUT.intervals);
	header.border = uint32_t(LAYOUT.border);
	header.step = PACK_STEP_M;
	header.r_ref = MARS_RADIUS_M;
	header.codec = CODEC_MED;
	header.max_level = PACK_MAX_LEVEL;
	PackWriter writer;
	writer.open(PACK_PATH, header, "");
	for (TileKey key : pack_keys()) {
		const std::vector<int32_t> samples = pack_tile_samples(key);
		writer.add(key, encode_tile(samples.data(), LAYOUT.side(), CODEC_MED, 1), 0, 0);
	}
	writer.finish();
}

SynthConfig synth_config() {
	SynthConfig config;
	config.profile = 1; // Mars
	config.r_ref = MARS_RADIUS_M;
	config.intervals = LAYOUT.intervals;
	config.detail_level = DETAIL_LEVEL;
	config.rough_level = ROUGH_LEVEL;
	config.relief_level = RELIEF_LEVEL;
	return config;
}

// The test pack behind a tile loader with synthesis on. Open
// source before producing tiles.
struct SynthLoader {
	PackTileSource source;
	const Synth synth{ synth_config() };
	TileCache cache{ 100000, 1 };
	TileLoader loader{ source, cache, 0, &synth };
};

struct EdgeTally {
	int compared = 0;
	int differ_within_face = 0;
	int differ_across_faces = 0;
	int differ_near_corner = 0;
	float worst_within_face = 0.0f;
	float worst_across_faces = 0.0f;
};

// An index on the tile's edge moves one step outside it;
// any other index stays.
int step_outside(int p_index) {
	if (p_index == 0) {
		return -1;
	}
	if (p_index == LAYOUT.intervals) {
		return LAYOUT.intervals + 1;
	}
	return p_index;
}

bool near_cube_corner(const DVec3 &p_dir) {
	const double x = std::fabs(p_dir.x), y = std::fabs(p_dir.y), z = std::fabs(p_dir.z);
	return std::fabs(x - y) < 0.05 && std::fabs(y - z) < 0.05;
}

// Sample (p_i, p_j) just outside a tile, read from the tile and
// from the neighbour that owns the same lattice point.
void compare_sample(TileLoader &p_loader, TileKey p_key, const HeightTile &p_tile, int p_i, int p_j,
		EdgeTally &r_tally) {
	const int n = LAYOUT.intervals;
	const int64_t edge_intervals = int64_t(n) << p_key.level();
	const uint32_t last_tile = (1u << p_key.level()) - 1;
	const IVec3 point =
			fold_lattice(p_key.face(), int64_t(p_key.x()) * n + p_i, int64_t(p_key.y()) * n + p_j, edge_intervals);
	const FaceSample owner = canonical_sample(point, edge_intervals);
	const uint32_t owner_x = std::min(uint32_t(owner.gi / n), last_tile);
	const uint32_t owner_y = std::min(uint32_t(owner.gj / n), last_tile);
	const TileKey owner_key = TileKey::make(owner.face, p_key.level(), owner_x, owner_y);
	if (owner_key == p_key) {
		return;
	}
	const HeightTilePtr owner_tile = p_loader.produce(owner_key);
	const float here = p_tile.at(LAYOUT, p_i, p_j);
	const float there =
			owner_tile->at(LAYOUT, int(owner.gi - int64_t(owner_x) * n), int(owner.gj - int64_t(owner_y) * n));
	++r_tally.compared;
	if (here == there) {
		return;
	}
	const float difference = std::fabs(here - there);
	if (near_cube_corner(Synth::sample_dir(p_key, LAYOUT, p_i, p_j))) {
		++r_tally.differ_near_corner;
		return;
	}
	if (owner.face != p_key.face()) {
		++r_tally.differ_across_faces;
		r_tally.worst_across_faces = std::max(r_tally.worst_across_faces, difference);
		return;
	}
	++r_tally.differ_within_face;
	r_tally.worst_within_face = std::max(r_tally.worst_within_face, difference);
}

// Every sample just outside the tile's interior edges.
void compare_edges(TileLoader &p_loader, TileKey p_key, EdgeTally &r_tally) {
	const HeightTilePtr tile = p_loader.produce(p_key);
	const int n = LAYOUT.intervals;
	for (int along = 0; along <= n; ++along) {
		const int edge_samples[4][2] = { { 0, along }, { n, along }, { along, 0 }, { along, n } };
		for (const auto &sample : edge_samples) {
			compare_sample(p_loader, p_key, *tile, step_outside(sample[0]), step_outside(sample[1]), r_tally);
		}
	}
}

} // namespace

TEST_CASE(synthesis_is_seamless) {
	write_pack();
	SynthLoader world;
	CHECK(world.source.open(PACK_PATH));
	std::mt19937 rng(3);
	for (int level : { 2, 3, 4, 5 }) {
		EdgeTally tally;
		const uint32_t last_tile = (1u << level) - 1;
		std::uniform_int_distribution<uint32_t> pick(0, last_tile);
		for (int k = 0; k < RANDOM_TILES_PER_LEVEL; ++k) {
			compare_edges(world.loader, TileKey::make(int(rng() % 6), level, pick(rng), pick(rng)), tally);
		}
		// face edges and a cube corner, on purpose
		compare_edges(world.loader, TileKey::make(0, level, last_tile, 0), tally);
		compare_edges(world.loader, TileKey::make(4, level, 0, last_tile), tally);
		std::printf("  level %d: %d shared samples; within a face %d differ (worst %.4f m); across faces %d (worst %.4f m); "
					"%d at cube corners\n",
				level, tally.compared, tally.differ_within_face, double(tally.worst_within_face),
				tally.differ_across_faces, double(tally.worst_across_faces), tally.differ_near_corner);
		CHECK(tally.compared > 0);
		// Beyond a face edge the border is extrapolated rather
		// than shared (it only feeds normals): reported only.
		CHECK(tally.worst_within_face < SEAM_TOLERANCE_M);
	}
	std::remove(PACK_PATH);
}

TEST_CASE(synthesis_fills_geology) {
	write_pack();
	SynthLoader world;
	CHECK(world.source.open(PACK_PATH));
	const HeightTilePtr tile = world.loader.produce(TileKey::make(1, 4, 3, 7));
	const size_t side = size_t(Synth::geology_side(LAYOUT));
	CHECK(tile->geology.size() == side * side * 4); // RGBA
	std::remove(PACK_PATH);
}

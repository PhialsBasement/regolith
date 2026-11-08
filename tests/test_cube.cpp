// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <array>
#include <cmath>
#include <random>

#include "regolith/cube.hpp"
#include "regolith/tile_key.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

constexpr int RANDOM_SAMPLES = 1000000;
constexpr int EDGE_SAMPLES_PER_FACE = 200;
constexpr double DIR_TOLERANCE_RAD = 1e-15;
constexpr double FACE_COORD_TOLERANCE = 1e-14;

DVec3 random_dir(std::mt19937_64 &p_rng) {
	std::normal_distribution<double> n(0.0, 1.0);
	return normalize(DVec3(n(p_rng), n(p_rng), n(p_rng)));
}

// The four samples at position p_along on each edge of a face.
std::array<FaceSample, 4> edge_samples(int p_face, int64_t p_along, int64_t p_edge_intervals) {
	return { FaceSample{ p_face, 0, p_along }, FaceSample{ p_face, p_edge_intervals, p_along },
		FaceSample{ p_face, p_along, 0 }, FaceSample{ p_face, p_along, p_edge_intervals } };
}

bool is_corner(const FaceSample &p_sample, int64_t p_edge_intervals) {
	const bool on_i_edge = p_sample.gi == 0 || p_sample.gi == p_edge_intervals;
	const bool on_j_edge = p_sample.gj == 0 || p_sample.gj == p_edge_intervals;
	return on_i_edge && on_j_edge;
}

// Every face that holds an edge sample must give the same
// lattice point and bit-identical direction. Returns how
// many faces were checked.
int check_shared_sample(const FaceSample &p_sample, int64_t p_edge_intervals) {
	const IVec3 point = lattice_point(p_sample, p_edge_intervals);
	FaceSample owners[3];
	const int owner_count = lattice_faces(point, p_edge_intervals, owners);
	CHECK(owner_count == (is_corner(p_sample, p_edge_intervals) ? 3 : 2));
	const DVec3 dir = lattice_dir(point, p_edge_intervals);
	const double m = double(p_edge_intervals);
	for (int o = 0; o < owner_count; ++o) {
		CHECK(lattice_point(owners[o], p_edge_intervals) == point);
		const FaceCoord coord{ owners[o].face, double(2 * owners[o].gi - p_edge_intervals) / m,
			double(2 * owners[o].gj - p_edge_intervals) / m };
		CHECK(face_dir(coord) == dir);
	}
	return owner_count;
}

int check_random_edge_samples(int p_intervals, int p_level, std::mt19937_64 &p_rng) {
	const int64_t edge_intervals = int64_t(p_intervals) << p_level;
	std::uniform_int_distribution<int64_t> along_edge(0, edge_intervals);
	int checked = 0;
	for (int face = 0; face < FACE_COUNT; ++face) {
		for (int n = 0; n < EDGE_SAMPLES_PER_FACE; ++n) {
			for (const FaceSample &sample : edge_samples(face, along_edge(p_rng), edge_intervals)) {
				checked += check_shared_sample(sample, edge_intervals);
			}
		}
	}
	return checked;
}

} // namespace

TEST_CASE(dir_round_trip) {
	std::mt19937_64 rng(1);
	const double inv_sqrt3 = 1.0 / std::sqrt(3.0);
	const double inv_sqrt2 = 1.0 / std::sqrt(2.0);
	// poles, an axis, cube corners and edge midpoints
	const DVec3 special_dirs[] = { { 0, 0, 1 }, { 0, 0, -1 }, { 1, 0, 0 }, { inv_sqrt3, inv_sqrt3, inv_sqrt3 },
		{ -inv_sqrt3, inv_sqrt3, -inv_sqrt3 }, { inv_sqrt2, inv_sqrt2, 0 }, { 0, -inv_sqrt2, inv_sqrt2 } };
	double worst = 0.0;
	auto measure = [&](const DVec3 &p_dir) {
		worst = std::fmax(worst, length(face_dir(dir_to_face(p_dir)) - p_dir));
	};
	for (const DVec3 &dir : special_dirs) {
		measure(dir);
	}
	for (int i = 0; i < RANDOM_SAMPLES; ++i) {
		measure(random_dir(rng));
	}
	std::printf("  worst dir round trip: %.3e rad\n", worst);
	CHECK(worst < DIR_TOLERANCE_RAD);
}

TEST_CASE(face_coord_round_trip) {
	std::mt19937_64 rng(2);
	std::uniform_real_distribution<double> coordinate(-1.0, 1.0);
	double worst = 0.0;
	for (int i = 0; i < RANDOM_SAMPLES; ++i) {
		const FaceCoord in{ int(i % FACE_COUNT), coordinate(rng), coordinate(rng) };
		const FaceCoord out = dir_to_face(face_dir(in));
		CHECK(out.face == in.face);
		worst = std::fmax(worst, std::fmax(std::abs(out.s - in.s), std::abs(out.t - in.t)));
	}
	std::printf("  worst face coord round trip: %.3e\n", worst);
	CHECK(worst < FACE_COORD_TOLERANCE);
}

TEST_CASE(shared_edges_are_bit_identical) {
	std::mt19937_64 rng(3);
	int shared = 0;
	for (int intervals : { 208, 256 }) {
		for (int level : { 0, 3, 7, 17 }) {
			shared += check_random_edge_samples(intervals, level, rng);
		}
	}
	std::printf("  %d shared edge/corner samples checked\n", shared);
}

TEST_CASE(interior_samples_have_one_face) {
	const int64_t edge_intervals = 208 << 5;
	for (int face = 0; face < FACE_COUNT; ++face) {
		const FaceSample interior{ face, 17, edge_intervals - 3 };
		FaceSample owners[3];
		CHECK(lattice_faces(lattice_point(interior, edge_intervals), edge_intervals, owners) == 1);
		CHECK(owners[0].face == face);
		CHECK(owners[0].gi == interior.gi);
		CHECK(owners[0].gj == interior.gj);
	}
}

TEST_CASE(tile_key_packs_its_fields) {
	const uint32_t largest_coordinate = (1u << 28) - 1;
	const TileKey key = TileKey::make(5, 28, largest_coordinate, 12345);
	CHECK(key.face() == 5);
	CHECK(key.level() == 28);
	CHECK(key.x() == largest_coordinate);
	CHECK(key.y() == 12345);
}

TEST_CASE(tile_key_children_and_parent) {
	const TileKey parent = TileKey::make(3, 7, 100, 41);
	for (int quadrant = 0; quadrant < 4; ++quadrant) {
		const TileKey child = parent.child(quadrant);
		CHECK(child.level() == 8);
		CHECK(child.parent() == parent);
		CHECK(child.x() == 200u + uint32_t(quadrant & 1));
		CHECK(child.y() == 82u + uint32_t(quadrant >> 1));
	}
}

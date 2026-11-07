// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cmath>
#include <random>

#include "regolith/cube.hpp"
#include "regolith/tile_key.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

DVec3 random_dir(std::mt19937_64 &p_rng) {
	std::normal_distribution<double> n(0.0, 1.0);
	return normalize(DVec3(n(p_rng), n(p_rng), n(p_rng)));
}

} // namespace

TEST_CASE(dir_round_trip) {
	std::mt19937_64 rng(1);
	const double r3 = 1.0 / std::sqrt(3.0);
	const double r2 = 1.0 / std::sqrt(2.0);
	const DVec3 special[] = {
		{ 0, 0, 1 }, { 0, 0, -1 }, { 1, 0, 0 }, { r3, r3, r3 }, { -r3, r3, -r3 }, { r2, r2, 0 }, { 0, -r2, r2 }
	};
	double worst = 0.0;
	auto check = [&](const DVec3 &p_d) {
		worst = std::fmax(worst, length(face_dir(dir_to_face(p_d)) - p_d));
	};
	for (const DVec3 &d : special) {
		check(d);
	}
	for (int i = 0; i < 1000000; ++i) {
		check(random_dir(rng));
	}
	std::printf("  worst dir round trip: %.3e rad\n", worst);
	CHECK(worst < 1e-15);
}

TEST_CASE(face_coord_round_trip) {
	std::mt19937_64 rng(2);
	std::uniform_real_distribution<double> c(-1.0, 1.0);
	double worst = 0.0;
	for (int i = 0; i < 1000000; ++i) {
		const FaceCoord in{ int(i % FACE_COUNT), c(rng), c(rng) };
		const FaceCoord out = dir_to_face(face_dir(in));
		CHECK(out.face == in.face);
		worst = std::fmax(worst, std::fmax(std::abs(out.s - in.s), std::abs(out.t - in.t)));
	}
	std::printf("  worst face coord round trip: %.3e\n", worst);
	CHECK(worst < 1e-14);
}

TEST_CASE(shared_edges_are_bit_identical) {
	std::mt19937_64 rng(3);
	int shared = 0;
	for (int intervals : { 208, 256 }) {
		for (int level : { 0, 3, 7, 17 }) {
			const int64_t m = int64_t(intervals) << level;
			std::uniform_int_distribution<int64_t> along(0, m);
			for (int face = 0; face < FACE_COUNT; ++face) {
				for (int n = 0; n < 200; ++n) {
					const int64_t k = along(rng);
					const FaceSample edge_samples[] = {
						{ face, 0, k }, { face, m, k }, { face, k, 0 }, { face, k, m }
					};
					for (const FaceSample &s : edge_samples) {
						const IVec3 p = lattice_point(s, m);
						FaceSample owners[3];
						const int count = lattice_faces(p, m, owners);
						const bool corner = (k == 0 || k == m);
						CHECK(count == (corner ? 3 : 2));
						const DVec3 d = lattice_dir(p, m);
						for (int o = 0; o < count; ++o) {
							CHECK(lattice_point(owners[o], m) == p);
							const FaceCoord fc{ owners[o].face, double(2 * owners[o].gi - m) / double(m),
								double(2 * owners[o].gj - m) / double(m) };
							CHECK(face_dir(fc) == d);
							++shared;
						}
					}
				}
			}
		}
	}
	std::printf("  %d shared edge/corner samples checked\n", shared);
}

TEST_CASE(interior_samples_have_one_face) {
	const int64_t m = 208 << 5;
	for (int face = 0; face < FACE_COUNT; ++face) {
		const FaceSample s{ face, 17, m - 3 };
		FaceSample owners[3];
		CHECK(lattice_faces(lattice_point(s, m), m, owners) == 1);
		CHECK(owners[0].face == face && owners[0].gi == 17 && owners[0].gj == m - 3);
	}
}

TEST_CASE(tile_key_packing) {
	const TileKey k = TileKey::make(5, 28, (1u << 28) - 1, 12345);
	CHECK(k.face() == 5);
	CHECK(k.level() == 28);
	CHECK(k.x() == (1u << 28) - 1);
	CHECK(k.y() == 12345);
	const TileKey p = TileKey::make(3, 7, 100, 41);
	for (int q = 0; q < 4; ++q) {
		const TileKey c = p.child(q);
		CHECK(c.level() == 8);
		CHECK(c.parent() == p);
		CHECK(c.x() == 200u + uint32_t(q & 1));
		CHECK(c.y() == 82u + uint32_t(q >> 1));
	}
}

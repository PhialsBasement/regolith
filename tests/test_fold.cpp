// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cmath>
#include <numbers>

#include "regolith/cube.hpp"
#include "test.hpp"

using namespace regolith;

TEST_CASE(fold_crosses_into_neighbour) {
	const int64_t m = 208 << 3;
	const double texel = std::numbers::pi / 2.0 / double(m);
	double worst_ratio_lo = 1e9;
	double worst_ratio_hi = 0.0;
	for (int face = 0; face < FACE_COUNT; ++face) {
		for (int64_t k : { int64_t(0), int64_t(1), m / 3, m - 1, m }) {
			// (edge sample, one step past it) for each of 4 edges
			const int64_t pairs[4][4] = {
				{ 0, k, -1, k }, { m, k, m + 1, k }, { k, 0, k, -1 }, { k, m, k, m + 1 }
			};
			for (const auto &pr : pairs) {
				const IVec3 edge = fold_lattice(face, pr[0], pr[1], m);
				const IVec3 past = fold_lattice(face, pr[2], pr[3], m);
				const FaceSample c = canonical_sample(past, m);
				CHECK(dot(face_axes(c.face).f, past) == m);
				const double ang = length(lattice_dir(past, m) - lattice_dir(edge, m));
				worst_ratio_lo = std::fmin(worst_ratio_lo, ang / texel);
				worst_ratio_hi = std::fmax(worst_ratio_hi, ang / texel);
			}
		}
	}
	std::printf("  one step across an edge: %.3f to %.3f texels\n", worst_ratio_lo, worst_ratio_hi);
	CHECK(worst_ratio_lo > 0.4);
	CHECK(worst_ratio_hi < 2.5);
}

TEST_CASE(fold_is_identity_inside_face) {
	const int64_t m = 256 << 2;
	for (int face = 0; face < FACE_COUNT; ++face) {
		const FaceSample s{ face, 5, m - 7 };
		CHECK(fold_lattice(face, s.gi, s.gj, m) == lattice_point(s, m));
	}
}

TEST_CASE(canonical_is_lowest_face) {
	const int64_t m = 208 << 4;
	for (int face = 0; face < FACE_COUNT; ++face) {
		for (const FaceSample &s : { FaceSample{ face, 0, 9 }, FaceSample{ face, m, m }, FaceSample{ face, 3, m } }) {
			const IVec3 p = lattice_point(s, m);
			FaceSample owners[3];
			const int n = lattice_faces(p, m, owners);
			const FaceSample c = canonical_sample(p, m);
			CHECK(n >= 1);
			CHECK(c.face == owners[0].face && c.gi == owners[0].gi && c.gj == owners[0].gj);
			CHECK(lattice_point(c, m) == p);
		}
	}
}

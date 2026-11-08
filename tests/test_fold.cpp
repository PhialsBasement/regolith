// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <array>
#include <cmath>
#include <numbers>

#include "regolith/cube.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

// A sample on a face edge and the sample one step past it.
struct EdgeStep {
	int64_t edge_i, edge_j;
	int64_t past_i, past_j;
};

// The step outward at position p_along on each of a face's
// four edges.
std::array<EdgeStep, 4> outward_steps(int64_t p_along, int64_t p_edge_intervals) {
	const int64_t m = p_edge_intervals;
	return { EdgeStep{ 0, p_along, -1, p_along }, EdgeStep{ m, p_along, m + 1, p_along },
		EdgeStep{ p_along, 0, p_along, -1 }, EdgeStep{ p_along, m, p_along, m + 1 } };
}

struct StepRange {
	double fewest_texels = 1e9;
	double most_texels = 0.0;
};

// One step past an edge lands on the neighbouring face, about
// one texel from the edge sample.
void measure_step(int p_face, const EdgeStep &p_step, int64_t p_edge_intervals, StepRange &r_range) {
	const double texel_rad = std::numbers::pi / 2.0 / double(p_edge_intervals);
	const IVec3 edge = fold_lattice(p_face, p_step.edge_i, p_step.edge_j, p_edge_intervals);
	const IVec3 past = fold_lattice(p_face, p_step.past_i, p_step.past_j, p_edge_intervals);
	const FaceSample owner = canonical_sample(past, p_edge_intervals);
	CHECK(dot(face_axes(owner.face).f, past) == p_edge_intervals);
	const double step_rad = length(lattice_dir(past, p_edge_intervals) - lattice_dir(edge, p_edge_intervals));
	r_range.fewest_texels = std::fmin(r_range.fewest_texels, step_rad / texel_rad);
	r_range.most_texels = std::fmax(r_range.most_texels, step_rad / texel_rad);
}

} // namespace

TEST_CASE(fold_crosses_into_neighbour) {
	const int64_t edge_intervals = 208 << 3;
	const int64_t positions[] = { 0, 1, edge_intervals / 3, edge_intervals - 1, edge_intervals };
	StepRange range;
	for (int face = 0; face < FACE_COUNT; ++face) {
		for (int64_t along : positions) {
			for (const EdgeStep &step : outward_steps(along, edge_intervals)) {
				measure_step(face, step, edge_intervals, range);
			}
		}
	}
	std::printf("  one step across an edge: %.3f to %.3f texels\n", range.fewest_texels, range.most_texels);
	CHECK(range.fewest_texels > 0.4);
	CHECK(range.most_texels < 2.5);
}

TEST_CASE(fold_is_identity_inside_face) {
	const int64_t edge_intervals = 256 << 2;
	for (int face = 0; face < FACE_COUNT; ++face) {
		const FaceSample interior{ face, 5, edge_intervals - 7 };
		CHECK(fold_lattice(face, interior.gi, interior.gj, edge_intervals) == lattice_point(interior, edge_intervals));
	}
}

TEST_CASE(canonical_is_lowest_face) {
	const int64_t edge_intervals = 208 << 4;
	for (int face = 0; face < FACE_COUNT; ++face) {
		const FaceSample edge_and_corner_samples[] = { { face, 0, 9 }, { face, edge_intervals, edge_intervals },
			{ face, 3, edge_intervals } };
		for (const FaceSample &sample : edge_and_corner_samples) {
			const IVec3 point = lattice_point(sample, edge_intervals);
			FaceSample owners[3];
			const int owner_count = lattice_faces(point, edge_intervals, owners);
			const FaceSample canonical = canonical_sample(point, edge_intervals);
			CHECK(owner_count >= 1);
			CHECK(canonical.face == owners[0].face);
			CHECK(canonical.gi == owners[0].gi);
			CHECK(canonical.gj == owners[0].gj);
			CHECK(lattice_point(canonical, edge_intervals) == point);
		}
	}
}

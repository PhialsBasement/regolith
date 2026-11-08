// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <algorithm>
#include <cmath>
#include <random>

#include "regolith/cube.hpp"
#include "regolith/patch.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

constexpr double CAMERA_HEIGHT_M = 2.0;
// Only vertices this close to the camera are measured.
constexpr double CAMERA_RANGE_M = 1000.0;
// Levels measured, from the body's finest one down.
constexpr int LEVELS_MEASURED = 7;
constexpr int SITES_PER_BODY = 50;
constexpr double MAX_VERTEX_ERROR_M = 1e-3;

struct Body {
	const char *name;
	double radius;
	int intervals;
	int finest_level;
};

// Synthetic relief, so the radius term gets exercised.
double relief(const DVec3 &p_dir, double p_radius) {
	return 0.02 * p_radius * std::sin(9.0 * p_dir.x) * std::cos(7.0 * p_dir.y + 3.0 * p_dir.z);
}

double surface_radius(const Body &p_body, const DVec3 &p_dir) {
	return p_body.radius + relief(p_dir, p_body.radius);
}

TileKey key_containing(const DVec3 &p_dir, int p_level) {
	const FaceCoord coord = dir_to_face(p_dir);
	const double tiles = double(uint64_t(1) << p_level);
	auto tile_index = [&](double p_coordinate) {
		return uint32_t(std::fmin(std::floor((p_coordinate + 1.0) * 0.5 * tiles), tiles - 1.0));
	};
	return TileKey::make(coord.face, p_level, tile_index(coord.s), tile_index(coord.t));
}

struct Errors {
	double difference_form = 0.0; // the patch code's error
	double plain_float = 0.0; // a plain float pipeline's error
	int vertices = 0;
};

// The camera hovering over a site, in the site's frame.
struct Viewpoint {
	RenderFrame frame;
	DVec3 camera;
};

// Worst vertex error near the camera for the node holding
// the site at p_level.
void measure_node(const Body &p_body, const DVec3 &p_site, int p_level, const Viewpoint &p_view, Errors &r_errors) {
	const PatchSpec spec{ key_containing(p_site, p_level), p_body.intervals };
	const int64_t edge_intervals = int64_t(spec.intervals) << p_level;
	auto dir_of = [&](int p_i, int p_j) {
		const FaceSample sample{ spec.key.face(), int64_t(spec.key.x()) * spec.intervals + p_i,
			int64_t(spec.key.y()) * spec.intervals + p_j };
		return lattice_dir(lattice_point(sample, edge_intervals), edge_intervals);
	};
	const int half = spec.intervals / 2;
	const double center_radius = surface_radius(p_body, dir_of(half, half));
	const NodeConstants constants = node_constants(spec, center_radius, p_view.frame);
	const FVec3 anchor_float = cast<float>(p_view.frame.anchor);
	for (int j = 0; j <= spec.intervals; ++j) {
		for (int i = 0; i <= spec.intervals; ++i) {
			const DVec3 dir = dir_of(i, j);
			const double radius = surface_radius(p_body, dir);
			const DVec3 reference = vertex_reference(spec, i, j, radius, p_view.frame);
			if (length(reference - p_view.camera) > CAMERA_RANGE_M) {
				continue;
			}
			const FVec3 vertex = vertex_float(constants, i, j, spec.intervals, float(radius - center_radius));
			r_errors.difference_form = std::fmax(r_errors.difference_form, length(cast<double>(vertex) - reference));

			const FVec3 world_float = cast<float>(dir * radius) - anchor_float;
			const DVec3 plain = p_view.frame.to_local_vec(cast<double>(world_float));
			r_errors.plain_float = std::fmax(r_errors.plain_float, length(plain - reference));
			++r_errors.vertices;
		}
	}
}

void measure_site(const Body &p_body, const DVec3 &p_site, Errors &r_errors) {
	const double site_radius = surface_radius(p_body, p_site);
	Viewpoint view;
	view.frame = RenderFrame::at(p_site * site_radius);
	view.camera = view.frame.to_local_point(p_site * (site_radius + CAMERA_HEIGHT_M));
	const int coarsest = std::max(p_body.finest_level - (LEVELS_MEASURED - 1), 0);
	for (int level = p_body.finest_level; level >= coarsest; --level) {
		measure_node(p_body, p_site, level, view, r_errors);
	}
}

} // namespace

TEST_CASE(difference_form_vertex_precision) {
	const Body bodies[] = {
		{ "Mars", 3389500.0, 208, 17 },
		{ "Phobos", 11080.0, 256, 8 },
	};
	const double inv_sqrt3 = 1.0 / std::sqrt(3.0);
	// poles, cube corners and an edge midpoint
	const DVec3 special_sites[] = { { 0, 0, 1 }, { 0, 0, -1 }, { inv_sqrt3, inv_sqrt3, inv_sqrt3 },
		{ -inv_sqrt3, -inv_sqrt3, inv_sqrt3 }, { 0.7071067811865476, 0.7071067811865475, 0.0 } };
	std::mt19937_64 rng(4);
	std::normal_distribution<double> n(0.0, 1.0);
	for (const Body &body : bodies) {
		Errors errors;
		int sites = 0;
		for (const DVec3 &site : special_sites) {
			measure_site(body, normalize(site), errors);
			++sites;
		}
		for (; sites < SITES_PER_BODY; ++sites) {
			measure_site(body, normalize(DVec3(n(rng), n(rng), n(rng))), errors);
		}
		std::printf("  %s: %d sites, %d vertices within 1 km: worst %.3e m (plain float %.3e m)\n", body.name, sites,
				errors.vertices, errors.difference_form, errors.plain_float);
		CHECK(errors.vertices > 0);
		CHECK(errors.difference_form < MAX_VERTEX_ERROR_M);
	}
}

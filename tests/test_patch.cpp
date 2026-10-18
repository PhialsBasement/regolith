#include <cmath>
#include <random>

#include "regolith/cube.hpp"
#include "regolith/patch.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

struct Body {
	const char *name;
	double radius;
	int intervals;
	int finest_level;
};

// Synthetic relief so the radius term gets exercised.
double relief(const DVec3 &p_dir, double p_radius) {
	return 0.02 * p_radius * std::sin(9.0 * p_dir.x) * std::cos(7.0 * p_dir.y + 3.0 * p_dir.z);
}

TileKey key_containing(const DVec3 &p_dir, int p_level) {
	const FaceCoord fc = dir_to_face(p_dir);
	const double tiles = double(uint64_t(1) << p_level);
	auto index = [&](double p_c) {
		return uint32_t(std::fmin(std::floor((p_c + 1.0) * 0.5 * tiles), tiles - 1.0));
	};
	return TileKey::make(fc.face, p_level, index(fc.s), index(fc.t));
}

struct Errors {
	double ours = 0.0;
	double naive = 0.0;
	int vertices = 0;
};

// Worst vertex error within 1 km of a camera hovering 2 m
// over p_site, for the node holding the site at each level.
void measure_site(const Body &p_body, const DVec3 &p_site, Errors &r_err) {
	const double r_site = p_body.radius + relief(p_site, p_body.radius);
	const RenderFrame frame = RenderFrame::at(p_site * r_site);
	const DVec3 camera = frame.to_local_point(p_site * (r_site + 2.0));
	const FVec3 anchor_f = cast<float>(frame.anchor);
	for (int level = p_body.finest_level; level >= p_body.finest_level - 6 && level >= 0; --level) {
		const PatchSpec spec{ key_containing(p_site, level), p_body.intervals };
		const int64_t m = int64_t(spec.intervals) << level;
		const int half = spec.intervals / 2;
		auto dir_of = [&](int p_i, int p_j) {
			const FaceSample s{ spec.key.face(), int64_t(spec.key.x()) * spec.intervals + p_i,
				int64_t(spec.key.y()) * spec.intervals + p_j };
			return lattice_dir(lattice_point(s, m), m);
		};
		const double r_center = p_body.radius + relief(dir_of(half, half), p_body.radius);
		const NodeConstants nc = node_constants(spec, r_center, frame);
		for (int j = 0; j <= spec.intervals; ++j) {
			for (int i = 0; i <= spec.intervals; ++i) {
				const DVec3 d = dir_of(i, j);
				const double r = p_body.radius + relief(d, p_body.radius);
				const DVec3 ref = vertex_reference(spec, i, j, r, frame);
				if (length(ref - camera) > 1000.0) {
					continue;
				}
				const FVec3 ours = vertex_float(nc, i, j, spec.intervals, float(r - r_center));
				r_err.ours = std::fmax(r_err.ours, length(cast<double>(ours) - ref));

				// What a plain float pipeline would produce.
				const FVec3 world_f = cast<float>(d * r) - anchor_f;
				const DVec3 naive = frame.to_local_vec(cast<double>(world_f));
				r_err.naive = std::fmax(r_err.naive, length(naive - ref));
				++r_err.vertices;
			}
		}
	}
}

} // namespace

TEST_CASE(difference_form_vertex_precision) {
	const Body bodies[] = {
		{ "Mars", 3389500.0, 208, 17 },
		{ "Phobos", 11080.0, 256, 8 },
	};
	const double r3 = 1.0 / std::sqrt(3.0);
	const DVec3 special[] = {
		{ 0, 0, 1 }, { 0, 0, -1 }, { r3, r3, r3 }, { -r3, -r3, r3 }, { 0.7071067811865476, 0.7071067811865475, 0.0 }
	};
	std::mt19937_64 rng(4);
	std::normal_distribution<double> n(0.0, 1.0);
	for (const Body &body : bodies) {
		Errors err;
		int sites = 0;
		for (const DVec3 &d : special) {
			measure_site(body, normalize(d), err);
			++sites;
		}
		while (sites < 50) {
			measure_site(body, normalize(DVec3(n(rng), n(rng), n(rng))), err);
			++sites;
		}
		std::printf("  %s: %d sites, %d vertices within 1 km: worst %.3e m (plain float %.3e m)\n",
				body.name, sites, err.vertices, err.ours, err.naive);
		CHECK(err.vertices > 0);
		CHECK(err.ours < 1e-3);
	}
}

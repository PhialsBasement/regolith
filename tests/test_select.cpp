// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>

#include "regolith/cube.hpp"
#include "regolith/select.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

constexpr double R_REF = 3389500.0;
constexpr double RELIEF = 4500.0; // bound on |relief|
constexpr int QUADS = 52;

// Broad swells plus a 70 km wavelength at slopes up to ~0.13.
double relief(const DVec3 &p_d) {
	return 3000.0 * std::sin(5.0 * p_d.x + 1.0) * std::cos(7.0 * p_d.y - 2.0 * p_d.z) +
			1500.0 * std::sin(300.0 * p_d.x) * std::cos(300.0 * p_d.y);
}

// Tile bounds as a loaded tile would report them.
bool tile_bounds(TileKey p_key, float &r_lo, float &r_hi) {
	const double tiles = double(uint64_t(1) << p_key.level());
	if (Selector::node_edge(R_REF, p_key.level()) > 20000.0) {
		r_lo = -float(RELIEF);
		r_hi = float(RELIEF);
		return true;
	}
	double lo = 1e9, hi = -1e9;
	for (int b = 0; b <= 4; ++b) {
		for (int a = 0; a <= 4; ++a) {
			const double s = -1.0 + 2.0 * (p_key.x() + a / 4.0) / tiles;
			const double t = -1.0 + 2.0 * (p_key.y() + b / 4.0) / tiles;
			const double h = relief(face_dir({ p_key.face(), s, t }));
			lo = std::min(lo, h);
			hi = std::max(hi, h);
		}
	}
	r_lo = float(lo - 50.0);
	r_hi = float(hi + 50.0);
	return true;
}

DVec3 surface(const DVec3 &p_d) {
	return p_d * (R_REF + relief(p_d));
}

double morph(const SelectedNode &p_n, double p_dist) {
	if (std::isinf(p_n.morph_end)) {
		return 0.0;
	}
	return std::clamp((p_dist - p_n.morph_start) / (p_n.morph_end - p_n.morph_start), 0.0, 1.0);
}

struct Lookup {
	std::unordered_map<uint64_t, size_t> index;
	int max_level = 0;

	const SelectedNode *find(const std::vector<SelectedNode> &p_nodes, const DVec3 &p_d) const {
		const FaceCoord fc = dir_to_face(p_d);
		for (int level = max_level; level >= 0; --level) {
			const double tiles = double(uint64_t(1) << level);
			auto cell = [&](double p_c) { return uint32_t(std::clamp(std::floor((p_c + 1.0) * 0.5 * tiles), 0.0, tiles - 1.0)); };
			auto it = index.find(TileKey::make(fc.face, level, cell(fc.s), cell(fc.t)).bits);
			if (it != index.end()) {
				return &p_nodes[it->second];
			}
		}
		return nullptr;
	}
};

struct Tally {
	long boundary_vertices = 0;
	long level_jumps = 0;
	long fine_not_morphed = 0;
	long coarse_morphing = 0;
	long missing = 0;
	size_t most_nodes = 0;
};

void check_camera(const DVec3 &p_cam_dir, double p_altitude, Tally &r_t) {
	SelectInput in;
	in.r_ref = R_REF;
	in.max_level = 15;
	in.camera = surface(p_cam_dir) + p_cam_dir * p_altitude;
	Selector sel;
	sel.rho_bounds = tile_bounds;
	sel.ready = [](TileKey) { return true; };
	std::vector<SelectedNode> nodes;
	std::vector<WantedTile> wanted;
	sel.select(in, nodes, wanted);
	r_t.most_nodes = std::max(r_t.most_nodes, nodes.size());

	Lookup look;
	look.max_level = in.max_level;
	for (size_t k = 0; k < nodes.size(); ++k) {
		look.index[nodes[k].key.bits] = k;
	}
	for (const SelectedNode &a : nodes) {
		const double tiles = double(uint64_t(1) << a.key.level());
		const double s0 = -1.0 + 2.0 * a.key.x() / tiles;
		const double t0 = -1.0 + 2.0 * a.key.y() / tiles;
		const double span = 2.0 / tiles;
		const double nudge = span * 1e-4;
		for (int edge = 0; edge < 4; ++edge) {
			for (int k = 0; k <= QUADS; ++k) {
				const double f = double(k) / QUADS;
				double s = 0, t = 0, ns = 0, nt = 0;
				switch (edge) {
					case 0: s = s0; t = t0 + f * span; ns = -nudge; break;
					case 1: s = s0 + span; t = t0 + f * span; ns = nudge; break;
					case 2: s = s0 + f * span; t = t0; nt = -nudge; break;
					default: s = s0 + f * span; t = t0 + span; nt = nudge; break;
				}
				const DVec3 d = face_dir({ a.key.face(), s, t });
				const SelectedNode *b = look.find(nodes, face_dir({ a.key.face(), s + ns, t + nt }));
				if (!b) {
					++r_t.missing;
					continue;
				}
				const int la = a.key.level(), lb = b->key.level();
				if (std::abs(la - lb) > 1) {
					++r_t.level_jumps;
				}
				if (la == lb + 1) {
					++r_t.boundary_vertices;
					const double dist = length(surface(d) - in.camera);
					if (morph(a, dist) < 1.0) {
						++r_t.fine_not_morphed;
					}
					if (morph(*b, dist) > 0.0) {
						++r_t.coarse_morphing;
					}
				}
			}
		}
	}
}

} // namespace

TEST_CASE(selection_is_crack_free) {
	std::mt19937_64 rng(11);
	std::normal_distribution<double> n(0.0, 1.0);
	const double r3 = 1.0 / std::sqrt(3.0);
	std::vector<DVec3> dirs = { { 0, 0, 1 }, { 0, 0, -1 }, { r3, r3, r3 }, { 0.7071067811865476, 0, 0.7071067811865475 } };
	while (dirs.size() < 12) {
		dirs.push_back(normalize(DVec3(n(rng), n(rng), n(rng))));
	}
	Tally t;
	for (const DVec3 &d : dirs) {
		for (double alt : { 2.0, 150.0, 12000.0, 400000.0, 6000000.0 }) {
			check_camera(d, alt, t);
		}
	}
	std::printf("  %zu cameras, up to %zu nodes; %ld boundary vertices between levels: "
				"%ld level jumps, %ld unmorphed fine, %ld morphing coarse, %ld unmatched\n",
			dirs.size() * 5, t.most_nodes, t.boundary_vertices, t.level_jumps, t.fine_not_morphed,
			t.coarse_morphing, t.missing);
	CHECK(t.boundary_vertices > 0);
	CHECK(t.level_jumps == 0);
	CHECK(t.fine_not_morphed == 0);
	CHECK(t.coarse_morphing == 0);
	CHECK(t.missing == 0);
}

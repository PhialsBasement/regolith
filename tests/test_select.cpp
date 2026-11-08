// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <algorithm>
#include <cmath>
#include <iterator>
#include <random>
#include <unordered_map>

#include "regolith/cube.hpp"
#include "regolith/select.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

constexpr double MARS_RADIUS_M = 3389500.0;
constexpr double RELIEF_BOUND_M = 4500.0; // bound on |relief|
constexpr int MAX_LEVEL = 15;
constexpr int QUADS_PER_EDGE = 52; // as drawn

// Nodes wider than this report the global relief bound, as
// a pack's coarse tiles do; finer ones sample the relief.
constexpr double COARSE_NODE_EDGE_M = 20000.0;
constexpr int BOUNDS_SAMPLES_PER_EDGE = 4;
constexpr double BOUNDS_MARGIN_M = 50.0;

constexpr size_t CAMERA_DIRECTIONS = 12;
constexpr double CAMERA_ALTITUDES_M[] = { 2.0, 150.0, 12000.0, 400000.0, 6000000.0 };

// Broad swells plus a 70 km wavelength at slopes up to ~0.13.
double relief(const DVec3 &p_dir) {
	return 3000.0 * std::sin(5.0 * p_dir.x + 1.0) * std::cos(7.0 * p_dir.y - 2.0 * p_dir.z) +
			1500.0 * std::sin(300.0 * p_dir.x) * std::cos(300.0 * p_dir.y);
}

DVec3 surface(const DVec3 &p_dir) {
	return p_dir * (MARS_RADIUS_M + relief(p_dir));
}

// Tile bounds as a loaded tile would report them.
bool tile_bounds(TileKey p_key, float &r_lowest, float &r_highest) {
	if (Selector::node_edge(MARS_RADIUS_M, p_key.level()) > COARSE_NODE_EDGE_M) {
		r_lowest = -float(RELIEF_BOUND_M);
		r_highest = float(RELIEF_BOUND_M);
		return true;
	}
	const double tiles = double(uint64_t(1) << p_key.level());
	const double samples = double(BOUNDS_SAMPLES_PER_EDGE);
	double lowest = 1e9, highest = -1e9;
	for (int b = 0; b <= BOUNDS_SAMPLES_PER_EDGE; ++b) {
		for (int a = 0; a <= BOUNDS_SAMPLES_PER_EDGE; ++a) {
			const double s = -1.0 + 2.0 * (p_key.x() + a / samples) / tiles;
			const double t = -1.0 + 2.0 * (p_key.y() + b / samples) / tiles;
			const double height = relief(face_dir({ p_key.face(), s, t }));
			lowest = std::min(lowest, height);
			highest = std::max(highest, height);
		}
	}
	r_lowest = float(lowest - BOUNDS_MARGIN_M);
	r_highest = float(highest + BOUNDS_MARGIN_M);
	return true;
}

// How far a node has morphed toward its parent at p_distance:
// 0 not at all, 1 fully.
double morph(const SelectedNode &p_node, double p_distance) {
	if (std::isinf(p_node.morph_end)) {
		return 0.0;
	}
	return std::clamp((p_distance - p_node.morph_start) / (p_node.morph_end - p_node.morph_start), 0.0, 1.0);
}

// The nodes selected for one camera, findable by direction.
struct Selection {
	DVec3 camera;
	std::vector<SelectedNode> nodes;
	std::unordered_map<uint64_t, size_t> index_by_key;

	// The finest selected node containing p_dir.
	const SelectedNode *node_at(const DVec3 &p_dir) const {
		const FaceCoord coord = dir_to_face(p_dir);
		for (int level = MAX_LEVEL; level >= 0; --level) {
			const double tiles = double(uint64_t(1) << level);
			auto cell = [&](double p_coordinate) {
				return uint32_t(std::clamp(std::floor((p_coordinate + 1.0) * 0.5 * tiles), 0.0, tiles - 1.0));
			};
			auto it = index_by_key.find(TileKey::make(coord.face, level, cell(coord.s), cell(coord.t)).bits);
			if (it != index_by_key.end()) {
				return &nodes[it->second];
			}
		}
		return nullptr;
	}
};

Selection select_for_camera(const DVec3 &p_camera_dir, double p_altitude) {
	Selection selection;
	selection.camera = surface(p_camera_dir) + p_camera_dir * p_altitude;
	SelectInput input;
	input.r_ref = MARS_RADIUS_M;
	input.max_level = MAX_LEVEL;
	input.camera = selection.camera;
	Selector selector;
	selector.rho_bounds = tile_bounds;
	selector.ready = [](TileKey) { return true; };
	std::vector<WantedTile> wanted;
	selector.select(input, selection.nodes, wanted);
	for (size_t k = 0; k < selection.nodes.size(); ++k) {
		selection.index_by_key[selection.nodes[k].key.bits] = k;
	}
	return selection;
}

// A point on one edge of a node, as face coordinates, and the
// small step that crosses that edge into the neighbour.
struct EdgePoint {
	double s, t;
	double step_s, step_t;
};

// p_edge: 0 low s, 1 high s, 2 low t, 3 high t; p_fraction
// runs 0 to 1 along it.
EdgePoint edge_point(TileKey p_key, int p_edge, double p_fraction) {
	const double tiles = double(uint64_t(1) << p_key.level());
	const double first_s = -1.0 + 2.0 * p_key.x() / tiles;
	const double first_t = -1.0 + 2.0 * p_key.y() / tiles;
	const double span = 2.0 / tiles;
	const double step = span * 1e-4;
	switch (p_edge) {
		case 0:
			return { first_s, first_t + p_fraction * span, -step, 0.0 };
		case 1:
			return { first_s + span, first_t + p_fraction * span, step, 0.0 };
		case 2:
			return { first_s + p_fraction * span, first_t, 0.0, -step };
		default:
			return { first_s + p_fraction * span, first_t + span, 0.0, step };
	}
}

struct Tally {
	long boundary_vertices = 0;
	long level_jumps = 0;
	long fine_not_morphed = 0;
	long coarse_morphing = 0;
	long missing = 0;
	size_t most_nodes = 0;
};

// Where a node meets a coarser neighbour, the fine node must
// be fully morphed and the coarse one not at all, or the
// shared vertex cracks.
void check_edge_point(const Selection &p_selection, const SelectedNode &p_node, const EdgePoint &p_point,
		Tally &r_tally) {
	const int face = p_node.key.face();
	const DVec3 dir = face_dir({ face, p_point.s, p_point.t });
	const DVec3 across = face_dir({ face, p_point.s + p_point.step_s, p_point.t + p_point.step_t });
	const SelectedNode *neighbour = p_selection.node_at(across);
	if (!neighbour) {
		++r_tally.missing;
		return;
	}
	const int level = p_node.key.level();
	const int neighbour_level = neighbour->key.level();
	if (std::abs(level - neighbour_level) > 1) {
		++r_tally.level_jumps;
	}
	if (level != neighbour_level + 1) {
		return;
	}
	++r_tally.boundary_vertices;
	const double distance = length(surface(dir) - p_selection.camera);
	if (morph(p_node, distance) < 1.0) {
		++r_tally.fine_not_morphed;
	}
	if (morph(*neighbour, distance) > 0.0) {
		++r_tally.coarse_morphing;
	}
}

void check_node_edges(const Selection &p_selection, const SelectedNode &p_node, Tally &r_tally) {
	for (int edge = 0; edge < 4; ++edge) {
		for (int k = 0; k <= QUADS_PER_EDGE; ++k) {
			const double fraction = double(k) / QUADS_PER_EDGE;
			check_edge_point(p_selection, p_node, edge_point(p_node.key, edge, fraction), r_tally);
		}
	}
}

void check_camera(const DVec3 &p_camera_dir, double p_altitude, Tally &r_tally) {
	const Selection selection = select_for_camera(p_camera_dir, p_altitude);
	r_tally.most_nodes = std::max(r_tally.most_nodes, selection.nodes.size());
	for (const SelectedNode &node : selection.nodes) {
		check_node_edges(selection, node, r_tally);
	}
}

} // namespace

TEST_CASE(selection_is_crack_free) {
	std::mt19937_64 rng(11);
	std::normal_distribution<double> n(0.0, 1.0);
	const double inv_sqrt3 = 1.0 / std::sqrt(3.0);
	// poles, a cube corner and an edge midpoint, then random
	std::vector<DVec3> camera_dirs = { { 0, 0, 1 }, { 0, 0, -1 }, { inv_sqrt3, inv_sqrt3, inv_sqrt3 },
		{ 0.7071067811865476, 0, 0.7071067811865475 } };
	while (camera_dirs.size() < CAMERA_DIRECTIONS) {
		camera_dirs.push_back(normalize(DVec3(n(rng), n(rng), n(rng))));
	}
	Tally tally;
	for (const DVec3 &dir : camera_dirs) {
		for (double altitude : CAMERA_ALTITUDES_M) {
			check_camera(dir, altitude, tally);
		}
	}
	std::printf("  %zu cameras, up to %zu nodes; %ld boundary vertices between levels: "
				"%ld level jumps, %ld unmorphed fine, %ld morphing coarse, %ld unmatched\n",
			camera_dirs.size() * std::size(CAMERA_ALTITUDES_M), tally.most_nodes, tally.boundary_vertices,
			tally.level_jumps, tally.fine_not_morphed, tally.coarse_morphing, tally.missing);
	CHECK(tally.boundary_vertices > 0);
	CHECK(tally.level_jumps == 0);
	CHECK(tally.fine_not_morphed == 0);
	CHECK(tally.coarse_morphing == 0);
	CHECK(tally.missing == 0);
}

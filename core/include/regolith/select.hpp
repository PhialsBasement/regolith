#pragma once

#include <array>
#include <functional>
#include <unordered_map>
#include <vector>

#include "regolith/tile_key.hpp"
#include "regolith/vec.hpp"

namespace regolith {

struct NodeSphere {
	DVec3 center; // body-fixed
	double radius = 0.0;
};

struct SelectInput {
	DVec3 camera; // body-fixed metres
	double r_ref = 0.0;
	// range(L) = range_factor * node edge at L; see docs/lod.md
	double range_factor = 6.0;
	double morph_start = 0.8; // fraction of range(L)
	int max_level = 0;
	double occluder_radius = 0.0; // horizon culling, 0 = off
	// Frustum planes, body-fixed: outside when n.p - d > 0.
	std::vector<std::array<double, 4>> planes;
};

struct SelectedNode {
	TileKey key;
	double morph_start = 0.0;
	double morph_end = 0.0;
	NodeSphere sphere;
};

struct WantedTile {
	TileKey key;
	float priority = 0.0f;
};

class Selector {
public:
	// Known rho min/max of a tile (false if unknown yet).
	std::function<bool(TileKey, float &, float &)> rho_bounds;
	// True once a tile can be drawn (its page is on the GPU).
	std::function<bool(TileKey)> ready;

	void select(const SelectInput &p_in, std::vector<SelectedNode> &r_nodes, std::vector<WantedTile> &r_wanted) const;
	NodeSphere bounds(const SelectInput &p_in, TileKey p_key) const;
	static double node_edge(double p_r_ref, int p_level);
	static double range(const SelectInput &p_in, int p_level);

private:
	void visit(const SelectInput &p_in, TileKey p_key, std::vector<SelectedNode> &r_nodes,
			std::vector<WantedTile> &r_wanted) const;
	bool culled(const SelectInput &p_in, const NodeSphere &p_s) const;
	NodeSphere compute_bounds(const SelectInput &p_in, TileKey p_key, TileKey p_known, float p_lo, float p_hi) const;

	// Spheres only change when a tile's height range does.
	struct CachedSphere {
		NodeSphere sphere;
		float lo = 0.0f;
		float hi = 0.0f;
		uint64_t known = 0;
	};
	mutable std::unordered_map<uint64_t, CachedSphere> spheres_;
};

} // namespace regolith

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <godot_cpp/variant/vector3.hpp>

#include "regolith/vec.hpp"

namespace godot {

class RegolithSystem;
class SceneTree;

// Nodes fixed to a body that a game puts in the group "regolith_fixed"
// (a few roots, each carrying many children), on the body named by
// their meta BODY_META, or the dominant one. While the render frame
// turns with their body they stay put by themselves. While it does not,
// their ground moves through the frame: roots on another body than the
// dominant one are put back on their spot every tick (they are what is
// seen across space); roots on the dominant body are put back every
// tick near the focus or the camera, and the others once, when they
// come near or the frame turns with the body again.
class FixedCarrier {
public:
	static constexpr const char *GROUP = "regolith_fixed";
	static constexpr const char *BODY_META = "regolith_body";

	// Start of a tick, before time moves on: the spots of roots the frame
	// carries now, and of roots new to the group; once the frame turns
	// with the body again, the stale roots put back.
	void note(const RegolithSystem &p_sys, SceneTree *p_tree);
	// After the bodies moved on: the roots within p_range_m of a point of
	// p_near on their spot; the rest marked to be put back later. Returns
	// how many were moved.
	int carry(const RegolithSystem &p_sys, SceneTree *p_tree, const std::vector<Vector3> &p_near, double p_range_m);

private:
	struct Spot {
		uint64_t body_id = 0; // the body it is fixed to (instance id)
		regolith::DVec3 origin, x, y, z; // body-fixed: position and basis columns
		bool stale = false; // not put back since the ground moved
	};
	static const class RegolithBody *body_of(const RegolithSystem &p_sys, const class Node3D *p_node);
	// the frame turns with p_body: what stands on it stays put
	static bool held(const RegolithSystem &p_sys, const class RegolithBody *p_body);
	static Spot spot_of(const RegolithSystem &p_sys, const class Node3D *p_node, const class RegolithBody *p_body);
	static void put_back(const RegolithSystem &p_sys, class Node3D *p_node, const Spot &p_spot);

	std::unordered_map<uint64_t, Spot> spots_; // node instance id -> its spot
};

} // namespace godot

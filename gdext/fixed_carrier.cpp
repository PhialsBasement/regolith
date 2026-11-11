// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "fixed_carrier.hpp"

#include <unordered_set>

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/variant/typed_array.hpp>

#include "convert.hpp"
#include "regolith_body.hpp"
#include "regolith_system.hpp"

using namespace godot;

FixedCarrier::Spot FixedCarrier::spot_of(const RegolithSystem &p_sys, const Node3D *p_node) {
	const RegolithBody *body = p_sys.dominant();
	const regolith::RenderFrame f = p_sys.frame_for(body);
	const Transform3D xf = p_node->get_global_transform();
	Spot s;
	s.body_id = body->get_instance_id();
	s.origin = regolith::to_body_point(f, regolith::to_d(xf.origin));
	s.x = regolith::to_body_vec(f, regolith::to_d(xf.basis.get_column(0)));
	s.y = regolith::to_body_vec(f, regolith::to_d(xf.basis.get_column(1)));
	s.z = regolith::to_body_vec(f, regolith::to_d(xf.basis.get_column(2)));
	return s;
}

void FixedCarrier::put_back(const RegolithSystem &p_sys, Node3D *p_node, const Spot &p_spot) {
	const RegolithBody *body = Object::cast_to<RegolithBody>(ObjectDB::get_instance(p_spot.body_id));
	if (!body) {
		return;
	}
	const regolith::RenderFrame f = p_sys.frame_for(body);
	Basis b;
	b.set_column(0, regolith::to_v(f.to_local_vec(p_spot.x)));
	b.set_column(1, regolith::to_v(f.to_local_vec(p_spot.y)));
	b.set_column(2, regolith::to_v(f.to_local_vec(p_spot.z)));
	p_node->set_global_transform(Transform3D(b, regolith::to_v(f.to_local_point(p_spot.origin))));
}

void FixedCarrier::note(const RegolithSystem &p_sys, SceneTree *p_tree) {
	if (!p_tree || !p_sys.dominant()) {
		return;
	}
	const bool carried = !p_sys.frame_moving(); // the frame turns with the body: what stands there stays
	TypedArray<Node> roots = p_tree->get_nodes_in_group(GROUP);
	std::unordered_set<uint64_t> here;
	for (int i = 0; i < roots.size(); ++i) {
		Node3D *n = Object::cast_to<Node3D>(roots[i]);
		if (!n || !n->is_inside_tree()) {
			continue;
		}
		const uint64_t id = n->get_instance_id();
		here.insert(id);
		auto it = spots_.find(id);
		if (it != spots_.end() && it->second.stale && carried) {
			put_back(p_sys, n, it->second); // the frame turns with the body again: back once, then it stays
		}
		if (carried || it == spots_.end()) {
			spots_[id] = spot_of(p_sys, n); // where the game has it now is its spot
		}
	}
	for (auto it = spots_.begin(); it != spots_.end();) {
		it = here.count(it->first) ? std::next(it) : spots_.erase(it);
	}
}

int FixedCarrier::carry(const RegolithSystem &p_sys, SceneTree *p_tree, const std::vector<Vector3> &p_near,
		double p_range_m) {
	int moved = 0;
	if (!p_tree || !p_sys.frame_moving() || spots_.empty()) {
		return moved;
	}
	TypedArray<Node> roots = p_tree->get_nodes_in_group(GROUP);
	for (int i = 0; i < roots.size(); ++i) {
		Node3D *n = Object::cast_to<Node3D>(roots[i]);
		auto it = n ? spots_.find(n->get_instance_id()) : spots_.end();
		if (it == spots_.end()) {
			continue;
		}
		const RegolithBody *body = Object::cast_to<RegolithBody>(ObjectDB::get_instance(it->second.body_id));
		if (!body) {
			continue;
		}
		const Vector3 at = regolith::to_v(p_sys.frame_for(body).to_local_point(it->second.origin));
		bool near = false;
		for (const Vector3 &p : p_near) {
			near = near || at.distance_to(p) < p_range_m;
		}
		if (!near) {
			it->second.stale = true; // put back once it comes near, or the frame turns with the body again
			continue;
		}
		put_back(p_sys, n, it->second);
		it->second.stale = false;
		++moved;
	}
	return moved;
}

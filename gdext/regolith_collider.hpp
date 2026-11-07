// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <godot_cpp/classes/node.hpp>

namespace godot {

// Put one under any physics body (walker, vehicle, drone, pod):
// the terrain keeps collision patches within `radius` of it,
// fetched ahead along its velocity for `lookahead` seconds.
class RegolithCollider : public Node {
	GDCLASS(RegolithCollider, Node)

public:
	void _enter_tree() override;

	void set_radius(double p_v) { radius_ = p_v; }
	double get_radius() const { return radius_; }
	void set_lookahead(double p_v) { lookahead_ = p_v; }
	double get_lookahead() const { return lookahead_; }

protected:
	static void _bind_methods();

private:
	double radius_ = 48.0;
	double lookahead_ = 1.0;
};

} // namespace godot

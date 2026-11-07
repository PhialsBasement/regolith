// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith_collider.hpp"

#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void RegolithCollider::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_radius", "metres"), &RegolithCollider::set_radius);
	ClassDB::bind_method(D_METHOD("get_radius"), &RegolithCollider::get_radius);
	ClassDB::bind_method(D_METHOD("set_lookahead", "seconds"), &RegolithCollider::set_lookahead);
	ClassDB::bind_method(D_METHOD("get_lookahead"), &RegolithCollider::get_lookahead);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "radius"), "set_radius", "get_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "lookahead"), "set_lookahead", "get_lookahead");
}

void RegolithCollider::_enter_tree() {
	add_to_group("regolith_collider");
}

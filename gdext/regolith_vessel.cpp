#include "regolith_vessel.hpp"

#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void RegolithVessel::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_thrusting", "thrusting"), &RegolithVessel::set_thrusting);
	ClassDB::bind_method(D_METHOD("get_thrusting"), &RegolithVessel::get_thrusting);
	ClassDB::bind_method(D_METHOD("set_rails_distance", "metres"), &RegolithVessel::set_rails_distance);
	ClassDB::bind_method(D_METHOD("get_rails_distance"), &RegolithVessel::get_rails_distance);
	ClassDB::bind_method(D_METHOD("is_parked"), &RegolithVessel::is_parked);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "thrusting"), "set_thrusting", "get_thrusting");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rails_distance"), "set_rails_distance", "get_rails_distance");
}

void RegolithVessel::_enter_tree() {
	add_to_group("regolith_vessel");
	if (Node *p = get_parent()) {
		p->add_to_group("regolith_anchored");
	}
}

void RegolithVessel::_ready() {
	// Gravity, damping and sleep are the plugin's business.
	if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(get_parent())) {
		rb->set_gravity_scale(0.0);
		rb->set_linear_damp_mode(RigidBody3D::DAMP_MODE_REPLACE);
		rb->set_linear_damp(0.0);
		rb->set_angular_damp_mode(RigidBody3D::DAMP_MODE_REPLACE);
		rb->set_angular_damp(0.0);
		rb->set_can_sleep(false);
	}
}

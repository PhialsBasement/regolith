#pragma once

#include <godot_cpp/classes/node.hpp>

namespace godot {

// Put one under any physics body (RigidBody3D or
// CharacterBody3D). The plugin then owns its frame shifts, its
// gravity (RigidBody3D) and parks it on rails while it is far
// from the focus and not thrusting.
class RegolithVessel : public Node {
	GDCLASS(RegolithVessel, Node)

public:
	void _enter_tree() override;
	void _ready() override;

	void set_thrusting(bool p_v) { thrusting_ = p_v; }
	bool get_thrusting() const { return thrusting_; }
	void set_rails_distance(double p_v) { rails_distance_ = p_v; }
	double get_rails_distance() const { return rails_distance_; }
	bool is_parked() const { return parked_; }

	// System-owned parking state (ICRF or body-fixed).
	bool parked_ = false;
	bool landed_ = false;
	const Node *park_body_ = nullptr;
	double park_time_ = 0.0;
	double park_r_[3] = {};
	double park_v_[3] = {};
	double park_basis_[9] = {};
	bool was_frozen_ = false;

protected:
	static void _bind_methods();

private:
	bool thrusting_ = false;
	double rails_distance_ = 2500.0;
};

} // namespace godot

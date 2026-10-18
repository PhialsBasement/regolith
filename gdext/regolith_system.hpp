#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "regolith/patch.hpp"

namespace godot {

class RegolithBody;

// Owns the floating origin: a body-fixed anchor near the focus
// node with east/up/south axes. Re-anchors (one rigid
// transform, invisible) as the focus moves, and answers
// gravity and position queries in render-frame coordinates.
class RegolithSystem : public Node {
	GDCLASS(RegolithSystem, Node)

public:
	RegolithSystem();
	void _ready() override;
	void _process(double p_delta) override;
	void _physics_process(double p_delta) override;

	const regolith::RenderFrame &frame() const { return frame_; }
	RegolithBody *body() const { return body_; }

	String get_version() const;
	Dictionary self_test() const;

	void set_body_path(const NodePath &p_path) { body_path_ = p_path; }
	NodePath get_body_path() const { return body_path_; }
	void set_focus_path(const NodePath &p_path) { focus_path_ = p_path; }
	NodePath get_focus_path() const { return focus_path_; }
	void set_sun_path(const NodePath &p_path) { sun_path_ = p_path; }
	NodePath get_sun_path() const { return sun_path_; }
	void set_start_latitude(double p_v) { start_lat_ = p_v; }
	double get_start_latitude() const { return start_lat_; }
	void set_start_longitude(double p_v) { start_lon_ = p_v; }
	double get_start_longitude() const { return start_lon_; }
	void set_start_local_time(double p_v) { start_local_time_ = p_v; }
	double get_start_local_time() const { return start_local_time_; }
	void set_manage_camera(bool p_v) { manage_camera_ = p_v; }
	bool get_manage_camera() const { return manage_camera_; }
	double get_time() const { return time_; }
	double get_local_solar_time(const Vector3 &p_pos) const;
	Vector3 get_sun_direction() const;

	Vector3 get_gravity(const Vector3 &p_pos) const;
	Vector3 get_up(const Vector3 &p_pos) const;
	Vector2 get_latlon(const Vector3 &p_pos) const;
	double get_altitude(const Vector3 &p_pos) const;
	double get_ground_clearance(const Vector3 &p_pos) const;
	double get_radius(const Vector3 &p_pos) const;
	int get_rebase_count() const { return rebases_; }

protected:
	static void _bind_methods();

private:
	void anchor_at_start();
	void rebase(const regolith::DVec3 &p_anchor);

	NodePath body_path_;
	NodePath focus_path_;
	NodePath sun_path_;
	double start_lat_ = -5.4;
	double start_lon_ = 137.8;
	double start_local_time_ = 10.0; // hours, local true solar
	bool manage_camera_ = true;
	double time_ = 0.0; // TT seconds since J2000
	RegolithBody *body_ = nullptr;
	regolith::RenderFrame frame_;
	bool anchored_ = false;
	int rebases_ = 0;
};

} // namespace godot

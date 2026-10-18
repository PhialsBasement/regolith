#pragma once

#include <unordered_map>
#include <vector>

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "regolith/patch.hpp"

namespace godot {

class Camera3D;
class CanvasLayer;
class RegolithBody;
class SubViewport;

// Floating origin in the frame of the body with the nearest
// surface; re-anchoring or switching bodies is one invisible
// rigid transform. Also runs the clock, gravity and cameras.
class RegolithSystem : public Node {
	GDCLASS(RegolithSystem, Node)

public:
	RegolithSystem();
	void _ready() override;
	void _process(double p_delta) override;
	void _physics_process(double p_delta) override;

	// The render frame expressed in p_body's own fixed axes.
	regolith::RenderFrame frame_for(const RegolithBody *p_body) const;
	// Changes whenever frame_for(p_body) does.
	uint64_t frame_version(const RegolithBody *p_body) const;
	RegolithBody *dominant() const { return dominant_; }

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
	double get_sun_visible() const { return sun_visible_; }
	Vector3 get_gravity(const Vector3 &p_pos) const;
	Vector3 get_up(const Vector3 &p_pos) const;
	Vector2 get_latlon(const Vector3 &p_pos) const;
	double get_altitude(const Vector3 &p_pos) const;
	double get_ground_clearance(const Vector3 &p_pos) const;
	double get_radius(const Vector3 &p_pos) const;
	String get_dominant_name() const;
	Array get_bodies_info(const Vector3 &p_pos) const;
	Vector3 get_body_position(const String &p_name) const;
	int get_rebase_count() const { return rebases_; }

protected:
	static void _bind_methods();

private:
	struct Pose {
		regolith::DVec3 position; // ICRF, central-body centred
		regolith::DVec3 axes[3]; // body-fixed axes as ICRF columns
	};

	void anchor_at_start();
	void update_poses();
	const Pose &pose_of(const RegolithBody *p_body) const;
	regolith::DVec3 to_inertial(const Vector3 &p_render) const;
	regolith::DVec3 inertial_vec_to_render(const regolith::DVec3 &p_v) const;
	void re_anchor(RegolithBody *p_body, const regolith::DVec3 &p_inertial);
	void setup_passes();
	void update_passes(Camera3D *p_cam);
	void update_sun();

	NodePath body_path_;
	NodePath focus_path_;
	NodePath sun_path_;
	double start_lat_ = -5.4;
	double start_lon_ = 137.8;
	double start_local_time_ = 10.0; // hours, local true solar
	bool manage_camera_ = true;
	double time_ = 0.0; // TT seconds since J2000

	std::vector<RegolithBody *> bodies_;
	RegolithBody *dominant_ = nullptr;
	std::unordered_map<const RegolithBody *, Pose> poses_;
	uint64_t pose_counter_ = 0;
	regolith::RenderFrame frame_; // in dominant_'s fixed axes
	bool anchored_ = false;
	int rebases_ = 0;

	SubViewport *far_view_ = nullptr;
	Camera3D *far_camera_ = nullptr;
	CanvasLayer *far_layer_ = nullptr;
	double sun_energy_ = -1.0;
	double sun_visible_ = 1.0;
};

} // namespace godot

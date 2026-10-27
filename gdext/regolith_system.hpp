#pragma once

#include <unordered_map>
#include <vector>

#include <godot_cpp/classes/environment.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/packed_float64_array.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "regolith/patch.hpp"

namespace godot {

class Camera3D;
class CanvasLayer;
class Node3D;
class RegolithBody;
class RegolithVessel;
class SubViewport;

// Owns the reference frame everything is simulated in (see
// docs/frames.md): a body-centred frame that turns with the
// body near its surface and is inertial above it, a floating
// origin, and a floating velocity. Also the clock, gravity,
// vessels on rails and the camera passes.
class RegolithSystem : public Node {
	GDCLASS(RegolithSystem, Node)

public:
	struct Pose {
		regolith::DVec3 position; // ICRF, central-body centred
		regolith::DVec3 velocity;
		regolith::DVec3 axes[3]; // body-fixed axes, ICRF columns
		regolith::DVec3 omega; // angular velocity, ICRF
		regolith::DVec3 omega_dot;
	};
	struct State {
		regolith::DVec3 x; // ICRF, central-body centred
		regolith::DVec3 v;
	};

	RegolithSystem();
	void _ready() override;
	void _process(double p_delta) override;
	void _physics_process(double p_delta) override;

	// The render frame expressed in p_body's fixed axes.
	regolith::RenderFrame frame_for(const RegolithBody *p_body) const;
	// Bumps whenever the frame is rebuilt (re-anchor, switch).
	uint64_t rebuild_count() const { return uint64_t(rebases_); }
	RegolithBody *dominant() const { return dominant_; }
	bool is_parked(const Node *p_node) const;
	const Pose &pose_of(const RegolithBody *p_body) const;
	State to_state(const Vector3 &p_pos, const Vector3 &p_vel) const;
	void from_state(const State &p_s, Vector3 &r_pos, Vector3 &r_vel) const;
	// Spin of p_body as seen in the render frame.
	Vector3 render_angular_velocity(const RegolithBody *p_body) const;
	// Columns: ICRF axes in render space.
	Basis icrf_to_render() const { return render_to_icrf().transposed(); }
	bool is_anchored() const { return anchored_; }
	RegolithBody *find_body(const String &p_name) const;

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
	void set_floating_origin(bool p_v) { floating_origin_ = p_v; }
	bool get_floating_origin() const { return floating_origin_; }
	void set_floating_velocity(bool p_v) { floating_velocity_enabled_ = p_v; }
	bool get_floating_velocity() const { return floating_velocity_enabled_; }

	double get_time() const { return time_; }
	double get_local_solar_time(const Vector3 &p_pos) const;
	Vector3 get_sun_direction() const;
	double get_sun_visible() const { return sun_visible_; }
	Vector3 get_gravity(const Vector3 &p_pos) const;
	Vector3 get_acceleration(const Vector3 &p_pos, const Vector3 &p_vel) const;
	Vector3 get_up(const Vector3 &p_pos) const;
	Vector2 get_latlon(const Vector3 &p_pos) const;
	double get_altitude(const Vector3 &p_pos) const;
	double get_ground_clearance(const Vector3 &p_pos) const;
	double get_radius(const Vector3 &p_pos) const;
	String get_dominant_name() const;
	bool get_frame_rotating() const { return rotating_; }
	// True while the dominant body's ground moves in render space.
	bool frame_moving() const { return !rotating_ || regolith::length(frame_velocity_) > 0.0; }
	Vector3 get_frame_velocity() const;
	Array get_bodies_info(const Vector3 &p_pos) const;
	Vector3 get_body_position(const String &p_name) const;
	Dictionary get_orbit_info(Node *p_node) const;
	PackedFloat64Array get_inertial_state(Node *p_node) const;
	void put_in_orbit(Node *p_node, const String &p_body, double p_altitude, double p_inclination_deg);
	// At rest relative to p_body's surface, p_height above it.
	void put_near(Node *p_node, const String &p_body, double p_lat_deg, double p_lon_deg, double p_height);
	int get_rebase_count() const { return rebases_; }
	int get_shift_count() const { return shifts_; }
	Dictionary get_stats() const { return stats_; }

protected:
	static void _bind_methods();

private:
	struct Live {
		Node3D *node;
		State s;
		Basis basis_icrf;
		Vector3 omega_icrf;
	};

	void anchor_at_start();
	void update_poses();
	const regolith::DVec3 *ref_axes() const;
	regolith::DVec3 ref_omega() const;
	regolith::DVec3 ref_omega_dot() const;
	Basis render_to_icrf() const;
	regolith::DVec3 field(const regolith::DVec3 &p_x, const RegolithBody *p_skip) const;
	regolith::DVec3 accel_icrf(const State &p_s, bool p_coriolis) const;
	regolith::DVec3 body_point(const RegolithBody *p_body, const Vector3 &p_pos) const;
	RegolithBody *nearest_surface(const regolith::DVec3 &p_x) const;
	double clearance(const RegolithBody *p_b, const regolith::DVec3 &p_x) const;

	Node3D *focus_node() const;
	bool node_state(Node3D *p_node, State &r_s) const;
	void rebuild_frame(RegolithBody *p_body, bool p_rotating, const State &p_focus, Node3D *p_focus_node);
	std::vector<Live> capture_live() const;
	void restore_live(const std::vector<Live> &p_live);
	void manage_frame(Node3D *p_focus);
	void floating_velocity(Node3D *p_focus);
	void update_vessels(Node3D *p_focus, double p_delta);
	void park(RegolithVessel *p_vessel, Node3D *p_node);
	void unpark(RegolithVessel *p_vessel, Node3D *p_node);
	State parked_state(const RegolithVessel *p_vessel) const;
	void place_parked(RegolithVessel *p_vessel, Node3D *p_node);

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
	// Near/far camera pair; off renders the whole range in one.
	bool split_passes_ = false;
	// Only single-precision builds need the 2 km leash; double
	// builds are exact at planet scale. Floating velocity stays
	// on in both: Jolt keeps velocities in float32.
#ifdef REAL_T_IS_DOUBLE
	bool floating_origin_ = false;
#else
	bool floating_origin_ = true;
#endif
	bool floating_velocity_enabled_ = true;
	double time_ = 0.0; // TT seconds since J2000

	std::vector<RegolithBody *> bodies_;
	std::unordered_map<const RegolithBody *, Pose> poses_;
	RegolithBody *dominant_ = nullptr;
	bool rotating_ = true;
	regolith::DVec3 frozen_[3]; // ref axes while inertial
	regolith::DVec3 frame_velocity_; // ref coords, m/s
	regolith::RenderFrame frame_; // ref coords
	bool anchored_ = false;
	int rebases_ = 0;
	int shifts_ = 0;
	Dictionary stats_;
	Node3D *last_focus_ = nullptr;

	SubViewport *far_view_ = nullptr;
	Camera3D *far_camera_ = nullptr;
	CanvasLayer *far_layer_ = nullptr;
	Ref<Environment> far_env_;
	Ref<Environment> near_env_;
	double sun_energy_ = -1.0;
	double sun_visible_ = 1.0;
};

} // namespace godot

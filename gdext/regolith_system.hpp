// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

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

#include "fixed_carrier.hpp"
#include "regolith/patch.hpp"

namespace godot {

class Camera3D;
class CanvasLayer;
class Node3D;
class RegolithBody;
class RegolithVessel;
class SubViewport;

// Camera-following passes run after the game's scripts, so they see
// this frame's camera; each adds an offset to keep their order.
constexpr int CAMERA_PASSES_PRIORITY = 1 << 24;

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
	// The body the others' positions are relative to: the one
	// without an orbit table. Its heliocentric orbit places the
	// sun (AU, ICRF), for every body.
	RegolithBody *central() const { return central_; }
	regolith::DVec3 central_heliocentric(double p_t) const;
	const RegolithBody *season_body() const;
	// the camera from the sun (ICRF, m), and the sun's brightness
	// there over the central body's (inverse square)
	regolith::DVec3 camera_heliocentric() const;
	double get_sun_scale() const;
	// unit vector to the sun in a body's fixed axes, now
	regolith::DVec3 sun_in_body(const RegolithBody *p_body) const;
	// the same at any time, from the body's own pose there
	regolith::DVec3 sun_in_body_at(const RegolithBody *p_body, double p_t) const;
	bool is_parked(const Node *p_node) const;
	const Pose &pose_of(const RegolithBody *p_body) const;
	bool has_body(const RegolithBody *p_body) const;
	void adopt(RegolithBody *p_body);
	State to_state(const Vector3 &p_pos, const Vector3 &p_vel) const;
	void from_state(const State &p_s, Vector3 &r_pos, Vector3 &r_vel) const;
	// Spin of p_body as seen in the render frame.
	Vector3 render_angular_velocity(const RegolithBody *p_body) const;
	// Columns: ICRF axes in render space.
	Basis icrf_to_render() const { return render_to_icrf().transposed(); }
	bool is_anchored() const { return anchored_; }
	RegolithBody *find_body(const String &p_name) const;
	// the body a point query is about: p_name, or the dominant one when
	// it is empty; null (warned once) for an unknown name
	RegolithBody *query_body(const String &p_name) const;
	// A render-space point in p_body's fixed frame (m).
	regolith::DVec3 body_point(const RegolithBody *p_body, const Vector3 &p_pos) const;

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
	void set_fixed_range(double p_v) { fixed_range_m_ = p_v; }
	double get_fixed_range() const { return fixed_range_m_; }
	bool get_floating_velocity() const { return floating_velocity_enabled_; }

	double get_time() const { return time_; }
	double get_local_solar_time(const Vector3 &p_pos, const String &p_body = String()) const;
	Vector3 get_sun_direction() const;
	double get_sun_visible() const { return sun_visible_; }
	// How much of the sun's disc the bodies leave visible at p_pos
	// (eclipses and transits; the body p_pos stands on is its night,
	// not an eclipse), 0..1; 1 without a body.
	double get_eclipse_visibility(const Vector3 &p_pos) const;
	Vector3 get_gravity(const Vector3 &p_pos) const;
	Vector3 get_acceleration(const Vector3 &p_pos, const Vector3 &p_vel) const;
	Vector3 get_up(const Vector3 &p_pos) const;
	// p_body names the body asked about; empty: the dominant one
	Vector2 get_latlon(const Vector3 &p_pos, const String &p_body = String()) const;
	double get_altitude(const Vector3 &p_pos, const String &p_body = String()) const;
	double get_ground_clearance(const Vector3 &p_pos, const String &p_body = String()) const;
	// How much of the sun's disc clears the terrain at p_pos, 0..1
	// (terrain only: eclipses are get_eclipse_visibility); NaN without a body.
	double get_sun_visibility(const Vector3 &p_pos, const String &p_body = String()) const;
	double get_radius(const Vector3 &p_pos, const String &p_body = String()) const;
	String get_dominant_name() const;
	bool get_frame_rotating() const { return rotating_; }
	// True while the dominant body's ground moves in render space.
	bool frame_moving() const { return !rotating_ || regolith::length(frame_velocity_) > 0.0; }
	Vector3 get_frame_velocity() const;
	Array get_bodies_info(const Vector3 &p_pos) const;
	Vector3 get_body_position(const String &p_name) const;
	Dictionary get_orbit_info(Node *p_node) const;
	PackedFloat64Array get_inertial_state(Node *p_node) const;
	// Places a node at an ICRF state (central-body centred: x, y,
	// z, vx, vy, vz in m and m/s), as get_inertial_state gives it.
	void set_inertial_state(Node *p_node, const PackedFloat64Array &p_state);
	// A body's (or "Sun"'s) ICRF state now, as above
	PackedFloat64Array get_body_state(const String &p_name) const;
	// the same at any time (TT seconds since J2000): for planning
	// transfers to where a body will be
	PackedFloat64Array get_body_state_at(const String &p_name, double p_time) const;
	// Sphere of influence radius (m): a (m / M)^0.4 about its
	// parent; INF for "Sun"
	double get_soi_radius(const String &p_name) const;
	// the same at any time
	double get_soi_radius_at(const String &p_name, double p_time) const;
	// A body's (or "Sun"'s) gravitational parameter, m3/s2; 0 if unknown
	double get_body_gm(const String &p_name) const;
	// The body a body orbits ("Sun" for a planet); "" for the sun or unknown
	String get_body_parent(const String &p_name) const;
	// The path ahead of an ICRF state at a time as patched conics, at
	// most p_patches: [{"body", "start", "end" (TT s), "state" (relative
	// to the body at the start, as get_inertial_state), "how" ("orbit",
	// "runs_on", "impact", "encounter", "escape"), "next" (the body met)}]
	Array predict_path(const PackedFloat64Array &p_state, double p_time, int p_patches) const;
	// Time warp: above 1 every vessel, the focus included, rides
	// on rails (Kepler orbits, patched at spheres of influence)
	void set_warp(double p_rate);
	double get_warp() const { return warp_; }
	void put_in_orbit(Node *p_node, const String &p_body, double p_altitude, double p_inclination_deg);
	// At rest relative to p_body's surface, p_height above it.
	void put_near(Node *p_node, const String &p_body, double p_lat_deg, double p_lon_deg, double p_height);
	// render-space point p_height above a body's surface (from
	// cached heights: it never blocks on a tile read)
	Vector3 get_surface_point(const String &p_body, double p_lat_deg, double p_lon_deg, double p_height);
	// ground points for (lat, lon) degree pairs in one call, from
	// cached heights; NaN where the ground is not streamed yet
	PackedVector3Array get_ground_points(const String &p_body, const PackedFloat64Array &p_latlon);
	// The render point at (lat, lon) and a fixed radius from the
	// body's centre: body-fixed, whatever the terrain cache holds
	Vector3 get_body_point(const String &p_body, double p_lat_deg, double p_lon_deg, double p_radius);
	// The central body's solar longitude (deg) for today's date,
	// the date the clock starts from (Mars's before it anchors)
	double get_solar_longitude_today() const;
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
	RegolithBody *nearest_surface(const regolith::DVec3 &p_x) const;
	double clearance(const RegolithBody *p_b, const regolith::DVec3 &p_x) const;

	Node3D *focus_node() const;
	bool node_state(Node3D *p_node, State &r_s) const;
	void rebuild_frame(RegolithBody *p_body, bool p_rotating, const State &p_focus, Node3D *p_focus_node);
	void carry_fixed();
	std::vector<Live> capture_live() const;
	void restore_live(const std::vector<Live> &p_live);
	void manage_frame(Node3D *p_focus);
	void floating_velocity(Node3D *p_focus);
	void update_vessels(Node3D *p_focus, double p_delta);
	void place_all_parked();
	bool rests_landed(Node3D *p_node, int p_depth);
	void park(RegolithVessel *p_vessel, Node3D *p_node);
	void unpark(RegolithVessel *p_vessel, Node3D *p_node);
	// back to physics for the focus: the frame takes its velocity
	void unpark_focus(RegolithVessel *p_vessel, Node3D *p_node);
	State parked_state(const RegolithVessel *p_vessel) const;
	State parked_state_at(const RegolithVessel *p_vessel, double p_t) const;
	void place_parked(RegolithVessel *p_vessel, Node3D *p_node);
	// A Kepler arc about body (null: the sun): its state relative to the body at t0.
	struct Arc {
		const RegolithBody *body = nullptr;
		regolith::DVec3 r, v;
		double t0 = 0.0;
	};
	// What an arc runs into first: a body's ground, or another sphere
	// of influence (body: the one struck, or the new primary).
	struct ArcEnd {
		enum Kind { RUNS_ON, IMPACT, CHANGE } kind = RUNS_ON;
		double t = 0.0;
		const RegolithBody *body = nullptr;
	};
	State arc_state(const Arc &p_arc, double p_t) const;
	Arc arc_of(const RegolithVessel *p_vessel) const;
	ArcEnd trace_arc(const Arc &p_arc, double p_t0, double p_t1, int p_max_steps) const;
	double trace_step(const Arc &p_arc, const State &p_s, double p_t, double p_min_soi) const;
	const RegolithBody *struck(const regolith::DVec3 &p_x, double p_t) const;
	double ground_clearance_at(const RegolithBody *p_body, const regolith::DVec3 &p_x, double p_t) const;
	const RegolithBody *parent_of(const RegolithBody *p_body) const;
	static String body_name(const RegolithBody *p_body);
	void land_parked(RegolithVessel *p_vessel, const RegolithBody *p_body, double p_t);
	// patched conics: re-park at each sphere-of-influence change
	// since the last check, in steps no ship can skip one in
	void advance_parked(RegolithVessel *p_vessel);
	void repark(RegolithVessel *p_vessel, const State &p_s, double p_t, const RegolithBody *p_primary);

	// The sun and the bodies at any time (null body: the sun).
	State sun_state_at(double p_t) const;
	State body_state_at(const RegolithBody *p_body, double p_t) const;
	double soi_at(const RegolithBody *p_body, double p_t) const;
	// the body whose sphere of influence holds p_x (the smallest
	// that does), or null for the sun
	const RegolithBody *primary_at(const regolith::DVec3 &p_x, double p_t) const;
	double gm_of(const RegolithBody *p_body) const;

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
	// Only single-precision builds need the frame to follow its focus
	// (re-centred at each rebuild and on a 2 km leash); double builds
	// are exact at planet scale and never move a position. Floating
	// velocity stays on in both: Jolt keeps velocities in float32.
#ifdef REAL_T_IS_DOUBLE
	bool floating_origin_ = false;
#else
	bool floating_origin_ = true;
#endif
	bool floating_velocity_enabled_ = true;
	double time_ = 0.0; // TT seconds since J2000
	double warp_ = 1.0;
	// the sun now (ICRF, central-body centred), with the poses
	regolith::DVec3 sun_pos_, sun_vel_;

	std::vector<RegolithBody *> bodies_;
	std::unordered_map<const RegolithBody *, Pose> poses_;
	RegolithBody *dominant_ = nullptr;
	RegolithBody *central_ = nullptr;
	bool rotating_ = true;
	regolith::DVec3 frozen_[3]; // ref axes while inertial
	regolith::DVec3 frame_velocity_; // ref coords, m/s
	regolith::RenderFrame frame_; // ref coords
	bool anchored_ = false;
	int rebases_ = 0;
	int shifts_ = 0;
	Dictionary stats_;
	FixedCarrier fixed_; // the game's planet-fixed roots, kept on their spots
	double fixed_range_m_ = 2500.0; // roots this near the focus or the camera are carried every tick
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

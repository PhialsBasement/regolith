// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <random>

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/visual_instance3d.hpp>
#include <godot_cpp/classes/canvas_layer.hpp>
#include <godot_cpp/classes/label.hpp>

#include "regolith_engine.hpp"
#include <godot_cpp/classes/character_body3d.hpp>
#include <godot_cpp/classes/collision_object3d.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/directional_light3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/environment.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/classes/texture_rect.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/viewport_texture.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>

#include "convert.hpp"
#include "regolith/cube.hpp"
#include "regolith/ephemeris.hpp"
#include "regolith/kepler.hpp"
#include "regolith_body.hpp"
#include "regolith_vessel.hpp"

using namespace godot;
using regolith::DVec3;

namespace {

constexpr double DEG = std::numbers::pi / 180.0;
constexpr double AU = 1.495978707e11;
constexpr double SUN_RADIUS = 6.957e8;
constexpr double GM_SUN = 1.32712440018e20; // m^3/s^2 (IAU)
// ecliptic pole of J2000 in ICRF: obliquity 23.4392911 deg
const DVec3 ECLIPTIC_POLE(0.0, -0.39777715593191, 0.91748206206918);
// time warp: the largest rate (rails make any rate exact)
constexpr double WARP_MAX = 1e7;
// Each camera pass spans this far/near ratio; float32
// projections lose the far plane well before 1e8.
constexpr double PASS_RATIO = 1e6;
// m: past Neptune, so every planet stays drawn (reversed depth
// keeps the precision near the camera)
constexpr double FAR_PLANE = 5e12;
// Floating velocity: Jolt never sees the focus faster than
// SHIFT_HIGH; under SHIFT_LOW true speed the frame stops.
constexpr double SHIFT_HIGH = 50.0;
constexpr double SHIFT_LOW = 30.0;
// Above this render speed an unparked rigid body would be
// clamped by Jolt, so it stays on rails.
constexpr double UNPARK_MAX_SPEED = 400.0;
constexpr double OMEGA_STEP = 20.0;

DVec3 to_icrf(const DVec3 p_axes[3], const DVec3 &p_v) {
	return p_axes[0] * p_v.x + p_axes[1] * p_v.y + p_axes[2] * p_v.z;
}

DVec3 from_icrf(const DVec3 p_axes[3], const DVec3 &p_v) {
	return { regolith::dot(p_axes[0], p_v), regolith::dot(p_axes[1], p_v), regolith::dot(p_axes[2], p_v) };
}

// Angular velocity taking axes p_a to p_b over p_dt seconds.
DVec3 omega_between(const DVec3 p_a[3], const DVec3 p_b[3], double p_dt) {
	double r[3][3] = {};
	for (int k = 0; k < 3; ++k) {
		const double b[3] = { p_b[k].x, p_b[k].y, p_b[k].z };
		const double a[3] = { p_a[k].x, p_a[k].y, p_a[k].z };
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				r[i][j] += b[i] * a[j];
			}
		}
	}
	return DVec3(r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]) * (0.5 / p_dt);
}

Basis basis_from(const DVec3 &p_x, const DVec3 &p_y, const DVec3 &p_z) {
	Basis b;
	b.set_column(0, regolith::to_v(p_x));
	b.set_column(1, regolith::to_v(p_y));
	b.set_column(2, regolith::to_v(p_z));
	return b;
}

// Jolt sweeps kinematic bodies (CharacterBody3D, frozen
// RigidBody3D) to a new transform over the next step; a frame
// change must teleport them outside the space instead.
void teleport(Node3D *p_node, const Transform3D &p_xf) {
	CollisionObject3D *co = Object::cast_to<CollisionObject3D>(p_node);
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	const RID rid = co ? co->get_rid() : RID();
	const RID space = rid.is_valid() ? ps->body_get_space(rid) : RID();
	if (space.is_valid()) {
		ps->body_set_space(rid, RID());
	}
	p_node->set_global_transform(p_xf);
	if (space.is_valid()) {
		ps->body_set_state(rid, PhysicsServer3D::BODY_STATE_TRANSFORM, p_node->get_global_transform());
		ps->body_set_space(rid, space);
	}
}

RegolithVessel *vessel_of(Node *p_node) {
	if (!p_node) {
		return nullptr;
	}
	for (int i = 0; i < p_node->get_child_count(); ++i) {
		if (RegolithVessel *v = Object::cast_to<RegolithVessel>(p_node->get_child(i))) {
			return v;
		}
	}
	return nullptr;
}

} // namespace

RegolithSystem::RegolithSystem() {
	set_process_priority(-100);
	set_physics_process_priority(-100);
}

void RegolithSystem::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_version"), &RegolithSystem::get_version);
	ClassDB::bind_method(D_METHOD("self_test"), &RegolithSystem::self_test);
	ClassDB::bind_method(D_METHOD("get_gravity", "position"), &RegolithSystem::get_gravity);
	ClassDB::bind_method(D_METHOD("get_acceleration", "position", "velocity"), &RegolithSystem::get_acceleration);
	ClassDB::bind_method(D_METHOD("get_up", "position"), &RegolithSystem::get_up);
	ClassDB::bind_method(D_METHOD("get_latlon", "position"), &RegolithSystem::get_latlon);
	ClassDB::bind_method(D_METHOD("get_altitude", "position"), &RegolithSystem::get_altitude);
	ClassDB::bind_method(D_METHOD("get_ground_clearance", "position"), &RegolithSystem::get_ground_clearance);
	ClassDB::bind_method(D_METHOD("get_radius", "position"), &RegolithSystem::get_radius);
	ClassDB::bind_method(D_METHOD("get_rebase_count"), &RegolithSystem::get_rebase_count);
	ClassDB::bind_method(D_METHOD("get_shift_count"), &RegolithSystem::get_shift_count);
	ClassDB::bind_method(D_METHOD("get_stats"), &RegolithSystem::get_stats);
	ClassDB::bind_method(D_METHOD("get_time"), &RegolithSystem::get_time);
	ClassDB::bind_method(D_METHOD("is_anchored"), &RegolithSystem::is_anchored);
	ClassDB::bind_method(D_METHOD("get_local_solar_time", "position"), &RegolithSystem::get_local_solar_time);
	ClassDB::bind_method(D_METHOD("get_sun_direction"), &RegolithSystem::get_sun_direction);
	ClassDB::bind_method(D_METHOD("get_sun_scale"), &RegolithSystem::get_sun_scale);
	ClassDB::bind_method(D_METHOD("get_sun_visible"), &RegolithSystem::get_sun_visible);
	ClassDB::bind_method(D_METHOD("get_dominant_name"), &RegolithSystem::get_dominant_name);
	ClassDB::bind_method(D_METHOD("get_frame_rotating"), &RegolithSystem::get_frame_rotating);
	ClassDB::bind_method(D_METHOD("get_frame_velocity"), &RegolithSystem::get_frame_velocity);
	ClassDB::bind_method(D_METHOD("get_bodies_info", "position"), &RegolithSystem::get_bodies_info);
	ClassDB::bind_method(D_METHOD("get_body_position", "name"), &RegolithSystem::get_body_position);
	ClassDB::bind_method(D_METHOD("get_orbit_info", "node"), &RegolithSystem::get_orbit_info);
	ClassDB::bind_method(D_METHOD("get_inertial_state", "node"), &RegolithSystem::get_inertial_state);
	ClassDB::bind_method(D_METHOD("set_inertial_state", "node", "state"), &RegolithSystem::set_inertial_state);
	ClassDB::bind_method(D_METHOD("get_body_state", "name"), &RegolithSystem::get_body_state);
	ClassDB::bind_method(D_METHOD("get_body_state_at", "name", "time"), &RegolithSystem::get_body_state_at);
	ClassDB::bind_method(D_METHOD("get_soi_radius", "name"), &RegolithSystem::get_soi_radius);
	ClassDB::bind_method(D_METHOD("set_warp", "rate"), &RegolithSystem::set_warp);
	ClassDB::bind_method(D_METHOD("get_warp"), &RegolithSystem::get_warp);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "warp", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NONE), "set_warp", "get_warp");
	ClassDB::bind_method(D_METHOD("put_in_orbit", "node", "body", "altitude", "inclination_deg"), &RegolithSystem::put_in_orbit);
	ClassDB::bind_method(D_METHOD("put_near", "node", "body", "lat_deg", "lon_deg", "height"), &RegolithSystem::put_near);
	ClassDB::bind_method(D_METHOD("get_surface_point", "body", "lat_deg", "lon_deg", "height"),
			&RegolithSystem::get_surface_point);
	ClassDB::bind_method(D_METHOD("get_ground_points", "body", "latlon_deg"), &RegolithSystem::get_ground_points);
	ClassDB::bind_method(D_METHOD("get_body_point", "body", "lat_deg", "lon_deg", "radius"), &RegolithSystem::get_body_point);
	ClassDB::bind_method(D_METHOD("get_solar_longitude_today"), &RegolithSystem::get_solar_longitude_today);

	ClassDB::bind_method(D_METHOD("set_body_path", "path"), &RegolithSystem::set_body_path);
	ClassDB::bind_method(D_METHOD("get_body_path"), &RegolithSystem::get_body_path);
	ClassDB::bind_method(D_METHOD("set_focus_path", "path"), &RegolithSystem::set_focus_path);
	ClassDB::bind_method(D_METHOD("get_focus_path"), &RegolithSystem::get_focus_path);
	ClassDB::bind_method(D_METHOD("set_sun_path", "path"), &RegolithSystem::set_sun_path);
	ClassDB::bind_method(D_METHOD("get_sun_path"), &RegolithSystem::get_sun_path);
	ClassDB::bind_method(D_METHOD("set_start_latitude", "degrees"), &RegolithSystem::set_start_latitude);
	ClassDB::bind_method(D_METHOD("get_start_latitude"), &RegolithSystem::get_start_latitude);
	ClassDB::bind_method(D_METHOD("set_start_longitude", "degrees"), &RegolithSystem::set_start_longitude);
	ClassDB::bind_method(D_METHOD("get_start_longitude"), &RegolithSystem::get_start_longitude);
	ClassDB::bind_method(D_METHOD("set_start_local_time", "hours"), &RegolithSystem::set_start_local_time);
	ClassDB::bind_method(D_METHOD("get_start_local_time"), &RegolithSystem::get_start_local_time);
	ClassDB::bind_method(D_METHOD("set_manage_camera", "enabled"), &RegolithSystem::set_manage_camera);
	ClassDB::bind_method(D_METHOD("get_manage_camera"), &RegolithSystem::get_manage_camera);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "body"), "set_body_path", "get_body_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "focus"), "set_focus_path", "get_focus_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "sun"), "set_sun_path", "get_sun_path");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "start_latitude"), "set_start_latitude", "get_start_latitude");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "start_longitude"), "set_start_longitude", "get_start_longitude");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "start_local_time", PROPERTY_HINT_RANGE, "0,24,0.1"), "set_start_local_time",
			"get_start_local_time");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "manage_camera"), "set_manage_camera", "get_manage_camera");
	ClassDB::bind_method(D_METHOD("set_floating_origin", "enabled"), &RegolithSystem::set_floating_origin);
	ClassDB::bind_method(D_METHOD("get_floating_origin"), &RegolithSystem::get_floating_origin);
	ClassDB::bind_method(D_METHOD("set_floating_velocity", "enabled"), &RegolithSystem::set_floating_velocity);
	ClassDB::bind_method(D_METHOD("get_floating_velocity"), &RegolithSystem::get_floating_velocity);
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "floating_origin"), "set_floating_origin", "get_floating_origin");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "floating_velocity"), "set_floating_velocity", "get_floating_velocity");
	ADD_SIGNAL(MethodInfo("frame_shifted", PropertyInfo(Variant::TRANSFORM3D, "transform")));
	// a vessel on rails struck a body: now landed there
	ADD_SIGNAL(MethodInfo("vessel_impact", PropertyInfo(Variant::OBJECT, "node"), PropertyInfo(Variant::STRING, "body"),
			PropertyInfo(Variant::FLOAT, "speed")));
}

String RegolithSystem::get_version() const {
	return "0.4.0-m4";
}

Dictionary RegolithSystem::self_test() const {
	std::mt19937_64 rng(7);
	std::normal_distribution<double> n(0.0, 1.0);
	double worst = 0.0;
	for (int i = 0; i < 100000; ++i) {
		const DVec3 d = regolith::normalize(DVec3(n(rng), n(rng), n(rng)));
		worst = std::fmax(worst, regolith::length(regolith::face_dir(regolith::dir_to_face(d)) - d));
	}
	char text[32];
	std::snprintf(text, sizeof(text), "%.3e", worst);
	Dictionary result;
	result["dir_round_trip_worst_text"] = String(text);
	result["ok"] = worst < 1e-15;
	return result;
}

void RegolithSystem::_ready() {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	if (!regolith_engine_double()) {
		// broken, so impossible to miss: a banner over the game
		CanvasLayer *layer = memnew(CanvasLayer);
		layer->set_layer(128);
		Label *l = memnew(Label);
		l->set_text("REGOLITH: SINGLE-PRECISION GODOT, BROKEN.\nUse a double-precision build (precision=double).");
		l->set_horizontal_alignment(HORIZONTAL_ALIGNMENT_CENTER);
		l->set_anchors_preset(Control::PRESET_TOP_WIDE);
		l->set_offset(SIDE_TOP, 160.0);
		l->add_theme_font_size_override("font_size", 40);
		l->add_theme_color_override("font_color", Color(1.0, 0.2, 0.15));
		l->add_theme_color_override("font_outline_color", Color(0, 0, 0));
		l->add_theme_constant_override("outline_size", 10);
		layer->add_child(l);
		add_child(layer);
	}
	dominant_ = Object::cast_to<RegolithBody>(get_node_or_null(body_path_));
	if (!dominant_) {
		ERR_PRINT("RegolithSystem: body path does not point at a RegolithBody");
		return;
	}
	TypedArray<Node> found = get_tree()->get_nodes_in_group("regolith_body");
	for (int i = 0; i < found.size(); ++i) {
		if (RegolithBody *b = Object::cast_to<RegolithBody>(found[i])) {
			if (b->ensure_open()) {
				bodies_.push_back(b);
				if (!central_ && !b->has_orbit_table()) {
					central_ = b;
				}
			}
		}
	}
	if (!central_) {
		central_ = dominant_;
	}
	anchor_at_start();
	if (manage_camera_) {
		if (split_passes_) {
			setup_passes();
		}
	}
}

void RegolithSystem::anchor_at_start() {
	// Real clock: today's date, moved forward to the next
	// start_local_time at the start longitude.
	time_ = regolith::j2000_seconds_from_unix(Time::get_singleton()->get_unix_time_from_system());
	auto local_time = [&](double p_t) {
		return regolith::local_solar_time(sun_in_body_at(dominant_, p_t), start_lon_);
	};
	// this body's solar day, from how fast its local time runs
	const double gained = std::fmod(local_time(time_ + 600.0) - local_time(time_) + 48.0, 24.0);
	const double sol = gained > 1e-9 ? 24.0 / gained * 600.0 : regolith::MARS_SOL;
	for (int k = 0; k < 4; ++k) {
		double dh = start_local_time_ - local_time(time_);
		dh = k == 0 ? std::fmod(dh + 24.0, 24.0) : std::remainder(dh, 24.0);
		time_ += dh / 24.0 * sol;
	}
	update_poses();
	const double lat = start_lat_ * DEG;
	const double lon = start_lon_ * DEG;
	const DVec3 dir(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
	rotating_ = true;
	frame_ = regolith::RenderFrame::at(dir * dominant_->surface_radius(dir, true));
	anchored_ = true;
}

RegolithSystem::State RegolithSystem::sun_state_at(double p_t) const {
	const double dt = 60.0;
	auto at = [&](double p_s) { return central_heliocentric(p_s) * (-AU); };
	return { at(p_t), (at(p_t + dt) - at(p_t - dt)) / (2.0 * dt) };
}

RegolithSystem::State RegolithSystem::body_state_at(const RegolithBody *p_body, double p_t) const {
	if (!p_body) {
		return sun_state_at(p_t);
	}
	State s;
	DVec3 axes[3];
	p_body->pose(p_t, s.x, s.v, axes);
	// another planet: its own orbit round the sun, relative to
	// the central body's
	if (p_body != central_ && !p_body->has_orbit_table()) {
		const double dt = 60.0;
		auto rel = [&](double p_s) {
			return (regolith::heliocentric(p_body->orbit_elements(), p_s) - central_heliocentric(p_s)) * AU;
		};
		s.x = rel(p_t);
		s.v = (rel(p_t + dt) - rel(p_t - dt)) / (2.0 * dt);
	}
	return s;
}

double RegolithSystem::gm_of(const RegolithBody *p_body) const {
	return p_body ? p_body->get_gm() : GM_SUN;
}

double RegolithSystem::soi_at(const RegolithBody *p_body, double p_t) const {
	if (!p_body) {
		return INFINITY;
	}
	const double gm = p_body->get_gm();
	if (gm <= 0.0) {
		return 0.0;
	}
	if (p_body == central_ || !p_body->has_orbit_table()) {
		// a planet, about the sun: its mean distance
		return p_body->orbit_elements().a[0] * AU * std::pow(gm / GM_SUN, 0.4);
	}
	// a moon, about the central body: its distance now
	const double d = regolith::length(body_state_at(p_body, p_t).x - body_state_at(central_, p_t).x);
	return d * std::pow(gm / std::max(central_->get_gm(), 1.0), 0.4);
}

const RegolithBody *RegolithSystem::primary_at(const DVec3 &p_x, double p_t) const {
	const RegolithBody *best = nullptr;
	double best_soi = INFINITY;
	for (const RegolithBody *b : bodies_) {
		const double soi = soi_at(b, p_t);
		const DVec3 pos = p_t == time_ ? pose_of(b).position : body_state_at(b, p_t).x;
		if (soi > 0.0 && soi < best_soi && regolith::length(p_x - pos) < soi) {
			best = b;
			best_soi = soi;
		}
	}
	return best;
}

void RegolithSystem::update_poses() {
	const State sun = sun_state_at(time_);
	sun_pos_ = sun.x;
	sun_vel_ = sun.v;
	for (RegolithBody *b : bodies_) {
		Pose &p = poses_[b];
		b->pose(time_, p.position, p.velocity, p.axes);
		if (b != central_ && !b->has_orbit_table()) {
			const State s = body_state_at(b, time_);
			p.position = s.x;
			p.velocity = s.v;
		}
		DVec3 a[5][3];
		DVec3 dummy_p, dummy_v;
		for (int k = 0; k < 5; ++k) {
			b->pose(time_ + (k - 2) * OMEGA_STEP, dummy_p, dummy_v, a[k]);
		}
		p.omega = omega_between(a[1], a[3], 2.0 * OMEGA_STEP);
		const DVec3 w_minus = omega_between(a[0], a[2], 2.0 * OMEGA_STEP);
		const DVec3 w_plus = omega_between(a[2], a[4], 2.0 * OMEGA_STEP);
		p.omega_dot = (w_plus - w_minus) / (2.0 * OMEGA_STEP);
	}
}

const RegolithSystem::Pose &RegolithSystem::pose_of(const RegolithBody *p_body) const {
	auto it = poses_.find(p_body);
	if (it != poses_.end()) {
		return it->second;
	}
	// A body the system does not hold (not anchored yet, or its
	// pack failed to open): never abort the game over it. Callers
	// that can tell check has_body first.
	static const Pose none = [] {
		Pose p;
		p.axes[0] = DVec3(1, 0, 0);
		p.axes[1] = DVec3(0, 1, 0);
		p.axes[2] = DVec3(0, 0, 1);
		return p;
	}();
	ERR_PRINT_ONCE("RegolithSystem: asked for a body it does not hold (not anchored yet, or its pack failed to open)");
	return none;
}

bool RegolithSystem::has_body(const RegolithBody *p_body) const {
	return poses_.count(p_body) != 0;
}

void RegolithSystem::adopt(RegolithBody *p_body) {
	// Bodies ready before the system are found by its _ready; one
	// ready later (added at runtime, or back in the tree) joins here.
	if (!anchored_ || !p_body || std::find(bodies_.begin(), bodies_.end(), p_body) != bodies_.end()) {
		return;
	}
	if (!p_body->ensure_open()) {
		return;
	}
	bodies_.push_back(p_body);
	update_poses();
}

const DVec3 *RegolithSystem::ref_axes() const {
	return rotating_ ? pose_of(dominant_).axes : frozen_;
}

DVec3 RegolithSystem::ref_omega() const {
	return rotating_ ? pose_of(dominant_).omega : DVec3();
}

DVec3 RegolithSystem::ref_omega_dot() const {
	return rotating_ ? pose_of(dominant_).omega_dot : DVec3();
}

Basis RegolithSystem::render_to_icrf() const {
	const DVec3 *a = ref_axes();
	return basis_from(to_icrf(a, frame_.east), to_icrf(a, frame_.up), to_icrf(a, frame_.south));
}

RegolithSystem::State RegolithSystem::to_state(const Vector3 &p_pos, const Vector3 &p_vel) const {
	const Pose &o = pose_of(dominant_);
	const DVec3 *a = ref_axes();
	const DVec3 r = to_icrf(a, regolith::to_body_point(frame_, regolith::to_d(p_pos)));
	const DVec3 v_ref = regolith::to_body_vec(frame_, regolith::to_d(p_vel)) + frame_velocity_;
	return { o.position + r, o.velocity + to_icrf(a, v_ref) + regolith::cross(ref_omega(), r) };
}

void RegolithSystem::from_state(const State &p_s, Vector3 &r_pos, Vector3 &r_vel) const {
	const Pose &o = pose_of(dominant_);
	const DVec3 *a = ref_axes();
	const DVec3 r = p_s.x - o.position;
	r_pos = regolith::to_v(frame_.to_local_point(from_icrf(a, r)));
	const DVec3 v_ref = from_icrf(a, p_s.v - o.velocity - regolith::cross(ref_omega(), r));
	r_vel = regolith::to_v(frame_.to_local_vec(v_ref - frame_velocity_));
}

RegolithBody *RegolithSystem::find_body(const String &p_name) const {
	for (RegolithBody *b : bodies_) {
		if (String(b->get_name()) == p_name) {
			return b;
		}
	}
	return nullptr;
}

Vector3 RegolithSystem::render_angular_velocity(const RegolithBody *p_body) const {
	const DVec3 w = pose_of(p_body).omega - ref_omega();
	return regolith::to_v(frame_.to_local_vec(from_icrf(ref_axes(), w)));
}

regolith::RenderFrame RegolithSystem::frame_for(const RegolithBody *p_body) const {
	if ((p_body == dominant_ && rotating_) || !anchored_) {
		return frame_;
	}
	const Pose &o = pose_of(dominant_);
	const Pose &b = pose_of(p_body);
	const DVec3 *a = ref_axes();
	regolith::RenderFrame g;
	g.anchor = from_icrf(b.axes, o.position + to_icrf(a, frame_.anchor) - b.position);
	g.east = from_icrf(b.axes, to_icrf(a, frame_.east));
	g.up = from_icrf(b.axes, to_icrf(a, frame_.up));
	g.south = from_icrf(b.axes, to_icrf(a, frame_.south));
	return g;
}

DVec3 RegolithSystem::field(const DVec3 &p_x, const RegolithBody *p_skip) const {
	DVec3 g;
	{
		// the sun: near a planet only its tide is left once the
		// frame's own fall is taken off (accel_icrf)
		const DVec3 d = p_x - sun_pos_;
		const double r = regolith::length(d);
		if (r > SUN_RADIUS * 0.5) {
			g = g - d * (GM_SUN / (r * r * r));
		}
	}
	for (const RegolithBody *b : bodies_) {
		if (b == p_skip) {
			continue;
		}
		const Pose &bp = pose_of(b);
		const DVec3 d = p_x - bp.position;
		const double r = regolith::length(d);
		if (r < 1.0) {
			continue;
		}
		const double gm = b->get_gm();
		g = g - d * (gm / (r * r * r));
		const double j2 = b->get_j2();
		if (j2 != 0.0) {
			// Oblateness: -(3/2) J2 GM R^2/r^4 [(1-5c^2) d/r + 2c k]
			const DVec3 k = bp.axes[2];
			const double c = regolith::dot(d, k) / r;
			const double rr = b->get_j2_radius();
			const double f = 1.5 * j2 * gm * rr * rr / (r * r * r * r);
			g = g - (d * ((1.0 - 5.0 * c * c) / r) + k * (2.0 * c)) * f;
		}
	}
	return g;
}

DVec3 RegolithSystem::accel_icrf(const State &p_s, bool p_coriolis) const {
	const Pose &o = pose_of(dominant_);
	// The frame's origin falls freely with the dominant body,
	// so only the tide of the others is left of their pull.
	DVec3 a = field(p_s.x, nullptr) - field(o.position, dominant_);
	if (rotating_) {
		const DVec3 w = o.omega;
		const DVec3 r = p_s.x - o.position;
		a = a - regolith::cross(w, regolith::cross(w, r)) - regolith::cross(o.omega_dot, r);
		if (p_coriolis) {
			const DVec3 v_rel = p_s.v - o.velocity - regolith::cross(w, r);
			a = a - regolith::cross(w, v_rel) * 2.0;
		}
	}
	return a;
}

DVec3 RegolithSystem::body_point(const RegolithBody *p_body, const Vector3 &p_pos) const {
	const Pose &b = pose_of(p_body);
	return from_icrf(b.axes, to_state(p_pos, Vector3()).x - b.position);
}

double RegolithSystem::clearance(const RegolithBody *p_b, const DVec3 &p_x) const {
	const Pose &b = pose_of(p_b);
	const DVec3 local = from_icrf(b.axes, p_x - b.position);
	const double surface = const_cast<RegolithBody *>(p_b)->surface_radius(regolith::normalize(local), false);
	const double r = regolith::length(local);
	return std::isnan(surface) ? r - p_b->r_ref() : r - surface;
}

RegolithBody *RegolithSystem::nearest_surface(const DVec3 &p_x) const {
	RegolithBody *best = nullptr;
	double best_c = 0.0;
	for (RegolithBody *b : bodies_) {
		const double c = (regolith::length(p_x - pose_of(b).position) - b->r_ref()) / b->r_ref();
		if (!best || c < best_c) {
			best = b;
			best_c = c;
		}
	}
	return best;
}

Node3D *RegolithSystem::focus_node() const {
	Node3D *focus = Object::cast_to<Node3D>(get_node_or_null(focus_path_));
	if (!focus && get_viewport()) {
		focus = get_viewport()->get_camera_3d();
	}
	return focus;
}

bool RegolithSystem::node_state(Node3D *p_node, State &r_s) const {
	if (!p_node) {
		return false;
	}
	if (RegolithVessel *v = vessel_of(p_node); v && v->parked_) {
		r_s = parked_state(v);
		return true;
	}
	Vector3 vel;
	if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(p_node)) {
		vel = cb->get_velocity();
	} else if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(p_node)) {
		vel = rb->get_linear_velocity();
	}
	r_s = to_state(p_node->get_global_position(), vel);
	return true;
}

bool RegolithSystem::is_parked(const Node *p_node) const {
	return p_node && p_node->is_in_group("regolith_parked");
}

std::vector<RegolithSystem::Live> RegolithSystem::capture_live() const {
	std::vector<Live> out;
	const Basis ra = render_to_icrf();
	const DVec3 w = ref_omega();
	TypedArray<Node> nodes = get_tree()->get_nodes_in_group("regolith_anchored");
	for (int i = 0; i < nodes.size(); ++i) {
		Node3D *n = Object::cast_to<Node3D>(nodes[i]);
		if (!n || is_parked(n)) {
			continue;
		}
		Live l;
		l.node = n;
		node_state(n, l.s);
		l.basis_icrf = ra * n->get_global_basis();
		Vector3 ang;
		if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(n)) {
			ang = rb->get_angular_velocity();
		}
		l.omega_icrf = ra.xform(ang) + regolith::to_v(w);
		out.push_back(l);
	}
	return out;
}

void RegolithSystem::restore_live(const std::vector<Live> &p_live) {
	const Basis to_render = render_to_icrf().transposed();
	const Vector3 w = regolith::to_v(ref_omega());
	for (const Live &l : p_live) {
		Vector3 pos, vel;
		from_state(l.s, pos, vel);
		Transform3D t(to_render * l.basis_icrf, pos);
		t.basis.orthonormalize();
		teleport(l.node, t);
		if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(l.node)) {
			cb->set_velocity(vel);
		} else if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(l.node)) {
			rb->set_linear_velocity(vel);
			rb->set_angular_velocity(to_render.xform(l.omega_icrf - w));
		}
	}
}

void RegolithSystem::rebuild_frame(RegolithBody *p_body, bool p_rotating, const State &p_focus, Node3D *p_focus_node) {
	// The focus takes p_focus, which may differ from where it
	// is (put_in_orbit): each node is written once, because
	// Jolt acts on a stale transform set in the same frame.
	std::vector<Live> live = capture_live();
	for (Live &l : live) {
		if (l.node == p_focus_node) {
			l.s = p_focus;
		}
	}
	const Basis old_ra = render_to_icrf();
	const State origin_before = to_state(Vector3(), Vector3());
	Vector3 old_focus_vel;
	if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(p_focus_node)) {
		old_focus_vel = cb->get_velocity();
	} else if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(p_focus_node)) {
		old_focus_vel = rb->get_linear_velocity();
	}
	if (is_parked(p_focus_node)) {
		old_focus_vel = Vector3();
	}

	dominant_ = p_body;
	rotating_ = p_rotating;
	const Pose &o = pose_of(dominant_);
	if (!rotating_) {
		for (int k = 0; k < 3; ++k) {
			frozen_[k] = o.axes[k];
		}
	}
	const DVec3 *a = ref_axes();
	const DVec3 r = p_focus.x - o.position;
	frame_ = regolith::RenderFrame::at(from_icrf(a, r));
	frame_velocity_ = DVec3();
	// Keep the focus's small render velocity: the rest of its
	// true velocity becomes the frame's.
	const DVec3 v_ref = from_icrf(a, p_focus.v - o.velocity - regolith::cross(ref_omega(), r));
	// Only in orbit: a moving frame near the ground drags every
	// planet-fixed node (rocks, lights, machines) along with it.
	if (floating_velocity_enabled_ && !rotating_ && regolith::length(v_ref) > SHIFT_LOW) {
		const Vector3 keep = render_to_icrf().transposed().xform(old_ra.xform(old_focus_vel));
		frame_velocity_ = v_ref - regolith::to_body_vec(frame_, regolith::to_d(keep));
	}
	restore_live(live);

	Vector3 c, unused;
	from_state(origin_before, c, unused);
	Transform3D t(render_to_icrf().transposed() * old_ra, c);
	++rebases_;
	emit_signal("frame_shifted", t);
}

void RegolithSystem::manage_frame(Node3D *p_focus) {
	State s;
	node_state(p_focus, s);
	RegolithBody *best = nearest_surface(s.x);
	auto closeness = [&](const RegolithBody *p_b) {
		return (regolith::length(s.x - pose_of(p_b).position) - p_b->r_ref()) / p_b->r_ref();
	};
	RegolithBody *body = dominant_;
	if (best != dominant_ && closeness(best) < 0.8 * closeness(dominant_)) {
		body = best;
	}
	const double alt = regolith::length(s.x - pose_of(body).position) - body->r_ref();
	const double limit = body->get_rotating_altitude();
	bool rotating = body == dominant_ ? rotating_ : alt < limit;
	if (rotating && alt > limit * 1.05) {
		rotating = false;
	} else if (!rotating && alt < limit * 0.95) {
		rotating = true;
	}
	const bool switched_focus = p_focus != last_focus_;
	last_focus_ = p_focus;

	Vector3 local = p_focus->get_global_position();
	if (is_parked(p_focus)) {
		Vector3 unused;
		from_state(s, local, unused);
	}
	const double r = body->r_ref();
	const double leash = std::fmax(std::fmin(2000.0, 0.02 * r), 0.05 * std::fabs(alt));
	const bool strayed = floating_origin_ && local.length() > leash;
	if (body != dominant_ || rotating != rotating_ || switched_focus || strayed) {
		rebuild_frame(body, rotating, s, p_focus);
	}
	if (RegolithVessel *v = vessel_of(p_focus); v && v->parked_ && warp_ <= 1.0) {
		unpark_focus(v, p_focus);
	}
}

void RegolithSystem::floating_velocity(Node3D *p_focus) {
	Vector3 v_focus;
	if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(p_focus)) {
		v_focus = cb->get_velocity();
	} else if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(p_focus)) {
		v_focus = rb->get_linear_velocity();
	} else {
		return;
	}
	const DVec3 v_true = regolith::to_body_vec(frame_, regolith::to_d(v_focus)) + frame_velocity_;
	Vector3 dv;
	if (regolith::length(frame_velocity_) > 0.0 && regolith::length(v_true) < SHIFT_LOW) {
		dv = -regolith::to_v(frame_.to_local_vec(frame_velocity_));
		frame_velocity_ = DVec3();
	} else if (v_focus.length() > SHIFT_HIGH) {
		dv = v_focus;
		frame_velocity_ = frame_velocity_ + regolith::to_body_vec(frame_, regolith::to_d(v_focus));
	} else {
		return;
	}
	TypedArray<Node> nodes = get_tree()->get_nodes_in_group("regolith_anchored");
	for (int i = 0; i < nodes.size(); ++i) {
		Node *n = Object::cast_to<Node>(nodes[i]);
		if (is_parked(n)) {
			continue;
		}
		if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(n)) {
			cb->set_velocity(cb->get_velocity() - dv);
		} else if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(n)) {
			rb->set_linear_velocity(rb->get_linear_velocity() - dv);
		}
	}
	++shifts_;
}

RegolithSystem::State RegolithSystem::parked_state(const RegolithVessel *p_vessel) const {
	return parked_state_at(p_vessel, time_);
}

RegolithSystem::State RegolithSystem::parked_state_at(const RegolithVessel *p_vessel, double p_t) const {
	const RegolithBody *b = Object::cast_to<RegolithBody>(const_cast<Node *>(p_vessel->park_body_));
	const DVec3 r(p_vessel->park_r_[0], p_vessel->park_r_[1], p_vessel->park_r_[2]);
	if (p_vessel->landed_) {
		const Pose &bp = pose_of(b);
		const DVec3 x = bp.position + to_icrf(bp.axes, r);
		return { x, bp.velocity + regolith::cross(bp.omega, x - bp.position) };
	}
	const State origin = p_t == time_ ? (b ? State{ pose_of(b).position, pose_of(b).velocity } : State{ sun_pos_, sun_vel_ })
									  : body_state_at(b, p_t);
	const DVec3 v(p_vessel->park_v_[0], p_vessel->park_v_[1], p_vessel->park_v_[2]);
	DVec3 rr, vv;
	regolith::kepler_propagate(r, v, gm_of(b), p_t - p_vessel->park_time_, rr, vv);
	return { origin.x + rr, origin.v + vv };
}

void RegolithSystem::repark(RegolithVessel *p_vessel, const State &p_s, double p_t, const RegolithBody *p_primary) {
	const State o = body_state_at(p_primary, p_t);
	const DVec3 r = p_s.x - o.x, v = p_s.v - o.v;
	for (int k = 0; k < 3; ++k) {
		p_vessel->park_r_[k] = k == 0 ? r.x : (k == 1 ? r.y : r.z);
		p_vessel->park_v_[k] = k == 0 ? v.x : (k == 1 ? v.y : v.z);
	}
	p_vessel->park_body_ = p_primary;
	p_vessel->park_time_ = p_t;
}

void RegolithSystem::advance_parked(RegolithVessel *p_vessel) {
	const double t0 = p_vessel->park_check_time_, t1 = time_;
	p_vessel->park_check_time_ = t1;
	if (p_vessel->landed_ || t1 <= t0) {
		return;
	}
	// steps short enough that no sphere of influence (the
	// smallest one a ship can be in) is crossed within one
	double min_soi = INFINITY;
	for (const RegolithBody *b : bodies_) {
		const double soi = soi_at(b, t0);
		if (soi > b->r_ref()) {
			min_soi = std::min(min_soi, soi);
		}
	}
	// height over the ground of a point near p_body at time p_t
	auto ground_clearance = [&](const RegolithBody *p_body, const DVec3 &p_x, double p_t) {
		DVec3 pos, vel, axes[3];
		p_body->pose(p_t, pos, vel, axes);
		pos = body_state_at(p_body, p_t).x;
		const DVec3 local = from_icrf(axes, p_x - pos);
		const double surface = const_cast<RegolithBody *>(p_body)->surface_radius(regolith::normalize(local), false);
		return regolith::length(local) - (std::isnan(surface) ? p_body->r_ref() : surface);
	};
	double t = t0;
	State s_prev = parked_state_at(p_vessel, t0);
	for (int iter = 0; iter < 20000 && t < t1; ++iter) {
		const RegolithBody *pb = Object::cast_to<RegolithBody>(const_cast<Node *>(p_vessel->park_body_));
		const DVec3 rel_v = s_prev.v - body_state_at(pb, t).v;
		const double speed = regolith::length(rel_v) + 1.0;
		double step = std::isfinite(min_soi) ? 0.1 * min_soi / speed : t1 - t;
		// near the ground of the body it orbits: finer, so no
		// pass through the planet slips between two steps
		if (pb) {
			const double h = ground_clearance(pb, s_prev.x, t);
			step = std::min(step, std::max(0.5 * h, 50.0) / speed);
		}
		const double t_next = std::min(t1, t + std::max(step, 1e-3));
		const State s = parked_state_at(p_vessel, t_next);
		if (pb && ground_clearance(pb, s.x, t_next) < 0.0) {
			// the impact: its moment by bisection, then set down
			double lo = t, hi = t_next;
			for (int k = 0; k < 40; ++k) {
				const double mid = 0.5 * (lo + hi);
				(ground_clearance(pb, parked_state_at(p_vessel, mid).x, mid) < 0.0 ? hi : lo) = mid;
			}
			const State si = parked_state_at(p_vessel, lo);
			DVec3 bpos, bvel, axes[3];
			pb->pose(lo, bpos, bvel, axes);
			const State bs = body_state_at(pb, lo);
			const DVec3 dir = regolith::normalize(from_icrf(axes, si.x - bs.x));
			double surf = const_cast<RegolithBody *>(pb)->surface_radius(dir, false);
			surf = std::isnan(surf) ? pb->r_ref() : surf;
			const DVec3 v_surf = bs.v + regolith::cross(pose_of(pb).omega, si.x - bs.x);
			const double hit_speed = regolith::length(si.v - v_surf);
			// landed where it struck, its attitude in body axes
			Basis stored;
			for (int c = 0; c < 3; ++c) {
				stored.set_column(c, Vector3(real_t(p_vessel->park_basis_[c * 3]), real_t(p_vessel->park_basis_[c * 3 + 1]),
											 real_t(p_vessel->park_basis_[c * 3 + 2])));
			}
			stored = basis_from(axes[0], axes[1], axes[2]).transposed() * stored;
			for (int c = 0; c < 3; ++c) {
				const Vector3 col = stored.get_column(c);
				p_vessel->park_basis_[c * 3 + 0] = col.x;
				p_vessel->park_basis_[c * 3 + 1] = col.y;
				p_vessel->park_basis_[c * 3 + 2] = col.z;
			}
			const DVec3 r = dir * (surf + 1.0);
			p_vessel->park_r_[0] = r.x;
			p_vessel->park_r_[1] = r.y;
			p_vessel->park_r_[2] = r.z;
			p_vessel->landed_ = true;
			p_vessel->park_body_ = pb;
			p_vessel->park_time_ = lo;
			emit_signal("vessel_impact", p_vessel->get_parent(), String(pb->get_name()), hit_speed);
			return;
		}
		const RegolithBody *p = primary_at(s.x, t_next);
		if (p != p_vessel->park_body_) {
			repark(p_vessel, s, t_next, p);
		}
		s_prev = s;
		t = t_next;
	}
}

void RegolithSystem::park(RegolithVessel *p_vessel, Node3D *p_node) {
	State s;
	node_state(p_node, s);
	Basis basis_icrf = render_to_icrf() * p_node->get_global_basis();
	// Landed: pinned to the ground it rests on. Otherwise on a
	// Kepler orbit around the strongest pull.
	RegolithBody *ground = nearest_surface(s.x);
	const Pose &gp = pose_of(ground);
	const DVec3 v_ground = s.v - gp.velocity - regolith::cross(gp.omega, s.x - gp.position);
	p_vessel->landed_ = clearance(ground, s.x) < 20.0 && regolith::length(v_ground) < 2.0;
	const RegolithBody *primary = ground;
	if (p_vessel->landed_) {
		const DVec3 r = from_icrf(gp.axes, s.x - gp.position);
		p_vessel->park_r_[0] = r.x;
		p_vessel->park_r_[1] = r.y;
		p_vessel->park_r_[2] = r.z;
		basis_icrf = basis_from(gp.axes[0], gp.axes[1], gp.axes[2]).transposed() * basis_icrf;
	} else {
		// a Kepler orbit about the sphere of influence it is in
		primary = primary_at(s.x, time_);
		repark(p_vessel, s, time_, primary);
	}
	for (int c = 0; c < 3; ++c) {
		const Vector3 col = basis_icrf.get_column(c);
		p_vessel->park_basis_[c * 3 + 0] = col.x;
		p_vessel->park_basis_[c * 3 + 1] = col.y;
		p_vessel->park_basis_[c * 3 + 2] = col.z;
	}
	p_vessel->park_body_ = primary;
	p_vessel->park_time_ = time_;
	p_vessel->park_check_time_ = time_;
	p_vessel->parked_ = true;
	if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(p_node)) {
		p_vessel->was_frozen_ = rb->is_freeze_enabled();
		rb->set_freeze_mode(RigidBody3D::FREEZE_MODE_KINEMATIC);
		rb->set_freeze_enabled(true);
	}
	p_node->set_physics_process(false);
	p_node->remove_from_group("regolith_anchored");
	p_node->add_to_group("regolith_parked");
}

void RegolithSystem::place_parked(RegolithVessel *p_vessel, Node3D *p_node) {
	const State s = parked_state(p_vessel);
	Vector3 pos, vel;
	from_state(s, pos, vel);
	Basis stored;
	for (int c = 0; c < 3; ++c) {
		stored.set_column(c, Vector3(real_t(p_vessel->park_basis_[c * 3]), real_t(p_vessel->park_basis_[c * 3 + 1]),
									 real_t(p_vessel->park_basis_[c * 3 + 2])));
	}
	if (p_vessel->landed_) {
		const Pose &gp = pose_of(Object::cast_to<RegolithBody>(const_cast<Node *>(p_vessel->park_body_)));
		stored = basis_from(gp.axes[0], gp.axes[1], gp.axes[2]) * stored;
	}
	Transform3D t(render_to_icrf().transposed() * stored, pos);
	t.basis.orthonormalize();
	teleport(p_node, t);
}

void RegolithSystem::unpark(RegolithVessel *p_vessel, Node3D *p_node) {
	place_parked(p_vessel, p_node);
	Vector3 pos, vel;
	from_state(parked_state(p_vessel), pos, vel);
	p_vessel->parked_ = false;
	p_node->remove_from_group("regolith_parked");
	p_node->add_to_group("regolith_anchored");
	p_node->set_physics_process(true);
	if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(p_node)) {
		cb->set_velocity(vel);
	} else if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(p_node)) {
		rb->set_freeze_enabled(p_vessel->was_frozen_);
		rb->set_linear_velocity(vel);
		rb->set_angular_velocity(Vector3());
	}
}

void RegolithSystem::unpark_focus(RegolithVessel *p_vessel, Node3D *p_node) {
	const State s = parked_state(p_vessel);
	unpark(p_vessel, p_node);
	// from rest in the frame: rebuild_frame hands the true
	// velocity to the frame, so Jolt never sees orbital speed
	if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(p_node)) {
		cb->set_velocity(Vector3());
	} else if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(p_node)) {
		rb->set_linear_velocity(Vector3());
	}
	rebuild_frame(dominant_, rotating_, s, p_node);
}

void RegolithSystem::set_warp(double p_rate) {
	const double rate = std::clamp(p_rate, 1.0, WARP_MAX);
	if (!anchored_ || !is_inside_tree()) {
		warp_ = rate;
		return;
	}
	if (rate > 1.0 && warp_ <= 1.0) {
		// every vessel onto rails, the focus too
		TypedArray<Node> vessels = get_tree()->get_nodes_in_group("regolith_vessel");
		for (int i = 0; i < vessels.size(); ++i) {
			RegolithVessel *v = Object::cast_to<RegolithVessel>(vessels[i]);
			Node3D *node = v ? Object::cast_to<Node3D>(v->get_parent()) : nullptr;
			if (node && !v->parked_) {
				park(v, node);
			}
		}
	} else if (rate <= 1.0 && warp_ > 1.0) {
		Node3D *focus = focus_node();
		if (RegolithVessel *v = vessel_of(focus); v && v->parked_) {
			advance_parked(v);
			unpark_focus(v, focus);
		}
	}
	warp_ = rate;
}

void RegolithSystem::update_vessels(Node3D *p_focus, double p_delta) {
	const Vector3 focus_pos = p_focus ? p_focus->get_global_position() : Vector3();
	TypedArray<Node> vessels = get_tree()->get_nodes_in_group("regolith_vessel");
	for (int i = 0; i < vessels.size(); ++i) {
		RegolithVessel *v = Object::cast_to<RegolithVessel>(vessels[i]);
		Node3D *node = v ? Object::cast_to<Node3D>(v->get_parent()) : nullptr;
		if (!node) {
			continue;
		}
		if (v->parked_) {
			advance_parked(v);
		}
		if (warp_ > 1.0) {
			// warping: all on rails, nothing comes off; placed once
			// the clock and the bodies have moved on
			if (!v->parked_) {
				park(v, node);
			}
			continue;
		}
		if (v->parked_) {
			Vector3 pos, vel;
			from_state(parked_state(v), pos, vel);
			const bool rigid = Object::cast_to<RigidBody3D>(node) != nullptr;
			const bool near = (pos - focus_pos).length() < v->get_rails_distance() * 0.9;
			if ((near || v->get_thrusting()) && !(rigid && vel.length() > UNPARK_MAX_SPEED)) {
				unpark(v, node);
			} else {
				place_parked(v, node);
				continue;
			}
		} else if (node != p_focus && !v->get_thrusting() &&
				(node->get_global_position() - focus_pos).length() > v->get_rails_distance()) {
			park(v, node);
			continue;
		}
		RigidBody3D *rb = Object::cast_to<RigidBody3D>(node);
		if (rb && !rb->is_freeze_enabled()) {
			const Vector3 a = get_acceleration(rb->get_global_position(), rb->get_linear_velocity());
			rb->apply_central_force(a * rb->get_mass());
		}
	}
}

void RegolithSystem::_physics_process(double p_delta) {
	if (!anchored_) {
		return;
	}
	Time *clock = Time::get_singleton();
	const uint64_t t0 = clock->get_ticks_usec();
	Node3D *focus = focus_node();
	if (focus) {
		manage_frame(focus);
		// the rotating (surface) frame stays planet-fixed: the
		// leash rebuild carries it, so static nodes stay put
		if (floating_velocity_enabled_ && !rotating_ && !is_parked(focus)) {
			floating_velocity(focus);
		}
	}
	const uint64_t t1 = clock->get_ticks_usec();
	// Forces use the state at time t; then the frame and the
	// bodies advance, and Jolt moves everything by its v dt.
	update_vessels(focus, p_delta);
	const uint64_t t2 = clock->get_ticks_usec();
	// simulated time: warp runs it faster, all vessels on rails
	const double dt = p_delta * warp_;
	time_ += dt;
	frame_.anchor = frame_.anchor + frame_velocity_ * dt;
	update_poses();
	if (warp_ > 1.0) {
		TypedArray<Node> vessels = get_tree()->get_nodes_in_group("regolith_vessel");
		for (int i = 0; i < vessels.size(); ++i) {
			RegolithVessel *v = Object::cast_to<RegolithVessel>(vessels[i]);
			Node3D *node = v ? Object::cast_to<Node3D>(v->get_parent()) : nullptr;
			if (node && v->parked_) {
				advance_parked(v);
				place_parked(v, node);
			}
		}
	}
	const uint64_t t3 = clock->get_ticks_usec();
	stats_["frame_ms"] = double(t1 - t0) * 1e-3;
	stats_["vessels_ms"] = double(t2 - t1) * 1e-3;
	stats_["poses_ms"] = double(t3 - t2) * 1e-3;
}

void RegolithSystem::_process(double p_delta) {
	if (!anchored_) {
		return;
	}
	Camera3D *cam = get_viewport() ? get_viewport()->get_camera_3d() : nullptr;
	if (cam && manage_camera_) {
		update_passes(cam);
	}
	update_sun();
}

void RegolithSystem::setup_passes() {
	Viewport *vp = get_viewport();
	Ref<World3D> world = vp ? vp->get_world_3d() : Ref<World3D>();
	if (world.is_null()) {
		return;
	}
	Ref<Environment> env = world->get_environment();
	if (env.is_null()) {
		env.instantiate();
		world->set_environment(env);
	}
	// The far pass keeps the scene's own background; the near
	// pass draws over it, showing it through canvas layer -100.
	Ref<Environment> far_env = env->duplicate();
	env->set_background(Environment::BG_CANVAS);
	env->set_canvas_max_layer(-50);

	far_view_ = memnew(SubViewport);
	far_view_->set_update_mode(SubViewport::UPDATE_ALWAYS);
	add_child(far_view_);
	far_camera_ = memnew(Camera3D);
	far_camera_->set_environment(far_env);
	far_env_ = far_env;
	near_env_ = env;
	far_view_->add_child(far_camera_);
	far_camera_->set_current(true);

	far_layer_ = memnew(CanvasLayer);
	far_layer_->set_layer(-100);
	add_child(far_layer_);
	TextureRect *rect = memnew(TextureRect);
	rect->set_texture(far_view_->get_texture());
	rect->set_expand_mode(TextureRect::EXPAND_IGNORE_SIZE);
	rect->set_stretch_mode(TextureRect::STRETCH_SCALE);
	rect->set_anchors_preset(Control::PRESET_FULL_RECT);
	far_layer_->add_child(rect);
}

void RegolithSystem::update_passes(Camera3D *p_cam) {
	double h = get_ground_clearance(p_cam->get_global_position());
	if (std::isnan(h)) {
		h = get_altitude(p_cam->get_global_position());
	}
	double near_plane = std::clamp(std::fmax(h, 0.0) * 0.002, 0.05, 100.0);
	// never past the nearest point of what the camera looks at: a
	// chase camera 30 m behind a craft high up clipped it at 100 m.
	// The focus's extent is its meshes' bounding sphere.
	if (Node3D *f = focus_node()) {
		const Vector3 c = f->get_global_position();
		double r = 0.0;
		TypedArray<Node> meshes = f->find_children("*", "VisualInstance3D", true, false);
		for (int i = 0; i < meshes.size(); ++i) {
			const VisualInstance3D *vi = Object::cast_to<VisualInstance3D>(meshes[i]);
			const AABB box = vi->get_global_transform().xform(vi->get_aabb());
			for (int k = 0; k < 8; ++k) {
				r = std::fmax(r, (box.get_endpoint(k) - c).length());
			}
		}
		const double d = (c - p_cam->get_global_position()).length();
		near_plane = std::fmax(std::fmin(near_plane, d - r), 0.05);
		// what surrounds the camera, the game knows (a cabin's walls)
		if (f->has_meta("regolith_near_max")) {
			near_plane = std::fmin(near_plane, double(f->get_meta("regolith_near_max")));
		}
	}
	const double split = near_plane * PASS_RATIO;
	p_cam->set_near(real_t(near_plane));
	// One pass: reverse-Z float depth spans the solar system.
	p_cam->set_far(real_t(split_passes_ ? split : FAR_PLANE));
	if (!far_view_) {
		return;
	}
	const Vector2 size = get_viewport()->get_visible_rect().size;
	if (far_view_->get_size() != Vector2i(size)) {
		far_view_->set_size(Vector2i(size));
	}
	// The far pass shares the scene's haze and ambient.
	if (far_env_.is_valid() && near_env_.is_valid()) {
		far_env_->set_fog_density(near_env_->get_fog_density());
		far_env_->set_fog_sky_affect(near_env_->get_fog_sky_affect());
		far_env_->set_fog_light_color(near_env_->get_fog_light_color());
		far_env_->set_ambient_light_energy(near_env_->get_ambient_light_energy());
	}
	far_camera_->set_global_transform(p_cam->get_global_transform());
	far_camera_->set_fov(p_cam->get_fov());
	far_camera_->set_keep_aspect_mode(p_cam->get_keep_aspect_mode());
	far_camera_->set_near(real_t(split * 0.99));
	far_camera_->set_far(real_t(split * 0.99 * PASS_RATIO));
}

void RegolithSystem::update_sun() {
	DirectionalLight3D *sun = Object::cast_to<DirectionalLight3D>(get_node_or_null(sun_path_));
	if (!sun) {
		return;
	}
	if (sun_energy_ < 0.0) {
		sun_energy_ = sun->get_param(Light3D::PARAM_ENERGY);
	}
	const Vector3 dir = get_sun_direction();
	// A light shines along its -Z: from the sun, toward us.
	const Vector3 helper = std::fabs(dir.y) < 0.99 ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
	sun->set_global_basis(Basis::looking_at(-dir, helper));

	// Eclipses and transits: how much of the solar disc the
	// other bodies leave visible from the camera.
	Camera3D *cam = get_viewport() ? get_viewport()->get_camera_3d() : nullptr;
	sun_visible_ = 1.0;
	if (cam) {
		const DVec3 x = to_state(cam->get_global_position(), Vector3()).x;
		const DVec3 sun_pos = central_heliocentric(time_) * (-AU);
		const DVec3 to_sun = sun_pos - x;
		const double sun_r = std::asin(SUN_RADIUS / regolith::length(to_sun));
		for (const RegolithBody *b : bodies_) {
			const DVec3 to_body = pose_of(b).position - x;
			if (regolith::length(to_body) < b->r_ref() * 1.5) {
				continue; // standing on it: that is night, not eclipse
			}
			const double body_r = std::asin(std::fmin(1.0, b->r_ref() / regolith::length(to_body)));
			const double sep = std::acos(std::clamp(regolith::dot(regolith::normalize(to_sun), regolith::normalize(to_body)), -1.0, 1.0));
			const double covered = std::clamp((body_r + sun_r - sep) / (2.0 * sun_r), 0.0, 1.0);
			const double area = std::fmin(1.0, (body_r * body_r) / (sun_r * sun_r));
			sun_visible_ = std::fmin(sun_visible_, 1.0 - covered * area);
		}
	}
	sun->set_param(Light3D::PARAM_ENERGY, real_t(sun_energy_ * sun_visible_ * get_sun_scale()));
}

DVec3 RegolithSystem::camera_heliocentric() const {
	DVec3 x;
	Camera3D *cam = is_inside_tree() && get_viewport() ? get_viewport()->get_camera_3d() : nullptr;
	if (cam && anchored_) {
		x = to_state(cam->get_global_position(), Vector3()).x;
	}
	return central_heliocentric(time_) * AU + x;
}

double RegolithSystem::get_sun_scale() const {
	// the light's energy is the central body's: inverse square
	const double d0 = regolith::length(central_heliocentric(time_) * AU);
	const double d = std::max(regolith::length(camera_heliocentric()), 1e9);
	return (d0 * d0) / (d * d);
}

Vector3 RegolithSystem::get_sun_direction() const {
	if (!anchored_) {
		return Vector3();
	}
	// from the camera: on another planet the sun stands elsewhere
	const DVec3 sun = regolith::normalize(camera_heliocentric() * -1.0);
	return regolith::to_v(frame_.to_local_vec(from_icrf(ref_axes(), sun)));
}

DVec3 RegolithSystem::central_heliocentric(double p_t) const {
	return regolith::heliocentric(central_ ? central_->orbit_elements() : regolith::MARS_ORBIT, p_t);
}

namespace {

// the sun from a body at an ICRF place, in its fixed axes
DVec3 sun_in_axes(const DVec3 &p_helio, const DVec3 &p_position, const DVec3 p_axes[3], double p_au) {
	const DVec3 s = regolith::normalize(p_helio * -p_au - p_position);
	return DVec3(regolith::dot(s, p_axes[0]), regolith::dot(s, p_axes[1]), regolith::dot(s, p_axes[2]));
}

} // namespace

DVec3 RegolithSystem::sun_in_body(const RegolithBody *p_body) const {
	const Pose &p = pose_of(p_body);
	return sun_in_axes(central_heliocentric(time_), p.position, p.axes, AU);
}

DVec3 RegolithSystem::sun_in_body_at(const RegolithBody *p_body, double p_t) const {
	DVec3 position, velocity, axes[3];
	p_body->pose(p_t, position, velocity, axes);
	return sun_in_axes(central_heliocentric(p_t), position, axes, AU);
}

double RegolithSystem::get_local_solar_time(const Vector3 &p_pos) const {
	if (!anchored_) {
		return 0.0;
	}
	const Vector2 ll = get_latlon(p_pos);
	return regolith::local_solar_time(sun_in_body(dominant_), double(ll.y));
}

Vector3 RegolithSystem::get_acceleration(const Vector3 &p_pos, const Vector3 &p_vel) const {
	if (!anchored_) {
		return Vector3(0, -3.71, 0);
	}
	const DVec3 a = accel_icrf(to_state(p_pos, p_vel), true);
	return regolith::to_v(frame_.to_local_vec(from_icrf(ref_axes(), a)));
}

Vector3 RegolithSystem::get_gravity(const Vector3 &p_pos) const {
	if (!anchored_) {
		return Vector3(0, -3.71, 0);
	}
	const DVec3 a = accel_icrf(to_state(p_pos, Vector3()), false);
	return regolith::to_v(frame_.to_local_vec(from_icrf(ref_axes(), a)));
}

Vector3 RegolithSystem::get_up(const Vector3 &p_pos) const {
	const Vector3 g = get_gravity(p_pos);
	if (g.length() > 1e-7) {
		return -g.normalized();
	}
	return regolith::to_v(frame_.to_local_vec(regolith::normalize(regolith::to_body_point(frame_, regolith::to_d(p_pos)))));
}

Vector3 RegolithSystem::get_frame_velocity() const {
	return regolith::to_v(frame_.to_local_vec(frame_velocity_));
}

Vector2 RegolithSystem::get_latlon(const Vector3 &p_pos) const {
	if (!anchored_) {
		return Vector2();
	}
	const DVec3 x = body_point(dominant_, p_pos);
	return Vector2(real_t(std::atan2(x.z, std::hypot(x.x, x.y)) / DEG), real_t(std::atan2(x.y, x.x) / DEG));
}

double RegolithSystem::get_radius(const Vector3 &p_pos) const {
	if (!anchored_) {
		return 0.0;
	}
	return regolith::length(body_point(dominant_, p_pos));
}

double RegolithSystem::get_altitude(const Vector3 &p_pos) const {
	if (!dominant_ || !anchored_) {
		return 0.0;
	}
	const DVec3 x = body_point(dominant_, p_pos);
	return regolith::length(x) - dominant_->datum_radius(regolith::normalize(x));
}

double RegolithSystem::get_ground_clearance(const Vector3 &p_pos) const {
	if (!dominant_ || !anchored_) {
		return std::nan("");
	}
	const DVec3 x = body_point(dominant_, p_pos);
	return regolith::length(x) - dominant_->surface_radius(regolith::normalize(x), false);
}

String RegolithSystem::get_dominant_name() const {
	return dominant_ ? String(dominant_->get_name()) : String();
}

Array RegolithSystem::get_bodies_info(const Vector3 &p_pos) const {
	Array out;
	if (!anchored_) {
		return out;
	}
	const DVec3 x = to_state(p_pos, Vector3()).x;
	for (const RegolithBody *b : bodies_) {
		Dictionary d;
		const double r = regolith::length(x - pose_of(b).position);
		d["name"] = String(b->get_name());
		d["distance"] = r;
		d["altitude"] = r - b->r_ref();
		out.push_back(d);
	}
	return out;
}

Vector3 RegolithSystem::get_body_position(const String &p_name) const {
	for (const RegolithBody *b : bodies_) {
		if (String(b->get_name()) == p_name) {
			Vector3 pos, vel;
			from_state({ pose_of(b).position, pose_of(b).velocity }, pos, vel);
			return pos;
		}
	}
	return Vector3();
}

PackedFloat64Array RegolithSystem::get_inertial_state(Node *p_node) const {
	PackedFloat64Array out;
	State s;
	if (!anchored_ || !node_state(Object::cast_to<Node3D>(p_node), s)) {
		return out;
	}
	for (double v : { s.x.x, s.x.y, s.x.z, s.v.x, s.v.y, s.v.z }) {
		out.push_back(v);
	}
	return out;
}

Dictionary RegolithSystem::get_orbit_info(Node *p_node) const {
	Dictionary d;
	State s;
	if (!anchored_ || !node_state(Object::cast_to<Node3D>(p_node), s)) {
		return d;
	}
	// about the sphere of influence it is in: a body, or the sun
	// (inclination then against the ecliptic)
	const RegolithBody *primary = primary_at(s.x, time_);
	const DVec3 origin = primary ? pose_of(primary).position : sun_pos_;
	const DVec3 origin_v = primary ? pose_of(primary).velocity : sun_vel_;
	const DVec3 pole = primary ? pose_of(primary).axes[2] : ECLIPTIC_POLE;
	const double radius = primary ? primary->r_ref() : SUN_RADIUS;
	const DVec3 r = s.x - origin, v = s.v - origin_v;
	const regolith::OrbitShape o = regolith::orbit_shape(r, v, gm_of(primary), pole);
	d["body"] = primary ? String(primary->get_name()) : String("Sun");
	d["altitude"] = regolith::length(r) - radius;
	d["periapsis"] = o.periapsis - radius;
	d["apoapsis"] = o.apoapsis - radius;
	d["semi_major_axis"] = o.a;
	d["eccentricity"] = o.e;
	d["inclination"] = o.inclination / DEG;
	d["period"] = o.period;
	d["orbital_speed"] = regolith::length(v);
	d["surface_speed"] = primary ? regolith::length(v - regolith::cross(pose_of(primary).omega, r)) : regolith::length(v);
	d["soi_radius"] = soi_at(primary, time_);
	if (RegolithVessel *ves = vessel_of(Object::cast_to<Node>(p_node))) {
		d["on_rails"] = ves->parked_;
		if (ves->parked_) {
			const Node *pb = ves->park_body_;
			d["rails_body"] = pb ? String(pb->get_name()) : String("Sun");
		}
	}
	return d;
}

void RegolithSystem::set_inertial_state(Node *p_node, const PackedFloat64Array &p_state) {
	Node3D *node = Object::cast_to<Node3D>(p_node);
	if (!node || !anchored_ || p_state.size() < 6) {
		ERR_PRINT("set_inertial_state: unknown node, or not 6 numbers");
		return;
	}
	for (int k = 0; k < 6; ++k) {
		if (!std::isfinite(p_state[k])) {
			ERR_PRINT("set_inertial_state: the state is not finite");
			return;
		}
	}
	if (RegolithVessel *v = vessel_of(node); v && v->parked_) {
		unpark(v, node);
	}
	const State s{ DVec3(p_state[0], p_state[1], p_state[2]), DVec3(p_state[3], p_state[4], p_state[5]) };
	RegolithBody *body = nearest_surface(s.x);
	const double alt = regolith::length(s.x - pose_of(body).position) - body->r_ref();
	if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(node)) {
		rb->set_linear_velocity(Vector3());
		rb->set_angular_velocity(Vector3());
	} else if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(node)) {
		cb->set_velocity(Vector3());
	}
	rebuild_frame(body, alt < body->get_rotating_altitude(), s, node);
	last_focus_ = node == focus_node() ? node : last_focus_;
}

PackedFloat64Array RegolithSystem::get_body_state(const String &p_name) const {
	PackedFloat64Array out;
	if (!anchored_) {
		return out;
	}
	State s;
	if (p_name == String("Sun")) {
		s = { sun_pos_, sun_vel_ };
	} else if (const RegolithBody *b = find_body(p_name)) {
		s = { pose_of(b).position, pose_of(b).velocity };
	} else {
		return out;
	}
	for (double v : { s.x.x, s.x.y, s.x.z, s.v.x, s.v.y, s.v.z }) {
		out.push_back(v);
	}
	return out;
}

PackedFloat64Array RegolithSystem::get_body_state_at(const String &p_name, double p_time) const {
	PackedFloat64Array out;
	const RegolithBody *b = p_name == String("Sun") ? nullptr : find_body(p_name);
	if (!anchored_ || (!b && p_name != String("Sun"))) {
		return out;
	}
	const State s = body_state_at(b, p_time);
	for (double v : { s.x.x, s.x.y, s.x.z, s.v.x, s.v.y, s.v.z }) {
		out.push_back(v);
	}
	return out;
}

double RegolithSystem::get_soi_radius(const String &p_name) const {
	if (p_name == String("Sun")) {
		return INFINITY;
	}
	const RegolithBody *b = find_body(p_name);
	return b && anchored_ ? soi_at(b, time_) : 0.0;
}

void RegolithSystem::put_in_orbit(Node *p_node, const String &p_body, double p_altitude, double p_inclination_deg) {
	Node3D *node = Object::cast_to<Node3D>(p_node);
	RegolithBody *body = nullptr;
	for (RegolithBody *b : bodies_) {
		if (String(b->get_name()) == p_body) {
			body = b;
		}
	}
	if (!node || !body || !anchored_) {
		ERR_PRINT("put_in_orbit: unknown node or body");
		return;
	}
	if (RegolithVessel *v = vessel_of(node); v && v->parked_) {
		unpark(v, node);
	}
	const Pose &bp = pose_of(body);
	const double r = body->r_ref() + p_altitude;
	const DVec3 n = bp.axes[0], k = bp.axes[2];
	const double inc = p_inclination_deg * DEG;
	const DVec3 h = regolith::normalize(k * std::cos(inc) - regolith::cross(k, n) * std::sin(inc));
	const State s{ bp.position + n * r,
		bp.velocity + regolith::normalize(regolith::cross(h, n)) * std::sqrt(body->get_gm() / r) };
	const bool rotating = p_altitude < body->get_rotating_altitude();
	if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(node)) {
		rb->set_linear_velocity(Vector3());
		rb->set_angular_velocity(Vector3());
	} else if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(node)) {
		cb->set_velocity(Vector3());
	}
	rebuild_frame(body, rotating, s, node);
	last_focus_ = node == focus_node() ? node : last_focus_;
}

void RegolithSystem::put_near(Node *p_node, const String &p_body, double p_lat_deg, double p_lon_deg, double p_height) {
	Node3D *node = Object::cast_to<Node3D>(p_node);
	RegolithBody *body = nullptr;
	for (RegolithBody *b : bodies_) {
		if (String(b->get_name()) == p_body) {
			body = b;
		}
	}
	if (!node || !body || !anchored_) {
		ERR_PRINT("put_near: unknown node or body");
		return;
	}
	if (RegolithVessel *v = vessel_of(node); v && v->parked_) {
		unpark(v, node);
	}
	const double lat = p_lat_deg * DEG, lon = p_lon_deg * DEG;
	const DVec3 dir(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
	const double r = body->surface_radius(dir, true) + p_height;
	const Pose &bp = pose_of(body);
	const DVec3 x = bp.position + to_icrf(bp.axes, dir * r);
	const State s{ x, bp.velocity + regolith::cross(bp.omega, x - bp.position) };
	if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(node)) {
		rb->set_linear_velocity(Vector3());
		rb->set_angular_velocity(Vector3());
	} else if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(node)) {
		cb->set_velocity(Vector3());
	}
	rebuild_frame(body, true, s, node);
	last_focus_ = node == focus_node() ? node : last_focus_;
	// Stand it upright on the local vertical.
	const Vector3 up = get_up(node->get_global_position());
	const Vector3 side = std::fabs(up.x) < 0.9 ? Vector3(1, 0, 0) : Vector3(0, 0, 1);
	const Vector3 x_axis = side.cross(up).normalized().cross(up).normalized();
	teleport(node, Transform3D(Basis(x_axis, up, x_axis.cross(up)), node->get_global_position()));
}

Vector3 RegolithSystem::get_surface_point(const String &p_body, double p_lat_deg, double p_lon_deg, double p_height) {
	for (RegolithBody *b : bodies_) {
		if (String(b->get_name()) != p_body) {
			continue;
		}
		const double lat = p_lat_deg * DEG, lon = p_lon_deg * DEG;
		const DVec3 dir(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
		const Pose &bp = pose_of(b);
		// cached heights only: called per frame, it must never
		// block on a tile read (the finest cached level answers)
		double rs = b->surface_radius(dir, false);
		if (std::isnan(rs)) {
			rs = b->r_ref();
		}
		const DVec3 x = bp.position + to_icrf(bp.axes, dir * (rs + p_height));
		Vector3 pos, vel;
		from_state({ x, bp.velocity }, pos, vel);
		return pos;
	}
	return Vector3();
}

double RegolithSystem::get_solar_longitude_today() const {
	const double now = regolith::j2000_seconds_from_unix(Time::get_singleton()->get_unix_time_from_system());
	if (!central_) {
		return regolith::mars_solar_longitude(now);
	}
	return regolith::solar_longitude(central_->orbit_elements(), central_->rotation(), now);
}

Vector3 RegolithSystem::get_body_point(const String &p_body, double p_lat_deg, double p_lon_deg, double p_radius) {
	for (RegolithBody *b : bodies_) {
		if (String(b->get_name()) != p_body) {
			continue;
		}
		const double lat = p_lat_deg * DEG, lon = p_lon_deg * DEG;
		const DVec3 dir(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
		const Pose &bp = pose_of(b);
		Vector3 pos, vel;
		from_state({ bp.position + to_icrf(bp.axes, dir * p_radius), bp.velocity }, pos, vel);
		return pos;
	}
	return Vector3();
}

PackedVector3Array RegolithSystem::get_ground_points(const String &p_body, const PackedFloat64Array &p_latlon) {
	PackedVector3Array out;
	for (RegolithBody *b : bodies_) {
		if (String(b->get_name()) != p_body) {
			continue;
		}
		const Pose &bp = pose_of(b);
		const int64_t n = p_latlon.size() / 2;
		out.resize(n);
		const double *ll = p_latlon.ptr();
		Vector3 *w = out.ptrw();
		for (int64_t i = 0; i < n; ++i) {
			const double lat = ll[i * 2] * DEG, lon = ll[i * 2 + 1] * DEG;
			const DVec3 dir(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
			// cached heights only, as get_surface_point; NaN marks
			// ground not streamed yet, for the caller to retry
			const double rs = b->surface_radius(dir, false);
			if (std::isnan(rs)) {
				w[i] = Vector3(NAN, NAN, NAN);
				continue;
			}
			Vector3 vel;
			from_state({ bp.position + to_icrf(bp.axes, dir * rs), bp.velocity }, w[i], vel);
		}
		break;
	}
	return out;
}

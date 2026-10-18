#include "regolith_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <random>

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/character_body3d.hpp>
#include <godot_cpp/classes/directional_light3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/core/class_db.hpp>

#include "convert.hpp"
#include "regolith/cube.hpp"
#include "regolith/ephemeris.hpp"
#include "regolith_body.hpp"

using namespace godot;
using regolith::DVec3;

namespace {

constexpr double DEG = std::numbers::pi / 180.0;

} // namespace

RegolithSystem::RegolithSystem() {
	set_process_priority(-100);
	set_physics_process_priority(-100);
}

void RegolithSystem::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_version"), &RegolithSystem::get_version);
	ClassDB::bind_method(D_METHOD("self_test"), &RegolithSystem::self_test);
	ClassDB::bind_method(D_METHOD("get_gravity", "position"), &RegolithSystem::get_gravity);
	ClassDB::bind_method(D_METHOD("get_up", "position"), &RegolithSystem::get_up);
	ClassDB::bind_method(D_METHOD("get_latlon", "position"), &RegolithSystem::get_latlon);
	ClassDB::bind_method(D_METHOD("get_altitude", "position"), &RegolithSystem::get_altitude);
	ClassDB::bind_method(D_METHOD("get_ground_clearance", "position"), &RegolithSystem::get_ground_clearance);
	ClassDB::bind_method(D_METHOD("get_radius", "position"), &RegolithSystem::get_radius);
	ClassDB::bind_method(D_METHOD("get_rebase_count"), &RegolithSystem::get_rebase_count);

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
	ClassDB::bind_method(D_METHOD("get_time"), &RegolithSystem::get_time);
	ClassDB::bind_method(D_METHOD("get_local_solar_time", "position"), &RegolithSystem::get_local_solar_time);
	ClassDB::bind_method(D_METHOD("get_sun_direction"), &RegolithSystem::get_sun_direction);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "body"), "set_body_path", "get_body_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "focus"), "set_focus_path", "get_focus_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "sun"), "set_sun_path", "get_sun_path");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "start_latitude"), "set_start_latitude", "get_start_latitude");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "start_longitude"), "set_start_longitude", "get_start_longitude");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "start_local_time", PROPERTY_HINT_RANGE, "0,24,0.1"), "set_start_local_time",
			"get_start_local_time");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "manage_camera"), "set_manage_camera", "get_manage_camera");
	ADD_SIGNAL(MethodInfo("frame_shifted", PropertyInfo(Variant::TRANSFORM3D, "transform")));
}

String RegolithSystem::get_version() const {
	return "0.2.0-m2";
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
	body_ = Object::cast_to<RegolithBody>(get_node_or_null(body_path_));
	if (!body_) {
		ERR_PRINT("RegolithSystem: body path does not point at a RegolithBody");
		return;
	}
	anchor_at_start();
}

void RegolithSystem::anchor_at_start() {
	if (!body_->ensure_open()) {
		return;
	}
	const double lat = start_lat_ * DEG;
	const double lon = start_lon_ * DEG;
	const DVec3 dir(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
	frame_ = regolith::RenderFrame::at(dir * body_->surface_radius(dir, true));
	anchored_ = true;

	// Real clock: today's date, moved forward to the next
	// start_local_time at the start longitude.
	time_ = regolith::j2000_seconds_from_unix(Time::get_singleton()->get_unix_time_from_system());
	for (int k = 0; k < 4; ++k) {
		double dh = start_local_time_ - regolith::mars_local_solar_time(time_, start_lon_);
		dh = k == 0 ? std::fmod(dh + 24.0, 24.0) : std::remainder(dh, 24.0);
		time_ += dh / 24.0 * regolith::MARS_SOL;
	}
}

Vector3 RegolithSystem::get_sun_direction() const {
	return regolith::to_v(frame_.to_local_vec(regolith::mars_sun_direction(time_)));
}

double RegolithSystem::get_local_solar_time(const Vector3 &p_pos) const {
	const Vector2 ll = get_latlon(p_pos);
	return regolith::mars_local_solar_time(time_, double(ll.y));
}

void RegolithSystem::rebase(const DVec3 &p_anchor) {
	const regolith::RenderFrame old = frame_;
	frame_ = regolith::RenderFrame::at(p_anchor);
	// Old render coords to new: p' = R_new (R_old^T p + A_old - A_new).
	Transform3D t;
	t.basis = regolith::basis_between(frame_, old);
	t.origin = regolith::to_v(frame_.to_local_point(old.anchor));
	TypedArray<Node> nodes = get_tree()->get_nodes_in_group("regolith_anchored");
	for (int i = 0; i < nodes.size(); ++i) {
		Node3D *n = Object::cast_to<Node3D>(nodes[i]);
		if (!n) {
			continue;
		}
		n->set_global_transform(t * n->get_global_transform());
		if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(n)) {
			cb->set_velocity(t.basis.xform(cb->get_velocity()));
		} else if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(n)) {
			rb->set_linear_velocity(t.basis.xform(rb->get_linear_velocity()));
			rb->set_angular_velocity(t.basis.xform(rb->get_angular_velocity()));
		}
	}
	++rebases_;
	emit_signal("frame_shifted", t);
}

void RegolithSystem::_physics_process(double p_delta) {
	if (!anchored_) {
		return;
	}
	Node3D *focus = Object::cast_to<Node3D>(get_node_or_null(focus_path_));
	if (!focus) {
		if (Viewport *vp = get_viewport()) {
			focus = vp->get_camera_3d();
		}
	}
	if (!focus) {
		return;
	}
	const DVec3 local = regolith::to_d(focus->get_global_position());
	const DVec3 world = regolith::to_body_point(frame_, local);
	const double altitude = regolith::length(world) - body_->r_ref();
	const double leash = std::fmax(2000.0, 0.05 * std::fabs(altitude));
	if (regolith::length(local) > leash) {
		rebase(world);
	}
}

void RegolithSystem::_process(double p_delta) {
	if (!anchored_) {
		return;
	}
	time_ += p_delta;
	Camera3D *cam = get_viewport() ? get_viewport()->get_camera_3d() : nullptr;
	if (cam && manage_camera_) {
		// float32 projection matrices lose the far plane past a
		// far/near ratio of ~1e7, so both planes follow height:
		// far reaches the horizon and the relief behind it.
		const double r = body_->r_ref();
		double h = get_ground_clearance(cam->get_global_position());
		if (std::isnan(h)) {
			h = get_altitude(cam->get_global_position());
		}
		h = std::fmax(h, 0.0);
		auto horizon = [r](double p_h) { return std::sqrt(2.0 * r * p_h + p_h * p_h); };
		cam->set_far(real_t(horizon(h) + horizon(30000.0) + 1000.0));
		cam->set_near(real_t(std::clamp(h * 0.002, 0.05, 100.0)));
	}
	DirectionalLight3D *sun = Object::cast_to<DirectionalLight3D>(get_node_or_null(sun_path_));
	if (sun) {
		const Vector3 dir = get_sun_direction();
		// A light shines along its -Z: from the sun, toward us.
		const Vector3 helper = std::fabs(dir.y) < 0.99 ? Vector3(0, 1, 0) : Vector3(1, 0, 0);
		sun->set_global_basis(Basis::looking_at(-dir, helper));
	}
}

Vector3 RegolithSystem::get_gravity(const Vector3 &p_pos) const {
	if (!body_) {
		return Vector3(0, -3.71, 0);
	}
	const DVec3 x = regolith::to_body_point(frame_, regolith::to_d(p_pos));
	const double r = regolith::length(x);
	const DVec3 g = x * (-body_->get_gm() / (r * r * r));
	return regolith::to_v(frame_.to_local_vec(g));
}

Vector3 RegolithSystem::get_up(const Vector3 &p_pos) const {
	const DVec3 x = regolith::to_body_point(frame_, regolith::to_d(p_pos));
	return regolith::to_v(frame_.to_local_vec(regolith::normalize(x)));
}

Vector2 RegolithSystem::get_latlon(const Vector3 &p_pos) const {
	const DVec3 x = regolith::to_body_point(frame_, regolith::to_d(p_pos));
	return Vector2(real_t(std::atan2(x.z, std::hypot(x.x, x.y)) / DEG), real_t(std::atan2(x.y, x.x) / DEG));
}

double RegolithSystem::get_radius(const Vector3 &p_pos) const {
	return regolith::length(regolith::to_body_point(frame_, regolith::to_d(p_pos)));
}

double RegolithSystem::get_altitude(const Vector3 &p_pos) const {
	if (!body_) {
		return 0.0;
	}
	const DVec3 x = regolith::to_body_point(frame_, regolith::to_d(p_pos));
	return regolith::length(x) - body_->datum_radius(regolith::normalize(x));
}

double RegolithSystem::get_ground_clearance(const Vector3 &p_pos) const {
	if (!body_) {
		return std::nan("");
	}
	const DVec3 x = regolith::to_body_point(frame_, regolith::to_d(p_pos));
	const DVec3 d = regolith::normalize(x);
	return regolith::length(x) - body_->surface_radius(d, false);
}

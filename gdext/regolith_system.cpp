#include "regolith_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <random>

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/canvas_layer.hpp>
#include <godot_cpp/classes/character_body3d.hpp>
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
#include "regolith_body.hpp"

using namespace godot;
using regolith::DVec3;

namespace {

constexpr double DEG = std::numbers::pi / 180.0;
constexpr double AU = 1.495978707e11;
constexpr double SUN_RADIUS = 6.957e8;
// Each camera pass spans this far/near ratio; float32
// projections lose the far plane well before 1e8.
constexpr double PASS_RATIO = 1e6;

DVec3 to_icrf(const DVec3 p_axes[3], const DVec3 &p_v) {
	return p_axes[0] * p_v.x + p_axes[1] * p_v.y + p_axes[2] * p_v.z;
}

DVec3 from_icrf(const DVec3 p_axes[3], const DVec3 &p_v) {
	return { regolith::dot(p_axes[0], p_v), regolith::dot(p_axes[1], p_v), regolith::dot(p_axes[2], p_v) };
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
	ClassDB::bind_method(D_METHOD("get_up", "position"), &RegolithSystem::get_up);
	ClassDB::bind_method(D_METHOD("get_latlon", "position"), &RegolithSystem::get_latlon);
	ClassDB::bind_method(D_METHOD("get_altitude", "position"), &RegolithSystem::get_altitude);
	ClassDB::bind_method(D_METHOD("get_ground_clearance", "position"), &RegolithSystem::get_ground_clearance);
	ClassDB::bind_method(D_METHOD("get_radius", "position"), &RegolithSystem::get_radius);
	ClassDB::bind_method(D_METHOD("get_rebase_count"), &RegolithSystem::get_rebase_count);
	ClassDB::bind_method(D_METHOD("get_time"), &RegolithSystem::get_time);
	ClassDB::bind_method(D_METHOD("get_local_solar_time", "position"), &RegolithSystem::get_local_solar_time);
	ClassDB::bind_method(D_METHOD("get_sun_direction"), &RegolithSystem::get_sun_direction);
	ClassDB::bind_method(D_METHOD("get_sun_visible"), &RegolithSystem::get_sun_visible);
	ClassDB::bind_method(D_METHOD("get_dominant_name"), &RegolithSystem::get_dominant_name);
	ClassDB::bind_method(D_METHOD("get_bodies_info", "position"), &RegolithSystem::get_bodies_info);
	ClassDB::bind_method(D_METHOD("get_body_position", "name"), &RegolithSystem::get_body_position);

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
	ADD_SIGNAL(MethodInfo("frame_shifted", PropertyInfo(Variant::TRANSFORM3D, "transform")));
}

String RegolithSystem::get_version() const {
	return "0.3.0-m3";
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
			}
		}
	}
	anchor_at_start();
	if (manage_camera_) {
		setup_passes();
	}
}

void RegolithSystem::anchor_at_start() {
	// Real clock: today's date, moved forward to the next
	// start_local_time at the start longitude.
	time_ = regolith::j2000_seconds_from_unix(Time::get_singleton()->get_unix_time_from_system());
	for (int k = 0; k < 4; ++k) {
		double dh = start_local_time_ - regolith::mars_local_solar_time(time_, start_lon_);
		dh = k == 0 ? std::fmod(dh + 24.0, 24.0) : std::remainder(dh, 24.0);
		time_ += dh / 24.0 * regolith::MARS_SOL;
	}
	update_poses();
	const double lat = start_lat_ * DEG;
	const double lon = start_lon_ * DEG;
	const DVec3 dir(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
	frame_ = regolith::RenderFrame::at(dir * dominant_->surface_radius(dir, true));
	anchored_ = true;
}

void RegolithSystem::update_poses() {
	for (RegolithBody *b : bodies_) {
		Pose &p = poses_[b];
		b->pose(time_, p.position, p.axes);
	}
	++pose_counter_;
}

const RegolithSystem::Pose &RegolithSystem::pose_of(const RegolithBody *p_body) const {
	return poses_.at(p_body);
}

DVec3 RegolithSystem::to_inertial(const Vector3 &p_render) const {
	const Pose &d = pose_of(dominant_);
	return d.position + to_icrf(d.axes, regolith::to_body_point(frame_, regolith::to_d(p_render)));
}

DVec3 RegolithSystem::inertial_vec_to_render(const DVec3 &p_v) const {
	return frame_.to_local_vec(from_icrf(pose_of(dominant_).axes, p_v));
}

regolith::RenderFrame RegolithSystem::frame_for(const RegolithBody *p_body) const {
	if (p_body == dominant_ || !anchored_) {
		return frame_;
	}
	const Pose &d = pose_of(dominant_);
	const Pose &b = pose_of(p_body);
	regolith::RenderFrame g;
	g.anchor = from_icrf(b.axes, d.position + to_icrf(d.axes, frame_.anchor) - b.position);
	g.east = from_icrf(b.axes, to_icrf(d.axes, frame_.east));
	g.up = from_icrf(b.axes, to_icrf(d.axes, frame_.up));
	g.south = from_icrf(b.axes, to_icrf(d.axes, frame_.south));
	return g;
}

uint64_t RegolithSystem::frame_version(const RegolithBody *p_body) const {
	// The dominant body is fixed in its own frame; the others
	// move through it every tick.
	return p_body == dominant_ ? uint64_t(rebases_) : pose_counter_ + (uint64_t(1) << 40);
}

void RegolithSystem::re_anchor(RegolithBody *p_body, const DVec3 &p_inertial) {
	const Pose &np = pose_of(p_body);
	const regolith::RenderFrame next = regolith::RenderFrame::at(from_icrf(np.axes, p_inertial - np.position));
	auto point_to_next = [&](const Vector3 &p_old) {
		return regolith::to_v(next.to_local_point(from_icrf(np.axes, to_inertial(p_old) - np.position)));
	};
	const Vector3 origin = point_to_next(Vector3());
	Transform3D t;
	t.origin = origin;
	t.basis.set_column(0, point_to_next(Vector3(1, 0, 0)) - origin);
	t.basis.set_column(1, point_to_next(Vector3(0, 1, 0)) - origin);
	t.basis.set_column(2, point_to_next(Vector3(0, 0, 1)) - origin);
	t.basis.orthonormalize();

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
	dominant_ = p_body;
	frame_ = next;
	++rebases_;
	emit_signal("frame_shifted", t);
}

void RegolithSystem::_physics_process(double p_delta) {
	if (!anchored_) {
		return;
	}
	time_ += p_delta;
	update_poses();
	Node3D *focus = Object::cast_to<Node3D>(get_node_or_null(focus_path_));
	if (!focus && get_viewport()) {
		focus = get_viewport()->get_camera_3d();
	}
	if (!focus) {
		return;
	}
	const DVec3 x = to_inertial(focus->get_global_position());

	// The frame belongs to the nearest surface, in body radii,
	// with some hysteresis so it never flips back and forth.
	RegolithBody *best = dominant_;
	auto closeness = [&](const RegolithBody *p_b) {
		return (regolith::length(x - pose_of(p_b).position) - p_b->r_ref()) / p_b->r_ref();
	};
	for (RegolithBody *b : bodies_) {
		if (closeness(b) < closeness(best)) {
			best = b;
		}
	}
	if (best != dominant_ && closeness(best) < 0.8 * closeness(dominant_)) {
		re_anchor(best, x);
		return;
	}
	const DVec3 local = regolith::to_d(focus->get_global_position());
	const double r = dominant_->r_ref();
	const double altitude = regolith::length(x - pose_of(dominant_).position) - r;
	const double leash = std::fmax(std::fmin(2000.0, 0.02 * r), 0.05 * std::fabs(altitude));
	if (regolith::length(local) > leash) {
		re_anchor(dominant_, x);
	}
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
	const double near_plane = std::clamp(std::fmax(h, 0.0) * 0.002, 0.05, 100.0);
	const double split = near_plane * PASS_RATIO;
	p_cam->set_near(real_t(near_plane));
	p_cam->set_far(real_t(split));
	if (!far_view_) {
		return;
	}
	const Vector2 size = get_viewport()->get_visible_rect().size;
	if (far_view_->get_size() != Vector2i(size)) {
		far_view_->set_size(Vector2i(size));
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
		const DVec3 x = to_inertial(cam->get_global_position());
		const DVec3 sun_pos = regolith::mars_heliocentric(time_) * (-AU);
		const DVec3 to_sun = sun_pos - x;
		const double sun_r = std::asin(SUN_RADIUS / regolith::length(to_sun));
		for (const RegolithBody *b : bodies_) {
			if (b == dominant_) {
				continue;
			}
			const DVec3 to_body = pose_of(b).position - x;
			const double body_r = std::asin(std::fmin(1.0, b->r_ref() / regolith::length(to_body)));
			const double sep = std::acos(std::clamp(regolith::dot(regolith::normalize(to_sun), regolith::normalize(to_body)), -1.0, 1.0));
			const double covered = std::clamp((body_r + sun_r - sep) / (2.0 * sun_r), 0.0, 1.0);
			const double area = std::fmin(1.0, (body_r * body_r) / (sun_r * sun_r));
			sun_visible_ = std::fmin(sun_visible_, 1.0 - covered * area);
		}
	}
	sun->set_param(Light3D::PARAM_ENERGY, real_t(sun_energy_ * sun_visible_));
}

Vector3 RegolithSystem::get_sun_direction() const {
	const DVec3 sun = regolith::normalize(regolith::mars_heliocentric(time_) * -1.0);
	return regolith::to_v(inertial_vec_to_render(sun));
}

double RegolithSystem::get_local_solar_time(const Vector3 &p_pos) const {
	const Vector2 ll = get_latlon(p_pos);
	return regolith::mars_local_solar_time(time_, double(ll.y));
}

Vector3 RegolithSystem::get_gravity(const Vector3 &p_pos) const {
	if (!anchored_) {
		return Vector3(0, -3.71, 0);
	}
	// Every body pulls; the frame's own centre falls toward the
	// others too, so that part is taken back out (tides remain).
	const DVec3 x = to_inertial(p_pos);
	const DVec3 centre = pose_of(dominant_).position;
	DVec3 g(0, 0, 0);
	for (const RegolithBody *b : bodies_) {
		const DVec3 d = x - pose_of(b).position;
		const double r = regolith::length(d);
		g = g - d * (b->get_gm() / (r * r * r));
		if (b != dominant_) {
			const DVec3 dc = centre - pose_of(b).position;
			const double rc = regolith::length(dc);
			g = g + dc * (b->get_gm() / (rc * rc * rc));
		}
	}
	return regolith::to_v(inertial_vec_to_render(g));
}

Vector3 RegolithSystem::get_up(const Vector3 &p_pos) const {
	const Vector3 g = get_gravity(p_pos);
	if (g.length() > 1e-7) {
		return -g.normalized();
	}
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
	if (!dominant_) {
		return 0.0;
	}
	const DVec3 x = regolith::to_body_point(frame_, regolith::to_d(p_pos));
	return regolith::length(x) - dominant_->datum_radius(regolith::normalize(x));
}

double RegolithSystem::get_ground_clearance(const Vector3 &p_pos) const {
	if (!dominant_) {
		return std::nan("");
	}
	const DVec3 x = regolith::to_body_point(frame_, regolith::to_d(p_pos));
	return regolith::length(x) - dominant_->surface_radius(regolith::normalize(x), false);
}

String RegolithSystem::get_dominant_name() const {
	return dominant_ ? String(dominant_->get_name()) : String();
}

Vector3 RegolithSystem::get_body_position(const String &p_name) const {
	for (const RegolithBody *b : bodies_) {
		if (String(b->get_name()) == p_name) {
			const Pose &d = pose_of(dominant_);
			return regolith::to_v(frame_.to_local_point(from_icrf(d.axes, pose_of(b).position - d.position)));
		}
	}
	return Vector3();
}

Array RegolithSystem::get_bodies_info(const Vector3 &p_pos) const {
	Array out;
	if (!anchored_) {
		return out;
	}
	const DVec3 x = to_inertial(p_pos);
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

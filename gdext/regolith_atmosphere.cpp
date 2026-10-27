#include "regolith_atmosphere.hpp"

#include <algorithm>
#include <cmath>

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/directional_light3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/environment.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/sky.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>

#include "convert.hpp"
#include "regolith/ephemeris.hpp"
#include "regolith_body.hpp"
#include "regolith_system.hpp"

using namespace godot;
using regolith::DVec3;

namespace {

constexpr double SCALE_HEIGHT = 11100.0; // m
constexpr double THIN_START = 12000.0; // full sky below this

double sstep(double p_a, double p_b, double p_x) {
	const double t = std::clamp((p_x - p_a) / (p_b - p_a), 0.0, 1.0);
	return t * t * (3.0 - 2.0 * t);
}

DVec3 from_icrf(const DVec3 p_axes[3], const DVec3 &p_v) {
	return { regolith::dot(p_axes[0], p_v), regolith::dot(p_axes[1], p_v), regolith::dot(p_axes[2], p_v) };
}

Vector3 dir_of(const Basis &p_icrf_to_render, const DVec3 &p_icrf) {
	return p_icrf_to_render.xform(regolith::to_v(regolith::normalize(p_icrf))).normalized();
}

} // namespace

RegolithAtmosphere::RegolithAtmosphere() {
	// After RegolithSystem (-100) has placed the sun.
	set_process_priority(-90);
}

void RegolithAtmosphere::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_system_path", "path"), &RegolithAtmosphere::set_system_path);
	ClassDB::bind_method(D_METHOD("get_system_path"), &RegolithAtmosphere::get_system_path);
	ClassDB::bind_method(D_METHOD("set_body_path", "path"), &RegolithAtmosphere::set_body_path);
	ClassDB::bind_method(D_METHOD("get_body_path"), &RegolithAtmosphere::get_body_path);
	ClassDB::bind_method(D_METHOD("set_sun_path", "path"), &RegolithAtmosphere::set_sun_path);
	ClassDB::bind_method(D_METHOD("get_sun_path"), &RegolithAtmosphere::get_sun_path);
	ClassDB::bind_method(D_METHOD("set_deimos_orbit_path", "path"), &RegolithAtmosphere::set_deimos_orbit_path);
	ClassDB::bind_method(D_METHOD("get_deimos_orbit_path"), &RegolithAtmosphere::get_deimos_orbit_path);
	ClassDB::bind_method(D_METHOD("set_dust_tau", "tau"), &RegolithAtmosphere::set_dust_tau);
	ClassDB::bind_method(D_METHOD("get_dust_tau"), &RegolithAtmosphere::get_dust_tau);
	ClassDB::bind_method(D_METHOD("set_sun_energy", "energy"), &RegolithAtmosphere::set_sun_energy);
	ClassDB::bind_method(D_METHOD("get_sun_energy"), &RegolithAtmosphere::get_sun_energy);
	ClassDB::bind_method(D_METHOD("set_fog_density", "density"), &RegolithAtmosphere::set_fog_density);
	ClassDB::bind_method(D_METHOD("get_fog_density"), &RegolithAtmosphere::get_fog_density);
	ClassDB::bind_method(D_METHOD("get_sky_thin"), &RegolithAtmosphere::get_sky_thin);
	ClassDB::bind_method(D_METHOD("get_terrain_sun"), &RegolithAtmosphere::get_terrain_sun);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "system"), "set_system_path", "get_system_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "body"), "set_body_path", "get_body_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "sun"), "set_sun_path", "get_sun_path");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "deimos_orbit_path", PROPERTY_HINT_FILE, "*.rge"), "set_deimos_orbit_path",
			"get_deimos_orbit_path");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dust_tau"), "set_dust_tau", "get_dust_tau");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "sun_energy"), "set_sun_energy", "get_sun_energy");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fog_density"), "set_fog_density", "get_fog_density");
}

void RegolithAtmosphere::_ready() {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	if (!deimos_orbit_path_.is_empty()) {
		has_deimos_ = deimos_.open(ProjectSettings::get_singleton()->globalize_path(deimos_orbit_path_).utf8().get_data());
	}
	Ref<World3D> world = get_viewport() ? get_viewport()->get_world_3d() : Ref<World3D>();
	Ref<Environment> env = world.is_valid() ? world->get_environment() : Ref<Environment>();
	if (env.is_valid() && env->get_sky().is_valid()) {
		sky_ = env->get_sky()->get_material();
	}
	if (sky_.is_null()) {
		ERR_PRINT("RegolithAtmosphere: the environment has no sky ShaderMaterial");
	}
}

// mars_daynight._sun_occlusion: the highest terrain along the
// sun's azimuth, 16 km to ~600 km out, against the sun.
void RegolithAtmosphere::occlusion(RegolithSystem *p_sys, RegolithBody *p_body, const DVec3 &p_eye, const DVec3 &p_sun,
		double &r_vis, double &r_horizon, double &r_dist) const {
	r_vis = 1.0;
	r_horizon = -0.1;
	r_dist = 1e5;
	const DVec3 up = regolith::normalize(p_eye);
	const double se = regolith::dot(p_sun, up);
	if (se > 0.22 || se < -0.05) {
		return;
	}
	DVec3 h = p_sun - up * se;
	if (regolith::length(h) < 1e-6) {
		return;
	}
	h = regolith::normalize(h);
	const double r = p_body->r_ref();
	double max_ang = -1.5707963;
	double d = 16000.0;
	for (int i = 0; i < 96; ++i) {
		const DVec3 dir = regolith::normalize(up * std::cos(d / r) + h * std::sin(d / r));
		const double sr = p_body->surface_radius(dir, false);
		if (!std::isnan(sr)) {
			const DVec3 v = dir * sr - p_eye;
			const double ang = std::asin(std::clamp(regolith::dot(regolith::normalize(v), up), -1.0, 1.0));
			if (ang > max_ang) {
				max_ang = ang;
				r_dist = d;
			}
		}
		d *= 1.046;
	}
	r_horizon = max_ang;
	r_vis = std::clamp((std::asin(std::clamp(se, -1.0, 1.0)) - max_ang) / 0.012 + 0.5, 0.0, 1.0);
}

void RegolithAtmosphere::_process(double p_delta) {
	RegolithSystem *sys = Object::cast_to<RegolithSystem>(get_node_or_null(system_path_));
	RegolithBody *body = Object::cast_to<RegolithBody>(get_node_or_null(body_path_));
	Camera3D *cam = get_viewport() ? get_viewport()->get_camera_3d() : nullptr;
	if (!sys || !body || !cam || !sys->is_anchored() || sky_.is_null()) {
		return;
	}
	const double t = sys->get_time();
	const RegolithSystem::Pose &mp = sys->pose_of(body);
	const DVec3 x = sys->to_state(cam->get_global_position(), Vector3()).x;
	const DVec3 r_icrf = x - mp.position;
	const DVec3 eye = from_icrf(mp.axes, r_icrf); // body-fixed
	const double r = regolith::length(eye);
	const double alt = std::max(r - body->datum_radius(regolith::normalize(eye)), 0.0);
	const Basis to_render = sys->icrf_to_render();

	// Air above you and the horizon dip, as mors.
	sky_thin_ = std::exp(-std::max(alt - THIN_START, 0.0) / SCALE_HEIGHT);
	const double dip = std::acos(std::clamp(body->r_ref() / (body->r_ref() + alt), 0.0, 1.0));
	const Vector3 up = sys->get_up(cam->get_global_position());
	const Vector3 sun_dir = sys->get_sun_direction();
	sky_->set_shader_parameter("sky_thin", sky_thin_);
	sky_->set_shader_parameter("horizon_dip", dip);
	sky_->set_shader_parameter("up_dir", up);
	sky_->set_shader_parameter("dust_tau", dust_tau_);

	// Stars fixed to ICRF; galactic pole for the Milky Way.
	sky_->set_shader_parameter("celestial_basis", to_render);
	sky_->set_shader_parameter("milkyway_normal", regolith::to_v(regolith::galactic_pole()));

	// Earth: real heliocentric orbits, phase from the triangle.
	const DVec3 mars_h = regolith::mars_heliocentric(t);
	const DVec3 earth_h = regolith::earth_heliocentric(t);
	const DVec3 to_earth = earth_h - mars_h;
	const double phase_ang = std::acos(std::clamp(regolith::dot(regolith::normalize(earth_h * -1.0), regolith::normalize(to_earth * -1.0)), -1.0, 1.0));
	const Vector3 earth_dir = dir_of(to_render, to_earth);
	const Vector3 pole = dir_of(to_render, mp.axes[2]);
	const Vector3 perp = pole.cross(earth_dir).normalized();
	sky_->set_shader_parameter("earth_dir", earth_dir);
	sky_->set_shader_parameter("earth_phase", (1.0 + std::cos(phase_ang)) * 0.5);
	sky_->set_shader_parameter("earth_moon_dir", earth_dir.rotated(perp, 0.0032));

	// Deimos: JPL orbit, topocentric.
	if (has_deimos_) {
		const DVec3 dpos = mp.position + deimos_.state(t).position;
		sky_->set_shader_parameter("deimos_dir", dir_of(to_render, dpos - x));
		sky_->set_shader_parameter("deimos_brightness", 0.9);
	} else {
		sky_->set_shader_parameter("deimos_brightness", 0.0);
	}

	// Terrain occlusion of a low sun, every 0.25 s, near ground.
	const DVec3 sun_icrf = regolith::normalize(regolith::mars_heliocentric(t) * -1.0);
	occl_cool_ -= p_delta;
	if (occl_cool_ <= 0.0) {
		occl_cool_ = 0.25;
		occl_target_ = 1.0;
		occl_horiz_target_ = -0.1;
		occl_dist_target_ = 1e5;
		if (alt < 15000.0) {
			occlusion(sys, body, eye, from_icrf(mp.axes, sun_icrf), occl_target_, occl_horiz_target_, occl_dist_target_);
		}
	}
	const double k = 1.0 - std::exp(-p_delta * 3.0);
	sun_vis_ += (occl_target_ - sun_vis_) * k;
	occl_horiz_ += (occl_horiz_target_ - occl_horiz_) * k;
	occl_dist_ += (occl_dist_target_ - occl_dist_) * k;
	sky_->set_shader_parameter("occl", 1.0 - sun_vis_);
	sky_->set_shader_parameter("occl_horizon", occl_horiz_);
	sky_->set_shader_parameter("occl_dist", occl_dist_);

	// Sun light: reddens and fades through the dust low down;
	// without air it is a hard edge. Eclipses come from the system.
	const double se = sun_dir.dot(up);
	const double se_h = std::sin(std::asin(std::clamp(se, -1.0, 1.0)) + dip);
	const double dust_dim = std::exp(-std::max(dust_tau_ - 0.35, 0.0) * 0.35);
	const double rise = sky_thin_ * sstep(-0.03, 0.18, se_h) + (1.0 - sky_thin_) * sstep(-0.004, 0.004, se_h);
	DirectionalLight3D *sun = Object::cast_to<DirectionalLight3D>(get_node_or_null(sun_path_));
	if (sun) {
		sun->set_param(Light3D::PARAM_ENERGY,
				real_t(sun_energy_ * rise * dust_dim * sun_vis_ * sys->get_sun_visible()));
		const double red = (1.0 - sstep(0.0, 0.3, se_h)) * sky_thin_;
		sun->set_color(Color(1, 1, 1).lerp(Color(1.0, 0.62, 0.42), real_t(red)));
		sun->set_shadow(se_h > -0.03 && sun_vis_ > 0.05);
	}

	// Ambient and haze follow the sun; unlit air does not glow.
	Ref<Environment> env = get_viewport()->get_world_3d()->get_environment();
	if (env.is_valid()) {
		const double dayness = sstep(-0.10, 0.15, se);
		const double amb = (0.04 + 0.20 * dayness) * (0.72 + 0.28 * sun_vis_);
		env->set_ambient_light_energy(real_t(0.03 + (amb - 0.03) * sky_thin_));
		Color fog = Color(0.045f, 0.045f, 0.085f).lerp(Color(0.72f, 0.55f, 0.42f), real_t(dayness));
		fog = fog * real_t(1.0 + ((1.0 + (0.45 - 1.0) * dayness) - 1.0) * (1.0 - sun_vis_));
		env->set_fog_light_color(fog);
		env->set_fog_density(real_t(fog_density_ * sky_thin_));
		// Godot fogs the sky at full strength whatever the
		// density; above the air there is nothing to fog.
		env->set_fog_sky_affect(real_t(fog_sky_affect_ * sky_thin_));
	}
}

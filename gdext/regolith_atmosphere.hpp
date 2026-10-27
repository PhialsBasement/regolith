#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/string.hpp>

#include "regolith/orbit.hpp"

namespace godot {

class RegolithBody;
class RegolithSystem;

// mors's mars_daynight.gd on the real geometry: drives the sky
// shader (sun, stars, Earth, Deimos, air above you), the sun's
// colour and energy, ambient, fog, and terrain occlusion of a
// low sun for one body's atmosphere.
class RegolithAtmosphere : public Node {
	GDCLASS(RegolithAtmosphere, Node)

public:
	RegolithAtmosphere();
	void _ready() override;
	void _process(double p_delta) override;

	void set_system_path(const NodePath &p_v) { system_path_ = p_v; }
	NodePath get_system_path() const { return system_path_; }
	void set_body_path(const NodePath &p_v) { body_path_ = p_v; }
	NodePath get_body_path() const { return body_path_; }
	void set_sun_path(const NodePath &p_v) { sun_path_ = p_v; }
	NodePath get_sun_path() const { return sun_path_; }
	void set_deimos_orbit_path(const String &p_v) { deimos_orbit_path_ = p_v; }
	String get_deimos_orbit_path() const { return deimos_orbit_path_; }
	void set_dust_tau(double p_v) { dust_tau_ = p_v; }
	double get_dust_tau() const { return dust_tau_; }
	void set_sun_energy(double p_v) { sun_energy_ = p_v; }
	double get_sun_energy() const { return sun_energy_; }
	void set_fog_density(double p_v) { fog_density_ = p_v; }
	double get_fog_density() const { return fog_density_; }
	double get_sky_thin() const { return sky_thin_; }
	double get_terrain_sun() const { return sun_vis_; }

protected:
	static void _bind_methods();

private:
	void occlusion(RegolithSystem *p_sys, RegolithBody *p_body, const regolith::DVec3 &p_eye,
			const regolith::DVec3 &p_sun, double &r_vis, double &r_horizon, double &r_dist) const;

	NodePath system_path_;
	NodePath body_path_;
	NodePath sun_path_;
	String deimos_orbit_path_;
	double dust_tau_ = 0.35; // mors's clear season
	double sun_energy_ = 1.7;
	double fog_density_ = 2.7e-5;
	double fog_sky_affect_ = 0.15;

	Ref<ShaderMaterial> sky_;
	regolith::OrbitTable deimos_;
	bool has_deimos_ = false;
	double sky_thin_ = 1.0;
	double sun_vis_ = 1.0;
	double occl_target_ = 1.0;
	double occl_horiz_ = -0.1, occl_horiz_target_ = -0.1;
	double occl_dist_ = 1e5, occl_dist_target_ = 1e5;
	double occl_cool_ = 0.0;
};

} // namespace godot

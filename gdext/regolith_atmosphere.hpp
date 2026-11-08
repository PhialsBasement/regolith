// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <atomic>
#include <mutex>

#include <godot_cpp/classes/compositor_effect.hpp>
#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/classes/render_data.hpp>
#include <godot_cpp/classes/stream_peer_buffer.hpp>
#include <godot_cpp/classes/texture2drd.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <vector>

#include <godot_cpp/variant/packed_string_array.hpp>

#include "regolith/air.hpp"
#include "regolith/orbit.hpp"
#include "regolith/vec.hpp"

namespace godot {

class Camera3D;
class RenderingDevice;
class RegolithBody;
class RegolithSystem;

// The sky, haze and sun as one physically based atmosphere
// (Hillaire 2020), a compute pass after the opaque scene: the
// background becomes the sky, geometry gets aerial perspective.
class RegolithAirEffect : public CompositorEffect {
	GDCLASS(RegolithAirEffect, CompositorEffect)

public:
	struct Frame {
		regolith::AirModel air;
		bool has_air = false;
		Basis to_body; // render vector to body frame
		regolith::DVec3 eye; // body frame, metres
		Vector3 sun; // body frame, unit
		double sun_energy = 0.0; // above the air
		double sun_radius = 0.0; // angular, rad
		double exposure = 1.0;
		double airglow = 0.0;
		Color glow_color = Color(1, 1, 1); // luminance 1
		// pi x the sun's illuminance at 1 AU x night vision: the
		// stars' and the Milky Way's units
		double star_scale = 0.0;
		Basis body_to_icrf;
		bool valid = false;
	};

	RegolithAirEffect();
	~RegolithAirEffect() override;
	void _render_callback(int32_t p_type, RenderData *p_data) override;
	void set_frame(const Frame &p_frame);
	// for a sun of illuminance 1: x mean over normals, y level
	// ground; one frame behind, from the GPU
	Vector3 ambient_mean() const;
	Vector3 ambient_up() const;
	// the view's mean luminance, last frame
	double view_luminance() const;
	// the tables the terrain reads per point, once built: the
	// transmittance and skylight tables, their base, top, datum
	bool tables(RID &r_t, RID &r_e, double &r_base, double &r_top, double &r_datum) const;

protected:
	static void _bind_methods();

private:
	bool ensure(RenderingDevice *p_rd);
	// the star catalogue binned by sky cell, and the Milky Way
	void load_sky();
	void free_all();
	void on_ambient(const PackedByteArray &p_data);

	mutable std::mutex mutex_;
	Frame frame_;
	Vector3 amb_mean_, amb_up_;
	double view_lum_ = 1.0;
	std::atomic<bool> reading_{ false };
	// Async readbacks land in an engine object: Godot delivers the
	// last ones after unloading extensions, so never into ours.
	Ref<StreamPeerBuffer> sink_;
	RenderingDevice *rd_ = nullptr;
	bool failed_ = false;
	regolith::AirModel built_air_;
	bool built_ = false;
	RID t_shader_, ms_shader_, sv_shader_, ap_shader_, comp_shader_, amb_shader_, e_shader_;
	RID t_pipe_, ms_pipe_, sv_pipe_, ap_pipe_, comp_pipe_, amb_pipe_, e_pipe_;
	RID lin_clamp_, nearest_, params_;
	RID t_lut_, ms_lut_, sv_lut_, ap_lut_, amb_tex_, e_lut_;
	double built_base_ = 0.0; // the tables' base radius
	bool tables_ready_ = false;
	RID stars_buf_, cells_buf_, ids_buf_, mw_tex_, mw_sampler_;
	double mw_unit_ = 0.0; // the Milky Way map's unit, suns per sr
};

// The node: a body's air (a preset or physical coefficients),
// fed to a RegolithAirEffect on the active camera; it also sets
// the sun's colour and energy and the ambient light from it.
class RegolithAtmosphere : public Node {
	GDCLASS(RegolithAtmosphere, Node)

public:
	RegolithAtmosphere();
	void _ready() override;
	void _process(double p_delta) override;
	void _exit_tree() override;

	void set_system_path(const NodePath &p_v) { system_path_ = p_v; }
	NodePath get_system_path() const { return system_path_; }
	void set_body_path(const NodePath &p_v) { body_path_ = p_v; }
	NodePath get_body_path() const { return body_path_; }
	void set_sun_path(const NodePath &p_v) { sun_path_ = p_v; }
	NodePath get_sun_path() const { return sun_path_; }
	// None (airless), Mars (CO2 and live dust), Earth, Custom
	void set_air(const String &p_v);
	String get_air() const { return air_; }
	void set_dust_tau(double p_v) { dust_tau_ = p_v; }
	double get_dust_tau() const { return dust_tau_; }
	void set_sun_energy(double p_v) { sun_energy_ = p_v; }
	double get_sun_energy() const { return sun_energy_; }
	void set_exposure(double p_v) { exposure_ = p_v; }
	double get_exposure() const { return exposure_; }
	void set_airglow(double p_v) { airglow_ = p_v; }
	double get_airglow() const { return airglow_; }
	void set_airglow_color(const Color &p_v) { airglow_color_ = p_v; }
	Color get_airglow_color() const { return airglow_color_; }
	// the most the eye adapts to darkness beyond the camera's own
	// exposure: the stars' gain in a dark view (1: true to the sun)
	void set_night_vision(double p_v) { night_vision_ = p_v; }
	double get_night_vision() const { return night_vision_; }
	double get_eye_gain() const { return eye_gain_; }
	double get_view_luminance() const { return effect_.is_valid() ? effect_->view_luminance() : 0.0; }
	// the coefficients, per km; heights in km
	void set_air_value(int p_index, const Variant &p_v);
	Variant get_air_value(int p_index) const;
	// the old fog knob: the haze is the air now
	void set_fog_density(double p_v) { fog_density_ = p_v; }
	double get_fog_density() const { return fog_density_; }
	double get_sky_thin() const { return sky_thin_; }
	double get_terrain_sun() const { return sun_vis_; }
	// the sun's colour and energy where the camera is
	Color get_sun_color() const { return sun_color_; }
	// the air's colour and extinction (1/m) at the camera, for
	// haze others draw (the clouds)
	Color get_haze() const { return haze_; }
	// where "Galactic Center" or "Milky Way" (its part opposite the
	// sun) stands: render space, or the body's fixed frame
	Vector3 get_sky_direction(const String &p_name, bool p_body_frame = false) const;

protected:
	static void _bind_methods();

private:
	// p_ground: the sphere the sky meets; p_datum: where the
	// coefficients hold
	regolith::AirModel model(double p_ground, double p_datum) const;
	void attach(Camera3D *p_cam);
	void detach();
	void occlusion(RegolithSystem *p_sys, RegolithBody *p_body, const regolith::DVec3 &p_eye,
			const regolith::DVec3 &p_sun, double &r_vis) const;

	NodePath system_path_;
	NodePath body_path_;
	NodePath sun_path_;
	String air_ = "Mars";
	regolith::AirModel custom_; // per metre, heights in metres
	double dust_tau_ = 0.35;
	double sun_energy_ = 1.7;
	double exposure_ = 1.0;
	// the zenith's glow per sr, in suns at 1 AU: 24 V mag/arcsec2,
	// faint enough to stay a halo at the limb
	double airglow_ = 2.1e-10;
	Color airglow_color_ = Color(1.0, 0.62, 0.42); // dusty warm
	// enough to lift a dark sky's band above the tonemapper's toe
	double night_vision_ = 300000.0;
	double eye_gain_ = 1.0; // adapted to the view, up to the above
	double fog_density_ = 0.0;

	Ref<RegolithAirEffect> effect_;
	uint64_t attached_id_ = 0; // instance id: the camera can be freed under us
	// the effect's tables, as textures the terrain can sample
	Ref<Texture2DRD> air_t_tex_, air_e_tex_;
	double sky_thin_ = 1.0;
	double sun_vis_ = 1.0;
	double occl_target_ = 1.0;
	double occl_cool_ = 0.0;
	Color sun_color_ = Color(1, 1, 1);
	Color haze_ = Color(0, 0, 0, 0);
};

} // namespace godot

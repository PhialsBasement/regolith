// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/string.hpp>

#include "regolith/climate.hpp"
#include "regolith/vec.hpp"

namespace godot {

class RegolithAtmosphere;
class RegolithBody;
class RegolithSystem;

// One body's weather: a closed-form climate (regolith::Climate)
// re-evaluated on a worker as the clock runs, its fields for
// gameplay and maps, and the dust tau for the atmosphere.
class RegolithWeather : public Node3D {
	GDCLASS(RegolithWeather, Node3D)

public:
	~RegolithWeather() override;
	void _process(double p_delta) override;
	void _exit_tree() override;

	void set_system_path(const NodePath &p_v) { system_path_ = p_v; }
	NodePath get_system_path() const { return system_path_; }
	void set_body_path(const NodePath &p_v) { body_path_ = p_v; }
	NodePath get_body_path() const { return body_path_; }
	void set_atmosphere_path(const NodePath &p_v) { atmosphere_path_ = p_v; }
	NodePath get_atmosphere_path() const { return atmosphere_path_; }
	void set_albedo_map_path(const String &p_v) { albedo_map_path_ = p_v; }
	String get_albedo_map_path() const { return albedo_map_path_; }
	void set_climatology_path(const String &p_v) { clim_path_ = p_v; }
	String get_climatology_path() const { return clim_path_; }
	// body and grid overrides (ClimateConfig's field names)
	void set_config(const Dictionary &p_v) { config_ = p_v; }
	Dictionary get_config() const { return config_; }

	// gameplay queries at a render-space point (main thread)
	double get_tau(const Vector3 &p_pos) const;
	Vector3 get_wind(const Vector3 &p_pos) const; // surface, render space
	Vector2 get_wind_local(const Vector3 &p_pos) const; // east, north
	Vector3 get_layer_wind(const Vector3 &p_pos) const;
	// the air at the point: the ground's under it, thinned with height
	double get_pressure(const Vector3 &p_pos) const;
	double get_density(const Vector3 &p_pos) const;
	double get_air_temperature(const Vector3 &p_pos) const;
	double get_ground_temperature(const Vector3 &p_pos) const;
	Vector2 get_frost(const Vector3 &p_pos) const; // CO2, water ice, kg/m2
	double get_cloud(const Vector3 &p_pos) const; // visible tau
	double get_vapor(const Vector3 &p_pos) const; // kg/m2
	// tau below 14 km, tau above, ice column kg/m2
	Vector3 get_cloud_layers(const Vector3 &p_pos) const;
	// unit vector to the sun, body-fixed (z north, x lon 0)
	Vector3 get_sun_direction() const;
	// the global packs (planet map overlays): clouds and dust,
	// CO2 ice and frost
	Ref<ImageTexture> get_global_texture() const { return global_tex_; }
	Ref<ImageTexture> get_surface_texture() const { return surface_tex_; }
	void inject_dust(const Vector3 &p_pos);
	bool is_ready() const { return ready_.load(); }
	Dictionary get_stats() const;
	// for a cloud renderer (C++ only): tau per level as NL x GH x
	// GW, lon fastest, and the ground (m); false if unchanged
	bool cloud_field(uint64_t &r_version, std::vector<float> &r_tau, std::vector<float> &r_topo, int &r_gw,
			int &r_gh) const;

protected:
	static void _bind_methods();

private:
	struct Published {
		std::vector<float> dust, cloud, lu, lv, su, sv, qv, ts, tair, p, co2, frost;
		std::array<std::vector<float>, 3> q;
		std::vector<uint8_t> global, surface;
		std::vector<float> tau_levels; // NL x N
		uint64_t version = 0;
		double ls = 0.0;
		bool fresh = false;
	};

	RegolithSystem *system() const;
	RegolithBody *body() const;
	void start();
	void run();
	void publish(const regolith::Climate &p_c);
	// render-space point -> planet pixels and the local axes
	bool locate(const Vector3 &p_pos, double &r_px, double &r_py, regolith::DVec3 *r_dir = nullptr) const;
	double pub_at(const std::vector<float> &p_f, const Vector3 &p_pos) const;
	Vector3 to_render(const regolith::DVec3 &p_dir, double p_east, double p_north) const;
	double height_over_ground(const Vector3 &p_pos) const;
	double r_gas() const;
	double gravity() const;
	void upload_maps();

	NodePath system_path_;
	NodePath body_path_;
	NodePath atmosphere_path_;
	String albedo_map_path_;
	String clim_path_;
	Dictionary config_;

	std::unique_ptr<regolith::Climate> climate_;
	RegolithBody *body_ptr_ = nullptr;
	std::thread thread_;
	std::atomic<bool> exit_{ false };
	std::atomic<bool> ready_{ false };
	bool started_ = false;
	std::vector<float> lum_;
	std::vector<std::array<double, 7>> clim_;

	// inputs, main -> worker
	mutable std::mutex mutex_;
	std::condition_variable wake_;
	double in_time_ = 0.0;
	std::vector<std::array<double, 2>> in_dust_;
	Published pub_;
	uint64_t pub_version_ = 0;
	double stat_update_ms_ = 0.0;
	double stat_time_ = 0.0;
	uint64_t stat_updates_ = 0;

	Ref<ImageTexture> global_tex_, surface_tex_;
	double tau_eased_ = -1.0;
};

} // namespace godot

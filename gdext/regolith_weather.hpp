#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/shader_material.hpp>
#include <godot_cpp/classes/sub_viewport.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/string.hpp>

#include "regolith/gcm.hpp"
#include "regolith/vec.hpp"

namespace godot {

class RegolithAtmosphere;
class RegolithBody;
class RegolithSystem;

// One body's weather: the GCM on its own thread, paced by the
// system clock (time warp included), its fields for gameplay
// queries, the ice decks and dust drawn around the camera, and
// the dust optical depth handed to the atmosphere.
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
	void set_sun_path(const NodePath &p_v) { sun_path_ = p_v; }
	NodePath get_sun_path() const { return sun_path_; }
	void set_albedo_map_path(const String &p_v) { albedo_map_path_ = p_v; }
	String get_albedo_map_path() const { return albedo_map_path_; }
	void set_climatology_path(const String &p_v) { clim_path_ = p_v; }
	String get_climatology_path() const { return clim_path_; }
	void set_save_path(const String &p_v) { save_path_ = p_v; }
	String get_save_path() const { return save_path_; }
	// A fresh world's start (made by set_bake), when no save.
	void set_initial_state_path(const String &p_v) { initial_state_path_ = p_v; }
	String get_initial_state_path() const { return initial_state_path_; }
	// Before entering the tree: spin p_sols up to now, write it.
	void set_bake(int p_sols, const String &p_path) {
		bake_sols_ = p_sols;
		bake_path_ = p_path;
	}
	bool is_bake_done() const { return bake_done_.load(); }
	void set_threads(int p_v) { threads_ = p_v; }
	int get_threads() const { return threads_; }
	void set_config(const Dictionary &p_v) { config_ = p_v; }
	Dictionary get_config() const { return config_; }
	void set_debug_view(int p_v) { debug_view_ = p_v; }
	int get_debug_view() const { return debug_view_; }
	void set_clouds_visible(bool p_v) { clouds_visible_ = p_v; }
	bool get_clouds_visible() const { return clouds_visible_; }

	// gameplay queries at a render-space point (main thread)
	double get_tau(const Vector3 &p_pos) const;
	Vector3 get_wind(const Vector3 &p_pos) const; // surface, render space
	Vector2 get_wind_local(const Vector3 &p_pos) const; // east, north
	Vector3 get_layer_wind(const Vector3 &p_pos) const;
	double get_pressure(const Vector3 &p_pos) const;
	double get_density(const Vector3 &p_pos) const;
	double get_air_temperature(const Vector3 &p_pos) const;
	double get_ground_temperature(const Vector3 &p_pos) const;
	Vector2 get_frost(const Vector3 &p_pos) const; // CO2 kg/m2, water
	double get_cloud(const Vector3 &p_pos) const;
	double get_vapor(const Vector3 &p_pos) const;
	Vector3 get_cloud_layers(const Vector3 &p_pos) const; // low, cirrus, meso
	Dictionary find_clouds(double p_min_sun_deg) const;
	void inject_dust(const Vector3 &p_pos);
	bool is_ready() const { return ready_.load(); }
	Dictionary get_stats() const;
	void save_now();

protected:
	static void _bind_methods();

private:
	struct Published {
		std::vector<float> dust, cloud, lu, lv, su, sv, qv, ts, tair, p, co2, frost;
		std::array<std::vector<float>, 3> q;
		std::vector<uint8_t> local, global, surface;
		double org_x = 0.0, org_y = 0.0;
		bool fresh = false;
	};

	RegolithSystem *system() const;
	RegolithBody *body() const;
	void start();
	void run();
	bool load(regolith::Gcm &p_g, double &r_sim_time) const;
	void save(const regolith::Gcm &p_g, double p_sim_time, const String &p_path) const;
	void publish(const regolith::Gcm &p_g);
	// render-space point -> planet pixels and the local axes
	bool locate(const Vector3 &p_pos, double &r_px, double &r_py, regolith::DVec3 *r_dir = nullptr) const;
	double pub_at(const std::vector<float> &p_f, const Vector3 &p_pos) const;
	Vector3 to_render(const regolith::DVec3 &p_dir, double p_east, double p_north) const;
	void setup_clouds();
	void update_clouds();

	NodePath system_path_;
	NodePath body_path_;
	NodePath atmosphere_path_;
	NodePath sun_path_;
	String albedo_map_path_;
	String clim_path_;
	String save_path_ = "user://regolith_weather.save";
	String save_global_;
	String initial_state_path_;
	PackedByteArray save_bytes_, initial_bytes_;
	int bake_sols_ = 0;
	String bake_path_, bake_global_;
	std::atomic<bool> bake_done_{ false };
	int threads_ = 4;
	Dictionary config_;
	bool clouds_visible_ = true;
	int debug_view_ = 0;

	std::unique_ptr<regolith::Gcm> gcm_;
	RegolithBody *body_ptr_ = nullptr;
	std::thread thread_;
	std::atomic<bool> exit_{ false };
	std::atomic<bool> ready_{ false };
	std::atomic<bool> save_request_{ false };
	bool started_ = false;
	std::vector<float> lum_;
	std::vector<double> clim_;

	// inputs, main -> thread
	mutable std::mutex mutex_;
	double in_time_ = 0.0;
	double in_px_ = 0.0, in_py_ = 0.0;
	bool in_dust_ = false;
	double in_dust_px_ = 0.0, in_dust_py_ = 0.0;
	Published pub_;
	// stats, thread -> main
	double stat_step_ms_ = 0.0;
	double stat_lag_s_ = 0.0;
	double stat_season_k_ = 1.0;
	double stat_sim_time_ = 0.0;
	uint64_t stat_steps_ = 0;

	// rendering
	static constexpr int SHADOW_TEXELS = 256;
	static constexpr double SHADOW_M = 100000.0;
	Ref<ShaderMaterial> mat_, shadow_mat_;
	MeshInstance3D *box_ = nullptr;
	SubViewport *shadow_vp_ = nullptr;
	Ref<ImageTexture> local_tex_, global_tex_, surface_tex_;
	double org_x_ = 0.0, org_y_ = 0.0;
	double ground_alt_ = 0.0;
	double eye_alt_ = 0.0;
	bool ground_set_ = false;
	regolith::DVec3 chart_up_;
	bool chart_set_ = false;
	regolith::DVec3 drift_air_, drift_dust_;
	double last_time_ = 0.0;
	double tau_eased_ = -1.0;
};

} // namespace godot

#pragma once

#include <array>
#include <mutex>
#include <vector>

#include <godot_cpp/classes/compositor_effect.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/render_data.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/color.hpp>
#include <godot_cpp/variant/node_path.hpp>
#include <godot_cpp/variant/projection.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/vector2i.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "regolith/vec.hpp"

namespace godot {

class Camera3D;
class RenderingDevice;
class RegolithWeather;

// Volumetric clouds as a compute pass after transparents. One
// cloud density, a function of world position, is marched per
// pixel; only its lighting is baked, into cascades.
class RegolithCloudEffect : public CompositorEffect {
	GDCLASS(RegolithCloudEffect, CompositorEffect)

public:
	static constexpr int NC = 4;

	struct Frame {
		Basis to_body; // render vector to body frame
		regolith::DVec3 eye; // body frame, metres
		double datum = 0.0;
		Vector3 sun; // body frame
		double sun_energy = 0.0;
		Color sun_color, ambient;
		Color fog; // rgb, a density (1/m)
		regolith::DVec3 drift; // wind offset, body frame (m)
		bool valid = false;
	};
	struct Look {
		double density = 1.0;
		double coverage_gain = 8000.0; // per 1/m of mean extinction
		double shape_period = 36000.0; // m
		double detail_period = 2000.0; // m
		double phase_g = 0.8; // ice crystals: strongly forward
		double resolution_scale = 0.5;
		int max_steps = 192;
	};
	struct Field {
		int gw = 0, gh = 0;
		std::vector<float> tau, topo; // NL x GH x GW, GH x GW
		std::vector<float> band; // GH x GW x 4: low, high, peak
		std::vector<float> edges;
	};

	RegolithCloudEffect();
	~RegolithCloudEffect() override;
	void _render_callback(int32_t p_type, RenderData *p_data) override;

	void set_frame(const Frame &p_frame);
	void set_look(const Look &p_look);
	void set_field(Field &&p_field);

protected:
	static void _bind_methods() {}

private:
	struct Cascade {
		regolith::DVec3 a, e, n;
		regolith::DVec3 drift; // the wind offset it was baked at
		bool built = false;
		double last = -1e9; // steady seconds of the last bake
	};

	bool ensure(RenderingDevice *p_rd);
	void free_all();
	int pick_job(const Frame &p_f, double p_now) const;
	void anchor(int p_k, const regolith::DVec3 &p_dir);
	std::vector<RID> mip_views(RID p_tex, int p_mips);

	std::mutex mutex_;
	Frame frame_;
	Look look_;
	Field field_;
	bool field_dirty_ = false;
	std::vector<float> edges_;

	RenderingDevice *rd_ = nullptr;
	bool failed_ = false;
	bool noise_done_ = false;
	RID noise_shader_, light_shader_, march_shader_, comp_shader_, mip_shader_, resolve_shader_;
	RID noise_pipe_, light_pipe_, march_pipe_, comp_pipe_, mip_pipe_, resolve_pipe_;
	RID lin_clamp_, nearest_, lin_repeat_, field_sampler_, params_;
	RID shape_tex_, detail_tex_, tau_tex_, topo_tex_, band_tex_;
	// one view per mip, for writing the chains
	std::vector<RID> shape_mips_, detail_mips_;
	std::array<RID, NC> light_;
	std::array<Cascade, NC> casc_;
	RID cloud_tex_, cloud_prev_, cloud_dist_;
	// this frame's traced quarter: colour and cloud depth
	RID trace_tex_, trace_depth_;
	Vector2i cloud_size_;
	bool history_valid_ = false;
	Projection prev_proj_;
	Basis prev_cam_;
	regolith::DVec3 prev_eye_;
	uint32_t frame_index_ = 0;
};

// The node: feeds a RegolithCloudEffect on the active camera
// from the weather's cloud field, the body frame and the sun.
class RegolithClouds : public Node3D {
	GDCLASS(RegolithClouds, Node3D)

public:
	void _ready() override;
	void _process(double p_delta) override;
	void _exit_tree() override;

	void set_system_path(const NodePath &p_v) { system_path_ = p_v; }
	NodePath get_system_path() const { return system_path_; }
	void set_body_path(const NodePath &p_v) { body_path_ = p_v; }
	NodePath get_body_path() const { return body_path_; }
	void set_weather_path(const NodePath &p_v) { weather_path_ = p_v; }
	NodePath get_weather_path() const { return weather_path_; }
	void set_sun_path(const NodePath &p_v) { sun_path_ = p_v; }
	NodePath get_sun_path() const { return sun_path_; }

	void set_density(double p_v) { look_.density = p_v; }
	double get_density() const { return look_.density; }
	void set_coverage_gain(double p_v) { look_.coverage_gain = p_v; }
	double get_coverage_gain() const { return look_.coverage_gain; }
	void set_shape_period(double p_v) { look_.shape_period = p_v; }
	double get_shape_period() const { return look_.shape_period; }
	void set_detail_period(double p_v) { look_.detail_period = p_v; }
	double get_detail_period() const { return look_.detail_period; }
	void set_phase_g(double p_v) { look_.phase_g = p_v; }
	double get_phase_g() const { return look_.phase_g; }
	void set_resolution_scale(double p_v) { look_.resolution_scale = p_v; }
	double get_resolution_scale() const { return look_.resolution_scale; }
	void set_max_steps(int p_v) { look_.max_steps = p_v; }
	int get_max_steps() const { return look_.max_steps; }

protected:
	static void _bind_methods();

private:
	void attach(Camera3D *p_cam);
	void detach();

	NodePath system_path_, body_path_, weather_path_, sun_path_;
	RegolithCloudEffect::Look look_;
	Ref<RegolithCloudEffect> effect_;
	Camera3D *attached_ = nullptr;
	uint64_t field_version_ = 0;
	regolith::DVec3 drift_;
	double last_time_ = -1.0;
	double radius_ = 0.0; // the volume's datum, fixed at start
};

} // namespace godot

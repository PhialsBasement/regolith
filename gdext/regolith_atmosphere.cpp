// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith_atmosphere.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/compositor.hpp>
#include <godot_cpp/classes/directional_light3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/environment.hpp>
#include <godot_cpp/classes/rd_texture_format.hpp>
#include <godot_cpp/classes/rd_texture_view.hpp>
#include <godot_cpp/classes/render_scene_buffers_rd.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/os.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/render_scene_data.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>

#include "convert.hpp"
#include "rd_util.hpp"
#include "regolith/ephemeris.hpp"
#include "regolith_body.hpp"
#include "regolith_system.hpp"

using namespace godot;
using namespace godot::rdu;
using regolith::AirModel;
using regolith::DVec3;
using regolith::RGB;

namespace {

constexpr double AU = 1.495978707e11;
constexpr double SUN_RADIUS = 6.957e8;
// a daylit view's mean luminance: the eye's set point
constexpr double VIEW_KEY = 0.1;
constexpr int T_W = 256, T_H = 64, MS_N = 32, SV_W = 192, SV_H = 108, AP_N = 32, E_W = 64, E_H = 16;
// the tables reach this far under the datum: under any ground
constexpr double TABLE_DEPTH = 15000.0;

bool same_rgb(const RGB &p_a, const RGB &p_b) {
	return p_a[0] == p_b[0] && p_a[1] == p_b[1] && p_a[2] == p_b[2];
}

// the tables only depend on these (not the ground under the
// camera); the datum may drift 50 m
bool same_air(const AirModel &p_a, const AirModel &p_b) {
	return std::fabs(p_a.top - p_b.top) < 50.0 && std::fabs(p_a.datum - p_b.datum) < 50.0 &&
			same_rgb(p_a.rayleigh, p_b.rayleigh) && p_a.rayleigh_height == p_b.rayleigh_height &&
			same_rgb(p_a.mie_scattering, p_b.mie_scattering) && same_rgb(p_a.mie_absorption, p_b.mie_absorption) &&
			p_a.mie_height == p_b.mie_height && same_rgb(p_a.mie_g, p_b.mie_g) && same_rgb(p_a.ozone, p_b.ozone) &&
			p_a.ozone_center == p_b.ozone_center && p_a.ozone_width == p_b.ozone_width &&
			same_rgb(p_a.ground_albedo, p_b.ground_albedo);
}

void push_rgb(std::vector<float> &r_v, const RGB &p_c, double p_w) {
	push4(r_v, p_c[0], p_c[1], p_c[2], p_w);
}

Vector3 to_v3(const RGB &p_c, double p_k) {
	return Vector3(real_t(p_c[0] * p_k), real_t(p_c[1] * p_k), real_t(p_c[2] * p_k));
}

RGB from_v3(const Vector3 &p_v, double p_k) {
	return { double(p_v.x) * p_k, double(p_v.y) * p_k, double(p_v.z) * p_k };
}

} // namespace

// The effect.

RegolithAirEffect::RegolithAirEffect() {
	set_effect_callback_type(EFFECT_CALLBACK_TYPE_POST_SKY);
	sink_.instantiate();
}

RegolithAirEffect::~RegolithAirEffect() {
	if (reading_ && sink_.is_valid()) {
		// in flight: the engine delivers it after we are gone, so
		// the buffer must outlive us (one small leak at quit)
		sink_->reference();
	}
	free_all();
}

void RegolithAirEffect::_bind_methods() {
}

void RegolithAirEffect::set_frame(const Frame &p_frame) {
	std::lock_guard<std::mutex> guard(mutex_);
	frame_ = p_frame;
}

Vector3 RegolithAirEffect::ambient_mean() const {
	std::lock_guard<std::mutex> guard(mutex_);
	return amb_mean_;
}

Vector3 RegolithAirEffect::ambient_up() const {
	std::lock_guard<std::mutex> guard(mutex_);
	return amb_up_;
}

bool RegolithAirEffect::tables(RID &r_t, RID &r_e, double &r_base, double &r_top, double &r_datum) const {
	std::lock_guard<std::mutex> guard(mutex_);
	if (!tables_ready_) {
		return false;
	}
	r_t = t_lut_;
	r_e = e_lut_;
	r_base = built_base_;
	r_top = built_air_.top;
	r_datum = built_air_.datum;
	return true;
}

double RegolithAirEffect::view_luminance() const {
	std::lock_guard<std::mutex> guard(mutex_);
	return view_lum_;
}

void RegolithAirEffect::on_ambient(const PackedByteArray &p_data) {
	if (p_data.size() >= 48) {
		const float *v = reinterpret_cast<const float *>(p_data.ptr());
		std::lock_guard<std::mutex> guard(mutex_);
		amb_mean_ = Vector3(v[0], v[1], v[2]);
		amb_up_ = Vector3(v[4], v[5], v[6]);
		view_lum_ = v[8];
	}
	reading_ = false;
}

void RegolithAirEffect::free_all() {
	if (!rd_) {
		return;
	}
	for (RID *r : { &t_pipe_, &ms_pipe_, &sv_pipe_, &ap_pipe_, &comp_pipe_, &amb_pipe_, &t_shader_, &ms_shader_,
				 &sv_shader_, &ap_shader_, &comp_shader_, &amb_shader_, &lin_clamp_, &nearest_, &params_, &t_lut_,
				 &ms_lut_, &sv_lut_, &ap_lut_, &amb_tex_, &e_pipe_, &e_shader_, &e_lut_, &stars_buf_, &cells_buf_, &ids_buf_, &mw_tex_, &mw_sampler_ }) {
		if (r->is_valid()) {
			rd_->free_rid(*r);
			*r = RID();
		}
	}
	rd_ = nullptr;
}

bool RegolithAirEffect::ensure(RenderingDevice *p_rd) {
	if (failed_) {
		return false;
	}
	if (rd_ == p_rd && comp_pipe_.is_valid()) {
		return true;
	}
	rd_ = p_rd;
	const char *names[7] = { "regolith_air_transmittance", "regolith_air_multiscatter", "regolith_air_skyview",
		"regolith_air_aerial", "regolith_air_composite", "regolith_air_ambient", "regolith_air_irradiance" };
	RID *shaders[7] = { &t_shader_, &ms_shader_, &sv_shader_, &ap_shader_, &comp_shader_, &amb_shader_, &e_shader_ };
	RID *pipes[7] = { &t_pipe_, &ms_pipe_, &sv_pipe_, &ap_pipe_, &comp_pipe_, &amb_pipe_, &e_pipe_ };
	for (int k = 0; k < 7; ++k) {
		*shaders[k] = compile(rd_, names[k]);
		if (!shaders[k]->is_valid()) {
			failed_ = true;
			return false;
		}
		*pipes[k] = rd_->compute_pipeline_create(*shaders[k]);
	}
	lin_clamp_ = make_sampler(rd_, true, RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE, RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE);
	nearest_ = make_sampler(rd_, false, RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE, RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE);
	params_ = rd_->uniform_buffer_create(512);
	t_lut_ = make_storage(rd_, RD::DATA_FORMAT_R16G16B16A16_SFLOAT, T_W, T_H, 1);
	ms_lut_ = make_storage(rd_, RD::DATA_FORMAT_R16G16B16A16_SFLOAT, MS_N, MS_N, 1);
	sv_lut_ = make_storage(rd_, RD::DATA_FORMAT_R16G16B16A16_SFLOAT, SV_W, SV_H, 1);
	ap_lut_ = make_storage(rd_, RD::DATA_FORMAT_R16G16B16A16_SFLOAT, AP_N, AP_N, AP_N);
	e_lut_ = make_storage(rd_, RD::DATA_FORMAT_R16G16B16A16_SFLOAT, E_W, E_H, 1);
	{
		// read back each frame: copyable
		Ref<RDTextureFormat> f;
		f.instantiate();
		f->set_format(RD::DATA_FORMAT_R32G32B32A32_SFLOAT);
		f->set_width(3);
		f->set_height(1);
		f->set_usage_bits(RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_CAN_COPY_FROM_BIT);
		Ref<RDTextureView> v;
		v.instantiate();
		amb_tex_ = rd_->texture_create(f, v);
	}
	load_sky();
	built_ = false;
	return true;
}

namespace {

constexpr int CELLS = 64;

// the cube face cell of an ICRF direction, as the shader finds it
uint32_t star_cell(const DVec3 &p_d) {
	const double mx = std::fabs(p_d.x), my = std::fabs(p_d.y), mz = std::fabs(p_d.z);
	uint32_t face;
	double u, v;
	if (mx >= my && mx >= mz) {
		face = p_d.x > 0.0 ? 0 : 1;
		u = p_d.y / mx;
		v = p_d.z / mx;
	} else if (my >= mz) {
		face = p_d.y > 0.0 ? 2 : 3;
		u = p_d.x / my;
		v = p_d.z / my;
	} else {
		face = p_d.z > 0.0 ? 4 : 5;
		u = p_d.x / mz;
		v = p_d.y / mz;
	}
	const int cu = std::clamp(int((u * 0.5 + 0.5) * CELLS), 0, CELLS - 1);
	const int cv = std::clamp(int((v * 0.5 + 0.5) * CELLS), 0, CELLS - 1);
	return face * CELLS * CELLS + uint32_t(cu * CELLS + cv);
}

} // namespace

void RegolithAirEffect::load_sky() {
	std::vector<float> star_data;
	std::vector<std::vector<uint32_t>> bins(size_t(6 * CELLS * CELLS));
	Ref<Image> si = ResourceLoader::get_singleton()->load("res://addons/regolith/regolith_stars.res");
	if (si.is_valid()) {
		const int n = int(si->get_meta("count", 0));
		// each star in every cell within its glow's reach
		const double reach = 0.35 * std::numbers::pi / 180.0;
		for (int i = 0; i < n; ++i) {
			const Color p = si->get_pixel((2 * i) % si->get_width(), (2 * i) / si->get_width());
			const Color c = si->get_pixel((2 * i + 1) % si->get_width(), (2 * i + 1) / si->get_width());
			star_data.insert(star_data.end(), { p.r, p.g, p.b, p.a, c.r, c.g, c.b, c.a });
			const DVec3 d(p.r, p.g, p.b);
			const DVec3 e = regolith::normalize(regolith::cross(d, std::fabs(d.z) < 0.9 ? DVec3(0, 0, 1) : DVec3(1, 0, 0)));
			const DVec3 f = regolith::cross(d, e);
			for (int a = -1; a <= 1; ++a) {
				for (int b = -1; b <= 1; ++b) {
					const uint32_t cell = star_cell(regolith::normalize(d + e * (a * reach) + f * (b * reach)));
					std::vector<uint32_t> &bin = bins[cell];
					if (bin.empty() || bin.back() != uint32_t(i)) {
						if (std::find(bin.begin(), bin.end(), uint32_t(i)) == bin.end()) {
							bin.push_back(uint32_t(i));
						}
					}
				}
			}
		}
	}
	std::vector<uint32_t> first, ids;
	first.reserve(bins.size() + 1);
	for (const std::vector<uint32_t> &bin : bins) {
		first.push_back(uint32_t(ids.size()));
		ids.insert(ids.end(), bin.begin(), bin.end());
	}
	first.push_back(uint32_t(ids.size()));
	// storage buffers may not be empty
	star_data.resize(std::max<size_t>(star_data.size(), 8), 0.0f);
	ids.resize(std::max<size_t>(ids.size(), 1), 0u);
	stars_buf_ = rd_->storage_buffer_create(uint32_t(star_data.size() * 4), bytes_of(star_data.data(), star_data.size() * 4));
	cells_buf_ = rd_->storage_buffer_create(uint32_t(first.size() * 4), bytes_of(first.data(), first.size() * 4));
	ids_buf_ = rd_->storage_buffer_create(uint32_t(ids.size() * 4), bytes_of(ids.data(), ids.size() * 4));

	Ref<Image> mw = ResourceLoader::get_singleton()->load("res://addons/regolith/regolith_milkyway.res");
	if (mw.is_valid() && mw->get_format() == Image::FORMAT_RGBE9995) {
		mw_unit_ = double(mw->get_meta("unit", 1.0));
		mw_tex_ = make_texture(rd_, RD::DATA_FORMAT_E5B9G9R9_UFLOAT_PACK32, mw->get_width(), mw->get_height(), 1,
				mw->get_data());
	} else {
		const uint32_t black = 0;
		mw_tex_ = make_texture(rd_, RD::DATA_FORMAT_E5B9G9R9_UFLOAT_PACK32, 1, 1, 1, bytes_of(&black, 4));
	}
	mw_sampler_ = make_sampler(rd_, true, RD::SAMPLER_REPEAT_MODE_REPEAT, RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE);
}

void RegolithAirEffect::_render_callback(int32_t p_type, RenderData *p_data) {
	if (p_type != EFFECT_CALLBACK_TYPE_POST_SKY || !p_data) {
		return;
	}
	RD *rd = RenderingServer::get_singleton()->get_rendering_device();
	Ref<RenderSceneBuffersRD> buffers = p_data->get_render_scene_buffers();
	RenderSceneData *scene = p_data->get_render_scene_data();
	if (!rd || buffers.is_null() || !scene || !ensure(rd)) {
		return;
	}
	Frame f;
	{
		std::lock_guard<std::mutex> guard(mutex_);
		f = frame_;
	}
	if (!f.valid) {
		return;
	}
	const Vector2i size = buffers->get_internal_size();
	if (size.x <= 0 || size.y <= 0) {
		return;
	}
	AirModel air = f.air;
	if (!f.has_air) {
		// no medium: only the ground sphere matters
		air = AirModel();
		air.bottom = f.air.bottom;
		air.datum = f.air.datum;
		air.top = f.air.bottom + 1.0;
		air.ground_albedo = f.air.ground_albedo;
		air.ozone_width = 1.0;
	}
	const bool rebuild = !built_ || !same_air(air, built_air_);
	const double r = regolith::length(f.eye);
	const double top = air.top, bot = air.bottom;
	const double inside = std::sqrt(std::max((top - r) * (top + r), 0.0));
	const double range = r < top ? inside + std::sqrt((top - bot) * (top + bot)) : r + top;
	const Projection proj = scene->get_cam_projection();
	const Basis cam = scene->get_cam_transform().basis;

	std::vector<float> u;
	u.reserve(128);
	push4(u, bot, top, r, f.has_air ? 1.0 : 0.0);
	push_rgb(u, air.rayleigh, air.rayleigh_height);
	push_rgb(u, air.mie_scattering, air.mie_height);
	push_rgb(u, air.mie_absorption, 0.0);
	push_rgb(u, air.mie_g, 0.0);
	push_rgb(u, air.ozone, air.ozone_center);
	const double datum = air.datum > 0.0 ? air.datum : bot;
	// the tables' base: fixed while they stand, under all ground
	const double base = rebuild ? datum - TABLE_DEPTH : built_base_;
	push4(u, air.ozone_width, f.sun_radius, datum, base);
	push_rgb(u, air.ground_albedo, 0.0);
	push3(u, f.eye, range);
	push3(u, f.sun, f.sun_energy);
	for (int k = 0; k < 3; ++k) {
		push3(u, f.to_body[k], 0.0);
	}
	for (int k = 0; k < 3; ++k) {
		push3(u, cam.get_column(k), 0.0);
	}
	push_mat(u, proj.inverse());
	push4(u, size.x, size.y, f.exposure, f.airglow);
	for (int k = 0; k < 3; ++k) {
		push3(u, f.body_to_icrf[k], 0.0);
	}
	// one pixel's angle: the stars' point spread
	const double pixel_angle = 2.0 / (std::fabs(double(proj.columns[1].y)) * size.y);
	push4(u, pixel_angle, f.star_scale, 0.0, mw_unit_);
	push4(u, f.glow_color.r, f.glow_color.g, f.glow_color.b, 0.0);
	rd_->buffer_update(params_, 0, uint32_t(u.size() * 4), bytes_of(u.data(), u.size() * 4));

	const Ref<RDUniform> ubo = uniform(RD::UNIFORM_TYPE_UNIFORM_BUFFER, 0, params_);
	const RID depth = buffers->get_depth_layer(0);
	const int64_t list = rd_->compute_list_begin();
	auto pass = [&](RID p_pipe, RID p_shader, const std::vector<Ref<RDUniform>> &p_u, uint32_t p_x, uint32_t p_y,
						uint32_t p_z) {
		std::vector<Ref<RDUniform>> all = { ubo };
		all.insert(all.end(), p_u.begin(), p_u.end());
		rd_->compute_list_bind_compute_pipeline(list, p_pipe);
		rd_->compute_list_bind_uniform_set(list, set_of(p_shader, all), 0);
		rd_->compute_list_dispatch(list, p_x, p_y, p_z);
		rd_->compute_list_add_barrier(list);
	};
	if (rebuild) {
		pass(t_pipe_, t_shader_, { image(1, t_lut_) }, groups(T_W, 8), groups(T_H, 8), 1);
		pass(ms_pipe_, ms_shader_, { image(1, ms_lut_), sampled(2, lin_clamp_, t_lut_) }, MS_N, MS_N, 1);
		pass(e_pipe_, e_shader_, { image(1, e_lut_), sampled(2, lin_clamp_, t_lut_), sampled(3, lin_clamp_, ms_lut_) },
				E_W, E_H, 1);
		{
			std::lock_guard<std::mutex> guard(mutex_);
			built_air_ = air;
			built_base_ = base;
			tables_ready_ = true;
		}
		built_ = true;
	}
	if (f.has_air) {
		pass(sv_pipe_, sv_shader_, { image(1, sv_lut_), sampled(2, lin_clamp_, t_lut_), sampled(3, lin_clamp_, ms_lut_) },
				groups(SV_W, 8), groups(SV_H, 8), 1);
		pass(ap_pipe_, ap_shader_, { image(1, ap_lut_), sampled(2, lin_clamp_, t_lut_), sampled(3, lin_clamp_, ms_lut_) },
				groups(AP_N, 8), groups(AP_N, 8), 1);
	}
	pass(comp_pipe_, comp_shader_,
			{ image(1, buffers->get_color_layer(0)), sampled(2, nearest_, depth), sampled(3, lin_clamp_, t_lut_),
					sampled(4, lin_clamp_, ms_lut_), sampled(5, lin_clamp_, sv_lut_), sampled(6, lin_clamp_, ap_lut_),
					uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 7, stars_buf_),
					uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 8, cells_buf_),
					uniform(RD::UNIFORM_TYPE_STORAGE_BUFFER, 9, ids_buf_), sampled(10, mw_sampler_, mw_tex_) },
			groups(size.x, 8), groups(size.y, 8), 1);
	pass(amb_pipe_, amb_shader_,
			{ image(1, amb_tex_), sampled(2, lin_clamp_, t_lut_), sampled(3, lin_clamp_, sv_lut_),
					sampled(4, lin_clamp_, buffers->get_color_layer(0)), sampled(5, nearest_, depth) },
			1, 1, 1);
	rd_->compute_list_end();
	// REGOLITH_AIR_DUMP: count bad texels per table, once
	static int dump_frame = 0;
	if (!OS::get_singleton()->get_environment("REGOLITH_AIR_DUMP").is_empty() && ++dump_frame == 30) {
		auto half = [](uint16_t p_h) {
			const uint32_t e = (p_h >> 10) & 31, m = p_h & 1023;
			return e == 31 ? (m ? NAN : INFINITY) : std::ldexp(double(e ? m | 1024 : m), int(e ? e : 1) - 25);
		};
		const char *names[5] = { "transmittance", "multiscatter", "skyview", "aerial", "irradiance" };
		RID texs[5] = { t_lut_, ms_lut_, sv_lut_, ap_lut_, e_lut_ };
		for (int k = 0; k < 5; ++k) {
			const PackedByteArray b = rd_->texture_get_data(texs[k], 0);
			const uint16_t *h = reinterpret_cast<const uint16_t *>(b.ptr());
			int64_t bad = 0, n = b.size() / 2;
			double hi = 0.0;
			for (int64_t i = 0; i < n; ++i) {
				const double v = half(h[i]);
				if (!std::isfinite(v)) {
					++bad;
				} else {
					hi = std::max(hi, v);
				}
			}
			UtilityFunctions::print("AIRDUMP ", names[k], ": ", bad, " bad of ", n, ", max ", hi, ", base ", base,
					", datum ", datum, ", bottom ", bot, ", top ", top);
		}
	}
	// last readback, landed in the sink since: take it
	if (reading_ && sink_->get_size() > 0) {
		on_ambient(sink_->get_data_array());
		sink_->clear();
		reading_ = false;
	}
	if (!reading_.exchange(true)) {
		if (rd_->texture_get_data_async(amb_tex_, 0, Callable(sink_.ptr(), "set_data_array")) != OK) {
			reading_ = false;
		}
	}
}

// The node.

RegolithAtmosphere::RegolithAtmosphere() {
	// after the system has placed the sun
	set_process_priority(CAMERA_PASSES_PRIORITY + 10);
	set_air("Mars");
}

void RegolithAtmosphere::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_system_path", "path"), &RegolithAtmosphere::set_system_path);
	ClassDB::bind_method(D_METHOD("get_system_path"), &RegolithAtmosphere::get_system_path);
	ClassDB::bind_method(D_METHOD("set_body_path", "path"), &RegolithAtmosphere::set_body_path);
	ClassDB::bind_method(D_METHOD("get_body_path"), &RegolithAtmosphere::get_body_path);
	ClassDB::bind_method(D_METHOD("set_sun_path", "path"), &RegolithAtmosphere::set_sun_path);
	ClassDB::bind_method(D_METHOD("get_sun_path"), &RegolithAtmosphere::get_sun_path);
	ClassDB::bind_method(D_METHOD("set_air", "preset"), &RegolithAtmosphere::set_air);
	ClassDB::bind_method(D_METHOD("get_air"), &RegolithAtmosphere::get_air);
	ClassDB::bind_method(D_METHOD("set_dust_tau", "tau"), &RegolithAtmosphere::set_dust_tau);
	ClassDB::bind_method(D_METHOD("get_dust_tau"), &RegolithAtmosphere::get_dust_tau);
	ClassDB::bind_method(D_METHOD("set_dust_scale", "scale"), &RegolithAtmosphere::set_dust_scale);
	ClassDB::bind_method(D_METHOD("get_dust_scale"), &RegolithAtmosphere::get_dust_scale);
	ClassDB::bind_method(D_METHOD("set_sun_energy", "energy"), &RegolithAtmosphere::set_sun_energy);
	ClassDB::bind_method(D_METHOD("get_sun_energy"), &RegolithAtmosphere::get_sun_energy);
	ClassDB::bind_method(D_METHOD("set_exposure", "exposure"), &RegolithAtmosphere::set_exposure);
	ClassDB::bind_method(D_METHOD("get_exposure"), &RegolithAtmosphere::get_exposure);
	ClassDB::bind_method(D_METHOD("set_airglow", "radiance"), &RegolithAtmosphere::set_airglow);
	ClassDB::bind_method(D_METHOD("get_airglow"), &RegolithAtmosphere::get_airglow);
	ClassDB::bind_method(D_METHOD("set_air_value", "index", "value"), &RegolithAtmosphere::set_air_value);
	ClassDB::bind_method(D_METHOD("get_air_value", "index"), &RegolithAtmosphere::get_air_value);
	ClassDB::bind_method(D_METHOD("set_fog_density", "density"), &RegolithAtmosphere::set_fog_density);
	ClassDB::bind_method(D_METHOD("get_fog_density"), &RegolithAtmosphere::get_fog_density);
	ClassDB::bind_method(D_METHOD("get_sky_thin"), &RegolithAtmosphere::get_sky_thin);
	ClassDB::bind_method(D_METHOD("get_terrain_sun"), &RegolithAtmosphere::get_terrain_sun);
	ClassDB::bind_method(D_METHOD("get_sun_color"), &RegolithAtmosphere::get_sun_color);
	ClassDB::bind_method(D_METHOD("get_haze"), &RegolithAtmosphere::get_haze);
	ClassDB::bind_method(D_METHOD("get_sky_direction", "name", "body_frame"), &RegolithAtmosphere::get_sky_direction,
			DEFVAL(false));
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "system"), "set_system_path", "get_system_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "body"), "set_body_path", "get_body_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "sun"), "set_sun_path", "get_sun_path");
	// the preset first: values after it in a scene override it
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "air", PROPERTY_HINT_ENUM, "None,Mars,Earth,Custom"), "set_air", "get_air");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dust_tau"), "set_dust_tau", "get_dust_tau");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "dust_scale", PROPERTY_HINT_RANGE, "0,2,0.01,or_greater"), "set_dust_scale",
			"get_dust_scale");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "sun_energy"), "set_sun_energy", "get_sun_energy");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "exposure"), "set_exposure", "get_exposure");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "airglow"), "set_airglow", "get_airglow");
	ClassDB::bind_method(D_METHOD("set_airglow_color", "color"), &RegolithAtmosphere::set_airglow_color);
	ClassDB::bind_method(D_METHOD("get_airglow_color"), &RegolithAtmosphere::get_airglow_color);
	ADD_PROPERTY(PropertyInfo(Variant::COLOR, "airglow_color"), "set_airglow_color", "get_airglow_color");
	ClassDB::bind_method(D_METHOD("set_night_vision", "factor"), &RegolithAtmosphere::set_night_vision);
	ClassDB::bind_method(D_METHOD("get_night_vision"), &RegolithAtmosphere::get_night_vision);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "night_vision", PROPERTY_HINT_RANGE, "1,10000000,1,exp"), "set_night_vision",
			"get_night_vision");
	ClassDB::bind_method(D_METHOD("get_eye_gain"), &RegolithAtmosphere::get_eye_gain);
	ClassDB::bind_method(D_METHOD("get_view_luminance"), &RegolithAtmosphere::get_view_luminance);
	ADD_GROUP("Air (per km, heights in km)", "air_");
	struct P {
		const char *name;
		Variant::Type type;
	};
	const P props[11] = { { "air_height", Variant::FLOAT }, { "air_rayleigh", Variant::VECTOR3 },
		{ "air_rayleigh_height", Variant::FLOAT }, { "air_mie_scattering", Variant::VECTOR3 },
		{ "air_mie_absorption", Variant::VECTOR3 }, { "air_mie_height", Variant::FLOAT },
		{ "air_mie_g", Variant::VECTOR3 }, { "air_ozone", Variant::VECTOR3 }, { "air_ozone_center", Variant::FLOAT },
		{ "air_ozone_width", Variant::FLOAT }, { "air_ground_albedo", Variant::COLOR } };
	for (int k = 0; k < 11; ++k) {
		ADD_PROPERTYI(PropertyInfo(props[k].type, props[k].name), "set_air_value", "get_air_value", k);
	}
	ADD_GROUP("", "");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "fog_density", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_NO_EDITOR),
			"set_fog_density", "get_fog_density");
}

Vector3 RegolithAtmosphere::get_sky_direction(const String &p_name, bool p_body_frame) const {
	RegolithSystem *sys = Object::cast_to<RegolithSystem>(get_node_or_null(system_path_));
	RegolithBody *body = Object::cast_to<RegolithBody>(get_node_or_null(body_path_));
	if (!sys || !body) {
		return Vector3();
	}
	DVec3 icrf;
	bool found = false;
	if (p_name == "Galactic Center") {
		// Sagittarius A*: RA 266.405, Dec -29.008 (ICRF)
		const double ra = 266.405 * std::numbers::pi / 180.0, dec = -29.008 * std::numbers::pi / 180.0;
		icrf = DVec3(std::cos(dec) * std::cos(ra), std::cos(dec) * std::sin(ra), std::sin(dec));
		found = true;
	} else if (p_name == "Milky Way") {
		// the galactic plane's point farthest from the sun: the
		// band's part best seen at night now
		const DVec3 n = regolith::galactic_pole();
		const DVec3 away = regolith::normalize(sys->central_heliocentric(sys->get_time()));
		icrf = regolith::normalize(away - n * regolith::dot(away, n));
		found = true;
	}
	if (!found) {
		return Vector3();
	}
	if (p_body_frame) {
		if (!sys->has_body(body)) {
			return Vector3();
		}
		const RegolithSystem::Pose &bp = sys->pose_of(body);
		return regolith::to_v(DVec3(regolith::dot(icrf, bp.axes[0]), regolith::dot(icrf, bp.axes[1]), regolith::dot(icrf, bp.axes[2])));
	}
	return sys->icrf_to_render().xform(regolith::to_v(icrf)).normalized();
}

void RegolithAtmosphere::set_dust_scale(double p_v) {
	if (!std::isfinite(p_v) || p_v < 0.0) {
		ERR_PRINT_ONCE("RegolithAtmosphere: dust_scale must be finite and not negative; kept " + String::num(dust_scale_));
		return;
	}
	dust_scale_ = p_v;
}

void RegolithAtmosphere::set_air(const String &p_v) {
	air_ = p_v;
	if (p_v == "Mars") {
		custom_ = regolith::mars_air(0.0, dust_tau_ * dust_scale_);
	} else if (p_v == "Earth") {
		custom_ = regolith::earth_air(0.0);
	}
	notify_property_list_changed();
}

// km in the editor, metres inside; heights over the ground
void RegolithAtmosphere::set_air_value(int p_index, const Variant &p_v) {
	AirModel &a = custom_;
	switch (p_index) {
		case 0: a.top = double(p_v) * 1000.0; break;
		case 1: a.rayleigh = from_v3(p_v, 1e-3); break;
		case 2: a.rayleigh_height = double(p_v) * 1000.0; break;
		case 3: a.mie_scattering = from_v3(p_v, 1e-3); break;
		case 4: a.mie_absorption = from_v3(p_v, 1e-3); break;
		case 5: a.mie_height = double(p_v) * 1000.0; break;
		case 6: a.mie_g = from_v3(p_v, 1.0); break;
		case 7: a.ozone = from_v3(p_v, 1e-3); break;
		case 8: a.ozone_center = double(p_v) * 1000.0; break;
		case 9: a.ozone_width = std::max(double(p_v), 0.001) * 1000.0; break;
		case 10: {
			const Color c = p_v;
			a.ground_albedo = { c.r, c.g, c.b };
		} break;
		default: break;
	}
}

Variant RegolithAtmosphere::get_air_value(int p_index) const {
	const AirModel &a = custom_;
	switch (p_index) {
		case 0: return a.top / 1000.0;
		case 1: return to_v3(a.rayleigh, 1e3);
		case 2: return a.rayleigh_height / 1000.0;
		case 3: return to_v3(a.mie_scattering, 1e3);
		case 4: return to_v3(a.mie_absorption, 1e3);
		case 5: return a.mie_height / 1000.0;
		case 6: return to_v3(a.mie_g, 1.0);
		case 7: return to_v3(a.ozone, 1e3);
		case 8: return a.ozone_center / 1000.0;
		case 9: return a.ozone_width / 1000.0;
		case 10: return Color(a.ground_albedo[0], a.ground_albedo[1], a.ground_albedo[2]);
		default: return Variant();
	}
}

AirModel RegolithAtmosphere::model(double p_ground, double p_datum) const {
	AirModel a = custom_;
	if (air_ == "Mars") {
		// the dust follows the weather; the rest stays as set
		const AirModel d = regolith::mars_air(0.0, dust_tau_ * dust_scale_);
		a.mie_scattering = d.mie_scattering;
		a.mie_absorption = d.mie_absorption;
	}
	a.bottom = p_ground;
	a.datum = p_datum;
	a.top = std::max(p_datum, p_ground) + std::max(custom_.top, 1000.0);
	return a;
}

void RegolithAtmosphere::_ready() {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	effect_.instantiate();
	Ref<World3D> world = get_viewport() ? get_viewport()->get_world_3d() : Ref<World3D>();
	Ref<Environment> env = world.is_valid() ? world->get_environment() : Ref<Environment>();
	if (env.is_valid()) {
		// the effect draws the sky; ambient comes from it below
		env->set_background(Environment::BG_COLOR);
		env->set_bg_color(Color(0, 0, 0));
		env->set_ambient_source(Environment::AMBIENT_SOURCE_COLOR);
		env->set_ambient_light_energy(1.0);
		env->set_reflection_source(Environment::REFLECTION_SOURCE_DISABLED);
		env->set_fog_enabled(false);
	}
}

void RegolithAtmosphere::attach(Camera3D *p_cam) {
	if (p_cam->get_instance_id() == attached_id_) {
		return;
	}
	detach();
	Ref<Compositor> c = p_cam->get_compositor();
	if (c.is_null()) {
		c.instantiate();
		p_cam->set_compositor(c);
	}
	TypedArray<Ref<CompositorEffect>> fx = c->get_compositor_effects();
	fx.push_back(effect_);
	c->set_compositor_effects(fx);
	attached_id_ = p_cam->get_instance_id();
}

void RegolithAtmosphere::detach() {
	// by id: the camera may have been freed since attach
	Camera3D *cam = Object::cast_to<Camera3D>(ObjectDB::get_instance(attached_id_));
	attached_id_ = 0;
	if (!cam) {
		return;
	}
	Ref<Compositor> c = cam->get_compositor();
	if (c.is_valid()) {
		TypedArray<Ref<CompositorEffect>> fx = c->get_compositor_effects();
		fx.erase(effect_);
		c->set_compositor_effects(fx);
	}
}

void RegolithAtmosphere::_exit_tree() {
	// the terrain lets go of the tables before the effect frees
	// them: wrappers left on freed textures crash at quit
	if (RegolithBody *body = Object::cast_to<RegolithBody>(get_node_or_null(body_path_))) {
		body->set_material_param("air_on", false);
		body->set_material_param("air_t", Variant());
		body->set_material_param("air_e", Variant());
	}
	for (Ref<Texture2DRD> *t : { &air_t_tex_, &air_e_tex_ }) {
		if (t->is_valid()) {
			(*t)->set_texture_rd_rid(RID());
			t->unref();
		}
	}
	detach();
}

// The highest terrain along the sun's azimuth, 16 km to about
// 600 km out: how much of the sun clears it.
void RegolithAtmosphere::occlusion(RegolithSystem *p_sys, RegolithBody *p_body, const DVec3 &p_eye,
		const DVec3 &p_sun, double &r_vis) const {
	r_vis = 1.0;
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
			max_ang = std::max(max_ang, std::asin(std::clamp(regolith::dot(regolith::normalize(v), up), -1.0, 1.0)));
		}
		d *= 1.046;
	}
	r_vis = std::clamp((std::asin(std::clamp(se, -1.0, 1.0)) - max_ang) / 0.012 + 0.5, 0.0, 1.0);
}

void RegolithAtmosphere::_process(double p_delta) {
	if (effect_.is_null()) {
		return;
	}
	RegolithSystem *sys = Object::cast_to<RegolithSystem>(get_node_or_null(system_path_));
	RegolithBody *body = Object::cast_to<RegolithBody>(get_node_or_null(body_path_));
	Camera3D *cam = get_viewport() ? get_viewport()->get_camera_3d() : nullptr;
	if (!sys || !body || !cam || !sys->is_anchored()) {
		return;
	}
	if (!sys->has_body(body)) {
		// its pack failed to open, or it is not a body of this system
		WARN_PRINT_ONCE("RegolithAtmosphere: its body is not part of the RegolithSystem; the air is off");
		return;
	}
	attach(cam);
	const double t = sys->get_time();
	const RegolithSystem::Pose &bp = sys->pose_of(body);
	const DVec3 eye = sys->body_point(body, cam->get_global_position());
	const double r = regolith::length(eye);
	// the sun from the camera, in the body's axes: right from
	// any planet, the same as from the body's centre near it
	const DVec3 to_sun = regolith::normalize(sys->camera_heliocentric() * -1.0);
	const DVec3 *ax = bp.axes;
	const DVec3 sun(regolith::dot(to_sun, ax[0]), regolith::dot(to_sun, ax[1]), regolith::dot(to_sun, ax[2]));
	// the sky meets the terrain under the camera, which may lie
	// km under the datum; the air's density follows the datum
	const DVec3 down = regolith::normalize(eye);
	const double datum = body->datum_radius(down);
	const double surface = body->surface_radius(down, false);
	// the air's floor under every bit of ground, so no terrain
	// dips below it and no sky ray ends on a fake sphere
	const double lowest = body->lowest_radius();
	const double floor_r = lowest > 0.0 ? lowest - 10.0 : (std::isnan(surface) ? datum : surface - 10.0);
	const double bottom = std::min(floor_r, r - 1.0);
	const AirModel air = model(bottom, datum);
	const bool has_air = air_ != "None";

	RegolithAirEffect::Frame f;
	f.air = air;
	f.has_air = has_air;
	const regolith::RenderFrame rf = sys->frame_for(body);
	f.to_body.set_column(0, regolith::to_v(rf.east));
	f.to_body.set_column(1, regolith::to_v(rf.up));
	f.to_body.set_column(2, regolith::to_v(rf.south));
	f.eye = eye;
	f.sun = regolith::to_v(sun);
	f.sun_energy = sun_energy_;
	const double dist = regolith::length(sys->central_heliocentric(t) * AU + bp.position);
	f.sun_radius = SUN_RADIUS / std::max(dist, 1e9);
	f.exposure = exposure_;
	f.airglow = has_air ? airglow_ : 0.0;
	{
		const Color c = airglow_color_;
		const float lum = std::max(0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b, 1e-4f);
		f.glow_color = Color(c.r / lum, c.g / lum, c.b / lum);
	}
	// the sun_energy is at this distance; starlight is not
	const double d_au = dist / AU;
	// the eye adapts to the view's brightness over a few seconds,
	// in log space; a daylit view keeps the stars true (gain 1)
	// a bad reading must never reach the eye: it would blank the sky
	const double lum = effect_->view_luminance();
	const double scene = std::isfinite(lum) ? std::max(lum, 1e-12) : 1e-12;
	if (!std::isfinite(eye_gain_)) {
		eye_gain_ = 1.0;
	}
	const double want = std::clamp(VIEW_KEY / scene, 1.0, std::max(night_vision_, 1.0));
	eye_gain_ = std::exp(std::log(eye_gain_) + (std::log(want) - std::log(eye_gain_)) * (1.0 - std::exp(-p_delta / 2.0)));
	const double base = std::numbers::pi * sun_energy_ * d_au * d_au * exposure_;
	f.star_scale = base * eye_gain_;
	for (int k = 0; k < 3; ++k) {
		f.body_to_icrf.set_column(k, regolith::to_v(bp.axes[k]));
	}
	f.valid = true;
	effect_->set_frame(f);

	const double alt = std::max(r - bottom, 0.0);
	sky_thin_ = has_air ? std::exp(-alt / std::max(air.rayleigh_height, 1.0)) : 0.0;
	body->set_material_param("dust_tau", has_air ? dust_tau_ * dust_scale_ : 0.0);

	// terrain hiding a low sun, every 0.25 s near the ground
	occl_cool_ -= p_delta;
	if (occl_cool_ <= 0.0) {
		occl_cool_ = 0.25;
		occl_target_ = 1.0;
		if (alt < 15000.0) {
			occlusion(sys, body, eye, sun, occl_target_);
		}
	}
	sun_vis_ += (occl_target_ - sun_vis_) * (1.0 - std::exp(-p_delta * 3.0));

	// the sun through the air to the camera
	const double mu_s = regolith::dot(regolith::normalize(eye), sun);
	RGB tr = { 1.0, 1.0, 1.0 };
	if (has_air) {
		tr = regolith::air_transmittance(air, std::max(r, bottom + 1.0), mu_s);
	} else if (mu_s < -std::sqrt(std::max(1.0 - (bottom / r) * (bottom / r), 0.0))) {
		tr = { 0.0, 0.0, 0.0 }; // behind the body
	}
	// the terrain lights itself per point from the same tables:
	// the night side stays dark whatever the camera sees
	RID t_rid, e_rid;
	double t_base = 0.0, t_top = 0.0, t_datum = 0.0;
	if (effect_->tables(t_rid, e_rid, t_base, t_top, t_datum)) {
		if (air_t_tex_.is_null()) {
			air_t_tex_.instantiate();
			air_t_tex_->set_texture_rd_rid(t_rid);
			air_e_tex_.instantiate();
			air_e_tex_->set_texture_rd_rid(e_rid);
		}
		body->set_material_param("air_on", true);
		body->set_material_param("air_t", air_t_tex_);
		body->set_material_param("air_e", air_e_tex_);
		body->set_material_param("air_shape", Vector4(real_t(t_base), real_t(t_top), real_t(t_datum), real_t(body->r_ref())));
		body->set_material_param("sun_top", real_t(sun_energy_ * exposure_ * sys->get_sun_visible()));
		// airglow overhead lights the night ground: pi L, in suns at
		// 1 AU, at the eye's gain; no air, no glow
		const real_t glow = has_air ? real_t(f.star_scale * airglow_) : real_t(0.0);
		body->set_material_param("night_glow", Vector3(f.glow_color.r, f.glow_color.g, f.glow_color.b) * glow);
	}
	const double peak = std::max({ tr[0], tr[1], tr[2], 1e-9 });
	sun_color_ = Color(tr[0] / peak, tr[1] / peak, tr[2] / peak);
	const double energy = sun_energy_ * peak * sun_vis_ * sys->get_sun_visible() * sys->get_sun_scale();
	if (DirectionalLight3D *light = Object::cast_to<DirectionalLight3D>(get_node_or_null(sun_path_))) {
		light->set_param(Light3D::PARAM_ENERGY, real_t(energy));
		light->set_color(sun_color_);
		light->set_shadow(energy > 1e-4);
	}

	// ambient: the sky and the sunlit ground, from the GPU
	Ref<Environment> env = get_viewport()->get_world_3d()->get_environment();
	const Vector3 mean = effect_->ambient_mean() * real_t(sun_energy_ * exposure_);
	if (env.is_valid()) {
		env->set_ambient_light_color(Color(mean.x, mean.y, mean.z));
	}
	const Vector3 up_e = effect_->ambient_up() * real_t(sun_energy_ * exposure_);
	double ext = 0.0;
	if (has_air) {
		const double h = r - datum;
		const double dr = std::exp(-h / air.rayleigh_height), dm = std::exp(-h / air.mie_height);
		for (int c = 0; c < 3; ++c) {
			ext += (air.rayleigh[size_t(c)] * dr + (air.mie_scattering[size_t(c)] + air.mie_absorption[size_t(c)]) * dm) / 3.0;
		}
	}
	haze_ = Color(up_e.x, up_e.y, up_e.z, real_t(ext));
}

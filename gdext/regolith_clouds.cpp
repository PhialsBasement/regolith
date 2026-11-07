#include "regolith_clouds.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/compositor.hpp>
#include <godot_cpp/classes/directional_light3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/environment.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/rd_sampler_state.hpp>
#include <godot_cpp/classes/rd_shader_file.hpp>
#include <godot_cpp/classes/rd_shader_source.hpp>
#include <godot_cpp/classes/rd_shader_spirv.hpp>
#include <godot_cpp/classes/rd_texture_format.hpp>
#include <godot_cpp/classes/rd_texture_view.hpp>
#include <godot_cpp/classes/rd_uniform.hpp>
#include <godot_cpp/classes/render_scene_buffers_rd.hpp>
#include <godot_cpp/classes/render_scene_data.hpp>
#include <godot_cpp/classes/rendering_device.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/uniform_set_cache_rd.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include "convert.hpp"
#include "rd_util.hpp"
#include "regolith/climate.hpp"
#include "regolith_atmosphere.hpp"
#include "regolith_body.hpp"
#include "regolith_system.hpp"
#include "regolith_weather.hpp"

using namespace godot;
using namespace godot::rdu;
using regolith::DVec3;

namespace {

constexpr int LIGHT_XY = 96, LIGHT_Z = 32;
constexpr int SHAPE_N = 128, DETAIL_N = 32;
constexpr int SHAPE_MIPS = 8, DETAIL_MIPS = 6; // down to 1
constexpr double Z_MIN = -8000.0, Z_MAX = 48000.0;
// light cascade half widths (m) and re-bake intervals (s)
constexpr double HALF[RegolithCloudEffect::NC] = { 24e3, 96e3, 384e3, 1536e3 };
constexpr double INTERVAL[RegolithCloudEffect::NC] = { 3.0, 8.0, 20.0, 60.0 };
double steady_now() {
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace

RegolithCloudEffect::RegolithCloudEffect() {
	set_effect_callback_type(EFFECT_CALLBACK_TYPE_POST_TRANSPARENT);
}

RegolithCloudEffect::~RegolithCloudEffect() {
	free_all();
}

void RegolithCloudEffect::free_all() {
	if (!rd_) {
		return;
	}
	for (std::vector<RID> *v : { &shape_mips_, &detail_mips_ }) {
		for (RID &m : *v) {
			rd_->free_rid(m);
		}
		v->clear();
	}
	std::vector<RID *> all = { &noise_pipe_, &light_pipe_, &march_pipe_, &comp_pipe_, &mip_pipe_, &resolve_pipe_,
		&noise_shader_, &light_shader_, &march_shader_, &comp_shader_, &mip_shader_, &resolve_shader_, &lin_clamp_,
		&nearest_, &lin_repeat_, &field_sampler_, &params_, &shape_tex_, &detail_tex_, &tau_tex_, &topo_tex_,
		&band_tex_, &cloud_tex_, &cloud_prev_, &cloud_dist_, &trace_tex_, &trace_depth_ };
	for (RID &l : light_) {
		all.push_back(&l);
	}
	for (RID *r : all) {
		if (r->is_valid()) {
			rd_->free_rid(*r);
			*r = RID();
		}
	}
	rd_ = nullptr;
}

void RegolithCloudEffect::set_frame(const Frame &p_frame) {
	std::lock_guard<std::mutex> guard(mutex_);
	frame_ = p_frame;
}

void RegolithCloudEffect::set_look(const Look &p_look) {
	std::lock_guard<std::mutex> guard(mutex_);
	look_ = p_look;
}

void RegolithCloudEffect::set_field(Field &&p_field) {
	std::lock_guard<std::mutex> guard(mutex_);
	field_ = std::move(p_field);
	field_dirty_ = true;
}

std::vector<RID> RegolithCloudEffect::mip_views(RID p_tex, int p_mips) {
	std::vector<RID> out;
	for (int m = 0; m < p_mips; ++m) {
		Ref<RDTextureView> v;
		v.instantiate();
		out.push_back(rd_->texture_create_shared_from_slice(v, p_tex, 0, uint32_t(m), 1, RD::TEXTURE_SLICE_3D));
	}
	return out;
}

bool RegolithCloudEffect::ensure(RD *p_rd) {
	if (failed_) {
		return false;
	}
	if (march_pipe_.is_valid()) {
		return true;
	}
	rd_ = p_rd;
	RID *shaders[] = { &noise_shader_, &light_shader_, &march_shader_, &comp_shader_, &mip_shader_, &resolve_shader_ };
	RID *pipes[] = { &noise_pipe_, &light_pipe_, &march_pipe_, &comp_pipe_, &mip_pipe_, &resolve_pipe_ };
	const char *names[] = { "regolith_cloud_noise", "regolith_cloud_light", "regolith_cloud_march",
		"regolith_cloud_composite", "regolith_cloud_mip", "regolith_cloud_resolve" };
	for (int i = 0; i < 6; ++i) {
		*shaders[i] = compile(rd_, names[i]);
		if (!shaders[i]->is_valid()) {
			failed_ = true;
			return false;
		}
		*pipes[i] = rd_->compute_pipeline_create(*shaders[i]);
	}
	lin_clamp_ = make_sampler(rd_, true, RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE, RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE);
	nearest_ = make_sampler(rd_, false, RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE, RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE);
	lin_repeat_ = make_sampler(rd_, true, RD::SAMPLER_REPEAT_MODE_REPEAT, RD::SAMPLER_REPEAT_MODE_REPEAT);
	// longitude wraps; latitude and level clamp
	field_sampler_ = make_sampler(rd_, true, RD::SAMPLER_REPEAT_MODE_REPEAT, RD::SAMPLER_REPEAT_MODE_CLAMP_TO_EDGE);
	params_ = rd_->uniform_buffer_create(1024);
	shape_tex_ = make_storage(rd_, RD::DATA_FORMAT_R8G8B8A8_UNORM, SHAPE_N, SHAPE_N, SHAPE_N, SHAPE_MIPS);
	detail_tex_ = make_storage(rd_, RD::DATA_FORMAT_R8G8B8A8_UNORM, DETAIL_N, DETAIL_N, DETAIL_N, DETAIL_MIPS);
	shape_mips_ = mip_views(shape_tex_, SHAPE_MIPS);
	detail_mips_ = mip_views(detail_tex_, DETAIL_MIPS);
	for (RID &l : light_) {
		l = make_storage(rd_, RD::DATA_FORMAT_R16G16_SFLOAT, LIGHT_XY, LIGHT_XY, LIGHT_Z);
	}
	return true;
}

// The next light cascade to bake: missing ones coarse first,
// then any the eye has left (fine first), then the most overdue.
int RegolithCloudEffect::pick_job(const Frame &p_f, double p_now) const {
	const DVec3 dir = regolith::normalize(p_f.eye);
	for (int k = NC - 1; k >= 0; --k) {
		if (!casc_[size_t(k)].built) {
			return k;
		}
	}
	for (int k = 0; k < NC; ++k) {
		const Cascade &c = casc_[size_t(k)];
		const double ca = regolith::dot(dir, c.a);
		if (ca < 0.7) {
			return k;
		}
		const double gx = p_f.datum * regolith::dot(dir, c.e) / ca;
		const double gy = p_f.datum * regolith::dot(dir, c.n) / ca;
		if (std::max(std::fabs(gx), std::fabs(gy)) > 0.25 * HALF[k]) {
			return k;
		}
	}
	int best = -1;
	double worst = 1.0;
	for (int k = 0; k < NC; ++k) {
		const double o = (p_now - casc_[size_t(k)].last) / INTERVAL[k];
		if (o > worst) {
			worst = o;
			best = k;
		}
	}
	return best;
}

void RegolithCloudEffect::anchor(int p_k, const DVec3 &p_dir) {
	Cascade &c = casc_[size_t(p_k)];
	c.a = p_dir;
	DVec3 e = regolith::cross(DVec3(0, 0, 1), p_dir);
	if (regolith::length(e) < 1e-6) {
		e = regolith::cross(DVec3(1, 0, 0), p_dir);
	}
	c.e = regolith::normalize(e);
	c.n = regolith::cross(p_dir, c.e);
}

void RegolithCloudEffect::_render_callback(int32_t p_type, RenderData *p_data) {
	if (p_type != EFFECT_CALLBACK_TYPE_POST_TRANSPARENT || !p_data) {
		return;
	}
	RD *rd = RenderingServer::get_singleton()->get_rendering_device();
	Ref<RenderSceneBuffersRD> buffers = p_data->get_render_scene_buffers();
	RenderSceneData *scene = p_data->get_render_scene_data();
	if (!rd || buffers.is_null() || !scene || !ensure(rd)) {
		return;
	}
	Frame f;
	Look look;
	{
		std::lock_guard<std::mutex> guard(mutex_);
		f = frame_;
		look = look_;
		if (field_dirty_ && field_.gw > 0) {
			const Field &fl = field_;
			const int nl = int(fl.tau.size()) / (fl.gw * fl.gh);
			const PackedByteArray t = bytes_of(fl.tau.data(), fl.tau.size() * 4);
			const PackedByteArray g = bytes_of(fl.topo.data(), fl.topo.size() * 4);
			const PackedByteArray b = bytes_of(fl.band.data(), fl.band.size() * 4);
			if (tau_tex_.is_valid()) {
				rd_->texture_update(tau_tex_, 0, t);
				rd_->texture_update(topo_tex_, 0, g);
				rd_->texture_update(band_tex_, 0, b);
			} else {
				tau_tex_ = make_texture(rd_, RD::DATA_FORMAT_R32_SFLOAT, fl.gw, fl.gh, nl, t);
				topo_tex_ = make_texture(rd_, RD::DATA_FORMAT_R32_SFLOAT, fl.gw, fl.gh, 1, g);
				band_tex_ = make_texture(rd_, RD::DATA_FORMAT_R32G32B32A32_SFLOAT, fl.gw, fl.gh, 1, b);
			}
			edges_ = fl.edges;
			field_dirty_ = false;
		}
	}
	if (!f.valid || !tau_tex_.is_valid() || edges_.size() < 9) {
		return;
	}
	const Vector2i size = buffers->get_internal_size();
	if (size.x <= 0 || size.y <= 0) {
		return;
	}
	const double scale = std::clamp(look.resolution_scale, 0.1, 1.0);
	const Vector2i csize(std::max(1, int(size.x * scale)), std::max(1, int(size.y * scale)));
	const Vector2i qsize((csize.x + 1) / 2, (csize.y + 1) / 2);
	if (csize != cloud_size_) {
		for (RID *r : { &cloud_tex_, &cloud_prev_, &cloud_dist_, &trace_tex_, &trace_depth_ }) {
			if (r->is_valid()) {
				rd_->free_rid(*r);
			}
		}
		cloud_tex_ = make_storage(rd_, RD::DATA_FORMAT_R16G16B16A16_SFLOAT, csize.x, csize.y, 1);
		cloud_prev_ = make_storage(rd_, RD::DATA_FORMAT_R16G16B16A16_SFLOAT, csize.x, csize.y, 1);
		cloud_dist_ = make_storage(rd_, RD::DATA_FORMAT_R32_SFLOAT, csize.x, csize.y, 1);
		trace_tex_ = make_storage(rd_, RD::DATA_FORMAT_R16G16B16A16_SFLOAT, qsize.x, qsize.y, 1);
		trace_depth_ = make_storage(rd_, RD::DATA_FORMAT_R32_SFLOAT, qsize.x, qsize.y, 1);
		cloud_size_ = csize;
		history_valid_ = false;
	}
	const int job_k = pick_job(f, steady_now());
	if (job_k >= 0) {
		Cascade &c = casc_[size_t(job_k)];
		anchor(job_k, regolith::normalize(f.eye));
		c.built = true;
		c.last = steady_now();
		c.drift = f.drift;
	}
	const Projection proj = scene->get_cam_projection();
	const Projection inv = proj.inverse();
	const Basis cam = scene->get_cam_transform().basis;

	// the parameter block, in the shaders' std140 order
	std::vector<float> u;
	u.reserve(256);
	for (int k = 0; k < NC; ++k) {
		const Cascade &c = casc_[size_t(k)];
		push3(u, c.a, HALF[k]);
		push3(u, c.e, 2.0 * HALF[k] / LIGHT_XY);
		push3(u, c.n, c.built ? 1.0 : 0.0);
		push4(u, Z_MIN, Z_MAX, 0.0, 0.0);
		push3(u, f.drift - c.drift, 0.0);
	}
	push4(u, f.datum, double(frame_index_++ % 1024), double(look.max_steps), double(std::max(job_k, 0)));
	push4(u, edges_[0], edges_[1], edges_[2], edges_[3]);
	push4(u, edges_[4], edges_[5], edges_[6], edges_[7]);
	push4(u, edges_[8], look.density, look.coverage_gain, look.shape_period);
	push3(u, f.drift, look.detail_period);
	push3(u, f.sun, f.sun_energy);
	push4(u, f.sun_color.r, f.sun_color.g, f.sun_color.b, look.phase_g);
	// one march pixel's angle: the footprint that filters detail
	const double pixel_angle = 2.0 / (std::fabs(double(proj.columns[1].y)) * csize.y);
	push4(u, f.ambient.r, f.ambient.g, f.ambient.b, pixel_angle);
	push4(u, f.fog.r, f.fog.g, f.fog.b, f.fog.a);
	push3(u, f.eye, regolith::length(f.eye));
	for (int r = 0; r < 3; ++r) {
		push3(u, f.to_body[r], 0.0);
	}
	for (int c = 0; c < 3; ++c) {
		push3(u, cam.get_column(c), 0.0);
	}
	push_mat(u, inv);
	push4(u, csize.x, csize.y, size.x, size.y);
	// last frame's march sits in cloud_prev_ after the swap
	std::swap(cloud_tex_, cloud_prev_);
	push_mat(u, prev_proj_);
	for (int r = 0; r < 3; ++r) {
		push3(u, prev_cam_.get_column(r), r == 2 && history_valid_ ? 1.0 : 0.0);
	}
	// the eye's move since last frame, body to render axes
	const Vector3 move = f.to_body.transposed().xform(regolith::to_v(f.eye - prev_eye_));
	push3(u, history_valid_ ? move : Vector3(), 0.0);
	// which pixel of each 2x2 block is traced, cycling all four
	constexpr int CELL[4][2] = { { 0, 0 }, { 1, 1 }, { 1, 0 }, { 0, 1 } };
	const int *cell = CELL[frame_index_ % 4];
	push4(u, qsize.x, qsize.y, cell[0], cell[1]);
	prev_proj_ = proj;
	prev_cam_ = cam;
	prev_eye_ = f.eye;
	history_valid_ = true;
	rd_->buffer_update(params_, 0, uint32_t(u.size() * 4), bytes_of(u.data(), u.size() * 4));

	const RID depth = buffers->get_depth_layer(0);
	// the cloud's own inputs, the same bindings in every pass
	const std::vector<Ref<RDUniform>> cloud_in = { uniform(RD::UNIFORM_TYPE_UNIFORM_BUFFER, 0, params_),
		sampled(20, field_sampler_, tau_tex_), sampled(21, field_sampler_, topo_tex_),
		sampled(22, lin_repeat_, shape_tex_), sampled(23, lin_repeat_, detail_tex_), sampled(24, nearest_, band_tex_) };
	const int64_t list = rd_->compute_list_begin();
	if (!noise_done_) {
		const RID s = set_of(noise_shader_, { image(0, shape_mips_[0]), image(1, detail_mips_[0]) });
		rd_->compute_list_bind_compute_pipeline(list, noise_pipe_);
		rd_->compute_list_bind_uniform_set(list, s, 0);
		rd_->compute_list_dispatch(list, groups(SHAPE_N, 4), groups(SHAPE_N, 4), groups(SHAPE_N, 4));
		rd_->compute_list_add_barrier(list);
		rd_->compute_list_bind_compute_pipeline(list, mip_pipe_);
		for (const std::vector<RID> *chain : { &shape_mips_, &detail_mips_ }) {
			const int top = chain == &shape_mips_ ? SHAPE_N : DETAIL_N;
			for (size_t m = 1; m < chain->size(); ++m) {
				const RID ms = set_of(mip_shader_, { image(0, (*chain)[m]), sampled(1, nearest_, (*chain)[m - 1]) });
				const int n = top >> m;
				rd_->compute_list_bind_uniform_set(list, ms, 0);
				rd_->compute_list_dispatch(list, groups(n, 4), groups(n, 4), groups(n, 4));
				rd_->compute_list_add_barrier(list);
			}
		}
		noise_done_ = true;
	}
	if (job_k >= 0) {
		std::vector<Ref<RDUniform>> lu = cloud_in;
		lu.push_back(image(1, light_[size_t(job_k)]));
		rd_->compute_list_bind_compute_pipeline(list, light_pipe_);
		rd_->compute_list_bind_uniform_set(list, set_of(light_shader_, lu), 0);
		rd_->compute_list_dispatch(list, groups(LIGHT_XY, 4), groups(LIGHT_XY, 4), groups(LIGHT_Z, 4));
		rd_->compute_list_add_barrier(list);
	}
	std::vector<Ref<RDUniform>> mu = cloud_in;
	mu.insert(mu.end(), { image(1, trace_tex_), image(2, trace_depth_), sampled(3, nearest_, depth) });
	for (int k = 0; k < NC; ++k) {
		mu.push_back(sampled(5 + k, lin_clamp_, light_[size_t(k)]));
	}
	rd_->compute_list_bind_compute_pipeline(list, march_pipe_);
	rd_->compute_list_bind_uniform_set(list, set_of(march_shader_, mu), 0);
	rd_->compute_list_dispatch(list, groups(qsize.x, 8), groups(qsize.y, 8), 1);
	rd_->compute_list_add_barrier(list);
	// the shared block declares the cloud inputs: bound here too
	std::vector<Ref<RDUniform>> ru = cloud_in;
	ru.insert(ru.end(), { image(1, cloud_tex_), image(2, cloud_dist_), sampled(3, nearest_, depth),
								 sampled(4, lin_clamp_, cloud_prev_), sampled(5, lin_clamp_, trace_tex_),
								 sampled(6, nearest_, trace_depth_) });
	rd_->compute_list_bind_compute_pipeline(list, resolve_pipe_);
	rd_->compute_list_bind_uniform_set(list, set_of(resolve_shader_, ru), 0);
	rd_->compute_list_dispatch(list, groups(csize.x, 8), groups(csize.y, 8), 1);
	rd_->compute_list_add_barrier(list);
	const RID cs = set_of(comp_shader_, { image(0, buffers->get_color_layer(0)), sampled(1, nearest_, depth),
												sampled(2, nearest_, cloud_tex_), sampled(3, nearest_, cloud_dist_) });
	std::vector<float> push;
	push_mat(push, inv);
	push4(push, size.x, size.y, csize.x, csize.y);
	rd_->compute_list_bind_compute_pipeline(list, comp_pipe_);
	rd_->compute_list_bind_uniform_set(list, cs, 0);
	rd_->compute_list_set_push_constant(list, bytes_of(push.data(), push.size() * 4), uint32_t(push.size() * 4));
	rd_->compute_list_dispatch(list, groups(size.x, 8), groups(size.y, 8), 1);
	rd_->compute_list_end();
}

void RegolithClouds::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_system_path", "path"), &RegolithClouds::set_system_path);
	ClassDB::bind_method(D_METHOD("get_system_path"), &RegolithClouds::get_system_path);
	ClassDB::bind_method(D_METHOD("set_body_path", "path"), &RegolithClouds::set_body_path);
	ClassDB::bind_method(D_METHOD("get_body_path"), &RegolithClouds::get_body_path);
	ClassDB::bind_method(D_METHOD("set_weather_path", "path"), &RegolithClouds::set_weather_path);
	ClassDB::bind_method(D_METHOD("get_weather_path"), &RegolithClouds::get_weather_path);
	ClassDB::bind_method(D_METHOD("set_sun_path", "path"), &RegolithClouds::set_sun_path);
	ClassDB::bind_method(D_METHOD("get_sun_path"), &RegolithClouds::get_sun_path);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "system_path"), "set_system_path", "get_system_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "body_path"), "set_body_path", "get_body_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "weather_path"), "set_weather_path", "get_weather_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "sun_path"), "set_sun_path", "get_sun_path");
	ClassDB::bind_method(D_METHOD("set_atmosphere_path", "path"), &RegolithClouds::set_atmosphere_path);
	ClassDB::bind_method(D_METHOD("get_atmosphere_path"), &RegolithClouds::get_atmosphere_path);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "atmosphere_path"), "set_atmosphere_path", "get_atmosphere_path");
#define CLOUD_PROP(name, type)                                                              \
	ClassDB::bind_method(D_METHOD("set_" #name, "value"), &RegolithClouds::set_##name);     \
	ClassDB::bind_method(D_METHOD("get_" #name), &RegolithClouds::get_##name);              \
	ADD_PROPERTY(PropertyInfo(Variant::type, #name), "set_" #name, "get_" #name);
	CLOUD_PROP(density, FLOAT)
	CLOUD_PROP(coverage_gain, FLOAT)
	CLOUD_PROP(shape_period, FLOAT)
	CLOUD_PROP(detail_period, FLOAT)
	CLOUD_PROP(phase_g, FLOAT)
	CLOUD_PROP(resolution_scale, FLOAT)
	CLOUD_PROP(max_steps, INT)
#undef CLOUD_PROP
}

void RegolithClouds::_ready() {
	if (Engine::get_singleton()->is_editor_hint()) {
		set_process(false);
		return;
	}
	effect_.instantiate();
	set_process(true);
}

void RegolithClouds::detach() {
	if (attached_ && effect_.is_valid()) {
		Ref<Compositor> c = attached_->get_compositor();
		if (c.is_valid()) {
			TypedArray<Ref<CompositorEffect>> fx = c->get_compositor_effects();
			fx.erase(effect_);
			c->set_compositor_effects(fx);
		}
	}
	attached_ = nullptr;
}

void RegolithClouds::_exit_tree() {
	detach();
}

void RegolithClouds::attach(Camera3D *p_cam) {
	if (p_cam == attached_) {
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
	attached_ = p_cam;
}

namespace {

// Per weather column, the band where cloud can be (m over the
// ground, half a level beyond the cloudy ones, since the
// density blends between level centres) and its peak
// extinction; then widened by a cell each way.
std::vector<float> cloud_band(const std::vector<float> &p_tau, int p_gw, int p_gh, const std::vector<float> &p_edges) {
	const int nl = int(p_edges.size()) - 1;
	const size_t n = size_t(p_gw) * p_gh;
	std::vector<float> raw(n * 4, 0.0f);
	for (size_t k = 0; k < n; ++k) {
		float lo = 1e9f, hi = -1e9f, peak = 0.0f;
		for (int l = 0; l < nl; ++l) {
			const float s = p_tau[size_t(l) * n + k] / (p_edges[size_t(l + 1)] - p_edges[size_t(l)]);
			if (s > 1e-8f) {
				const float below = l > 0 ? 0.5f * (p_edges[size_t(l - 1)] + p_edges[size_t(l)]) : 0.0f;
				const float above = l + 1 < nl ? 0.5f * (p_edges[size_t(l + 1)] + p_edges[size_t(l + 2)])
											   : p_edges[size_t(nl)];
				lo = std::min(lo, below);
				hi = std::max(hi, above);
				peak = std::max(peak, s);
			}
		}
		raw[k * 4] = lo;
		raw[k * 4 + 1] = hi;
		raw[k * 4 + 2] = peak;
	}
	std::vector<float> out(n * 4, 0.0f);
	for (int j = 0; j < p_gh; ++j) {
		for (int i = 0; i < p_gw; ++i) {
			float lo = 1e9f, hi = -1e9f, peak = 0.0f;
			for (int dj = -1; dj <= 1; ++dj) {
				const int jj = std::clamp(j + dj, 0, p_gh - 1);
				for (int di = -1; di <= 1; ++di) {
					const size_t s = (size_t(jj) * p_gw + size_t((i + di + p_gw) % p_gw)) * 4;
					if (raw[s + 2] > 0.0f) {
						lo = std::min(lo, raw[s]);
						hi = std::max(hi, raw[s + 1]);
						peak = std::max(peak, raw[s + 2]);
					}
				}
			}
			const size_t k = (size_t(j) * p_gw + i) * 4;
			out[k] = peak > 0.0f ? lo : 0.0f;
			out[k + 1] = peak > 0.0f ? hi : 0.0f;
			out[k + 2] = peak;
		}
	}
	return out;
}

} // namespace

void RegolithClouds::_process(double p_delta) {
	RegolithSystem *sys = Object::cast_to<RegolithSystem>(get_node_or_null(system_path_));
	RegolithBody *b = Object::cast_to<RegolithBody>(get_node_or_null(body_path_));
	RegolithWeather *w = Object::cast_to<RegolithWeather>(get_node_or_null(weather_path_));
	Camera3D *cam = get_viewport() ? get_viewport()->get_camera_3d() : nullptr;
	if (!effect_.is_valid() || !sys || !b || !w || !cam) {
		return;
	}
	attach(cam);
	RegolithCloudEffect::Frame f;
	// no air, no clouds: an invalid frame draws nothing
	const RegolithAtmosphere *atmo = Object::cast_to<RegolithAtmosphere>(get_node_or_null(atmosphere_path_));
	if (!sys->is_anchored() || (atmo && atmo->get_air() == String("None"))) {
		effect_->set_frame(f);
		return;
	}
	const Vector3 eye = cam->get_global_position();
	const regolith::RenderFrame rf = sys->frame_for(b);
	f.to_body.set_column(0, regolith::to_v(rf.east));
	f.to_body.set_column(1, regolith::to_v(rf.up));
	f.to_body.set_column(2, regolith::to_v(rf.south));
	f.eye = sys->body_point(b, eye);
	if (radius_ <= 0.0) {
		radius_ = b->datum_radius(regolith::normalize(f.eye));
	}
	f.datum = radius_;
	const double now = sys->get_time();
	f.sun = regolith::to_v(sys->sun_in_body(b));
	DirectionalLight3D *sun = Object::cast_to<DirectionalLight3D>(get_node_or_null(sun_path_));
	if (sun && sun->is_visible_in_tree()) {
		f.sun_energy = sun->get_param(Light3D::PARAM_ENERGY);
		f.sun_color = sun->get_color();
	}
	Ref<Environment> env = get_viewport()->get_world_3d()->get_environment();
	if (env.is_valid()) {
		f.ambient = env->get_ambient_light_color() * float(env->get_ambient_light_energy());
		if (env->is_fog_enabled()) {
			const Color fc = env->get_fog_light_color() * float(env->get_fog_light_energy());
			f.fog = Color(fc.r, fc.g, fc.b, env->get_fog_density());
		}
	}
	// the physical air's haze, where there is one
	if (RegolithAtmosphere *atmo = Object::cast_to<RegolithAtmosphere>(get_node_or_null(atmosphere_path_))) {
		f.fog = atmo->get_haze();
	}
	// the clouds ride the winds aloft, smoothly and unwrapped
	const double dt = last_time_ < 0.0 ? 0.0 : now - last_time_;
	last_time_ = now;
	if (dt > 0.0 && dt < 3600.0) {
		drift_ = drift_ + regolith::to_d(f.to_body.xform(w->get_layer_wind(eye))) * dt;
	}
	f.drift = drift_;
	f.valid = true;
	RegolithCloudEffect::Field fl;
	if (w->cloud_field(field_version_, fl.tau, fl.topo, fl.gw, fl.gh)) {
		fl.edges.assign(std::begin(regolith::Climate::LEVEL_M), std::end(regolith::Climate::LEVEL_M));
		fl.band = cloud_band(fl.tau, fl.gw, fl.gh, fl.edges);
		effect_->set_field(std::move(fl));
	}
	effect_->set_frame(f);
	effect_->set_look(look_);
}

#include "regolith_weather.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <numbers>

#ifdef __linux__
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include <godot_cpp/classes/box_mesh.hpp>
#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/color_rect.hpp>
#include <godot_cpp/classes/directional_light3d.hpp>
#include <godot_cpp/classes/fast_noise_lite.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/noise_texture3d.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/classes/texture2d.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/viewport_texture.hpp>
#include <godot_cpp/core/class_db.hpp>

#include "convert.hpp"
#include "regolith/ephemeris.hpp"
#include "regolith_atmosphere.hpp"
#include "regolith_body.hpp"
#include "regolith_gcm.hpp"
#include "regolith_system.hpp"

using namespace godot;
using regolith::DVec3;
using regolith::Gcm;

namespace {

constexpr double PI = std::numbers::pi;
constexpr double DEG = PI / 180.0;
constexpr int SAVE_VERSION = 2; // 2: water in kg/m2, 8 levels
constexpr int SPINUP_STEPS = 320; // mors: ~10.7 model hours
// Behind the clock by more than this, the skipped time goes
// to the season clock (caps, deep soil, haze) instead.
constexpr double MAX_LAG = 3600.0;
constexpr double SEASON_K_MAX = 100.0;
constexpr double LOOP_BUDGET_MS = 40.0;
constexpr int TOPO_LEVEL = 2; // ~20 km samples for 60 km cells

const char *SAVE_FIELDS[] = { "u", "v", "h", "wv", "dust", "dustc", "vap0", "vap1", "vap2", "vap3", "vap4", "vap5",
	"vap6", "vap7", "ice0", "ice1", "ice2", "ice3", "ice4", "ice5", "ice6", "ice7", "frost", "soil", "ts", "ta",
	"tdeep", "tmean", "ttm", "co2", "wfr", "wlf", "wdu", "wlc", "wdc" };

} // namespace

void RegolithWeather::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_system_path", "path"), &RegolithWeather::set_system_path);
	ClassDB::bind_method(D_METHOD("get_system_path"), &RegolithWeather::get_system_path);
	ClassDB::bind_method(D_METHOD("set_body_path", "path"), &RegolithWeather::set_body_path);
	ClassDB::bind_method(D_METHOD("get_body_path"), &RegolithWeather::get_body_path);
	ClassDB::bind_method(D_METHOD("set_atmosphere_path", "path"), &RegolithWeather::set_atmosphere_path);
	ClassDB::bind_method(D_METHOD("get_atmosphere_path"), &RegolithWeather::get_atmosphere_path);
	ClassDB::bind_method(D_METHOD("set_sun_path", "path"), &RegolithWeather::set_sun_path);
	ClassDB::bind_method(D_METHOD("get_sun_path"), &RegolithWeather::get_sun_path);
	ClassDB::bind_method(D_METHOD("set_albedo_map_path", "path"), &RegolithWeather::set_albedo_map_path);
	ClassDB::bind_method(D_METHOD("get_albedo_map_path"), &RegolithWeather::get_albedo_map_path);
	ClassDB::bind_method(D_METHOD("set_climatology_path", "path"), &RegolithWeather::set_climatology_path);
	ClassDB::bind_method(D_METHOD("get_climatology_path"), &RegolithWeather::get_climatology_path);
	ClassDB::bind_method(D_METHOD("set_save_path", "path"), &RegolithWeather::set_save_path);
	ClassDB::bind_method(D_METHOD("get_save_path"), &RegolithWeather::get_save_path);
	ClassDB::bind_method(D_METHOD("set_initial_state_path", "path"), &RegolithWeather::set_initial_state_path);
	ClassDB::bind_method(D_METHOD("get_initial_state_path"), &RegolithWeather::get_initial_state_path);
	ClassDB::bind_method(D_METHOD("set_bake", "sols", "path"), &RegolithWeather::set_bake);
	ClassDB::bind_method(D_METHOD("is_bake_done"), &RegolithWeather::is_bake_done);
	ClassDB::bind_method(D_METHOD("set_threads", "threads"), &RegolithWeather::set_threads);
	ClassDB::bind_method(D_METHOD("get_threads"), &RegolithWeather::get_threads);
	ClassDB::bind_method(D_METHOD("set_config", "config"), &RegolithWeather::set_config);
	ClassDB::bind_method(D_METHOD("get_config"), &RegolithWeather::get_config);
	ClassDB::bind_method(D_METHOD("set_debug_view", "view"), &RegolithWeather::set_debug_view);
	ClassDB::bind_method(D_METHOD("get_debug_view"), &RegolithWeather::get_debug_view);
	ClassDB::bind_method(D_METHOD("set_clouds_visible", "visible"), &RegolithWeather::set_clouds_visible);
	ClassDB::bind_method(D_METHOD("get_clouds_visible"), &RegolithWeather::get_clouds_visible);
	ClassDB::bind_method(D_METHOD("get_tau", "position"), &RegolithWeather::get_tau);
	ClassDB::bind_method(D_METHOD("get_wind", "position"), &RegolithWeather::get_wind);
	ClassDB::bind_method(D_METHOD("get_wind_local", "position"), &RegolithWeather::get_wind_local);
	ClassDB::bind_method(D_METHOD("get_layer_wind", "position"), &RegolithWeather::get_layer_wind);
	ClassDB::bind_method(D_METHOD("get_pressure", "position"), &RegolithWeather::get_pressure);
	ClassDB::bind_method(D_METHOD("get_density", "position"), &RegolithWeather::get_density);
	ClassDB::bind_method(D_METHOD("get_air_temperature", "position"), &RegolithWeather::get_air_temperature);
	ClassDB::bind_method(D_METHOD("get_ground_temperature", "position"), &RegolithWeather::get_ground_temperature);
	ClassDB::bind_method(D_METHOD("get_frost", "position"), &RegolithWeather::get_frost);
	ClassDB::bind_method(D_METHOD("get_cloud", "position"), &RegolithWeather::get_cloud);
	ClassDB::bind_method(D_METHOD("get_vapor", "position"), &RegolithWeather::get_vapor);
	ClassDB::bind_method(D_METHOD("get_cloud_layers", "position"), &RegolithWeather::get_cloud_layers);
	ClassDB::bind_method(D_METHOD("find_clouds", "min_sun_deg"), &RegolithWeather::find_clouds);
	ClassDB::bind_method(D_METHOD("get_cloud_levels"), &RegolithWeather::get_cloud_levels);
	ClassDB::bind_method(D_METHOD("get_cloud_frame"), &RegolithWeather::get_cloud_frame);
	ClassDB::bind_method(D_METHOD("inject_dust", "position"), &RegolithWeather::inject_dust);
	ClassDB::bind_method(D_METHOD("is_ready"), &RegolithWeather::is_ready);
	ClassDB::bind_method(D_METHOD("get_stats"), &RegolithWeather::get_stats);
	ClassDB::bind_method(D_METHOD("save_now"), &RegolithWeather::save_now);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "system_path"), "set_system_path", "get_system_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "body_path"), "set_body_path", "get_body_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "atmosphere_path"), "set_atmosphere_path", "get_atmosphere_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "sun_path"), "set_sun_path", "get_sun_path");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "albedo_map_path", PROPERTY_HINT_FILE), "set_albedo_map_path", "get_albedo_map_path");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "climatology_path", PROPERTY_HINT_FILE), "set_climatology_path",
			"get_climatology_path");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "save_path"), "set_save_path", "get_save_path");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "initial_state_path", PROPERTY_HINT_FILE), "set_initial_state_path",
			"get_initial_state_path");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "threads"), "set_threads", "get_threads");
	ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "config"), "set_config", "get_config");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "clouds_visible"), "set_clouds_visible", "get_clouds_visible");
}

RegolithWeather::~RegolithWeather() {
	exit_ = true;
	if (thread_.joinable()) {
		thread_.join();
	}
}

void RegolithWeather::_exit_tree() {
	exit_ = true;
	if (thread_.joinable()) {
		thread_.join();
	}
}

RegolithSystem *RegolithWeather::system() const {
	return Object::cast_to<RegolithSystem>(get_node_or_null(system_path_));
}

RegolithBody *RegolithWeather::body() const {
	return Object::cast_to<RegolithBody>(get_node_or_null(body_path_));
}

// Main-thread setup: the albedo proxy and climatology are read
// here; the DEM grid and spin-up run on the weather thread.
void RegolithWeather::start() {
	gcm_ = std::make_unique<Gcm>(RegolithGcm::config_from(config_));
	const int gw = gcm_->GW, gh = gcm_->GH;
	if (!albedo_map_path_.is_empty()) {
		Ref<Texture2D> tex = ResourceLoader::get_singleton()->load(albedo_map_path_);
		Ref<Image> img = tex.is_valid() ? tex->get_image() : Ref<Image>();
		if (img.is_valid()) {
			img = img->duplicate();
			if (img->is_compressed()) {
				img->decompress();
			}
			img->convert(Image::FORMAT_RGB8);
			img->resize(gw, gh, Image::INTERPOLATE_BILINEAR);
			lum_.resize(size_t(gw) * gh);
			for (int j = 0; j < gh; ++j) {
				for (int i = 0; i < gw; ++i) {
					lum_[size_t(j) * gw + i] = img->get_pixel(i, j).get_luminance();
				}
			}
		}
	}
	if (!clim_path_.is_empty()) {
		const PackedStringArray lines = FileAccess::get_file_as_string(clim_path_).split("\n");
		for (const String &line : lines) {
			const String t = line.strip_edges();
			if (t.is_empty() || t.begins_with("#")) {
				continue;
			}
			const PackedStringArray p = t.split(" ", false);
			if (p.size() >= 3) {
				clim_.push_back(p[2].to_float());
			}
		}
		if (clim_.size() != 24) {
			clim_.clear();
		}
	}
	save_global_ = ProjectSettings::get_singleton()->globalize_path(save_path_);
	bake_global_ = ProjectSettings::get_singleton()->globalize_path(bake_path_);
	if (FileAccess::file_exists(save_path_)) {
		save_bytes_ = FileAccess::get_file_as_bytes(save_path_);
	}
	if (!initial_state_path_.is_empty() && FileAccess::file_exists(initial_state_path_)) {
		initial_bytes_ = FileAccess::get_file_as_bytes(initial_state_path_);
	}
	body_ptr_ = body(); // the thread may not look nodes up
	setup_clouds();
	started_ = true;
	thread_ = std::thread([this] { run(); });
}

void RegolithWeather::run() {
#ifdef __linux__
	// below the game and the terrain feeders' main work
	setpriority(PRIO_PROCESS, pid_t(syscall(SYS_gettid)), 10);
#endif
	Gcm &g = *gcm_;
	RegolithBody *b = body_ptr_;
	std::vector<float> topo;
	b->height_grid(g.GW, g.GH, TOPO_LEVEL, topo);
	double now;
	{
		std::lock_guard<std::mutex> guard(mutex_);
		now = in_time_;
	}
	// A bake runs the model flat out from bake_sols back and
	// writes the state it reaches: a fresh world's start.
	bool baking = bake_sols_ > 0;
	double sim_time = now - SPINUP_STEPS * Gcm::DT - (baking ? bake_sols_ * g.config().sol_s : 0.0);
	// caps and the mean state seeded for the season it starts in
	g.create(topo.data(), lum_.empty() ? nullptr : lum_.data(), regolith::mars_solar_longitude(sim_time), threads_);
	g.clim_max = clim_;
	const bool resumed = !baking && load(g, sim_time);
	// a state from ahead of the clock (a bake finishing later)
	// is taken as now: one step runs at once
	sim_time = std::min(sim_time, now - Gcm::DT);
	int spin = resumed ? 0 : SPINUP_STEPS;
	double season_credit = 0.0;
	auto last_save = std::chrono::steady_clock::now();
	const double sol_hours = g.config().sol_hours;
	while (!exit_) {
		double target, px, py;
		bool dust;
		double dpx, dpy;
		{
			std::lock_guard<std::mutex> guard(mutex_);
			target = in_time_;
			px = in_px_;
			py = in_py_;
			dust = in_dust_;
			dpx = in_dust_px_;
			dpy = in_dust_py_;
			in_dust_ = false;
		}
		if (dust) {
			g.inject_dust(dpx, dpy);
		}
		if (target - sim_time > MAX_LAG && spin == 0 && !baking) {
			const double skip = target - sim_time - MAX_LAG;
			season_credit += skip;
			sim_time += skip;
		}
		const auto t0 = std::chrono::steady_clock::now();
		const double start_time = sim_time;
		int steps = 0;
		double ms = 0.0, k_used = 1.0;
		while (target - sim_time >= Gcm::DT && !exit_) {
			Gcm::Inputs in;
			in.ls = regolith::mars_solar_longitude(sim_time);
			in.mtc = regolith::mars_local_solar_time(sim_time, 0.0) / 24.0 * sol_hours;
			const double c = std::min(season_credit, Gcm::DT * (SEASON_K_MAX - 1.0));
			season_credit -= c;
			k_used = (Gcm::DT + c) / Gcm::DT;
			in.year_sols = k_used > 1.0 ? g.YEAR_SOLS / k_used : -1.0;
			g.step(in, spin > 0);
			spin = std::max(spin - 1, 0);
			sim_time += Gcm::DT;
			++steps;
			ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
			if (ms > LOOP_BUDGET_MS) {
				break;
			}
		}
		if (steps > 0 && !g.finite()) {
			g.reset_dynamics();
		}
		g.local_follow(px, py);
		g.local_tick(std::min(sim_time - start_time, 3600.0));
		publish(g);
		{
			std::lock_guard<std::mutex> guard(mutex_);
			if (steps > 0) {
				stat_step_ms_ = ms / steps;
			}
			stat_lag_s_ = target - sim_time + season_credit;
			stat_season_k_ = k_used;
			stat_sim_time_ = sim_time;
			stat_steps_ += uint64_t(steps);
		}
		ready_ = spin == 0 && !baking;
		if (baking && target - sim_time < Gcm::DT) {
			save(g, sim_time, bake_global_);
			baking = false;
			bake_done_ = true;
		}
		const auto now_c = std::chrono::steady_clock::now();
		if (ready_ && (save_request_.exchange(false) || now_c - last_save > std::chrono::seconds(60))) {
			save(g, sim_time, save_global_);
			last_save = now_c;
		}
		// half duty, as mors: rest as long as the steps took
		// (spin-up and bakes run flat out: nothing is drawn yet)
		const int rest = spin > 0 || baking ? 1 : std::clamp(int(ms), 20, 600);
		std::this_thread::sleep_for(std::chrono::milliseconds(rest));
	}
	if (ready_) {
		save(g, sim_time, save_global_);
	}
}

void RegolithWeather::publish(const Gcm &p_g) {
	Published p;
	const size_t n = size_t(p_g.N);
	p.dust.resize(n);
	p.cloud.resize(n);
	p.su.resize(n);
	p.sv.resize(n);
	p.q[2].resize(n);
	for (size_t k = 0; k < n; ++k) {
		p.dust[k] = p_g.dust[k] + p_g.dustc[k];
		float ice = 0.0f;
		for (int l = 0; l < Gcm::NL; ++l) {
			ice += p_g.ice[size_t(l)][k];
		}
		p.q[2][k] = ice;
		p.cloud[k] = p_g.tau_low[k] + p_g.tau_high[k];
		p.su[k] = float(p_g.u[k] * Gcm::SURF_WIND + p_g.wsx[k]);
		p.sv[k] = float(p_g.v[k] * Gcm::SURF_WIND + p_g.wsy[k]);
	}
	p.tau_levels.resize(size_t(Gcm::NL) * n);
	for (int l = 0; l < Gcm::NL; ++l) {
		std::memcpy(p.tau_levels.data() + size_t(l) * n, p_g.tau[size_t(l)].data(), n * sizeof(float));
	}
	p.version = pub_version_.fetch_add(1) + 1;
	p.lu = p_g.u;
	p.lv = p_g.v;
	p.q[0] = p_g.tau_low;
	p.q[1] = p_g.tau_high;
	p.qv = p_g.wv;
	p.ts = p_g.ts;
	p.tair = p_g.tair;
	p.p = p_g.psurf;
	p.co2 = p_g.co2;
	p.frost = p_g.frost;
	p_g.pack_local(p.local);
	p_g.pack_global(p.global);
	p_g.pack_surface(p.surface);
	p.org_x = p_g.local_org_x();
	p.org_y = p_g.local_org_y();
	p.fresh = true;
	std::lock_guard<std::mutex> guard(mutex_);
	pub_ = std::move(p);
}

// Binary state: version, grid, sim time, scalars, the fields.
void RegolithWeather::save(const Gcm &p_g, double p_sim_time, const String &p_path) const {
	if (!p_g.finite() || p_path.is_empty()) {
		return;
	}
	const std::string path = p_path.utf8().get_data();
	std::ofstream f(path + ".tmp", std::ios::binary);
	if (!f) {
		return;
	}
	auto put = [&](const auto &p_v) { f.write(reinterpret_cast<const char *>(&p_v), sizeof(p_v)); };
	put(SAVE_VERSION);
	put(p_g.GW);
	put(p_g.GH);
	put(p_sim_time);
	for (double v : { p_g.hbar, p_g.ttbar, p_g.ttmbar, p_g.msbar, p_g.bc_timer, p_g.bc_phase[0], p_g.bc_phase[1],
				 p_g.bc_period[0], p_g.bc_period[1] }) {
		put(v);
	}
	put(p_g.bc_m[0]);
	put(p_g.bc_m[1]);
	put(int(p_g.ttbar_set));
	Gcm &g = const_cast<Gcm &>(p_g);
	for (const char *name : SAVE_FIELDS) {
		const Gcm::Field *fl = RegolithGcm::field_of(g, name);
		f.write(reinterpret_cast<const char *>(fl->data()), std::streamsize(fl->size() * sizeof(float)));
	}
	f.close();
	std::rename((path + ".tmp").c_str(), path.c_str());
}

// The player's save, else the shipped spun-up state (read on
// the main thread: res:// may live in a pack).
bool RegolithWeather::load(Gcm &p_g, double &r_sim_time) const {
	const PackedByteArray &bytes = !save_bytes_.is_empty() ? save_bytes_ : initial_bytes_;
	size_t at = 0;
	bool ok = !bytes.is_empty();
	auto get = [&](auto &r_v) {
		if (at + sizeof(r_v) > size_t(bytes.size())) {
			ok = false;
			return;
		}
		std::memcpy(&r_v, bytes.ptr() + at, sizeof(r_v));
		at += sizeof(r_v);
	};
	int version = 0, gw = 0, gh = 0;
	get(version);
	get(gw);
	get(gh);
	if (!ok || version != SAVE_VERSION || gw != p_g.GW || gh != p_g.GH) {
		return false;
	}
	double sim_time = 0.0, s[9];
	get(sim_time);
	for (double &v : s) {
		get(v);
	}
	int m0 = 0, m1 = 0, set = 0;
	get(m0);
	get(m1);
	get(set);
	std::vector<std::vector<float>> fields;
	const size_t field_bytes = size_t(p_g.N) * sizeof(float);
	for (size_t i = 0; i < sizeof(SAVE_FIELDS) / sizeof(SAVE_FIELDS[0]); ++i) {
		if (at + field_bytes > size_t(bytes.size())) {
			return false;
		}
		std::vector<float> v(size_t(p_g.N));
		std::memcpy(v.data(), bytes.ptr() + at, field_bytes);
		at += field_bytes;
		fields.push_back(std::move(v));
	}
	if (!ok) {
		return false;
	}
	for (size_t i = 0; i < fields.size(); ++i) {
		*RegolithGcm::field_of(p_g, SAVE_FIELDS[i]) = std::move(fields[i]);
	}
	p_g.hbar = s[0];
	p_g.ttbar = s[1];
	p_g.ttmbar = s[2];
	p_g.msbar = s[3];
	p_g.bc_timer = s[4];
	p_g.bc_phase = { s[5], s[6] };
	p_g.bc_period = { s[7], s[8] };
	p_g.bc_m = { m0, m1 };
	p_g.ttbar_set = set != 0;
	if (!p_g.finite()) {
		return false;
	}
	// the saved weather resumes at its own time; a clock far
	// ahead is caught up through the season clock
	r_sim_time = sim_time;
	return true;
}

bool RegolithWeather::locate(const Vector3 &p_pos, double &r_px, double &r_py, DVec3 *r_dir) const {
	const RegolithSystem *sys = system();
	const RegolithBody *b = body();
	if (!sys || !b || !gcm_) {
		return false;
	}
	const DVec3 d = regolith::normalize(sys->body_point(b, p_pos));
	const double lat = std::atan2(d.z, std::hypot(d.x, d.y)) / DEG;
	const double lon = std::atan2(d.y, d.x) / DEG;
	r_px = (lon + 180.0) / 360.0 * gcm_->W_PX;
	r_py = (90.0 - lat) / 180.0 * gcm_->H_PX;
	if (r_dir) {
		*r_dir = d;
	}
	return true;
}

double RegolithWeather::pub_at(const std::vector<float> &p_f, const Vector3 &p_pos) const {
	double px, py;
	if (!locate(p_pos, px, py) || p_f.empty()) {
		return std::nan("");
	}
	return gcm_->sample(p_f, px, py);
}

// A local (east, north) vector at p_dir, in render space.
Vector3 RegolithWeather::to_render(const DVec3 &p_dir, double p_east, double p_north) const {
	const RegolithSystem *sys = system();
	const DVec3 east = regolith::normalize(regolith::cross(DVec3(0, 0, 1), p_dir));
	const DVec3 north = regolith::cross(p_dir, east);
	return regolith::to_v(sys->frame_for(body()).to_local_vec(east * p_east + north * p_north));
}

double RegolithWeather::get_tau(const Vector3 &p_pos) const {
	std::lock_guard<std::mutex> guard(mutex_);
	const double d = pub_at(pub_.dust, p_pos);
	return std::isnan(d) ? 0.35 : Gcm::TAU_FLOOR + Gcm::TAU_PER_DUST * d;
}

Vector2 RegolithWeather::get_wind_local(const Vector3 &p_pos) const {
	std::lock_guard<std::mutex> guard(mutex_);
	const double u = pub_at(pub_.su, p_pos), v = pub_at(pub_.sv, p_pos);
	// grid v points south
	return std::isnan(u) ? Vector2() : Vector2(real_t(u), real_t(-v));
}

Vector3 RegolithWeather::get_wind(const Vector3 &p_pos) const {
	double px, py;
	DVec3 dir;
	if (!locate(p_pos, px, py, &dir)) {
		return Vector3();
	}
	const Vector2 w = get_wind_local(p_pos);
	return to_render(dir, w.x, w.y);
}

Vector3 RegolithWeather::get_layer_wind(const Vector3 &p_pos) const {
	double px, py;
	DVec3 dir;
	if (!locate(p_pos, px, py, &dir)) {
		return Vector3();
	}
	std::lock_guard<std::mutex> guard(mutex_);
	if (pub_.lu.empty()) {
		return Vector3();
	}
	return to_render(dir, gcm_->sample(pub_.lu, px, py), -gcm_->sample(pub_.lv, px, py));
}

double RegolithWeather::get_pressure(const Vector3 &p_pos) const {
	std::lock_guard<std::mutex> guard(mutex_);
	const double p = pub_at(pub_.p, p_pos);
	return std::isnan(p) ? 610.0 : p;
}

double RegolithWeather::get_air_temperature(const Vector3 &p_pos) const {
	std::lock_guard<std::mutex> guard(mutex_);
	const double t = pub_at(pub_.tair, p_pos);
	return std::isnan(t) ? 210.0 : t;
}

double RegolithWeather::get_ground_temperature(const Vector3 &p_pos) const {
	std::lock_guard<std::mutex> guard(mutex_);
	const double t = pub_at(pub_.ts, p_pos);
	return std::isnan(t) ? 215.0 : t;
}

double RegolithWeather::get_density(const Vector3 &p_pos) const {
	const double t = get_air_temperature(p_pos);
	return get_pressure(p_pos) / (gcm_ ? gcm_->R_GAS : 191.8) * 1.0 / std::max(t, 100.0);
}

Vector2 RegolithWeather::get_frost(const Vector3 &p_pos) const {
	std::lock_guard<std::mutex> guard(mutex_);
	const double c = pub_at(pub_.co2, p_pos), w = pub_at(pub_.frost, p_pos);
	return std::isnan(c) ? Vector2() : Vector2(real_t(c), real_t(w));
}

double RegolithWeather::get_cloud(const Vector3 &p_pos) const {
	std::lock_guard<std::mutex> guard(mutex_);
	const double c = pub_at(pub_.cloud, p_pos);
	return std::isnan(c) ? 0.0 : c;
}

Vector3 RegolithWeather::get_cloud_layers(const Vector3 &p_pos) const {
	double px, py;
	if (!locate(p_pos, px, py)) {
		return Vector3();
	}
	std::lock_guard<std::mutex> guard(mutex_);
	if (pub_.q[0].empty()) {
		return Vector3();
	}
	return Vector3(real_t(gcm_->sample(pub_.q[0], px, py)), real_t(gcm_->sample(pub_.q[1], px, py)),
			real_t(gcm_->sample(pub_.q[2], px, py)));
}

// The cloudiest cell with the sun at least p_min_sun_deg up:
// lat, lon (deg), and the three ice layers there.
Dictionary RegolithWeather::find_clouds(double p_min_sun_deg) const {
	Dictionary out;
	RegolithSystem *sys = system();
	std::lock_guard<std::mutex> guard(mutex_);
	if (!gcm_ || pub_.q[0].empty() || !sys) {
		return out;
	}
	const DVec3 sun = regolith::mars_sun_direction(sys->get_time());
	const double min_mu = std::sin(p_min_sun_deg * DEG);
	const int gw = gcm_->GW, gh = gcm_->GH;
	double best = 0.0;
	int best_k = -1;
	for (int j = 0; j < gh; ++j) {
		const double lat = (90.0 - (j + 0.5) / gh * 180.0) * DEG;
		for (int i = 0; i < gw; ++i) {
			const double lon = ((i + 0.5) / gw * 360.0 - 180.0) * DEG;
			const DVec3 d(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
			if (regolith::dot(d, sun) < min_mu) {
				continue;
			}
			const size_t k = size_t(j) * gw + i;
			// the column's visible optical depth
			const double score = pub_.q[0][k] + pub_.q[1][k];
			if (score > best) {
				best = score;
				best_k = int(k);
			}
		}
	}
	if (best_k >= 0) {
		const int i = best_k % gw, j = best_k / gw;
		out["lat"] = 90.0 - (j + 0.5) / gh * 180.0;
		out["lon"] = (i + 0.5) / gw * 360.0 - 180.0;
		out["layers"] = Vector3(pub_.q[0][size_t(best_k)], pub_.q[1][size_t(best_k)], pub_.q[2][size_t(best_k)]);
	}
	return out;
}

Dictionary RegolithWeather::get_cloud_levels() const {
	Dictionary d;
	std::lock_guard<std::mutex> guard(mutex_);
	if (!gcm_ || pub_.tau_levels.empty()) {
		return d;
	}
	PackedFloat32Array tau, topo, edges;
	tau.resize(int64_t(pub_.tau_levels.size()));
	std::memcpy(tau.ptrw(), pub_.tau_levels.data(), pub_.tau_levels.size() * sizeof(float));
	// fixed after create(): safe to read beside the thread
	topo.resize(int64_t(gcm_->topo.size()));
	std::memcpy(topo.ptrw(), gcm_->topo.data(), gcm_->topo.size() * sizeof(float));
	for (double e : Gcm::LEVEL_M) {
		edges.push_back(float(e));
	}
	d["gw"] = gcm_->GW;
	d["gh"] = gcm_->GH;
	d["levels"] = Gcm::NL;
	d["tau"] = tau;
	d["topo"] = topo;
	d["edges"] = edges;
	d["version"] = int64_t(pub_.version);
	return d;
}

Dictionary RegolithWeather::get_cloud_frame() const {
	Dictionary d;
	RegolithSystem *sys = system();
	RegolithBody *b = body();
	Camera3D *cam = get_viewport() ? get_viewport()->get_camera_3d() : nullptr;
	if (!sys || !b || !cam || !sys->is_anchored()) {
		return d;
	}
	const Vector3 eye = cam->get_global_position();
	const regolith::RenderFrame f = sys->frame_for(b);
	const DVec3 eye_b = sys->body_point(b, eye);
	Basis to_body;
	to_body.set_column(0, regolith::to_v(f.east));
	to_body.set_column(1, regolith::to_v(f.up));
	to_body.set_column(2, regolith::to_v(f.south));
	d["centre"] = regolith::to_v(f.to_local_point(DVec3()) - regolith::to_d(eye));
	d["to_body"] = to_body;
	d["eye_body"] = regolith::to_v(eye_b);
	d["datum"] = b->datum_radius(regolith::normalize(eye_b));
	d["sun_body"] = regolith::to_v(regolith::mars_sun_direction(sys->get_time()));
	d["sun_visible"] = sys->get_sun_visible();
	return d;
}

double RegolithWeather::get_vapor(const Vector3 &p_pos) const {
	std::lock_guard<std::mutex> guard(mutex_);
	const double q = pub_at(pub_.qv, p_pos);
	return std::isnan(q) ? 0.0 : q;
}

void RegolithWeather::inject_dust(const Vector3 &p_pos) {
	double px, py;
	if (!locate(p_pos, px, py)) {
		return;
	}
	std::lock_guard<std::mutex> guard(mutex_);
	in_dust_ = true;
	in_dust_px_ = px;
	in_dust_py_ = py;
}

void RegolithWeather::save_now() {
	save_request_ = true;
}

Dictionary RegolithWeather::get_stats() const {
	std::lock_guard<std::mutex> guard(mutex_);
	Dictionary d;
	d["ready"] = ready_.load();
	d["step_ms"] = stat_step_ms_;
	d["lag_s"] = stat_lag_s_;
	d["season_k"] = stat_season_k_;
	d["sim_time"] = stat_sim_time_;
	d["steps"] = int64_t(stat_steps_);
	d["ls"] = regolith::mars_solar_longitude(stat_sim_time_);
	d["ground_alt"] = ground_alt_;
	d["eye_alt"] = eye_alt_;
	return d;
}

void RegolithWeather::setup_clouds() {
	const int gw = gcm_->GW, gh = gcm_->GH, ln = gcm_->LN;
	// mors's noise volume: simplex FBM, seed 640, 72^3 seamless
	Ref<FastNoiseLite> fnl;
	fnl.instantiate();
	fnl->set_seed(640);
	fnl->set_noise_type(FastNoiseLite::TYPE_SIMPLEX);
	fnl->set_fractal_type(FastNoiseLite::FRACTAL_FBM);
	fnl->set_fractal_octaves(4);
	fnl->set_frequency(0.04f);
	Ref<NoiseTexture3D> n3d;
	n3d.instantiate();
	n3d->set_noise(fnl);
	n3d->set_width(72);
	n3d->set_height(72);
	n3d->set_depth(72);
	n3d->set_seamless(true);
	local_tex_ = ImageTexture::create_from_image(Image::create_empty(ln, ln, false, Image::FORMAT_RGBA8));
	global_tex_ = ImageTexture::create_from_image(Image::create_empty(gw, gh, false, Image::FORMAT_RGBA8));
	surface_tex_ = ImageTexture::create_from_image(Image::create_empty(gw, gh, false, Image::FORMAT_RGBA8));
	auto shared = [&](const Ref<ShaderMaterial> &p_m) {
		p_m->set_shader_parameter("noise_tex", n3d);
		p_m->set_shader_parameter("local_tex", local_tex_);
		p_m->set_shader_parameter("global_tex", global_tex_);
		p_m->set_shader_parameter("planet_px", Vector2(real_t(gcm_->W_PX), real_t(gcm_->H_PX)));
		p_m->set_shader_parameter("win_px", real_t(ln * gcm_->LCELL_PX));
	};
	mat_.instantiate();
	mat_->set_shader(ResourceLoader::get_singleton()->load("res://addons/regolith/regolith_clouds.gdshader"));
	shared(mat_);
	Ref<BoxMesh> box;
	box.instantiate();
	box->set_size(Vector3(1, 1, 1));
	box_ = memnew(MeshInstance3D);
	box_->set_mesh(box);
	box_->set_material_override(mat_);
	box_->set_cast_shadows_setting(GeometryInstance3D::SHADOW_CASTING_SETTING_OFF);
	box_->set_scale(Vector3(1.2e6, 1.2e6, 1.2e6));
	box_->set_extra_cull_margin(16384.0);
	add_child(box_);
	// the sun's transmittance on the ground around the eye
	shadow_mat_.instantiate();
	shadow_mat_->set_shader(ResourceLoader::get_singleton()->load("res://addons/regolith/regolith_cloud_shadow.gdshader"));
	shared(shadow_mat_);
	shadow_vp_ = memnew(SubViewport);
	shadow_vp_->set_size(Vector2i(SHADOW_TEXELS, SHADOW_TEXELS));
	shadow_vp_->set_update_mode(SubViewport::UPDATE_ALWAYS);
	shadow_vp_->set_disable_3d(true);
	shadow_vp_->set_use_hdr_2d(true);
	ColorRect *rect = memnew(ColorRect);
	rect->set_size(Vector2(SHADOW_TEXELS, SHADOW_TEXELS));
	rect->set_material(shadow_mat_);
	shadow_vp_->add_child(rect);
	add_child(shadow_vp_);
	if (RegolithBody *b = body()) {
		// the viewport owns its texture: hand over the RID
		const RID tex = shadow_vp_->get_texture()->get_rid();
		b->set_material_param("cloud_shadow", tex);
		b->set_material_param("cloud_size", SHADOW_M);
		b->set_material_param("has_cloud_shadow", true);
	}
}

void RegolithWeather::update_clouds() {
	RegolithSystem *sys = system();
	RegolithBody *b = body();
	Camera3D *cam = get_viewport()->get_camera_3d();
	if (!mat_.is_valid() || !cam) {
		return;
	}
	{
		std::lock_guard<std::mutex> guard(mutex_);
		if (pub_.fresh) {
			const int gw = gcm_->GW, gh = gcm_->GH, ln = gcm_->LN;
			auto bytes = [](const std::vector<uint8_t> &p_v) {
				PackedByteArray a;
				a.resize(int64_t(p_v.size()));
				std::memcpy(a.ptrw(), p_v.data(), p_v.size());
				return a;
			};
			local_tex_->update(Image::create_from_data(ln, ln, false, Image::FORMAT_RGBA8, bytes(pub_.local)));
			global_tex_->update(Image::create_from_data(gw, gh, false, Image::FORMAT_RGBA8, bytes(pub_.global)));
			surface_tex_->update(Image::create_from_data(gw, gh, false, Image::FORMAT_RGBA8, bytes(pub_.surface)));
			org_x_ = pub_.org_x;
			org_y_ = pub_.org_y;
			pub_.fresh = false;
		}
	}
	const Vector3 eye = cam->get_global_position();
	const regolith::RenderFrame f = sys->frame_for(b);
	const DVec3 eye_b = sys->body_point(b, eye);
	const DVec3 up = regolith::normalize(eye_b);
	const double datum = b->datum_radius(up);
	const double eye_alt = regolith::length(eye_b) - datum;
	eye_alt_ = eye_alt;
	const double clear = sys->get_ground_clearance(eye);
	const double ground = std::isnan(clear) ? 0.0 : eye_alt - clear;
	ground_alt_ = ground_set_ ? ground_alt_ + (ground - ground_alt_) * 0.02 : ground;
	ground_set_ = true;
	// the noise chart: east and south at an anchor, moved only
	// when the eye has gone far (or is above the clouds)
	if (!chart_set_ || regolith::dot(up, chart_up_) < std::cos(2.0e6 / datum)) {
		chart_up_ = up;
		chart_set_ = true;
		drift_air_ = drift_dust_ = DVec3();
	}
	const DVec3 ce = regolith::normalize(regolith::cross(DVec3(0, 0, 1), chart_up_));
	const DVec3 cs = regolith::cross(ce, chart_up_); // south
	// the decks drift with the model's winds on the sim clock
	const double now = sys->get_time();
	const double sim_dt = now - last_time_;
	last_time_ = now;
	if (sim_dt > 0.0 && sim_dt < 3600.0 && ready_) {
		double px, py;
		locate(eye, px, py);
		std::lock_guard<std::mutex> guard(mutex_);
		if (!pub_.lu.empty()) {
			// grid (east, south) is the chart's (x, z) near it
			drift_air_.x += gcm_->sample(pub_.lu, px, py) * sim_dt;
			drift_air_.z += gcm_->sample(pub_.lv, px, py) * sim_dt;
			drift_dust_.x += gcm_->sample(pub_.su, px, py) * sim_dt;
			drift_dust_.z += gcm_->sample(pub_.sv, px, py) * sim_dt;
		}
	}
	Basis to_body;
	to_body.set_column(0, regolith::to_v(f.east));
	to_body.set_column(1, regolith::to_v(f.up));
	to_body.set_column(2, regolith::to_v(f.south));
	// camera-relative, in double before the cast
	const DVec3 eye_r = regolith::to_d(eye);
	const Vector3 centre = regolith::to_v(f.to_local_point(DVec3()) - eye_r);
	const DVec3 sun = regolith::mars_sun_direction(sys->get_time());
	for (const Ref<ShaderMaterial> &m : { mat_, shadow_mat_ }) {
		m->set_shader_parameter("window_org_px", Vector2(real_t(org_x_), real_t(org_y_)));
		m->set_shader_parameter("u_center", centre);
		m->set_shader_parameter("u_eye_b", regolith::to_v(eye_b));
		m->set_shader_parameter("u_to_body", to_body);
		m->set_shader_parameter("u_chart_e", regolith::to_v(ce));
		m->set_shader_parameter("u_chart_s", regolith::to_v(cs));
		m->set_shader_parameter("u_datum_r", real_t(datum));
		m->set_shader_parameter("ground_alt", real_t(ground_alt_));
		m->set_shader_parameter("sun_body", regolith::to_v(sun));
		m->set_shader_parameter("drift_air", Vector2(real_t(drift_air_.x), real_t(drift_air_.z)));
		m->set_shader_parameter("drift_dust", Vector2(real_t(drift_dust_.x), real_t(drift_dust_.z)));
	}
	box_->set_global_position(eye);
	// from orbit the field is the globe's to show
	box_->set_visible(clouds_visible_ && ready_ && eye_alt < 100000.0);
	mat_->set_shader_parameter("orbit_fade", real_t(1.0 - std::clamp((eye_alt - 60000.0) / 40000.0, 0.0, 1.0)));
	mat_->set_shader_parameter("debug_view", debug_view_);
	// shadow map: the tangent plane under the eye, render axes
	const DVec3 e_b = regolith::normalize(regolith::cross(DVec3(0, 0, 1), up));
	const DVec3 s_b = regolith::cross(e_b, up);
	const Vector3 e_r = regolith::to_v(f.to_local_vec(e_b));
	const Vector3 s_r = regolith::to_v(f.to_local_vec(s_b));
	const Vector3 origin = regolith::to_v(f.to_local_point(up * (datum + ground_alt_)) - eye_r);
	shadow_mat_->set_shader_parameter("shadow_origin", origin);
	shadow_mat_->set_shader_parameter("shadow_e", e_r);
	shadow_mat_->set_shader_parameter("shadow_s", s_r);
	shadow_mat_->set_shader_parameter("shadow_size", real_t(SHADOW_M));
	b->set_material_param("cloud_origin", origin);
	b->set_material_param("cloud_e", e_r);
	b->set_material_param("cloud_s", s_r);
	b->set_material_param("has_cloud_shadow", clouds_visible_ && ready_.load());
}

void RegolithWeather::_process(double p_delta) {
	RegolithSystem *sys = system();
	RegolithBody *b = body();
	if (!sys || !b) {
		return;
	}
	Camera3D *cam = get_viewport()->get_camera_3d();
	const Vector3 eye = cam ? cam->get_global_position() : Vector3();
	double px = 0.0, py = 0.0;
	if (!started_) {
		if (!b->ensure_open()) {
			return;
		}
		{
			std::lock_guard<std::mutex> guard(mutex_);
			in_time_ = sys->get_time();
		}
		start();
		last_time_ = sys->get_time();
	}
	locate(eye, px, py);
	{
		std::lock_guard<std::mutex> guard(mutex_);
		in_time_ = sys->get_time();
		in_px_ = px;
		in_py_ = py;
	}
	update_clouds();
	// the dust over you, eased, for the sky, fog and ground
	if (ready_) {
		const double tau = get_tau(eye);
		const double k = 1.0 - std::exp(-p_delta * 0.4);
		tau_eased_ = tau_eased_ < 0.0 ? tau : tau_eased_ + (tau - tau_eased_) * k;
		if (RegolithAtmosphere *atmo = Object::cast_to<RegolithAtmosphere>(get_node_or_null(atmosphere_path_))) {
			atmo->set_dust_tau(tau_eased_);
		}
	}
}

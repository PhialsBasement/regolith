#include "regolith_weather.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <numbers>

#ifdef __linux__
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#endif

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/texture2d.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/core/class_db.hpp>

#include "convert.hpp"
#include "regolith/ephemeris.hpp"
#include "regolith_atmosphere.hpp"
#include "regolith_body.hpp"
#include "regolith_system.hpp"

using namespace godot;
using regolith::Climate;
using regolith::DVec3;

namespace {

constexpr double PI = std::numbers::pi;
constexpr double DEG = PI / 180.0;
constexpr int TOPO_LEVEL = 1; // ~40 km samples for 2 deg cells
constexpr int REFRESH_MS = 250;

regolith::ClimateConfig config_from(const Dictionary &p_d) {
	regolith::ClimateConfig c;
	c.radius = double(p_d.get("radius", c.radius));
	c.gravity = double(p_d.get("gravity", c.gravity));
	c.sol_s = double(p_d.get("sol_s", c.sol_s));
	c.obliquity_deg = double(p_d.get("obliquity_deg", c.obliquity_deg));
	c.eccentricity = double(p_d.get("eccentricity", c.eccentricity));
	c.perihelion_ls = double(p_d.get("perihelion_ls", c.perihelion_ls));
	c.solar_flux = double(p_d.get("solar_flux", c.solar_flux));
	c.scale_height = double(p_d.get("scale_height", c.scale_height));
	c.datum_pressure = double(p_d.get("datum_pressure", c.datum_pressure));
	c.r_gas = double(p_d.get("r_gas", c.r_gas));
	c.gw = int(p_d.get("gw", c.gw));
	c.gh = int(p_d.get("gh", c.gh));
	c.px_per_turn = double(p_d.get("px_per_turn", c.px_per_turn));
	return c;
}

PackedByteArray bytes_of(const std::vector<uint8_t> &p_v) {
	PackedByteArray a;
	a.resize(int64_t(p_v.size()));
	std::memcpy(a.ptrw(), p_v.data(), p_v.size());
	return a;
}

} // namespace

void RegolithWeather::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_system_path", "path"), &RegolithWeather::set_system_path);
	ClassDB::bind_method(D_METHOD("get_system_path"), &RegolithWeather::get_system_path);
	ClassDB::bind_method(D_METHOD("set_body_path", "path"), &RegolithWeather::set_body_path);
	ClassDB::bind_method(D_METHOD("get_body_path"), &RegolithWeather::get_body_path);
	ClassDB::bind_method(D_METHOD("set_atmosphere_path", "path"), &RegolithWeather::set_atmosphere_path);
	ClassDB::bind_method(D_METHOD("get_atmosphere_path"), &RegolithWeather::get_atmosphere_path);
	ClassDB::bind_method(D_METHOD("set_albedo_map_path", "path"), &RegolithWeather::set_albedo_map_path);
	ClassDB::bind_method(D_METHOD("get_albedo_map_path"), &RegolithWeather::get_albedo_map_path);
	ClassDB::bind_method(D_METHOD("set_climatology_path", "path"), &RegolithWeather::set_climatology_path);
	ClassDB::bind_method(D_METHOD("get_climatology_path"), &RegolithWeather::get_climatology_path);
	ClassDB::bind_method(D_METHOD("set_config", "config"), &RegolithWeather::set_config);
	ClassDB::bind_method(D_METHOD("get_config"), &RegolithWeather::get_config);
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
	ClassDB::bind_method(D_METHOD("get_sun_direction"), &RegolithWeather::get_sun_direction);
	ClassDB::bind_method(D_METHOD("get_global_texture"), &RegolithWeather::get_global_texture);
	ClassDB::bind_method(D_METHOD("get_surface_texture"), &RegolithWeather::get_surface_texture);
	ClassDB::bind_method(D_METHOD("inject_dust", "position"), &RegolithWeather::inject_dust);
	ClassDB::bind_method(D_METHOD("is_ready"), &RegolithWeather::is_ready);
	ClassDB::bind_method(D_METHOD("get_stats"), &RegolithWeather::get_stats);
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "system_path"), "set_system_path", "get_system_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "body_path"), "set_body_path", "get_body_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "atmosphere_path"), "set_atmosphere_path", "get_atmosphere_path");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "albedo_map_path", PROPERTY_HINT_FILE), "set_albedo_map_path", "get_albedo_map_path");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "climatology_path", PROPERTY_HINT_FILE), "set_climatology_path",
			"get_climatology_path");
	ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "config"), "set_config", "get_config");
}

RegolithWeather::~RegolithWeather() {
	exit_ = true;
	wake_.notify_all();
	if (thread_.joinable()) {
		thread_.join();
	}
}

void RegolithWeather::_exit_tree() {
	exit_ = true;
	wake_.notify_all();
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
// here; the terrain grid is sampled on the worker.
void RegolithWeather::start() {
	climate_ = std::make_unique<Climate>(config_from(config_));
	const int gw = climate_->GW, gh = climate_->GH;
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
		// rows: ls_bin then six tau columns and an area
		const PackedStringArray lines = FileAccess::get_file_as_string(clim_path_).split("\n");
		for (const String &line : lines) {
			const String t = line.strip_edges();
			if (t.is_empty() || t.begins_with("#")) {
				continue;
			}
			const PackedStringArray p = t.split(" ", false);
			if (p.size() >= 8) {
				std::array<double, 7> row;
				for (int c = 0; c < 7; ++c) {
					row[size_t(c)] = p[c + 1].to_float();
				}
				clim_.push_back(row);
			}
		}
	}
	global_tex_ = ImageTexture::create_from_image(Image::create_empty(gw, gh, false, Image::FORMAT_RGBA8));
	surface_tex_ = ImageTexture::create_from_image(Image::create_empty(gw, gh, false, Image::FORMAT_RGBA8));
	body_ptr_ = body(); // the worker may not look nodes up
	started_ = true;
	thread_ = std::thread([this] { run(); });
}

void RegolithWeather::run() {
#ifdef __linux__
	// below the game and the terrain feeders' main work
	setpriority(PRIO_PROCESS, pid_t(syscall(SYS_gettid)), 10);
#endif
	Climate &c = *climate_;
	std::vector<float> topo;
	body_ptr_->height_grid(c.GW, c.GH, TOPO_LEVEL, topo);
	c.create(topo.data(), lum_.empty() ? nullptr : lum_.data(), clim_);
	double last = -1e300;
	while (!exit_) {
		double now;
		std::vector<std::array<double, 2>> dust;
		{
			std::unique_lock<std::mutex> lock(mutex_);
			wake_.wait_for(lock, std::chrono::milliseconds(REFRESH_MS));
			now = in_time_;
			dust.swap(in_dust_);
		}
		if (exit_) {
			break;
		}
		for (const auto &d : dust) {
			c.inject_dust(d[0], d[1], now);
		}
		if (now == last && dust.empty()) {
			continue;
		}
		last = now;
		const auto t0 = std::chrono::steady_clock::now();
		c.update(now);
		publish(c);
		const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
		{
			std::lock_guard<std::mutex> guard(mutex_);
			stat_update_ms_ = ms;
			stat_time_ = now;
			++stat_updates_;
		}
		ready_ = true;
	}
}

void RegolithWeather::publish(const Climate &p_c) {
	Published p;
	const size_t n = size_t(p_c.N);
	p.dust = p_c.dust;
	p.cloud.resize(n);
	p.q[2].resize(n);
	for (size_t k = 0; k < n; ++k) {
		float ice = 0.0f;
		for (int l = 0; l < Climate::NL; ++l) {
			ice += p_c.ice[size_t(l)][k];
		}
		p.q[2][k] = ice;
		p.cloud[k] = p_c.tau_low[k] + p_c.tau_high[k];
	}
	p.su = p_c.su;
	p.sv = p_c.sv;
	p.lu = p_c.lu;
	p.lv = p_c.lv;
	p.q[0] = p_c.tau_low;
	p.q[1] = p_c.tau_high;
	p.qv = p_c.wv;
	p.ts = p_c.ts;
	p.tair = p_c.tair;
	p.p = p_c.ps;
	p.co2 = p_c.co2;
	p.frost = p_c.frost;
	p_c.pack_global(p.global);
	p_c.pack_surface(p.surface);
	p.tau_levels.resize(size_t(Climate::NL) * n);
	for (int l = 0; l < Climate::NL; ++l) {
		std::memcpy(p.tau_levels.data() + size_t(l) * n, p_c.tau[size_t(l)].data(), n * sizeof(float));
	}
	p.ls = p_c.ls;
	p.fresh = true;
	std::lock_guard<std::mutex> guard(mutex_);
	p.version = ++pub_version_;
	pub_ = std::move(p);
}

bool RegolithWeather::cloud_field(uint64_t &r_version, std::vector<float> &r_tau, std::vector<float> &r_topo,
		int &r_gw, int &r_gh) const {
	std::lock_guard<std::mutex> guard(mutex_);
	if (!climate_ || !ready_ || pub_.version == r_version || pub_.tau_levels.empty()) {
		return false;
	}
	r_version = pub_.version;
	r_tau = pub_.tau_levels;
	// fixed after create(), which precedes ready_
	r_topo = climate_->topo;
	r_gw = climate_->GW;
	r_gh = climate_->GH;
	return true;
}

bool RegolithWeather::locate(const Vector3 &p_pos, double &r_px, double &r_py, DVec3 *r_dir) const {
	const RegolithSystem *sys = system();
	const RegolithBody *b = body();
	if (!sys || !b || !climate_) {
		return false;
	}
	const DVec3 d = regolith::normalize(sys->body_point(b, p_pos));
	const double lat = std::atan2(d.z, std::hypot(d.x, d.y)) / DEG;
	const double lon = std::atan2(d.y, d.x) / DEG;
	r_px = (lon + 180.0) / 360.0 * climate_->W_PX;
	r_py = (90.0 - lat) / 180.0 * climate_->H_PX;
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
	return climate_->sample(p_f, px, py);
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
	return std::isnan(d) ? 0.35 : Climate::TAU_FLOOR + Climate::TAU_PER_DUST * d;
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
	return to_render(dir, climate_->sample(pub_.lu, px, py), -climate_->sample(pub_.lv, px, py));
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
	const double r = climate_ ? climate_->config().r_gas : 191.8;
	return get_pressure(p_pos) / r / std::max(t, 100.0);
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
	return Vector3(real_t(climate_->sample(pub_.q[0], px, py)), real_t(climate_->sample(pub_.q[1], px, py)),
			real_t(climate_->sample(pub_.q[2], px, py)));
}

Vector3 RegolithWeather::get_sun_direction() const {
	const RegolithSystem *sys = system();
	return sys ? regolith::to_v(regolith::mars_sun_direction(sys->get_time())) : Vector3();
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
	{
		std::lock_guard<std::mutex> guard(mutex_);
		in_dust_.push_back({ px, py });
	}
	wake_.notify_all();
}

Dictionary RegolithWeather::get_stats() const {
	std::lock_guard<std::mutex> guard(mutex_);
	Dictionary d;
	d["ready"] = ready_.load();
	d["update_ms"] = stat_update_ms_;
	d["sim_time"] = stat_time_;
	d["updates"] = int64_t(stat_updates_);
	d["ls"] = pub_.ls;
	return d;
}

void RegolithWeather::upload_maps() {
	std::lock_guard<std::mutex> guard(mutex_);
	if (!pub_.fresh) {
		return;
	}
	const int gw = climate_->GW, gh = climate_->GH;
	global_tex_->update(Image::create_from_data(gw, gh, false, Image::FORMAT_RGBA8, bytes_of(pub_.global)));
	surface_tex_->update(Image::create_from_data(gw, gh, false, Image::FORMAT_RGBA8, bytes_of(pub_.surface)));
	pub_.fresh = false;
}

void RegolithWeather::_process(double p_delta) {
	RegolithSystem *sys = system();
	RegolithBody *b = body();
	if (!sys || !b) {
		return;
	}
	if (!started_) {
		if (!b->ensure_open()) {
			return;
		}
		{
			std::lock_guard<std::mutex> guard(mutex_);
			in_time_ = sys->get_time();
		}
		start();
	}
	{
		std::lock_guard<std::mutex> guard(mutex_);
		in_time_ = sys->get_time();
	}
	upload_maps();
	// the dust over the eye, eased, for the sky, fog and ground
	if (ready_) {
		Camera3D *cam = get_viewport()->get_camera_3d();
		const Vector3 eye = cam ? cam->get_global_position() : Vector3();
		const double tau = get_tau(eye);
		const double k = 1.0 - std::exp(-p_delta * 0.4);
		tau_eased_ = tau_eased_ < 0.0 ? tau : tau_eased_ + (tau - tau_eased_) * k;
		if (RegolithAtmosphere *atmo = Object::cast_to<RegolithAtmosphere>(get_node_or_null(atmosphere_path_))) {
			atmo->set_dust_tau(tau_eased_);
		}
	}
}

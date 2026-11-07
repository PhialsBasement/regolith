#include "regolith_body.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <numbers>
#include <limits>
#include <thread>
#include <atomic>

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/character_body3d.hpp>
#include <godot_cpp/classes/collision_object3d.hpp>
#include <godot_cpp/classes/engine.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/mesh.hpp>
#include <godot_cpp/classes/physics_server3d.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/rendering_server.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>
#include <godot_cpp/classes/scene_tree.hpp>
#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/classes/world3d.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/packed_vector4_array.hpp>
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/vector4.hpp>

#include "convert.hpp"
#include "regolith/cube.hpp"
#include "regolith/ephemeris.hpp"
#include "regolith_collider.hpp"
#include "regolith_engine.hpp"
#include "regolith_system.hpp"

using namespace godot;
using regolith::DVec3;
using regolith::HeightTilePtr;
using regolith::TileKey;

namespace {

constexpr int PINNED_LEVEL = 3;
constexpr int UPLOADS_PER_FRAME = 24;
constexpr int PATCH_BUILDS_PER_TICK = 4;
constexpr uint64_t PATCH_KEEP_TICKS = 180;
// Terrain shades itself from horizon maps; it casts into the
// shadow maps only near the camera, for the objects there.
constexpr double SHADOW_RANGE = 300.0;
// Re-anchor the push frame when the camera strays this far.
constexpr double PUSH_RADIUS = 2000.0;
constexpr float NO_MORPH = 1e30f;

// Patches are virtual tiles two levels below the collision
// tiles: a 4x4 split of each.
constexpr int PATCH_SPLIT_LEVELS = 2;

std::string path_of(const String &p_res) {
	return ProjectSettings::get_singleton()->globalize_path(p_res).utf8().get_data();
}

TileKey key_at(const regolith::FaceCoord &p_fc, int p_level) {
	const double tiles = double(uint64_t(1) << p_level);
	auto cell = [&](double p_c) { return uint32_t(std::clamp(std::floor((p_c + 1.0) * 0.5 * tiles), 0.0, tiles - 1.0)); };
	return TileKey::make(p_fc.face, p_level, cell(p_fc.s), cell(p_fc.t));
}

// Clipmap page table: a W x W window of tiles per face and level.
constexpr int PAGE_WINDOW = 8;
// rings past the finest-data ring sample level 0 at strides up to 16
constexpr int STRIDE_RINGS = 4;

// One axis of an unfolded face0 grid walked over the cube, in
// cube units (2g - m): along-axis and normal coordinates, and
// their rates per unit of x. Same as the clipmap shader's walk.
struct Walk {
	int64_t a;
	int64_t f;
	int da;
	int df;
};

Walk walk(int64_t p_x, int64_t p_m) {
	p_x = std::clamp(p_x, -5 * p_m, 5 * p_m);
	if (p_x > p_m) {
		const int64_t o = p_x - p_m;
		return o <= 2 * p_m ? Walk{ p_m, p_m - o, 0, -1 } : Walk{ 4 * p_m - p_x, -p_m, -1, 0 };
	}
	if (p_x < -p_m) {
		const int64_t o = -p_m - p_x;
		return o <= 2 * p_m ? Walk{ -p_m, p_m - o, 0, 1 } : Walk{ -4 * p_m - p_x, -p_m, -1, 0 };
	}
	return { p_x, p_m, 1, 0 };
}

// Cube point of face0 grid point (gx, gy), which may lie past
// the face: the overshoot folds onto the neighbouring faces.
regolith::IVec3 fold_grid(int p_face, int64_t p_gx, int64_t p_gy, int64_t p_m) {
	const regolith::FaceAxes &a = regolith::face_axes(p_face);
	const Walk wu = walk(2 * p_gx - p_m, p_m);
	const int64_t y = 2 * p_gy - p_m;
	int64_t cv, cf;
	if (wu.da == 1) {
		const Walk wv = walk(y, p_m);
		cv = wv.a;
		cf = wv.f;
	} else {
		// past a corner: the rest folds into the normal axis
		cv = std::clamp(y, -p_m, p_m);
		cf = wu.f - std::abs(y - cv);
	}
	return a.f * cf + a.u * wu.a + a.v * cv;
}

// fold_grid's cube units per face0 grid unit along x and y
void fold_rates(int p_face, int64_t p_gx, int64_t p_gy, int64_t p_m, regolith::IVec3 &r_ex, regolith::IVec3 &r_ey) {
	const regolith::FaceAxes &a = regolith::face_axes(p_face);
	const Walk wu = walk(2 * p_gx - p_m, p_m);
	r_ex = (a.u * wu.da + a.f * wu.df) * 2;
	const int64_t y = 2 * p_gy - p_m;
	if (wu.da == 1) {
		const Walk wv = walk(y, p_m);
		r_ey = (a.v * wv.da + a.f * wv.df) * 2;
	} else {
		r_ey = (y > p_m || y < -p_m) ? a.f * -2 : a.v * 2;
	}
}

int64_t floor_div(int64_t p_a, int64_t p_b) {
	return p_a >= 0 ? p_a / p_b : -((-p_a + p_b - 1) / p_b);
}

int64_t wrap(int64_t p_a, int64_t p_n) {
	const int64_t r = p_a % p_n;
	return r < 0 ? r + p_n : r;
}

// horizon directions, horizon.hpp's order: (i, j) grid steps
constexpr int HORIZON_DIR[8][2] = { { 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 }, { -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 } };

// Bilinear over a tile's samples at fractional (i, j).
double tile_bilinear(const regolith::HeightTile &p_t, const regolith::TileLayout &p_lay, double p_i, double p_j) {
	const double lo = -double(p_lay.border), hi = double(p_lay.intervals + p_lay.border) - 1.0;
	const double fi = std::clamp(p_i, lo, hi), fj = std::clamp(p_j, lo, hi);
	const int i0 = int(std::floor(fi)), j0 = int(std::floor(fj));
	const double ti = fi - i0, tj = fj - j0;
	const double a = p_t.at(p_lay, i0, j0), b = p_t.at(p_lay, i0 + 1, j0);
	const double c = p_t.at(p_lay, i0, j0 + 1), d = p_t.at(p_lay, i0 + 1, j0 + 1);
	return (a * (1 - ti) + b * ti) * (1 - tj) + (c * (1 - ti) + d * ti) * tj;
}

// Bilinear over one RGBA8 channel of a quarter-resolution page
// (geology, horizon) at grid point (i, j): texel i / 4 + 1.
double page_bilinear(const uint8_t *p_page, int p_side, int p_channel, double p_i, double p_j) {
	const double x = std::clamp(p_i / 4.0 + 1.0, 0.0, double(p_side) - 1.001);
	const double y = std::clamp(p_j / 4.0 + 1.0, 0.0, double(p_side) - 1.001);
	const int x0 = int(x), y0 = int(y);
	const double tx = x - x0, ty = y - y0;
	auto at = [&](int p_x, int p_y) { return double(p_page[(size_t(p_y) * p_side + size_t(p_x)) * 4 + size_t(p_channel)]); };
	return (at(x0, y0) * (1 - tx) + at(x0 + 1, y0) * tx) * (1 - ty) + (at(x0, y0 + 1) * (1 - tx) + at(x0 + 1, y0 + 1) * tx) * ty;
}

// Bilinear rho at a face coordinate inside a tile.
double sample_tile(const regolith::HeightTile &p_t, const regolith::TileLayout &p_lay, const regolith::FaceCoord &p_fc) {
	const double tiles = double(uint64_t(1) << p_t.key.level());
	const double gi = (p_fc.s + 1.0) * 0.5 * tiles * p_lay.intervals - double(p_t.key.x()) * p_lay.intervals;
	const double gj = (p_fc.t + 1.0) * 0.5 * tiles * p_lay.intervals - double(p_t.key.y()) * p_lay.intervals;
	const double fi = std::clamp(gi, 0.0, double(p_lay.intervals));
	const double fj = std::clamp(gj, 0.0, double(p_lay.intervals));
	const int i0 = std::min(int(fi), p_lay.intervals - 1);
	const int j0 = std::min(int(fj), p_lay.intervals - 1);
	const double ti = fi - i0;
	const double tj = fj - j0;
	const double a = p_t.at(p_lay, i0, j0), b = p_t.at(p_lay, i0 + 1, j0);
	const double c = p_t.at(p_lay, i0, j0 + 1), d = p_t.at(p_lay, i0 + 1, j0 + 1);
	return (a * (1 - ti) + b * ti) * (1 - tj) + (c * (1 - ti) + d * ti) * tj;
}

} // namespace

// planet_streamer.gd _load_dark_grid: the Viking mosaic at
// 2048 x 1024; darkness ramps in below 92 % of the mosaic's mean
// luminance (0.4128) and saturates by 68 %.
std::shared_ptr<const regolith::DarkField> RegolithBody::load_dark_field(const String &p_path) {
	Ref<Image> img = Image::load_from_file(ProjectSettings::get_singleton()->globalize_path(p_path));
	if (img.is_null() || img->is_empty()) {
		ERR_PRINT("RegolithBody: cannot load dark map " + p_path);
		return nullptr;
	}
	if (img->is_compressed()) {
		img->decompress();
	}
	img->convert(Image::FORMAT_RGB8);
	img->resize(2048, 1024, Image::INTERPOLATE_BILINEAR);
	auto field = std::make_shared<regolith::DarkField>();
	field->w = 2048;
	field->h = 1024;
	field->v.resize(size_t(field->w) * field->h);
	const PackedByteArray d = img->get_data();
	for (size_t k = 0; k < field->v.size(); ++k) {
		const double lum = (double(d[int64_t(k * 3)]) + d[int64_t(k * 3 + 1)] + d[int64_t(k * 3 + 2)]) / 765.0;
		field->v[k] = float(std::clamp((0.92 - lum / 0.4128) / 0.24, 0.0, 1.0));
	}
	return field;
}

// The isomorphic map: the ground shader's layer blend as seen
// from flight height (gravel and bedrock handed to the sands),
// per equirect texel, from the DEM slope and the geology.
// p_layer_means: mean albedo of the six layers, tint applied.
Ref<Image> RegolithBody::bake_iso_map(int p_width, int p_height, const PackedColorArray &p_layer_means, int p_level) {
	if (!ensure_open() || p_layer_means.size() < 6) {
		return Ref<Image>();
	}
	const regolith::TileLayout lay = source_.layout();
	auto radius_at = [&](const DVec3 &p_dir) {
		const regolith::FaceCoord fc = regolith::dir_to_face(p_dir);
		HeightTilePtr t = loader_->produce(key_at(fc, p_level));
		return t ? source_.r_ref() + sample_tile(*t, lay, fc) : source_.r_ref();
	};
	auto sstep = [](double p_a, double p_b, double p_x) {
		const double t = std::clamp((p_x - p_a) / (p_b - p_a), 0.0, 1.0);
		return t * t * (3.0 - 2.0 * t);
	};
	Color mean[6];
	for (int k = 0; k < 6; ++k) {
		mean[k] = p_layer_means[k];
	}
	// macro variation at its mean (noise 0.5)
	const Color macro = Color(0.92f, 0.92f, 0.94f) * Color(1.0f, 0.975f, 0.95f);
	const double step = std::numbers::pi / double(p_height); // one texel
	PackedByteArray out;
	out.resize(int64_t(p_width) * p_height * 3);
	uint8_t *w = out.ptrw();
	std::atomic<int> next{ 0 };
	auto row_job = [&]() {
		for (int y = next++; y < p_height; y = next++) {
			const double lat = std::numbers::pi / 2.0 - (y + 0.5) * step;
			for (int x = 0; x < p_width; ++x) {
				const double lon = (x + 0.5) / p_width * 2.0 * std::numbers::pi - std::numbers::pi;
				const DVec3 d(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
				DVec3 east = regolith::cross(DVec3(0, 0, 1), d);
				east = regolith::length(east) > 1e-9 ? regolith::normalize(east) : DVec3(0, 1, 0);
				const DVec3 north = regolith::cross(d, east);
				const double r0 = radius_at(d);
				const double ds = step * source_.r_ref();
				const double he = radius_at(regolith::normalize(d + east * step)) - r0;
				const double hn = radius_at(regolith::normalize(d + north * step)) - r0;
				const DVec3 n = regolith::normalize(d - east * (he / ds) - north * (hn / ds));
				const double slope = std::clamp(1.0 - regolith::dot(n, d), 0.0, 1.0);
				uint8_t g[4];
				synth_->geology_at(d, g);
				const double ice = g[2] / 255.0, dark = g[3] / 255.0;
				const double w_rock = sstep(0.10, 0.22, slope);
				const double inv_ice = 1.0 - ice;
				const double wgt[6] = { (1.0 - w_rock) * inv_ice * (1.0 - dark), w_rock * inv_ice, 0.0, 0.0, ice,
					(1.0 - w_rock) * inv_ice * dark };
				Color c(0, 0, 0);
				for (int k = 0; k < 6; ++k) {
					c += mean[k] * real_t(wgt[k]);
				}
				c *= macro;
				uint8_t *o = w + (size_t(y) * p_width + size_t(x)) * 3;
				o[0] = uint8_t(std::clamp(c.r * 255.0f, 0.0f, 255.0f));
				o[1] = uint8_t(std::clamp(c.g * 255.0f, 0.0f, 255.0f));
				o[2] = uint8_t(std::clamp(c.b * 255.0f, 0.0f, 255.0f));
			}
		}
	};
	std::vector<std::thread> pool;
	for (unsigned t = 0; t < std::max(1u, std::thread::hardware_concurrency()); ++t) {
		pool.emplace_back(row_job);
	}
	for (std::thread &t : pool) {
		t.join();
	}
	return Image::create_from_data(p_width, p_height, false, Image::FORMAT_RGB8, out);
}

void RegolithBody::set_material_param(const StringName &p_name, const Variant &p_value) {
	// The server takes textures by RID; keep the resource alive.
	Variant value = p_value;
	if (Resource *res = Object::cast_to<Resource>(p_value)) {
		material_refs_[p_name] = p_value;
		value = res->get_rid();
	}
	material_params_[p_name] = value;
	if (material_.is_valid()) {
		RenderingServer::get_singleton()->material_set_param(material_, p_name, value);
	}
}

RegolithBody::RegolithBody() {
	set_process_priority(-50);
	set_physics_process_priority(-50);
}

RegolithBody::~RegolithBody() {
	free_render_resources();
}

void RegolithBody::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_pack_path", "path"), &RegolithBody::set_pack_path);
	ClassDB::bind_method(D_METHOD("get_pack_path"), &RegolithBody::get_pack_path);
	ClassDB::bind_method(D_METHOD("set_datum_path", "path"), &RegolithBody::set_datum_path);
	ClassDB::bind_method(D_METHOD("get_datum_path"), &RegolithBody::get_datum_path);
	ClassDB::bind_method(D_METHOD("set_system_path", "path"), &RegolithBody::set_system_path);
	ClassDB::bind_method(D_METHOD("get_system_path"), &RegolithBody::get_system_path);
	ClassDB::bind_method(D_METHOD("set_gm", "gm"), &RegolithBody::set_gm);
	ClassDB::bind_method(D_METHOD("get_gm"), &RegolithBody::get_gm);
	ClassDB::bind_method(D_METHOD("set_orbit_path", "path"), &RegolithBody::set_orbit_path);
	ClassDB::bind_method(D_METHOD("get_orbit_path"), &RegolithBody::get_orbit_path);
	ClassDB::bind_method(D_METHOD("set_rotation_model", "model"), &RegolithBody::set_rotation_model);
	ClassDB::bind_method(D_METHOD("get_rotation_model"), &RegolithBody::get_rotation_model);
	ClassDB::bind_method(D_METHOD("set_j2", "j2"), &RegolithBody::set_j2);
	ClassDB::bind_method(D_METHOD("get_j2"), &RegolithBody::get_j2);
	ClassDB::bind_method(D_METHOD("set_j2_radius", "metres"), &RegolithBody::set_j2_radius);
	ClassDB::bind_method(D_METHOD("get_j2_radius"), &RegolithBody::get_j2_radius);
	ClassDB::bind_method(D_METHOD("set_rotating_altitude", "metres"), &RegolithBody::set_rotating_altitude);
	ClassDB::bind_method(D_METHOD("get_rotating_altitude"), &RegolithBody::get_rotating_altitude);
	ClassDB::bind_method(D_METHOD("set_max_level", "level"), &RegolithBody::set_max_level);
	ClassDB::bind_method(D_METHOD("get_max_level"), &RegolithBody::get_max_level);
	ClassDB::bind_method(D_METHOD("set_collision_level", "level"), &RegolithBody::set_collision_level);
	ClassDB::bind_method(D_METHOD("get_collision_level"), &RegolithBody::get_collision_level);
	ClassDB::bind_method(D_METHOD("set_dense_levels", "levels"), &RegolithBody::set_dense_levels);
	ClassDB::bind_method(D_METHOD("get_dense_levels"), &RegolithBody::get_dense_levels);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "dense_levels"), "set_dense_levels", "get_dense_levels");
	ClassDB::bind_method(D_METHOD("set_quads", "quads"), &RegolithBody::set_quads);
	ClassDB::bind_method(D_METHOD("get_quads"), &RegolithBody::get_quads);
	ClassDB::bind_method(D_METHOD("set_clip_quads", "quads"), &RegolithBody::set_clip_quads);
	ClassDB::bind_method(D_METHOD("get_clip_quads"), &RegolithBody::get_clip_quads);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "clip_quads"), "set_clip_quads", "get_clip_quads");
	ClassDB::bind_method(D_METHOD("set_range_factor", "factor"), &RegolithBody::set_range_factor);
	ClassDB::bind_method(D_METHOD("get_range_factor"), &RegolithBody::get_range_factor);
	ClassDB::bind_method(D_METHOD("set_page_capacity", "pages"), &RegolithBody::set_page_capacity);
	ClassDB::bind_method(D_METHOD("get_page_capacity"), &RegolithBody::get_page_capacity);
	ClassDB::bind_method(D_METHOD("set_debug_mode", "mode"), &RegolithBody::set_debug_mode);
	ClassDB::bind_method(D_METHOD("get_debug_mode"), &RegolithBody::get_debug_mode);
	ClassDB::bind_method(D_METHOD("get_stats"), &RegolithBody::get_stats);
	ClassDB::bind_method(D_METHOD("get_geology", "lat_deg", "lon_deg"), &RegolithBody::get_geology);
	ClassDB::bind_method(D_METHOD("set_synthesis", "profile"), &RegolithBody::set_synthesis);
	ClassDB::bind_method(D_METHOD("get_synthesis"), &RegolithBody::get_synthesis);
	ClassDB::bind_method(D_METHOD("set_dark_map_path", "path"), &RegolithBody::set_dark_map_path);
	ClassDB::bind_method(D_METHOD("get_dark_map_path"), &RegolithBody::get_dark_map_path);
	ClassDB::bind_method(D_METHOD("set_material_param", "name", "value"), &RegolithBody::set_material_param);
	ClassDB::bind_method(D_METHOD("bake_iso_map", "width", "height", "layer_means", "level"), &RegolithBody::bake_iso_map);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "synthesis", PROPERTY_HINT_ENUM, "None,Mars,Phobos"), "set_synthesis", "get_synthesis");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "dark_map_path", PROPERTY_HINT_FILE, "*.jpg,*.png"), "set_dark_map_path",
			"get_dark_map_path");
	ClassDB::bind_method(D_METHOD("is_collision_ready_at", "position"), &RegolithBody::is_collision_ready_at);
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "pack_path", PROPERTY_HINT_FILE, "*.rgp"), "set_pack_path", "get_pack_path");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "datum_path", PROPERTY_HINT_FILE, "*.rgp"), "set_datum_path", "get_datum_path");
	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "system"), "set_system_path", "get_system_path");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "gm"), "set_gm", "get_gm");
	ADD_PROPERTY(PropertyInfo(Variant::STRING, "orbit_path", PROPERTY_HINT_FILE, "*.rge"), "set_orbit_path", "get_orbit_path");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "rotation_model", PROPERTY_HINT_ENUM, "Mars IAU,Synchronous"), "set_rotation_model",
			"get_rotation_model");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "j2"), "set_j2", "get_j2");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "j2_radius"), "set_j2_radius", "get_j2_radius");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rotating_altitude"), "set_rotating_altitude", "get_rotating_altitude");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "max_level"), "set_max_level", "get_max_level");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "collision_level"), "set_collision_level", "get_collision_level");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "quads"), "set_quads", "get_quads");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "range_factor"), "set_range_factor", "get_range_factor");	ADD_PROPERTY(PropertyInfo(Variant::INT, "page_capacity"), "set_page_capacity", "get_page_capacity");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "debug_mode", PROPERTY_HINT_ENUM, "Shaded,Levels,Morph,Normals"), "set_debug_mode", "get_debug_mode");
}

RegolithSystem *RegolithBody::system() const {
	return Object::cast_to<RegolithSystem>(get_node_or_null(system_path_));
}

bool RegolithBody::ensure_open() {
	if (opened_) {
		return true;
	}
	if (!source_.open(path_of(pack_path_))) {
		ERR_PRINT("RegolithBody: cannot open pack " + pack_path_);
		return false;
	}
	if (!orbit_path_.is_empty()) {
		has_orbit_ = orbit_.open(path_of(orbit_path_));
		if (!has_orbit_) {
			ERR_PRINT("RegolithBody: cannot open orbit " + orbit_path_);
		}
	}
	if (!datum_path_.is_empty() && datum_source_.open(path_of(datum_path_))) {
		has_datum_ = true;
		datum_level_ = datum_source_.max_level();
		const uint32_t n = 1u << datum_level_;
		for (int face = 0; face < regolith::FACE_COUNT; ++face) {
			for (uint32_t x = 0; x < n; ++x) {
				for (uint32_t y = 0; y < n; ++y) {
					auto t = std::make_shared<regolith::HeightTile>();
					const TileKey k = TileKey::make(face, datum_level_, x, y);
					if (datum_source_.load(k, *t)) {
						datum_tiles_[k.bits] = t;
					}
				}
			}
		}
	}
	regolith::SynthConfig sc;
	sc.profile = synthesis_;
	sc.r_ref = source_.r_ref();
	sc.intervals = source_.layout().intervals;
	synth_ = std::make_unique<regolith::Synth>(sc);
	if (synthesis_ == 1 && !dark_map_path_.is_empty()) {
		synth_->set_dark(load_dark_field(dark_map_path_));
	}
	cache_ = std::make_unique<regolith::TileCache>(size_t(page_capacity_) * 2, PINNED_LEVEL);
	// Half the cores: the rest stay free for the game.
	const int threads = std::max(1, int(std::thread::hardware_concurrency()) / 2);
	loader_ = std::make_unique<regolith::TileLoader>(source_, *cache_, threads, synth_.get());

	// The never-a-hole set, loaded before the first frame.
	float lowest = std::numeric_limits<float>::infinity();
	for (int face = 0; face < regolith::FACE_COUNT; ++face) {
		for (int level = 0; level <= std::min(PINNED_LEVEL, source_.max_level()); ++level) {
			const uint32_t n = 1u << level;
			for (uint32_t x = 0; x < n; ++x) {
				for (uint32_t y = 0; y < n; ++y) {
					if (HeightTilePtr t = loader_->produce(TileKey::make(face, level, x, y))) {
						lowest = std::min(lowest, t->min);
					}
				}
			}
		}
	}
	occluder_radius_ = source_.r_ref() + double(lowest);

	// Main thread only: ranges of uploaded pages, no locking.
	selector_.rho_bounds = [this](TileKey p_k, float &r_lo, float &r_hi) {
		auto it = page_bounds_.find(p_k.bits);
		if (it != page_bounds_.end()) {
			r_lo = it->second.first;
			r_hi = it->second.second;
			return true;
		}
		if (p_k.level() <= source_.max_level() && source_.bounds(p_k, r_lo, r_hi)) {
			// Synthesis may reach past the pack's bounds.
			r_lo -= float(synth_->bounds_pad());
			r_hi += float(synth_->bounds_pad());
			return true;
		}
		return false;
	};
	selector_.ready = [this](TileKey p_k) { return pages_.count(p_k.bits) != 0; };
	opened_ = true;
	return true;
}

double RegolithBody::surface_radius(const DVec3 &p_dir, bool p_exact) {
	if (!ensure_open()) {
		return 0.0;
	}
	const regolith::FaceCoord fc = regolith::dir_to_face(p_dir);
	HeightTilePtr tile;
	if (p_exact) {
		tile = loader_->produce(key_at(fc, collision_level_));
	} else {
		for (int level = collision_level_; level >= 0 && !tile; --level) {
			tile = cache_->get(key_at(fc, level));
		}
	}
	if (!tile) {
		return std::numeric_limits<double>::quiet_NaN();
	}
	return source_.r_ref() + sample_tile(*tile, source_.layout(), fc);
}

void RegolithBody::height_grid(int p_w, int p_h, int p_level, std::vector<float> &r_out) {
	r_out.assign(size_t(p_w) * size_t(p_h), 0.0f);
	if (!opened_) {
		return;
	}
	const regolith::TileLayout lay = source_.layout();
	constexpr double DEG = std::numbers::pi / 180.0;
	for (int j = 0; j < p_h; ++j) {
		const double lat = (90.0 - (j + 0.5) / p_h * 180.0) * DEG;
		for (int i = 0; i < p_w; ++i) {
			const double lon = ((i + 0.5) / p_w * 360.0 - 180.0) * DEG;
			const DVec3 dir(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
			const regolith::FaceCoord fc = regolith::dir_to_face(dir);
			HeightTilePtr t = loader_->produce(key_at(fc, p_level));
			const double r = t ? source_.r_ref() + sample_tile(*t, lay, fc) : source_.r_ref();
			r_out[size_t(j) * p_w + size_t(i)] = float(r - datum_radius(dir));
		}
	}
}

double RegolithBody::datum_radius(const DVec3 &p_dir) const {
	if (!has_datum_) {
		return source_.r_ref();
	}
	const regolith::FaceCoord fc = regolith::dir_to_face(p_dir);
	auto it = datum_tiles_.find(key_at(fc, datum_level_).bits);
	if (it == datum_tiles_.end()) {
		return source_.r_ref();
	}
	return source_.r_ref() + sample_tile(*it->second, datum_source_.layout(), fc);
}

void RegolithBody::_ready() {
	if (Engine::get_singleton()->is_editor_hint()) {
		return;
	}
	set_as_top_level(true);
	set_global_transform(Transform3D());
	if (ensure_open()) {
		create_render_resources();
	}
}

void RegolithBody::create_render_resources() {
	RenderingServer *rs = RenderingServer::get_singleton();
	regolith_engine_warn_once();
	const String shader_path = regolith_terrain_shader(clip_quads_ > 0 ? "regolith_clipmap" : "regolith_terrain");
	shader_ = ResourceLoader::get_singleton()->load(shader_path);
	if (shader_.is_null()) {
		ERR_PRINT("RegolithBody: missing " + shader_path);
		return;
	}
	const regolith::TileLayout lay = source_.layout();
	material_ = rs->material_create();
	rs->material_set_shader(material_, shader_->get_rid());
	Ref<Image> blank = Image::create_empty(lay.side(), lay.side(), false, Image::FORMAT_RF);
	TypedArray<Image> layers;
	for (int i = 0; i < page_capacity_; ++i) {
		layers.push_back(blank);
	}
	atlas_ = rs->texture_2d_layered_create(layers, RenderingServer::TEXTURE_LAYERED_2D_ARRAY);
	rs->material_set_param(material_, "heights", atlas_);
	const int gs = regolith::Synth::geology_side(lay);
	Ref<Image> blank_geo = Image::create_empty(gs, gs, false, Image::FORMAT_RGBA8);
	TypedArray<Image> geo_layers;
	for (int i = 0; i < page_capacity_; ++i) {
		geo_layers.push_back(blank_geo);
	}
	geology_atlas_ = rs->texture_2d_layered_create(geo_layers, RenderingServer::TEXTURE_LAYERED_2D_ARRAY);
	rs->material_set_param(material_, "geology", geology_atlas_);
	const int hs = regolith::horizon_side(lay);
	Ref<Image> blank_hor = Image::create_empty(hs, hs, false, Image::FORMAT_RGBA8);
	TypedArray<Image> hor_layers;
	for (int i = 0; i < page_capacity_; ++i) {
		hor_layers.push_back(blank_hor);
	}
	for (int k = 0; k < 2; ++k) {
		horizon_atlas_[k] = rs->texture_2d_layered_create(hor_layers, RenderingServer::TEXTURE_LAYERED_2D_ARRAY);
	}
	rs->material_set_param(material_, "horizon_a", horizon_atlas_[0]);
	rs->material_set_param(material_, "horizon_b", horizon_atlas_[1]);
	rs->material_set_param(material_, "horizon_side", hs);
	rs->material_set_param(material_, "geology_side", gs);
	rs->material_set_param(material_, "face_edge_m", std::numbers::pi / 2.0 * source_.r_ref());
	const Array keys = material_params_.keys();
	for (int i = 0; i < keys.size(); ++i) {
		rs->material_set_param(material_, keys[i], material_params_[keys[i]]);
	}
	rs->material_set_param(material_, "intervals", lay.intervals);
	rs->material_set_param(material_, "border", lay.border);
	rs->material_set_param(material_, "debug_mode", debug_mode_);
	if (clip_quads_ > 0) {
		const int w = PAGE_WINDOW * regolith::FACE_COUNT, h = PAGE_WINDOW * (max_level_ + 1);
		page_bytes_.resize(int64_t(w) * h * 4 * int64_t(sizeof(float)));
		float *p = reinterpret_cast<float *>(page_bytes_.ptrw());
		for (int64_t i = 0; i < int64_t(w) * h; ++i) {
			p[i * 4] = -1.0f;
			p[i * 4 + 1] = p[i * 4 + 2] = p[i * 4 + 3] = 0.0f;
		}
		page_table_ = rs->texture_2d_create(Image::create_from_data(w, h, false, Image::FORMAT_RGBAF, page_bytes_));
		page_origin_.assign(size_t(regolith::FACE_COUNT) * (max_level_ + 1) * 2, INT64_MIN);
		page_dirty_ = true;
		rs->material_set_param(material_, "page_table", page_table_);
		rs->material_set_param(material_, "page_window", PAGE_WINDOW);
		rs->material_set_param(material_, "clip_quads", clip_quads_);
		mesh_ring_ = make_ring_mesh(clip_quads_, false);
		mesh_ring_full_ = make_ring_mesh(clip_quads_, true);
		const RID scenario = get_world_3d()->get_scenario();
		rings_.assign(size_t(max_level_ + 1 + STRIDE_RINGS), Ring());
		for (Ring &r : rings_) {
			r.instance = rs->instance_create2(mesh_ring_, scenario);
			// owned by this body (picking, per-draw profiling)
			rs->instance_attach_object_instance_id(r.instance, get_instance_id());
			rs->instance_set_visible(r.instance, false);
			rs->instance_geometry_set_cast_shadows_setting(r.instance, RenderingServer::SHADOW_CASTING_SETTING_OFF);
		}
		// ring textures, one layer per ring
		ring_t_ = clip_quads_ + 4;
		ring_tq_ = clip_quads_ / 4 + 4;
		auto layered = [&](int p_side, Image::Format p_format) {
			Ref<Image> blank = Image::create_empty(p_side, p_side, false, p_format);
			TypedArray<Image> layers;
			for (size_t i = 0; i < rings_.size(); ++i) {
				layers.push_back(blank);
			}
			return rs->texture_2d_layered_create(layers, RenderingServer::TEXTURE_LAYERED_2D_ARRAY);
		};
		ring_heights_ = layered(ring_t_, Image::FORMAT_RF);
		ring_horizon_[0] = layered(ring_tq_, Image::FORMAT_RGBA8);
		ring_horizon_[1] = layered(ring_tq_, Image::FORMAT_RGBA8);
		ring_geology_ = layered(ring_tq_, Image::FORMAT_RGBA8);
		rs->material_set_param(material_, "ring_heights", ring_heights_);
		rs->material_set_param(material_, "ring_horizon_a", ring_horizon_[0]);
		rs->material_set_param(material_, "ring_horizon_b", ring_horizon_[1]);
		rs->material_set_param(material_, "ring_geology", ring_geology_);
		rs->material_set_param(material_, "ring_size", ring_t_);
		rs->material_set_param(material_, "ring_qsize", ring_tq_);
	} else {
		mesh_coarse_ = make_grid_mesh(quads_);
		mesh_fine_ = make_grid_mesh(lay.intervals);
	}
	free_layers_.clear();
	for (int i = page_capacity_ - 1; i >= 0; --i) {
		free_layers_.push_back(i);
	}
}

RID RegolithBody::make_grid_mesh(int p_quads) {
	RenderingServer *rs = RenderingServer::get_singleton();
	const int q = p_quads;
	PackedVector3Array verts;
	PackedInt32Array idx;
	auto vid = [q](int p_i, int p_j) { return p_j * (q + 1) + p_i; };
	for (int j = 0; j <= q; ++j) {
		for (int i = 0; i <= q; ++i) {
			verts.push_back(Vector3(real_t(i), 0, real_t(j)));
		}
	}
	for (int j = 0; j < q; ++j) {
		for (int i = 0; i < q; ++i) {
			const int a = vid(i, j), b = vid(i + 1, j), c = vid(i + 1, j + 1), d = vid(i, j + 1);
			idx.append_array(PackedInt32Array({ a, c, b, a, d, c }));
		}
	}
	// Skirts: each boundary vertex gets a copy flagged y = -1 that
	// the shader pushes down, hiding cracks while tiles stream.
	std::vector<int> ring;
	for (int i = 0; i < q; ++i) {
		ring.push_back(vid(i, 0));
	}
	for (int j = 0; j < q; ++j) {
		ring.push_back(vid(q, j));
	}
	for (int i = q; i > 0; --i) {
		ring.push_back(vid(i, q));
	}
	for (int j = q; j > 0; --j) {
		ring.push_back(vid(0, j));
	}
	const int base = verts.size();
	for (int v : ring) {
		Vector3 p = verts[v];
		p.y = -1;
		verts.push_back(p);
	}
	const int count = int(ring.size());
	for (int k = 0; k < count; ++k) {
		const int e0 = ring[size_t(k)], e1 = ring[size_t((k + 1) % count)];
		const int s0 = base + k, s1 = base + (k + 1) % count;
		idx.append_array(PackedInt32Array({ e0, e1, s0, e1, s1, s0 }));
	}
	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = verts;
	arrays[Mesh::ARRAY_INDEX] = idx;
	RID mesh = rs->mesh_create();
	rs->mesh_add_surface_from_arrays(mesh, RenderingServer::PRIMITIVE_TRIANGLES, arrays);
	rs->mesh_set_custom_aabb(mesh, AABB(Vector3(-1e7, -1e7, -1e7), Vector3(2e7, 2e7, 2e7)));
	rs->mesh_surface_set_material(mesh, 0, material_);
	return mesh;
}

// q x q grid quads. Quads always in the finer ring's hole are
// left out; quads that may be get own corners, UV = the quad,
// for the shader to collapse. p_full: no hole (finest ring).
RID RegolithBody::make_ring_mesh(int p_quads, bool p_full) {
	RenderingServer *rs = RenderingServer::get_singleton();
	const int q = p_quads;
	const int in_lo = q / 4 + 1, in_hi = 3 * q / 4 - 1;
	const int band_lo = q / 4 - 1, band_hi = 3 * q / 4 + 1;
	PackedVector3Array verts;
	PackedVector2Array uvs;
	PackedInt32Array idx;
	auto vid = [q](int p_i, int p_j) { return p_j * (q + 1) + p_i; };
	auto inside = [](int p_i, int p_j, int p_lo, int p_hi) {
		return p_i >= p_lo && p_i + 1 <= p_hi && p_j >= p_lo && p_j + 1 <= p_hi;
	};
	for (int j = 0; j <= q; ++j) {
		for (int i = 0; i <= q; ++i) {
			verts.push_back(Vector3(real_t(i), 0, real_t(j)));
			uvs.push_back(Vector2(-1, -1));
		}
	}
	for (int j = 0; j < q; ++j) {
		for (int i = 0; i < q; ++i) {
			if (!p_full && inside(i, j, in_lo, in_hi)) {
				continue;
			}
			if (!p_full && inside(i, j, band_lo, band_hi)) {
				const int base = verts.size();
				const int corners[4][2] = { { 0, 0 }, { 1, 0 }, { 1, 1 }, { 0, 1 } };
				for (const auto &c : corners) {
					verts.push_back(Vector3(real_t(i + c[0]), 0, real_t(j + c[1])));
					uvs.push_back(Vector2(real_t(i), real_t(j)));
				}
				idx.append_array(PackedInt32Array({ base, base + 2, base + 1, base, base + 3, base + 2 }));
				continue;
			}
			const int a = vid(i, j), b = vid(i + 1, j), c = vid(i + 1, j + 1), d = vid(i, j + 1);
			idx.append_array(PackedInt32Array({ a, c, b, a, d, c }));
		}
	}
	// Skirts on the outer edge, as make_grid_mesh's
	std::vector<int> ring;
	for (int i = 0; i < q; ++i) {
		ring.push_back(vid(i, 0));
	}
	for (int j = 0; j < q; ++j) {
		ring.push_back(vid(q, j));
	}
	for (int i = q; i > 0; --i) {
		ring.push_back(vid(i, q));
	}
	for (int j = q; j > 0; --j) {
		ring.push_back(vid(0, j));
	}
	const int base = verts.size();
	for (int v : ring) {
		Vector3 p = verts[v];
		p.y = -1;
		verts.push_back(p);
		uvs.push_back(Vector2(-1, -1));
	}
	const int count = int(ring.size());
	for (int k = 0; k < count; ++k) {
		const int e0 = ring[size_t(k)], e1 = ring[size_t((k + 1) % count)];
		const int s0 = base + k, s1 = base + (k + 1) % count;
		idx.append_array(PackedInt32Array({ e0, e1, s0, e1, s1, s0 }));
	}
	Array arrays;
	arrays.resize(Mesh::ARRAY_MAX);
	arrays[Mesh::ARRAY_VERTEX] = verts;
	arrays[Mesh::ARRAY_TEX_UV] = uvs;
	arrays[Mesh::ARRAY_INDEX] = idx;
	RID mesh = rs->mesh_create();
	rs->mesh_add_surface_from_arrays(mesh, RenderingServer::PRIMITIVE_TRIANGLES, arrays);
	rs->mesh_set_custom_aabb(mesh, AABB(Vector3(-1e7, -1e7, -1e7), Vector3(2e7, 2e7, 2e7)));
	rs->mesh_surface_set_material(mesh, 0, material_);
	return mesh;
}

void RegolithBody::touch_page(uint64_t p_bits) {
	auto it = pages_.find(p_bits);
	if (it == pages_.end()) {
		return;
	}
	it->second.used = frame_;
	if (!it->second.pinned) {
		lru_.splice(lru_.end(), lru_, it->second.lru);
	}
}

// Per face and level, the W x W tiles nearest the camera's
// projection on that face, written at (tx mod W, ty mod W) as
// (layer or -1, tx, ty). Rebuilt when a window or page changes.
void RegolithBody::update_page_table(const DVec3 &p_cam_dir) {
	const int w_tiles = PAGE_WINDOW;
	const int levels = max_level_ + 1;
	bool moved = false;
	for (int face = 0; face < regolith::FACE_COUNT; ++face) {
		const regolith::FaceAxes &a = regolith::face_axes(face);
		const double w = std::max(regolith::dot(regolith::cast<double>(a.f), p_cam_dir), 1e-3);
		const double s = regolith::unwarp(regolith::dot(regolith::cast<double>(a.u), p_cam_dir) / w);
		const double t = regolith::unwarp(regolith::dot(regolith::cast<double>(a.v), p_cam_dir) / w);
		for (int level = 0; level < levels; ++level) {
			const int64_t n = int64_t(1) << level;
			const int64_t span = std::max<int64_t>(n - w_tiles, 0);
			const int64_t ox = std::clamp(int64_t(std::floor((s + 1.0) * 0.5 * double(n))) - w_tiles / 2, int64_t(0), span);
			const int64_t oy = std::clamp(int64_t(std::floor((t + 1.0) * 0.5 * double(n))) - w_tiles / 2, int64_t(0), span);
			const size_t k = (size_t(face) * levels + size_t(level)) * 2;
			if (page_origin_[k] != ox || page_origin_[k + 1] != oy) {
				page_origin_[k] = ox;
				page_origin_[k + 1] = oy;
				moved = true;
			}
		}
	}
	if (!moved && !page_dirty_) {
		return;
	}
	const int width = w_tiles * regolith::FACE_COUNT;
	float *p = reinterpret_cast<float *>(page_bytes_.ptrw());
	for (int64_t i = 0; i < int64_t(width) * w_tiles * levels; ++i) {
		p[i * 4] = -1.0f;
	}
	for (int face = 0; face < regolith::FACE_COUNT; ++face) {
		for (int level = 0; level < levels; ++level) {
			const int64_t n = int64_t(1) << level;
			const size_t k = (size_t(face) * levels + size_t(level)) * 2;
			for (int64_t tx = page_origin_[k]; tx < std::min(n, page_origin_[k] + w_tiles); ++tx) {
				for (int64_t ty = page_origin_[k + 1]; ty < std::min(n, page_origin_[k + 1] + w_tiles); ++ty) {
					auto it = pages_.find(TileKey::make(face, level, uint32_t(tx), uint32_t(ty)).bits);
					if (it == pages_.end()) {
						continue;
					}
					const int64_t x = face * w_tiles + tx % w_tiles, y = level * w_tiles + ty % w_tiles;
					float *texel = p + (y * width + x) * 4;
					texel[0] = float(it->second.layer);
					texel[1] = float(tx);
					texel[2] = float(ty);
					texel[3] = 1.0f;
				}
			}
		}
	}
	RenderingServer::get_singleton()->texture_2d_update(page_table_,
			Image::create_from_data(width, w_tiles * levels, false, Image::FORMAT_RGBAF, page_bytes_), 0);
	page_dirty_ = false;
}

void RegolithBody::set_debug_mode(int p_v) {
	debug_mode_ = p_v;
	if (material_.is_valid()) {
		RenderingServer::get_singleton()->material_set_param(material_, "debug_mode", debug_mode_);
	}
}

bool RegolithBody::upload(TileKey p_key, bool p_make_horizon) {
	if (pages_.count(p_key.bits)) {
		return true;
	}
	HeightTilePtr tile = cache_->get(p_key);
	if (!tile) {
		return false;
	}
	// the horizon a worker made; without it, back to a worker
	// (computing it here stalled the frame by tens of ms)
	const auto page = p_make_horizon ? loader_->horizon(p_key) : loader_->horizon_ready(p_key);
	if (!page) {
		loader_->request(p_key, std::numeric_limits<float>::max());
		return false;
	}
	int layer;
	if (!free_layers_.empty()) {
		layer = free_layers_.back();
		free_layers_.pop_back();
	} else {
		// least recently used first: if that one is in use, all are
		if (lru_.empty()) {
			return false;
		}
		auto victim = pages_.find(lru_.front());
		if (victim == pages_.end() || victim->second.used + 1 >= frame_) {
			return false;
		}
		layer = victim->second.layer;
		lru_.pop_front();
		pages_.erase(victim);
	}
	PackedByteArray bytes;
	bytes.resize(int64_t(tile->rho.size() * sizeof(float)));
	std::memcpy(bytes.ptrw(), tile->rho.data(), tile->rho.size() * sizeof(float));
	const int side = source_.layout().side();
	Ref<Image> img = Image::create_from_data(side, side, false, Image::FORMAT_RF, bytes);
	RenderingServer::get_singleton()->texture_2d_update(atlas_, img, layer);
	if (!tile->geology.empty()) {
		const int gs = regolith::Synth::geology_side(source_.layout());
		PackedByteArray geo;
		geo.resize(int64_t(tile->geology.size()));
		std::memcpy(geo.ptrw(), tile->geology.data(), tile->geology.size());
		RenderingServer::get_singleton()->texture_2d_update(geology_atlas_,
				Image::create_from_data(gs, gs, false, Image::FORMAT_RGBA8, geo), layer);
	}
	// Made on the workers; only the pinned set is made here.
	{
		const int hs = regolith::horizon_side(source_.layout());
		const size_t bytes_per = size_t(hs) * hs * 4;
		for (int k = 0; k < 2; ++k) {
			PackedByteArray hor;
			hor.resize(int64_t(bytes_per));
			std::memcpy(hor.ptrw(), page->data() + k * bytes_per, bytes_per);
			RenderingServer::get_singleton()->texture_2d_update(horizon_atlas_[k],
					Image::create_from_data(hs, hs, false, Image::FORMAT_RGBA8, hor), layer);
		}
	}
	Page pg;
	pg.layer = layer;
	pg.used = frame_;
	pg.pinned = p_key.level() <= PINNED_LEVEL;
	if (!pg.pinned) {
		pg.lru = lru_.insert(lru_.end(), p_key.bits);
	}
	pages_[p_key.bits] = pg;
	page_dirty_ = true;
	page_bounds_[p_key.bits] = { tile->min, tile->max };
	++stat_uploads_;
	return true;
}

void RegolithBody::upload_finished(int p_budget) {
	for (TileKey k : loader_->take_finished()) {
		to_upload_.push_back(k);
	}
	// Coarse first: a fine page is useless until its parents are up.
	std::sort(to_upload_.begin(), to_upload_.end(),
			[](TileKey p_a, TileKey p_b) { return p_a.level() < p_b.level(); });
	to_upload_.erase(std::unique(to_upload_.begin(), to_upload_.end()), to_upload_.end());
	int done = 0;
	size_t k = 0;
	for (; k < to_upload_.size() && done < p_budget; ++k) {
		if (pages_.count(to_upload_[k].bits) == 0 && upload(to_upload_[k])) {
			++done;
		}
	}
	to_upload_.erase(to_upload_.begin(), to_upload_.begin() + long(k));
}

void RegolithBody::_process(double p_delta) {
	RegolithSystem *sys = system();
	if (!opened_ || !atlas_.is_valid() || !sys) {
		return;
	}
	Viewport *vp = get_viewport();
	Camera3D *cam = vp ? vp->get_camera_3d() : nullptr;
	if (!cam) {
		return;
	}
	++frame_;
	stat_uploads_ = 0;
	const uint64_t t_start = Time::get_singleton()->get_ticks_usec();
	if (frame_ == 1) {
		// Upload the pinned set before anything is drawn.
		for (int face = 0; face < regolith::FACE_COUNT; ++face) {
			for (int level = 0; level <= std::min(PINNED_LEVEL, source_.max_level()); ++level) {
				const uint32_t n = 1u << level;
				for (uint32_t x = 0; x < n; ++x) {
					for (uint32_t y = 0; y < n; ++y) {
						upload(TileKey::make(face, level, x, y), true);
					}
				}
			}
		}
	}
	upload_finished(UPLOADS_PER_FRAME);

	const uint64_t t0 = Time::get_singleton()->get_ticks_usec();
	if (clip_quads_ > 0) {
		draw_clipmap();
		loader_->end_frame();
		stat_select_ms_ = 0.0;
		stat_draw_ms_ = double(Time::get_singleton()->get_ticks_usec() - t0) * 1e-3;
		stat_upload_ms_ = double(t0 - t_start) * 1e-3;
		return;
	}
	const regolith::RenderFrame f = sys->frame_for(this);
	regolith::SelectInput in;
	in.camera = regolith::to_body_point(f, regolith::to_d(cam->get_global_position()));
	in.r_ref = source_.r_ref();
	in.range_factor = range_factor_;
	in.max_level = max_level_;
	in.occluder_radius = occluder_radius_;
	// Detail depends on where the camera is, never where it
	// looks: Godot's own culling decides what is drawn, so
	// turning around changes nothing.
	std::vector<regolith::SelectedNode> nodes;
	std::vector<regolith::WantedTile> wanted;
	selector_.select(in, nodes, wanted);
	for (const regolith::WantedTile &w : wanted) {
		loader_->request(w.key, w.priority);
	}
	loader_->end_frame();
	const uint64_t t1 = Time::get_singleton()->get_ticks_usec();
	stat_select_ms_ = double(t1 - t0) * 1e-3;
	stat_nodes_ = int(nodes.size());
	stat_wanted_ = int(wanted.size());
	draw(nodes);
	const uint64_t t2 = Time::get_singleton()->get_ticks_usec();
	stat_draw_ms_ = double(t2 - t1) * 1e-3;
	stat_upload_ms_ = double(t0 - t_start) * 1e-3;
}

bool RegolithBody::update_push_frame(DVec3 &r_cam_body) {
	RenderingServer *rs = RenderingServer::get_singleton();
	RegolithSystem *sys = system();
	Camera3D *camera = get_viewport() ? get_viewport()->get_camera_3d() : nullptr;
	if (!sys || !camera) {
		return false;
	}
	const regolith::RenderFrame f = sys->frame_for(this);
	const Vector3 cam = camera->get_global_position();

	// Node constants live in a push frame anchored at the camera,
	// so the GPU's float32 only ever sees small numbers. They are
	// re-pushed when the camera strays or the frame is rebuilt;
	// otherwise one shared instance transform carries them.
	const regolith::DVec3 cam_body = regolith::to_body_point(f, regolith::to_d(cam));
	Transform3D xf;
	bool drifted = true;
	if (push_valid_) {
		xf.basis = regolith::basis_between(f, push_frame_);
		xf.origin = regolith::to_v(f.to_local_point(push_frame_.anchor));
		drifted = regolith::length(cam_body - push_frame_.anchor) > PUSH_RADIUS ||
				(xf.basis.get_column(0) - Vector3(1, 0, 0)).length() > 0.01 ||
				(xf.basis.get_column(1) - Vector3(0, 1, 0)).length() > 0.01;
	}
	if (!push_valid_ || push_rebuild_ != sys->rebuild_count() || drifted) {
		push_frame_ = f;
		push_frame_.anchor = cam_body;
		push_valid_ = true;
		push_rebuild_ = sys->rebuild_count();
		++push_id_;
		xf.basis = Basis();
		xf.origin = regolith::to_v(f.to_local_point(push_frame_.anchor));
		// node space -> body-fixed, for the orbital look
		Basis to_body;
		to_body.set_column(0, regolith::to_v(push_frame_.east));
		to_body.set_column(1, regolith::to_v(push_frame_.up));
		to_body.set_column(2, regolith::to_v(push_frame_.south));
		rs->material_set_param(material_, "u_to_body", to_body);
	}
	{
		const RegolithSystem::Pose &bp = sys->pose_of(this);
		const regolith::DVec3 sun = regolith::normalize(regolith::mars_heliocentric(sys->get_time()) * -1.0 - bp.position / 1.495978707e11);
		rs->material_set_param(material_, "sun_body",
				Vector3(real_t(regolith::dot(bp.axes[0], sun)), real_t(regolith::dot(bp.axes[1], sun)), real_t(regolith::dot(bp.axes[2], sun))));
	}
	// the main camera in push-frame metres, for the morph
	rs->material_set_param(material_, "u_eye", regolith::to_v(push_frame_.to_local_point(cam_body)));
	if (!xf.is_equal_approx(push_xform_) || xform_id_ == 0) {
		push_xform_ = xf;
		++xform_id_;
	}
	r_cam_body = cam_body;
	return true;
}

void RegolithBody::draw(const std::vector<regolith::SelectedNode> &p_nodes) {
	RenderingServer *rs = RenderingServer::get_singleton();
	const regolith::TileLayout lay = source_.layout();
	const RID scenario = get_world_3d()->get_scenario();
	const Vector3 cam = get_viewport()->get_camera_3d()->get_global_position();
	DVec3 cam_body;
	if (!update_push_frame(cam_body)) {
		return;
	}
	stat_fine_levels_.clear();
	stat_tiles_.clear();
	for (const regolith::SelectedNode &n : p_nodes) {
		const int level = n.key.level();
		const double dist = std::max(0.0, regolith::length(n.sphere.center - cam_body) - n.sphere.radius);
		const bool fine = level > max_level_ - dense_levels_;
		if (fine) {
			// per level: count, nearest and farthest sphere surface
			auto &fl = stat_fine_levels_[level];
			fl.count++;
			fl.dmin = std::min(fl.dmin, dist);
			fl.dmax = std::max(fl.dmax, dist);
		}

		// Mark the node and its whole ancestry as in use.
		for (TileKey a = n.key;; a = a.parent()) {
			touch_page(a.bits);
			if (a.level() == 0) {
				break;
			}
		}
		const auto self_page = pages_.find(n.key.bits);
		if (self_page == pages_.end()) {
			continue;
		}
		const int self_layer = self_page->second.layer;
		const auto parent_page = level > 0 ? pages_.find(n.key.parent().bits) : pages_.end();
		// Without the parent page there is nothing to morph to.
		const bool morphs = parent_page != pages_.end();
		const int parent_layer = morphs ? parent_page->second.layer : self_layer;
		const int quadrant = int(n.key.x() & 1u) | int((n.key.y() & 1u) << 1);

		auto found = drawn_.find(n.key.bits);
		if (found == drawn_.end()) {
			Drawn d;
			d.fine = fine;
			std::vector<RID> &spare = fine ? spare_fine_ : spare_coarse_;
			if (!spare.empty()) {
				d.instance = spare.back();
				spare.pop_back();
			} else {
				d.instance = rs->instance_create2(fine ? mesh_fine_ : mesh_coarse_, scenario);
				// owned by this body (picking, per-draw profiling)
				rs->instance_attach_object_instance_id(d.instance, get_instance_id());
			}
			found = drawn_.emplace(n.key.bits, d).first;
		}
		Drawn &d = found->second;
		// per drawn tile: render instance, level, grid, distance
		stat_tiles_.push_back({ uint64_t(d.instance.get_id()), level, fine, dist });
		const bool reveal = d.seen == 0;
		d.seen = frame_;
		if (d.xform_id != xform_id_) {
			rs->instance_set_transform(d.instance, push_xform_);
			d.xform_id = xform_id_;
		}
		const Vector3 centre = regolith::to_v(push_frame_.to_local_point(n.sphere.center));
		const real_t r = real_t(n.sphere.radius);
		const bool shadow = (push_xform_.xform(centre) - cam).length() - r < SHADOW_RANGE;
		if (reveal || shadow != d.shadow) {
			rs->instance_geometry_set_cast_shadows_setting(d.instance,
					shadow ? RenderingServer::SHADOW_CASTING_SETTING_ON : RenderingServer::SHADOW_CASTING_SETTING_OFF);
			d.shadow = shadow;
		}
		if (d.self_layer == self_layer && d.parent_layer == parent_layer && d.push_id == push_id_) {
			continue;
		}
		d.self_layer = self_layer;
		d.parent_layer = parent_layer;
		d.push_id = push_id_;
		const RID inst = d.instance;

		float lo = 0.0f, hi = 0.0f;
		selector_.rho_bounds(n.key, lo, hi);
		const double rho_c = 0.5 * (double(lo) + double(hi));
		const regolith::PatchSpec spec{ n.key, lay.intervals };
		const regolith::NodeConstants c = regolith::node_constants(spec, source_.r_ref() + rho_c, push_frame_);
		rs->instance_geometry_set_shader_parameter(inst, "u_center", Vector4(c.center.x, c.center.y, c.center.z, c.r_center));
		rs->instance_geometry_set_shader_parameter(inst, "u_axis_u", Vector4(c.axis_u.x, c.axis_u.y, c.axis_u.z, c.s_c));
		rs->instance_geometry_set_shader_parameter(inst, "u_axis_v", Vector4(c.axis_v.x, c.axis_v.y, c.axis_v.z, c.t_c));
		rs->instance_geometry_set_shader_parameter(inst, "u_qc", Vector4(c.q_center.x, c.q_center.y, c.q_center.z, c.q_len));
		rs->instance_geometry_set_shader_parameter(inst, "u_misc",
				Vector4(c.cos_sc, c.cos_tc, c.inv_m, morphs ? float(n.morph_start) : NO_MORPH));
		rs->instance_geometry_set_shader_parameter(inst, "u_pages",
				Vector4(float(self_layer), float(parent_layer), float(quadrant), morphs ? float(n.morph_end) : NO_MORPH));
		const double skirt = 1.0 + 0.01 * regolith::Selector::node_edge(source_.r_ref(), level);
		rs->instance_geometry_set_shader_parameter(inst, "u_debug",
				Vector4(float(level), float(fine ? lay.intervals : quads_), float(rho_c), float(skirt)));
		push_texture_domains(inst, n.key, rho_c);

		rs->instance_set_custom_aabb(inst, AABB(centre - Vector3(r, r, r), Vector3(r, r, r) * 2));
		if (reveal) {
			rs->instance_set_visible(inst, true);
		}
	}
	// Nodes no longer selected go back to the pool, hidden.
	for (auto it = drawn_.begin(); it != drawn_.end();) {
		if (it->second.seen == frame_) {
			++it;
			continue;
		}
		rs->instance_set_visible(it->second.instance, false);
		(it->second.fine ? spare_fine_ : spare_coarse_).push_back(it->second.instance);
		it = drawn_.erase(it);
	}
}

bool RegolithBody::ring_rho(int p_face, int p_level, int64_t p_gi, int64_t p_gj, float &r_rho) {
	const regolith::TileLayout lay = source_.layout();
	const int I = lay.intervals;
	for (int up = 0; up <= p_level; ++up) {
		const int lv = p_level - up;
		const int64_t n = int64_t(1) << lv;
		const double fi = double(p_gi) / double(int64_t(1) << up), fj = double(p_gj) / double(int64_t(1) << up);
		const int64_t tx = std::clamp(int64_t(std::floor(fi / I)), int64_t(0), n - 1);
		const int64_t ty = std::clamp(int64_t(std::floor(fj / I)), int64_t(0), n - 1);
		const TileKey key = TileKey::make(p_face, lv, uint32_t(tx), uint32_t(ty));
		if (key.bits != memo_key_) {
			memo_tile_ = cache_->get(key);
			memo_key_ = key.bits;
		}
		if (memo_tile_) {
			r_rho = float(tile_bilinear(*memo_tile_, lay, fi - double(tx * I), fj - double(ty * I)));
			return up == 0;
		}
	}
	r_rho = 0.0f;
	return false;
}

void RegolithBody::update_ring_textures(int p_r, const RingGeo &p_g, int p_face0, bool p_pages_changed) {
	Ring &ring = rings_[size_t(p_r)];
	const regolith::TileLayout lay = source_.layout();
	const int I = lay.intervals;
	const int64_t T = ring_t_, TQ = ring_tq_;
	const int64_t s = p_g.s;
	const int64_t N = clip_quads_;
	// window: grid steps from one before the ring's first to two
	// past its last (fragment normals reach one step out)
	const int64_t kx0 = (p_g.cx - N / 2 * s) / s - 1, ky0 = (p_g.cy - N / 2 * s) / s - 1;
	const int64_t qx0 = floor_div(kx0, 4) - 1, qy0 = floor_div(ky0, 4) - 1;
	if (ring.heights.empty()) {
		ring.heights.assign(size_t(T * T), 0.0f);
		ring.hor_a.assign(size_t(TQ * TQ * 4), 0);
		ring.hor_b.assign(size_t(TQ * TQ * 4), 0);
		ring.geo.assign(size_t(TQ * TQ * 4), 0);
	}
	const bool all = ring.tex_face != p_face0 || (p_pages_changed && !ring.complete);
	if (!all && ring.kx0 == kx0 && ring.ky0 == ky0 && ring.qx0 == qx0 && ring.qy0 == qy0) {
		return;
	}
	memo_key_ = UINT64_MAX;
	memo_tile_.reset();
	bool complete = all || ring.complete;
	auto kept = [all](int64_t p_v, int64_t p_o, int64_t p_n) { return !all && p_o != INT64_MIN && p_v >= p_o && p_v < p_o + p_n; };

	for (int64_t ky = ky0; ky < ky0 + T; ++ky) {
		const bool row_kept = kept(ky, ring.ky0, T);
		for (int64_t kx = kx0; kx < kx0 + T; ++kx) {
			if (row_kept && kept(kx, ring.kx0, T)) {
				continue;
			}
			const regolith::FaceSample fs = regolith::canonical_sample(fold_grid(p_face0, kx * s, ky * s, p_g.m), p_g.m);
			float rho;
			complete = ring_rho(fs.face, p_g.level, fs.gi, fs.gj, rho) && complete;
			ring.heights[size_t(wrap(ky, T) * T + wrap(kx, T))] = rho;
		}
	}

	const int hs = regolith::horizon_side(lay), gs = regolith::Synth::geology_side(lay);
	for (int64_t qy = qy0; qy < qy0 + TQ; ++qy) {
		const bool row_kept = kept(qy, ring.qy0, TQ);
		for (int64_t qx = qx0; qx < qx0 + TQ; ++qx) {
			if (row_kept && kept(qx, ring.qx0, TQ)) {
				continue;
			}
			const int64_t gx = 4 * qx * s, gy = 4 * qy * s;
			const regolith::FaceSample fs = regolith::canonical_sample(fold_grid(p_face0, gx, gy, p_g.m), p_g.m);
			// the finest cached tile holding it, and the point in it
			regolith::HeightTilePtr t;
			double li = 0.0, lj = 0.0;
			for (int up = 0; up <= p_g.level && !t; ++up) {
				const int lv = p_g.level - up;
				const int64_t n = int64_t(1) << lv;
				const double fi = double(fs.gi) / double(int64_t(1) << up), fj = double(fs.gj) / double(int64_t(1) << up);
				const int64_t tx = std::clamp(int64_t(std::floor(fi / I)), int64_t(0), n - 1);
				const int64_t ty = std::clamp(int64_t(std::floor(fj / I)), int64_t(0), n - 1);
				t = cache_->get(TileKey::make(fs.face, lv, uint32_t(tx), uint32_t(ty)));
				li = fi - double(tx * I);
				lj = fj - double(ty * I);
				complete = complete && (up > 0 || t);
			}
			const size_t o = size_t(wrap(qy, TQ) * TQ + wrap(qx, TQ)) * 4;
			for (int c = 0; c < 4; ++c) {
				ring.geo[o + size_t(c)] = (t && !t->geology.empty()) ? uint8_t(std::lround(page_bilinear(t->geology.data(), gs, c, li, lj))) : 0;
			}
			// the loader sets a tile's horizon under its lock: ask it
			const auto horizon = t ? loader_->horizon_ready(t->key) : nullptr;
			if (!horizon) {
				complete = false;
				for (int c = 0; c < 4; ++c) {
					ring.hor_a[o + size_t(c)] = ring.hor_b[o + size_t(c)] = 0;
				}
				continue;
			}
			// horizon directions are the page's grid's: turn them to
			// the ring's (face0 grid) where the ring folds over
			regolith::IVec3 ex, ey;
			fold_rates(p_face0, gx, gy, p_g.m, ex, ey);
			const regolith::FaceAxes &ca = regolith::face_axes(fs.face);
			const int64_t sxi = regolith::dot(ca.u, ex) / 2, sxj = regolith::dot(ca.v, ex) / 2;
			const int64_t syi = regolith::dot(ca.u, ey) / 2, syj = regolith::dot(ca.v, ey) / 2;
			for (int ku = 0; ku < 8; ++ku) {
				const int64_t di = sxi * HORIZON_DIR[ku][0] + syi * HORIZON_DIR[ku][1];
				const int64_t dj = sxj * HORIZON_DIR[ku][0] + syj * HORIZON_DIR[ku][1];
				int kc = ku;
				for (int k = 0; k < 8; ++k) {
					if (HORIZON_DIR[k][0] == (di > 0) - (di < 0) && HORIZON_DIR[k][1] == (dj > 0) - (dj < 0)) {
						kc = k;
					}
				}
				const uint8_t *page = horizon->data() + size_t(kc / 4) * size_t(hs) * size_t(hs) * 4;
				(ku < 4 ? ring.hor_a : ring.hor_b)[o + size_t(ku % 4)] = uint8_t(std::lround(page_bilinear(page, hs, kc % 4, li, lj)));
			}
		}
	}
	ring.kx0 = kx0;
	ring.ky0 = ky0;
	ring.qx0 = qx0;
	ring.qy0 = qy0;
	ring.tex_face = p_face0;
	ring.complete = complete;

	RenderingServer *rs = RenderingServer::get_singleton();
	auto bytes = [](const void *p_data, size_t p_size) {
		PackedByteArray b;
		b.resize(int64_t(p_size));
		std::memcpy(b.ptrw(), p_data, p_size);
		return b;
	};
	rs->texture_2d_update(ring_heights_,
			Image::create_from_data(int(T), int(T), false, Image::FORMAT_RF, bytes(ring.heights.data(), ring.heights.size() * sizeof(float))), p_r);
	rs->texture_2d_update(ring_horizon_[0], Image::create_from_data(int(TQ), int(TQ), false, Image::FORMAT_RGBA8, bytes(ring.hor_a.data(), ring.hor_a.size())), p_r);
	rs->texture_2d_update(ring_horizon_[1], Image::create_from_data(int(TQ), int(TQ), false, Image::FORMAT_RGBA8, bytes(ring.hor_b.data(), ring.hor_b.size())), p_r);
	rs->texture_2d_update(ring_geology_, Image::create_from_data(int(TQ), int(TQ), false, Image::FORMAT_RGBA8, bytes(ring.geo.data(), ring.geo.size())), p_r);
}

// Nested N x N rings at data levels max - r, centred on the
// camera in its face's lattice. Constant work: a few rings and
// 25 page checks each, no tree, no per-tile instances.
void RegolithBody::draw_clipmap() {
	RenderingServer *rs = RenderingServer::get_singleton();
	DVec3 cam_body;
	if (!update_push_frame(cam_body)) {
		return;
	}
	const int I = source_.layout().intervals;
	const int N = clip_quads_;
	const int L = max_level_;
	const int levels = L + 1;
	const double r_ref = source_.r_ref();
	const double edge_m = std::numbers::pi / 2.0 * r_ref;
	const DVec3 cam_dir = regolith::normalize(cam_body);
	const regolith::FaceCoord fc = regolith::dir_to_face(cam_dir);
	const int face0 = fc.face;
	update_page_table(cam_dir);

	// the eye's horizon plus a 30 km peak's: rings past it are hidden
	const double alt = std::max(1.0, regolith::length(cam_body) - occluder_radius_);
	const double see = std::sqrt(alt * (2.0 * r_ref + alt)) + std::sqrt(2.0 * r_ref * 30000.0);

	auto in_window = [&](TileKey p_k) {
		const size_t k = (size_t(p_k.face()) * levels + size_t(p_k.level())) * 2;
		return int64_t(p_k.x()) >= page_origin_[k] && int64_t(p_k.x()) < page_origin_[k] + PAGE_WINDOW &&
				int64_t(p_k.y()) >= page_origin_[k + 1] && int64_t(p_k.y()) < page_origin_[k + 1] + PAGE_WINDOW;
	};
	std::vector<RingGeo> geo;
	geo.reserve(rings_.size());
	for (int r = 0; r < int(rings_.size()); ++r) {
		RingGeo g;
		g.level = std::max(L - r, 0);
		g.s = int64_t(1) << std::max(r - L, 0);
		g.m = int64_t(I) << g.level;
		g.spacing = edge_m / double(g.m) * double(g.s);
		// past the horizon, or wider than the fold reaches
		if (r > 0 && (double(N / 4) * g.spacing > see || int64_t(N / 2) * g.s > 2 * g.m)) {
			break;
		}
		// centre: the camera, snapped to every second grid step
		g.cx = 2 * g.s * int64_t(std::llround((fc.s + 1.0) * 0.5 * double(g.m) / double(2 * g.s)));
		g.cy = 2 * g.s * int64_t(std::llround((fc.t + 1.0) * 0.5 * double(g.m) / double(2 * g.s)));
		// its pages and the next level's (the morph), resident and
		// in the page table; 5 x 5 samples hit every tile it covers
		g.ready = true;
		const int64_t half = int64_t(N / 2) * g.s;
		for (int a = 0; a <= 4; ++a) {
			for (int b = 0; b <= 4; ++b) {
				const regolith::IVec3 p = fold_grid(face0, g.cx - half + half * a / 2, g.cy - half + half * b / 2, g.m);
				const regolith::FaceSample fs = regolith::canonical_sample(p, g.m);
				for (int up = 0; up < 2 && up <= g.level; ++up) {
					const int lv = g.level - up;
					const int64_t n = int64_t(1) << lv;
					auto tile = [&](int64_t p_g) { return uint32_t(std::clamp((p_g >> up) / I, int64_t(0), n - 1)); };
					const TileKey key = TileKey::make(fs.face, lv, tile(fs.gi), tile(fs.gj));
					if (pages_.count(key.bits) && in_window(key)) {
						touch_page(key.bits);
					} else {
						g.ready = false;
						loader_->request(key, 1.0f / float(1 + r));
					}
				}
			}
		}
		geo.push_back(g);
	}
	const int active = int(geo.size());
	// the finest ring from which every coarser one is ready
	int finest = active - 1;
	while (finest > 0 && geo[size_t(finest - 1)].ready) {
		--finest;
	}
	// each drawn ring's textures and lattice; zeros past the last
	// tell the shader there is no coarser ring to morph to
	PackedVector4Array ring_geo;
	ring_geo.resize(int64_t(rings_.size()));
	const bool pages_changed = stat_uploads_ > 0;
	for (int r = finest; r < active; ++r) {
		const RingGeo &g = geo[size_t(r)];
		update_ring_textures(r, g, face0, pages_changed);
		ring_geo.set(r, Vector4(real_t(g.cx - int64_t(N / 2) * g.s), real_t(g.cy - int64_t(N / 2) * g.s), real_t(g.s),
								real_t(g.level)));
	}
	rs->material_set_param(material_, "ring_geo", ring_geo);

	int drawn = 0;
	for (int r = 0; r < int(rings_.size()); ++r) {
		Ring &ring = rings_[size_t(r)];
		const bool vis = r >= finest && r < active;
		if (vis != ring.visible) {
			rs->instance_set_visible(ring.instance, vis);
			ring.visible = vis;
		}
		if (!vis) {
			continue;
		}
		++drawn;
		const RingGeo &g = geo[size_t(r)];
		const RID inst = ring.instance;
		const bool full = r == finest;
		if (full != ring.full) {
			rs->instance_set_base(inst, full ? mesh_ring_full_ : mesh_ring_);
			ring.full = full;
			// a new base: push everything again
			ring.push_id = 0;
			ring.xform_id = 0;
			ring.hole = Vector4(-2, -2, -2, -2);
			ring.shadow = !ring.shadow;
		}
		const int64_t min_x = g.cx - int64_t(N / 2) * g.s, min_y = g.cy - int64_t(N / 2) * g.s;
		if (ring.cx != g.cx || ring.cy != g.cy || ring.face != face0 || ring.push_id != push_id_) {
			const regolith::IVec3 c = fold_grid(face0, g.cx, g.cy, g.m);
			const double md = double(g.m);
			const DVec3 qc(regolith::warp(double(c.x) / md), regolith::warp(double(c.y) / md), regolith::warp(double(c.z) / md));
			const regolith::FaceSample cs = regolith::canonical_sample(c, g.m);
			const int64_t n = int64_t(1) << g.level;
			float lo = 0.0f, hi = 0.0f;
			selector_.rho_bounds(TileKey::make(cs.face, g.level, uint32_t(std::min(cs.gi / I, n - 1)),
										 uint32_t(std::min(cs.gj / I, n - 1))),
					lo, hi);
			const double rho_c = 0.5 * (double(lo) + double(hi));
			const double r_c = r_ref + rho_c;
			const Vector3 centre = regolith::to_v(push_frame_.to_local_point(regolith::normalize(qc) * r_c));
			const Vector3 qp = regolith::to_v(push_frame_.to_local_vec(qc));
			constexpr double QP = std::numbers::pi / 4.0;
			rs->instance_geometry_set_shader_parameter(inst, "u_center", Vector4(centre.x, centre.y, centre.z, real_t(r_c)));
			rs->instance_geometry_set_shader_parameter(inst, "u_qc", Vector4(qp.x, qp.y, qp.z, real_t(regolith::length(qc))));
			rs->instance_geometry_set_shader_parameter(inst, "u_cc", Vector4(real_t(c.x), real_t(c.y), real_t(c.z), real_t(g.m)));
			rs->instance_geometry_set_shader_parameter(inst, "u_ccos",
					Vector4(real_t(std::cos(QP * double(c.x) / md)), real_t(std::cos(QP * double(c.y) / md)),
							real_t(std::cos(QP * double(c.z) / md)), real_t(g.s)));
			rs->instance_geometry_set_shader_parameter(inst, "u_ring",
					Vector4(real_t(min_x), real_t(min_y), real_t(g.level), real_t(face0)));
			rs->instance_geometry_set_shader_parameter(inst, "u_debug",
					Vector4(real_t(r), real_t(N), real_t(rho_c), real_t(1.0 + 0.01 * double(N) * g.spacing)));
			push_domains(inst, double(2 * g.cx - g.m) / md, double(2 * g.cy - g.m) / md, rho_c);
			const real_t ext = real_t(double(N) * g.spacing + 3000.0);
			rs->instance_set_custom_aabb(inst, AABB(centre - Vector3(ext, ext, ext), Vector3(ext, ext, ext) * 2));
			ring.cx = g.cx;
			ring.cy = g.cy;
			ring.face = face0;
			ring.push_id = push_id_;
		}
		// the finer ring's footprint, in this ring's grid steps
		Vector4 hole(1, 1, 0, 0);
		if (!full) {
			const RingGeo &f = geo[size_t(r - 1)];
			const int64_t k = int64_t(1) << (f.level - g.level);
			const int64_t h = int64_t(N / 2) * f.s;
			hole = Vector4(real_t(((f.cx - h) / k - min_x) / g.s), real_t(((f.cy - h) / k - min_y) / g.s),
					real_t(((f.cx + h) / k - min_x) / g.s), real_t(((f.cy + h) / k - min_y) / g.s));
		}
		if (hole != ring.hole) {
			rs->instance_geometry_set_shader_parameter(inst, "u_hole", hole);
			ring.hole = hole;
		}
		if (ring.xform_id != xform_id_) {
			rs->instance_set_transform(inst, push_xform_);
			ring.xform_id = xform_id_;
		}
		const bool shadow = full || double(N / 4) * g.spacing < SHADOW_RANGE;
		if (shadow != ring.shadow) {
			rs->instance_geometry_set_cast_shadows_setting(inst,
					shadow ? RenderingServer::SHADOW_CASTING_SETTING_ON : RenderingServer::SHADOW_CASTING_SETTING_OFF);
			ring.shadow = shadow;
		}
	}
	stat_nodes_ = drawn;
	stat_wanted_ = finest;
}

// Ground texture coordinates are face metres. The shader adds
// small node-local offsets to what is computed here in double:
// for each texture domain, the fractional texture coordinate and
// the hex-tiling lattice position mod 256 (the hash period), so
// tiles and cells line up exactly across every node edge.
void RegolithBody::push_texture_domains(RID p_inst, TileKey p_key, double p_rho_c) {
	const regolith::TileLayout lay = source_.layout();
	const int64_t m = int64_t(lay.intervals) << p_key.level();
	const double s_c = double(2 * (int64_t(p_key.x()) * lay.intervals + lay.intervals / 2) - m) / double(m);
	const double t_c = double(2 * (int64_t(p_key.y()) * lay.intervals + lay.intervals / 2) - m) / double(m);
	push_domains(p_inst, s_c, t_c, p_rho_c);
}

void RegolithBody::push_domains(RID p_inst, double p_s, double p_t, double p_rho_c) {
	// mors's layer uv scales times 0.5, as its shader uses them
	static constexpr double SCALES[5] = { 0.5, 0.0225, 0.25, 0.3335, 0.04 };
	static constexpr double NOISE_PERIOD = 20000.0;
	RenderingServer *rs = RenderingServer::get_singleton();
	const double edge = std::numbers::pi / 2.0 * source_.r_ref();
	const double fx = (p_s + 1.0) * 0.5 * edge, fy = (p_t + 1.0) * 0.5 * edge;
	auto domain = [](double p_bx, double p_by, double p_scale) {
		const double dx = p_bx * p_scale + 0.5, dy = p_by * p_scale + 0.5;
		const double ax = 0.7 * dx, ay = 0.7 * dy;
		const double px = ax - 0.57735027 * ay, py = 1.15470054 * ay;
		auto wrap = [](double p_v, double p_n) { return p_v - p_n * std::floor(p_v / p_n); };
		return Vector4(float(wrap(dx, 1.0)), float(wrap(dy, 1.0)), float(wrap(px, 256.0)), float(wrap(py, 256.0)));
	};
	static const StringName names[5] = { "u_dom0", "u_dom1", "u_dom2", "u_dom3", "u_dom4" };
	for (int k = 0; k < 5; ++k) {
		rs->instance_geometry_set_shader_parameter(p_inst, names[k], domain(fx, fy, SCALES[k]));
	}
	// cliff, projected on vertical faces: (face metre, -height)
	rs->instance_geometry_set_shader_parameter(p_inst, "u_cliff_u", domain(fx, -p_rho_c, SCALES[1]));
	rs->instance_geometry_set_shader_parameter(p_inst, "u_cliff_v", domain(fy, -p_rho_c, SCALES[1]));
	rs->instance_geometry_set_shader_parameter(p_inst, "u_nmod",
			Vector4(float(std::fmod(fx, NOISE_PERIOD)), float(std::fmod(fy, NOISE_PERIOD)), 0.0f, 0.0f));
}

void RegolithBody::_physics_process(double p_delta) {
	RegolithSystem *sys = system();
	if (!opened_ || !sys) {
		return;
	}
	++physics_tick_;
	if (sys->dominant() == this) {
		const uint64_t t0 = Time::get_singleton()->get_ticks_usec();
		update_collision();
		stat_collision_ms_ = double(Time::get_singleton()->get_ticks_usec() - t0) * 1e-3;
		return;
	}
	// Only the body whose frame we are in has ground to touch.
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	for (auto &entry : patches_) {
		ps->free_rid(entry.second.body);
		ps->free_rid(entry.second.shape);
	}
	patches_.clear();
}

// A body in noclip (every shape disabled) touches nothing.
bool RegolithBody::has_live_shape(Node3D *p_node) {
	CollisionObject3D *co = Object::cast_to<CollisionObject3D>(p_node);
	if (!co) {
		return true;
	}
	const PackedInt32Array owners = co->get_shape_owners();
	for (int i = 0; i < owners.size(); ++i) {
		if (!co->is_shape_owner_disabled(uint32_t(owners[i]))) {
			return true;
		}
	}
	return false;
}

void RegolithBody::_enter_tree() {
	add_to_group("regolith_body");
}

void RegolithBody::pose(double p_t, DVec3 &r_position, DVec3 &r_velocity, DVec3 r_axes[3]) const {
	if (has_orbit_) {
		const regolith::OrbitState s = orbit_.state(p_t);
		r_position = s.position;
		r_velocity = s.velocity;
		if (rotation_model_ == ROTATION_SYNCHRONOUS) {
			regolith::synchronous_axes(s, r_axes);
			return;
		}
	} else {
		r_position = DVec3(0.0, 0.0, 0.0);
		r_velocity = DVec3(0.0, 0.0, 0.0);
	}
	r_axes[0] = regolith::mars_body_to_inertial(p_t, DVec3(1.0, 0.0, 0.0));
	r_axes[1] = regolith::mars_body_to_inertial(p_t, DVec3(0.0, 1.0, 0.0));
	r_axes[2] = regolith::mars_body_to_inertial(p_t, DVec3(0.0, 0.0, 1.0));
}

void RegolithBody::update_collision() {
	RegolithSystem *sys = system();
	const regolith::RenderFrame f = sys->frame_for(this);
	const int patch_level = collision_level_ + PATCH_SPLIT_LEVELS;
	const double patch_edge = regolith::Selector::node_edge(source_.r_ref(), patch_level);

	struct Need {
		TileKey patch;
		double dist;
	};
	std::vector<Need> needs;
	TypedArray<Node> colliders = get_tree()->get_nodes_in_group("regolith_collider");
	for (int i = 0; i < colliders.size(); ++i) {
		RegolithCollider *c = Object::cast_to<RegolithCollider>(colliders[i]);
		Node3D *anchor = c ? Object::cast_to<Node3D>(c->get_parent()) : nullptr;
		if (!anchor || sys->is_parked(anchor) || !has_live_shape(anchor)) {
			continue;
		}
		Vector3 vel;
		if (CharacterBody3D *cb = Object::cast_to<CharacterBody3D>(anchor)) {
			vel = cb->get_velocity();
		} else if (RigidBody3D *rb = Object::cast_to<RigidBody3D>(anchor)) {
			vel = rb->get_linear_velocity();
		}
		const DVec3 here = regolith::to_d(anchor->get_global_position());
		for (double t : { 0.0, 0.5, 1.0 }) {
			const DVec3 centre = regolith::to_body_point(f, here + regolith::to_d(vel) * (t * c->get_lookahead()));
			const double reach = c->get_radius();
			// High above the ground there is nothing to touch yet.
			const double clearance = regolith::length(centre) - surface_radius(regolith::normalize(centre), false);
			if (clearance > reach) {
				continue;
			}
			const regolith::RenderFrame tangent = regolith::RenderFrame::at(centre);
			const int steps = int(std::ceil(reach / (0.5 * patch_edge)));
			for (int b = -steps; b <= steps; ++b) {
				for (int a = -steps; a <= steps; ++a) {
					const DVec3 off = tangent.east * (a * 0.5 * patch_edge) + tangent.south * (b * 0.5 * patch_edge);
					if (regolith::length(off) > reach + 0.5 * patch_edge) {
						continue;
					}
					const regolith::FaceCoord fc = regolith::dir_to_face(regolith::normalize(centre + off));
					needs.push_back({ key_at(fc, patch_level), regolith::length(off) + t * 10.0 });
				}
			}
		}
	}
	std::sort(needs.begin(), needs.end(), [](const Need &p_a, const Need &p_b) { return p_a.dist < p_b.dist; });
	int built = 0;
	for (const Need &nd : needs) {
		auto it = patches_.find(nd.patch.bits);
		if (it != patches_.end()) {
			it->second.used = physics_tick_;
			continue;
		}
		const TileKey tile = TileKey::make(nd.patch.face(), collision_level_, nd.patch.x() >> PATCH_SPLIT_LEVELS,
				nd.patch.y() >> PATCH_SPLIT_LEVELS);
		if (built < PATCH_BUILDS_PER_TICK && cache_->contains(tile)) {
			const int mask = (1 << PATCH_SPLIT_LEVELS) - 1;
			const uint64_t tb = Time::get_singleton()->get_ticks_usec();
			if (build_patch(nd.patch.bits, tile, int(nd.patch.x()) & mask, int(nd.patch.y()) & mask)) {
				++built;
			}
			stat_build_us_ += Time::get_singleton()->get_ticks_usec() - tb;
		} else if (!cache_->contains(tile)) {
			loader_->request(tile, 1000.0f + float(1.0 / (1.0 + nd.dist)));
		}
	}
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	for (auto it = patches_.begin(); it != patches_.end();) {
		if (it->second.used + PATCH_KEEP_TICKS < physics_tick_) {
			ps->free_rid(it->second.body);
			ps->free_rid(it->second.shape);
			it = patches_.erase(it);
		} else {
			++it;
		}
	}
	// Ground only moves through the frame when it is inertial or
	// carrying velocity; a rebuilt frame is a teleport.
	const bool rebuilt = sys->rebuild_count() != patch_rebuild_;
	patch_rebuild_ = sys->rebuild_count();
	const uint64_t tp = Time::get_singleton()->get_ticks_usec();
	// One more pass on the tick it stops, to zero their velocity.
	const bool moving = sys->frame_moving();
	if (rebuilt || moving || patches_moving_) {
		for (auto &entry : patches_) {
			place_patch(entry.second, rebuilt);
		}
	}
	patches_moving_ = moving;
	stat_place_us_ += Time::get_singleton()->get_ticks_usec() - tp;
	stat_builds_ += uint64_t(built);
}

bool RegolithBody::build_patch(uint64_t p_id, TileKey p_tile, int p_bx, int p_by) {
	HeightTilePtr t = cache_->get(p_tile);
	if (!t) {
		return false;
	}
	const regolith::TileLayout lay = source_.layout();
	const int n = lay.intervals;
	const int size = n >> PATCH_SPLIT_LEVELS;
	const int i0 = p_bx * size;
	const int j0 = p_by * size;
	const int64_t m = int64_t(n) << p_tile.level();
	auto pos = [&](int p_i, int p_j) {
		const regolith::FaceSample s{ p_tile.face(), int64_t(p_tile.x()) * n + p_i, int64_t(p_tile.y()) * n + p_j };
		return regolith::lattice_dir(regolith::lattice_point(s, m), m) * (source_.r_ref() + double(t->at(lay, p_i, p_j)));
	};
	const DVec3 origin = pos(i0 + size / 2, j0 + size / 2);
	const regolith::RenderFrame pf = regolith::RenderFrame::at(origin);
	std::vector<Vector3> local(size_t(size + 1) * size_t(size + 1));
	for (int j = 0; j <= size; ++j) {
		for (int i = 0; i <= size; ++i) {
			local[size_t(j * (size + 1) + i)] = regolith::to_v(pf.to_local_point(pos(i0 + i, j0 + j)));
		}
	}
	PackedVector3Array faces;
	faces.resize(int64_t(size) * size * 6);
	Vector3 *w = faces.ptrw();
	auto at = [&](int p_i, int p_j) { return local[size_t(p_j * (size + 1) + p_i)]; };
	int64_t k = 0;
	for (int j = 0; j < size; ++j) {
		for (int i = 0; i < size; ++i) {
			const Vector3 a = at(i, j), b = at(i + 1, j), c = at(i + 1, j + 1), d = at(i, j + 1);
			w[k++] = a;
			w[k++] = c;
			w[k++] = b;
			w[k++] = a;
			w[k++] = d;
			w[k++] = c;
		}
	}
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	Patch p;
	p.shape = ps->concave_polygon_shape_create();
	Dictionary data;
	data["faces"] = faces;
	data["backface_collision"] = true;
	ps->shape_set_data(p.shape, data);
	p.body = ps->body_create();
	// Kinematic: the ground can move through the frame, and
	// contacts must see that velocity.
	ps->body_set_mode(p.body, PhysicsServer3D::BODY_MODE_KINEMATIC);
	ps->body_add_shape(p.body, p.shape);
	p.origin = origin;
	p.used = physics_tick_;
	place_patch(p, true);
	ps->body_set_space(p.body, get_world_3d()->get_space());
	patches_[p_id] = p;
	return true;
}

void RegolithBody::place_patch(Patch &r_patch, bool p_teleport) const {
	RegolithSystem *sys = system();
	const regolith::RenderFrame f = sys->frame_for(this);
	const regolith::RenderFrame pf = regolith::RenderFrame::at(r_patch.origin);
	Transform3D xf;
	xf.basis = regolith::basis_between(f, pf);
	xf.origin = regolith::to_v(f.to_local_point(r_patch.origin));
	// Velocity of this piece of ground in the render frame.
	const RegolithSystem::Pose &bp = sys->pose_of(this);
	RegolithSystem::State s;
	s.x = bp.position + bp.axes[0] * r_patch.origin.x + bp.axes[1] * r_patch.origin.y + bp.axes[2] * r_patch.origin.z;
	s.v = bp.velocity + regolith::cross(bp.omega, s.x - bp.position);
	Vector3 unused, vel;
	sys->from_state(s, unused, vel);
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	// Jolt sweeps a kinematic body to a new transform over the
	// next step. A rebuilt frame must be a true teleport, done
	// outside the space, or the sweep bats away whatever is in
	// its path.
	const RID space = ps->body_get_space(r_patch.body);
	if (p_teleport && space.is_valid()) {
		ps->body_set_space(r_patch.body, RID());
	}
	ps->body_set_state(r_patch.body, PhysicsServer3D::BODY_STATE_TRANSFORM, xf);
	if (p_teleport && space.is_valid()) {
		ps->body_set_space(r_patch.body, space);
	}
	ps->body_set_state(r_patch.body, PhysicsServer3D::BODY_STATE_LINEAR_VELOCITY, vel);
	ps->body_set_state(r_patch.body, PhysicsServer3D::BODY_STATE_ANGULAR_VELOCITY, sys->render_angular_velocity(this));
}

bool RegolithBody::is_collision_ready_at(const Vector3 &p_pos) const {
	RegolithSystem *sys = system();
	if (!sys || !opened_) {
		return false;
	}
	const DVec3 x = regolith::to_body_point(sys->frame_for(this), regolith::to_d(p_pos));
	const regolith::FaceCoord fc = regolith::dir_to_face(regolith::normalize(x));
	return patches_.count(key_at(fc, collision_level_ + PATCH_SPLIT_LEVELS).bits) != 0;
}

Color RegolithBody::get_geology(double p_lat_deg, double p_lon_deg) const {
	uint8_t g[4] = { 0, 0, 0, 0 };
	if (synth_) {
		const double lat = p_lat_deg * Math::PI / 180.0, lon = p_lon_deg * Math::PI / 180.0;
		synth_->geology_at(regolith::DVec3(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)), g);
	}
	return Color(g[0] / 255.0f, g[1] / 255.0f, g[2] / 255.0f, g[3] / 255.0f);
}

Dictionary RegolithBody::get_stats() const {
	Dictionary d;
	// how far the drawn terrain is from where it belongs now: the
	// push frame placed into the render frame as of this call
	RegolithSystem *sys = system();
	if (sys && push_valid_) {
		const regolith::RenderFrame f = sys->frame_for(this);
		const Vector3 now = regolith::to_v(f.to_local_point(push_frame_.anchor));
		d["push_lag_m"] = double((now - push_xform_.origin).length());
	}
	// fine tiles last draw: [level, count, edge m, quads per edge,
	// nearest m, farthest m]
	Array fine;
	for (const auto &[level, fl] : stat_fine_levels_) {
		fine.push_back(Array::make(level, fl.count, regolith::Selector::node_edge(source_.r_ref(), level),
				int(source_.layout().intervals), fl.dmin, fl.dmax));
	}
	d["fine_levels"] = fine;
	// every tile drawn: [render instance RID id, level, fine, m]
	Array tiles;
	for (const TileStat &ts : stat_tiles_) {
		tiles.push_back(Array::make(int64_t(ts.rid), ts.level, ts.fine, ts.dist));
	}
	d["tiles"] = tiles;
	d["nodes"] = stat_nodes_;
	d["pages"] = int(pages_.size());
	d["uploads"] = stat_uploads_;
	d["wanted"] = stat_wanted_;
	d["pending"] = loader_ ? int(loader_->pending()) : 0;
	d["cached"] = cache_ ? int(cache_->size()) : 0;
	d["patches"] = int(patches_.size());
	d["select_ms"] = stat_select_ms_;
	d["draw_ms"] = stat_draw_ms_;
	d["upload_ms"] = stat_upload_ms_;
	d["collision_ms"] = stat_collision_ms_;
	d["builds"] = int64_t(stat_builds_);
	d["build_ms_total"] = double(stat_build_us_) * 1e-3;
	d["place_ms_total"] = double(stat_place_us_) * 1e-3;
	Dictionary levels;
	for (int level = 0; loader_ && level <= max_level_; ++level) {
		double ms = 0.0;
		uint64_t count = 0;
		loader_->level_timing(level, ms, count);
		if (count) {
			levels[level] = Vector2(real_t(ms), real_t(count));
		}
	}
	d["make_ms"] = levels;
	return d;
}

void RegolithBody::_exit_tree() {
	free_render_resources();
}

void RegolithBody::free_render_resources() {
	RenderingServer *rs = RenderingServer::get_singleton();
	PhysicsServer3D *ps = PhysicsServer3D::get_singleton();
	if (rs) {
		for (auto &entry : drawn_) {
			rs->free_rid(entry.second.instance);
		}
		for (RID r : spare_coarse_) {
			rs->free_rid(r);
		}
		for (RID r : spare_fine_) {
			rs->free_rid(r);
		}
		drawn_.clear();
		spare_coarse_.clear();
		spare_fine_.clear();
		for (Ring &r : rings_) {
			rs->free_rid(r.instance);
		}
		rings_.clear();
		for (RID *r : { &mesh_coarse_, &mesh_fine_, &mesh_ring_, &mesh_ring_full_, &page_table_, &ring_heights_,
					 &ring_horizon_[0], &ring_horizon_[1], &ring_geology_, &atlas_, &geology_atlas_,
					 &horizon_atlas_[0], &horizon_atlas_[1], &material_ }) {
			if (r->is_valid()) {
				rs->free_rid(*r);
				*r = RID();
			}
		}
	}
	if (ps) {
		for (auto &entry : patches_) {
			ps->free_rid(entry.second.body);
			ps->free_rid(entry.second.shape);
		}
	}
	patches_.clear();
	pages_.clear();
	lru_.clear();
	page_bounds_.clear();
	loader_.reset();
}

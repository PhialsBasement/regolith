#include "regolith_body.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <thread>

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/character_body3d.hpp>
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
#include <godot_cpp/variant/typed_array.hpp>
#include <godot_cpp/variant/vector4.hpp>

#include "convert.hpp"
#include "regolith/cube.hpp"
#include "regolith/ephemeris.hpp"
#include "regolith_collider.hpp"
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
constexpr double SHADOW_RANGE = 4000.0;
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
	ClassDB::bind_method(D_METHOD("set_quads", "quads"), &RegolithBody::set_quads);
	ClassDB::bind_method(D_METHOD("get_quads"), &RegolithBody::get_quads);
	ClassDB::bind_method(D_METHOD("set_range_factor", "factor"), &RegolithBody::set_range_factor);
	ClassDB::bind_method(D_METHOD("get_range_factor"), &RegolithBody::get_range_factor);
	ClassDB::bind_method(D_METHOD("set_page_capacity", "pages"), &RegolithBody::set_page_capacity);
	ClassDB::bind_method(D_METHOD("get_page_capacity"), &RegolithBody::get_page_capacity);
	ClassDB::bind_method(D_METHOD("set_debug_mode", "mode"), &RegolithBody::set_debug_mode);
	ClassDB::bind_method(D_METHOD("get_debug_mode"), &RegolithBody::get_debug_mode);
	ClassDB::bind_method(D_METHOD("get_stats"), &RegolithBody::get_stats);
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
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "range_factor"), "set_range_factor", "get_range_factor");
	ADD_PROPERTY(PropertyInfo(Variant::INT, "page_capacity"), "set_page_capacity", "get_page_capacity");
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
	cache_ = std::make_unique<regolith::TileCache>(size_t(page_capacity_) * 2, PINNED_LEVEL);
	const int threads = std::max(1, int(std::thread::hardware_concurrency()) - 2);
	loader_ = std::make_unique<regolith::TileLoader>(source_, *cache_, threads);

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

	selector_.rho_bounds = [this](TileKey p_k, float &r_lo, float &r_hi) {
		if (p_k.level() <= source_.max_level()) {
			return source_.bounds(p_k, r_lo, r_hi);
		}
		if (HeightTilePtr t = cache_->get(p_k)) {
			r_lo = t->min;
			r_hi = t->max;
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
	shader_ = ResourceLoader::get_singleton()->load("res://addons/regolith/regolith_terrain.gdshader");
	if (shader_.is_null()) {
		ERR_PRINT("RegolithBody: missing res://addons/regolith/regolith_terrain.gdshader");
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
	rs->material_set_param(material_, "intervals", lay.intervals);
	rs->material_set_param(material_, "border", lay.border);
	rs->material_set_param(material_, "debug_mode", debug_mode_);
	mesh_coarse_ = make_grid_mesh(quads_);
	mesh_fine_ = make_grid_mesh(lay.intervals);
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

void RegolithBody::set_debug_mode(int p_v) {
	debug_mode_ = p_v;
	if (material_.is_valid()) {
		RenderingServer::get_singleton()->material_set_param(material_, "debug_mode", debug_mode_);
	}
}

bool RegolithBody::upload(TileKey p_key) {
	if (pages_.count(p_key.bits)) {
		return true;
	}
	HeightTilePtr tile = cache_->get(p_key);
	if (!tile) {
		return false;
	}
	int layer;
	if (!free_layers_.empty()) {
		layer = free_layers_.back();
		free_layers_.pop_back();
	} else {
		auto victim = pages_.end();
		for (auto it = pages_.begin(); it != pages_.end(); ++it) {
			if (it->second.used + 1 < frame_ && (victim == pages_.end() || it->second.used < victim->second.used)) {
				victim = it;
			}
		}
		if (victim == pages_.end()) {
			return false;
		}
		layer = victim->second.layer;
		pages_.erase(victim);
	}
	PackedByteArray bytes;
	bytes.resize(int64_t(tile->rho.size() * sizeof(float)));
	std::memcpy(bytes.ptrw(), tile->rho.data(), tile->rho.size() * sizeof(float));
	const int side = source_.layout().side();
	Ref<Image> img = Image::create_from_data(side, side, false, Image::FORMAT_RF, bytes);
	RenderingServer::get_singleton()->texture_2d_update(atlas_, img, layer);
	pages_[p_key.bits] = { layer, frame_ };
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
	if (frame_ == 1) {
		// Upload the pinned set before anything is drawn.
		for (int face = 0; face < regolith::FACE_COUNT; ++face) {
			for (int level = 0; level <= std::min(PINNED_LEVEL, source_.max_level()); ++level) {
				const uint32_t n = 1u << level;
				for (uint32_t x = 0; x < n; ++x) {
					for (uint32_t y = 0; y < n; ++y) {
						upload(TileKey::make(face, level, x, y));
					}
				}
			}
		}
	}
	upload_finished(UPLOADS_PER_FRAME);

	const uint64_t t0 = Time::get_singleton()->get_ticks_usec();
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
	stat_select_ms_ = double(Time::get_singleton()->get_ticks_usec() - t0) * 1e-3;
	stat_nodes_ = int(nodes.size());
	stat_wanted_ = int(wanted.size());
	draw(nodes);
}

void RegolithBody::draw(const std::vector<regolith::SelectedNode> &p_nodes) {
	RenderingServer *rs = RenderingServer::get_singleton();
	RegolithSystem *sys = system();
	const regolith::RenderFrame f = sys->frame_for(this);
	const regolith::TileLayout lay = source_.layout();
	const RID scenario = get_world_3d()->get_scenario();
	const Vector3 cam = get_viewport()->get_camera_3d()->get_global_position();

	// Re-push node constants only when the frame was rebuilt or
	// has drifted far from the push frame; otherwise the shared
	// instance transform follows the frame.
	Transform3D xf;
	if (push_valid_) {
		xf.basis = regolith::basis_between(f, push_frame_);
		xf.origin = regolith::to_v(f.to_local_point(push_frame_.anchor));
	}
	const bool drifted = xf.origin.length() > 5000.0 ||
			(xf.basis.get_column(0) - Vector3(1, 0, 0)).length() > 0.01 ||
			(xf.basis.get_column(1) - Vector3(0, 1, 0)).length() > 0.01;
	if (!push_valid_ || push_rebuild_ != sys->rebuild_count() || drifted) {
		push_frame_ = f;
		push_valid_ = true;
		push_rebuild_ = sys->rebuild_count();
		++push_id_;
		xf = Transform3D();
	}
	if (!xf.is_equal_approx(push_xform_) || xform_id_ == 0) {
		push_xform_ = xf;
		++xform_id_;
	}
	for (const regolith::SelectedNode &n : p_nodes) {
		const int level = n.key.level();
		const bool fine = level >= max_level_ - 1;

		// Mark the node and its whole ancestry as in use.
		for (TileKey a = n.key;; a = a.parent()) {
			auto it = pages_.find(a.bits);
			if (it != pages_.end()) {
				it->second.used = frame_;
			}
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
			}
			found = drawn_.emplace(n.key.bits, d).first;
		}
		Drawn &d = found->second;
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

void RegolithBody::_physics_process(double p_delta) {
	RegolithSystem *sys = system();
	if (!opened_ || !sys) {
		return;
	}
	++physics_tick_;
	if (sys->dominant() == this) {
		update_collision();
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
		if (!anchor || sys->is_parked(anchor)) {
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
			if (build_patch(nd.patch.bits, tile, int(nd.patch.x()) & mask, int(nd.patch.y()) & mask)) {
				++built;
			}
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
	// The frame may move or turn every tick (floating velocity,
	// inertial mode), so the ground is re-placed every tick; a
	// rebuilt frame is a teleport, not motion.
	const bool rebuilt = sys->rebuild_count() != patch_rebuild_;
	patch_rebuild_ = sys->rebuild_count();
	for (auto &entry : patches_) {
		place_patch(entry.second, rebuilt);
	}
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

Dictionary RegolithBody::get_stats() const {
	Dictionary d;
	d["nodes"] = stat_nodes_;
	d["pages"] = int(pages_.size());
	d["uploads"] = stat_uploads_;
	d["wanted"] = stat_wanted_;
	d["pending"] = loader_ ? int(loader_->pending()) : 0;
	d["cached"] = cache_ ? int(cache_->size()) : 0;
	d["patches"] = int(patches_.size());
	d["select_ms"] = stat_select_ms_;
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
		for (RID *r : { &mesh_coarse_, &mesh_fine_, &atlas_, &material_ }) {
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
	loader_.reset();
}

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <algorithm>
#include <climits>
#include <list>
#include <map>
#include <memory>
#include <unordered_map>
#include <vector>

#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector4.hpp>

#include "regolith/ephemeris.hpp"
#include "regolith/height_tile.hpp"
#include "regolith/horizon.hpp"
#include "regolith/orbit.hpp"
#include "regolith/patch.hpp"
#include "regolith/select.hpp"
#include "regolith/synth.hpp"
#include "regolith/tile_loader.hpp"

namespace godot {

class RegolithSystem;

// One celestial body: streams its height pack, draws it as a
// CDLOD cube-sphere and keeps terrain collision around every
// RegolithCollider.
class RegolithBody : public Node3D {
	GDCLASS(RegolithBody, Node3D)

public:
	enum RotationModel {
		ROTATION_IAU = 0, // the rotation elements below
		ROTATION_MARS_IAU = ROTATION_IAU, // the old name
		ROTATION_SYNCHRONOUS = 1,
	};

	RegolithBody();
	~RegolithBody() override;
	void _enter_tree() override;
	void _ready() override;
	void _process(double p_delta) override;
	void _physics_process(double p_delta) override;
	void _exit_tree() override;

	bool ensure_open();
	double r_ref() const { return source_.r_ref(); }
	// Centre (m, ICRF, relative to the central body) and the
	// body-fixed axes as ICRF columns, at TT seconds p_t.
	void pose(double p_t, regolith::DVec3 &r_position, regolith::DVec3 &r_velocity, regolith::DVec3 r_axes[3]) const;
	// A real body's spin, orbit and gravity in one go (Mercury,
	// Venus, Earth, Moon, Mars, Phobos, Deimos, Vesta, Ceres);
	// the settings below can then be changed one by one.
	void set_preset(const String &p_v);
	String get_preset() const { return preset_; }
	// IAU rotation: pole RA, Dec (deg, per century), meridian
	// (deg, per day). The heliocentric orbit sets the sun for a
	// central body (one without an orbit table).
	void set_rotation_element(int p_index, double p_v);
	double get_rotation_element(int p_index) const;
	void set_orbit_element(int p_index, double p_v);
	double get_orbit_element(int p_index) const;
	const regolith::Rotation &rotation() const { return rotation_; }
	const regolith::OrbitElements &orbit_elements() const { return orbit_elements_; }
	bool has_orbit_table() const { return has_orbit_; }
	void set_j2(double p_v) { j2_ = p_v; }
	double get_j2() const { return j2_; }
	void set_j2_radius(double p_v) { j2_radius_ = p_v; }
	double get_j2_radius() const { return j2_radius_; }
	void set_rotating_altitude(double p_v) { rotating_altitude_ = p_v; }
	double get_rotating_altitude() const { return rotating_altitude_; }
	// Surface radius under a direction; p_exact produces the
	// finest tile synchronously, otherwise the deepest cached.
	double surface_radius(const regolith::DVec3 &p_dir, bool p_exact);
	// Areoid (datum) radius under a direction.
	double datum_radius(const regolith::DVec3 &p_dir) const;
	// the lowest ground anywhere on the body (m from its centre),
	// 0 before the first frame
	double lowest_radius() const { return occluder_radius_; }
	// Height over the datum at equirect cell centres (row 0
	// north, column 0 at -180), from level p_level; any thread.
	void height_grid(int p_w, int p_h, int p_level, std::vector<float> &r_out);

	void set_pack_path(const String &p_v) { pack_path_ = p_v; }
	String get_pack_path() const { return pack_path_; }
	void set_datum_path(const String &p_v) { datum_path_ = p_v; }
	String get_datum_path() const { return datum_path_; }
	void set_system_path(const NodePath &p_v) { system_path_ = p_v; }
	NodePath get_system_path() const { return system_path_; }
	void set_gm(double p_v) { gm_ = p_v; }
	double get_gm() const { return gm_; }
	void set_orbit_path(const String &p_v) { orbit_path_ = p_v; }
	String get_orbit_path() const { return orbit_path_; }
	void set_rotation_model(int p_v) { rotation_model_ = p_v; }
	int get_rotation_model() const { return rotation_model_; }
	void set_max_level(int p_v) { max_level_ = p_v; }
	int get_max_level() const { return max_level_; }
	void set_collision_level(int p_v) { collision_level_ = p_v; }
	int get_collision_level() const { return collision_level_; }
	// The finest levels drawn with the full-density mesh.
	void set_dense_levels(int p_v) { dense_levels_ = p_v; }
	int get_dense_levels() const { return dense_levels_; }
	void set_quads(int p_v) { quads_ = p_v; }
	int get_quads() const { return quads_; }
	// Clipmap ring width in quads (a multiple of 4). 0: LEGACY
	// CDLOD path, MORS's fixed Mars material; no surface_path,
	// no per-point air light. Kept for reference only.
	void set_clip_quads(int p_v) { clip_quads_ = p_v <= 0 ? 0 : std::max(8, p_v / 4 * 4); }
	int get_clip_quads() const { return clip_quads_; }
	void set_range_factor(double p_v) { range_factor_ = p_v; }
	double get_range_factor() const { return range_factor_; }	void set_page_capacity(int p_v) { page_capacity_ = p_v; }
	int get_page_capacity() const { return page_capacity_; }
	void set_debug_mode(int p_v);
	int get_debug_mode() const { return debug_mode_; }
	void set_synthesis(int p_v) { synthesis_ = p_v; }
	int get_synthesis() const { return synthesis_; }
	// What the ground looks like: a .gdshaderinc defining
	// surface(TerrainPoint, inout TerrainSurface); empty: default
	void set_surface_path(const String &p_v) { surface_path_ = p_v; }
	String get_surface_path() const { return surface_path_; }
	void set_dark_map_path(const String &p_v) { dark_map_path_ = p_v; }
	String get_dark_map_path() const { return dark_map_path_; }
	// Ground-shader uniforms (texture arrays, tints...), kept
	// and applied whenever the material exists.
	void set_material_param(const StringName &p_name, const Variant &p_value);
	Ref<Image> bake_iso_map(int p_width, int p_height, const PackedColorArray &p_layer_means, int p_level);
	Dictionary get_stats() const;
	// The ground shader's geology at a lat/lon: r gravel province,
	// g bedrock patch (both noise remapped to 0..1), b ice, a dark
	Color get_geology(double p_lat_deg, double p_lon_deg) const;
	bool is_collision_ready_at(const Vector3 &p_pos) const;

protected:
	static void _bind_methods();

private:
	struct Page {
		int layer = -1;
		uint64_t used = 0;
		// place in lru_, oldest first; pinned pages are kept out
		std::list<uint64_t>::iterator lru;
		bool pinned = false;
	};
	// One clipmap ring: a fixed grid around the camera, one
	// render instance, constants pushed when its centre moves.
	struct Ring {
		RID instance;
		bool visible = false;
		bool full = false; // the finest drawn: no hole
		uint64_t shown_usec = 0; // when it last came into view
		float fade = 1.0f; // pushed u_fade
		bool shadow = false;
		int64_t cx = INT64_MIN; // centre, face0 lattice units
		int64_t cy = INT64_MIN;
		int face = -1;
		uint64_t push_id = 0;
		uint64_t xform_id = 0;
		Vector4 hole;
		// texture window origin (grid steps, absolute) and face it
		// was filled for; complete: no coarser data stood in
		int64_t kx0 = INT64_MIN;
		int64_t ky0 = INT64_MIN;
		int64_t qx0 = INT64_MIN;
		int64_t qy0 = INT64_MIN;
		int tex_face = -1;
		bool complete = false;
		std::vector<float> heights; // T x T, toroidal
		std::vector<uint8_t> hor_a; // Tq x Tq RGBA8, directions 0-3
		std::vector<uint8_t> hor_b; // directions 4-7
		std::vector<uint8_t> geo; // Tq x Tq RGBA8
	};
	// One ring's lattice: data level, stride, centre (face0 grid)
	struct RingGeo {
		int level;
		int64_t s;
		int64_t m;
		int64_t cx, cy;
		double spacing;
		bool ready;
	};
	struct Patch {
		RID body;
		RID shape;
		regolith::DVec3 origin; // body-fixed
		uint64_t used = 0;
	};
	// One render instance per drawn node, mesh fixed for life;
	// parameters are pushed only when something changed.
	struct Drawn {
		RID instance;
		bool fine = false;
		int self_layer = -1;
		int parent_layer = -1;
		uint64_t push_id = 0;
		uint64_t xform_id = 0;
		bool shadow = false;
		uint64_t seen = 0;
	};

	RegolithSystem *system() const;
	static bool has_live_shape(Node3D *p_node);
	static std::shared_ptr<const regolith::DarkField> load_dark_field(const String &p_path);
	void create_render_resources();
	RID make_grid_mesh(int p_quads);
	void upload_finished(int p_budget);
	// p_make_horizon only for the first-frame pinned set: in play
	// a page waits for a worker's horizon, the main thread never
	// computes one
	bool upload(regolith::TileKey p_key, bool p_make_horizon = false);
	void draw(const std::vector<regolith::SelectedNode> &p_nodes);
	// the push frame, sun and eye for this frame; false: no camera
	bool update_push_frame(regolith::DVec3 &r_cam_body);
	void draw_clipmap();
	// Bring ring r's textures to its current window, writing only
	// texels that entered it (all of them after a face change or
	// when coarser data stood in and pages arrived since).
	void update_ring_textures(int p_r, const RingGeo &p_g, int p_face0, bool p_pages_changed);
	// rho at a face lattice point, from the finest cached tile at
	// or above p_level; false when a coarser tile stood in
	bool ring_rho(int p_face, int p_level, int64_t p_gi, int64_t p_gj, float &r_rho);
	RID make_ring_mesh(int p_quads, bool p_full);
	void touch_page(uint64_t p_bits);
	void update_page_table(const regolith::DVec3 &p_cam_dir);
	void push_texture_domains(RID p_inst, regolith::TileKey p_key, double p_rho_c);
	// p_s, p_t: the linear face parameters of the domain origin
	void push_domains(RID p_inst, double p_s, double p_t, double p_rho_c);
	void update_collision();
	bool build_patch(uint64_t p_id, regolith::TileKey p_tile, int p_bx, int p_by);
	void place_patch(Patch &r_patch, bool p_teleport) const;
	void free_render_resources();
	void make_loader();

	String pack_path_;
	String datum_path_;
	String orbit_path_;
	NodePath system_path_;
	double gm_ = 4.282837e13;
	int rotation_model_ = ROTATION_MARS_IAU;
	String preset_ = "Mars";
	regolith::Rotation rotation_ = regolith::MARS_ROTATION;
	regolith::OrbitElements orbit_elements_ = regolith::MARS_ORBIT;
	int synthesis_ = 0; // 0 none, 1 Mars, 2 Phobos
	String dark_map_path_;
	String surface_path_;
	std::unique_ptr<regolith::Synth> synth_;
	Dictionary material_params_;
	Dictionary material_refs_;
	RID geology_atlas_;
	RID horizon_atlas_[2];
	double j2_ = 1.96045e-3;
	double j2_radius_ = 3396200.0;
	double rotating_altitude_ = 100000.0;
	regolith::OrbitTable orbit_;
	bool has_orbit_ = false;
	int max_level_ = 15;
	int collision_level_ = 15;
	int quads_ = 52;
	int dense_levels_ = 2;
	double range_factor_ = 6.0;
	int page_capacity_ = 4096;
	int debug_mode_ = 0;

	bool opened_ = false;
	regolith::PackTileSource source_;
	regolith::PackTileSource datum_source_;
	bool has_datum_ = false;
	std::unique_ptr<regolith::TileCache> cache_;
	std::unique_ptr<regolith::TileLoader> loader_;
	std::unordered_map<uint64_t, regolith::HeightTilePtr> datum_tiles_;
	int datum_level_ = 0;
	regolith::Selector selector_;

	Ref<Shader> shader_;
	RID material_;
	RID atlas_;
	RID mesh_coarse_;
	RID mesh_fine_;
	std::unordered_map<uint64_t, Drawn> drawn_;
	std::vector<RID> spare_coarse_;
	std::vector<RID> spare_fine_;
	std::unordered_map<uint64_t, Page> pages_;
	std::list<uint64_t> lru_;
	std::unordered_map<uint64_t, std::pair<float, float>> page_bounds_;
	// clipmap
	int clip_quads_ = 256;
	RID mesh_ring_;
	RID mesh_ring_full_;
	std::vector<Ring> rings_;
	// per ring layer: heights (T = clip_quads + 4) and, at quarter
	// resolution (Tq = clip_quads / 4 + 4), horizon and geology
	RID ring_heights_;
	RID ring_horizon_[2];
	RID ring_geology_;
	int ring_t_ = 0;
	int ring_tq_ = 0;
	// last tile looked up while filling (texels mostly share one)
	uint64_t memo_key_ = UINT64_MAX;
	regolith::HeightTilePtr memo_tile_;
	// (face, level, tile) -> atlas layer, a W x W toroidal window
	// per face and level around the camera; see update_page_table
	RID page_table_;
	PackedByteArray page_bytes_;
	std::vector<int64_t> page_origin_; // window origin per face/level
	bool page_dirty_ = true;
	std::vector<int> free_layers_;
	std::vector<regolith::TileKey> to_upload_;
	uint64_t frame_ = 0;
	double occluder_radius_ = 0.0;

	// Node constants are pushed in push_frame_; each frame one
	// shared instance transform carries them into the current
	// render frame, so a moving frame costs no re-push.
	regolith::RenderFrame push_frame_;
	bool push_valid_ = false;
	uint64_t push_rebuild_ = 0;
	uint64_t push_id_ = 0;
	Transform3D push_xform_;
	uint64_t xform_id_ = 0;

	std::unordered_map<uint64_t, Patch> patches_;
	uint64_t physics_tick_ = 0;
	uint64_t patch_rebuild_ = 0;
	bool patches_moving_ = false;

	// stats
	int stat_nodes_ = 0;
	int stat_uploads_ = 0;
	int stat_wanted_ = 0;
	double stat_select_ms_ = 0.0;
	double stat_draw_ms_ = 0.0;
	struct FineLevelStat {
		int count = 0;
		double dmin = 1e300;
		double dmax = 0.0;
	};
	std::map<int, FineLevelStat> stat_fine_levels_; // drawn fine tiles
	struct TileStat {
		uint64_t rid = 0;
		int level = 0;
		bool fine = false;
		double dist = 0.0;
	};
	std::vector<TileStat> stat_tiles_; // every tile drawn last frame
	double stat_upload_ms_ = 0.0;
	double stat_collision_ms_ = 0.0;
	uint64_t stat_builds_ = 0;
	uint64_t stat_build_us_ = 0;
	uint64_t stat_place_us_ = 0;
};

} // namespace godot

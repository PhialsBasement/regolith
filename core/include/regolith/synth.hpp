// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "regolith/height_tile.hpp"
#include "regolith/tile_layout.hpp"
#include "regolith/vec.hpp"

namespace regolith {

// mors's terrain detail, ported from planet_streamer.gd and
// terra_fine.gd (see docs/synth.md). Stages run at the level
// whose spacing matches mors's grid: detail and dunes at 200 m,
// the roughness band at 50 m, relief and thermal at 12.5 m.
struct SynthConfig {
	int profile = 0; // 0 none, 1 Mars
	double r_ref = 0.0;
	int intervals = 208;
	int detail_level = 7; // 200 m
	int rough_level = 9; // 50 m
	int relief_level = 11; // 12.5 m
	int relief_margin = 48; // thermal reach (40 steps) + slack
};

// Viking-mosaic darkness, 0..1, equirect (lon -180..180).
struct DarkField {
	int w = 0;
	int h = 0;
	std::vector<float> v;
	float sample(double p_lat_deg, double p_lon_deg) const;
};

class Synth {
public:
	explicit Synth(const SynthConfig &p_config);
	~Synth();

	const SynthConfig &config() const { return config_; }
	bool enabled() const { return config_.profile != 0; }
	void set_dark(std::shared_ptr<const DarkField> p_dark) { dark_ = std::move(p_dark); }
	// Height margin synthesis can add over the pack's bounds.
	double bounds_pad() const { return enabled() ? 140.0 : 0.0; }

	// Per-level passes on a full tile (border included).
	void detail(HeightTile &r_tile, const TileLayout &p_lay) const;
	void roughness(HeightTile &r_tile, const TileLayout &p_lay) const;
	// Also fills r_tile.repose for thermal().
	void relief(HeightTile &r_tile, const TileLayout &p_lay) const;
	void craters(HeightTile &r_tile, const TileLayout &p_lay, int p_band) const;
	void borehole(HeightTile &r_tile, const TileLayout &p_lay) const;
	// Thermal relaxation on a square grid of side p_n.
	void thermal(std::vector<float> &r_h, const std::vector<float> &p_rep, int p_n, double p_spacing) const;
	// Geology page: RGBA8 at quarter resolution, border 1.
	void geology(TileKey p_key, const TileLayout &p_lay, std::vector<uint8_t> &r_rgba) const;
	void geology_at(const DVec3 &p_dir, uint8_t r_out[4]) const;
	static int geology_side(const TileLayout &p_lay) { return p_lay.intervals / 4 + 3; }

	double spacing(int p_level) const;
	static DVec3 sample_dir(TileKey p_key, const TileLayout &p_lay, int p_i, int p_j);

private:
	struct Noises;
	struct SmallCrater {
		DVec3 dir;
		double ry = 0.0;
		double depth = 0.0;
		double cos_reach = 0.0;
	};
	void small_craters_near(const DVec3 &p_dir, double p_radius, int p_band, std::vector<SmallCrater> &r_out) const;
	double small_crater_sum(const DVec3 &p_dir, const std::vector<SmallCrater> &p_list) const;

	SynthConfig config_;
	std::unique_ptr<Noises> noise_;
	std::shared_ptr<const DarkField> dark_;
};

} // namespace regolith

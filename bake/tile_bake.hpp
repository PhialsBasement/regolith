// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "regolith/cube.hpp"
#include "regolith/pack.hpp"
#include "regolith/tile_key.hpp"
#include "regolith/tile_layout.hpp"

namespace regolith::bake {

// Produces rho (radius minus r_ref, metres) for one lattice
// point, always through its canonical sample. NaN = invalid.
class LevelSampler {
public:
	virtual ~LevelSampler() = default;
	virtual double rho(int p_level, const FaceSample &p_canonical, int64_t p_m, const IVec3 &p_point) const = 0;
};

void compute_tile(const LevelSampler &p_sampler, TileKey p_key, const TileLayout &p_layout, std::vector<double> &r_rho);

struct QuantizedTile {
	std::vector<int32_t> values;
	float min = 0.0f; // interior [0, N]^2 only
	float max = 0.0f;
	int invalid = 0;
};

QuantizedTile quantize_tile(const std::vector<double> &p_rho, const TileLayout &p_layout, double p_step);

struct BakeConfig {
	std::string path;
	PackHeader header;
	std::string attribution;
	TileLayout layout;
	int min_level = 0;
	int max_level = 0;
	int zstd_level = 9;
};

struct BakeStats {
	uint64_t tiles = 0;
	uint64_t bytes = 0;
	uint64_t invalid_samples = 0;
	double seconds = 0.0;
};

bool bake_pack(const LevelSampler &p_sampler, const BakeConfig &p_config, BakeStats &r_stats);

// Compares codecs, steps and zstd levels on a tile sample.
void codec_trial(const LevelSampler &p_sampler, const TileLayout &p_layout, const std::vector<TileKey> &p_keys);

} // namespace regolith::bake

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "regolith/pack.hpp"
#include "regolith/tile_key.hpp"
#include "regolith/tile_layout.hpp"

namespace regolith {

// rho (radius minus r_ref, metres) for every sample of a tile,
// border included, in TileLayout order.
struct HeightTile {
	TileKey key;
	std::vector<float> rho;
	float min = 0.0f; // interior [0, N]^2
	float max = 0.0f;
	std::vector<float> repose; // relief level only, before thermal
	std::vector<uint8_t> geology; // RGBA8, Synth::geology_side()^2
	// 2 x RGBA8, horizon_side()^2; set once, under the loader's
	// horizon lock (TileLoader::horizon).
	mutable std::shared_ptr<const std::vector<uint8_t>> horizon;

	void update_bounds(const TileLayout &p_layout);

	float at(const TileLayout &p_layout, int p_i, int p_j) const { return rho[size_t(p_layout.index(p_i, p_j))]; }
};

using HeightTilePtr = std::shared_ptr<const HeightTile>;

// Reads and decodes data-level tiles from an RGP1 pack.
class PackTileSource {
public:
	bool open(const std::string &p_path);
	const PackHeader &header() const { return reader_.header(); }
	TileLayout layout() const { return { int(reader_.header().intervals), int(reader_.header().border) }; }
	int max_level() const { return int(reader_.header().max_level); }
	double r_ref() const { return reader_.header().r_ref; }
	// Index min/max of a data-level tile; false if absent.
	bool bounds(TileKey p_key, float &r_min, float &r_max) const;
	// Thread-safe decode.
	bool load(TileKey p_key, HeightTile &r_tile) const;

private:
	PackReader reader_;
};

// The level below p_parent by 4-point interpolatory subdivision
// (symmetric midpoint rule, so shared edges stay bit-identical).
void subdivide(const HeightTile &p_parent, const TileLayout &p_layout, int p_quadrant, HeightTile &r_child);

} // namespace regolith

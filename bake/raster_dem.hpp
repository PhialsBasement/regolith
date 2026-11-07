#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace regolith::bake {

// A raw global equirectangular raster in a file: row 0 at the
// north edge (+90), column 0 at longitude -180, little endian.
struct RasterSpec {
	enum Type {
		INT16,
		FLOAT32,
	};

	std::string path;
	uint64_t offset = 0; // bytes before the first pixel
	int64_t width = 0;
	int64_t height = 0;
	Type type = FLOAT32;
	// degrees per pixel; 0 means 360 / width
	double pixel_deg = 0.0;
	// pixels equal to this are missing (NaN always is)
	double nodata = std::numeric_limits<double>::quiet_NaN();
};

// The raster and its mip pyramid (box means over valid pixels),
// sampled bilinearly across the antimeridian as docs/rgp1.md
// specifies. NaN where nothing is valid.
class RasterDem {
public:
	RasterDem() = default;
	RasterDem(const RasterDem &) = delete;
	RasterDem &operator=(const RasterDem &) = delete;
	~RasterDem();

	bool open(const RasterSpec &p_spec, int p_mip_count);
	double sample(int p_mip, double p_lat, double p_lon) const;
	int mip_count() const { return int(mips_.size()); }
	double pixel_deg() const { return mips_.empty() ? 0.0 : mips_[0].deg; }
	// mean of the first and last rows: the poles' heights
	double north_cap() const { return north_cap_; }
	double south_cap() const { return south_cap_; }

private:
	struct Mip {
		int64_t w = 0;
		int64_t h = 0;
		double deg = 0.0;
		std::vector<float> px; // empty for mip 0 (the raster)
	};

	double raw_value(int64_t p_index) const;
	double value(const Mip &p_mip, int64_t p_x, int64_t p_y) const;
	void build_mip(size_t p_level);
	double row_mean(int64_t p_y) const;

	RasterSpec spec_;
	const uint8_t *data_ = nullptr;
	void *map_ = nullptr;
	size_t map_size_ = 0;
#ifdef _WIN32
	void *file_ = nullptr;
	void *mapping_ = nullptr;
#endif
	std::vector<Mip> mips_;
	double north_cap_ = 0.0;
	double south_cap_ = 0.0;
};

} // namespace regolith::bake

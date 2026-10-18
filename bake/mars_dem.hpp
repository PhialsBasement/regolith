#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace regolith::bake {

// The HRSC/MOLA blended DEM with its mip pyramid, sampled as
// docs/rgp1.md specifies. Returns NaN where nothing is valid.
class MarsDem {
public:
	static constexpr int64_t SRC_W = 106694;
	static constexpr int64_t SRC_H = 53347;
	static constexpr uint64_t DATA_OFFSET = 854272;
	static constexpr double PIXEL_DEG = 0.003374120830641;
	static constexpr int16_t NODATA = -32768;

	~MarsDem();
	bool open(const std::string &p_path, int p_mip_count);
	double sample(int p_mip, double p_lat, double p_lon) const;
	int mip_count() const { return int(mips_.size()); }
	double north_cap() const { return north_cap_; }
	double south_cap() const { return south_cap_; }

private:
	struct Mip {
		int64_t w = 0;
		int64_t h = 0;
		double deg = 0.0;
		std::vector<float> px; // empty for mip 0 (the raster)
	};

	double value(const Mip &p_mip, int64_t p_x, int64_t p_y) const;
	void build_mip(size_t p_level);
	double row_mean(int64_t p_y) const;

	const int16_t *raw_ = nullptr;
	void *map_ = nullptr;
	size_t map_size_ = 0;
	std::vector<Mip> mips_;
	double north_cap_ = 0.0;
	double south_cap_ = 0.0;
};

} // namespace regolith::bake

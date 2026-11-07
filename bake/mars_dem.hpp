// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cstdint>
#include <string>

#include "raster_dem.hpp"

namespace regolith::bake {

// The HRSC/MOLA blended DEM: a RasterDem over the GeoTIFF's one
// uncompressed strip, whose place and size are fixed.
class MarsDem : public RasterDem {
public:
	static constexpr int64_t SRC_W = 106694;
	static constexpr int64_t SRC_H = 53347;
	static constexpr uint64_t DATA_OFFSET = 854272;
	static constexpr double PIXEL_DEG = 0.003374120830641;
	static constexpr int16_t NODATA = -32768;

	bool open(const std::string &p_path, int p_mip_count) {
		RasterSpec s;
		s.path = p_path;
		s.offset = DATA_OFFSET;
		s.width = SRC_W;
		s.height = SRC_H;
		s.type = RasterSpec::INT16;
		s.pixel_deg = PIXEL_DEG;
		s.nodata = NODATA;
		return RasterDem::open(s, p_mip_count);
	}
};

} // namespace regolith::bake

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "areoid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace regolith::bake {

bool Areoid::open(const std::string &p_path) {
	FILE *f = std::fopen(p_path.c_str(), "rb");
	if (!f) {
		std::fprintf(stderr, "areoid: cannot open %s\n", p_path.c_str());
		return false;
	}
	std::vector<uint8_t> bytes(size_t(W) * H * 2);
	const bool ok = std::fread(bytes.data(), 1, bytes.size(), f) == bytes.size();
	std::fclose(f);
	if (!ok) {
		return false;
	}
	r_.resize(size_t(W) * H);
	for (size_t k = 0; k < r_.size(); ++k) {
		const int16_t v = int16_t(uint16_t(bytes[2 * k]) << 8 | bytes[2 * k + 1]);
		r_[k] = double(v) + 3396000.0;
	}
	return true;
}

double Areoid::radius(double p_lat, double p_lon) const {
	const double lon360 = p_lon < 0.0 ? p_lon + 360.0 : p_lon;
	const double fx = lon360 * 4.0 - 0.5;
	const double fy = std::clamp((90.0 - p_lat) * 4.0 - 0.5, 0.0, double(H - 1));
	const double x_floor = std::floor(fx);
	const double tx = fx - x_floor;
	const int x0 = (int(x_floor) % W + W) % W;
	const int x1 = (x0 + 1) % W;
	const int y0 = int(std::floor(fy));
	const int y1 = std::min(y0 + 1, H - 1);
	const double ty = fy - double(y0);
	auto at = [&](int p_x, int p_y) { return r_[size_t(p_y) * W + p_x]; };
	return (1 - tx) * (1 - ty) * at(x0, y0) + tx * (1 - ty) * at(x1, y0) + (1 - tx) * ty * at(x0, y1) +
			tx * ty * at(x1, y1);
}

} // namespace regolith::bake

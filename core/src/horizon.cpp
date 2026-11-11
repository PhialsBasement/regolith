// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith/horizon.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace regolith {

namespace {

constexpr int DIR[HORIZON_DIRS][2] = { { 1, 0 }, { 1, 1 }, { 0, 1 }, { -1, 1 }, { -1, 0 }, { -1, -1 }, { 0, -1 }, { 1, -1 } };

double decode(uint8_t p_b) {
	return HORIZON_MIN + double(p_b) / 255.0 * HORIZON_SPAN;
}

uint8_t encode(double p_angle) {
	return uint8_t(std::clamp(std::lround((p_angle - HORIZON_MIN) / HORIZON_SPAN * 255.0), 0L, 255L));
}

// Parent page, bilinear, at child sample (i, j).
double parent_horizon(const std::vector<uint8_t> &p_page, const TileLayout &p_lay, int p_quadrant, double p_i, double p_j,
		int p_dir) {
	const double pi = ((p_quadrant & 1) * p_lay.intervals + p_i) * 0.5;
	const double pj = ((p_quadrant >> 1) * p_lay.intervals + p_j) * 0.5;
	return horizon_angle(p_page, p_lay, pi, pj, p_dir);
}

double smoothstep(double p_edge0, double p_edge1, double p_x) {
	const double t = std::clamp((p_x - p_edge0) / (p_edge1 - p_edge0), 0.0, 1.0);
	return t * t * (3.0 - 2.0 * t);
}

} // namespace

double horizon_angle(const std::vector<uint8_t> &p_page, const TileLayout &p_lay, double p_i, double p_j, int p_dir) {
	const int side = horizon_side(p_lay);
	const double gx = std::clamp(p_i / 4.0 + 1.0, 0.0, double(side - 1));
	const double gy = std::clamp(p_j / 4.0 + 1.0, 0.0, double(side - 1));
	const int x0 = std::min(int(gx), side - 2), y0 = std::min(int(gy), side - 2);
	const double tx = gx - x0, ty = gy - y0;
	const int layer = p_dir / 4, ch = p_dir % 4;
	auto at = [&](int p_x, int p_y) {
		return decode(p_page[(size_t(layer) * side * side + size_t(p_y) * side + size_t(p_x)) * 4 + size_t(ch)]);
	};
	return (at(x0, y0) * (1 - tx) + at(x0 + 1, y0) * tx) * (1 - ty) + (at(x0, y0 + 1) * (1 - tx) + at(x0 + 1, y0 + 1) * tx) * ty;
}

double sun_visibility(const std::vector<uint8_t> &p_page, const TileLayout &p_lay, double p_i, double p_j,
		const DVec3 &p_up, const DVec3 &p_ti, const DVec3 &p_tj, const DVec3 &p_sun) {
	const DVec3 up = normalize(p_up), sun = normalize(p_sun);
	const double elevation = std::asin(std::clamp(dot(sun, up), -1.0, 1.0));
	// the sun's bearing in the grid: solve sun_flat = a ti + b tj
	const DVec3 flat = sun - up * dot(sun, up);
	const DVec3 ti = normalize(p_ti), tj = normalize(p_tj);
	const double aa = dot(ti, ti), ab = dot(ti, tj), bb = dot(tj, tj);
	const double si = dot(flat, ti), sj = dot(flat, tj);
	const double det = aa * bb - ab * ab;
	const double a = (si * bb - sj * ab) / det;
	const double b = (sj * aa - si * ab) / det;
	const double f = std::fmod(std::atan2(b, a) / (std::numbers::pi / 4.0) + 8.0, 8.0);
	const int k0 = int(std::floor(f)) % HORIZON_DIRS;
	const int k1 = (k0 + 1) % HORIZON_DIRS;
	const double t = f - std::floor(f);
	const double horizon = horizon_angle(p_page, p_lay, p_i, p_j, k0) * (1.0 - t) + horizon_angle(p_page, p_lay, p_i, p_j, k1) * t;
	return smoothstep(-SUN_EDGE_RAD, SUN_EDGE_RAD, elevation - horizon);
}

void compute_horizon(const std::vector<DVec3> &p_pos, TileKey p_key, const TileLayout &p_lay,
		const std::vector<uint8_t> *p_parent, std::vector<uint8_t> &r_page) {
	const int n = p_lay.intervals;
	const int lo = -HORIZON_PAD, hi = n + HORIZON_PAD, w = hi - lo + 1;
	const int side = horizon_side(p_lay);
	const int quadrant = int(p_key.x() & 1u) | int((p_key.y() & 1u) << 1);
	r_page.assign(size_t(2) * side * side * 4, 0);
	for (int qj = 0; qj < side; ++qj) {
		for (int qi = 0; qi < side; ++qi) {
			const int i0 = (qi - 1) * 4, j0 = (qj - 1) * 4;
			const DVec3 p = p_pos[size_t(j0 - lo) * w + size_t(i0 - lo)];
			const DVec3 up = normalize(p);
			for (int d = 0; d < HORIZON_DIRS; ++d) {
				// sin|sin| of the elevation: same order, no sqrt
				double best = -1.0;
				int i = i0 + DIR[d][0], j = j0 + DIR[d][1];
				for (int k = 0; k < HORIZON_PAD && i >= lo && i <= hi && j >= lo && j <= hi;
						++k, i += DIR[d][0], j += DIR[d][1]) {
					const DVec3 v = p_pos[size_t(j - lo) * w + size_t(i - lo)] - p;
					const double h = dot(v, up);
					best = std::max(best, h * std::fabs(h) / dot(v, v));
				}
				const double s = best < 0.0 ? -std::sqrt(-best) : std::sqrt(best);
				double angle = std::asin(std::clamp(s, -1.0, 1.0));
				if (p_parent && !p_parent->empty()) {
					angle = std::max(angle, parent_horizon(*p_parent, p_lay, quadrant, i0, j0, d));
				}
				r_page[(size_t(d / 4) * side * side + size_t(qj) * side + size_t(qi)) * 4 + size_t(d % 4)] = encode(angle);
			}
		}
	}
}

} // namespace regolith

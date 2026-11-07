// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "catalog.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>

#include "regolith/cube.hpp"

namespace regolith::bake {

namespace {

constexpr uint32_t CRATER_MAGIC = 0x3152434D;
constexpr uint32_t PIT_MAGIC = 0x3156434D;
constexpr double GRID_W = 106496.0;
constexpr double GRID_H = 53248.0;
constexpr double M_PER_PX = 200.0;
constexpr uint32_t FLAG_PIT = 4;

double smoothstep(double p_e0, double p_e1, double p_x) {
	const double t = std::clamp((p_x - p_e0) / (p_e1 - p_e0), 0.0, 1.0);
	return t * t * (3.0 - 2.0 * t);
}

// mors tables store px = (lon + 180) / 360 * W, py likewise.
DVec3 dir_of_px(double p_px, double p_py) {
	const double lon = (p_px / GRID_W * 360.0 - 180.0) * std::numbers::pi / 180.0;
	const double lat = (90.0 - p_py / GRID_H * 180.0) * std::numbers::pi / 180.0;
	return { std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat) };
}

bool read_table(const std::string &p_path, uint32_t p_magic, int p_stride, std::vector<float> &r_values) {
	FILE *f = std::fopen(p_path.c_str(), "rb");
	if (!f) {
		std::fprintf(stderr, "catalog: cannot open %s\n", p_path.c_str());
		return false;
	}
	uint32_t head[2];
	bool ok = std::fread(head, 4, 2, f) == 2 && head[0] == p_magic;
	if (ok) {
		r_values.resize(size_t(head[1]) * p_stride);
		ok = std::fread(r_values.data(), 4, r_values.size(), f) == r_values.size();
	}
	std::fclose(f);
	if (!ok) {
		std::fprintf(stderr, "catalog: bad table %s\n", p_path.c_str());
	}
	return ok;
}

double reach(double p_radius, uint32_t p_flags) {
	return p_radius * ((p_flags & FLAG_PIT) ? 1.3 : 1.5);
}

int cell_of(double p_c) {
	return std::clamp(int(std::floor((p_c + 1.0) * 0.5 * Catalog::GRID)), 0, Catalog::GRID - 1);
}

} // namespace

bool Catalog::load_craters(const std::string &p_path) {
	std::vector<float> v;
	if (!read_table(p_path, CRATER_MAGIC, 6, v)) {
		return false;
	}
	for (size_t k = 0; k < v.size(); k += 6) {
		craters_.push_back({ dir_of_px(v[k], v[k + 1]), double(v[k + 3]) * M_PER_PX, double(v[k + 4]),
				uint32_t(v[k + 5]) & 3u });
	}
	return true;
}

bool Catalog::load_pits(const std::string &p_path) {
	std::vector<float> v;
	if (!read_table(p_path, PIT_MAGIC, 4, v)) {
		return false;
	}
	for (size_t k = 0; k < v.size(); k += 4) {
		pits_.push_back({ dir_of_px(v[k], v[k + 1]), double(v[k + 2]) * M_PER_PX, double(v[k + 3]), FLAG_PIT });
	}
	return true;
}

void Catalog::build_index() {
	features_ = craters_;
	features_.insert(features_.end(), pits_.begin(), pits_.end());
	cells_.assign(size_t(FACE_COUNT) * GRID * GRID, {});
	for (uint32_t idx = 0; idx < features_.size(); ++idx) {
		Feature &f = features_[idx];
		// Circle outline plus centre, projected onto every face
		// they can reach, bound the cells the feature touches.
		const double alpha = reach(f.radius, f.flags) / BODY_RADIUS;
		f.cos_reach = std::cos(alpha * 1.001);
		DVec3 east = cross(DVec3(0.0, 0.0, 1.0), f.dir);
		east = length(east) > 1e-12 ? normalize(east) : DVec3(0.0, 1.0, 0.0);
		const DVec3 north = cross(f.dir, east);
		std::vector<DVec3> outline{ f.dir };
		for (int k = 0; k < 32; ++k) {
			const double a = 2.0 * std::numbers::pi * k / 32.0;
			const DVec3 off = east * std::cos(a) + north * std::sin(a);
			outline.push_back(f.dir * std::cos(alpha) + off * std::sin(alpha));
		}
		for (int face = 0; face < FACE_COUNT; ++face) {
			const FaceAxes &ax = face_axes(face);
			double s_lo = 1e9, s_hi = -1e9, t_lo = 1e9, t_hi = -1e9;
			bool any = false;
			for (const DVec3 &d : outline) {
				const double w = dot(cast<double>(ax.f), d);
				if (w <= 0.3) {
					continue;
				}
				any = true;
				const double s = unwarp(dot(cast<double>(ax.u), d) / w);
				const double t = unwarp(dot(cast<double>(ax.v), d) / w);
				s_lo = std::min(s_lo, s);
				s_hi = std::max(s_hi, s);
				t_lo = std::min(t_lo, t);
				t_hi = std::max(t_hi, t);
			}
			// Lookups may overshoot the face edge by a fraction of a
			// texel, so features just past it still count here.
			const double edge = 1.01;
			if (!any || s_hi < -edge || s_lo > edge || t_hi < -edge || t_lo > edge) {
				continue;
			}
			// 32 outline points undershoot the circle by under 0.5%.
			const double pad_s = 0.01 * (s_hi - s_lo) + 1e-6;
			const double pad_t = 0.01 * (t_hi - t_lo) + 1e-6;
			for (int y = cell_of(t_lo - pad_t); y <= cell_of(t_hi + pad_t); ++y) {
				for (int x = cell_of(s_lo - pad_s); x <= cell_of(s_hi + pad_s); ++x) {
					cells_[(size_t(face) * GRID + size_t(y)) * GRID + size_t(x)].push_back(idx);
				}
			}
		}
	}
}

double Catalog::profile(const Feature &p_f, double p_rr) const {
	if (p_f.flags & FLAG_PIT) {
		return p_rr < 1.3 ? -p_f.amount * (1.0 - smoothstep(0.7, 1.15, p_rr)) : 0.0;
	}
	if (p_rr >= 1.5) {
		return 0.0;
	}
	const double deepen = p_f.amount;
	double dh;
	if (p_f.flags & 1u) {
		const double tw = std::clamp((p_rr - 0.45) / 0.55, 0.0, 1.0);
		dh = -deepen * (1.0 - tw * tw * (3.0 - 2.0 * tw));
		if (p_f.flags & 2u) {
			dh += 0.35 * deepen * std::exp(-(p_rr * p_rr) / 0.0324);
		}
	} else {
		dh = p_rr < 1.0 ? -deepen * (1.0 - p_rr * p_rr) : 0.0;
	}
	const double re = (p_rr - 1.0) / 0.16;
	dh += 0.28 * deepen * std::exp(-re * re);
	return dh * (1.0 - smoothstep(1.1, 1.5, p_rr));
}

double Catalog::stamp(int p_face, double p_s, double p_t, const DVec3 &p_dir, double p_min_diameter) const {
	if (cells_.empty()) {
		return 0.0;
	}
	const std::vector<uint32_t> &list = cells_[(size_t(p_face) * GRID + size_t(cell_of(p_t))) * GRID + size_t(cell_of(p_s))];
	double sum = 0.0;
	for (uint32_t idx : list) {
		const Feature &f = features_[idx];
		const double c = dot(f.dir, p_dir);
		if (c < f.cos_reach || 2.0 * f.radius < p_min_diameter) {
			continue;
		}
		const double ang = std::atan2(length(cross(f.dir, p_dir)), c);
		const double rr = ang * BODY_RADIUS / f.radius;
		sum += profile(f, rr);
	}
	return sum;
}

} // namespace regolith::bake

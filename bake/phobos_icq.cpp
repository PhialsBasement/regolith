// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "phobos_icq.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "regolith/cube.hpp"

namespace regolith::bake {

namespace {

int cell_of(double p_c) {
	return int(std::floor((p_c + 1.0) * 0.5 * IcqShape::GRID));
}

} // namespace

bool IcqShape::open(const std::string &p_path) {
	FILE *f = std::fopen(p_path.c_str(), "r");
	if (!f) {
		std::fprintf(stderr, "icq: cannot open %s\n", p_path.c_str());
		return false;
	}
	if (std::fscanf(f, "%d", &q_) != 1 || q_ <= 0) {
		std::fclose(f);
		return false;
	}
	const size_t per_face = size_t(q_ + 1) * size_t(q_ + 1);
	verts_.resize(6 * per_face);
	for (DVec3 &v : verts_) {
		if (std::fscanf(f, "%lf %lf %lf", &v.x, &v.y, &v.z) != 3) {
			std::fclose(f);
			return false;
		}
		v = v * 1000.0;
	}
	std::fclose(f);

	for (uint32_t face = 0; face < 6; ++face) {
		const uint32_t base = face * uint32_t(per_face);
		for (int j = 0; j < q_; ++j) {
			for (int i = 0; i < q_; ++i) {
				const uint32_t a = base + uint32_t(j * (q_ + 1) + i);
				const uint32_t b = a + 1;
				const uint32_t c = a + uint32_t(q_ + 1) + 1;
				const uint32_t d = a + uint32_t(q_ + 1);
				tris_.push_back({ a, b, c });
				tris_.push_back({ a, c, d });
			}
		}
	}

	// Each triangle goes into the buckets of every face whose
	// coordinate box it touches, so edge rays still find it.
	buckets_.assign(size_t(FACE_COUNT) * GRID * GRID, {});
	for (uint32_t t = 0; t < tris_.size(); ++t) {
		for (int face = 0; face < FACE_COUNT; ++face) {
			const FaceAxes &ax = face_axes(face);
			const DVec3 fa = cast<double>(ax.f);
			const DVec3 ua = cast<double>(ax.u);
			const DVec3 va = cast<double>(ax.v);
			double s_lo = 1e9, s_hi = -1e9, t_lo = 1e9, t_hi = -1e9;
			bool facing = true;
			for (uint32_t vi : tris_[t]) {
				const DVec3 d = normalize(verts_[vi]);
				const double w = dot(fa, d);
				if (w <= 0.2) {
					facing = false;
					break;
				}
				const double s = unwarp(dot(ua, d) / w);
				const double tt = unwarp(dot(va, d) / w);
				s_lo = std::min(s_lo, s);
				s_hi = std::max(s_hi, s);
				t_lo = std::min(t_lo, tt);
				t_hi = std::max(t_hi, tt);
			}
			const double eps = 2.0 * 2.0 / GRID;
			if (!facing || s_hi < -1.0 - eps || s_lo > 1.0 + eps || t_hi < -1.0 - eps || t_lo > 1.0 + eps) {
				continue;
			}
			const int x0 = std::clamp(cell_of(s_lo) - 1, 0, GRID - 1);
			const int x1 = std::clamp(cell_of(s_hi) + 1, 0, GRID - 1);
			const int y0 = std::clamp(cell_of(t_lo) - 1, 0, GRID - 1);
			const int y1 = std::clamp(cell_of(t_hi) + 1, 0, GRID - 1);
			for (int y = y0; y <= y1; ++y) {
				for (int x = x0; x <= x1; ++x) {
					buckets_[(size_t(face) * GRID + size_t(y)) * GRID + size_t(x)].push_back(t);
				}
			}
		}
	}
	return true;
}

double IcqShape::radius(const DVec3 &p_dir, int *r_hits) const {
	const FaceCoord fc = dir_to_face(p_dir);
	const int x = std::clamp(cell_of(fc.s), 0, GRID - 1);
	const int y = std::clamp(cell_of(fc.t), 0, GRID - 1);
	const std::vector<uint32_t> &bucket = buckets_[(size_t(fc.face) * GRID + size_t(y)) * GRID + size_t(x)];

	double hits[8];
	int count = 0;
	for (uint32_t t : bucket) {
		const DVec3 &v0 = verts_[tris_[t][0]];
		const DVec3 e1 = verts_[tris_[t][1]] - v0;
		const DVec3 e2 = verts_[tris_[t][2]] - v0;
		const DVec3 pvec = cross(p_dir, e2);
		const double det = dot(e1, pvec);
		if (std::abs(det) < 1e-12) {
			continue;
		}
		const double inv = 1.0 / det;
		const DVec3 tvec = v0 * -1.0;
		const double u = dot(tvec, pvec) * inv;
		if (u < -1e-9 || u > 1.0 + 1e-9) {
			continue;
		}
		const DVec3 qvec = cross(tvec, e1);
		const double v = dot(p_dir, qvec) * inv;
		if (v < -1e-9 || u + v > 1.0 + 1e-9) {
			continue;
		}
		const double dist = dot(e2, qvec) * inv;
		if (dist <= 0.0) {
			continue;
		}
		bool seen = false;
		for (int k = 0; k < count; ++k) {
			seen = seen || std::abs(hits[k] - dist) < 1e-6;
		}
		if (!seen && count < 8) {
			hits[count++] = dist;
		}
	}
	if (r_hits) {
		*r_hits = count;
	}
	if (count == 0) {
		return std::numeric_limits<double>::quiet_NaN();
	}
	return *std::max_element(hits, hits + count);
}

} // namespace regolith::bake

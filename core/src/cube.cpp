// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith/cube.hpp"

#include <cmath>
#include <numbers>

namespace regolith {

namespace {

constexpr double QUARTER_PI = std::numbers::pi / 4.0;

const FaceAxes AXES[FACE_COUNT] = {
	{ { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } },
	{ { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, 1 } },
	{ { 0, 1, 0 }, { -1, 0, 0 }, { 0, 0, 1 } },
	{ { 0, -1, 0 }, { 1, 0, 0 }, { 0, 0, 1 } },
	{ { 0, 0, 1 }, { 0, 1, 0 }, { -1, 0, 0 } },
	{ { 0, 0, -1 }, { 0, 1, 0 }, { 1, 0, 0 } },
};

double warp_lattice(int64_t p_c, int64_t p_m) {
	return warp(static_cast<double>(p_c) / static_cast<double>(p_m));
}

} // namespace

const FaceAxes &face_axes(int p_face) {
	return AXES[p_face];
}

double warp(double p_c) {
	if (p_c == 1.0) {
		return 1.0;
	}
	if (p_c == -1.0) {
		return -1.0;
	}
	return std::tan(QUARTER_PI * p_c);
}

double unwarp(double p_x) {
	return std::atan(p_x) / QUARTER_PI;
}

IVec3 lattice_point(const FaceSample &p_sample, int64_t p_m) {
	const FaceAxes &a = AXES[p_sample.face];
	return a.f * p_m + a.u * (2 * p_sample.gi - p_m) + a.v * (2 * p_sample.gj - p_m);
}

DVec3 lattice_dir(const IVec3 &p_point, int64_t p_m) {
	const DVec3 q(warp_lattice(p_point.x, p_m), warp_lattice(p_point.y, p_m), warp_lattice(p_point.z, p_m));
	return normalize(q);
}

int lattice_faces(const IVec3 &p_point, int64_t p_m, FaceSample r_out[3]) {
	int count = 0;
	for (int face = 0; face < FACE_COUNT; ++face) {
		const FaceAxes &a = AXES[face];
		if (dot(a.f, p_point) != p_m) {
			continue;
		}
		r_out[count++] = { face, (dot(a.u, p_point) + p_m) / 2, (dot(a.v, p_point) + p_m) / 2 };
	}
	return count;
}

IVec3 fold_lattice(int p_face, int64_t p_gi, int64_t p_gj, int64_t p_m) {
	const FaceAxes &a = AXES[p_face];
	int64_t cu = 2 * p_gi - p_m;
	int64_t cv = 2 * p_gj - p_m;
	int64_t cf = p_m;
	auto fold = [&](int64_t &r_c) {
		if (r_c > p_m) {
			cf -= r_c - p_m;
			r_c = p_m;
		} else if (r_c < -p_m) {
			cf -= -p_m - r_c;
			r_c = -p_m;
		}
	};
	fold(cu);
	fold(cv);
	return a.f * cf + a.u * cu + a.v * cv;
}

FaceSample canonical_sample(const IVec3 &p_point, int64_t p_m) {
	for (int face = 0; face < FACE_COUNT; ++face) {
		const FaceAxes &a = AXES[face];
		if (dot(a.f, p_point) == p_m) {
			return { face, (dot(a.u, p_point) + p_m) / 2, (dot(a.v, p_point) + p_m) / 2 };
		}
	}
	return {};
}

DVec3 face_dir(const FaceCoord &p_coord) {
	const FaceAxes &a = AXES[p_coord.face];
	const DVec3 q = cast<double>(a.f) + cast<double>(a.u) * warp(p_coord.s) + cast<double>(a.v) * warp(p_coord.t);
	return normalize(q);
}

FaceCoord dir_to_face(const DVec3 &p_dir) {
	const double ax = std::abs(p_dir.x);
	const double ay = std::abs(p_dir.y);
	const double az = std::abs(p_dir.z);
	int face;
	if (ax >= ay && ax >= az) {
		face = p_dir.x >= 0.0 ? FACE_POS_X : FACE_NEG_X;
	} else if (ay >= az) {
		face = p_dir.y >= 0.0 ? FACE_POS_Y : FACE_NEG_Y;
	} else {
		face = p_dir.z >= 0.0 ? FACE_POS_Z : FACE_NEG_Z;
	}
	const FaceAxes &a = AXES[face];
	const double w = dot(cast<double>(a.f), p_dir);
	return { face, unwarp(dot(cast<double>(a.u), p_dir) / w), unwarp(dot(cast<double>(a.v), p_dir) / w) };
}

} // namespace regolith

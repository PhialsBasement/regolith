#pragma once

#include <cstdint>

#include "regolith/vec.hpp"

namespace regolith {

// IAU body-fixed frame: +Z north pole, +X prime meridian.
enum Face : uint8_t {
	FACE_POS_X,
	FACE_NEG_X,
	FACE_POS_Y,
	FACE_NEG_Y,
	FACE_POS_Z,
	FACE_NEG_Z,
};
constexpr int FACE_COUNT = 6;

struct FaceAxes {
	IVec3 f; // outward normal
	IVec3 u; // direction of increasing s
	IVec3 v; // direction of increasing t, u x v == f
};

const FaceAxes &face_axes(int p_face);

// Grid sample on one face; gi and gj run 0..m intervals.
struct FaceSample {
	int face = 0;
	int64_t gi = 0;
	int64_t gj = 0;
};

// Integer point on the cube surface, components in [-m, m].
// Every face that shares an edge or corner gets the same one.
IVec3 lattice_point(const FaceSample &p_sample, int64_t p_m);

// Bit-identical whichever face produced the point: the axes
// are warped and summed in fixed x, y, z order.
DVec3 lattice_dir(const IVec3 &p_point, int64_t p_m);

// Faces holding the point: 1 inside, 2 on an edge, 3 at a
// corner. Returns the count written to r_out.
int lattice_faces(const IVec3 &p_point, int64_t p_m, FaceSample r_out[3]);

// Continuous face parameters, s and t in [-1, 1].
struct FaceCoord {
	int face = 0;
	double s = 0.0;
	double t = 0.0;
};

DVec3 face_dir(const FaceCoord &p_coord);
FaceCoord dir_to_face(const DVec3 &p_dir);

// tan(pi/4 * c), exactly +-1 on the face edges.
double warp(double p_c);
double unwarp(double p_x);

} // namespace regolith

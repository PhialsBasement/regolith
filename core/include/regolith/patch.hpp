// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "regolith/tile_key.hpp"
#include "regolith/vec.hpp"

namespace regolith {

// Floating origin: an anchor point with east/up/south axes,
// so the render frame is X east, Y up, Z south.
struct RenderFrame {
	DVec3 anchor;
	DVec3 east;
	DVec3 up;
	DVec3 south;

	static RenderFrame at(const DVec3 &p_anchor);
	DVec3 to_local_vec(const DVec3 &p_v) const;
	DVec3 to_local_point(const DVec3 &p_p) const;
};

struct PatchSpec {
	TileKey key;
	int intervals = 256; // tile edge length in sample steps
};

// What the vertex shader receives per node. Every value is
// small or unit scale; nothing planet sized.
struct NodeConstants {
	FVec3 center; // node centre, render frame
	FVec3 axis_u; // face axes, render frame
	FVec3 axis_v;
	FVec3 q_center; // unnormalised cube point of the centre
	float q_len = 0.0f;
	float s_c = 0.0f;
	float t_c = 0.0f;
	float cos_sc = 0.0f; // cos(pi/4 * s_c)
	float cos_tc = 0.0f;
	float inv_m = 0.0f; // 1 / face intervals at this level
	float r_center = 0.0f;
};

NodeConstants node_constants(const PatchSpec &p_spec, double p_r_center, const RenderFrame &p_frame);

// Vertex at tile-local grid (i, j) in float, as the shader
// computes it. p_dh is its radius minus the centre's.
FVec3 vertex_float(const NodeConstants &p_nc, int p_i, int p_j, int p_intervals, float p_dh);

// The same vertex from first principles, in double.
DVec3 vertex_reference(const PatchSpec &p_spec, int p_i, int p_j, double p_r, const RenderFrame &p_frame);

} // namespace regolith

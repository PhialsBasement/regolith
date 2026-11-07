#[compute]
#version 450

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

// Skylight on level ground, per (sun cosine, radius): the sky's
// radiance times cosine over the upper hemisphere, for a sun of
// illuminance 1. The terrain reads it per point.

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

#define AIR_HAS_LUTS
#include "regolith_air_common.glsli"

const ivec2 E_SIZE = ivec2(64, 16);

layout(rgba16f, set = 0, binding = 1) uniform writeonly image2D e_out;
layout(set = 0, binding = 2) uniform sampler2D t_lut;
layout(set = 0, binding = 3) uniform sampler2D ms_lut;

shared vec3 part[64];

void main() {
	ivec2 px = ivec2(gl_WorkGroupID.xy);
	uint k = gl_LocalInvocationIndex;
	vec2 uv = (vec2(px) + 0.5) / vec2(E_SIZE);
	uv = vec2(sub_to_unit(uv.x, float(E_SIZE.x)), sub_to_unit(uv.y, float(E_SIZE.y)));
	float mu_s = clamp(uv.x * 2.0 - 1.0, -1.0, 1.0);
	float r = mix(a.misc.w + 1.0, a.radii.y - 1.0, uv.y);
	vec3 o = vec3(0.0, 0.0, r);
	vec3 sun = vec3(sqrt(max(1.0 - mu_s * mu_s, 0.0)), 0.0, mu_s);
	// 64 directions over the upper hemisphere, equal solid angle
	float cz = 1.0 - (float(k / 8u) + 0.5) / 8.0;
	float az = 2.0 * PI * (float(k % 8u) + 0.5) / 8.0;
	float sz = sqrt(max(1.0 - cz * cz, 0.0));
	vec3 d = vec3(sz * cos(az), sz * sin(az), cz);
	float t_max = max(sphere_far(o, d, a.radii.y), 0.0);
	Scatter s = integrate(o, d, t_max, 16, sun, a.misc.w, t_lut, ms_lut);
	part[k] = s.L * cz;
	barrier();
	for (uint w = 32u; w > 0u; w >>= 1u) {
		if (k < w) {
			part[k] += part[k + w];
		}
		barrier();
	}
	if (k == 0u) {
		// each direction stands for 2 pi / 64 sr
		vec3 e = part[0] * (2.0 * PI / 64.0);
		e = any(isnan(e)) || any(isinf(e)) ? vec3(0.0) : e;
		imageStore(e_out, px, vec4(e, 1.0));
	}
}

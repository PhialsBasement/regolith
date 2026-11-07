#[compute]
#version 450

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

// Aerial perspective: light scattered in, and the mean
// transmittance, from the camera to 32 depths per screen cell
// (slice k ends at range * ((k + 1) / 32)^2).

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

#define AIR_HAS_LUTS
#include "regolith_air_common.glsli"

layout(rgba16f, set = 0, binding = 1) uniform writeonly image3D ap_out;
layout(set = 0, binding = 2) uniform sampler2D t_lut;
layout(set = 0, binding = 3) uniform sampler2D ms_lut;

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, AP_SIZE.xy))) {
		return;
	}
	vec2 uv = (vec2(px) + 0.5) / vec2(AP_SIZE.xy);
	vec4 vn = a.inv_proj * vec4(uv * 2.0 - 1.0, 1.0, 1.0);
	vec3 dv = normalize(vn.xyz / vn.w);
	vec3 dr = a.cam_x.xyz * dv.x + a.cam_y.xyz * dv.y + a.cam_z.xyz * dv.z;
	vec3 d = normalize(to_body(dr));
	vec3 o = a.eye.xyz;
	// from outside, the march starts where the ray enters the air
	float t_in = length(o) > a.radii.y ? sphere_near(o, d, a.radii.y) : 0.0;
	float range = a.eye.w;
	vec3 L = vec3(0.0), T = vec3(1.0);
	float t_prev = 0.0;
	for (int k = 0; k < AP_SIZE.z; k++) {
		float t_end = range * pow(float(k + 1) / float(AP_SIZE.z), 2.0);
		if (t_in >= 0.0 && t_end > t_in) {
			float t0 = max(t_prev, t_in);
			vec3 p0 = o + d * t0;
			float ground = sphere_near(p0, d, a.radii.x);
			float seg = t_end - t0;
			if (ground >= 0.0) {
				seg = min(seg, ground);
			}
			if (seg > 0.0) {
				Scatter s = integrate(p0, d, seg, 3, a.sun.xyz, a.radii.x, t_lut, ms_lut);
				L += T * s.L;
				T *= s.T;
			}
		}
		imageStore(ap_out, ivec3(px, k), vec4(L, dot(T, vec3(1.0 / 3.0))));
		t_prev = t_end;
	}
}

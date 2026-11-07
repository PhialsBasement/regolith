#[compute]
#version 450

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

// Light for one cascade: optical depth toward the sun (R) and
// straight up to the sky (G), from the same cloud density the
// march sees. The march never marches toward the light itself.

layout(local_size_x = 4, local_size_y = 4, local_size_z = 4) in;

#include "regolith_cloud_common.glsli"

layout(rg16f, set = 0, binding = 1) uniform writeonly image3D light_out;

void main() {
	ivec3 v = ivec3(gl_GlobalInvocationID);
	if (any(greaterThanEqual(v, LIGHT))) {
		return;
	}
	int k = int(p.planet.w);
	vec3 pb = from_uvw(k, (vec3(v) + 0.5) / vec3(LIGHT));
	float r = length(pb);
	vec3 up = pb / r;
	float vox = p.c[k].e.w;
	// in the planet's shadow the sun is simply gone
	float mu = dot(up, p.sun.xyz);
	bool night = mu < 0.0 && r * sqrt(max(1.0 - mu * mu, 0.0)) < p.planet.x;
	float ts = 50.0;
	if (!night) {
		ts = 0.0;
		float t = 0.0, ds = 150.0;
		for (int i = 0; i < 14; i++) {
			ts += cloud_density(pb + p.sun.xyz * (t + 0.5 * ds), vox) * ds;
			t += ds;
			ds *= 1.5;
		}
	}
	float tu = 0.0, t = 0.0, ds = 150.0;
	for (int i = 0; i < 10; i++) {
		tu += cloud_density(pb + up * (t + 0.5 * ds), vox) * ds;
		t += ds;
		ds *= 1.6;
	}
	imageStore(light_out, v, vec4(min(ts, 50.0), min(tu, 50.0), 0.0, 0.0));
}

#[compute]
#version 450

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

// Tileable cloud noise, made once on the GPU. shape: R
// Perlin-Worley, GBA Worley FBM at 2x, 4x, 8x. detail: RGB
// Worley FBM at 1x, 2x, 4x (Schneider 2015).

layout(local_size_x = 4, local_size_y = 4, local_size_z = 4) in;

layout(rgba8, set = 0, binding = 0) uniform writeonly image3D shape_out;
layout(rgba8, set = 0, binding = 1) uniform writeonly image3D detail_out;

// one dispatch over the shape cube; its corner fills detail
const int SHAPE_N = 128;
const int DETAIL_N = 32;

uint hash3(ivec3 c, uint seed) {
	uint h = seed * 0x9E3779B1u;
	h ^= uint(c.x) * 0x85EBCA77u;
	h = (h << 13) | (h >> 19);
	h ^= uint(c.y) * 0xC2B2AE3Du;
	h = (h << 13) | (h >> 19);
	h ^= uint(c.z) * 0x27D4EB2Fu;
	h ^= h >> 16;
	h *= 0x7FEB352Du;
	h ^= h >> 15;
	h *= 0x846CA68Bu;
	h ^= h >> 16;
	return h;
}

float unit(uint h) {
	return float(h & 0xFFFFFFu) / 16777216.0;
}

ivec3 wrap(ivec3 v, int n) {
	return ((v % n) + n) % n;
}

vec3 fade(vec3 t) {
	return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

const vec3 GRAD[12] = vec3[12](vec3(1, 1, 0), vec3(-1, 1, 0), vec3(1, -1, 0), vec3(-1, -1, 0), vec3(1, 0, 1),
		vec3(-1, 0, 1), vec3(1, 0, -1), vec3(-1, 0, -1), vec3(0, 1, 1), vec3(0, -1, 1), vec3(0, 1, -1),
		vec3(0, -1, -1));

// gradient noise with period n cells, about -1..1
float perlin(vec3 x, int n, uint seed) {
	ivec3 i0 = ivec3(floor(x));
	vec3 f = x - vec3(i0);
	float c[8];
	for (int i = 0; i < 8; i++) {
		ivec3 o = ivec3(i & 1, (i >> 1) & 1, (i >> 2) & 1);
		vec3 g = GRAD[hash3(wrap(i0 + o, n), seed) % 12u];
		c[i] = dot(g, f - vec3(o));
	}
	vec3 u = fade(f);
	return mix(mix(mix(c[0], c[1], u.x), mix(c[2], c[3], u.x), u.y),
			mix(mix(c[4], c[5], u.x), mix(c[6], c[7], u.x), u.y), u.z);
}

// inverted cellular noise with period n cells, 1 at the points
float worley(vec3 x, int n, uint seed) {
	ivec3 i0 = ivec3(floor(x));
	float best = 1e9;
	for (int dz = -1; dz <= 1; dz++) {
		for (int dy = -1; dy <= 1; dy++) {
			for (int dx = -1; dx <= 1; dx++) {
				ivec3 c = i0 + ivec3(dx, dy, dz);
				uint h = hash3(wrap(c, n), seed);
				vec3 pt = vec3(c) + vec3(unit(h), unit(h * 747796405u + 1u), unit(h * 2891336453u + 7u));
				vec3 d = pt - x;
				best = min(best, dot(d, d));
			}
		}
	}
	return 1.0 - min(sqrt(best), 1.0);
}

float worley_fbm(vec3 u, int n, uint seed) {
	return worley(u * float(n), n, seed) * 0.625 + worley(u * float(n * 2), n * 2, seed + 1u) * 0.25 +
			worley(u * float(n * 4), n * 4, seed + 2u) * 0.125;
}

void main() {
	ivec3 v = ivec3(gl_GlobalInvocationID);
	if (any(greaterThanEqual(v, ivec3(SHAPE_N)))) {
		return;
	}
	vec3 u = (vec3(v) + 0.5) / float(SHAPE_N);
	{
		float pn = 0.0, amp = 1.0, norm = 0.0;
		for (int o = 0; o < 4; o++) {
			int cells = 4 << o;
			pn += amp * perlin(u * float(cells), cells, 11u + uint(o));
			norm += amp;
			amp *= 0.5;
		}
		pn = pn / norm * 0.5 + 0.5;
		// Perlin dilated by the Worley FBM (Schneider)
		float wf = worley_fbm(u, 4, 101u);
		float pw = clamp((pn - (wf - 1.0)) / (2.0 - wf), 0.0, 1.0);
		imageStore(shape_out, v, vec4(pw, worley_fbm(u, 8, 201u), worley_fbm(u, 16, 301u), worley_fbm(u, 32, 401u)));
	}
	if (all(lessThan(v, ivec3(DETAIL_N)))) {
		vec3 w = (vec3(v) + 0.5) / float(DETAIL_N);
		imageStore(detail_out, v,
				vec4(worley_fbm(w, 2, 501u), worley_fbm(w, 4, 601u), worley_fbm(w, 8, 701u), 1.0));
	}
}

#[compute]
#version 450

// Multiple scattering as an isotropic source per (radius, sun
// cosine): second order light over all directions, summed as a
// geometric series of further orders (Hillaire 2020, sec. 5.5).

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

#include "regolith_air_common.glsli"

layout(rgba16f, set = 0, binding = 1) uniform writeonly image2D ms_out;
layout(set = 0, binding = 2) uniform sampler2D t_lut;

shared vec3 lum[64];
shared vec3 fms[64];

void main() {
	ivec2 px = ivec2(gl_WorkGroupID.xy);
	uint k = gl_LocalInvocationIndex;
	vec2 uv = (vec2(px) + 0.5) / vec2(MS_SIZE);
	uv = vec2(sub_to_unit(uv.x, float(MS_SIZE.x)), sub_to_unit(uv.y, float(MS_SIZE.y)));
	float mu_s = clamp(uv.x * 2.0 - 1.0, -1.0, 1.0);
	// over the table's whole height; its ground is the table's
	// base, so no sample sits inside the planet
	float r = mix(a.misc.w + 1.0, a.radii.y - 1.0, uv.y);
	float ground_r = a.misc.w;
	vec3 o = vec3(0.0, 0.0, r);
	vec3 sun = vec3(0.0, sqrt(max(1.0 - mu_s * mu_s, 0.0)), mu_s);
	// one of 64 directions spread over the sphere
	float i = (float(k % 8u) + 0.5) / 8.0;
	float j = (float(k / 8u) + 0.5) / 8.0;
	float theta = 2.0 * PI * i;
	float phi = acos(1.0 - 2.0 * j);
	vec3 d = vec3(cos(theta) * sin(phi), sin(theta) * sin(phi), cos(phi));
	float ground = sphere_near(o, d, ground_r);
	float t_max = ground > 0.0 ? ground : max(sphere_far(o, d, a.radii.y), 0.0);
	const int STEPS = 20;
	const float ISO = 1.0 / (4.0 * PI);
	vec3 L = vec3(0.0), F = vec3(0.0), T = vec3(1.0);
	float t = 0.0;
	for (int s = 0; s < STEPS; s++) {
		float t_next = t_max * float(s + 1) / float(STEPS);
		float dt = t_next - t;
		vec3 p = o + d * (t + 0.5 * dt);
		float rr = length(p);
		float ms = dot(p / rr, sun);
		Medium m = medium(rr);
		vec3 scat = m.scat_r + m.scat_m;
		vec3 t_sun = textureLod(t_lut, transmittance_uv(rr, ms), 0.0).rgb;
		float lit = sphere_near(p, sun, ground_r) < 0.0 ? 1.0 : 0.0;
		vec3 st = exp(-m.ext * dt);
		vec3 ext = max(m.ext, vec3(1e-12));
		vec3 S = t_sun * lit * scat * ISO;
		L += T * (S - S * st) / ext;
		F += T * (scat - scat * st) / ext;
		T *= st;
		t = t_next;
	}
	if (ground > 0.0) {
		// sunlight off the ground, Lambertian
		vec3 p = o + d * ground;
		vec3 n = normalize(p);
		vec3 t_sun = textureLod(t_lut, transmittance_uv(length(p), dot(n, sun)), 0.0).rgb;
		L += T * t_sun * max(dot(n, sun), 0.0) * a.albedo.rgb / PI;
	}
	// each direction stands for 4 pi / 64 of the sphere
	lum[k] = L * (4.0 * PI / 64.0);
	fms[k] = F * (4.0 * PI / 64.0);
	barrier();
	for (uint w = 32u; w > 0u; w >>= 1u) {
		if (k < w) {
			lum[k] += lum[k + w];
			fms[k] += fms[k + w];
		}
		barrier();
	}
	if (k == 0u) {
		vec3 l2 = lum[0] * ISO;
		vec3 f = fms[0] * ISO;
		vec3 ms = l2 / max(vec3(1.0) - f, vec3(1e-3));
		// never pass a bad value on: every sky pass reads this
		ms = any(isnan(ms)) || any(isinf(ms)) ? vec3(0.0) : min(ms, vec3(100.0));
		imageStore(ms_out, px, vec4(ms, 1.0));
	}
}

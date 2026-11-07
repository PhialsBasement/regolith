#[compute]
#version 450

// Transmittance to the top of the air, per (radius, cosine).

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

#include "regolith_air_common.glsli"

layout(rgba16f, set = 0, binding = 1) uniform writeonly image2D t_out;

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, T_SIZE))) {
		return;
	}
	vec2 uv = (vec2(px) + 0.5) / vec2(T_SIZE);
	uv = vec2(sub_to_unit(uv.x, float(T_SIZE.x)), sub_to_unit(uv.y, float(T_SIZE.y)));
	float r, mu;
	uv_to_transmittance(uv, r, mu);
	vec3 o = vec3(0.0, 0.0, r);
	vec3 d = vec3(sqrt(max(1.0 - mu * mu, 0.0)), 0.0, mu);
	float len = max(sphere_far(o, d, a.radii.y), 0.0);
	// steps packed toward the start, where the ray is lowest
	const int STEPS = 128;
	vec3 depth = vec3(0.0);
	for (int i = 0; i < STEPS; i++) {
		float t0 = len * pow(float(i) / float(STEPS), 2.0);
		float t1 = len * pow(float(i + 1) / float(STEPS), 2.0);
		vec3 p = o + d * (0.5 * (t0 + t1));
		depth += medium(length(p)).ext * (t1 - t0);
	}
	imageStore(t_out, px, vec4(exp(-depth), 1.0));
}

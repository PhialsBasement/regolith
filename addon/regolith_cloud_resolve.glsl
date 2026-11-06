#[compute]
#version 450

// The full march image from this frame's quarter of traced
// pixels and last frame's image, reprojected by cloud depth and
// held to the range of the fresh samples around it.

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

#include "regolith_cloud_common.glsli"

layout(rgba16f, set = 0, binding = 1) uniform writeonly image2D cloud_out;
layout(r32f, set = 0, binding = 2) uniform writeonly image2D dist_out;
layout(set = 0, binding = 3) uniform sampler2D depth_tex;
layout(set = 0, binding = 4) uniform sampler2D history_tex;
layout(set = 0, binding = 5) uniform sampler2D trace_tex;
layout(set = 0, binding = 6) uniform sampler2D trace_depth;

void main() {
	ivec2 pix = ivec2(gl_GlobalInvocationID.xy);
	if (pix.x >= int(p.size.x) || pix.y >= int(p.size.y)) {
		return;
	}
	vec2 uv = (vec2(pix) + 0.5) / p.size.xy;
	vec2 ndc = uv * 2.0 - 1.0;
	float depth = texture(depth_tex, uv).r;
	float scene = 1e30;
	if (depth > 0.0) {
		vec4 vp = p.inv_proj * vec4(ndc, depth, 1.0);
		scene = length(vp.xyz / vp.w);
	}
	imageStore(dist_out, pix, vec4(scene));
	ivec2 q = pix / 2;
	ivec2 qmax = ivec2(p.tile.xy) - 1;
	bool own = all(equal(pix - q * 2, ivec2(p.tile.zw)));
	// the fresh samples around: their range bounds the history
	vec4 lo = vec4(1e9), hi = vec4(-1e9);
	for (int y = -1; y <= 1; y++) {
		for (int x = -1; x <= 1; x++) {
			vec4 s = texelFetch(trace_tex, clamp(q + ivec2(x, y), ivec2(0), qmax), 0);
			lo = min(lo, s);
			hi = max(hi, s);
		}
	}
	vec4 fresh = own ? texelFetch(trace_tex, q, 0) : textureLod(trace_tex, uv, 0.0);
	vec4 res = fresh;
	if (p.prev_z.w > 0.5) {
		// the cloud's point, seen from last frame's eye
		vec4 vn = p.inv_proj * vec4(ndc, 1.0, 1.0);
		vec3 dv = normalize(vn.xyz / vn.w);
		vec3 dr = normalize(p.cam_x.xyz * dv.x + p.cam_y.xyz * dv.y + p.cam_z.xyz * dv.z);
		float ct = texelFetch(trace_depth, q, 0).r;
		vec3 rel = dr * ct + p.prev_move.xyz;
		vec3 pv = vec3(dot(rel, p.prev_x.xyz), dot(rel, p.prev_y.xyz), dot(rel, p.prev_z.xyz));
		vec4 clip = p.prev_proj * vec4(pv, 1.0);
		if (clip.w > 1e-6) {
			vec2 puv = clip.xy / clip.w * 0.5 + 0.5;
			if (all(greaterThan(puv, vec2(0.0))) && all(lessThan(puv, vec2(1.0)))) {
				vec4 hist = clamp(textureLod(history_tex, puv, 0.0), lo, hi);
				res = own ? mix(hist, fresh, 0.35) : hist;
			}
		}
	}
	imageStore(cloud_out, pix, res);
}

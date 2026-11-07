#[compute]
#version 450

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

// The reduced-resolution clouds over the full colour: a joint
// bilateral upsample on scene distance, so silhouettes keep
// their own clouds.

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

layout(rgba16f, set = 0, binding = 0) uniform image2D color_image;
layout(set = 0, binding = 1) uniform sampler2D depth_tex;
layout(set = 0, binding = 2) uniform sampler2D cloud_tex;
layout(set = 0, binding = 3) uniform sampler2D cloud_dist;

layout(push_constant, std430) uniform Push {
	mat4 inv_proj;
	vec4 size; // xy full size, zw cloud size
} p;

void main() {
	ivec2 pix = ivec2(gl_GlobalInvocationID.xy);
	if (pix.x >= int(p.size.x) || pix.y >= int(p.size.y)) {
		return;
	}
	vec2 uv = (vec2(pix) + 0.5) / p.size.xy;
	float depth = texture(depth_tex, uv).r;
	float scene = 1e30;
	if (depth > 0.0) {
		vec4 vp = p.inv_proj * vec4(uv * 2.0 - 1.0, depth, 1.0);
		scene = length(vp.xyz / vp.w);
	}
	// the four nearest cloud samples, weighted by how alike
	// their scene distance is to this pixel's
	vec2 cp = uv * p.size.zw - 0.5;
	ivec2 c0 = ivec2(floor(cp));
	vec2 f = cp - vec2(c0);
	vec4 acc = vec4(0.0);
	float wsum = 0.0;
	for (int i = 0; i < 4; i++) {
		ivec2 o = ivec2(i & 1, i >> 1);
		ivec2 q = clamp(c0 + o, ivec2(0), ivec2(p.size.zw) - 1);
		float wb = (o.x == 1 ? f.x : 1.0 - f.x) * (o.y == 1 ? f.y : 1.0 - f.y);
		float d = texelFetch(cloud_dist, q, 0).r;
		float rel = abs(min(d, 1e7) - min(scene, 1e7)) / max(min(scene, 1e7), 1.0);
		float w = wb * exp(-rel * 20.0) + 1e-5;
		acc += texelFetch(cloud_tex, q, 0) * w;
		wsum += w;
	}
	vec4 cl = acc / wsum;
	vec4 c = imageLoad(color_image, pix);
	imageStore(color_image, pix, vec4(c.rgb * cl.a + cl.rgb, c.a));
}

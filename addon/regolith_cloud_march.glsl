#[compute]
#version 450

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

// The cloud march: one pixel of each 2x2 block a frame, the
// resolve pass fills the rest. Every step reads the one cloud
// density (the same field at any distance); light comes baked.

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

#include "regolith_cloud_common.glsli"

layout(rgba16f, set = 0, binding = 1) uniform writeonly image2D trace_out;
layout(r32f, set = 0, binding = 2) uniform writeonly image2D trace_depth;
layout(set = 0, binding = 3) uniform sampler2D depth_tex;
layout(set = 0, binding = 5) uniform sampler3D light0;
layout(set = 0, binding = 6) uniform sampler3D light1;
layout(set = 0, binding = 7) uniform sampler3D light2;
layout(set = 0, binding = 8) uniform sampler3D light3;

vec2 light_tap(int k, vec3 uvw) {
	if (k == 0) {
		return textureLod(light0, uvw, 0.0).rg;
	} else if (k == 1) {
		return textureLod(light1, uvw, 0.0).rg;
	} else if (k == 2) {
		return textureLod(light2, uvw, 0.0).rg;
	}
	return textureLod(light3, uvw, 0.0).rg;
}

// a point in cascade k, moved back by the wind since its bake
vec3 cascade_at(int k, vec3 pb) {
	vec3 q = pb - p.c[k].off.xyz;
	float r = length(q);
	return to_uvw(k, q / r, r);
}

// optical depth to the sun and the sky, from the finest light
// cascade holding the point, faded into the next at its edge;
// past every cascade (far from the eye, seen from orbit) only the
// planet's shadow, so the night side is not lit by the hidden sun
vec2 light_at(vec3 pb) {
	for (int k = 0; k < NC; k++) {
		if (p.c[k].n.w < 0.5) {
			continue;
		}
		vec3 u = cascade_at(k, pb);
		if (!inside(u, 0.0)) {
			continue;
		}
		vec2 lt = light_tap(k, u);
		float e = min(min(u.x, 1.0 - u.x), min(u.y, 1.0 - u.y));
		float w = smoothstep(0.0, 0.1, e);
		if (w < 1.0 && k + 1 < NC && p.c[k + 1].n.w > 0.5) {
			vec3 u2 = cascade_at(k + 1, pb);
			if (inside(u2, 0.0)) {
				lt = mix(light_tap(k + 1, u2), lt, w);
			}
		}
		return lt;
	}
	return vec2(sun_hidden(pb) ? SUN_GONE_DEPTH : 0.0, 0.0);
}

vec3 to_body(vec3 v) {
	return vec3(dot(p.body_r0.xyz, v), dot(p.body_r1.xyz, v), dot(p.body_r2.xyz, v));
}

// [near, far] of the ray in a sphere of radius R about the
// body centre, from the eye at radius r, direction cosine mu
vec2 shell(float r, float mu, float R) {
	float s = sqrt(max(1.0 - mu * mu, 0.0));
	float disc = (R - r * s) * (R + r * s);
	if (disc < 0.0) {
		return vec2(1.0, -1.0);
	}
	float sq = sqrt(disc);
	return vec2(-r * mu - sq, -r * mu + sq);
}

float hg(float g, float mu) {
	float d = 1.0 + g * g - 2.0 * g * mu;
	return (1.0 - g * g) / (4.0 * PI * d * sqrt(d));
}

float ign(vec2 px) {
	px += 5.588238 * mod(p.planet.y, 64.0);
	return fract(52.9829189 * fract(dot(px, vec2(0.06711056, 0.00583715))));
}

void main() {
	ivec2 q = ivec2(gl_GlobalInvocationID.xy);
	if (q.x >= int(p.tile.x) || q.y >= int(p.tile.y)) {
		return;
	}
	// this frame's pixel of the block
	ivec2 pix = min(q * 2 + ivec2(p.tile.zw), ivec2(p.size.xy) - 1);
	vec2 uv = (vec2(pix) + 0.5) / p.size.xy;
	vec2 ndc = uv * 2.0 - 1.0;
	// reverse Z: 1 is the near plane, 0 the far
	vec4 vn = p.inv_proj * vec4(ndc, 1.0, 1.0);
	vec3 dv = normalize(vn.xyz / vn.w);
	vec3 dr = normalize(p.cam_x.xyz * dv.x + p.cam_y.xyz * dv.y + p.cam_z.xyz * dv.z);
	float depth = texture(depth_tex, uv).r;
	float scene = 1e30;
	if (depth > 0.0) {
		vec4 vp = p.inv_proj * vec4(ndc, depth, 1.0);
		scene = length(vp.xyz / vp.w);
	}
	vec4 cur = vec4(0.0, 0.0, 0.0, 1.0);
	float cloud_t = min(scene, 1e6);
	vec3 db = normalize(to_body(dr));
	float r = p.eye.w;
	float mu = dot(p.eye.xyz / r, db);
	float R = p.planet.x;
	vec2 outer = shell(r, mu, R + p.c[0].z.y);
	float t0 = max(outer.x, 0.0);
	float t1 = min(outer.y, scene);
	vec2 inner = shell(r, mu, R + p.c[0].z.x);
	if (inner.x < inner.y && inner.x > 0.0) {
		t1 = min(t1, inner.x);
	}
	if (outer.y > 0.0 && t1 > t0) {
		float cos_s = dot(db, p.sun.xyz);
		float g = p.sun_color.w;
		// three octaves of multiple scattering (Wrenninge)
		float ph0 = mix(hg(g, cos_s), hg(-0.25, cos_s), 0.2);
		float ph1 = mix(hg(g * 0.5, cos_s), hg(-0.12, cos_s), 0.2);
		float ph2 = hg(g * 0.25, cos_s);
		vec3 e_sun = p.sun_color.rgb * p.sun.w * 4.0 * PI;
		vec3 ins = vec3(0.0);
		float trans = 1.0, tw = 0.0, ww = 0.0;
		float leap = 0.0, hold = -1.0;
		float t = t0 + 50.0 * ign(vec2(pix));
		int steps = int(p.planet.z);
		for (int i = 0; i < steps && t < t1 && trans > 0.01; i++) {
			vec3 pb = p.eye.xyz + db * t;
			float rr = length(pb);
			vec3 d = pb / rr;
			float cu = dot(db, d); // + climbing
			vec2 ll = lonlat(d);
			float h = rr - R - texture(topo_tex, ll).r;
			vec4 bd = texture(band_tex, ll);
			bool in_band = bd.b > 0.0 && h > bd.r - 300.0 && h < bd.g + 300.0;
			if (!in_band && t >= hold) {
				// to the band's altitude if the ray heads there,
				// else a stride; then back once it is reached
				float need = 20000.0;
				if (bd.b > 0.0 && h < bd.r && cu > 0.02) {
					need = (bd.r - h) / cu;
				} else if (bd.b > 0.0 && h > bd.g && cu < -0.02) {
					need = (h - bd.g) / -cu;
				}
				leap = clamp(need, 200.0, 20000.0);
				t += leap;
				continue;
			}
			if (leap > 0.0) {
				hold = t;
				t = max(t - leap, t0);
				leap = 0.0;
				continue;
			}
			// resolve the sheets along the ray, and near the eye
			// the detail too: steps grow with the distance
			float dt = p.sheets.y * 0.4 / max(abs(cu), 0.03);
			dt = clamp(min(dt, max(t * 0.04, 50.0)), 50.0, 8000.0);
			float foot = t * p.ambient.w;
			float sigma = cloud_density(pb, foot);
			if (sigma > 1e-8) {
				vec2 lt = light_at(pb);
				vec3 sun = e_sun * (ph0 * exp(-lt.x) + 0.5 * ph1 * exp(-0.5 * lt.x) + 0.25 * ph2 * exp(-0.25 * lt.x));
				vec3 amb = p.ambient.rgb * (0.3 + 0.7 * exp(-0.6 * lt.y));
				// small ice crystals: a touch bluer than the light
				vec3 src = (sun + amb) * vec3(0.92, 0.97, 1.1) * sigma;
				float ext = exp(-sigma * dt);
				ins += trans * (src - src * ext) / sigma;
				float w = trans * (1.0 - ext);
				tw += t * w;
				ww += w;
				trans *= ext;
			}
			t += dt;
		}
		if (ww > 0.0) {
			cloud_t = tw / ww;
			// haze over the cloud's depth: it fades into what
			// lies behind, which already carries the scene's fog
			float blend = 1.0 - exp(-p.fog.w * cloud_t);
			ins *= 1.0 - blend;
			trans = mix(trans, 1.0, blend);
		}
		cur = vec4(ins, trans);
	}
	imageStore(trace_out, q, cur);
	imageStore(trace_depth, q, vec4(cloud_t));
}

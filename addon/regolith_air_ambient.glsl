#[compute]
#version 450

// Ambient light at the camera, for a sun of illuminance 1:
// x the sky and sunlit ground averaged over all normals (one
// colour for every surface), y irradiance on level ground.

layout(local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

#define AIR_HAS_LUTS
#include "regolith_air_common.glsli"

layout(rgba32f, set = 0, binding = 1) uniform writeonly image2D amb_out;
layout(set = 0, binding = 2) uniform sampler2D t_lut;
layout(set = 0, binding = 3) uniform sampler2D sv_lut;
// how bright the view is, for the eye: the frame where there is
// geometry, the scattered sky elsewhere (no stars: they answer
// to the eye's gain, which must not answer to them)
layout(set = 0, binding = 4) uniform sampler2D color_tex;
layout(set = 0, binding = 5) uniform sampler2D depth_tex;

shared vec3 sky_all[64];
shared vec3 sky_up[64];
shared float view_log[64];
shared float sky_lum[64];
shared float sky_cnt[64];

void main() {
	uint k = gl_LocalInvocationIndex;
	float r = clamp(a.radii.z, a.radii.x + 1.0, a.radii.y - 1.0);
	vec3 up = normalize(a.eye.xyz);
	float mu_s = dot(up, a.sun.xyz);
	// 64 directions over the upper hemisphere, equal solid angle
	vec3 all_sum = vec3(0.0), up_sum = vec3(0.0);
	for (uint j = 0u; j < 4u; j++) {
		uint s = k * 4u + j;
		float cz = 1.0 - (float(s / 16u) + 0.5) / 16.0;
		float az = 2.0 * PI * (float(s % 16u) + 0.5) / 16.0;
		vec3 L = a.radii.w > 0.5 ? textureLod(sv_lut, skyview_uv(r, cz, cos(az), false), 0.0).rgb : vec3(0.0);
		all_sum += L;
		up_sum += L * cz;
	}
	sky_all[k] = all_sum;
	sky_up[k] = up_sum;
	// a 16 x 16 grid over the view: the eye adapts to the sky
	// it looks into (black space beside a lit planet too), and
	// to the whole view only when no sky shows
	float lg = 0.0, sky_sum = 0.0, sky_n = 0.0;
	for (uint j = 0u; j < 4u; j++) {
		uint s = k * 4u + j;
		vec2 uv = (vec2(float(s % 16u), float(s / 16u)) + 0.5) / 16.0;
		vec3 c = textureLod(color_tex, uv, 0.0).rgb;
		bool is_sky = textureLod(depth_tex, uv, 0.0).r <= 0.0;
		if (is_sky) {
			c = vec3(0.0);
			float rr = length(a.eye.xyz);
			if (a.radii.w > 0.5 && rr < a.radii.y) {
				vec4 vn = a.inv_proj * vec4(uv * 2.0 - 1.0, 1.0, 1.0);
				vec3 dv = normalize(vn.xyz / vn.w);
				vec3 d = normalize(to_body(a.cam_x.xyz * dv.x + a.cam_y.xyz * dv.y + a.cam_z.xyz * dv.z));
				float cz = dot(d, up);
				vec3 d_h = d - up * cz;
				vec3 s_h = a.sun.xyz - up * mu_s;
				float lvc = length(d_h) > 1e-6 && length(s_h) > 1e-6 ? dot(normalize(d_h), normalize(s_h)) : 1.0;
				bool gnd = sphere_near(a.eye.xyz, d, a.radii.x) > 0.0;
				c = textureLod(sv_lut, skyview_uv(rr, cz, lvc, gnd), 0.0).rgb * (PI * a.sun.w * a.size.z);
			}
		}
		float lum = dot(c, vec3(0.2126, 0.7152, 0.0722));
		lg += lum;
		if (is_sky) {
			sky_sum += lum;
			sky_n += 1.0;
		}
	}
	view_log[k] = lg;
	sky_lum[k] = sky_sum;
	sky_cnt[k] = sky_n;
	barrier();
	for (uint w = 32u; w > 0u; w >>= 1u) {
		if (k < w) {
			sky_all[k] += sky_all[k + w];
			sky_up[k] += sky_up[k + w];
			view_log[k] += view_log[k + w];
			sky_lum[k] += sky_lum[k + w];
			sky_cnt[k] += sky_cnt[k + w];
		}
		barrier();
	}
	if (k == 0u) {
		// each of the 256 samples covers 2 pi / 256 sr
		float dw = 2.0 * PI / 256.0;
		vec3 radiance_sum = sky_all[0] * dw; // integral of L dw
		vec3 e_up = sky_up[0] * dw; // integral of L cos dw
		vec3 t_sun = textureLod(t_lut, transmittance_uv(r, mu_s), 0.0).rgb;
		vec3 ground = a.albedo.rgb * (t_sun * max(mu_s, 0.0) + e_up) / PI;
		// mean over normals of the irradiance: a quarter of the
		// radiance over the sphere, the ground filling the lower half
		vec3 mean_e = (radiance_sum + 2.0 * PI * ground) * 0.25;
		imageStore(amb_out, ivec2(0, 0), vec4(mean_e, 1.0));
		imageStore(amb_out, ivec2(1, 0), vec4(e_up, 1.0));
		float adapt = sky_cnt[0] >= 4.0 ? sky_lum[0] / sky_cnt[0] : view_log[0] / 256.0;
		imageStore(amb_out, ivec2(2, 0), vec4(adapt));
	}
}

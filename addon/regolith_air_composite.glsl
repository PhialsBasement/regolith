#[compute]
#version 450

// The sky and the air over the rendered scene: background pixels
// become the sky (the sky view inside the air, a ray march from
// space), sun, stars and bodies seen through it; geometry gets
// the aerial perspective for its distance.

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

#define AIR_HAS_LUTS
#include "regolith_air_common.glsli"

layout(rgba16f, set = 0, binding = 1) uniform image2D color_image;
layout(set = 0, binding = 2) uniform sampler2D depth_tex;
layout(set = 0, binding = 3) uniform sampler2D t_lut;
layout(set = 0, binding = 4) uniform sampler2D ms_lut;
layout(set = 0, binding = 5) uniform sampler2D sv_lut;
layout(set = 0, binding = 6) uniform sampler3D ap_lut;

#include "regolith_sky_objects.glsli"

// Airglow: the air's own faint light, emitted in proportion to
// its density, so a ray's glow is the air along it: faint at the
// zenith (size.w per sr there), an arc along the limb from orbit
float airglow(vec3 o, vec3 d, float t_end) {
	if (a.radii.w < 0.5 || a.size.w <= 0.0) {
		return 0.0;
	}
	float r0 = length(o);
	float t0 = r0 > a.radii.y ? sphere_near(o, d, a.radii.y) : 0.0;
	float t1 = r0 > a.radii.y ? sphere_far(o, d, a.radii.y) : max(sphere_far(o, d, a.radii.y), 0.0);
	if (t0 < 0.0 || t1 <= t0) {
		return 0.0;
	}
	t1 = min(t1, t_end);
	float h_scale = a.rayleigh.w;
	float sum = 0.0;
	const int STEPS = 16;
	for (int i = 0; i < STEPS; i++) {
		float ta = mix(t0, t1, float(i) / float(STEPS));
		float tb = mix(t0, t1, float(i + 1) / float(STEPS));
		float h = length(o + d * (0.5 * (ta + tb))) - a.misc.z;
		sum += exp(-h / h_scale) * (tb - ta);
	}
	// emission per metre at the datum: the zenith column is size.w
	return sum * a.size.w / h_scale;
}

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (px.x >= int(a.size.x) || px.y >= int(a.size.y)) {
		return;
	}
	vec2 uv = (vec2(px) + 0.5) / a.size.xy;
	vec2 ndc = uv * 2.0 - 1.0;
	float depth = texture(depth_tex, uv).r;
	vec4 vn = a.inv_proj * vec4(ndc, 1.0, 1.0);
	vec3 dv = normalize(vn.xyz / vn.w);
	vec3 dr = a.cam_x.xyz * dv.x + a.cam_y.xyz * dv.y + a.cam_z.xyz * dv.z;
	vec3 d = normalize(to_body(dr));
	vec3 o = a.eye.xyz;
	float r = length(o);
	bool air = a.radii.w > 0.5;
	float scale = PI * a.sun.w * a.size.z;
	vec4 c = imageLoad(color_image, px);
	if (depth > 0.0) {
		if (!air) {
			return;
		}
		vec4 vp = a.inv_proj * vec4(ndc, depth, 1.0);
		float dist = length(vp.xyz / vp.w);
		float range = a.eye.w;
		float f = sqrt(dist / range) * float(AP_SIZE.z) - 1.0;
		float w = (max(f, 0.0) + 0.5) / float(AP_SIZE.z);
		vec4 ap = textureLod(ap_lut, vec3(uv, w), 0.0);
		if (f < 0.0) {
			// inside the first slice: scale toward the eye
			float k = max(f + 1.0, 0.0);
			ap = vec4(ap.rgb * k, mix(1.0, ap.a, k));
		}
		// the glowing air between the eye and the ground, in the
		// stars' units (the eye's gain)
		vec3 glow = a.glow_color.rgb * (airglow(o, d, dist) * a.stars.y);
		imageStore(color_image, px, vec4(c.rgb * ap.a + ap.rgb * scale + glow, c.a));
		return;
	}
	vec3 L = vec3(0.0);
	vec3 Tv = vec3(1.0);
	bool ground = sphere_near(o, d, a.radii.x) > 0.0;
	if (air && r < a.radii.y) {
		vec3 up = o / r;
		float cz = dot(d, up);
		vec3 d_h = d - up * cz;
		vec3 s_h = a.sun.xyz - up * dot(a.sun.xyz, up);
		float lvc = length(d_h) > 1e-6 && length(s_h) > 1e-6 ? dot(normalize(d_h), normalize(s_h)) : 1.0;
		L = textureLod(sv_lut, skyview_uv(r, cz, lvc, ground), 0.0).rgb;
		Tv = ground ? vec3(0.0) : textureLod(t_lut, transmittance_uv(r, cz), 0.0).rgb;
	} else if (air) {
		float t_in = sphere_near(o, d, a.radii.y);
		if (t_in >= 0.0) {
			vec3 p = o + d * t_in;
			float g = sphere_near(p, d, a.radii.x);
			float t_max = g > 0.0 ? g : max(sphere_far(p, d, a.radii.y), 0.0);
			Scatter s = integrate(p, d, t_max, 24, a.sun.xyz, a.radii.x, t_lut, ms_lut);
			L = s.L;
			Tv = g > 0.0 ? vec3(0.0) : s.T;
		}
	} else if (ground) {
		Tv = vec3(0.0);
	}
	vec3 col = L * scale;
	// the sun's disc, limb darkened, through the air
	float cos_sun = dot(d, a.sun.xyz);
	float sr = a.misc.y;
	float ang = acos(clamp(cos_sun, -1.0, 1.0));
	if (ang < sr) {
		float q = ang / sr;
		float limb = 1.0 - 0.6 * (1.0 - sqrt(max(1.0 - q * q, 0.0)));
		vec3 disc = vec3(a.sun.w * a.size.z / (sr * sr)) * limb;
		col += min(disc, vec3(2000.0)) * Tv;
	}
	col += sky_objects(d, Tv, ang);
	// airglow along this ray, to the ground or out of the air
	float t_ground = sphere_near(o, d, a.radii.x);
	col += a.glow_color.rgb * (airglow(o, d, t_ground > 0.0 ? t_ground : 1e30) * a.stars.y);
	imageStore(color_image, px, vec4(col, c.a));
}

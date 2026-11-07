#[compute]
#version 450

// The sky around the camera (inside the air): luminance per
// view zenith and angle from the sun's azimuth.

layout(local_size_x = 8, local_size_y = 8, local_size_z = 1) in;

#define AIR_HAS_LUTS
#include "regolith_air_common.glsli"

layout(rgba16f, set = 0, binding = 1) uniform writeonly image2D sv_out;
layout(set = 0, binding = 2) uniform sampler2D t_lut;
layout(set = 0, binding = 3) uniform sampler2D ms_lut;

void main() {
	ivec2 px = ivec2(gl_GlobalInvocationID.xy);
	if (any(greaterThanEqual(px, SV_SIZE))) {
		return;
	}
	vec2 uv = (vec2(px) + 0.5) / vec2(SV_SIZE);
	uv = vec2(sub_to_unit(uv.x, float(SV_SIZE.x)), sub_to_unit(uv.y, float(SV_SIZE.y)));
	float bot = a.radii.x;
	float r = min(a.radii.z, a.radii.y - 1.0);
	float v_horizon = sqrt(max(sq_diff(r, bot), 0.0));
	float beta = acos(clamp(v_horizon / r, -1.0, 1.0));
	float zenith_horizon = PI - beta;
	float view_zenith;
	if (uv.y < 0.5) {
		float coord = 1.0 - 2.0 * uv.y;
		view_zenith = zenith_horizon * (1.0 - coord * coord);
	} else {
		float coord = uv.y * 2.0 - 1.0;
		view_zenith = zenith_horizon + beta * coord * coord;
	}
	float light_view = -((uv.x * uv.x) * 2.0 - 1.0);
	// the local frame: up, toward the sun's azimuth, and across
	vec3 up = normalize(a.eye.xyz);
	vec3 sun_h = a.sun.xyz - up * dot(a.sun.xyz, up);
	vec3 x = length(sun_h) > 1e-5 ? normalize(sun_h) : normalize(cross(up, abs(up.z) < 0.9 ? vec3(0, 0, 1) : vec3(1, 0, 0)));
	vec3 y = cross(up, x);
	float sz = sin(view_zenith);
	float az = acos(clamp(light_view, -1.0, 1.0));
	vec3 d = normalize(x * (sz * cos(az)) + y * (sz * sin(az)) + up * cos(view_zenith));
	vec3 o = up * r;
	float ground = sphere_near(o, d, bot);
	float t_max = ground > 0.0 ? ground : max(sphere_far(o, d, a.radii.y), 0.0);
	Scatter s = integrate(o, d, t_max, 30, a.sun.xyz, a.radii.x, t_lut, ms_lut);
	imageStore(sv_out, px, vec4(s.L, 1.0));
}

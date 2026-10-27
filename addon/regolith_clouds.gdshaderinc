// mors's cloudscape density (mars_cloudscape.gdshader) on the
// real sphere: the field comes from latitude and longitude, the
// noise from a tangent chart of the body (east, altitude, south).
uniform sampler2D local_tex : filter_linear, repeat_disable;
uniform sampler2D global_tex : filter_linear, repeat_enable;
uniform sampler3D noise_tex : filter_linear, repeat_enable;
uniform vec2 window_org_px = vec2(0.0);
uniform vec2 planet_px = vec2(106496.0, 53248.0);
uniform float win_px = 2000.0; // fine window, planet px
uniform vec3 u_center; // body centre, relative to the camera
uniform vec3 u_eye_b; // the eye, body-fixed metres
uniform mat3 u_to_body; // render to body-fixed
uniform vec3 u_chart_e = vec3(0.0, 1.0, 0.0);
uniform vec3 u_chart_s = vec3(-1.0, 0.0, 0.0);
uniform float u_datum_r = 3396000.0;
uniform float ground_alt = 0.0; // ground under the eye, over datum
uniform vec3 sun_body = vec3(1.0, 0.0, 0.0);
uniform vec2 drift_air = vec2(0.0);
uniform vec2 drift_dust = vec2(0.0);
uniform float density_mult = 1.0;

const float PI_C = 3.14159265;

vec2 px_of(vec3 dir) {
	float lat = asin(clamp(dir.z, -1.0, 1.0));
	float lon = atan(dir.y, dir.x);
	return vec2((lon / (2.0 * PI_C) + 0.5) * planet_px.x, (0.5 - lat / PI_C) * planet_px.y);
}

// the simulated field (0..1.5), domain warped so grid cells
// never read as squares
float field_q(vec3 dir, int layer) {
	vec2 px = px_of(dir);
	vec2 wrp = vec2(
		texture(noise_tex, vec3(px * 0.004, 0.17 + float(layer) * 0.29)).r,
		texture(noise_tex, vec3(px * 0.004, 0.62 + float(layer) * 0.29)).r) - 0.5;
	px += wrp * 280.0;
	vec2 luv = (px - window_org_px) / win_px;
	vec2 guv = vec2(fract(px.x / planet_px.x), clamp(px.y / planet_px.y, 0.0, 1.0));
	float scl = layer == 3 ? 2.0 : 1.5;
	vec4 sel = vec4(float(layer == 0), float(layer == 1), float(layer == 2), float(layer == 3));
	float g = dot(texture(global_tex, guv), sel) * scl;
	if (luv.x <= 0.0 || luv.y <= 0.0 || luv.x >= 1.0 || luv.y >= 1.0) {
		return g;
	}
	float l = dot(texture(local_tex, luv), sel) * scl;
	float rim = smoothstep(0.0, 0.08, min(min(luv.x, luv.y), min(1.0 - luv.x, 1.0 - luv.y)));
	return mix(g, l, rim);
}

// slab per layer in altitude over the datum (3: lifted dust)
void slab_bounds(int layer, out float b, out float t) {
	if (layer == 0) {
		b = ground_alt + 2200.0;
		t = ground_alt + 6800.0;
	} else if (layer == 1) {
		// cirrus lifts over high terrain (Tharsis)
		b = max(18500.0, ground_alt + 3000.0);
		t = max(26500.0, b + 4000.0);
	} else if (layer == 2) {
		b = max(60000.0, ground_alt + 3000.0);
		t = max(78000.0, ground_alt + 7000.0);
	} else {
		b = ground_alt - 800.0;
		t = ground_alt + 5200.0;
	}
}

// q: body-fixed metres
float layer_density(vec3 q, int layer) {
	float rq = length(q);
	vec3 dir = q / rq;
	float alt = rq - u_datum_r;
	float b;
	float tp;
	slab_bounds(layer, b, tp);
	float h = (alt - b) / (tp - b);
	if (h < 0.0 || h > 1.0) {
		return 0.0;
	}
	float q_f = field_q(dir, layer);
	if (q_f < 0.05) {
		return 0.0;
	}
	float env = layer == 3 ? 1.0 - smoothstep(0.25, 1.0, h)
			: smoothstep(0.0, 0.25, h) * (1.0 - smoothstep(0.55, 1.0, h));
	if (env <= 0.001) {
		return 0.0;
	}
	// mors's world point: x east, y height, z south
	vec3 p = vec3(dot(q, u_chart_e), alt, dot(q, u_chart_s));
	if (layer == 3) {
		vec3 spd = (p - vec3(drift_dust.x, 0.0, drift_dust.y)) * vec3(7e-5, 1.2e-4, 7e-5);
		float nd = texture(noise_tex, spd).r * 0.7 + texture(noise_tex, spd * 3.3 + vec3(0.3)).r * 0.45;
		float covd = clamp(q_f * 0.85, 0.0, 1.0);
		return max(nd - (1.0 - covd), 0.0) * env * 5.0 * density_mult;
	}
	if (layer == 0) {
		vec3 sp = (p - vec3(drift_air.x, 0.0, drift_air.y)) * vec3(9e-5, 1.6e-4, 9e-5);
		float n = texture(noise_tex, sp).r * 0.65 + texture(noise_tex, sp * 3.1 + vec3(0.4)).r * 0.5;
		float cov = clamp(q_f * 1.5, 0.0, 1.0);
		return max(n - (1.04 - cov), 0.0) * env * 3.2 * density_mult;
	}
	// wind-combed cirrus, the comb turning with position
	float ca = (texture(noise_tex, vec3(p.xz * 3.5e-6, 0.81)).r - 0.5) * 2.4;
	float cc2 = cos(ca);
	float ss2 = sin(ca);
	vec3 pr = vec3(p.x * cc2 - p.z * ss2, p.y, p.x * ss2 + p.z * cc2);
	vec2 dr = vec2(drift_air.x * cc2 - drift_air.y * ss2, drift_air.x * ss2 + drift_air.y * cc2);
	vec3 sp = (pr - vec3(dr.x, 0.0, dr.y)) * vec3(2.2e-5, 1.1e-4, 8e-5);
	float n = texture(noise_tex, sp).r * 0.6 + texture(noise_tex, sp * vec3(4.7, 2.0, 4.7) + vec3(0.7)).r * 0.5;
	float big = texture(noise_tex, vec3(p.xz * 1.6e-6, 0.29)).r;
	n *= 0.5 + 0.95 * big;
	float thin = layer == 2 ? 0.55 : 1.0;
	float cov2 = clamp(q_f * 1.8, 0.0, 1.0) * 0.92;
	return max(n - (1.04 - cov2), 0.0) * env * 1.9 * thin * density_mult;
}

// noctilucent: altitude alt sees the sun sqrt(2h/R) past the
// ground horizon
float sunlit_at(vec3 q) {
	float rq = length(q);
	float dep = sqrt(max(2.0 * max(rq - u_datum_r, 0.0) / u_datum_r, 0.0));
	return smoothstep(-0.02, 0.08, dot(sun_body, q / rq) + dep);
}

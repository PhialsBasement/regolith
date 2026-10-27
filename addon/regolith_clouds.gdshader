shader_type spatial;
render_mode unshaded, blend_mix, depth_draw_never, cull_front, fog_disabled;

// mors's cloudscape (mars_cloudscape.gdshader): three ice slabs
// and the lifted dust, ray marched against the GCM's fields, as
// shells around the true body centre.
#include "res://addons/regolith/regolith_clouds.gdshaderinc"

uniform sampler2D depth_tex : hint_depth_texture, filter_linear;
uniform float orbit_fade : hint_range(0.0, 1.0) = 1.0;

// Entry and exit of a shell of radius rc + dr for a ray from a
// point rc from the centre, mu the cosine to the outward radial:
// t^2 + 2 b t - k = 0, solved without cancellation.
vec2 shell_hits(float rc, float mu, float dr) {
	float r = rc + dr;
	float k = dr * (rc + r);
	float b = rc * mu;
	float disc = b * b + k;
	if (disc < 0.0) {
		return vec2(-1.0, -1.0);
	}
	float s = sqrt(disc);
	float far_t = b > 0.0 ? k / (b + s) : s - b;
	float near_t = b > 0.0 ? -(b + s) : (abs(far_t) > 0.0 ? -k / far_t : -s - b);
	return vec2(min(near_t, far_t), max(near_t, far_t));
}

void shell_interval(float rc, float mu, float hb, float ht, float h_eye, out float t0, out float t1) {
	vec2 hi = shell_hits(rc, mu, hb - h_eye);
	vec2 ho = shell_hits(rc, mu, ht - h_eye);
	if (h_eye < hb) {
		t0 = hi.y;
		t1 = ho.y;
	} else if (h_eye <= ht) {
		t0 = 0.0;
		t1 = hi.x > 0.0 ? hi.x : ho.y;
	} else if (ho.x < 0.0) {
		t0 = 1.0e12;
		t1 = -1.0;
	} else {
		t0 = ho.x;
		t1 = hi.x > 0.0 ? hi.x : ho.y;
	}
}

vec4 march_slab(vec3 ro_b, vec3 rd_b, float t0, float t1, int layer, int steps, float jit) {
	float dt = (t1 - t0) / float(steps);
	float t = t0 + dt * jit;
	float trans = 1.0;
	vec3 acc = vec3(0.0);
	float mu = max(dot(rd_b, sun_body), 0.0);
	float phase = 0.55 + 0.55 * mu * mu + 0.6 * pow(mu, 10.0);
	float sun_up = dot(sun_body, normalize(ro_b));
	float twil = 1.0 - smoothstep(0.05, 0.25, sun_up);
	vec3 ice = mix(vec3(0.96, 0.93, 0.90), vec3(1.0, 0.72, 0.70), twil * (layer > 0 ? 1.0 : 0.4));
	if (layer == 3) {
		ice = vec3(0.66, 0.47, 0.32);
	}
	float ext = layer == 0 ? 0.0018 : (layer == 3 ? 0.0030 : 0.0009);
	for (int i = 0; i < steps; i++) {
		vec3 q = ro_b + rd_b * t;
		float den = layer_density(q, layer);
		if (den > 0.004) {
			float lit = sunlit_at(q);
			float od = layer_density(q + sun_body * 500.0, layer) * 500.0
				+ layer_density(q + sun_body * 1500.0, layer) * 1000.0;
			float sun_t = exp(-od * ext);
			float q_raw = field_q(normalize(q), layer);
			float dark = layer == 0 ? smoothstep(0.85, 1.35, q_raw) : 0.0;
			vec3 lit_col = ice * (1.0 - 0.45 * dark);
			vec3 shad_col = (layer == 3 ? vec3(0.24, 0.15, 0.10) : vec3(0.34, 0.30, 0.29)) * (1.0 - 0.5 * dark);
			vec3 lum = mix(shad_col, lit_col * phase, sun_t) * lit
				+ vec3(0.020, 0.017, 0.016) * (0.3 + 0.7 * lit);
			float aa = 1.0 - exp(-den * dt * ext);
			acc += trans * aa * lum;
			trans *= 1.0 - aa;
			if (trans < 0.02) {
				break;
			}
		}
		t += dt;
	}
	acc = acc / (1.0 + acc * 0.3);
	return vec4(acc, 1.0 - trans);
}

void fragment() {
	vec3 dir_view = normalize(VERTEX);
	vec3 rd = normalize((INV_VIEW_MATRIX * vec4(dir_view, 0.0)).xyz);
	vec3 ro_b = u_to_body * (CAMERA_POSITION_WORLD - u_center);
	vec3 rd_b = normalize(u_to_body * rd);
	float rc = length(ro_b);
	float h_eye = rc - u_datum_r;
	float mu = dot(rd_b, ro_b / rc);

	float depth_raw = texture(depth_tex, SCREEN_UV).x;
	float t_scene = 1.0e12;
	if (depth_raw > 1.0e-6) {
		vec3 ndc = vec3(SCREEN_UV * 2.0 - 1.0, depth_raw);
		vec4 vpos = INV_PROJECTION_MATRIX * vec4(ndc, 1.0);
		vpos.xyz /= vpos.w;
		t_scene = -vpos.z / max(-dir_view.z, 1.0e-4);
	}
	float t_max = min(t_scene, 480000.0);

	float ts[4];
	float te[4];
	int order[4] = {0, 1, 2, 3};
	for (int l = 0; l < 4; l++) {
		float b;
		float tp;
		slab_bounds(l, b, tp);
		float t0;
		float t1;
		shell_interval(rc, mu, b, tp, h_eye, t0, t1);
		ts[l] = max(t0, 0.0);
		te[l] = min(min(t1, t_max), ts[l] + 260000.0);
	}
	for (int a = 0; a < 3; a++) {
		for (int b2 = a + 1; b2 < 4; b2++) {
			if (ts[order[b2]] < ts[order[a]]) {
				int tmp = order[a];
				order[a] = order[b2];
				order[b2] = tmp;
			}
		}
	}
	vec3 col = vec3(0.0);
	float alpha = 0.0;
	for (int k = 0; k < 4; k++) {
		int l = order[k];
		if (te[l] <= ts[l]) {
			continue;
		}
		int steps = l == 0 ? 12 : (l == 1 ? 9 : (l == 2 ? 6 : 10));
		// interleaved gradient noise per pixel
		float jit = fract(52.9829189 * fract(
			dot(FRAGCOORD.xy + vec2(float(l) * 17.0, float(l) * 29.0), vec2(0.06711056, 0.00583715))));
		vec4 c = march_slab(ro_b, rd_b, ts[l], te[l], l, steps, jit);
		col += (1.0 - alpha) * c.rgb * c.a;
		alpha += (1.0 - alpha) * c.a;
		if (alpha > 0.97) {
			break;
		}
	}
	alpha *= orbit_fade;
	if (alpha < 0.004) {
		discard;
	}
	ALBEDO = col / max(alpha, 1.0e-4);
	ALPHA = alpha;
}

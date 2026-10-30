shader_type spatial;
render_mode cull_disabled, diffuse_burley, specular_schlick_ggx, motion_vectors_rigid;

// Geometry: clipmap rings of a cube-sphere (RegolithBody::
// draw_clipmap). Shading: regolith_terrain.gdshader's, with each
// pixel finding its own height page through the page table.

// Per ring, set by RegolithBody::draw_clipmap.
instance uniform vec4 u_center; // ring centre (push frame), r_center
instance uniform vec4 u_qc; // warped cube point of the centre, |q|
instance uniform vec4 u_cc; // centre cube point (lattice), m
instance uniform vec4 u_ccos; // cos(pi/4 * centre / m), stride
instance uniform vec4 u_ring; // grid min corner (face0 lattice), level, face0
instance uniform vec4 u_debug; // ring, quads, rho_c, skirt depth
instance uniform vec4 u_hole; // finer ring's footprint, grid steps
instance uniform vec4 u_dom0; // texture domains: fract uv, hex lattice
instance uniform vec4 u_dom1;
instance uniform vec4 u_dom2;
instance uniform vec4 u_dom3;
instance uniform vec4 u_dom4;
instance uniform vec4 u_cliff_u;
instance uniform vec4 u_cliff_v;
instance uniform vec4 u_nmod; // face metres mod 20 km

uniform sampler2DArray heights : filter_nearest, repeat_disable;
// (layer or -1, tx, ty) per face, level and tile mod page_window
uniform sampler2D page_table : filter_nearest, repeat_disable;
uniform int page_window = 8;
uniform int clip_quads = 256;
// Per ring r (layer r): heights at its grid points, horizon and
// geology at a quarter of that, toroidal (absolute step mod size)
// ring_geo[r]: grid min corner (face0 lattice), stride, level;
// stride 0: no such ring
uniform vec4 ring_geo[24];
uniform sampler2DArray ring_heights : filter_linear, repeat_enable;
uniform sampler2DArray ring_horizon_a : filter_linear, repeat_enable;
uniform sampler2DArray ring_horizon_b : filter_linear, repeat_enable;
uniform sampler2DArray ring_geology : filter_linear, repeat_enable;
uniform int ring_size = 260;
uniform int ring_qsize = 68;
uniform sampler2DArray geology : filter_linear, repeat_disable;
// Terrain shadows: horizon angles in 8 grid directions
uniform sampler2DArray horizon_a : filter_linear, repeat_disable;
uniform sampler2DArray horizon_b : filter_linear, repeat_disable;
uniform int horizon_side = 55;
// Cloud shadows: sun transmittance on a ground-plane map around
// the eye (RegolithWeather); clear outside it.
uniform sampler2D cloud_shadow : filter_linear, repeat_disable;
uniform vec3 cloud_origin; // relative to the camera
uniform vec3 cloud_e;
uniform vec3 cloud_s;
uniform float cloud_size = 100000.0;
uniform bool has_cloud_shadow = false;
uniform int intervals = 208;
uniform int border = 2;
uniform int geology_side = 55;
uniform float face_edge_m = 5324000.0;
uniform int debug_mode = 0;
// profiling: each set bit skips one section, neutral output;
// 1 heights normal, 2 sun, 4 noise, 8<<id blend layer id,
// 512 macro, 2048 full work in depth passes
uniform int prof_skip = 0;
uniform vec3 u_eye; // main camera, push-frame metres
// Dust optical depth: sets how much sunlight arrives direct
// (shadowable) and how much as scattered skylight.
uniform float dust_tau = 0.35;

// mors ground material (Terrain3D asset values)
uniform sampler2DArray albedo_array : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2DArray normal_array : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2D noise_texture : filter_linear_mipmap, repeat_enable;
uniform bool has_textures = false;
const vec4 layer_color[6] = { vec4(1.0, 1.0, 0.82, 1.0), vec4(1.0, 1.0, 0.82, 1.0), vec4(1.0, 1.0, 0.82, 1.0),
	vec4(1.0, 1.0, 0.82, 1.0), vec4(1.0), vec4(1.0) };
const float layer_normal_depth[6] = { 1.0, 0.5, 1.0, 1.0, 0.5, 1.0 };
const float layer_ao_strength[6] = { 0.5, 0.5, 0.5, 0.5, 0.5, 0.5 };
const float layer_roughness_mod[6] = { 0.0, 0.35, 0.0, 0.0, 0.0, 0.0 };
uniform float blend_sharpness : hint_range(0, 1) = 0.65;
uniform float bedrock_amount : hint_range(0., 1.) = 0.35;
uniform float gravel_amount : hint_range(0., 1.) = 0.45;
uniform float slope_rock_start : hint_range(0., 1.) = 0.10;
uniform bool enable_macro_variation = true;
uniform vec3 macro_variation1 : source_color = vec3(0.84, 0.84, 0.88);
uniform vec3 macro_variation2 : source_color = vec3(1.0, 0.95, 0.90);
uniform float macro_variation_slope : hint_range(0., 1.) = 0.4;
uniform float frost_amt : hint_range(0., 1.) = 0.0;
uniform float roughness_floor : hint_range(0., 1.) = 0.9;
uniform float specular_amount : hint_range(0., 1.) = 0.15;
// mors's isomorphic far view; atmosphere 0 = airless
uniform sampler2D iso_map : source_color, filter_linear_mipmap, repeat_enable;
uniform bool has_iso_map = false;
uniform float shade_vex = 12.0; // the I-mode drama knob
uniform float iso_night = 0.16; // unlit ground floor
uniform float atmosphere = 0.0;
uniform mat3 u_to_body = mat3(1.0); // node space -> body-fixed
uniform vec3 sun_body = vec3(0.0, 0.0, 1.0);
uniform vec3 dust_colour : source_color = vec3(0.62, 0.36, 0.22);
uniform vec3 rock_colour : source_color = vec3(0.30, 0.21, 0.16);

varying vec2 v_g; // data units from the ring's min corner
varying vec2 v_local; // face metres from the ring centre
varying float v_rho;
varying vec3 v_ts; // position per data unit along grid x
varying vec3 v_tt; // and along grid y
varying vec3 v_up;
varying float v_morph;
varying float v_sun_vis; // terrain x clouds, for the sun only
varying vec3 v_up_view; // local vertical, view space
varying float v_spec; // the material's SPECULAR, for light()

const float QP = 0.7853981633974483;
// regolith/cube.cpp's face axes: f normal, u x v == f
const ivec3 FA_F[6] = { ivec3(1, 0, 0), ivec3(-1, 0, 0), ivec3(0, 1, 0), ivec3(0, -1, 0), ivec3(0, 0, 1), ivec3(0, 0, -1) };
const ivec3 FA_U[6] = { ivec3(0, 1, 0), ivec3(0, -1, 0), ivec3(-1, 0, 0), ivec3(1, 0, 0), ivec3(0, 1, 0), ivec3(0, 1, 0) };
const ivec3 FA_V[6] = { ivec3(0, 0, 1), ivec3(0, 0, 1), ivec3(0, 0, 1), ivec3(0, 0, 1), ivec3(-1, 0, 0), ivec3(1, 0, 0) };

int idot(ivec3 a, ivec3 b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

float texel(int layer, ivec2 ij) {
	return texelFetch(heights, ivec3(ij + ivec2(border), layer), 0).r;
}

float bilinear(int layer, vec2 ij) {
	vec2 f = floor(ij);
	vec2 t = ij - f;
	ivec2 i0 = ivec2(f);
	float a = texel(layer, i0);
	float b = texel(layer, i0 + ivec2(1, 0));
	float c = texel(layer, i0 + ivec2(0, 1));
	float d = texel(layer, i0 + ivec2(1, 1));
	return mix(mix(a, b, t.x), mix(c, d, t.x), t.y);
}

// One unfolded axis x (cube units, int + small float) walked over
// the cube: along-axis a and normal f, and their rates per x.
// RegolithBody's walk() in C++ does the same.
void walk(int xi, float xf, int m, out int ai, out float af, out int fi, out float ff, out vec2 rate) {
	xi = clamp(xi, -5 * m, 5 * m);
	float over = float(xi - m) + xf;
	float under = float(-m - xi) - xf;
	if (over > 0.0) {
		if (over <= float(2 * m)) {
			ai = m; af = 0.0; fi = 2 * m - xi; ff = -xf; rate = vec2(0.0, -1.0);
		} else {
			ai = 4 * m - xi; af = -xf; fi = -m; ff = 0.0; rate = vec2(-1.0, 0.0);
		}
	} else if (under > 0.0) {
		if (under <= float(2 * m)) {
			ai = -m; af = 0.0; fi = 2 * m + xi; ff = xf; rate = vec2(0.0, 1.0);
		} else {
			ai = -4 * m - xi; af = -xf; fi = -m; ff = 0.0; rate = vec2(-1.0, 0.0);
		}
	} else {
		ai = xi; af = xf; fi = m; ff = 0.0; rate = vec2(1.0, 0.0);
	}
}

// Cube point (int part pi, float part pf) of face0 grid point g,
// folded past the face edges; ex, ey: cube units per grid unit.
void fold_grid(ivec2 gi, vec2 gf, int m, int face, out ivec3 pi, out vec3 pf, out vec3 ex, out vec3 ey) {
	int ui; int fi; int vi;
	float uf; float ff; float vf;
	vec2 ru; vec2 rv;
	walk(2 * gi.x - m, 2.0 * gf.x, m, ui, uf, fi, ff, ru);
	int yi = 2 * gi.y - m;
	float yf = 2.0 * gf.y;
	if (ru.x == 1.0) {
		walk(yi, yf, m, vi, vf, fi, ff, rv);
	} else {
		// past a corner: the rest folds into the normal axis
		float over = float(yi - m) + yf;
		float under = float(-m - yi) - yf;
		vi = yi; vf = yf; rv = vec2(1.0, 0.0);
		if (over > 0.0) {
			vi = m; vf = 0.0; fi -= yi - m; ff -= yf; rv = vec2(0.0, -1.0);
		} else if (under > 0.0) {
			vi = -m; vf = 0.0; fi -= -m - yi; ff += yf; rv = vec2(0.0, -1.0);
		}
	}
	ivec3 F = FA_F[face];
	ivec3 U = FA_U[face];
	ivec3 V = FA_V[face];
	pi = F * fi + U * ui + V * vi;
	pf = vec3(F) * ff + vec3(U) * uf + vec3(V) * vf;
	ex = 2.0 * (vec3(U) * ru.x + vec3(F) * ru.y);
	ey = 2.0 * (vec3(V) * rv.x + vec3(F) * rv.y);
}

// The lowest-index face holding the cube point (canonical_sample)
int canon_face(ivec3 pi, vec3 pf, int m, int fallback) {
	for (int k = 0; k < 6; k++) {
		if (idot(FA_F[k], pi) == m && dot(vec3(FA_F[k]), pf) == 0.0) {
			return k;
		}
	}
	return fallback;
}

int page_of(int level, int face, ivec2 t) {
	vec4 e = texelFetch(page_table, ivec2(face * page_window + t.x % page_window, level * page_window + t.y % page_window), 0);
	return (e.r >= 0.0 && int(e.g) == t.x && int(e.b) == t.y) ? int(e.r) : -1;
}

// The finest resident page at or above level holding face point
// g (int gi + float gf): its layer, the point in it, levels up.
int find_page(int level, int face, ivec2 gi, vec2 gf, out vec2 ij, out int up) {
	ij = vec2(0.0);
	up = level;
	int found = 0;
	bool done = false;
	for (int d = 0; d <= level; d++) {
		if (!done) {
			int l = level - d;
			int n = 1 << l;
			ivec2 t = clamp(gi / intervals, ivec2(0), ivec2(n - 1));
			int layer = page_of(l, face, t);
			if (layer >= 0) {
				ij = vec2(gi - t * intervals) + gf;
				up = d;
				found = layer;
				done = true;
			} else {
				ivec2 h = gi / 2;
				gf = (vec2(gi - h * 2) + gf) * 0.5;
				gi = h;
			}
		}
	}
	return found;
}

float height(int level, int face, ivec2 gi, vec2 gf) {
	vec2 ij;
	int up;
	int layer = find_page(level, face, gi, gf, ij, up);
	return bilinear(layer, clamp(ij, vec2(-float(border)), vec2(float(intervals + border - 1))));
}

int floor_div(int a, int d) {
	return a >= 0 ? a / d : -((-a + d - 1) / d);
}

// ring r's texture coordinate at absolute grid step ki + kf,
// in units of `per` steps per texel and a toroid of n texels
vec3 ring_uv(int r, ivec2 ki, vec2 kf, int per, int n) {
	ivec2 qi = ivec2(floor_div(ki.x, per), floor_div(ki.y, per));
	vec2 qf = (vec2(ki - qi * per) + kf) / float(per);
	ivec2 w = ((qi % n) + n) % n;
	return vec3((vec2(w) + qf + 0.5) / float(n), float(r));
}

float ring_h(int r, ivec2 ki, vec2 kf) {
	return textureLod(ring_heights, ring_uv(r, ki, kf, 1, ring_size), 0.0).r;
}

// face0 grid point -> canonical face, its lattice point (int +
// float), and the folded cube point's step axes
int canonical(ivec2 gi_u, vec2 gf_u, int m, int face0, out ivec2 cg, out vec2 cgf,
		out ivec3 pi, out vec3 pf, out vec3 ex, out vec3 ey) {
	fold_grid(gi_u, gf_u, m, face0, pi, pf, ex, ey);
	int cf = canon_face(pi, pf, m, face0);
	cg = ivec2((idot(FA_U[cf], pi) + m) / 2, (idot(FA_V[cf], pi) + m) / 2);
	cgf = vec2(dot(vec3(FA_U[cf]), pf), dot(vec3(FA_V[cf]), pf)) * 0.5;
	return cf;
}

void vertex() {
	int nq = clip_quads;
	// a quad that may lie in the finer ring's hole carries its
	// own corners, UV = the quad: collapsed whole when it does
	bool in_hole = UV.x >= 0.0 && UV.x >= u_hole.x && UV.x + 1.0 <= u_hole.z && UV.y >= u_hole.y && UV.y + 1.0 <= u_hole.w;
	if (in_hole) {
		VERTEX = vec3(0.0);
		NORMAL = vec3(0.0, 1.0, 0.0);
		v_g = vec2(0.0);
	} else {
		ivec2 k = ivec2(VERTEX.xz);
		bool skirt = VERTEX.y < -0.5;
		int level = int(u_ring.z);
		int face0 = int(u_ring.w);
		int s = int(u_ccos.w);
		int m = int(u_cc.w);
		// odd grid points slide onto even ones over the outer band,
		// reaching them on the edge the coarser ring meets
		float half_n = float(nq) * 0.5;
		float d = max(abs(float(k.x) - half_n), abs(float(k.y) - half_n));
		float morph = clamp((d - 0.7 * half_n) / (0.25 * half_n), 0.0, 1.0);
		// profiler sections: 0 fold, 1 heights, 2 position
		uint t_fold = profile_clock(0);
		vec2 km = vec2(k) - mod(vec2(k), 2.0) * morph;
		vec2 g_rel = km * float(s);
		ivec2 gi_u = ivec2(u_ring.xy) + ivec2(floor(g_rel));
		vec2 gf_u = fract(g_rel);
		ivec3 pi;
		vec3 pf;
		vec3 ex;
		vec3 ey;
		fold_grid(gi_u, gf_u, m, face0, pi, pf, ex, ey);
		profile_add(0, t_fold);
		uint t_h = profile_clock(1);
		// this ring's height, morphing toward the next ring's
		int r = int(u_debug.x);
		ivec2 k0 = ivec2(u_ring.xy) / s;
		float h = ring_h(r, k0 + ivec2(floor(km)), fract(km));
		vec4 rp = ring_geo[min(r + 1, 23)];
		if (morph > 0.0 && rp.z > 0.0) {
			// the point in the coarser ring's grid steps
			int d = (1 << (level - int(rp.w))) * int(rp.z);
			ivec2 g = ivec2(u_ring.xy) + ivec2(floor(g_rel));
			ivec2 ci = ivec2(floor_div(g.x, d), floor_div(g.y, d));
			vec2 cfr = (vec2(g - ci * d) + gf_u) / float(d);
			h = mix(h, ring_h(r + 1, ci, cfr), morph);
		}
		profile_add(1, t_h);
		uint t_pos = profile_clock(2);
		// Offset from the ring centre in difference form, per axis
		// tan(a) - tan(b) = sin(a - b) / (cos a cos b): nothing
		// planet-sized is subtracted (see regolith/patch.hpp).
		float mf = float(m);
		vec3 dp = vec3(pi - ivec3(u_cc.xyz)) + pf;
		vec3 a = (vec3(pi) + pf) / mf;
		vec3 ca = cos(QP * a);
		mat3 to_push = transpose(u_to_body);
		vec3 dq = to_push * (sin(QP * dp / mf) / (ca * u_ccos.xyz));
		vec3 q = u_qc.xyz + dq;
		float n0 = u_qc.w;
		float n2 = length(q);
		float kq = (2.0 * dot(u_qc.xyz, dq) + dot(dq, dq)) / (n0 * n2 * (n0 + n2));
		vec3 dd = dq / n2 - u_qc.xyz * kq;
		vec3 dir = q / n2;
		vec3 p = u_center.xyz + dd * u_center.w + dir * (h - u_debug.z);
		// tangents per grid unit: d(tan)/dP = QP/m / cos^2
		vec3 dt = (QP / mf) / (ca * ca);
		vec3 dqx = to_push * (ex * dt);
		vec3 dqy = to_push * (ey * dt);
		float rad = (u_center.w + h - u_debug.z) / n2;
		v_ts = (dqx - dir * dot(dir, dqx)) * rad;
		v_tt = (dqy - dir * dot(dir, dqy)) * rad;
		if (skirt) {
			p -= dir * u_debug.w;
		}
		v_up = dir;
		v_g = g_rel;
		v_rho = h;
		v_local = (g_rel - vec2(half_n * float(s))) * (face_edge_m / mf);
		v_morph = morph;
		VERTEX = p;
		NORMAL = dir;
		profile_add(2, t_pos);
	}
}

// two lookups at incommensurate scales: no visible period
float tnoise(vec2 p) {
	float n = texture(noise_texture, p).r + texture(noise_texture, p * 1.6 + vec2(0.37)).r;
	return clamp((n - 1.0) * 0.7 + 0.5, 0.0, 1.0);
}

vec2 hex_hash(vec2 p) {
	p = mod(p, 256.0);
	return fract(sin(vec2(dot(p, vec2(127.1, 311.7)), dot(p, vec2(269.5, 183.3)))) * 43758.5453);
}

// mors's stochastic hex tiling (Heitz and Neyret 2018), with the
// large part of the coordinate supplied exactly per ring in dom.
void hex_sample(vec2 local_uv, vec4 dom, float layer, vec4 dd, out vec4 alb, out vec4 nrm) {
	const mat2 skew = mat2(vec2(1.0, 0.0), vec2(-0.57735027, 1.15470054));
	vec2 d_uv = dom.xy + local_uv;
	vec2 p = dom.zw + skew * (local_uv * 0.7);
	vec2 i = floor(p);
	vec3 t = vec3(fract(p), 0.0);
	t.z = 1.0 - t.x - t.y;
	vec3 w;
	vec2 v1, v2, v3;
	if (t.z > 0.0) {
		w = t.zyx;
		v1 = i;
		v2 = i + vec2(0.0, 1.0);
		v3 = i + vec2(1.0, 0.0);
	} else {
		w = vec3(-t.z, 1.0 - t.y, 1.0 - t.x);
		v1 = i + vec2(1.0, 1.0);
		v2 = i + vec2(1.0, 0.0);
		v3 = i + vec2(0.0, 1.0);
	}
	w = w * w * w;
	w /= w.x + w.y + w.z;
	// Far away the mips average a scan to near uniform and the
	// blend hides nothing: slide to the dominant cell alone (16
	// to 64 texels per pixel), skipping samples left at zero.
	float lod = log2(max(length(dd.xy), length(dd.zw)) * 2048.0);
	float k = smoothstep(4.0, 6.0, lod);
	if (k > 0.0) {
		vec3 one = w.x >= w.y && w.x >= w.z ? vec3(1.0, 0.0, 0.0) : (w.y >= w.z ? vec3(0.0, 1.0, 0.0) : vec3(0.0, 0.0, 1.0));
		w = mix(w, one, k);
	}
	alb = vec4(0.0);
	nrm = vec4(0.0);
	if (w.x > 0.0) {
		vec3 u = vec3(d_uv + hex_hash(v1), layer);
		alb += w.x * textureGrad(albedo_array, u, dd.xy, dd.zw);
		nrm += w.x * textureGrad(normal_array, u, dd.xy, dd.zw);
	}
	if (w.y > 0.0) {
		vec3 u = vec3(d_uv + hex_hash(v2), layer);
		alb += w.y * textureGrad(albedo_array, u, dd.xy, dd.zw);
		nrm += w.y * textureGrad(normal_array, u, dd.xy, dd.zw);
	}
	if (w.z > 0.0) {
		vec3 u = vec3(d_uv + hex_hash(v3), layer);
		alb += w.z * textureGrad(albedo_array, u, dd.xy, dd.zw);
		nrm += w.z * textureGrad(normal_array, u, dd.xy, dd.zw);
	}
}

float horizon_dir(vec3 hp, int k) {
	vec4 v = k < 4 ? textureLod(ring_horizon_a, hp, 0.0) : textureLod(ring_horizon_b, hp, 0.0);
	int c = k % 4;
	float b = c == 0 ? v.r : (c == 1 ? v.g : (c == 2 ? v.b : v.a));
	return -0.25 + b * 1.25; // HORIZON_MIN, HORIZON_SPAN
}

// Terrain shadow: the sun against the horizon in its own grid
// direction, soft over the solar disc; then the clouds.
float sun_visibility(vec3 up, vec3 ts, vec3 tt, vec3 hp, vec3 world) {
	vec3 s = normalize(transpose(u_to_body) * sun_body);
	float s_e = dot(s, up);
	vec3 sh = s - up * s_e;
	vec3 ti = normalize(ts), tj = normalize(tt);
	float aa = dot(ti, ti), ab = dot(ti, tj), bb = dot(tj, tj);
	float si = dot(sh, ti), sj = dot(sh, tj);
	float det = aa * bb - ab * ab;
	float a = (si * bb - sj * ab) / det;
	float b = (sj * aa - si * ab) / det;
	float f = mod(atan(b, a) / 0.78539816 + 8.0, 8.0);
	int k0 = int(floor(f)) % 8;
	int k1 = (k0 + 1) % 8;
	float hz = mix(horizon_dir(hp, k0), horizon_dir(hp, k1), fract(f));
	float vis = smoothstep(-0.006, 0.006, asin(clamp(s_e, -1.0, 1.0)) - hz);
	if (has_cloud_shadow) {
		vec3 dw = world - cloud_origin;
		vec2 uv = vec2(dot(dw, cloud_e), dot(dw, cloud_s)) / cloud_size + 0.5;
		if (all(greaterThan(uv, vec2(0.0))) && all(lessThan(uv, vec2(1.0)))) {
			vis *= texture(cloud_shadow, uv).r;
		}
	}
	return vis;
}

vec4 domain(int id) {
	return id == 1 ? u_dom1 : (id == 2 ? u_dom2 : (id == 3 ? u_dom3 : (id == 4 ? u_dom4 : u_dom0)));
}

float domain_scale(int id) {
	return id == 1 ? 0.0225 : (id == 2 ? 0.25 : (id == 3 ? 0.3335 : (id == 4 ? 0.04 : 0.5)));
}

void fragment() {
	// profiler sections: 0 page lookup, 1 heights normal, 2 sun,
	// 3 geology and noise, 4 layers, 5 macro and outputs, 6 orbit
	uint t_page = profile_clock(0);
	// this pixel in its ring's textures: absolute grid step
	int r = int(u_debug.x);
	int s = int(u_ccos.w);
	vec2 kv = v_g / float(s);
	ivec2 ki = ivec2(u_ring.xy) / s + ivec2(floor(kv));
	vec2 kf = fract(kv);
	vec3 quv = ring_uv(r, ki, kf, 4, ring_qsize);
	profile_add(0, t_page);
	uint t_hn = profile_clock(1);
	float hx = 0.0;
	float hy = 0.0;
	if ((prof_skip & 1) == 0) {
		// one grid step is s data units; v_ts is per data unit
		float per = 1.0 / float(s);
		hx = (ring_h(r, ki, kf + vec2(1.0, 0.0)) - ring_h(r, ki, kf - vec2(1.0, 0.0))) * per;
		hy = (ring_h(r, ki, kf + vec2(0.0, 1.0)) - ring_h(r, ki, kf - vec2(0.0, 1.0))) * per;
	}
	profile_add(1, t_hn);
	vec3 up = normalize(v_up);
	vec3 n = normalize(cross(v_ts + up * (0.5 * hx), v_tt + up * (0.5 * hy)));
	mat3 to_world = mat3(MODEL_MATRIX);
	vec3 n_view = normalize((VIEW_MATRIX * vec4(to_world * n, 0.0)).xyz);
	// depth, normal-roughness and shadow passes: the ground's shape
	// is all SSAO and shadows read, so no texture work there
	if (IN_DEPTH_PREPASS && (prof_skip & 2048) == 0) {
		NORMAL = n_view;
		ROUGHNESS = roughness_floor;
	} else {
	// camera-relative: world origins differ in double builds
	uint t_sun = profile_clock(2);
	v_sun_vis = (prof_skip & 2) == 0 ? sun_visibility(up, v_ts, v_tt, quv, mat3(INV_VIEW_MATRIX) * VERTEX) : 1.0;
	profile_add(2, t_sun);
	v_up_view = normalize(mat3(VIEW_MATRIX) * (mat3(MODEL_MATRIX) * up));
	v_spec = 0.5; // Godot's default
	// height of the eye over this ground: -VERTEX is the view
	// space vector to the camera, small at any world position
	float eye_h = dot(-VERTEX, v_up_view);
	vec3 axis_x = normalize(v_ts);
	vec3 axis_y = normalize(v_tt);

	if (debug_mode != 0 || !has_textures) {
		vec3 albedo = mix(dust_colour, rock_colour, smoothstep(0.08, 0.45, 1.0 - dot(n, up)));
		if (debug_mode == 1) {
			albedo = 0.5 + 0.5 * cos(6.2831853 * (u_debug.x / 7.0 + vec3(0.0, 0.33, 0.67)));
		} else if (debug_mode == 2) {
			albedo = vec3(v_morph, 1.0 - v_morph, 0.15);
		} else if (debug_mode == 3) {
			albedo = n * 0.5 + 0.5;
		} else if (debug_mode == 4) {
			// sun visibility, tinted by ring
			albedo = vec3(v_sun_vis) * (0.75 + 0.25 * cos(6.2831853 * (u_debug.x / 7.0 + vec3(0.0, 0.33, 0.67))));
		}
		ALBEDO = albedo;
		ROUGHNESS = 0.93;
		NORMAL = n_view;
	} else {
		// geology page: R gravel province, G bedrock patch,
		// B ice, A Viking darkness (planet_streamer.gd)
		uint t_geo = profile_clock(3);
		vec4 color_map = textureLod(ring_geology, quv, 0.0);
		vec2 wm = u_nmod.xy + v_local; // noise metres, 20 km period
		vec4 base_dd = vec4(dFdxCoarse(v_local), dFdyCoarse(v_local));

		float slope = clamp(1.0 - dot(n, up), 0.0, 1.0);
		float n_prov = color_map.r;
		float n_patch = color_map.g;
		bool noise_on = (prof_skip & 4) == 0;
		float n_mot = noise_on ? texture(noise_texture, wm * 0.0004 + vec2(0.29)).r : 0.5;
		float n_f1 = noise_on ? tnoise(wm * 0.03 + vec2(0.13)) : 0.5;
		float n_f2 = noise_on ? tnoise(wm * 0.0025 + vec2(0.77)) : 0.5;
		float n_f3 = noise_on ? tnoise(wm * 0.004 + vec2(0.52)) : 0.5;

		float w_ice = color_map.b;
		float w_rock = smoothstep(slope_rock_start, slope_rock_start + 0.12, slope);
		float loc = smoothstep(0.35, 0.75, noise_on ? tnoise(wm * 0.001 + vec2(0.23)) : 0.5) * (1.0 - color_map.a);
		float bed_l = smoothstep(0.60, 0.70, n_f2 * 0.75 + n_f1 * 0.25) * loc;
		float w_bed = max(smoothstep(1.0 - bedrock_amount * 0.5, 1.0 - bedrock_amount * 0.5 + 0.10,
			n_patch * 0.72 + n_mot * 0.28), bed_l) * clamp(1.0 - slope * 4.0, 0.0, 1.0) * (1.0 - w_rock);
		float grv_l = smoothstep(0.60, 0.70, n_f3 * 0.75 + n_f1 * 0.25) * loc;
		float w_grv = max(smoothstep(1.0 - gravel_amount * 0.6, 1.0 - gravel_amount * 0.6 + 0.14,
			n_prov * 0.68 + n_mot * 0.32), grv_l) * (1.0 - w_rock) * (1.0 - w_bed);
		float w_sand = max(1.0 - w_rock - w_bed - w_grv, 0.0);
		float inv_ice = 1.0 - w_ice;
		float w_dark = color_map.a;
		float wgt[6] = {w_sand * inv_ice * (1.0 - w_dark), w_rock * inv_ice,
			w_grv * inv_ice, w_bed * inv_ice, w_ice, w_sand * inv_ice * w_dark};
		// from flight heights gravel and bedrock hand their weight
		// to the sand layers
		float view_h = clamp((eye_h - 3000.0) / 12000.0, 0.0, 1.0);
		wgt[0] += (wgt[2] + wgt[3]) * view_h * (1.0 - w_dark);
		wgt[5] += (wgt[2] + wgt[3]) * view_h * w_dark;
		wgt[2] *= 1.0 - view_h;
		wgt[3] *= 1.0 - view_h;

		vec4 albedo_height = vec4(0.);
		vec4 normal_rough = vec4(0.);
		float normal_map_depth = 0.;
		float ao_strength = 0.;
		float total_weight = 0.;
		float sharp = fma(10.0, blend_sharpness, 2.0);
		vec2 wrp = noise_on ? (vec2(texture(noise_texture, wm * 0.0007).r,
			texture(noise_texture, wm * 0.0007 + vec2(0.41)).r) - 0.5) * 8.0 : vec2(0.0);
		profile_add(3, t_geo);
		uint t_layers = profile_clock(4);
		for (int id = 0; id < 6; id++) {
			float w = wgt[id];
			if (w < 0.004) {
				continue;
			}
			int dom = id == 5 ? 0 : id;
			float s = domain_scale(dom);
			vec4 i_dd = base_dd * s;
			vec4 alb;
			vec4 nrm;
			if ((prof_skip & (8 << id)) != 0) {
				// profiling: this layer's sampling off, a flat mid tone
				alb = vec4(0.5, 0.5, 0.5, 0.5);
				nrm = vec4(0.5, 0.5, 1.0, 0.8);
			} else if (id == 1) {
				// cliff: planar and projected domains by steepness
				float proj_f = smoothstep(0.12, 0.30, slope);
				alb = vec4(0.);
				nrm = vec4(0.);
				if (proj_f < 0.98) {
					vec4 a1;
					vec4 n1;
					hex_sample((v_local + wrp) * s, u_dom1, 1.0, i_dd, a1, n1);
					alb += a1 * (1.0 - proj_f);
					nrm += n1 * (1.0 - proj_f);
				}
				if (proj_f > 0.02) {
					bool along_u = abs(dot(n, axis_x)) > abs(dot(n, axis_y));
					vec2 puv = along_u ? vec2(v_local.y, -(v_rho - u_debug.z)) : vec2(v_local.x, -(v_rho - u_debug.z));
					vec4 a1;
					vec4 n1;
					hex_sample((puv + wrp) * s, along_u ? u_cliff_v : u_cliff_u, 1.0, i_dd, a1, n1);
					alb += a1 * proj_f;
					nrm += n1 * proj_f;
				}
			} else {
				vec2 w_amt = wrp * ((id == 0 || id == 5) ? 0.3 : 1.0);
				hex_sample((v_local + w_amt) * s, domain(dom), float(id), i_dd, alb, nrm);
			}
			alb.rgb *= layer_color[id].rgb;
			nrm.a = clamp(nrm.a + layer_roughness_mod[id], 0., 1.);
			nrm.xyz = fma(nrm.xzy, vec3(2.0), vec3(-1.0));
			float id_weight = exp2(sharp * log2(clamp(w * (0.35 + 0.65 * alb.a), 1e-4, 1.0)));
			albedo_height += alb * id_weight;
			normal_rough += nrm * id_weight;
			normal_map_depth += layer_normal_depth[id] * id_weight;
			ao_strength += layer_ao_strength[id] * id_weight;
			total_weight += id_weight;
		}
		float weight_inv = 1.0 / max(total_weight, 1e-6);
		albedo_height *= weight_inv;
		normal_rough *= weight_inv;
		normal_map_depth *= weight_inv;
		ao_strength *= weight_inv;
		profile_add(4, t_layers);
		uint t_macro = profile_clock(5);

		vec3 macrov = vec3(1.);
		if (enable_macro_variation && (prof_skip & 512) == 0) {
			float noise1 = tnoise(wm * 0.00125);
			float noise2 = tnoise(wm * 0.007 + vec2(0.61));
			macrov = mix(macro_variation1, vec3(1.), noise1);
			macrov *= mix(macro_variation2, vec3(1.), noise2);
			macrov = mix(vec3(1.0), macrov, clamp(dot(n, up) + macro_variation_slope, 0., 1.));
		}
		float roughness = clamp(max(normal_rough.a, roughness_floor), 0., 1.);
		ALBEDO = albedo_height.rgb * macrov;
		if (frost_amt > 0.001) {
			float level_f = clamp(dot(n, up) * 1.4 - 0.4, 0., 1.);
			float fr = frost_amt * (0.35 + 0.65 * level_f);
			ALBEDO = mix(ALBEDO, vec3(0.86, 0.88, 0.90), fr);
			roughness = mix(roughness, 0.75, fr);
		}
		ROUGHNESS = roughness;
		SPECULAR = specular_amount * (1.0 - view_h);
		v_spec = SPECULAR;
		// texture u runs along grid x
		vec3 t_model = normalize(axis_x - n * dot(n, axis_x));
		vec3 b_model = normalize(cross(n, t_model));
		NORMAL = n_view;
		TANGENT = normalize((VIEW_MATRIX * vec4(to_world * t_model, 0.0)).xyz);
		BINORMAL = normalize((VIEW_MATRIX * vec4(to_world * b_model, 0.0)).xyz);
		NORMAL_MAP = fma(normalize(normal_rough.xzy), vec3(0.5), vec3(0.5));
		NORMAL_MAP_DEPTH = normal_map_depth;
		float ao = (1. - (albedo_height.a * log(2.1 - ao_strength))) * (1. - normal_rough.y);
		AO = clamp(1. - ao * ao_strength, albedo_height.a, 1.0);
		AO_LIGHT_AFFECT = (1.0 - albedo_height.a) * clamp(normal_rough.y, 0., 1.);
		profile_add(5, t_macro);
	}

	// From far away (30 to 100 km up): mors's isomorphic map
	// (mars_globe_iso.gdshader), the DEM's own relief at
	// shade_vex over the mosaic, unshaded, with its air.
	uint t_orbit = profile_clock(6);
	float orbit = smoothstep(30000.0, 100000.0, eye_h);
	if (has_iso_map && debug_mode == 0 && orbit > 0.0) {
		vec3 d = normalize(u_to_body * up);
		float lat = asin(clamp(d.z, -1.0, 1.0));
		float lon = atan(d.y, d.x);
		// two u's, so the antimeridian seam picks the smooth one
		vec2 uv_a = vec2(lon / 6.2831853 + 0.5, 0.5 - lat / 3.1415927);
		vec2 uv_b = vec2(fract(uv_a.x + 0.5) - 0.5, uv_a.y);
		bool use_a = fwidth(uv_a.x) <= fwidth(uv_b.x);
		vec2 uv = use_a ? uv_a : uv_b;
		vec2 gx = vec2(use_a ? dFdx(uv_a.x) : dFdx(uv_b.x), dFdx(uv.y));
		vec2 gy = vec2(use_a ? dFdy(uv_a.x) : dFdy(uv_b.x), dFdy(uv.y));
		vec3 map = textureGrad(iso_map, uv, gx, gy).rgb;
		// the DEM gradient, exaggerated shade_vex times
		vec3 tilt = n - up * dot(n, up);
		vec3 pert = normalize(u_to_body * normalize(up + tilt * shade_vex));
		vec3 sun = normalize(sun_body);
		// wrapped Lambert: true hillshade blacks every away slope
		float lam = clamp((dot(pert, sun) + 0.25) / 1.25, 0.0, 1.0);
		vec3 col = map * (iso_night + (1.0 - iso_night) * lam);
		vec3 n_curve = v_up_view;
		float fres = pow(1.0 - clamp(dot(n_curve, VIEW), 0.0, 1.0), 2.2);
		float day = clamp(dot(d, sun) * 0.5 + 0.5, 0.0, 1.0);
		col *= mix(1.0, 0.45, fres * atmosphere);
		col += vec3(0.80, 0.44, 0.24) * pow(fres, 1.7) * (0.25 + 0.75 * day) * 0.9 * atmosphere;
		ALBEDO *= 1.0 - orbit;
		EMISSION = col * orbit;
	}
	profile_add(6, t_orbit);
	} // the depth-pass branch above
}

// Godot's Burley diffuse and GGX specular. The sun splits by the
// dust: the direct beam exp(-tau * airmass), which terrain and
// object shadows block, and skylight the dust scatters down,
// which no shadow removes (lit by the sky dome over the ground).
void light() {
	float ndl = clamp(dot(NORMAL, LIGHT), 0.0, 1.0);
	float direct = 1.0;
	if (LIGHT_IS_DIRECTIONAL) {
		float s_e = dot(LIGHT, v_up_view);
		direct = exp(-dust_tau / max(s_e + 0.05, 0.05));
		// most dust scatter is forward: about 3/4 reaches the ground
		float sky = (1.0 - direct) * 0.75 * clamp(s_e, 0.0, 1.0);
		float dome = 0.5 + 0.5 * dot(NORMAL, v_up_view);
		DIFFUSE_LIGHT += LIGHT_COLOR * sky * dome / PI;
	}
	if (ndl > 0.0) {
		// Godot's own BRDF (scene_forward_lights_inc.glsl)
		float vis = LIGHT_IS_DIRECTIONAL ? v_sun_vis * direct : 1.0;
		vec3 hv = normalize(LIGHT + VIEW);
		float ndv = max(dot(NORMAL, VIEW), 1e-4);
		float ldh = clamp(dot(LIGHT, hv), 0.0, 1.0);
		float ndh = clamp(dot(NORMAL, hv), 0.0, 1.0);
		float r = ROUGHNESS;
		float fd90m1 = 2.0 * ldh * ldh * r - 0.5;
		float fl = 1.0 + fd90m1 * pow(1.0 - ndl, 5.0);
		float fv = 1.0 + fd90m1 * pow(1.0 - ndv, 5.0);
		vec3 lit = LIGHT_COLOR * ATTENUATION * vis;
		DIFFUSE_LIGHT += lit * fl * fv * ndl / PI;
		// dielectric f0 from the material's specular (fragment);
		// SPECULAR_AMOUNT here is the light's own multiplier
		float a = r * r;
		float a2 = a * a;
		float dd = 1.0 + (a2 - 1.0) * ndh * ndh;
		float d = a2 / (PI * dd * dd);
		float g = 0.5 / (ndl * sqrt(ndv * ndv * (1.0 - a2) + a2) + ndv * sqrt(ndl * ndl * (1.0 - a2) + a2));
		float f0 = 0.16 * v_spec * v_spec;
		float f90 = clamp(f0 * 50.0, 0.0, 1.0);
		float f = f0 + (f90 - f0) * pow(1.0 - ldh, 5.0);
		SPECULAR_LIGHT += lit * ndl * d * f * g * SPECULAR_AMOUNT;
	}
}

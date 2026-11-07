shader_type spatial;
render_mode cull_disabled, diffuse_burley, specular_schlick_ggx, motion_vectors_rigid, ambient_light_disabled;

// Regolith's terrain: clipmap rings of a cube-sphere (RegolithBody::
// draw_clipmap), lit by the sun and the air per point. What the
// ground looks like is the surface, inlined at the marker below.

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
uniform int intervals = 208;
uniform int border = 2;
uniform int geology_side = 55;
uniform float face_edge_m = 5324000.0;
// 1 rings, 2 morph, 3 normals, 4 terrain shadow
uniform int debug_mode = 0;
// profiling: each set bit skips one section, neutral output;
// 1 heights normal, 2 sun, 2048 full work in depth passes
uniform int prof_skip = 0;
uniform vec3 u_eye; // main camera, push-frame metres
// Dust optical depth, for the sun without RegolithAtmosphere:
// 0 by default, so a body with no atmosphere node is airless
uniform float dust_tau = 0.0;
uniform mat3 u_to_body = mat3(1.0); // node space -> body-fixed
uniform vec3 sun_body = vec3(0.0, 0.0, 1.0);
// RegolithAtmosphere's air, per point: the sun through it (its
// transmittance table) and the sky's light (irradiance table)
uniform bool air_on = false;
uniform sampler2D air_t : filter_linear, repeat_disable;
uniform sampler2D air_e : filter_linear, repeat_disable;
uniform vec4 air_shape = vec4(0.0); // table base, top, datum, r_ref
uniform float sun_top = 0.0; // the sun's energy above the air
// the airglow's light on level ground, at the eye's adaptation
uniform vec3 night_glow = vec3(0.0);
varying vec3 v_sun_t; // the sun's colour through the air here

varying vec2 v_g; // data units from the ring's min corner
varying vec2 v_local; // face metres from the ring centre
varying float v_rho;
varying vec3 v_ts; // position per data unit along grid x
varying vec3 v_tt; // and along grid y
varying vec3 v_up;
varying float v_morph;
varying float v_sun_vis; // terrain shadow, for the sun only
varying vec3 v_up_view; // local vertical, view space
varying float v_spec; // the material's SPECULAR, for light()

// What a surface is given, per pixel.
struct TerrainPoint {
	vec3 normal; // geometric normal, model space
	vec3 up; // local vertical, model space
	float slope; // 1 - dot(normal, up): 0 level
	float height; // metres over the body's reference radius
	vec3 body_dir; // unit, body-fixed: lat asin(z), lon atan(y, x)
	vec2 local; // face metres from the ring centre: precise near
	vec2 noise_m; // metres, wrapping every 20 km: noise coordinates
	vec4 dd; // d(local)/dx, d(local)/dy: texture gradients
	vec4 geology; // RegolithBody's geology map here
	vec3 axis_x; // model-space tangent along the face grid's x
	vec3 axis_y; // and its y
	float eye_height; // the camera over this point, metres
	float ring; // clipmap ring, 0 nearest
};

// What a surface returns; it starts at the defaults below.
struct TerrainSurface {
	vec3 albedo;
	float roughness;
	float specular;
	vec3 normal; // model space; starts as the geometric normal
	vec3 normal_map; // tangent space along axis_x / axis_y
	float normal_map_depth; // 0: no normal map
	float ao;
	float ao_light_affect;
	vec3 emission;
};

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

float horizon_dir(vec3 hp, int k) {
	vec4 v = k < 4 ? textureLod(ring_horizon_a, hp, 0.0) : textureLod(ring_horizon_b, hp, 0.0);
	int c = k % 4;
	float b = c == 0 ? v.r : (c == 1 ? v.g : (c == 2 ? v.b : v.a));
	return -0.25 + b * 1.25; // HORIZON_MIN, HORIZON_SPAN
}

// Terrain shadow: the sun against the horizon in its own grid
// direction, soft over the solar disc.
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
	return smoothstep(-0.006, 0.006, asin(clamp(s_e, -1.0, 1.0)) - hz);
}

float air_sub(float u, float res) {
	return (u + 0.5 / res) * (res / (res + 1.0));
}

// regolith_air_common.glsli's transmittance_uv, 256 x 64
vec2 air_t_uv(float r, float mu) {
	float bot = air_shape.x, top = air_shape.y;
	float h2 = (top - bot) * (top + bot);
	float big_h = sqrt(max(h2, 0.0));
	float rho = sqrt(max((r - bot) * (r + bot), 0.0));
	float disc = (top - r) * (top + r) + r * r * mu * mu;
	float d = max(0.0, -r * mu + sqrt(max(disc, 0.0)));
	float d_min = top - r;
	float d_max = rho + big_h;
	float x_mu = clamp((d - d_min) / max(d_max - d_min, 1e-3), 0.0, 1.0);
	return vec2(air_sub(x_mu, 256.0), air_sub(clamp(rho / big_h, 0.0, 1.0), 64.0));
}

// regolith_air_irradiance.glsl's table, 64 x 16
vec2 air_e_uv(float r, float mu) {
	vec2 uv = vec2(mu * 0.5 + 0.5, (r - air_shape.x) / (air_shape.y - air_shape.x));
	return clamp(vec2(air_sub(uv.x, 64.0), air_sub(uv.y, 16.0)), 0.0, 1.0);
}

// Precise texture domains for surfaces: per ring, a texture's
// large coordinate part (fract uv, hex lattice) for scales
// domain_scale(id), so tiling stays exact at any distance.
vec4 domain(int id) {
	return id == 1 ? u_dom1 : (id == 2 ? u_dom2 : (id == 3 ? u_dom3 : (id == 4 ? u_dom4 : u_dom0)));
}

float domain_scale(int id) {
	return id == 1 ? 0.0225 : (id == 2 ? 0.25 : (id == 3 ? 0.3335 : (id == 4 ? 0.04 : 0.5)));
}

// @surface

void fragment() {
	// profiler sections: 0 page lookup, 1 heights normal, 2 sun,
	// 3 the surface
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
	// is all SSAO and shadows read, so no surface work there
	if (IN_DEPTH_PREPASS && (prof_skip & 2048) == 0) {
		NORMAL = n_view;
		ROUGHNESS = 0.9;
	} else {
	uint t_sun = profile_clock(2);
	v_sun_vis = (prof_skip & 2) == 0 ? sun_visibility(up, v_ts, v_tt, quv, mat3(INV_VIEW_MATRIX) * VERTEX) : 1.0;
	profile_add(2, t_sun);
	v_up_view = normalize(mat3(VIEW_MATRIX) * (mat3(MODEL_MATRIX) * up));
	// height of the eye over this ground: -VERTEX is the view
	// space vector to the camera, small at any world position
	float eye_h = dot(-VERTEX, v_up_view);
	vec3 axis_x = normalize(v_ts);
	vec3 axis_y = normalize(v_tt);

	TerrainPoint tp;
	tp.normal = n;
	tp.up = up;
	tp.slope = clamp(1.0 - dot(n, up), 0.0, 1.0);
	tp.height = v_rho;
	tp.body_dir = normalize(u_to_body * up);
	tp.local = v_local;
	tp.noise_m = u_nmod.xy + v_local;
	tp.dd = vec4(dFdxCoarse(v_local), dFdyCoarse(v_local));
	tp.geology = textureLod(ring_geology, quv, 0.0);
	tp.axis_x = axis_x;
	tp.axis_y = axis_y;
	tp.eye_height = eye_h;
	tp.ring = u_debug.x;
	TerrainSurface sf;
	sf.albedo = vec3(0.5);
	sf.roughness = 0.9;
	sf.specular = 0.5;
	sf.normal = n;
	sf.normal_map = vec3(0.5, 0.5, 1.0);
	sf.normal_map_depth = 0.0;
	sf.ao = 1.0;
	sf.ao_light_affect = 0.0;
	sf.emission = vec3(0.0);
	uint t_surf = profile_clock(3);
	surface(tp, sf);
	profile_add(3, t_surf);

	if (debug_mode == 1) {
		sf.albedo = 0.5 + 0.5 * cos(6.2831853 * (u_debug.x / 7.0 + vec3(0.0, 0.33, 0.67)));
	} else if (debug_mode == 2) {
		sf.albedo = vec3(v_morph, 1.0 - v_morph, 0.15);
	} else if (debug_mode == 3) {
		sf.albedo = n * 0.5 + 0.5;
	} else if (debug_mode == 4) {
		// sun visibility, tinted by ring
		sf.albedo = vec3(v_sun_vis) * (0.75 + 0.25 * cos(6.2831853 * (u_debug.x / 7.0 + vec3(0.0, 0.33, 0.67))));
	}
	ALBEDO = sf.albedo;
	ROUGHNESS = sf.roughness;
	SPECULAR = sf.specular;
	v_spec = sf.specular;
	NORMAL = normalize((VIEW_MATRIX * vec4(to_world * normalize(sf.normal), 0.0)).xyz);
	// the normal map's u runs along grid x
	vec3 t_model = normalize(axis_x - sf.normal * dot(sf.normal, axis_x));
	vec3 b_model = normalize(cross(sf.normal, t_model));
	TANGENT = normalize((VIEW_MATRIX * vec4(to_world * t_model, 0.0)).xyz);
	BINORMAL = normalize((VIEW_MATRIX * vec4(to_world * b_model, 0.0)).xyz);
	NORMAL_MAP = sf.normal_map;
	NORMAL_MAP_DEPTH = sf.normal_map_depth;
	AO = sf.ao;
	AO_LIGHT_AFFECT = sf.ao_light_affect;
	EMISSION = sf.emission;
	if (air_on) {
		// this point's own sun and sky, wherever the camera is
		vec3 db = tp.body_dir;
		float rr = air_shape.w + v_rho;
		float mu = dot(db, normalize(sun_body));
		// behind the planet's own limb: night
		float dip = sqrt(max((rr - air_shape.z) * (rr + air_shape.z), 0.0)) / rr;
		float day_side = smoothstep(-0.004, 0.004, mu + dip);
		vec3 tr = textureLod(air_t, air_t_uv(rr, mu), 0.0).rgb * day_side;
		v_sun_t = tr;
		vec3 sky = textureLod(air_e, air_e_uv(rr, mu), 0.0).rgb;
		// the sky above, the lit ground around, by the facing
		float dome = 0.5 + 0.5 * dot(NORMAL, v_up_view);
		vec3 bounce = ALBEDO * (tr * max(mu, 0.0) + sky);
		EMISSION += ALBEDO * ((sky * dome + bounce * (1.0 - dome)) * sun_top + night_glow * dome);
	}
	} // the depth-pass branch above
}

// Godot's Burley diffuse and GGX specular. Without the physical
// air the sun splits by the dust: the direct beam, which shadows
// block, and skylight scattered down, which no shadow removes.
void light() {
	float ndl = clamp(dot(NORMAL, LIGHT), 0.0, 1.0);
	float direct = 1.0;
	// the physical air: the sun through it here, the sky as above
	bool own_sun = air_on && LIGHT_IS_DIRECTIONAL;
	if (LIGHT_IS_DIRECTIONAL && !own_sun && dust_tau > 0.0) {
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
		vec3 lit = own_sun ? sun_top * PI * v_sun_t * ATTENUATION * v_sun_vis : LIGHT_COLOR * ATTENUATION * vis;
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

shader_type spatial;
render_mode cull_disabled, diffuse_burley, specular_schlick_ggx;

// Per node, set by RegolithBody (see node_constants()).
instance uniform vec4 u_center; // render-frame centre, r_center
instance uniform vec4 u_axis_u; // face u axis, s_c
instance uniform vec4 u_axis_v; // face v axis, t_c
instance uniform vec4 u_qc; // cube point of the centre, |q|
instance uniform vec4 u_misc; // cos s_c, cos t_c, 1/m, morph start
instance uniform vec4 u_pages; // layer, parent layer, quadrant, morph end
instance uniform vec4 u_debug; // level, quads, rho_c, skirt depth

uniform sampler2DArray heights : filter_nearest, repeat_disable;
uniform int intervals = 208;
uniform int border = 2;
uniform int debug_mode = 0;
uniform vec3 dust_colour : source_color = vec3(0.62, 0.36, 0.22);
uniform vec3 rock_colour : source_color = vec3(0.30, 0.21, 0.16);

varying vec2 v_ij;
varying vec3 v_ts;
varying vec3 v_tt;
varying vec3 v_up;
varying float v_morph;

const float QP = 0.7853981633974483;

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

// Offset from the node centre in difference form: nothing
// planet-sized is ever subtracted (see regolith/patch.hpp).
vec3 offset_of(vec2 ij, float rho, out vec3 dir) {
	float n = float(intervals);
	float ds = (2.0 * ij.x - n) * u_misc.z;
	float dt = (2.0 * ij.y - n) * u_misc.z;
	float dx = sin(QP * ds) / (cos(QP * (u_axis_u.w + ds)) * u_misc.x);
	float dy = sin(QP * dt) / (cos(QP * (u_axis_v.w + dt)) * u_misc.y);
	vec3 dq = u_axis_u.xyz * dx + u_axis_v.xyz * dy;
	vec3 q = u_qc.xyz + dq;
	float n0 = u_qc.w;
	float n2 = length(q);
	float k = (2.0 * dot(u_qc.xyz, dq) + dot(dq, dq)) / (n0 * n2 * (n0 + n2));
	vec3 dd = dq / n2 - u_qc.xyz * k;
	dir = q / n2;
	return dd * u_center.w + dir * (rho - u_debug.z);
}

void vertex() {
	int layer = int(u_pages.x);
	int parent_layer = int(u_pages.y);
	int quadrant = int(u_pages.z);
	float step = float(intervals) / u_debug.y;
	vec2 g = VERTEX.xz;
	bool skirt = VERTEX.y < -0.5;
	vec3 dir;
	vec2 ij = g * step;
	vec3 p = u_center.xyz + offset_of(ij, texel(layer, ivec2(ij)), dir);
	// Morph by distance to the MAIN camera, also in shadow passes.
	vec3 eye = MAIN_CAM_INV_VIEW_MATRIX[3].xyz;
	float morph = clamp((distance(p, eye) - u_misc.w) / max(u_pages.w - u_misc.w, 1e-3), 0.0, 1.0);
	vec2 gm = g - mod(g, 2.0) * morph;
	vec2 ijm = gm * step;
	vec2 pij = (vec2(float(quadrant & 1), float(quadrant >> 1)) * float(intervals) + ijm) * 0.5;
	float h = mix(bilinear(layer, ijm), bilinear(parent_layer, pij), morph);
	p = u_center.xyz + offset_of(ijm, h, dir);
	vec3 spare;
	v_ts = offset_of(ijm + vec2(0.5, 0.0), h, spare) - offset_of(ijm - vec2(0.5, 0.0), h, spare);
	v_tt = offset_of(ijm + vec2(0.0, 0.5), h, spare) - offset_of(ijm - vec2(0.0, 0.5), h, spare);
	if (skirt) {
		p -= dir * u_debug.w;
	}
	v_up = dir;
	v_ij = ijm;
	v_morph = morph;
	VERTEX = p;
	NORMAL = dir;
}

void fragment() {
	int layer = int(u_pages.x);
	vec2 ij = v_ij;
	float hx = bilinear(layer, ij + vec2(1.0, 0.0)) - bilinear(layer, ij - vec2(1.0, 0.0));
	float hy = bilinear(layer, ij + vec2(0.0, 1.0)) - bilinear(layer, ij - vec2(0.0, 1.0));
	vec3 up = normalize(v_up);
	vec3 n = normalize(cross(v_ts + up * (0.5 * hx), v_tt + up * (0.5 * hy)));
	vec3 albedo = mix(dust_colour, rock_colour, smoothstep(0.08, 0.45, 1.0 - dot(n, up)));
	if (debug_mode == 1) {
		albedo = 0.5 + 0.5 * cos(6.2831853 * (u_debug.x / 7.0 + vec3(0.0, 0.33, 0.67)));
	} else if (debug_mode == 2) {
		albedo = vec3(v_morph, 1.0 - v_morph, 0.15);
	} else if (debug_mode == 3) {
		albedo = n * 0.5 + 0.5;
	}
	ALBEDO = albedo;
	ROUGHNESS = 0.93;
	NORMAL = normalize((VIEW_MATRIX * vec4(n, 0.0)).xyz);
}

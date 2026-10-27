shader_type spatial;
render_mode cull_disabled, diffuse_burley, specular_schlick_ggx;

// Geometry: CDLOD patches of a cube-sphere (see RegolithBody).
// Shading: mors's mars_ground.gdshader procedural geology,
// with world XZ metres replaced by face metres plus exact
// per-node offsets (RegolithBody::push_texture_domains).

// Per node, set by RegolithBody (see node_constants()).
instance uniform vec4 u_center; // centre, r_center
instance uniform vec4 u_axis_u; // face u axis, s_c
instance uniform vec4 u_axis_v; // face v axis, t_c
instance uniform vec4 u_qc; // cube point of the centre, |q|
instance uniform vec4 u_misc; // cos s_c, cos t_c, 1/m, morph start
instance uniform vec4 u_pages; // layer, parent layer, quadrant, morph end
instance uniform vec4 u_debug; // level, quads, rho_c, skirt depth
instance uniform vec4 u_dom0; // texture domains: fract uv, hex lattice
instance uniform vec4 u_dom1;
instance uniform vec4 u_dom2;
instance uniform vec4 u_dom3;
instance uniform vec4 u_dom4;
instance uniform vec4 u_cliff_u;
instance uniform vec4 u_cliff_v;
instance uniform vec4 u_nmod; // face metres mod 20 km

uniform sampler2DArray heights : filter_nearest, repeat_disable;
uniform sampler2DArray geology : filter_linear, repeat_disable;
uniform int intervals = 208;
uniform int border = 2;
uniform int geology_side = 55;
uniform float face_edge_m = 5324000.0;
uniform int debug_mode = 0;

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
uniform vec3 dust_colour : source_color = vec3(0.62, 0.36, 0.22);
uniform vec3 rock_colour : source_color = vec3(0.30, 0.21, 0.16);

varying vec2 v_ij;
varying vec2 v_local; // face metres from the node centre
varying float v_rho;
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
	vec3 p_world = (MODEL_MATRIX * vec4(p, 1.0)).xyz;
	float morph = clamp((distance(p_world, eye) - u_misc.w) / max(u_pages.w - u_misc.w, 1e-3), 0.0, 1.0);
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
	v_rho = h;
	v_local = (2.0 * ijm - float(intervals)) * u_misc.z * 0.5 * face_edge_m;
	v_morph = morph;
	VERTEX = p;
	NORMAL = dir;
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
// large part of the coordinate supplied exactly per node in dom.
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
	vec2 u1 = d_uv + hex_hash(v1);
	vec2 u2 = d_uv + hex_hash(v2);
	vec2 u3 = d_uv + hex_hash(v3);
	alb = w.x * textureGrad(albedo_array, vec3(u1, layer), dd.xy, dd.zw)
		+ w.y * textureGrad(albedo_array, vec3(u2, layer), dd.xy, dd.zw)
		+ w.z * textureGrad(albedo_array, vec3(u3, layer), dd.xy, dd.zw);
	nrm = w.x * textureGrad(normal_array, vec3(u1, layer), dd.xy, dd.zw)
		+ w.y * textureGrad(normal_array, vec3(u2, layer), dd.xy, dd.zw)
		+ w.z * textureGrad(normal_array, vec3(u3, layer), dd.xy, dd.zw);
}

vec4 domain(int id) {
	return id == 1 ? u_dom1 : (id == 2 ? u_dom2 : (id == 3 ? u_dom3 : (id == 4 ? u_dom4 : u_dom0)));
}

float domain_scale(int id) {
	return id == 1 ? 0.0225 : (id == 2 ? 0.25 : (id == 3 ? 0.3335 : (id == 4 ? 0.04 : 0.5)));
}

void fragment() {
	int layer = int(u_pages.x);
	vec2 ij = v_ij;
	float hx = bilinear(layer, ij + vec2(1.0, 0.0)) - bilinear(layer, ij - vec2(1.0, 0.0));
	float hy = bilinear(layer, ij + vec2(0.0, 1.0)) - bilinear(layer, ij - vec2(0.0, 1.0));
	vec3 up = normalize(v_up);
	vec3 n = normalize(cross(v_ts + up * (0.5 * hx), v_tt + up * (0.5 * hy)));
	mat3 to_world = mat3(MODEL_MATRIX);
	vec3 n_view = normalize((VIEW_MATRIX * vec4(to_world * n, 0.0)).xyz);

	if (debug_mode != 0 || !has_textures) {
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
		NORMAL = n_view;
	} else {
		// geology page: R gravel province, G bedrock patch,
		// B ice, A Viking darkness (planet_streamer.gd)
		vec2 guv = (v_ij * 0.25 + 1.5) / float(geology_side);
		vec4 color_map = textureLod(geology, vec3(guv, float(layer)), 0.0);
		vec2 wm = u_nmod.xy + v_local; // noise metres, 20 km period
		vec4 base_dd = vec4(dFdxCoarse(v_local), dFdyCoarse(v_local));

		float slope = clamp(1.0 - dot(n, up), 0.0, 1.0);
		float n_prov = color_map.r;
		float n_patch = color_map.g;
		float n_mot = texture(noise_texture, wm * 0.0004 + vec2(0.29)).r;
		float n_f1 = tnoise(wm * 0.03 + vec2(0.13));
		float n_f2 = tnoise(wm * 0.0025 + vec2(0.77));
		float n_f3 = tnoise(wm * 0.004 + vec2(0.52));

		float w_ice = color_map.b;
		float w_rock = smoothstep(slope_rock_start, slope_rock_start + 0.12, slope);
		float loc = smoothstep(0.35, 0.75, tnoise(wm * 0.001 + vec2(0.23))) * (1.0 - color_map.a);
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
		vec3 cam = INV_VIEW_MATRIX[3].xyz;
		vec3 frag = (INV_VIEW_MATRIX * vec4(VERTEX, 1.0)).xyz;
		vec3 up_world = normalize(to_world * up);
		float view_h = clamp((dot(cam - frag, up_world) - 3000.0) / 12000.0, 0.0, 1.0);
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
		vec2 wrp = (vec2(texture(noise_texture, wm * 0.0007).r,
			texture(noise_texture, wm * 0.0007 + vec2(0.41)).r) - 0.5) * 8.0;
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
			if (id == 1) {
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
					bool along_u = abs(dot(n, u_axis_u.xyz)) > abs(dot(n, u_axis_v.xyz));
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

		vec3 macrov = vec3(1.);
		if (enable_macro_variation) {
			float noise1 = tnoise(wm * 0.00125);
			float noise2 = tnoise(wm * 0.007 + vec2(0.61));
			macrov = mix(macro_variation1, vec3(1.), noise1);
			macrov *= mix(macro_variation2, vec3(1.), noise2);
			macrov = mix(vec3(1.0), macrov, clamp(dot(n, up) + macro_variation_slope, 0., 1.));
		}
		float roughness = clamp(max(normal_rough.a, roughness_floor), 0., 1.);
		ALBEDO = albedo_height.rgb * macrov;
		if (frost_amt > 0.001) {
			float level = clamp(dot(n, up) * 1.4 - 0.4, 0., 1.);
			float fr = frost_amt * (0.35 + 0.65 * level);
			ALBEDO = mix(ALBEDO, vec3(0.86, 0.88, 0.90), fr);
			roughness = mix(roughness, 0.75, fr);
		}
		ROUGHNESS = roughness;
		SPECULAR = specular_amount * (1.0 - view_h);
		// texture u runs along the face u axis
		vec3 t_model = normalize(u_axis_u.xyz - n * dot(n, u_axis_u.xyz));
		vec3 b_model = normalize(cross(n, t_model));
		NORMAL = n_view;
		TANGENT = normalize((VIEW_MATRIX * vec4(to_world * t_model, 0.0)).xyz);
		BINORMAL = normalize((VIEW_MATRIX * vec4(to_world * b_model, 0.0)).xyz);
		NORMAL_MAP = fma(normalize(normal_rough.xzy), vec3(0.5), vec3(0.5));
		NORMAL_MAP_DEPTH = normal_map_depth;
		float ao = (1. - (albedo_height.a * log(2.1 - ao_strength))) * (1. - normal_rough.y);
		AO = clamp(1. - ao * ao_strength, albedo_height.a, 1.0);
		AO_LIGHT_AFFECT = (1.0 - albedo_height.a) * clamp(normal_rough.y, 0., 1.);
	}
}

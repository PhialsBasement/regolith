// An example surface: Phobos's dark carbon-rich regolith, a
// little lighter on fresh steep slopes, with fine grain noise.

uniform vec3 regolith_color : source_color = vec3(0.16, 0.14, 0.12);
uniform vec3 fresh_color : source_color = vec3(0.24, 0.21, 0.18);

float ph_hash(vec2 i) {
	i = mod(i, 400.0);
	return fract(sin(dot(i, vec2(127.1, 311.7))) * 43758.5453);
}

float ph_noise(vec2 p) {
	vec2 i = floor(p);
	vec2 f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	return mix(mix(ph_hash(i), ph_hash(i + vec2(1.0, 0.0)), f.x), mix(ph_hash(i + vec2(0.0, 1.0)), ph_hash(i + vec2(1.0)), f.x), f.y);
}

void surface(TerrainPoint p, inout TerrainSurface s) {
	// 50 m cells: 400 of them span noise_m's 20 km period
	float pixel = max(length(p.dd.xy), length(p.dd.zw));
	float grain = mix(0.5, ph_noise(p.noise_m / 50.0), 1.0 - smoothstep(25.0, 100.0, pixel));
	float fresh = smoothstep(0.15, 0.45, p.slope);
	s.albedo = mix(regolith_color, fresh_color, fresh) * (0.85 + 0.3 * grain);
	s.roughness = 0.95;
	s.specular = 0.2;
}

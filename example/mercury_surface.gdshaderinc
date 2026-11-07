// An example surface: Mercury's grey, faintly brown regolith,
// fresher and brighter on steep crater walls.

// Mercury's albedo is about 0.12 to 0.14 (darker than Mars)
uniform vec3 plain_color : source_color = vec3(0.13, 0.125, 0.12);
uniform vec3 wall_color : source_color = vec3(0.19, 0.18, 0.17);

float me_hash(vec2 i) {
	i = mod(i, 200.0);
	return fract(sin(dot(i, vec2(127.1, 311.7))) * 43758.5453);
}

float me_noise(vec2 p) {
	vec2 i = floor(p);
	vec2 f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	return mix(mix(me_hash(i), me_hash(i + vec2(1.0, 0.0)), f.x), mix(me_hash(i + vec2(0.0, 1.0)), me_hash(i + vec2(1.0)), f.x), f.y);
}

void surface(TerrainPoint p, inout TerrainSurface s) {
	// 100 m cells: 200 of them span noise_m's 20 km period
	float pixel = max(length(p.dd.xy), length(p.dd.zw));
	float grain = mix(0.5, me_noise(p.noise_m / 100.0), 1.0 - smoothstep(50.0, 200.0, pixel));
	float wall = smoothstep(0.10, 0.35, p.slope);
	s.albedo = mix(plain_color, wall_color, wall) * (0.85 + 0.3 * grain);
	s.roughness = 0.95;
	s.specular = 0.2;
}

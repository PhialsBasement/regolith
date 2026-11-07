// Regolith's default surface, no textures: ground coloured by
// height, rock on slopes, noise that fades before it aliases.
// Write your own the same way; README, "Terrain surfaces".

uniform vec3 low_color : source_color = vec3(0.55, 0.36, 0.24);
uniform vec3 high_color : source_color = vec3(0.70, 0.52, 0.38);
uniform vec3 rock_color : source_color = vec3(0.30, 0.22, 0.17);
// heights over the reference radius (m): low_color at and under
// height_low, high_color at and over height_high
uniform float height_low = -4000.0;
uniform float height_high = 8000.0;
// slope (1 - cos) where rock starts and where it is all rock
uniform float rock_start : hint_range(0.0, 1.0) = 0.08;
uniform float rock_full : hint_range(0.0, 1.0) = 0.30;
uniform float variation : hint_range(0.0, 1.0) = 0.35;
uniform float ground_roughness : hint_range(0.0, 1.0) = 0.92;
uniform float rock_roughness : hint_range(0.0, 1.0) = 0.80;

// noise_m wraps every 20 km: lattices that divide it tile with it
float rg_hash(vec2 i, float period) {
	i = mod(i, period);
	return fract(sin(dot(i, vec2(127.1, 311.7))) * 43758.5453);
}

float rg_value(vec2 p, float period) {
	vec2 i = floor(p);
	vec2 f = fract(p);
	f = f * f * (3.0 - 2.0 * f);
	float a = rg_hash(i, period);
	float b = rg_hash(i + vec2(1.0, 0.0), period);
	float c = rg_hash(i + vec2(0.0, 1.0), period);
	float d = rg_hash(i + vec2(1.0, 1.0), period);
	return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

// cells of `cell` metres, faded out as they shrink under a pixel
float rg_octave(vec2 m, float cell, float pixel) {
	float fade = 1.0 - smoothstep(0.5, 2.0, pixel / cell);
	return mix(0.5, rg_value(m / cell, 20000.0 / cell), fade);
}

void surface(TerrainPoint p, inout TerrainSurface s) {
	float pixel = max(length(p.dd.xy), length(p.dd.zw));
	float n = rg_octave(p.noise_m, 2000.0, pixel) * 0.5 + rg_octave(p.noise_m, 400.0, pixel) * 0.3
			+ rg_octave(p.noise_m, 50.0, pixel) * 0.2;
	float h = smoothstep(height_low, height_high, p.height + (n - 0.5) * 2000.0);
	vec3 ground = mix(low_color, high_color, h);
	float rock = smoothstep(rock_start, rock_full, p.slope + (n - 0.5) * 0.08);
	vec3 albedo = mix(ground, rock_color, rock);
	s.albedo = albedo * (1.0 + (n - 0.5) * 2.0 * variation);
	s.roughness = mix(ground_roughness, rock_roughness, rock);
	s.specular = 0.3;
}

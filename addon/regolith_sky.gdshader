shader_type sky;

// mors's mars_sky.gdshader: dust-scatter day, blue aureole,
// long twilight, stars, Milky Way, Deimos and Earth. Driven by
// RegolithAtmosphere from the real geometry. Sun = LIGHT0.

uniform float sky_exposure : hint_range(0.0, 4.0) = 1.0;
// Air left above the camera, exp(-h/H): 0 in space.
uniform float sky_thin : hint_range(0.0, 1.0) = 1.0;
// Horizon dip acos(R/(R+h)) seen from altitude.
uniform float horizon_dip : hint_range(0.0, 1.5) = 0.0;
// Local vertical at the camera, in render space.
uniform vec3 up_dir = vec3(0.0, 1.0, 0.0);

// Pancam sky chromaticity (Bell 2006)
uniform vec3 day_zenith : source_color = vec3(0.42, 0.34, 0.27);
uniform vec3 day_horizon : source_color = vec3(0.80, 0.61, 0.46);
uniform vec3 night_zenith : source_color = vec3(0.004, 0.005, 0.011);
uniform vec3 night_horizon : source_color = vec3(0.014, 0.016, 0.030);
uniform vec3 twilight_tint : source_color = vec3(0.30, 0.22, 0.38);
uniform vec3 ground_color : source_color = vec3(0.05, 0.03, 0.02);

uniform float sun_disc_radius = 0.0031; // rad
uniform vec3 sun_tint : source_color = vec3(1.0, 0.97, 0.92);
uniform vec3 blue_halo : source_color = vec3(0.35, 0.48, 0.75);

uniform float star_intensity : hint_range(0.0, 3.0) = 0.5;
// Dust optical depth: 0.35 clear, ~1 dusty, 4-5 storm.
uniform float dust_tau : hint_range(0.1, 9.0) = 0.35;

// Terrain occlusion of the sun (mountain shadow on the air).
uniform float occl : hint_range(0.0, 1.0) = 0.0;
uniform float occl_horizon = -0.1;
uniform float occl_dist = 1e5;

uniform float milkyway_strength : hint_range(0.0, 2.0) = 0.25;
uniform vec3 milkyway_tint : source_color = vec3(0.55, 0.65, 0.95);
// Galactic pole in celestial (ICRF) coordinates.
uniform vec3 milkyway_normal = vec3(-0.8676, -0.1981, 0.4560);
// Columns: ICRF axes in render space.
uniform mat3 celestial_basis = mat3(1.0);

uniform vec3 deimos_dir = vec3(-0.5, 0.6, 0.6);
uniform float deimos_brightness = 0.9;
uniform vec3 earth_dir = vec3(0.0, -1.0, 0.0);
uniform float earth_phase = 1.0; // lit fraction seen from Mars
uniform vec3 earth_moon_dir = vec3(0.0, -1.0, 0.0);
uniform vec3 earth_tint : source_color = vec3(0.60, 0.76, 1.0);

float hash13(vec3 p) {
	p = fract(p * 0.1031);
	p += dot(p, p.zyx + 31.32);
	return fract((p.x + p.y) * p.z);
}

vec3 hash33(vec3 p) {
	p = fract(p * vec3(0.1031, 0.1030, 0.0973));
	p += dot(p, p.yxz + 33.33);
	return fract((p.xxy + p.yxx) * p.zyx);
}

float vnoise(vec3 p) {
	vec3 i = floor(p);
	vec3 f = fract(p);
	vec3 u = f * f * (3.0 - 2.0 * f);
	return mix(
		mix(mix(hash13(i), hash13(i + vec3(1.0, 0.0, 0.0)), u.x),
			mix(hash13(i + vec3(0.0, 1.0, 0.0)), hash13(i + vec3(1.0, 1.0, 0.0)), u.x), u.y),
		mix(mix(hash13(i + vec3(0.0, 0.0, 1.0)), hash13(i + vec3(1.0, 0.0, 1.0)), u.x),
			mix(hash13(i + vec3(0.0, 1.0, 1.0)), hash13(i + vec3(1.0, 1.0, 1.0)), u.x), u.y),
		u.z);
}

float fbm(vec3 p) {
	float amp = 0.5;
	float sum = 0.0;
	for (int i = 0; i < 4; i++) {
		sum += amp * vnoise(p);
		p *= 2.17;
		amp *= 0.5;
	}
	return sum;
}

// Stars do not twinkle: 1% atmosphere cannot scintillate.
vec3 star_layer(vec3 dir, float scale, float intensity) {
	vec3 p = dir * scale;
	vec3 id = floor(p);
	vec3 rnd = hash33(id);
	if (rnd.z < 0.86) {
		return vec3(0.0);
	}
	vec3 star_pos = id + vec3(0.25) + rnd * 0.5;
	float d = length(p - star_pos);
	float mag = pow(hash13(id + 13.1), 6.0) * 3.0 + 0.06;
	float m = exp(-d * d * 28.0) + exp(-d * d * 8.0) * 0.10 * mag;
	m *= smoothstep(0.5, 0.3, d);
	vec3 tint = mix(vec3(0.72, 0.84, 1.0), vec3(1.0, 0.86, 0.68), rnd.x * rnd.x);
	return tint * m * mag * intensity;
}

void sky() {
	vec3 up = normalize(up_dir);
	vec3 dir = EYEDIR;
	vec3 sdir = LIGHT0_ENABLED ? LIGHT0_DIRECTION : up;
	float dir_e = dot(dir, up);
	float sdir_e = dot(sdir, up);
	vec3 dir_h = dir - up * dir_e;
	vec3 sdir_h = sdir - up * sdir_e;
	// sin(sun elevation) against the true horizon
	float se = sin(asin(clamp(sdir_e, -1.0, 1.0)) + horizon_dip);

	float day = smoothstep(-0.05, 0.15, se);
	// dust glow lasts until the sun is ~17 deg down
	float twilight = smoothstep(-0.30, -0.02, se) * (1.0 - day);
	float night = 1.0 - max(day, twilight * 0.7);

	// Mountain shadow: each view ray dims by the unlit share
	// of its scattering path through the occluder's shadow.
	float shade = 0.0;
	if (occl > 0.001 && sky_thin > 0.01) {
		float hl = length(sdir_h) * length(dir_h);
		float azi = hl > 1e-6 ? smoothstep(0.10, 0.85, dot(sdir_h, dir_h) / hl) : 0.0;
		if (azi > 0.001) {
			float ev = asin(clamp(dir_e, -1.0, 1.0));
			float s_ang = asin(clamp(se, -1.0, 1.0));
			float h0 = occl_dist * max(tan(occl_horizon) - tan(s_ang), 0.0);
			float xstar = min(h0 / max(tan(ev) - tan(s_ang), 1e-4), 600000.0);
			float sigma = dust_tau / 11100.0;
			shade = occl * azi * (1.0 - exp(-sigma * xstar));
		}
	}

	// heavy dust turns the sky murky brown
	float murk = smoothstep(0.8, 4.5, dust_tau) * sky_thin;
	float upw = pow(clamp(dir_e, 0.0, 1.0), 0.65);
	vec3 day_col = mix(day_horizon, day_zenith, upw);
	day_col = mix(day_col, vec3(0.30, 0.19, 0.12) * (1.0 - 0.5 * upw), murk);
	vec3 night_col = mix(night_horizon, night_zenith, upw) * (1.0 - 0.85 * murk);
	// scattered daylight needs air above you
	vec3 col = mix(night_col, day_col * (1.0 - 0.6 * shade), day * sky_thin);

	float ang_sun = acos(clamp(dot(dir, sdir), -1.0, 1.0));

	if (twilight > 0.001 && dir_e >= 0.0) {
		float hd = length(dir_h) * length(sdir_h);
		float toward = 0.35 + 0.65 * max(hd > 1e-6 ? dot(dir_h, sdir_h) / hd : 0.0, 0.0);
		col += twilight_tint * twilight * toward * exp(-dir_e * 5.0) * sky_thin * (1.0 - 0.8 * shade);
	}

	if (dir_e < 0.0) {
		// below the horizon: dirt backscatter, or space
		col = mix(col, ground_color, clamp(-dir_e * 6.0, 0.0, 1.0) * sky_thin);
	}

	// blue aureole: dust forward scatter, strongest low
	float low_sun = 1.0 - smoothstep(0.15, 0.45, abs(se));
	float aureole = exp(-ang_sun * ang_sun * 22.0);
	col += blue_halo * aureole * low_sun * max(day, twilight) * 0.9 * (1.0 - 0.85 * murk) * sky_thin * (1.0 - 0.85 * shade);
	col += vec3(0.55, 0.40, 0.28) * exp(-ang_sun * ang_sun * 4.0) * day * 0.35 * (1.0 - 0.75 * murk) * sky_thin * (1.0 - 0.85 * shade);

	// sun disc dims with slant optical depth
	float airmass = 1.0 / max(se + 0.12, 0.12);
	float sun_vis = mix(1.0, exp(-max(dust_tau - 0.30, 0.0) * 0.40 * airmass), sky_thin);
	if (se > -0.06) {
		float disc = smoothstep(sun_disc_radius * 1.1, sun_disc_radius * 0.9, ang_sun);
		col += sun_tint * disc * 4.0 * smoothstep(-0.06, 0.0, se) * sun_vis;
		// glare of an unattenuated sun in vacuum
		col += sun_tint * exp(-ang_sun * ang_sun * 260.0) * 2.2 * (1.0 - sky_thin) * smoothstep(-0.06, 0.0, se);
		col += mix(sun_tint, vec3(0.55, 0.36, 0.22), murk) * exp(-ang_sun * ang_sun * 160.0) * murk * 0.22 * day;
	}

	// Night: stars, Milky Way, Deimos; dust buries them low.
	float extinction = smoothstep(-0.02, 0.22, dir_e) * (0.6 + 0.4 * smoothstep(0.22, 0.6, dir_e));
	extinction *= exp(-max(dust_tau - 0.35, 0.0) * 1.1);
	extinction = mix(1.0, extinction, sky_thin);
	// in space the stars are out at noon
	float dark = clamp(max(night, 1.0 - sky_thin) + twilight * 0.15, 0.0, 1.0);
	if (dark > 0.02 && extinction > 0.001) {
		vec3 cdir = dir * celestial_basis; // render -> ICRF
		float band_x = dot(cdir, normalize(milkyway_normal)) * 3.0;
		float band = exp(-band_x * band_x);
		float neb = fbm(cdir * 3.5 + 7.0);
		vec3 mw = milkyway_tint * band * milkyway_strength * (0.10 + 0.55 * neb * neb);
		vec3 stars = star_layer(cdir, 110.0, 1.0) + star_layer(cdir, 260.0, 0.55)
			+ star_layer(cdir, 520.0, 0.30) * (0.6 + 1.6 * band);
		stars *= star_intensity;
		col += (stars + mw) * extinction * dark * dark;

		float ang_d = acos(clamp(dot(dir, normalize(deimos_dir)), -1.0, 1.0));
		col += vec3(0.9, 0.88, 0.82) * exp(-ang_d * ang_d * 3.0e6) * deimos_brightness * extinction * dark;
		col += vec3(0.9, 0.88, 0.82) * exp(-ang_d * ang_d * 2.0e4) * 0.03 * deimos_brightness * extinction * dark;
	}

	// Earth: the inner-planet evening star, with Luna.
	{
		float e_show = clamp(max(max(night, twilight * 0.9), 1.0 - sky_thin), 0.0, 1.0);
		float e_horiz = mix(1.0, smoothstep(-0.03, 0.14, dir_e), sky_thin);
		float e_dust = exp(-max(dust_tau - 0.35, 0.0) * 1.1);
		float e_vis = e_show * e_horiz * mix(1.0, e_dust, sky_thin) * (1.0 - 0.85 * murk);
		if (e_vis > 0.002) {
			float e_bright = 0.35 + 0.65 * clamp(earth_phase, 0.0, 1.0);
			float ang_e = acos(clamp(dot(dir, normalize(earth_dir)), -1.0, 1.0));
			col += earth_tint * exp(-ang_e * ang_e * 2.6e6) * 2.4 * e_bright * e_vis;
			col += earth_tint * exp(-ang_e * ang_e * 1.4e4) * 0.045 * e_bright * e_vis;
			float ang_m = acos(clamp(dot(dir, normalize(earth_moon_dir)), -1.0, 1.0));
			col += vec3(0.82, 0.80, 0.78) * exp(-ang_m * ang_m * 4.0e6) * 0.35 * e_bright * e_vis;
		}
	}

	COLOR = col * sky_exposure;
}

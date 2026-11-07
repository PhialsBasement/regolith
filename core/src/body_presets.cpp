// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith/body_presets.hpp"

namespace regolith {

namespace {

// JPL approximate positions, Table 1 (Standish), 1800 to 2050
const OrbitElements MERCURY_ORBIT = { { 0.38709927, 0.00000037 }, { 0.20563593, 0.00001906 },
	{ 7.00497902, -0.00594749 }, { 252.25032350, 149472.67411175 }, { 77.45779628, 0.16047689 },
	{ 48.33076593, -0.12534081 } };
const OrbitElements VENUS_ORBIT = { { 0.72333566, 0.00000390 }, { 0.00677672, -0.00004107 },
	{ 3.39467605, -0.00078890 }, { 181.97909950, 58517.81538729 }, { 131.60246718, 0.00268329 },
	{ 76.67984255, -0.27769418 } };
const OrbitElements JUPITER_ORBIT = { { 5.20288700, -0.00011607 }, { 0.04838624, -0.00013253 },
	{ 1.30439695, -0.00183714 }, { 34.39644051, 3034.74612775 }, { 14.72847983, 0.21252668 },
	{ 100.47390909, 0.20469106 } };
const OrbitElements SATURN_ORBIT = { { 9.53667594, -0.00125060 }, { 0.05386179, -0.00050991 },
	{ 2.48599187, 0.00193609 }, { 49.95424423, 1222.49362201 }, { 92.59887831, -0.41897216 },
	{ 113.66242448, -0.28867794 } };
const OrbitElements URANUS_ORBIT = { { 19.18916464, -0.00196176 }, { 0.04725744, -0.00004397 },
	{ 0.77263783, -0.00242939 }, { 313.23810451, 428.48202785 }, { 170.95427630, 0.40805281 },
	{ 74.01692503, 0.04240589 } };
const OrbitElements NEPTUNE_ORBIT = { { 30.06992276, 0.00026291 }, { 0.00859048, 0.00005105 },
	{ 1.77004347, 0.00035372 }, { -55.12002969, 218.45945325 }, { 44.96476227, -0.32241464 },
	{ 131.78422574, -0.00508664 } };

BodyPreset make(const char *p_name, bool p_sync, const Rotation &p_rot, const OrbitElements &p_orbit, double p_gm,
		double p_j2, double p_j2_radius, double p_radius, double p_albedo, double p_r, double p_g, double p_b,
		bool p_helio = true) {
	BodyPreset b;
	b.name = p_name;
	b.synchronous = p_sync;
	b.rotation = p_rot;
	b.orbit = p_orbit;
	b.gm = p_gm;
	b.j2 = p_j2;
	b.j2_radius = p_j2_radius;
	b.radius = p_radius;
	b.albedo = p_albedo;
	b.tint[0] = p_r;
	b.tint[1] = p_g;
	b.tint[2] = p_b;
	b.heliocentric = p_helio;
	return b;
}

std::vector<BodyPreset> make_presets() {
	// rotations: IAU WGCCRE 2015 (Archinal et al. 2018), small
	// periodic terms left out; radii, albedos: NASA fact sheets
	std::vector<BodyPreset> p;
	p.push_back(make("Mercury", false, { { 281.0103, -0.0328 }, { 61.4155, -0.0049 }, { 329.5988, 6.1385108 } },
			MERCURY_ORBIT, 2.2032e13, 5.03e-5, 2440000.0, 2439700.0, 0.142, 0.85, 0.80, 0.75));
	p.push_back(make("Venus", false, { { 272.76, 0.0 }, { 67.16, 0.0 }, { 160.20, -1.4813688 } }, VENUS_ORBIT,
			3.24859e14, 4.458e-6, 6051800.0, 6051800.0, 0.689, 1.0, 0.95, 0.82));
	p.push_back(make("Earth", false, { { 0.00, -0.641 }, { 90.00, -0.557 }, { 190.147, 360.9856235 } }, EARTH_ORBIT,
			3.986004418e14, 1.08263e-3, 6378137.0, 6371000.0, 0.434, 0.55, 0.70, 1.0));
	p.push_back(make("Moon", false, { { 269.9949, 0.0031 }, { 66.5392, 0.0130 }, { 38.3213, 13.17635815 } },
			EARTH_ORBIT, 4.9028e12, 2.0330e-4, 1738000.0, 1737400.0, 0.12, 0.90, 0.85, 0.80, false));
	p.push_back(make("Mars", false, MARS_ROTATION, MARS_ORBIT, 4.282837e13, 1.96045e-3, 3396200.0, 3389500.0, 0.170,
			1.0, 0.60, 0.40));
	p.push_back(make("Phobos", true, {}, MARS_ORBIT, 7.087e5, 0.0, 11080.0, 11080.0, 0.071, 0.80, 0.75, 0.70, false));
	p.push_back(make("Deimos", true, {}, MARS_ORBIT, 9.62e4, 0.0, 6200.0, 6200.0, 0.068, 0.85, 0.78, 0.70, false));
	// osculating elements from JPL's small-body database, epoch
	// JD 2461200.5; GM and J2 from Dawn (Konopliv 2014, Park 2016)
	p.push_back(make("Vesta", false, { { 309.031, 0.0 }, { 42.235, 0.0 }, { 285.39, 1617.3329428 } },
			elements_from_osculating(2.361365965127599, 0.09020374382834395, 7.143925545058711, 103.701293265032,
					151.4686478221564, 81.19015607686903, 0.2716183613599909, 2461200.5),
			1.72882844e10, 0.0317799, 265000.0, 262700.0, 0.42, 0.85, 0.85, 0.85));
	p.push_back(make("Ceres", false, { { 291.418, 0.0 }, { 66.764, 0.0 }, { 170.650, 952.1532 } },
			elements_from_osculating(2.765552595034094, 0.07969229514816586, 10.58802780183462, 80.24862682043221,
					73.29421453021587, 274.4193463761342, 0.21430445064843, 2461200.5),
			6.26284e10, 0.026499, 470000.0, 469700.0, 0.09, 0.80, 0.80, 0.80));
	p.push_back(make("Jupiter", false, { { 268.056595, -0.006499 }, { 64.495303, 0.002413 }, { 284.95, 870.5360000 } },
			JUPITER_ORBIT, 1.26686534e17, 0.014736, 71492000.0, 69911000.0, 0.538, 1.0, 0.90, 0.75));
	p.push_back(make("Saturn", false, { { 40.589, -0.036 }, { 83.537, -0.004 }, { 38.90, 810.7939024 } }, SATURN_ORBIT,
			3.7931187e16, 0.016298, 60268000.0, 58232000.0, 0.499, 1.0, 0.90, 0.70));
	p.push_back(make("Uranus", false, { { 257.311, 0.0 }, { -15.175, 0.0 }, { 203.81, -501.1600928 } }, URANUS_ORBIT,
			5.793939e15, 0.003341, 25559000.0, 25362000.0, 0.488, 0.70, 0.90, 1.0));
	p.push_back(make("Neptune", false, { { 299.36, 0.0 }, { 43.46, 0.0 }, { 249.978, 541.1397757 } }, NEPTUNE_ORBIT,
			6.836529e15, 0.003411, 24764000.0, 24622000.0, 0.442, 0.50, 0.65, 1.0));
	return p;
}

} // namespace

const std::vector<BodyPreset> &body_presets() {
	static const std::vector<BodyPreset> presets = make_presets();
	return presets;
}

const BodyPreset *find_body_preset(const std::string &p_name) {
	for (const BodyPreset &b : body_presets()) {
		if (b.name == p_name) {
			return &b;
		}
	}
	return nullptr;
}

} // namespace regolith

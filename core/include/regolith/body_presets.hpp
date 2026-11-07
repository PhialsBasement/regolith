// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <vector>

#include "regolith/ephemeris.hpp"

namespace regolith {

// A real body's spin, heliocentric orbit and gravity, to fill a
// RegolithBody's settings in one go.
struct BodyPreset {
	std::string name;
	bool synchronous = false; // tidally locked to its orbit
	Rotation rotation{};
	// the sun's path for a central body; a satellite's is its
	// planet's (the Moon's, the Earth-Moon barycentre's)
	OrbitElements orbit{};
	double gm = 0.0; // m^3/s^2
	double j2 = 0.0;
	double j2_radius = 0.0; // m
	// as a light in another body's sky
	double radius = 0.0; // mean, m
	double albedo = 0.0; // geometric, V
	double tint[3] = { 1.0, 1.0, 1.0 };
	// orbits the Sun itself (a planet or asteroid), with this orbit
	bool heliocentric = true;
};

// Mercury, Venus, Earth, Moon, Mars, Phobos, Deimos, Vesta,
// Ceres, Jupiter, Saturn, Uranus, Neptune
const std::vector<BodyPreset> &body_presets();
const BodyPreset *find_body_preset(const std::string &p_name);

} // namespace regolith

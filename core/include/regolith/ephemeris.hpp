// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "regolith/vec.hpp"

namespace regolith {

// TT seconds since J2000 (2000-01-01 12:00 TT) from Unix.
double j2000_seconds_from_unix(double p_unix);

// A heliocentric orbit as JPL's approximate elements (Standish):
// value at J2000 and rate per Julian century. a in AU, angles in
// degrees, ecliptic of J2000; mean_long and peri are longitudes.
struct OrbitElements {
	double a[2], e[2], inc[2], mean_long[2], peri[2], node[2];
};

// Osculating elements at an epoch (as JPL's small-body database
// gives them): a (AU), e, i, node, argument of perihelion, mean
// anomaly (deg), mean motion (deg/day), epoch (JD TDB).
OrbitElements elements_from_osculating(double p_a, double p_e, double p_inc, double p_node, double p_arg_peri,
		double p_mean_anom, double p_motion, double p_epoch_jd);

// IAU rotation (WGCCRE): pole right ascension and declination
// (deg, rate per century), prime meridian W (deg, rate per day).
struct Rotation {
	double ra[2], dec[2], w[2];
};

// Heliocentric position, J2000 equatorial, AU.
DVec3 heliocentric(const OrbitElements &p_orbit, double p_seconds);
// Body-fixed vector to J2000 equatorial and back.
DVec3 body_to_inertial(const Rotation &p_rot, double p_seconds, const DVec3 &p_body);
DVec3 inertial_to_body(const Rotation &p_rot, double p_seconds, const DVec3 &p_inertial);
// Solar longitude (deg 0..360): the sun's place on the body's
// orbit from its northern spring equinox.
double solar_longitude(const OrbitElements &p_orbit, const Rotation &p_rot, double p_seconds);
// Unit vector to the sun in the body-fixed coordinates of a body on
// p_orbit turning as p_rot.
DVec3 sun_direction(const OrbitElements &p_orbit, const Rotation &p_rot, double p_seconds);

extern const OrbitElements MARS_ORBIT;
extern const OrbitElements EARTH_ORBIT; // Earth-Moon barycentre
extern const Rotation MARS_ROTATION;

// Mars, as the functions above with Mars's numbers.
DVec3 mars_heliocentric(double p_seconds);
DVec3 earth_heliocentric(double p_seconds);
// North galactic pole, ICRF unit vector.
DVec3 galactic_pole();
// Body-fixed Mars vector to J2000 equatorial (IAU WGCCRE pole and
// prime meridian; Archinal et al.).
DVec3 mars_body_to_inertial(double p_seconds, const DVec3 &p_body);
DVec3 mars_inertial_to_body(double p_seconds, const DVec3 &p_inertial);
// Unit vector to the sun, Mars body-fixed coordinates.
DVec3 mars_sun_direction(double p_seconds);
// Areocentric solar longitude Ls, degrees 0..360.
double mars_solar_longitude(double p_seconds);
// Local true solar time (0..24 h) at an east longitude.
double mars_local_solar_time(double p_seconds, double p_lon_deg);
// Mean solar day of Mars, seconds.
constexpr double MARS_SOL = 88775.244;

// The Moon from the Earth's centre, J2000 equatorial, metres
// (Meeus ch. 47, main terms: about 0.3 deg).
DVec3 moon_geocentric(double p_seconds);

// Local solar time (0..24 h) at an east longitude, from the sun's
// direction in body-fixed coordinates.
double local_solar_time(const DVec3 &p_sun_body, double p_lon_deg);

} // namespace regolith

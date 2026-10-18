#pragma once

#include "regolith/vec.hpp"

namespace regolith {

// TT seconds since J2000 (2000-01-01 12:00 TT) from Unix.
double j2000_seconds_from_unix(double p_unix);

// Mars heliocentric position, J2000 equatorial, AU, from
// Standish's mean elements (JPL, valid 1800 to 2050).
DVec3 mars_heliocentric(double p_seconds);

// Body-fixed Mars vector to J2000 equatorial (IAU WGCCRE
// pole and prime meridian; Archinal et al.).
DVec3 mars_body_to_inertial(double p_seconds, const DVec3 &p_body);
DVec3 mars_inertial_to_body(double p_seconds, const DVec3 &p_inertial);

// Unit vector to the sun, Mars body-fixed coordinates.
DVec3 mars_sun_direction(double p_seconds);

// Local true solar time (0..24 h) at an east longitude.
double mars_local_solar_time(double p_seconds, double p_lon_deg);

// Mean solar day of Mars, seconds.
constexpr double MARS_SOL = 88775.244;

} // namespace regolith

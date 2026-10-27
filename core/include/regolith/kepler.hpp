#pragma once

#include "regolith/vec.hpp"

namespace regolith {

// Two-body propagation by universal variables: works for
// ellipses, parabolas and hyperbolas alike.
void kepler_propagate(const DVec3 &p_r0, const DVec3 &p_v0, double p_gm, double p_dt, DVec3 &r_r, DVec3 &r_v);

struct OrbitShape {
	double a = 0.0; // semi-major axis, negative if hyperbolic
	double e = 0.0;
	double inclination = 0.0; // radians, against p_pole
	double periapsis = 0.0; // radius
	double apoapsis = 0.0; // radius, infinite if unbound
	double period = 0.0; // seconds, infinite if unbound
};

OrbitShape orbit_shape(const DVec3 &p_r, const DVec3 &p_v, double p_gm, const DVec3 &p_pole);

} // namespace regolith

#include "regolith/kepler.hpp"

#include <cmath>
#include <limits>
#include <numbers>

namespace regolith {

namespace {

double stumpff_c(double p_z) {
	if (p_z > 1e-6) {
		return (1.0 - std::cos(std::sqrt(p_z))) / p_z;
	}
	if (p_z < -1e-6) {
		return (std::cosh(std::sqrt(-p_z)) - 1.0) / -p_z;
	}
	return 0.5 - p_z / 24.0 + p_z * p_z / 720.0;
}

double stumpff_s(double p_z) {
	if (p_z > 1e-6) {
		const double s = std::sqrt(p_z);
		return (s - std::sin(s)) / (s * s * s);
	}
	if (p_z < -1e-6) {
		const double s = std::sqrt(-p_z);
		return (std::sinh(s) - s) / (s * s * s);
	}
	return 1.0 / 6.0 - p_z / 120.0 + p_z * p_z / 5040.0;
}

} // namespace

void kepler_propagate(const DVec3 &p_r0, const DVec3 &p_v0, double p_gm, double p_dt, DVec3 &r_r, DVec3 &r_v) {
	const double r0 = length(p_r0);
	const double vr0 = dot(p_r0, p_v0) / r0;
	const double alpha = 2.0 / r0 - dot(p_v0, p_v0) / p_gm;
	const double sq = std::sqrt(p_gm);
	double dt = p_dt;
	// Whole revolutions change nothing; drop them for Newton.
	if (alpha > 1e-12) {
		const double period = 2.0 * std::numbers::pi / (sq * std::pow(alpha, 1.5));
		dt = std::fmod(dt, period);
	}
	double chi = sq * std::fabs(alpha) * dt;
	if (alpha <= 1e-12) {
		chi = sq * dt / r0;
	}
	for (int k = 0; k < 60; ++k) {
		const double z = alpha * chi * chi;
		const double c = stumpff_c(z), s = stumpff_s(z);
		const double f = r0 * vr0 / sq * chi * chi * c + (1.0 - alpha * r0) * chi * chi * chi * s + r0 * chi - sq * dt;
		const double df = r0 * vr0 / sq * chi * (1.0 - alpha * chi * chi * s) + (1.0 - alpha * r0) * chi * chi * c + r0;
		const double step = f / df;
		chi -= step;
		if (std::fabs(step) < 1e-12 * std::fmax(1.0, std::fabs(chi))) {
			break;
		}
	}
	const double z = alpha * chi * chi;
	const double c = stumpff_c(z), s = stumpff_s(z);
	const double f = 1.0 - chi * chi / r0 * c;
	const double g = dt - chi * chi * chi / sq * s;
	r_r = p_r0 * f + p_v0 * g;
	const double r = length(r_r);
	const double fdot = sq / (r * r0) * (alpha * chi * chi * chi * s - chi);
	const double gdot = 1.0 - chi * chi / r * c;
	r_v = p_r0 * fdot + p_v0 * gdot;
}

OrbitShape orbit_shape(const DVec3 &p_r, const DVec3 &p_v, double p_gm, const DVec3 &p_pole) {
	OrbitShape o;
	const double r = length(p_r);
	const DVec3 h = cross(p_r, p_v);
	const DVec3 evec = cross(p_v, h) / p_gm - p_r / r;
	o.e = length(evec);
	const double energy = 0.5 * dot(p_v, p_v) - p_gm / r;
	o.a = -p_gm / (2.0 * energy);
	const double p = dot(h, h) / p_gm;
	o.periapsis = p / (1.0 + o.e);
	o.inclination = std::acos(std::fmax(-1.0, std::fmin(1.0, dot(normalize(h), p_pole))));
	if (o.e < 1.0) {
		o.apoapsis = p / (1.0 - o.e);
		o.period = 2.0 * std::numbers::pi * std::sqrt(o.a * o.a * o.a / p_gm);
	} else {
		o.apoapsis = std::numeric_limits<double>::infinity();
		o.period = std::numeric_limits<double>::infinity();
	}
	return o;
}

} // namespace regolith

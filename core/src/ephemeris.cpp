#include "regolith/ephemeris.hpp"

#include <cmath>
#include <numbers>

namespace regolith {

namespace {

constexpr double DEG = std::numbers::pi / 180.0;
constexpr double DAY = 86400.0;
constexpr double CENTURY = 36525.0 * DAY;
constexpr double OBLIQUITY = 23.43928 * DEG;

DVec3 rot_x(const DVec3 &p_v, double p_a) {
	const double c = std::cos(p_a), s = std::sin(p_a);
	return { p_v.x, c * p_v.y - s * p_v.z, s * p_v.y + c * p_v.z };
}

DVec3 rot_z(const DVec3 &p_v, double p_a) {
	const double c = std::cos(p_a), s = std::sin(p_a);
	return { c * p_v.x - s * p_v.y, s * p_v.x + c * p_v.y, p_v.z };
}

struct MarsOrientation {
	double ra; // pole right ascension, rad
	double dec; // pole declination, rad
	double w; // prime meridian angle, rad
};

MarsOrientation orientation(double p_seconds) {
	const double d = p_seconds / DAY;
	const double t = p_seconds / CENTURY;
	return {
		(317.68143 - 0.1061 * t) * DEG,
		(52.88650 - 0.0609 * t) * DEG,
		std::fmod(176.630 + 350.89198226 * d, 360.0) * DEG,
	};
}

} // namespace

double j2000_seconds_from_unix(double p_unix) {
	// J2000 = Unix 946727935.816 UTC. TT - UTC was 64.184 s
	// then and is 69.184 s now (5 leap seconds since).
	return p_unix - 946727935.816 + 69.184 - 64.184;
}

namespace {

// Standish's elements and their rates per century: a (AU),
// e, I, L, long. perihelion, long. node (degrees).
struct Elements {
	double a[2], e[2], inc[2], mean_long[2], peri[2], node[2];
};

constexpr Elements MARS_ELEMENTS = { { 1.52371034, 0.00001847 }, { 0.09339410, 0.00007882 },
	{ 1.84969142, -0.00813131 }, { -4.55343205, 19140.30268499 }, { -23.94362959, 0.44441088 },
	{ 49.55953891, -0.29257343 } };
// Earth-Moon barycentre.
constexpr Elements EARTH_ELEMENTS = { { 1.00000261, 0.00000562 }, { 0.01671123, -0.00004392 },
	{ -0.00001531, -0.01294668 }, { 100.46457166, 35999.37244981 }, { 102.93768193, 0.32327364 },
	{ 0.0, 0.0 } };

DVec3 heliocentric(const Elements &p_el, double p_seconds) {
	const double t = p_seconds / CENTURY;
	const double a = p_el.a[0] + p_el.a[1] * t;
	const double e = p_el.e[0] + p_el.e[1] * t;
	const double inc = (p_el.inc[0] + p_el.inc[1] * t) * DEG;
	const double mean_long = (p_el.mean_long[0] + p_el.mean_long[1] * t) * DEG;
	const double peri = (p_el.peri[0] + p_el.peri[1] * t) * DEG;
	const double node = (p_el.node[0] + p_el.node[1] * t) * DEG;
	const double arg_peri = peri - node;
	const double mean_anom = std::remainder(mean_long - peri, 2.0 * std::numbers::pi);
	double ecc = mean_anom + e * std::sin(mean_anom);
	for (int k = 0; k < 8; ++k) {
		ecc -= (ecc - e * std::sin(ecc) - mean_anom) / (1.0 - e * std::cos(ecc));
	}
	const DVec3 orbital(a * (std::cos(ecc) - e), a * std::sqrt(1.0 - e * e) * std::sin(ecc), 0.0);
	const DVec3 ecliptic = rot_z(rot_x(rot_z(orbital, arg_peri), inc), node);
	return rot_x(ecliptic, OBLIQUITY);
}

} // namespace

DVec3 mars_heliocentric(double p_seconds) {
	return heliocentric(MARS_ELEMENTS, p_seconds);
}

DVec3 earth_heliocentric(double p_seconds) {
	return heliocentric(EARTH_ELEMENTS, p_seconds);
}

double mars_solar_longitude(double p_seconds) {
	const DVec3 r = mars_heliocentric(p_seconds);
	const DVec3 n = normalize(cross(r, mars_heliocentric(p_seconds + 3600.0) - mars_heliocentric(p_seconds - 3600.0)));
	const DVec3 pole = mars_body_to_inertial(p_seconds, DVec3(0, 0, 1));
	// the sun crosses the equator northward along this direction
	const DVec3 eq = normalize(cross(pole, n));
	const DVec3 s = normalize(r * -1.0);
	const double ls = std::atan2(dot(s, cross(n, eq)), dot(s, eq)) * 180.0 / std::numbers::pi;
	return ls < 0.0 ? ls + 360.0 : ls;
}

DVec3 galactic_pole() {
	// RA 192.85948, Dec 27.12825 (ICRF)
	const double ra = 192.85948 * DEG, dec = 27.12825 * DEG;
	return { std::cos(dec) * std::cos(ra), std::cos(dec) * std::sin(ra), std::sin(dec) };
}

DVec3 mars_body_to_inertial(double p_seconds, const DVec3 &p_body) {
	const MarsOrientation o = orientation(p_seconds);
	return rot_z(rot_x(rot_z(p_body, o.w), std::numbers::pi / 2.0 - o.dec), o.ra + std::numbers::pi / 2.0);
}

DVec3 mars_inertial_to_body(double p_seconds, const DVec3 &p_inertial) {
	const MarsOrientation o = orientation(p_seconds);
	return rot_z(rot_x(rot_z(p_inertial, -(o.ra + std::numbers::pi / 2.0)), -(std::numbers::pi / 2.0 - o.dec)), -o.w);
}

DVec3 mars_sun_direction(double p_seconds) {
	return normalize(mars_inertial_to_body(p_seconds, mars_heliocentric(p_seconds) * -1.0));
}

double mars_local_solar_time(double p_seconds, double p_lon_deg) {
	const DVec3 s = mars_sun_direction(p_seconds);
	const double subsolar = std::atan2(s.y, s.x) / DEG;
	return std::fmod(std::fmod(12.0 + (p_lon_deg - subsolar) / 15.0, 24.0) + 24.0, 24.0);
}

} // namespace regolith

#include "regolith/ephemeris.hpp"

#include <cmath>
#include <numbers>

namespace regolith {

namespace {

constexpr double DEG = std::numbers::pi / 180.0;
constexpr double DAY = 86400.0;
constexpr double CENTURY = 36525.0 * DAY;
constexpr double OBLIQUITY = 23.43928 * DEG;
constexpr double J2000_JD = 2451545.0;

DVec3 rot_x(const DVec3 &p_v, double p_a) {
	const double c = std::cos(p_a), s = std::sin(p_a);
	return { p_v.x, c * p_v.y - s * p_v.z, s * p_v.y + c * p_v.z };
}

DVec3 rot_z(const DVec3 &p_v, double p_a) {
	const double c = std::cos(p_a), s = std::sin(p_a);
	return { c * p_v.x - s * p_v.y, s * p_v.x + c * p_v.y, p_v.z };
}

struct Orientation {
	double ra; // pole right ascension, rad
	double dec; // pole declination, rad
	double w; // prime meridian angle, rad
};

Orientation orientation(const Rotation &p_rot, double p_seconds) {
	const double d = p_seconds / DAY;
	const double t = p_seconds / CENTURY;
	return {
		(p_rot.ra[0] + p_rot.ra[1] * t) * DEG,
		(p_rot.dec[0] + p_rot.dec[1] * t) * DEG,
		std::fmod(p_rot.w[0] + p_rot.w[1] * d, 360.0) * DEG,
	};
}

} // namespace

// Standish's elements and their rates per century: a (AU), e,
// I, L, long. perihelion, long. node (degrees).
const OrbitElements MARS_ORBIT = { { 1.52371034, 0.00001847 }, { 0.09339410, 0.00007882 },
	{ 1.84969142, -0.00813131 }, { -4.55343205, 19140.30268499 }, { -23.94362959, 0.44441088 },
	{ 49.55953891, -0.29257343 } };
const OrbitElements EARTH_ORBIT = { { 1.00000261, 0.00000562 }, { 0.01671123, -0.00004392 },
	{ -0.00001531, -0.01294668 }, { 100.46457166, 35999.37244981 }, { 102.93768193, 0.32327364 },
	{ 0.0, 0.0 } };
// Archinal et al. 2018 (WGCCRE 2015)
const Rotation MARS_ROTATION = { { 317.68143, -0.1061 }, { 52.88650, -0.0609 }, { 176.630, 350.89198226 } };

double j2000_seconds_from_unix(double p_unix) {
	// J2000 = Unix 946727935.816 UTC. TT - UTC was 64.184 s
	// then and is 69.184 s now (5 leap seconds since).
	return p_unix - 946727935.816 + 69.184 - 64.184;
}

OrbitElements elements_from_osculating(double p_a, double p_e, double p_inc, double p_node, double p_arg_peri,
		double p_mean_anom, double p_motion, double p_epoch_jd) {
	const double peri = p_node + p_arg_peri;
	// the mean longitude back at J2000, then advancing at n
	const double mean_long = std::fmod(peri + p_mean_anom - p_motion * (p_epoch_jd - J2000_JD), 360.0);
	return { { p_a, 0.0 }, { p_e, 0.0 }, { p_inc, 0.0 }, { mean_long, p_motion * 36525.0 }, { peri, 0.0 },
		{ p_node, 0.0 } };
}

DVec3 heliocentric(const OrbitElements &p_el, double p_seconds) {
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

DVec3 body_to_inertial(const Rotation &p_rot, double p_seconds, const DVec3 &p_body) {
	const Orientation o = orientation(p_rot, p_seconds);
	return rot_z(rot_x(rot_z(p_body, o.w), std::numbers::pi / 2.0 - o.dec), o.ra + std::numbers::pi / 2.0);
}

DVec3 inertial_to_body(const Rotation &p_rot, double p_seconds, const DVec3 &p_inertial) {
	const Orientation o = orientation(p_rot, p_seconds);
	return rot_z(rot_x(rot_z(p_inertial, -(o.ra + std::numbers::pi / 2.0)), -(std::numbers::pi / 2.0 - o.dec)), -o.w);
}

double solar_longitude(const OrbitElements &p_orbit, const Rotation &p_rot, double p_seconds) {
	const DVec3 r = heliocentric(p_orbit, p_seconds);
	const DVec3 n = normalize(
			cross(r, heliocentric(p_orbit, p_seconds + 3600.0) - heliocentric(p_orbit, p_seconds - 3600.0)));
	const DVec3 pole = body_to_inertial(p_rot, p_seconds, DVec3(0, 0, 1));
	// the sun crosses the equator northward along this direction
	const DVec3 eq = normalize(cross(pole, n));
	const DVec3 s = normalize(r * -1.0);
	const double ls = std::atan2(dot(s, cross(n, eq)), dot(s, eq)) * 180.0 / std::numbers::pi;
	return ls < 0.0 ? ls + 360.0 : ls;
}

DVec3 moon_geocentric(double p_seconds) {
	const double t = p_seconds / CENTURY;
	const double lp = 218.3165 + 481267.8813 * t;
	const double mp = (134.9634 + 477198.8676 * t) * DEG;
	const double f = (93.2721 + 483202.0175 * t) * DEG;
	const double d = (297.8502 + 445267.1115 * t) * DEG;
	const double m = (357.5291 + 35999.0503 * t) * DEG;
	const double lon = (lp + 6.289 * std::sin(mp) - 1.274 * std::sin(mp - 2.0 * d) + 0.658 * std::sin(2.0 * d) +
							   0.214 * std::sin(2.0 * mp) - 0.186 * std::sin(m) - 0.114 * std::sin(2.0 * f)) *
			DEG;
	const double lat = (5.128 * std::sin(f) + 0.281 * std::sin(mp + f) + 0.278 * std::sin(mp - f) +
							   0.173 * std::sin(2.0 * d - f)) *
			DEG;
	const double km = 385001.0 - 20905.0 * std::cos(mp) - 3699.0 * std::cos(2.0 * d - mp) - 2956.0 * std::cos(2.0 * d);
	const DVec3 ecl(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
	return rot_x(ecl, OBLIQUITY) * (km * 1000.0);
}

double local_solar_time(const DVec3 &p_sun_body, double p_lon_deg) {
	const double subsolar = std::atan2(p_sun_body.y, p_sun_body.x) / DEG;
	return std::fmod(std::fmod(12.0 + (p_lon_deg - subsolar) / 15.0, 24.0) + 24.0, 24.0);
}

DVec3 mars_heliocentric(double p_seconds) {
	return heliocentric(MARS_ORBIT, p_seconds);
}

DVec3 earth_heliocentric(double p_seconds) {
	return heliocentric(EARTH_ORBIT, p_seconds);
}

double mars_solar_longitude(double p_seconds) {
	return solar_longitude(MARS_ORBIT, MARS_ROTATION, p_seconds);
}

DVec3 galactic_pole() {
	// RA 192.85948, Dec 27.12825 (ICRF)
	const double ra = 192.85948 * DEG, dec = 27.12825 * DEG;
	return { std::cos(dec) * std::cos(ra), std::cos(dec) * std::sin(ra), std::sin(dec) };
}

DVec3 mars_body_to_inertial(double p_seconds, const DVec3 &p_body) {
	return body_to_inertial(MARS_ROTATION, p_seconds, p_body);
}

DVec3 mars_inertial_to_body(double p_seconds, const DVec3 &p_inertial) {
	return inertial_to_body(MARS_ROTATION, p_seconds, p_inertial);
}

DVec3 mars_sun_direction(double p_seconds) {
	return normalize(mars_inertial_to_body(p_seconds, mars_heliocentric(p_seconds) * -1.0));
}

double mars_local_solar_time(double p_seconds, double p_lon_deg) {
	return local_solar_time(mars_sun_direction(p_seconds), p_lon_deg);
}

} // namespace regolith

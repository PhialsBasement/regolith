#include <cmath>
#include <numbers>

#include "regolith/body_presets.hpp"
#include "regolith/ephemeris.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

double declination_deg(double p_unix) {
	const DVec3 s = mars_sun_direction(j2000_seconds_from_unix(p_unix));
	return std::asin(s.z) * 180.0 / std::numbers::pi;
}

} // namespace

TEST_CASE(mars_seasons) {
	// Mars Year 37 began (Ls 0) on 2022-12-26; Ls 90 came
	// 199 days later, 2023-07-13. Expect ~0 and ~+25.2 deg.
	const double equinox = declination_deg(1672056000.0);
	const double solstice = declination_deg(1689249600.0);
	std::printf("  sun declination: %.2f deg at Ls 0, %.2f deg at Ls 90\n", equinox, solstice);
	CHECK(std::abs(equinox) < 2.0);
	CHECK(std::abs(solstice - 25.19) < 1.0);
	const double r = length(mars_heliocentric(j2000_seconds_from_unix(1672056000.0)));
	CHECK(r > 1.38 && r < 1.67);
}

namespace {

// Allison and McEwen 2000 (Mars24), eq. B-4 to B-5: an
// independent Ls from TT days since J2000.
double ls_mars24(double p_dt) {
	constexpr double D = std::numbers::pi / 180.0;
	const double m = (19.3871 + 0.52402073 * p_dt) * D;
	const double afms = 270.3871 + 0.524038496 * p_dt;
	constexpr double A[7] = { 0.0071, 0.0057, 0.0039, 0.0037, 0.0021, 0.0020, 0.0018 };
	constexpr double TAU[7] = { 2.2353, 2.7543, 1.1177, 15.7866, 2.1354, 2.4694, 32.8493 };
	constexpr double PHI[7] = { 49.409, 168.173, 191.837, 21.736, 15.704, 95.528, 49.095 };
	double pbs = 0.0;
	for (int i = 0; i < 7; ++i) {
		pbs += A[i] * std::cos((0.985626 * p_dt / TAU[i] + PHI[i]) * D);
	}
	const double nu_m = (10.691 + 3.0e-7 * p_dt) * std::sin(m) + 0.623 * std::sin(2 * m) + 0.050 * std::sin(3 * m) +
			0.005 * std::sin(4 * m) + 0.0005 * std::sin(5 * m) + pbs;
	return std::fmod(afms + nu_m + 3600.0, 360.0);
}

} // namespace

TEST_CASE(solar_longitude_matches_mars24) {
	double worst = 0.0;
	for (int k = 0; k < 120; ++k) {
		// one Mars year from 2023, every 5.7 days
		const double t = j2000_seconds_from_unix(1672531200.0) + k * 686.98 / 120.0 * 86400.0;
		const double err = std::fabs(std::remainder(mars_solar_longitude(t) - ls_mars24(t / 86400.0), 360.0));
		worst = std::fmax(worst, err);
	}
	std::printf("  Ls against Mars24 over a Mars year: worst %.3f deg\n", worst);
	CHECK(worst < 0.1);
}

TEST_CASE(earth_at_mars_opposition) {
	// Mars opposition 2025-01-16 02:32 UTC: from Mars, Earth sits
	// in front of the sun (inferior conjunction).
	const double t = j2000_seconds_from_unix(1736994720.0);
	// Equal heliocentric ecliptic longitudes (not latitudes:
	// Mars's 1.85 deg tilt keeps Earth off the solar disc).
	auto ecl_lon = [](const DVec3 &p_v) {
		const double eps = 23.43928 * std::numbers::pi / 180.0;
		const double y = p_v.y * std::cos(eps) + p_v.z * std::sin(eps);
		return std::atan2(y, p_v.x) * 180.0 / std::numbers::pi;
	};
	const double d = std::remainder(ecl_lon(earth_heliocentric(t)) - ecl_lon(mars_heliocentric(t)), 360.0);
	std::printf("  heliocentric longitude difference at opposition: %.3f deg\n", d);
	CHECK(std::fabs(d) < 0.3);
}

TEST_CASE(mars_local_solar_time_runs_one_sol) {
	const double t = j2000_seconds_from_unix(1760000000.0);
	const double a = mars_local_solar_time(t, 137.8);
	const double b = mars_local_solar_time(t + MARS_SOL / 4.0, 137.8);
	const double gained = std::fmod(b - a + 24.0, 24.0);
	std::printf("  a quarter sol advances local solar time %.3f h\n", gained);
	CHECK(std::abs(gained - 6.0) < 0.1);
	const DVec3 s = mars_sun_direction(t);
	const double subsolar = std::atan2(s.y, s.x) * 180.0 / std::numbers::pi;
	CHECK(std::abs(mars_local_solar_time(t, subsolar) - 12.0) < 1e-6);
}

namespace {

// the sun, body-fixed, for a central body preset
DVec3 preset_sun(const BodyPreset &p_b, double p_t) {
	return normalize(inertial_to_body(p_b.rotation, p_t, heliocentric(p_b.orbit, p_t) * -1.0));
}

} // namespace

TEST_CASE(earth_preset_noon_at_greenwich_j2000) {
	// 2000-01-01 12:00 TT: noon less the equation of time, about
	// 3.3 minutes early in early January
	const BodyPreset *earth = find_body_preset("Earth");
	CHECK(earth != nullptr);
	const double lst = local_solar_time(preset_sun(*earth, 0.0), 0.0);
	std::printf("  Greenwich local solar time at J2000: %.4f h\n", lst);
	CHECK(std::abs(lst - (12.0 - 3.3 / 60.0)) < 0.05);
}

TEST_CASE(vesta_preset_orbit_and_day) {
	const BodyPreset *vesta = find_body_preset("Vesta");
	CHECK(vesta != nullptr);
	double lo = 1e9, hi = 0.0;
	for (int k = 0; k < 400; ++k) {
		const double r = length(heliocentric(vesta->orbit, k * 86400.0 * 4.0));
		lo = std::min(lo, r);
		hi = std::max(hi, r);
	}
	std::printf("  Vesta from the sun: %.3f to %.3f AU\n", lo, hi);
	CHECK(lo > 2.14 && lo < 2.16 && hi > 2.56 && hi < 2.58);
	// a solar day a little longer than the 5.3421 h sidereal one
	const double t = 7.8e8;
	const double a = local_solar_time(preset_sun(*vesta, t), 0.0);
	const double b = local_solar_time(preset_sun(*vesta, t + 3600.0), 0.0);
	const double day_h = 24.0 / std::fmod(b - a + 24.0, 24.0);
	std::printf("  Vesta solar day: %.4f h\n", day_h);
	CHECK(day_h > 5.3421 && day_h < 5.35);
}

TEST_CASE(moon_opposite_the_sun_at_the_2000_eclipse) {
	// total lunar eclipse, 2000-01-21 04:44 UT (19.697 d after J2000)
	const double t = 19.697 * 86400.0;
	const DVec3 moon = moon_geocentric(t);
	const DVec3 anti_sun = normalize(earth_heliocentric(t));
	const double sep = std::acos(dot(normalize(moon), anti_sun)) * 180.0 / std::numbers::pi;
	std::printf("  moon %.0f km away, %.2f deg from the anti-sun\n", length(moon) / 1000.0, sep);
	CHECK(sep < 1.0);
	CHECK(length(moon) > 356e6 && length(moon) < 407e6);
}

TEST_CASE(preset_rotations_round_trip) {
	const DVec3 v = normalize(DVec3(0.3, -0.7, 0.4));
	for (const BodyPreset &b : body_presets()) {
		const DVec3 back = inertial_to_body(b.rotation, 5.0e8, body_to_inertial(b.rotation, 5.0e8, v));
		CHECK(length(back - v) < 1e-12);
	}
}

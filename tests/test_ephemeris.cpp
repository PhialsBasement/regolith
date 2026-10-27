#include <cmath>
#include <numbers>

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

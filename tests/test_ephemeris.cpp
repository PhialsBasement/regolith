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

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cmath>
#include <numbers>

#include "regolith/body_presets.hpp"
#include "regolith/ephemeris.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

constexpr double DEG = std::numbers::pi / 180.0;
constexpr double SECONDS_PER_DAY = 86400.0;

// Mars Year 37 began (Ls 0) on 2022-12-26; Ls 90 came 199
// days later, on 2023-07-13 (Mars24).
constexpr double MARS_YEAR_37_LS_0_UNIX = 1672056000.0;
constexpr double MARS_YEAR_37_LS_90_UNIX = 1689249600.0;
constexpr double MARS_OBLIQUITY_DEG = 25.19;

constexpr double MARS_PERIHELION_AU = 1.38;
constexpr double MARS_APHELION_AU = 1.67;
constexpr double MARS_YEAR_DAYS = 686.98;

constexpr double YEAR_2023_START_UNIX = 1672531200.0;
// Mars opposition, 2025-01-16 02:32 UTC
constexpr double MARS_OPPOSITION_2025_UNIX = 1736994720.0;
constexpr double EARTH_OBLIQUITY_DEG = 23.43928;

double sun_declination_deg(double p_unix) {
	const DVec3 sun = mars_sun_direction(j2000_seconds_from_unix(p_unix));
	return std::asin(sun.z) * 180.0 / std::numbers::pi;
}

// Allison and McEwen 2000 (Mars24), eq. B-4 to B-5: an
// independent Ls from TT days since J2000.
double ls_mars24(double p_days) {
	const double mean_anomaly = (19.3871 + 0.52402073 * p_days) * DEG;
	const double fictitious_mean_sun = 270.3871 + 0.524038496 * p_days;
	constexpr double AMPLITUDE[7] = { 0.0071, 0.0057, 0.0039, 0.0037, 0.0021, 0.0020, 0.0018 };
	constexpr double PERIOD[7] = { 2.2353, 2.7543, 1.1177, 15.7866, 2.1354, 2.4694, 32.8493 };
	constexpr double PHASE[7] = { 49.409, 168.173, 191.837, 21.736, 15.704, 95.528, 49.095 };
	double perturbations = 0.0;
	for (int i = 0; i < 7; ++i) {
		perturbations += AMPLITUDE[i] * std::cos((0.985626 * p_days / PERIOD[i] + PHASE[i]) * DEG);
	}
	const double m = mean_anomaly;
	const double equation_of_center = (10.691 + 3.0e-7 * p_days) * std::sin(m) + 0.623 * std::sin(2 * m) +
			0.050 * std::sin(3 * m) + 0.005 * std::sin(4 * m) + 0.0005 * std::sin(5 * m) + perturbations;
	return std::fmod(fictitious_mean_sun + equation_of_center + 3600.0, 360.0);
}

// Heliocentric ecliptic longitude of an equatorial vector.
double ecliptic_longitude_deg(const DVec3 &p_equatorial) {
	const double obliquity = EARTH_OBLIQUITY_DEG * std::numbers::pi / 180.0;
	const double y = p_equatorial.y * std::cos(obliquity) + p_equatorial.z * std::sin(obliquity);
	return std::atan2(y, p_equatorial.x) * 180.0 / std::numbers::pi;
}

// The sun, body-fixed, for a central body preset.
DVec3 preset_sun(const BodyPreset &p_body, double p_t) {
	return normalize(inertial_to_body(p_body.rotation, p_t, heliocentric(p_body.orbit, p_t) * -1.0));
}

// The length of a solar day in hours, from the local solar
// time gained at longitude 0 over p_seconds.
double solar_day_hours(const BodyPreset &p_body, double p_t, double p_seconds) {
	const double before = local_solar_time(preset_sun(p_body, p_t), 0.0);
	const double after = local_solar_time(preset_sun(p_body, p_t + p_seconds), 0.0);
	return 24.0 / std::fmod(after - before + 24.0, 24.0);
}

} // namespace

TEST_CASE(mars_seasons) {
	const double equinox = sun_declination_deg(MARS_YEAR_37_LS_0_UNIX);
	const double solstice = sun_declination_deg(MARS_YEAR_37_LS_90_UNIX);
	std::printf("  sun declination: %.2f deg at Ls 0, %.2f deg at Ls 90\n", equinox, solstice);
	CHECK(std::abs(equinox) < 2.0);
	CHECK(std::abs(solstice - MARS_OBLIQUITY_DEG) < 1.0);
}

TEST_CASE(mars_sun_distance_within_its_orbit) {
	const double distance_au = length(mars_heliocentric(j2000_seconds_from_unix(MARS_YEAR_37_LS_0_UNIX)));
	CHECK(distance_au > MARS_PERIHELION_AU);
	CHECK(distance_au < MARS_APHELION_AU);
}

TEST_CASE(solar_longitude_matches_mars24) {
	// one Mars year from 2023, every 5.7 days
	const int samples = 120;
	double worst = 0.0;
	for (int k = 0; k < samples; ++k) {
		const double t = j2000_seconds_from_unix(YEAR_2023_START_UNIX) + k * MARS_YEAR_DAYS / samples * SECONDS_PER_DAY;
		const double error = std::fabs(std::remainder(mars_solar_longitude(t) - ls_mars24(t / SECONDS_PER_DAY), 360.0));
		worst = std::fmax(worst, error);
	}
	std::printf("  Ls against Mars24 over a Mars year: worst %.3f deg\n", worst);
	CHECK(worst < 0.1);
}

TEST_CASE(earth_at_mars_opposition) {
	// At opposition, Earth sits between Mars and the sun: equal
	// ecliptic longitudes. Latitudes differ, by Mars's 1.85 deg
	// tilt, so Earth stays off the solar disc.
	const double t = j2000_seconds_from_unix(MARS_OPPOSITION_2025_UNIX);
	const double earth = ecliptic_longitude_deg(earth_heliocentric(t));
	const double mars = ecliptic_longitude_deg(mars_heliocentric(t));
	const double difference = std::remainder(earth - mars, 360.0);
	std::printf("  heliocentric longitude difference at opposition: %.3f deg\n", difference);
	CHECK(std::fabs(difference) < 0.3);
}

TEST_CASE(mars_local_solar_time_runs_one_sol) {
	const double longitude_deg = 137.8;
	const double t = j2000_seconds_from_unix(1760000000.0);
	const double before = mars_local_solar_time(t, longitude_deg);
	const double after = mars_local_solar_time(t + MARS_SOL / 4.0, longitude_deg);
	const double gained_h = std::fmod(after - before + 24.0, 24.0);
	std::printf("  a quarter sol advances local solar time %.3f h\n", gained_h);
	CHECK(std::abs(gained_h - 6.0) < 0.1);
}

TEST_CASE(mars_noon_is_under_the_sun) {
	const double t = j2000_seconds_from_unix(1760000000.0);
	const DVec3 sun = mars_sun_direction(t);
	const double subsolar_longitude_deg = std::atan2(sun.y, sun.x) * 180.0 / std::numbers::pi;
	CHECK(std::abs(mars_local_solar_time(t, subsolar_longitude_deg) - 12.0) < 1e-6);
}

TEST_CASE(earth_preset_noon_at_greenwich_j2000) {
	// 2000-01-01 12:00 TT: noon less the equation of time,
	// about 3.3 minutes early in early January
	const BodyPreset *earth = find_body_preset("Earth");
	CHECK(earth != nullptr);
	if (!earth) {
		return;
	}
	const double expected_h = 12.0 - 3.3 / 60.0;
	const double local_time_h = local_solar_time(preset_sun(*earth, 0.0), 0.0);
	std::printf("  Greenwich local solar time at J2000: %.4f h\n", local_time_h);
	CHECK(std::abs(local_time_h - expected_h) < 0.05);
}

TEST_CASE(vesta_preset_orbit) {
	const BodyPreset *vesta = find_body_preset("Vesta");
	CHECK(vesta != nullptr);
	if (!vesta) {
		return;
	}
	double nearest_au = 1e9, farthest_au = 0.0;
	for (int k = 0; k < 400; ++k) {
		const double distance_au = length(heliocentric(vesta->orbit, k * SECONDS_PER_DAY * 4.0));
		nearest_au = std::min(nearest_au, distance_au);
		farthest_au = std::max(farthest_au, distance_au);
	}
	std::printf("  Vesta from the sun: %.3f to %.3f AU\n", nearest_au, farthest_au);
	CHECK(nearest_au > 2.14 && nearest_au < 2.16);
	CHECK(farthest_au > 2.56 && farthest_au < 2.58);
}

TEST_CASE(vesta_preset_day) {
	// a solar day a little longer than the 5.3421 h sidereal one
	const double sidereal_day_h = 5.3421;
	const BodyPreset *vesta = find_body_preset("Vesta");
	CHECK(vesta != nullptr);
	if (!vesta) {
		return;
	}
	const double day_h = solar_day_hours(*vesta, 7.8e8, 3600.0);
	std::printf("  Vesta solar day: %.4f h\n", day_h);
	CHECK(day_h > sidereal_day_h && day_h < 5.35);
}

TEST_CASE(moon_opposite_the_sun_at_the_2000_eclipse) {
	// total lunar eclipse, 2000-01-21 04:44 UT (19.697 days
	// after J2000)
	const double t = 19.697 * SECONDS_PER_DAY;
	const DVec3 moon = moon_geocentric(t);
	const DVec3 anti_sun = normalize(earth_heliocentric(t));
	const double separation_deg = std::acos(dot(normalize(moon), anti_sun)) * 180.0 / std::numbers::pi;
	std::printf("  moon %.0f km away, %.2f deg from the anti-sun\n", length(moon) / 1000.0, separation_deg);
	CHECK(separation_deg < 1.0);
	// between perigee and apogee
	CHECK(length(moon) > 356e6 && length(moon) < 407e6);
}

TEST_CASE(preset_rotations_round_trip) {
	const DVec3 dir = normalize(DVec3(0.3, -0.7, 0.4));
	const double t = 5.0e8;
	for (const BodyPreset &body : body_presets()) {
		const DVec3 back = inertial_to_body(body.rotation, t, body_to_inertial(body.rotation, t, dir));
		CHECK(length(back - dir) < 1e-12);
	}
}

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cmath>
#include <cstdio>

#include "regolith/air.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

constexpr double EARTH_GROUND_RADIUS_M = 6360000.0;
constexpr double MARS_GROUND_RADIUS_M = 3389500.0;
constexpr double MARS_DUST_TAU = 0.5;

// Nearly all of the Rayleigh air lies below this height.
constexpr double RAYLEIGH_TOP_M = 100000.0;
constexpr double TRANSMITTANCE_TOLERANCE = 1e-3;

// View directions, as the cosine of the zenith angle.
constexpr double STRAIGHT_UP = 1.0;
constexpr double LOOKING_DOWN = -0.5;
constexpr double GRAZING = 0.05; // about 3 deg over the horizon

} // namespace

TEST_CASE(earth_zenith_transmittance_matches_layer_depths) {
	const AirModel air = earth_air(EARTH_GROUND_RADIUS_M);
	const RGB transmittance = air_transmittance(air, air.bottom, STRAIGHT_UP);
	for (int channel = 0; channel < 3; ++channel) {
		const size_t c = size_t(channel);
		// Straight up, a layer's optical depth is its coefficient
		// times its thickness; ozone's tent is width times peak.
		const double rayleigh_depth =
				air.rayleigh[c] * air.rayleigh_height * (1.0 - std::exp(-RAYLEIGH_TOP_M / air.rayleigh_height));
		const double mie_depth = (air.mie_scattering[c] + air.mie_absorption[c]) * air.mie_height;
		const double ozone_depth = air.ozone[c] * air.ozone_width;
		const double expected = std::exp(-(rayleigh_depth + mie_depth + ozone_depth));
		std::printf("  channel %d: %.5f, expected %.5f\n", channel, transmittance[c], expected);
		CHECK(std::abs(transmittance[c] - expected) < TRANSMITTANCE_TOLERANCE);
	}
}

// Mars's air for the profile tests: CO2 under Mars's gravity, the
// climate's 640 Pa annual mean at a 230 K afternoon ground
constexpr double MARS_R_GAS = 191.8;      // J/(kg K)
constexpr double MARS_GRAVITY = 3.711;    // m/s2
constexpr double GROUND_PA = 640.0;
constexpr double GROUND_K = 230.0;

// Hydrostatic pressure by small steps up the same temperature profile:
// dp/dh = -p g / (R T(h)). An independent check of the closed form.
static double pressure_by_steps(double p_height_m) {
	constexpr int STEPS = 200000;
	const double dh = p_height_m / STEPS;
	double p = GROUND_PA;
	for (int i = 0; i < STEPS; ++i) {
		const double t = air_temperature_at_height(GROUND_K, (i + 0.5) * dh);
		p *= std::exp(-MARS_GRAVITY * dh / (MARS_R_GAS * t));
	}
	return p;
}

TEST_CASE(air_cools_at_nasa_glenns_mars_lapse_rates) {
	// 0.998 K/km to 7 km, 2.22 K/km above (NASA Glenn's Mars model)
	const double at_7km = air_temperature_at_height(GROUND_K, 7000.0);
	const double at_20km = air_temperature_at_height(GROUND_K, 20000.0);
	const double expected_7km = GROUND_K - 0.998 * 7.0;
	const double expected_20km = expected_7km - 2.22 * 13.0;
	std::printf("  7 km %.3f K (expected %.3f), 20 km %.3f K (expected %.3f)\n", at_7km, expected_7km, at_20km, expected_20km);
	CHECK(std::abs(at_7km - expected_7km) < 1e-9);
	CHECK(std::abs(at_20km - expected_20km) < 1e-9);
	CHECK(air_temperature_at_height(GROUND_K, -50.0) == GROUND_K);
}

TEST_CASE(air_no_colder_than_the_floor) {
	// 230 K reaches the 140 K floor near 44 km
	const double at_80km = air_temperature_at_height(GROUND_K, 80000.0);
	std::printf("  80 km %.1f K (floor %.1f K)\n", at_80km, MARS_AIR_FLOOR_K);
	CHECK(at_80km == MARS_AIR_FLOOR_K);
}

TEST_CASE(pressure_matches_the_profile_integrated_by_steps) {
	// through the low layer, the high layer, and past the floor
	for (const double height_m : { 0.0, 3000.0, 7000.0, 20000.0, 60000.0, 100000.0 }) {
		const double closed = pressure_at_height(GROUND_PA, height_m, GROUND_K, MARS_R_GAS, MARS_GRAVITY);
		const double stepped = height_m > 0.0 ? pressure_by_steps(height_m) : GROUND_PA;
		const double error = std::abs(closed - stepped) / stepped;
		std::printf("  %6.0f m: %.6g Pa, by steps %.6g Pa, relative error %.1e\n", height_m, closed, stepped, error);
		CHECK(error < 1e-6);
	}
}

TEST_CASE(speed_of_sound_matches_perseverances_240_ms) {
	// Perseverance heard about 240 m/s near the ground (Maurice et al.
	// 2022); daytime air there is about 230 to 245 K
	const double at_235k = speed_of_sound(235.0, MARS_R_GAS);
	const double expected = std::sqrt(9.0 / 7.0 * MARS_R_GAS * 235.0);
	std::printf("  235 K: %.1f m/s (gamma 9/7: %.1f)\n", at_235k, expected);
	CHECK(std::abs(at_235k - expected) < 1e-9);
	CHECK(std::abs(at_235k - 240.0) < 5.0);
}

TEST_CASE(air_transmittance_is_zero_into_the_ground) {
	const AirModel air = mars_air(MARS_GROUND_RADIUS_M, MARS_DUST_TAU);
	const RGB into_ground = air_transmittance(air, air.bottom + 1000.0, LOOKING_DOWN);
	CHECK(into_ground[0] == 0.0);
	CHECK(into_ground[2] == 0.0);
}

TEST_CASE(grazing_sight_line_crosses_more_air_than_zenith) {
	const AirModel air = mars_air(MARS_GROUND_RADIUS_M, MARS_DUST_TAU);
	const RGB zenith = air_transmittance(air, air.bottom + 1.0, STRAIGHT_UP);
	const RGB grazing = air_transmittance(air, air.bottom + 1.0, GRAZING);
	std::printf("  Mars, tau 0.5: zenith %.3f %.3f %.3f, 3 deg %.3f %.3f %.3f\n", zenith[0], zenith[1], zenith[2],
			grazing[0], grazing[1], grazing[2]);
	CHECK(grazing[1] < zenith[1] * 0.5);
}

TEST_CASE(dust_reddens_a_low_sun) {
	// Mars dust absorbs blue, so light low over the horizon
	// keeps more red than blue.
	const AirModel air = mars_air(MARS_GROUND_RADIUS_M, MARS_DUST_TAU);
	const RGB grazing = air_transmittance(air, air.bottom + 1.0, GRAZING);
	CHECK(grazing[0] > grazing[2]);
}

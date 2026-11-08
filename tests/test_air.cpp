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

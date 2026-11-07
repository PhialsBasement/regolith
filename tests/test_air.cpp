// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cmath>
#include <cstdio>

#include "regolith/air.hpp"
#include "test.hpp"

using namespace regolith;

TEST_CASE(earth_zenith_transmittance_matches_layer_depths) {
	const AirModel a = earth_air(6360000.0);
	const RGB t = air_transmittance(a, a.bottom, 1.0);
	// straight up: each layer's depth is its coefficient times its
	// scale height (nearly all below 100 km); ozone's tent has
	// area width times peak
	for (int c = 0; c < 3; ++c) {
		const size_t k = size_t(c);
		const double depth = a.rayleigh[k] * a.rayleigh_height * (1.0 - std::exp(-100000.0 / a.rayleigh_height)) +
				(a.mie_scattering[k] + a.mie_absorption[k]) * a.mie_height + a.ozone[k] * a.ozone_width;
		std::printf("  channel %d: %.5f, expected %.5f\n", c, t[k], std::exp(-depth));
		CHECK(std::abs(t[k] - std::exp(-depth)) < 1e-3);
	}
}

TEST_CASE(air_transmittance_is_zero_into_the_ground) {
	const AirModel a = mars_air(3389500.0, 0.5);
	const RGB down = air_transmittance(a, a.bottom + 1000.0, -0.5);
	CHECK(down[0] == 0.0 && down[2] == 0.0);
	// grazing the horizon from the ground: far more air than up
	const RGB up = air_transmittance(a, a.bottom + 1.0, 1.0);
	const RGB low = air_transmittance(a, a.bottom + 1.0, 0.05);
	std::printf("  Mars, tau 0.5: zenith %.3f %.3f %.3f, 3 deg %.3f %.3f %.3f\n", up[0], up[1], up[2], low[0], low[1],
			low[2]);
	CHECK(low[1] < up[1] * 0.5);
	// dust absorbs blue: the low sun is redder than it is blue
	CHECK(low[0] > low[2]);
}

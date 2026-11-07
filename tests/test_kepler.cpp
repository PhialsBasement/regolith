// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cmath>
#include <numbers>

#include "regolith/kepler.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

constexpr double GM = 4.282837e13;

} // namespace

TEST_CASE(kepler_circular_quarter) {
	const double r = 3789500.0;
	const double v = std::sqrt(GM / r);
	const double period = 2.0 * std::numbers::pi * r / v;
	DVec3 rr, vv;
	kepler_propagate({ r, 0, 0 }, { 0, v, 0 }, GM, period / 4.0, rr, vv);
	CHECK(length(rr - DVec3(0, r, 0)) < 1e-3);
	CHECK(length(vv - DVec3(-v, 0, 0)) < 1e-6);
	kepler_propagate({ r, 0, 0 }, { 0, v, 0 }, GM, period * 10.0, rr, vv);
	std::printf("  circular orbit after 10 periods: %.2e m off\n", length(rr - DVec3(r, 0, 0)));
	CHECK(length(rr - DVec3(r, 0, 0)) < 1e-2);
}

TEST_CASE(kepler_round_trip_all_conics) {
	int checked = 0;
	for (double vs : { 0.7, 1.2, 1.41421356, 1.9 }) {
		const DVec3 r0(3600000.0, 400000.0, -200000.0);
		const DVec3 v0 = DVec3(-300.0, 3000.0, 900.0) * vs;
		DVec3 r1, v1, r2, v2;
		kepler_propagate(r0, v0, GM, 4000.0, r1, v1);
		kepler_propagate(r1, v1, GM, -4000.0, r2, v2);
		CHECK(length(r2 - r0) < 1e-2);
		CHECK(length(v2 - v0) < 1e-5);
		// Energy and angular momentum are invariants.
		const double e0 = 0.5 * dot(v0, v0) - GM / length(r0);
		const double e1 = 0.5 * dot(v1, v1) - GM / length(r1);
		CHECK(std::fabs(e1 - e0) < 1e-6 * std::fabs(e0));
		CHECK(length(cross(r1, v1) - cross(r0, v0)) < 1e-6 * length(cross(r0, v0)));
		++checked;
	}
	std::printf("  %d conics propagated forward and back\n", checked);
}

TEST_CASE(orbit_shape_of_circle) {
	const double r = 3789500.0;
	const OrbitShape o = orbit_shape({ r, 0, 0 }, { 0, std::sqrt(GM / r), 0 }, GM, { 0, 0, 1 });
	CHECK(std::fabs(o.a - r) < 1e-3);
	CHECK(o.e < 1e-12);
	CHECK(std::fabs(o.periapsis - r) < 1e-3 && std::fabs(o.apoapsis - r) < 1e-3);
	CHECK(o.inclination < 1e-12);
}

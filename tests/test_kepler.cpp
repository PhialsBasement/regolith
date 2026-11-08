// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <cmath>
#include <numbers>

#include "regolith/kepler.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

constexpr double MARS_GM = 4.282837e13; // m^3/s^2
constexpr double SUN_GM = 1.32712440018e20; // m^3/s^2
constexpr double ORBIT_RADIUS_M = 3789500.0; // 400 km over Mars
constexpr int STEPS_PER_ARC = 1000;

double circular_speed(double p_gm, double p_radius) {
	return std::sqrt(p_gm / p_radius);
}

// Energy per unit mass: kinetic plus potential.
double specific_energy(const DVec3 &p_position, const DVec3 &p_velocity, double p_gm) {
	return 0.5 * dot(p_velocity, p_velocity) - p_gm / length(p_position);
}

DVec3 angular_momentum(const DVec3 &p_position, const DVec3 &p_velocity) {
	return cross(p_position, p_velocity);
}

struct Arc {
	DVec3 start_position;
	DVec3 start_velocity;
	double gm;
	double duration_s;
};

// The same arc in many small steps, to compare with one step.
void propagate_in_steps(const Arc &p_arc, DVec3 &r_position, DVec3 &r_velocity) {
	r_position = p_arc.start_position;
	r_velocity = p_arc.start_velocity;
	for (int step = 0; step < STEPS_PER_ARC; ++step) {
		DVec3 next_position, next_velocity;
		kepler_propagate(r_position, r_velocity, p_arc.gm, p_arc.duration_s / double(STEPS_PER_ARC), next_position,
				next_velocity);
		r_position = next_position;
		r_velocity = next_velocity;
	}
}

} // namespace

TEST_CASE(kepler_circular_quarter_turn) {
	const double speed = circular_speed(MARS_GM, ORBIT_RADIUS_M);
	const double period = 2.0 * std::numbers::pi * ORBIT_RADIUS_M / speed;
	DVec3 position, velocity;
	kepler_propagate({ ORBIT_RADIUS_M, 0, 0 }, { 0, speed, 0 }, MARS_GM, period / 4.0, position, velocity);
	CHECK(length(position - DVec3(0, ORBIT_RADIUS_M, 0)) < 1e-3);
	CHECK(length(velocity - DVec3(-speed, 0, 0)) < 1e-6);
}

TEST_CASE(kepler_circular_ten_periods) {
	const double speed = circular_speed(MARS_GM, ORBIT_RADIUS_M);
	const double period = 2.0 * std::numbers::pi * ORBIT_RADIUS_M / speed;
	DVec3 position, velocity;
	kepler_propagate({ ORBIT_RADIUS_M, 0, 0 }, { 0, speed, 0 }, MARS_GM, period * 10.0, position, velocity);
	const double miss_m = length(position - DVec3(ORBIT_RADIUS_M, 0, 0));
	std::printf("  circular orbit after 10 periods: %.2e m off\n", miss_m);
	CHECK(miss_m < 1e-2);
}

TEST_CASE(kepler_round_trip_all_conics) {
	// from a low ellipse (0.7) to a hyperbola (1.9)
	const double speed_scales[] = { 0.7, 1.2, 1.41421356, 1.9 };
	const double duration_s = 4000.0;
	int checked = 0;
	for (double speed_scale : speed_scales) {
		const DVec3 start_position(3600000.0, 400000.0, -200000.0);
		const DVec3 start_velocity = DVec3(-300.0, 3000.0, 900.0) * speed_scale;
		DVec3 position, velocity, back_position, back_velocity;
		kepler_propagate(start_position, start_velocity, MARS_GM, duration_s, position, velocity);
		kepler_propagate(position, velocity, MARS_GM, -duration_s, back_position, back_velocity);
		CHECK(length(back_position - start_position) < 1e-2);
		CHECK(length(back_velocity - start_velocity) < 1e-5);

		// energy and angular momentum are conserved
		const double start_energy = specific_energy(start_position, start_velocity, MARS_GM);
		const double energy = specific_energy(position, velocity, MARS_GM);
		const DVec3 start_momentum = angular_momentum(start_position, start_velocity);
		CHECK(std::fabs(energy - start_energy) < 1e-6 * std::fabs(start_energy));
		CHECK(length(angular_momentum(position, velocity) - start_momentum) < 1e-6 * length(start_momentum));
		++checked;
	}
	std::printf("  %d conics propagated forward and back\n", checked);
}

TEST_CASE(kepler_long_arcs) {
	const double escape_plus_3_kms = std::sqrt(9.0e6 + 2.0 * MARS_GM / ORBIT_RADIUS_M);
	const Arc arcs[] = {
		// Mars escape at 3 km/s over escape speed, out past
		// the sphere of influence, forward and back
		{ { ORBIT_RADIUS_M, 0, 0 }, { 0, escape_plus_3_kms, 0 }, MARS_GM, 1.0e6 },
		{ { ORBIT_RADIUS_M, 0, 0 }, { 0, escape_plus_3_kms, 0 }, MARS_GM, -1.0e6 },
		// a Sun-scale hyperbola
		{ { 2.0e11, 1.0e10, 0 }, { -5000.0, 40000.0, 3000.0 }, SUN_GM, 3.0e8 },
		// an ellipse over many turns
		{ { ORBIT_RADIUS_M, 0, 0 }, { 0, circular_speed(MARS_GM, ORBIT_RADIUS_M) * 1.1, 0 }, MARS_GM, 2.0e6 },
	};
	int checked = 0;
	for (const Arc &arc : arcs) {
		DVec3 position, velocity;
		kepler_propagate(arc.start_position, arc.start_velocity, arc.gm, arc.duration_s, position, velocity);
		DVec3 stepped_position, stepped_velocity;
		propagate_in_steps(arc, stepped_position, stepped_velocity);
		const double scale = std::fmax(length(position), length(arc.start_position));
		CHECK(length(position - stepped_position) < 1e-7 * scale);

		const double start_energy = specific_energy(arc.start_position, arc.start_velocity, arc.gm);
		const double energy = specific_energy(position, velocity, arc.gm);
		const DVec3 start_momentum = angular_momentum(arc.start_position, arc.start_velocity);
		CHECK(std::fabs(energy - start_energy) < 1e-9 * std::fabs(start_energy));
		CHECK(length(angular_momentum(position, velocity) - start_momentum) < 1e-9 * length(start_momentum));
		++checked;
	}
	std::printf("  %d long arcs: one step agrees with 1000\n", checked);
}

TEST_CASE(orbit_shape_of_circle) {
	const double speed = circular_speed(MARS_GM, ORBIT_RADIUS_M);
	const OrbitShape shape = orbit_shape({ ORBIT_RADIUS_M, 0, 0 }, { 0, speed, 0 }, MARS_GM, { 0, 0, 1 });
	CHECK(std::fabs(shape.a - ORBIT_RADIUS_M) < 1e-3);
	CHECK(shape.e < 1e-12);
	CHECK(std::fabs(shape.periapsis - ORBIT_RADIUS_M) < 1e-3);
	CHECK(std::fabs(shape.apoapsis - ORBIT_RADIUS_M) < 1e-3);
	CHECK(shape.inclination < 1e-12);
}

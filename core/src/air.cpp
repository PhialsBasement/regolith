// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith/air.hpp"

#include <algorithm>
#include <cmath>

namespace regolith {

namespace {

// distance along (r, mu) to a sphere of radius p_radius, -1 if
// the ray misses it (outward crossing for the outer shell)
double to_sphere(double p_r, double p_mu, double p_radius, bool p_far) {
	const double disc = p_r * p_r * (p_mu * p_mu - 1.0) + p_radius * p_radius;
	if (disc < 0.0) {
		return -1.0;
	}
	const double s = std::sqrt(disc);
	const double t = p_far ? -p_r * p_mu + s : -p_r * p_mu - s;
	return t;
}

} // namespace

AirModel earth_air(double p_bottom) {
	AirModel a;
	a.bottom = p_bottom;
	a.top = p_bottom + 100000.0;
	a.rayleigh = { 5.802e-6, 13.558e-6, 33.1e-6 };
	a.rayleigh_height = 8000.0;
	a.mie_scattering = { 3.996e-6, 3.996e-6, 3.996e-6 };
	a.mie_absorption = { 4.40e-6, 4.40e-6, 4.40e-6 };
	a.mie_height = 1200.0;
	a.mie_g = { 0.8, 0.8, 0.8 };
	a.ozone = { 0.650e-6, 1.881e-6, 0.085e-6 };
	a.ozone_center = 25000.0;
	a.ozone_width = 15000.0;
	a.ground_albedo = { 0.3, 0.3, 0.3 };
	return a;
}

AirModel mars_air(double p_bottom, double p_dust_tau) {
	AirModel a;
	a.bottom = p_bottom;
	a.top = p_bottom + 100000.0;
	// Earth's air scaled to Mars: 0.83% the number density (610
	// Pa, 210 K), CO2 scattering about 2.4 times as strongly
	a.rayleigh = { 0.115e-6, 0.269e-6, 0.656e-6 };
	a.rayleigh_height = 11100.0;
	// the dust: well mixed, its vertical optical depth the tau
	const double height = 11100.0;
	const double ext = std::max(p_dust_tau, 0.0) / height;
	const RGB albedo = { 0.975, 0.93, 0.80 };
	for (int c = 0; c < 3; ++c) {
		a.mie_scattering[size_t(c)] = ext * albedo[size_t(c)];
		a.mie_absorption[size_t(c)] = ext * (1.0 - albedo[size_t(c)]);
	}
	a.mie_height = height;
	// blue diffracts more narrowly forward: the blue aureole
	a.mie_g = { 0.64, 0.70, 0.78 };
	a.ozone = { 0.0, 0.0, 0.0 };
	a.ground_albedo = { 0.30, 0.20, 0.13 };
	return a;
}

RGB air_transmittance(const AirModel &p_air, double p_r, double p_mu) {
	const double r = std::clamp(p_r, p_air.bottom, p_air.top);
	const double ground = to_sphere(r, p_mu, p_air.bottom, false);
	if (ground > 0.0) {
		return { 0.0, 0.0, 0.0 };
	}
	const double len = std::max(to_sphere(r, p_mu, p_air.top, true), 0.0);
	// fine enough for a 1.2 km haze layer to integrate to 0.5%
	constexpr int STEPS = 256;
	RGB depth{ 0.0, 0.0, 0.0 };
	const double dt = len / STEPS;
	const double ref = p_air.datum > 0.0 ? p_air.datum : p_air.bottom;
	for (int i = 0; i < STEPS; ++i) {
		const double t = (i + 0.5) * dt;
		const double h = std::sqrt(r * r + t * t + 2.0 * r * p_mu * t) - ref;
		const double dr = std::exp(-h / p_air.rayleigh_height);
		const double dm = std::exp(-h / p_air.mie_height);
		const double dozone = std::max(0.0, 1.0 - std::abs(h - p_air.ozone_center) / p_air.ozone_width);
		for (int c = 0; c < 3; ++c) {
			const size_t k = size_t(c);
			depth[k] += (p_air.rayleigh[k] * dr + (p_air.mie_scattering[k] + p_air.mie_absorption[k]) * dm +
								p_air.ozone[k] * dozone) *
					dt;
		}
	}
	return { std::exp(-depth[0]), std::exp(-depth[1]), std::exp(-depth[2]) };
}

double air_temperature_at_height(double p_ground_k, double p_height_m) {
	const double h = std::max(p_height_m, 0.0);
	const double low = std::min(h, MARS_LAPSE_TURN_M);
	const double high = h - low;
	return std::max(p_ground_k - MARS_LAPSE_LOW * low - MARS_LAPSE_HIGH * high, MARS_AIR_FLOOR_K);
}

namespace {

// Pressure ratio across a layer cooling at p_lapse (K/m) from p_t0 to
// p_t1 (K); isothermal when p_lapse is 0, its thickness p_dh (m).
double layer_ratio(double p_t0, double p_t1, double p_lapse, double p_dh, double p_r_gas, double p_gravity) {
	if (p_lapse == 0.0 || p_t0 == p_t1) {
		return std::exp(-p_gravity * p_dh / (p_r_gas * p_t0));
	}
	return std::pow(p_t1 / p_t0, p_gravity / (p_r_gas * p_lapse));
}

} // namespace

double pressure_at_height(double p_ground_pa, double p_height_m, double p_ground_k, double p_r_gas,
		double p_gravity) {
	const double h = std::max(p_height_m, 0.0);
	const double ground_k = std::max(p_ground_k, MARS_AIR_FLOOR_K);
	// the profile's layers: low lapse, high lapse, then the floor
	const double edges[] = { 0.0, MARS_LAPSE_TURN_M, INFINITY };
	const double lapses[] = { MARS_LAPSE_LOW, MARS_LAPSE_HIGH };
	double p = p_ground_pa;
	for (int k = 0; k < 2 && h > edges[k]; ++k) {
		const double top = std::min(h, edges[k + 1]);
		const double t0 = air_temperature_at_height(ground_k, edges[k]);
		const double t1 = air_temperature_at_height(ground_k, top);
		// where this layer reaches the floor, the rest of it is isothermal
		const double to_floor = (t0 - MARS_AIR_FLOOR_K) / lapses[k];
		const double cooling = std::min(top - edges[k], to_floor);
		p *= layer_ratio(t0, t0 - lapses[k] * cooling, lapses[k], cooling, p_r_gas, p_gravity);
		p *= layer_ratio(t1, t1, 0.0, top - edges[k] - cooling, p_r_gas, p_gravity);
	}
	return p;
}

double speed_of_sound(double p_temperature_k, double p_r_gas) {
	return std::sqrt(MARS_GAMMA * p_r_gas * p_temperature_k);
}

} // namespace regolith

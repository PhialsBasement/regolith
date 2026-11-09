// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <array>

namespace regolith {

using RGB = std::array<double, 3>;

// A body's air as Hillaire 2020 models it: gas (Rayleigh) and
// aerosol (Mie) scattering falling off exponentially with
// height, and an absorbing layer (ozone). Coefficients are per
// metre at the bottom of the atmosphere, red green blue.
struct AirModel {
	double bottom = 0.0; // m, the ground's radius
	double top = 0.0; // m, where the air ends
	// m, where the densities below hold; 0: the bottom. Lower
	// ground (a basin under the datum) sits in denser air.
	double datum = 0.0;
	RGB rayleigh{};
	double rayleigh_height = 8000.0;
	RGB mie_scattering{};
	RGB mie_absorption{};
	double mie_height = 1200.0;
	RGB mie_g{ 0.8, 0.8, 0.8 }; // forward phase, per colour
	RGB ozone{}; // absorption at the layer's peak
	double ozone_center = 25000.0;
	double ozone_width = 15000.0; // half width of the tent
	RGB ground_albedo{ 0.3, 0.3, 0.3 };
};

// Earth: Bruneton 2017 and Hillaire 2020's reference values.
AirModel earth_air(double p_bottom);
// Mars: thin CO2 and suspended dust of visible optical depth
// p_dust_tau, whose single-scattering albedo falls toward the
// blue (Wolff et al. 2009): the butterscotch sky and blue dusk.
AirModel mars_air(double p_bottom, double p_dust_tau);

// The air's pressure p_height_m over the ground (Pa), from p_ground_pa
// there: an isothermal column at p_temperature_k, falling by e every
// scale height p_r_gas * p_temperature_k / p_gravity (J/(kg K), m/s2).
double pressure_at_height(double p_ground_pa, double p_height_m, double p_temperature_k, double p_r_gas,
		double p_gravity);

// Transmittance from radius p_r along a ray at cosine p_mu to
// the local vertical, out of the air; zero if it meets the
// ground. The CPU twin of the transmittance LUT.
RGB air_transmittance(const AirModel &p_air, double p_r, double p_mu);

} // namespace regolith

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "regolith/vec.hpp"

namespace regolith {

// The body, its air, and the grid the fields live on.
// Positions are "planet pixels": px_per_turn around the
// equator, half as many pole to pole.
struct ClimateConfig {
	double radius = 106496.0 * 200.0 / 6.283185307179586; // m
	double gravity = 3.711; // m/s2
	double sol_s = 88775.0; // s
	double obliquity_deg = 25.19;
	double eccentricity = 0.0934;
	double perihelion_ls = 251.0; // deg
	double solar_flux = 586.0; // W/m2 at mean distance
	double scale_height = 10800.0; // m
	double datum_pressure = 640.0; // Pa, the annual mean
	double r_gas = 191.8; // J/kg/K
	int gw = 180;
	int gh = 90;
	double px_per_turn = 106496.0;
};

// Game weather: every field is a closed-form function of the
// season, the hour, the terrain and seeded storms. No state
// is integrated, so any time can be evaluated directly.
class Climate {
public:
	static constexpr double TAU_FLOOR = 0.10;
	static constexpr double TAU_PER_DUST = 2.6;
	static constexpr double PACK_TAU_MAX = 3.0;
	static constexpr double PACK_DUST_MAX = 2.0;
	// cloud levels: edges in metres over the ground
	static constexpr int NL = 8;
	static constexpr int NL_LOW = 4; // below 14 km
	static constexpr double LEVEL_M[NL + 1] = { 0.0, 2000.0, 5000.0, 9000.0, 14000.0, 20000.0, 27000.0, 35000.0,
		45000.0 };

	using Field = std::vector<float>;

	explicit Climate(const ClimateConfig &p_config = ClimateConfig());
	const ClimateConfig &config() const { return cfg_; }

	// p_topo: height (m) at cell centres; p_lum: albedo proxy
	// 0..1 per cell; either may be null. p_clim: the dust
	// climatology rows (24 bins of 7 values), or empty.
	void create(const float *p_topo, const float *p_lum, const std::vector<std::array<double, 7>> &p_clim);
	// Evaluate every field at TT seconds since J2000.
	void update(double p_time);
	// dust raised around a planet pixel, fading over a sol
	void inject_dust(double p_px, double p_py, double p_time);

	double sample(const Field &p_f, double p_px, double p_py) const;
	void fold_px(double &r_px, double &r_py) const;

	// RGBA8 packs the renderer decodes: R low cloud tau, G high
	// (both over PACK_TAU_MAX), A dust over PACK_DUST_MAX
	void pack_global(std::vector<uint8_t> &r_out) const;
	// R CO2 ice / 400, G water ice / 1 (kg/m2), B ground T and
	// A air T as (T - 140) / 160
	void pack_surface(std::vector<uint8_t> &r_out) const;

	// dust is (tau - TAU_FLOOR) / TAU_PER_DUST; winds in m/s,
	// u east, v south (the grid is y-down); su/sv at the
	// surface, lu/lv aloft
	Field dust, su, sv, lu, lv, ps, ts, tair, co2, frost, wv;
	std::array<Field, NL> tau, ice;
	Field tau_low, tau_high, topo;
	double ls = 0.0;

	const int GW, GH, N;
	const double W_PX, H_PX, GCELL_PX;

private:
	struct Storm {
		DVec3 centre;
		double radius, peak, env;
	};
	struct Plume {
		DVec3 centre;
		double t0;
	};

	void storms_at(double p_time, std::vector<Storm> &r_out) const;
	double clim_at(int p_col, double p_ls) const;

	ClimateConfig cfg_;
	std::vector<DVec3> dir_; // cell unit vectors
	std::vector<double> lat_, lon_;
	Field alb_, slope_e_, slope_n_;
	std::vector<std::array<double, 7>> clim_;
	std::vector<Plume> plumes_;
};

} // namespace regolith

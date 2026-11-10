// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "regolith/air.hpp"
#include "regolith/climate_model.hpp"
#include "regolith/ephemeris.hpp"
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
	LapseProfile lapse; // the air cooling with height
	double gamma = 9.0 / 7.0; // adiabatic index: CO2 with its bending mode relaxed (Maurice et al. 2022)
	double mean_air_k = 210.0; // the air before the fields are worked out
	// the body's path and spin, for its seasons and hours
	OrbitElements orbit = MARS_ORBIT;
	Rotation rotation = MARS_ROTATION;
	ClimateModel model; // its dust, winds, caps, water and clouds
	int gw = 180;
	int gh = 90;
	double px_per_turn = 106496.0;
};

// Game weather: every field is a closed-form function of the
// season, the hour, the terrain and seeded storms. No state
// is integrated, so any time can be evaluated directly.
class Climate {
public:
	static constexpr double PACK_TAU_MAX = 3.0;
	static constexpr double PACK_DUST_MAX = 2.0;
	// cloud levels (edges: the model's level_edges_m); the lower NL_LOW are the low clouds
	static constexpr int NL = CLIMATE_LEVELS;
	static constexpr int NL_LOW = 4;

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
	// R cap ice over the model's cap_pack_kg_m2, G water ice / 1 (kg/m2),
	// B ground T and A air T over the model's ground_min_k to ground_max_k
	void pack_surface(std::vector<uint8_t> &r_out) const;

	// dust is (tau - dust_tau_floor) / tau_per_dust; winds in m/s,
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

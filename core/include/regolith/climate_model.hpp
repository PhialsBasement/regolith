// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <array>
#include <vector>

namespace regolith {

constexpr int CLIMATE_LEVELS = 8; // cloud levels the fields are kept on
using LevelWeights = std::array<double, CLIMATE_LEVELS>; // how a cloud kind spreads over the levels

// A stretch of the year, Ls degrees, both ends excluded; it wraps past
// 360 when to < from.
struct LsWindow {
	double from = 0.0, to = 360.0;
};

// A smooth rise from rise_from to rise_to, then a fall from fall_from to
// fall_to (Ls degrees).
struct LsRamp {
	double rise_from = 0.0, rise_to = 0.0, fall_from = 360.0, fall_to = 360.0;
};

// A band of a seasonal cap: at lat_deg the frost arrives at Ls ls_on,
// peaks at ls_peak (kg_m2) and is gone by ls_off (Ls past 360 runs on
// into the next year).
struct CapBand {
	double lat_deg, ls_on, ls_peak, ls_off, kg_m2;
};

// Where regional dust storms rise, and in which hemisphere's season.
struct StormSite {
	double lat_deg, lon_deg;
	bool south;
};

struct Place {
	double lat_deg, lon_deg;
};

// A basin that fills with fog and haze: its centre, radius and share.
struct CloudBasin {
	double lat_deg, lon_deg, radius_m, weight;
};

// What the climate knows about its body beyond its orbit, air and grid:
// where and when dust rises, how the winds blow, how warm the ground
// gets, the seasonal caps, the water and the clouds. Defaults: Mars
// (mors's tables and fits). Lengths m, speeds m/s, temperatures K,
// columns kg/m2, seasons Ls degrees, hours local solar time.
struct ClimateModel {
	// dust: tau = dust_tau_floor + tau_per_dust * dust
	double dust_tau_floor = 0.10;
	double tau_per_dust = 2.6;
	double dust_tau_default = 0.3; // the background without a dust climatology
	double tau_unknown = 0.35; // before the fields are worked out
	double dust_patch_m = 400e3;
	double plume_tau = 1.6; // dust a game raises (inject_dust)
	double plume_radius_m = 120e3;
	double plume_decay_sols = 0.4;
	double plume_life_sols = 3.0;

	// regional storms
	std::vector<StormSite> storm_sites = { { -42.0, 70.0, true }, { -50.0, -43.0, true }, { -26.0, -90.0, true },
		{ -45.0, 0.0, true }, { 25.0, -40.0, false }, { 45.0, 110.0, false }, { 12.0, 87.0, false },
		{ 55.0, -30.0, false } };
	LsWindow storm_season_south = { 140.0, 360.0 };
	LsWindow storm_season_north = { 180.0, 40.0 };
	LsRamp storm_season_default = { 180.0, 230.0, 270.0, 340.0 }; // storm activity without a climatology
	double storm_area_default = 2.0; // its peak, 1e6 km2 of tau > 2
	int storms_per_sol = 2;
	int storm_life_max_sols = 18;
	double storm_radius_m = 250e3;
	double storm_radius_spread_m = 900e3;
	double storm_radius_max_m = 3.0e6;
	double storm_peak_tau = 0.5;
	double storm_peak_spread_tau = 1.8;
	double storm_off_season = 0.35; // share of the peak out of its season
	double storm_drift_ms = 5.0;
	double storm_swirl_ms = 14.0;
	double storm_gust_ms = 6.0;

	// the surface pressure through the year (shares of the datum pressure)
	double pressure_annual = 0.075;
	double pressure_annual_ls = 255.0;
	double pressure_semiannual = 0.04;
	double pressure_semiannual_ls = 55.0;

	// winds
	double trade_ms = 5.0; // easterly, about the equator
	double trade_width_deg = 25.0;
	double jet_ms = 3.0; // westerly, at jet_lat_deg
	double jet_winter_ms = 9.0; // added in each hemisphere's winter
	double jet_lat_deg = 55.0;
	double jet_width_deg = 15.0;
	double hadley_ms = 4.0; // the Hadley cell's surface branch, toward summer
	double hadley_width_deg = 30.0;
	double tide_east_ms = 3.0;
	double tide_north_ms = 2.0;
	double slope_night_ms = 9.0; // down the slopes at night
	double slope_day_ms = 5.0; // up them by day
	double gust_ms = 4.0;

	// temperatures
	double albedo_min = 0.11; // the albedo map's darkest 5% to brightest 5%
	double albedo_range = 0.20;
	double absorbed = 0.88; // share of the sunlight the ground takes up
	double emissivity = 0.85 * 0.97;
	double mean_ground_min_k = 145.0;
	double mean_ground_max_k = 265.0;
	double ground_min_k = 140.0;
	double ground_max_k = 300.0;
	double air_min_k = 140.0;
	double air_max_k = 290.0;
	double diurnal_k = 38.0; // the ground's day-night swing on a clear day
	double diurnal_dust_damping = 0.35; // per unit of tau
	double ground_peak_hour = 13.5;
	double air_peak_hour = 15.0;
	double air_below_ground_k = 8.0;
	double dust_warming_k = 6.0; // the air by day, per unit of tau over dust_tau_default

	// seasonal caps and frost: the condensing gas's frost point is
	// frost_a_k / (frost_b - ln(p / Pa))
	std::vector<CapBand> cap_north = { { 60.0, 230.0, 322.5, 390.0, 90.0 }, { 67.5, 215.0, 335.0, 417.0, 250.0 },
		{ 75.0, 200.0, 347.0, 442.0, 400.0 }, { 82.5, 185.0, 357.0, 455.0, 550.0 }, { 90.0, 170.0, 367.5, 460.0, 700.0 } };
	std::vector<CapBand> cap_south = { { 50.0, 60.0, 100.0, 140.0, 40.0 }, { 60.0, 20.0, 143.0, 200.0, 100.0 },
		{ 67.5, 15.0, 155.0, 220.0, 300.0 }, { 75.0, 10.0, 165.0, 240.0, 500.0 }, { 82.5, 5.0, 177.0, 255.0, 750.0 },
		{ 90.0, 0.0, 188.0, 270.0, 950.0 } };
	double frost_a_k = 3182.48;
	double frost_b = 27.9546;
	double cap_air_above_frost_k = 25.0;
	double perennial_frost_lat_deg = 80.0; // north of this, water ice all year
	double perennial_frost_kg_m2 = 1000.0;
	double frost_fringe_deg = 6.0; // thin frost this far outside a cap
	double frost_fringe_kg_m2 = 0.05;
	double cap_pack_kg_m2 = 400.0; // the surface texture's full scale

	// water vapour column (pr-um) at Ls 0, 90, 180, 270, every 30 deg
	// from 90 S (TES, Montmessin 2004 Fig. 2)
	std::array<std::array<double, 7>, 4> vapour_column_pr_um = { { { 2.0, 6.0, 10.0, 12.0, 12.0, 10.0, 8.0 },
		{ 0.5, 2.0, 8.0, 15.0, 20.0, 30.0, 60.0 }, { 1.0, 5.0, 10.0, 12.0, 15.0, 15.0, 10.0 },
		{ 22.0, 15.0, 12.0, 12.0, 10.0, 5.0, 1.0 } } };
	double wet_column_kg_m2 = 0.012; // a column this wet makes full cloud

	// clouds: level edges over the ground, and each kind
	std::array<double, CLIMATE_LEVELS + 1> level_edges_m = { 0.0, 2000.0, 5000.0, 9000.0, 14000.0, 20000.0, 27000.0,
		35000.0, 45000.0 };
	double cloud_patch_m = 250e3;
	double cloud_drift_ms = 8.0;
	double ice_per_tau = 0.008; // kg/m2 of ice per unit of tau
	LsRamp belt_season = { 30.0, 90.0, 120.0, 170.0 }; // the aphelion cloud belt
	double belt_tau = 0.6;
	double belt_lat_deg = 12.0;
	double belt_width_deg = 16.0;
	std::vector<Place> peaks = { { 18.6, -134.0 }, { 11.9, -104.4 }, { 0.9, -113.3 }, { -8.3, -120.1 }, { 25.0, 147.0 } };
	double peak_cloud_radius_m = 350e3; // afternoon clouds over the peaks
	double peak_cloud_tau = 0.15;
	double peak_cloud_afternoon_tau = 0.25;
	double peak_cloud_belt_tau = 0.6; // added in the belt's season
	double hood_tau = 0.8; // the polar hoods
	double hood_lat_deg = 58.0;
	double hood_width_deg = 9.0;
	double hood_peak_ls_north = 270.0;
	double hood_peak_ls_south = 90.0;
	std::vector<CloudBasin> basins = { { -42.4, 70.5, 900e3, 0.7 }, { -49.7, -43.4, 450e3, 0.25 } };
	double basin_peak_ls = 100.0;
	double fog_tau = 0.25; // morning fog on low ground
	double fog_top_m = -1000.0;
	double fog_full_m = -5000.0;
	double high_tau = 0.03;
	double patch_tau = 0.18; // the year-round thin haze
	LevelWeights belt_levels = { 0, 0, 0, 0, 0.6, 0.4, 0, 0 };
	LevelWeights peak_levels = { 0, 0, 0, 0.5, 0.5, 0, 0, 0 };
	LevelWeights hood_levels = { 0, 0, 0.3, 0.7, 0, 0, 0, 0 };
	LevelWeights fog_levels = { 1.0, 0, 0, 0, 0, 0, 0, 0 };
	LevelWeights high_levels = { 0, 0, 0, 0, 0, 0, 0.6, 0.4 };
	LevelWeights patch_levels = { 0, 0, 0, 0, 0.3, 0.7, 0, 0 };
	LevelWeights basin_levels = { 0.3, 0.3, 0.15, 0.1, 0.15, 0, 0, 0 };
};

} // namespace regolith

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "climate_model_config.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

#include <godot_cpp/core/error_macros.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

using namespace godot;
using regolith::ClimateConfig;
using regolith::ClimateModel;
using regolith::LapseProfile;

namespace {

constexpr double ANY = std::numeric_limits<double>::infinity();
constexpr double POSITIVE = 1e-9; // the least a scale or width may be: it is divided by
constexpr double MIN_AIR_K = 1.0; // a gas colder than this is no air to compute with
constexpr int GRID_W_MIN = 4, GRID_W_MAX = 1440; // the climate grid: down to 0.25 deg cells
constexpr int GRID_H_MIN = 2, GRID_H_MAX = 720;
constexpr int STORMS_PER_SOL_MAX = 16;
constexpr int STORM_LIFE_SOLS_MAX = 1000;

void refuse(const char *p_key, const char *p_why) {
	WARN_PRINT(vformat("RegolithWeather: config %s %s; keeping the default", p_key, p_why));
}

bool finite_number(const Variant &p_v, double &r_v) {
	if (p_v.get_type() != Variant::FLOAT && p_v.get_type() != Variant::INT) {
		return false;
	}
	r_v = double(p_v);
	return std::isfinite(r_v);
}

// p_v as exactly p_count numbers in [p_min, p_max].
bool numbers(const Variant &p_v, int p_count, double p_min, double p_max, std::vector<double> &r_out) {
	if (p_v.get_type() != Variant::ARRAY) {
		return false;
	}
	const Array a = p_v;
	if (a.size() != p_count) {
		return false;
	}
	r_out.resize(size_t(p_count));
	for (int i = 0; i < p_count; ++i) {
		if (!finite_number(a[i], r_out[size_t(i)]) || r_out[size_t(i)] < p_min || r_out[size_t(i)] > p_max) {
			return false;
		}
	}
	return true;
}

// p_v as rows of p_width numbers each; any number of rows.
bool rows(const Variant &p_v, int p_width, std::vector<std::vector<double>> &r_out) {
	if (p_v.get_type() != Variant::ARRAY) {
		return false;
	}
	const Array a = p_v;
	r_out.assign(size_t(a.size()), {});
	for (int i = 0; i < a.size(); ++i) {
		if (!numbers(a[i], p_width, -ANY, ANY, r_out[size_t(i)])) {
			return false;
		}
	}
	return true;
}

struct NumberKey {
	const char *key;
	double ClimateModel::*field;
	double min, max;
};

// Every plain number of the model, with what it may be.
const NumberKey NUMBER_KEYS[] = {
	{ "dust_tau_floor", &ClimateModel::dust_tau_floor, 0.0, ANY },
	{ "tau_per_dust", &ClimateModel::tau_per_dust, POSITIVE, ANY },
	{ "dust_tau_default", &ClimateModel::dust_tau_default, 0.0, ANY },
	{ "tau_unknown", &ClimateModel::tau_unknown, 0.0, ANY },
	{ "dust_patch_m", &ClimateModel::dust_patch_m, POSITIVE, ANY },
	{ "plume_tau", &ClimateModel::plume_tau, 0.0, ANY },
	{ "plume_radius_m", &ClimateModel::plume_radius_m, POSITIVE, ANY },
	{ "plume_decay_sols", &ClimateModel::plume_decay_sols, POSITIVE, ANY },
	{ "plume_life_sols", &ClimateModel::plume_life_sols, 0.0, ANY },
	{ "storm_area_default", &ClimateModel::storm_area_default, 0.0, ANY },
	{ "storm_radius_m", &ClimateModel::storm_radius_m, 0.0, ANY },
	{ "storm_radius_spread_m", &ClimateModel::storm_radius_spread_m, 0.0, ANY },
	{ "storm_radius_max_m", &ClimateModel::storm_radius_max_m, POSITIVE, ANY },
	{ "storm_peak_tau", &ClimateModel::storm_peak_tau, 0.0, ANY },
	{ "storm_peak_spread_tau", &ClimateModel::storm_peak_spread_tau, 0.0, ANY },
	{ "storm_off_season", &ClimateModel::storm_off_season, 0.0, ANY },
	{ "storm_drift_ms", &ClimateModel::storm_drift_ms, -ANY, ANY },
	{ "storm_swirl_ms", &ClimateModel::storm_swirl_ms, -ANY, ANY },
	{ "storm_gust_ms", &ClimateModel::storm_gust_ms, -ANY, ANY },
	{ "pressure_annual", &ClimateModel::pressure_annual, -ANY, ANY },
	{ "pressure_annual_ls", &ClimateModel::pressure_annual_ls, -ANY, ANY },
	{ "pressure_semiannual", &ClimateModel::pressure_semiannual, -ANY, ANY },
	{ "pressure_semiannual_ls", &ClimateModel::pressure_semiannual_ls, -ANY, ANY },
	{ "trade_ms", &ClimateModel::trade_ms, -ANY, ANY },
	{ "trade_width_deg", &ClimateModel::trade_width_deg, POSITIVE, ANY },
	{ "jet_ms", &ClimateModel::jet_ms, -ANY, ANY },
	{ "jet_winter_ms", &ClimateModel::jet_winter_ms, -ANY, ANY },
	{ "jet_lat_deg", &ClimateModel::jet_lat_deg, 0.0, 90.0 },
	{ "jet_width_deg", &ClimateModel::jet_width_deg, POSITIVE, ANY },
	{ "hadley_ms", &ClimateModel::hadley_ms, -ANY, ANY },
	{ "hadley_width_deg", &ClimateModel::hadley_width_deg, POSITIVE, ANY },
	{ "tide_east_ms", &ClimateModel::tide_east_ms, -ANY, ANY },
	{ "tide_north_ms", &ClimateModel::tide_north_ms, -ANY, ANY },
	{ "slope_night_ms", &ClimateModel::slope_night_ms, -ANY, ANY },
	{ "slope_day_ms", &ClimateModel::slope_day_ms, -ANY, ANY },
	{ "gust_ms", &ClimateModel::gust_ms, -ANY, ANY },
	{ "albedo_min", &ClimateModel::albedo_min, 0.0, 1.0 },
	{ "albedo_range", &ClimateModel::albedo_range, 0.0, 1.0 },
	{ "absorbed", &ClimateModel::absorbed, 0.0, 1.0 },
	{ "emissivity", &ClimateModel::emissivity, POSITIVE, 1.0 },
	{ "mean_ground_min_k", &ClimateModel::mean_ground_min_k, 0.0, ANY },
	{ "mean_ground_max_k", &ClimateModel::mean_ground_max_k, 0.0, ANY },
	{ "ground_min_k", &ClimateModel::ground_min_k, 0.0, ANY },
	{ "ground_max_k", &ClimateModel::ground_max_k, 0.0, ANY },
	{ "air_min_k", &ClimateModel::air_min_k, 0.0, ANY },
	{ "air_max_k", &ClimateModel::air_max_k, 0.0, ANY },
	{ "diurnal_k", &ClimateModel::diurnal_k, 0.0, ANY },
	{ "diurnal_dust_damping", &ClimateModel::diurnal_dust_damping, 0.0, ANY },
	{ "ground_peak_hour", &ClimateModel::ground_peak_hour, 0.0, 24.0 },
	{ "air_peak_hour", &ClimateModel::air_peak_hour, 0.0, 24.0 },
	{ "air_below_ground_k", &ClimateModel::air_below_ground_k, -ANY, ANY },
	{ "dust_warming_k", &ClimateModel::dust_warming_k, -ANY, ANY },
	{ "frost_a_k", &ClimateModel::frost_a_k, POSITIVE, ANY },
	{ "frost_b", &ClimateModel::frost_b, -ANY, ANY },
	{ "cap_air_above_frost_k", &ClimateModel::cap_air_above_frost_k, -ANY, ANY },
	{ "perennial_frost_lat_deg", &ClimateModel::perennial_frost_lat_deg, 0.0, 90.0 },
	{ "perennial_frost_kg_m2", &ClimateModel::perennial_frost_kg_m2, 0.0, ANY },
	{ "frost_fringe_deg", &ClimateModel::frost_fringe_deg, 0.0, 90.0 },
	{ "frost_fringe_kg_m2", &ClimateModel::frost_fringe_kg_m2, 0.0, ANY },
	{ "cap_pack_kg_m2", &ClimateModel::cap_pack_kg_m2, POSITIVE, ANY },
	{ "wet_column_kg_m2", &ClimateModel::wet_column_kg_m2, POSITIVE, ANY },
	{ "cloud_patch_m", &ClimateModel::cloud_patch_m, POSITIVE, ANY },
	{ "cloud_drift_ms", &ClimateModel::cloud_drift_ms, -ANY, ANY },
	{ "ice_per_tau", &ClimateModel::ice_per_tau, 0.0, ANY },
	{ "belt_tau", &ClimateModel::belt_tau, 0.0, ANY },
	{ "belt_lat_deg", &ClimateModel::belt_lat_deg, -90.0, 90.0 },
	{ "belt_width_deg", &ClimateModel::belt_width_deg, POSITIVE, ANY },
	{ "peak_cloud_radius_m", &ClimateModel::peak_cloud_radius_m, POSITIVE, ANY },
	{ "peak_cloud_tau", &ClimateModel::peak_cloud_tau, 0.0, ANY },
	{ "peak_cloud_afternoon_tau", &ClimateModel::peak_cloud_afternoon_tau, 0.0, ANY },
	{ "peak_cloud_belt_tau", &ClimateModel::peak_cloud_belt_tau, 0.0, ANY },
	{ "hood_tau", &ClimateModel::hood_tau, 0.0, ANY },
	{ "hood_lat_deg", &ClimateModel::hood_lat_deg, 0.0, 90.0 },
	{ "hood_width_deg", &ClimateModel::hood_width_deg, POSITIVE, ANY },
	{ "hood_peak_ls_north", &ClimateModel::hood_peak_ls_north, -ANY, ANY },
	{ "hood_peak_ls_south", &ClimateModel::hood_peak_ls_south, -ANY, ANY },
	{ "basin_peak_ls", &ClimateModel::basin_peak_ls, -ANY, ANY },
	{ "fog_tau", &ClimateModel::fog_tau, 0.0, ANY },
	{ "fog_top_m", &ClimateModel::fog_top_m, -ANY, ANY },
	{ "fog_full_m", &ClimateModel::fog_full_m, -ANY, ANY },
	{ "high_tau", &ClimateModel::high_tau, 0.0, ANY },
	{ "patch_tau", &ClimateModel::patch_tau, 0.0, ANY },
};

struct BodyKey {
	const char *key;
	double ClimateConfig::*field;
	double min, max;
};

// The body and its air.
const BodyKey BODY_KEYS[] = {
	{ "radius", &ClimateConfig::radius, POSITIVE, ANY },
	{ "gravity", &ClimateConfig::gravity, POSITIVE, ANY },
	{ "sol_s", &ClimateConfig::sol_s, POSITIVE, ANY },
	{ "obliquity_deg", &ClimateConfig::obliquity_deg, 0.0, 180.0 },
	{ "eccentricity", &ClimateConfig::eccentricity, 0.0, 0.99 },
	{ "perihelion_ls", &ClimateConfig::perihelion_ls, -ANY, ANY },
	{ "solar_flux", &ClimateConfig::solar_flux, 0.0, ANY },
	{ "scale_height", &ClimateConfig::scale_height, POSITIVE, ANY },
	{ "datum_pressure", &ClimateConfig::datum_pressure, 0.0, ANY },
	{ "r_gas", &ClimateConfig::r_gas, POSITIVE, ANY },
	{ "gamma", &ClimateConfig::gamma, 1.0, ANY },
	{ "mean_air_k", &ClimateConfig::mean_air_k, MIN_AIR_K, ANY },
	{ "px_per_turn", &ClimateConfig::px_per_turn, POSITIVE, ANY },
};

struct LapseKey {
	const char *key;
	double LapseProfile::*field;
	double min;
};

const LapseKey LAPSE_KEYS[] = {
	{ "lapse_low", &LapseProfile::low_k_per_m, 0.0 },
	{ "lapse_high", &LapseProfile::high_k_per_m, 0.0 },
	{ "lapse_turn_m", &LapseProfile::turn_m, 0.0 },
	{ "air_floor_k", &LapseProfile::floor_k, MIN_AIR_K },
};

// Keys read by their own functions below.
const char *const OTHER_KEYS[] = { "gw", "gh", "storms_per_sol", "storm_life_max_sols", "storm_season_south",
	"storm_season_north", "storm_season_default", "belt_season", "cap_north", "cap_south", "storm_sites", "peaks",
	"basins", "level_edges_m", "vapour_column_pr_um" };

struct WeightKey {
	const char *key;
	regolith::LevelWeights ClimateModel::*field;
};

const WeightKey WEIGHT_KEYS[] = {
	{ "belt_levels", &ClimateModel::belt_levels },
	{ "peak_levels", &ClimateModel::peak_levels },
	{ "hood_levels", &ClimateModel::hood_levels },
	{ "fog_levels", &ClimateModel::fog_levels },
	{ "high_levels", &ClimateModel::high_levels },
	{ "patch_levels", &ClimateModel::patch_levels },
	{ "basin_levels", &ClimateModel::basin_levels },
};

// p_d's p_key into r_v if it is a number in [p_min, p_max]; else said.
void read_number(const Dictionary &p_d, const char *p_key, double p_min, double p_max, double &r_v) {
	if (!p_d.has(p_key)) {
		return;
	}
	double v;
	if (!finite_number(p_d[p_key], v) || v < p_min || v > p_max) {
		refuse(p_key, vformat("is not a number from %s to %s", p_min, p_max).utf8().get_data());
		return;
	}
	r_v = v;
}

void read_numbers(const Dictionary &p_d, ClimateConfig &r_c) {
	for (const BodyKey &k : BODY_KEYS) {
		read_number(p_d, k.key, k.min, k.max, r_c.*k.field);
	}
	for (const LapseKey &k : LAPSE_KEYS) {
		read_number(p_d, k.key, k.min, ANY, r_c.lapse.*k.field);
	}
	for (const NumberKey &k : NUMBER_KEYS) {
		read_number(p_d, k.key, k.min, k.max, r_c.model.*k.field);
	}
}

bool known_key(const String &p_key) {
	auto is = [&p_key](const char *p_name) { return p_key == p_name; };
	return std::any_of(std::begin(BODY_KEYS), std::end(BODY_KEYS), [&](const BodyKey &k) { return is(k.key); }) ||
			std::any_of(std::begin(LAPSE_KEYS), std::end(LAPSE_KEYS), [&](const LapseKey &k) { return is(k.key); }) ||
			std::any_of(std::begin(NUMBER_KEYS), std::end(NUMBER_KEYS), [&](const NumberKey &k) { return is(k.key); }) ||
			std::any_of(std::begin(WEIGHT_KEYS), std::end(WEIGHT_KEYS), [&](const WeightKey &k) { return is(k.key); }) ||
			std::any_of(std::begin(OTHER_KEYS), std::end(OTHER_KEYS), is);
}

// Keys that are none of the above: a typo would otherwise keep Mars unsaid.
void report_unknown(const Dictionary &p_d) {
	const Array keys = p_d.keys();
	for (int i = 0; i < keys.size(); ++i) {
		const String key = keys[i];
		if (!known_key(key)) {
			WARN_PRINT("RegolithWeather: config " + key + " is not a weather setting; ignored");
		}
	}
}

void read_integer(const Dictionary &p_d, const char *p_key, int p_min, int p_max, int &r_v) {
	if (!p_d.has(p_key)) {
		return;
	}
	double v;
	if (!finite_number(p_d[p_key], v) || v != std::floor(v) || v < p_min || v > p_max) {
		refuse(p_key, "is not a whole number in its range");
		return;
	}
	r_v = int(v);
}

void read_window(const Dictionary &p_d, const char *p_key, regolith::LsWindow &r_w) {
	std::vector<double> v;
	if (!p_d.has(p_key)) {
		return;
	}
	if (!numbers(p_d[p_key], 2, -ANY, ANY, v)) {
		refuse(p_key, "is not [from, to] in Ls degrees");
		return;
	}
	r_w = { v[0], v[1] };
}

void read_ramp(const Dictionary &p_d, const char *p_key, regolith::LsRamp &r_r) {
	std::vector<double> v;
	if (!p_d.has(p_key)) {
		return;
	}
	if (!numbers(p_d[p_key], 4, -ANY, ANY, v) || !(v[0] < v[1] && v[2] < v[3])) {
		refuse(p_key, "is not [rise_from, rise_to, fall_from, fall_to] with each ramp's start before its end");
		return;
	}
	r_r = { v[0], v[1], v[2], v[3] };
}

// [[lat, ls_on, ls_peak, ls_off, kg_m2], ...], latitudes rising.
void read_caps(const Dictionary &p_d, const char *p_key, std::vector<regolith::CapBand> &r_bands) {
	std::vector<std::vector<double>> r;
	if (!p_d.has(p_key)) {
		return;
	}
	bool ok = rows(p_d[p_key], 5, r);
	for (size_t i = 0; ok && i < r.size(); ++i) {
		ok = r[i][4] >= 0.0 && (i == 0 || r[i][0] > r[i - 1][0]);
	}
	if (!ok) {
		refuse(p_key, "is not rows of [lat, ls_on, ls_peak, ls_off, kg_m2] with latitudes rising");
		return;
	}
	r_bands.clear();
	for (const std::vector<double> &b : r) {
		r_bands.push_back({ b[0], b[1], b[2], b[3], b[4] });
	}
}

void read_places(const Dictionary &p_d, ClimateModel &r_m) {
	std::vector<std::vector<double>> r;
	if (p_d.has("storm_sites")) {
		if (rows(p_d["storm_sites"], 3, r)) {
			r_m.storm_sites.clear();
			for (const std::vector<double> &s : r) {
				r_m.storm_sites.push_back({ s[0], s[1], s[2] != 0.0 });
			}
		} else {
			refuse("storm_sites", "is not rows of [lat, lon, south (1 or 0)]");
		}
	}
	if (p_d.has("peaks")) {
		if (rows(p_d["peaks"], 2, r)) {
			r_m.peaks.clear();
			for (const std::vector<double> &s : r) {
				r_m.peaks.push_back({ s[0], s[1] });
			}
		} else {
			refuse("peaks", "is not rows of [lat, lon]");
		}
	}
	if (p_d.has("basins")) {
		bool ok = rows(p_d["basins"], 4, r);
		for (size_t i = 0; ok && i < r.size(); ++i) {
			ok = r[i][2] > 0.0;
		}
		if (ok) {
			r_m.basins.clear();
			for (const std::vector<double> &s : r) {
				r_m.basins.push_back({ s[0], s[1], s[2], s[3] });
			}
		} else {
			refuse("basins", "is not rows of [lat, lon, radius_m (above 0), weight]");
		}
	}
}

void read_tables(const Dictionary &p_d, ClimateModel &r_m) {
	std::vector<double> v;
	if (p_d.has("level_edges_m")) {
		bool ok = numbers(p_d["level_edges_m"], regolith::CLIMATE_LEVELS + 1, -ANY, ANY, v);
		for (size_t i = 1; ok && i < v.size(); ++i) {
			ok = v[i] > v[i - 1];
		}
		if (ok) {
			std::copy(v.begin(), v.end(), r_m.level_edges_m.begin());
		} else {
			refuse("level_edges_m", "is not 9 rising heights (m)");
		}
	}
	for (const WeightKey &k : WEIGHT_KEYS) {
		if (!p_d.has(k.key)) {
			continue;
		}
		if (!numbers(p_d[k.key], regolith::CLIMATE_LEVELS, 0.0, ANY, v)) {
			refuse(k.key, "is not 8 weights of 0 or more, one per level");
			continue;
		}
		std::copy(v.begin(), v.end(), (r_m.*k.field).begin());
	}
	if (p_d.has("vapour_column_pr_um")) {
		std::vector<std::vector<double>> r;
		bool ok = rows(p_d["vapour_column_pr_um"], 7, r) && r.size() == 4;
		for (size_t s = 0; ok && s < r.size(); ++s) {
			for (const double c : r[s]) {
				ok = ok && c >= 0.0;
			}
		}
		if (ok) {
			for (size_t s = 0; s < 4; ++s) {
				std::copy(r[s].begin(), r[s].end(), r_m.vapour_column_pr_um[s].begin());
			}
		} else {
			refuse("vapour_column_pr_um", "is not 4 rows (Ls 0, 90, 180, 270) of 7 columns of 0 or more (90 S to 90 N)");
		}
	}
}

// Pairs that must stay in order whatever was set one by one.
void check_pairs(ClimateModel &r_m) {
	const ClimateModel mars;
	if (!(r_m.ground_max_k > r_m.ground_min_k)) {
		refuse("ground_min_k and ground_max_k", "are not min below max");
		r_m.ground_min_k = mars.ground_min_k;
		r_m.ground_max_k = mars.ground_max_k;
	}
	if (!(r_m.mean_ground_max_k > r_m.mean_ground_min_k)) {
		refuse("mean_ground_min_k and mean_ground_max_k", "are not min below max");
		r_m.mean_ground_min_k = mars.mean_ground_min_k;
		r_m.mean_ground_max_k = mars.mean_ground_max_k;
	}
	if (!(r_m.air_max_k > r_m.air_min_k)) {
		refuse("air_min_k and air_max_k", "are not min below max");
		r_m.air_min_k = mars.air_min_k;
		r_m.air_max_k = mars.air_max_k;
	}
	if (r_m.fog_top_m == r_m.fog_full_m) {
		refuse("fog_top_m and fog_full_m", "are the same height");
		r_m.fog_top_m = mars.fog_top_m;
		r_m.fog_full_m = mars.fog_full_m;
	}
}

} // namespace

regolith::ClimateConfig godot::climate_config_from(const Dictionary &p_config) {
	ClimateConfig c;
	ClimateModel &m = c.model;
	report_unknown(p_config);
	read_numbers(p_config, c);
	read_integer(p_config, "gw", GRID_W_MIN, GRID_W_MAX, c.gw);
	read_integer(p_config, "gh", GRID_H_MIN, GRID_H_MAX, c.gh);
	read_integer(p_config, "storms_per_sol", 0, STORMS_PER_SOL_MAX, m.storms_per_sol);
	read_integer(p_config, "storm_life_max_sols", 1, STORM_LIFE_SOLS_MAX, m.storm_life_max_sols);
	read_window(p_config, "storm_season_south", m.storm_season_south);
	read_window(p_config, "storm_season_north", m.storm_season_north);
	read_ramp(p_config, "storm_season_default", m.storm_season_default);
	read_ramp(p_config, "belt_season", m.belt_season);
	read_caps(p_config, "cap_north", m.cap_north);
	read_caps(p_config, "cap_south", m.cap_south);
	read_places(p_config, m);
	read_tables(p_config, m);
	check_pairs(m);
	return c;
}

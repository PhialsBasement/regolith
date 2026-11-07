// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith/climate.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "regolith/ephemeris.hpp"

namespace regolith {

namespace {

constexpr double PI = std::numbers::pi;
constexpr double TAU = 2.0 * PI;
constexpr double DEG = PI / 180.0;
constexpr double SIGMA = 5.670e-8;

// Seasonal CO2 caps (mors's tables): per band of latitude,
// the Ls the frost arrives, peaks and leaves, and its kg/m2
constexpr double CAP_N_LAT[5] = { 60.0, 67.5, 75.0, 82.5, 90.0 };
constexpr double CAP_N_ON[5] = { 230.0, 215.0, 200.0, 185.0, 170.0 };
constexpr double CAP_N_PEAK[5] = { 322.5, 335.0, 347.0, 357.0, 367.5 };
constexpr double CAP_N_OFF[5] = { 390.0, 417.0, 442.0, 455.0, 460.0 };
constexpr double CAP_N_MASS[5] = { 90.0, 250.0, 400.0, 550.0, 700.0 };
constexpr double CAP_S_LAT[6] = { 50.0, 60.0, 67.5, 75.0, 82.5, 90.0 };
constexpr double CAP_S_ON[6] = { 60.0, 20.0, 15.0, 10.0, 5.0, 0.0 };
constexpr double CAP_S_PEAK[6] = { 100.0, 143.0, 155.0, 165.0, 177.0, 188.0 };
constexpr double CAP_S_OFF[6] = { 140.0, 200.0, 220.0, 240.0, 255.0, 270.0 };
constexpr double CAP_S_MASS[6] = { 40.0, 100.0, 300.0, 500.0, 750.0, 950.0 };

// TES zonal-mean vapour column (pr-um) at Ls 0, 90, 180, 270,
// every 30 deg from 90 S (Montmessin 2004 Fig. 2)
constexpr double TES_COLUMN[4][7] = {
	{ 2.0, 6.0, 10.0, 12.0, 12.0, 10.0, 8.0 },
	{ 0.5, 2.0, 8.0, 15.0, 20.0, 30.0, 60.0 },
	{ 1.0, 5.0, 10.0, 12.0, 15.0, 15.0, 10.0 },
	{ 22.0, 15.0, 12.0, 12.0, 10.0, 5.0, 1.0 },
};

// where regional storms rise: lat, lon (deg), south (1/0)
struct Site {
	double lat, lon;
	bool south;
};
constexpr Site SITES[8] = { { -42.0, 70.0, true }, { -50.0, -43.0, true }, { -26.0, -90.0, true },
	{ -45.0, 0.0, true }, { 25.0, -40.0, false }, { 45.0, 110.0, false }, { 12.0, 87.0, false },
	{ 55.0, -30.0, false } };

// Tharsis and Elysium: afternoon orographic clouds
struct Peak {
	double lat, lon;
};
constexpr Peak PEAKS[5] = { { 18.6, -134.0 }, { 11.9, -104.4 }, { 0.9, -113.3 }, { -8.3, -120.1 }, { 25.0, 147.0 } };

// how each cloud kind spreads over the levels
// Mars's water clouds sit high (15 to 30 km) in thin sheets;
// the hoods lower, fog on the ground
constexpr double W_ACB[Climate::NL] = { 0, 0, 0, 0, 0.6, 0.4, 0, 0 };
constexpr double W_ORO[Climate::NL] = { 0, 0, 0, 0.5, 0.5, 0, 0, 0 };
constexpr double W_HOOD[Climate::NL] = { 0, 0, 0.3, 0.7, 0, 0, 0, 0 };
constexpr double W_FOG[Climate::NL] = { 1.0, 0, 0, 0, 0, 0, 0, 0 };
constexpr double W_HIGH[Climate::NL] = { 0, 0, 0, 0, 0, 0, 0.6, 0.4 };
constexpr double W_PATCH[Climate::NL] = { 0, 0, 0, 0, 0.3, 0.7, 0, 0 };
// basin fog and haze low down, some cloud aloft
constexpr double W_BASIN[Climate::NL] = { 0.3, 0.3, 0.15, 0.1, 0.15, 0, 0, 0 };

// impact basins that fill with cloud: lat, lon, radius (m),
// strength
struct Basin {
	double lat, lon, radius, weight;
};
constexpr Basin BASINS[2] = { { -42.4, 70.5, 900e3, 0.7 }, { -49.7, -43.4, 450e3, 0.25 } };

constexpr int MAX_LIFE_SOLS = 18;
constexpr int STORMS_PER_SOL = 2;

double clampd(double p_v, double p_lo, double p_hi) {
	return p_v < p_lo ? p_lo : (p_v > p_hi ? p_hi : p_v);
}
double sstep(double p_a, double p_b, double p_x) {
	const double s = clampd((p_x - p_a) / (p_b - p_a), 0.0, 1.0);
	return s * s * (3.0 - 2.0 * s);
}
double lerpd(double p_a, double p_b, double p_t) {
	return p_a + (p_b - p_a) * p_t;
}
double fposmod(double p_x, double p_y) {
	const double v = std::fmod(p_x, p_y);
	return v < 0.0 ? v + p_y : v;
}

uint64_t mix(uint64_t p_x) {
	p_x += 0x9e3779b97f4a7c15ull;
	p_x = (p_x ^ (p_x >> 30)) * 0xbf58476d1ce4e5b9ull;
	p_x = (p_x ^ (p_x >> 27)) * 0x94d049bb133111ebull;
	return p_x ^ (p_x >> 31);
}
double unit(uint64_t p_h) {
	return double(p_h >> 11) * (1.0 / 9007199254740992.0);
}

// Value noise on a lattice, 0..1, smooth between points.
double lattice(int64_t p_x, int64_t p_y, int64_t p_z, uint64_t p_seed) {
	return unit(mix(uint64_t(p_x) * 73856093ull ^ uint64_t(p_y) * 19349663ull ^ uint64_t(p_z) * 83492791ull ^ p_seed));
}
double vnoise(double p_x, double p_y, double p_z, uint64_t p_seed) {
	const double fx = std::floor(p_x), fy = std::floor(p_y), fz = std::floor(p_z);
	const int64_t x = int64_t(fx), y = int64_t(fy), z = int64_t(fz);
	auto s = [](double p_t) { return p_t * p_t * (3.0 - 2.0 * p_t); };
	const double tx = s(p_x - fx), ty = s(p_y - fy), tz = s(p_z - fz);
	auto row = [&](int64_t p_y0, int64_t p_z0) {
		return lerpd(lattice(x, p_y0, p_z0, p_seed), lattice(x + 1, p_y0, p_z0, p_seed), tx);
	};
	return lerpd(lerpd(row(y, z), row(y + 1, z), ty), lerpd(row(y, z + 1), row(y + 1, z + 1), ty), tz);
}
double fbm(const DVec3 &p_p, uint64_t p_seed) {
	return 0.65 * vnoise(p_p.x, p_p.y, p_p.z, p_seed) +
			0.35 * vnoise(p_p.x * 2.13 + 7.1, p_p.y * 2.13 - 3.3, p_p.z * 2.13 + 1.9, p_seed + 1);
}

double interp_band(const double *p_lats, const double *p_vals, int p_n, double p_lat) {
	if (p_lat <= p_lats[0]) {
		return p_vals[0];
	}
	for (int b = 1; b < p_n; ++b) {
		if (p_lat <= p_lats[b]) {
			return lerpd(p_vals[b - 1], p_vals[b], (p_lat - p_lats[b - 1]) / (p_lats[b] - p_lats[b - 1]));
		}
	}
	return p_vals[p_n - 1];
}

double cap_mass(double p_lat_eff, double p_ls, bool p_north) {
	const double *lats = p_north ? CAP_N_LAT : CAP_S_LAT;
	const int n = p_north ? 5 : 6;
	const double lo = lats[0];
	if (p_lat_eff < lo - 5.0) {
		return 0.0;
	}
	const double l = clampd(p_lat_eff, lo, 90.0);
	const double on = interp_band(lats, p_north ? CAP_N_ON : CAP_S_ON, n, l);
	const double pk = interp_band(lats, p_north ? CAP_N_PEAK : CAP_S_PEAK, n, l);
	const double off = interp_band(lats, p_north ? CAP_N_OFF : CAP_S_OFF, n, l);
	const double m = interp_band(lats, p_north ? CAP_N_MASS : CAP_S_MASS, n, l) *
			clampd((p_lat_eff - lo + 5.0) / 5.0, 0.0, 1.0);
	const double x = fposmod(p_ls - on, 360.0) + on;
	if (x > off) {
		return 0.0;
	}
	return x <= pk ? m * (x - on) / std::max(pk - on, 1.0) : m * (off - x) / std::max(off - pk, 1.0);
}

double frost_point(double p_p) {
	return 3182.48 / (27.9546 - std::log(std::max(p_p, 1.0)));
}

// Diurnal-mean ground temperature from the day's insolation.
double t_mean(double p_lat, double p_dec, double p_flux, double p_alb, double p_solar) {
	const double h0 = std::acos(clampd(-std::tan(p_lat) * std::tan(p_dec), -1.0, 1.0));
	const double q = p_solar * p_flux / PI *
			(std::sin(p_lat) * std::sin(p_dec) * h0 + std::cos(p_lat) * std::cos(p_dec) * std::sin(h0));
	const double t4 = std::max((1.0 - p_alb) * 0.88 * q, 2.0) / (0.85 * 0.97 * SIGMA);
	return clampd(std::pow(t4, 0.25), 145.0, 265.0);
}

double tes_column(double p_lat, double p_ls) {
	const double s = fposmod(p_ls, 360.0) / 90.0;
	const int s0 = int(s) % 4, s1 = (s0 + 1) % 4;
	const double y = clampd((p_lat + 90.0) / 30.0, 0.0, 6.0);
	const int y0 = std::min(int(y), 5);
	auto at = [&](int p_s) { return lerpd(TES_COLUMN[p_s][y0], TES_COLUMN[p_s][y0 + 1], y - y0); };
	return lerpd(at(s0), at(s1), s - std::floor(s)) * 1.0e-3; // kg/m2
}

DVec3 dir_of(double p_lat, double p_lon) {
	return { std::cos(p_lat) * std::cos(p_lon), std::cos(p_lat) * std::sin(p_lon), std::sin(p_lat) };
}

void pack_fields(const Climate::Field &p_low, const Climate::Field &p_high, const Climate::Field &p_du, size_t p_n,
		std::vector<uint8_t> &r_out) {
	const double tau_s = 255.0 / Climate::PACK_TAU_MAX;
	const double dust_s = 255.0 / Climate::PACK_DUST_MAX;
	r_out.resize(p_n * 4);
	for (size_t k = 0; k < p_n; ++k) {
		r_out[k * 4] = uint8_t(clampd(p_low[k], 0.0, Climate::PACK_TAU_MAX) * tau_s);
		r_out[k * 4 + 1] = uint8_t(clampd(p_high[k], 0.0, Climate::PACK_TAU_MAX) * tau_s);
		r_out[k * 4 + 2] = 0;
		r_out[k * 4 + 3] = uint8_t(clampd(p_du[k], 0.0, Climate::PACK_DUST_MAX) * dust_s);
	}
}

} // namespace

Climate::Climate(const ClimateConfig &p_config) :
		GW(p_config.gw),
		GH(p_config.gh),
		N(p_config.gw * p_config.gh),
		W_PX(p_config.px_per_turn),
		H_PX(p_config.px_per_turn * 0.5),
		GCELL_PX(p_config.px_per_turn / p_config.gw),
		cfg_(p_config) {}

void Climate::create(const float *p_topo, const float *p_lum, const std::vector<std::array<double, 7>> &p_clim) {
	for (Field *f : { &dust, &su, &sv, &lu, &lv, &ps, &ts, &tair, &co2, &frost, &wv, &tau_low, &tau_high, &topo, &alb_,
				 &slope_e_, &slope_n_ }) {
		f->assign(size_t(N), 0.0f);
	}
	for (int l = 0; l < NL; ++l) {
		tau[size_t(l)].assign(size_t(N), 0.0f);
		ice[size_t(l)].assign(size_t(N), 0.0f);
	}
	clim_ = p_clim.size() == 24 ? p_clim : std::vector<std::array<double, 7>>();
	dir_.resize(size_t(N));
	lat_.resize(size_t(GH));
	lon_.resize(size_t(GW));
	for (int j = 0; j < GH; ++j) {
		lat_[size_t(j)] = (90.0 - (j + 0.5) / GH * 180.0) * DEG;
	}
	for (int i = 0; i < GW; ++i) {
		lon_[size_t(i)] = ((i + 0.5) / GW * 360.0 - 180.0) * DEG;
	}
	double lo = 0.0, hi = 1.0;
	if (p_lum) {
		std::vector<float> sorted(p_lum, p_lum + N);
		std::sort(sorted.begin(), sorted.end());
		lo = sorted[size_t(N * 0.05)];
		hi = std::max(double(sorted[size_t(N * 0.95)]), lo + 1e-3);
	}
	for (int j = 0; j < GH; ++j) {
		for (int i = 0; i < GW; ++i) {
			const size_t k = size_t(j) * GW + i;
			dir_[k] = dir_of(lat_[size_t(j)], lon_[size_t(i)]);
			topo[k] = p_topo ? p_topo[k] : 0.0f;
			const double n = p_lum ? clampd((p_lum[k] - lo) / (hi - lo), 0.0, 1.0) : 0.5;
			alb_[k] = float(0.11 + 0.20 * n);
		}
	}
	// terrain gradient (m/m), east and north, for slope winds
	const double dy = PI * cfg_.radius / GH;
	for (int j = 0; j < GH; ++j) {
		const double dx = TAU * cfg_.radius * std::max(std::cos(lat_[size_t(j)]), 0.1) / GW;
		const int jn = std::max(j - 1, 0), js = std::min(j + 1, GH - 1);
		for (int i = 0; i < GW; ++i) {
			const size_t k = size_t(j) * GW + i;
			const int iw = (i + GW - 1) % GW, ie = (i + 1) % GW;
			slope_e_[k] = float((topo[size_t(j) * GW + ie] - topo[size_t(j) * GW + iw]) / (2.0 * dx));
			slope_n_[k] = float((topo[size_t(jn) * GW + i] - topo[size_t(js) * GW + i]) / (dy * (js - jn)));
		}
	}
}

// One climatology column at a season, by 15 deg bins.
double Climate::clim_at(int p_col, double p_ls) const {
	const double x = fposmod(p_ls, 360.0) / 15.0 - 0.5;
	const int i0 = (int(std::floor(x)) + 24) % 24, i1 = (i0 + 1) % 24;
	return lerpd(clim_[size_t(i0)][size_t(p_col)], clim_[size_t(i1)][size_t(p_col)], x - std::floor(x));
}

// Seeded regional storms: up to two may rise each sol, from
// the season's storm activity, and live a few sols each.
void Climate::storms_at(double p_time, std::vector<Storm> &r_out) const {
	r_out.clear();
	const int64_t today = int64_t(std::floor(p_time / cfg_.sol_s));
	for (int64_t s = today - MAX_LIFE_SOLS; s <= today; ++s) {
		const double t0 = double(s) * cfg_.sol_s;
		const double ls0 = mars_solar_longitude(t0);
		// clim area of tau > 2 (1e6 km2); a dusty-season bump
		const double area = clim_.empty() ? 2.0 * sstep(180.0, 230.0, ls0) * (1.0 - sstep(270.0, 340.0, ls0))
										  : clim_at(6, ls0);
		for (int c = 0; c < STORMS_PER_SOL; ++c) {
			uint64_t h = mix(uint64_t(s) * 2654435761ull + uint64_t(c) * 40503ull + 0x5eedull);
			auto next = [&h] {
				h = mix(h);
				return unit(h);
			};
			if (next() > clampd(0.10 + 0.08 * area, 0.0, 0.8)) {
				continue;
			}
			const Site &site = SITES[size_t(next() * 8.0) % 8];
			const bool in_season = site.south ? (ls0 > 140.0) : (ls0 > 180.0 || ls0 < 40.0);
			const double life = std::min(2.0 + next() * 6.0 + area * 0.8, double(MAX_LIFE_SOLS));
			const double age = (p_time - t0) / cfg_.sol_s - next();
			if (age < 0.0 || age > life) {
				continue;
			}
			const double a = age / life;
			const double u = next();
			const double radius = std::min((250e3 + 900e3 * u * u) * (1.0 + 0.25 * std::min(area, 8.0)), 3.0e6);
			const double peak = (0.5 + 1.8 * next() + 0.1 * std::min(area, 4.0)) * (in_season ? 1.0 : 0.35);
			// eastward drift at ~5 m/s, easing toward the equator
			const double lat = (site.lat + (next() - 0.5) * 8.0) * (1.0 - 0.25 * a);
			const double drift = 5.0 * age * cfg_.sol_s / (cfg_.radius * std::max(std::cos(lat * DEG), 0.2));
			const double lon = site.lon * DEG + (next() - 0.5) * 0.15 + drift;
			const double env = sstep(0.0, 0.2, a) * (1.0 - sstep(0.45, 1.0, a));
			r_out.push_back({ dir_of(lat * DEG, lon), radius, peak, env });
		}
	}
}

void Climate::update(double p_time) {
	const double sol = cfg_.sol_s;
	ls = mars_solar_longitude(p_time);
	const double lt0 = mars_local_solar_time(p_time, 0.0);
	const double ls_r = ls * DEG;
	const double dec = std::asin(std::sin(cfg_.obliquity_deg * DEG) * std::sin(ls_r));
	const double ecc = cfg_.eccentricity;
	const double flux = std::pow((1.0 + ecc * std::cos(ls_r - cfg_.perihelion_ls * DEG)) / (1.0 - ecc * ecc), 2.0);
	// +1 in northern summer, -1 in southern
	const double sd = std::sin(dec) / std::sin(cfg_.obliquity_deg * DEG);
	// Viking-like datum pressure through the year
	const double p_datum = cfg_.datum_pressure *
			(1.0 + 0.075 * std::cos((ls - 255.0) * DEG) + 0.04 * std::cos(2.0 * (ls - 55.0) * DEG));
	const double acb = sstep(30.0, 90.0, ls) * (1.0 - sstep(120.0, 170.0, ls));
	const double hood_n = clampd(0.5 + 0.7 * std::cos((ls - 270.0) * DEG), 0.0, 1.0);
	const double hood_s = clampd(0.5 + 0.7 * std::cos((ls - 90.0) * DEG), 0.0, 1.0);
	double base[3] = { 0.3, 0.3, 0.3 }; // south, global, north
	if (!clim_.empty()) {
		base[0] = clim_at(2, ls);
		base[1] = clim_at(0, ls);
		base[2] = clim_at(4, ls);
	}
	std::vector<Storm> storms;
	storms_at(p_time, storms);
	std::vector<double> storm_cos(storms.size());
	for (size_t s = 0; s < storms.size(); ++s) {
		storm_cos[s] = std::cos(std::min(3.0 * storms[s].radius / cfg_.radius, PI));
	}
	plumes_.erase(std::remove_if(plumes_.begin(), plumes_.end(),
						  [&](const Plume &p_p) { return p_time < p_p.t0 || p_time - p_p.t0 > 3.0 * sol; }),
			plumes_.end());
	std::vector<double> plume_k(plumes_.size());
	for (size_t q = 0; q < plumes_.size(); ++q) {
		plume_k[q] = 1.6 * std::exp(-(p_time - plumes_[q].t0) / (0.4 * sol));
	}
	std::vector<DVec3> peaks(5);
	for (size_t q = 0; q < 5; ++q) {
		peaks[q] = dir_of(PEAKS[q].lat * DEG, PEAKS[q].lon * DEG);
	}
	std::vector<DVec3> basins(2);
	std::vector<double> basin_cos(2);
	for (size_t q = 0; q < 2; ++q) {
		basins[q] = dir_of(BASINS[q].lat * DEG, BASINS[q].lon * DEG);
		basin_cos[q] = std::cos(std::min(3.0 * BASINS[q].radius / cfg_.radius, PI));
	}
	// southern autumn and winter, never fully clear
	const double basin_season = clampd(0.55 + 0.55 * std::cos((ls - 100.0) * DEG), 0.2, 1.0);
	const double days = p_time / sol;
	// dust patches evolve over sols; clouds ride the jet
	const double dust_k = cfg_.radius / 400e3;
	const double cloud_k = cfg_.radius / 250e3;
	const double rot = std::fmod(p_time * 8.0 / cfg_.radius, TAU);
	const double cr = std::cos(rot), sr = std::sin(rot);
	const double peak_cos = std::cos(350e3 * 2.5 / cfg_.radius);
	const double plume_cos = std::cos(120e3 * 3.0 / cfg_.radius);
	for (int j = 0; j < GH; ++j) {
		const double lat = lat_[size_t(j)];
		const double latd = lat / DEG;
		const double clat = std::cos(lat);
		// the zonal mean: easterly trades, the winter jet
		const double winter = clampd(-sd * (lat >= 0.0 ? 1.0 : -1.0), 0.0, 1.0);
		const double u_zonal = -5.0 * std::exp(-(latd / 25.0) * (latd / 25.0)) +
				(3.0 + 9.0 * winter) * std::exp(-((std::fabs(latd) - 55.0) / 15.0) * ((std::fabs(latd) - 55.0) / 15.0));
		// the Hadley cell's surface branch crosses to summer
		const double v_had = 4.0 * sd * std::exp(-(latd / 30.0) * (latd / 30.0));
		const double band = latd < 0.0 ? lerpd(base[0], base[1], clampd(latd / -45.0, 0.0, 1.0))
									   : lerpd(base[1], base[2], clampd(latd / 45.0, 0.0, 1.0));
		const DVec3 east_z(0.0, 0.0, 1.0);
		for (int i = 0; i < GW; ++i) {
			const size_t k = size_t(j) * GW + i;
			const DVec3 &d = dir_[k];
			const double h = topo[k];
			const double lt = fposmod(lt0 + lon_[size_t(i)] / DEG / 15.0, 24.0);
			const double w_day = TAU / 24.0 * (lt - 13.5);
			const double nd = fbm(DVec3(d.x * dust_k, d.y * dust_k, d.z * dust_k + days * 0.15), 11);
			const DVec3 cd(d.x * cr + d.y * sr, -d.x * sr + d.y * cr, d.z);
			// squeezed north-south: bands drawn out along the jet
			const double nc = fbm(DVec3(cd.x * cloud_k, cd.y * cloud_k, cd.z * cloud_k * 3.5 + days * 0.3), 23);
			const double ns = vnoise(d.x * 6.0, d.y * 6.0, d.z * 6.0, 37);
			// dust: the season's band, patches, storms, plumes
			double t_dust = band * (0.8 + 0.4 * nd);
			const DVec3 east = normalize(cross(east_z, d));
			const DVec3 north = cross(d, east);
			DVec3 w3;
			for (size_t s = 0; s < storms.size(); ++s) {
				const Storm &st = storms[s];
				const double c = dot(d, st.centre);
				if (c < storm_cos[s] || st.env <= 0.0) {
					continue;
				}
				const double x = std::acos(clampd(c, -1.0, 1.0)) * cfg_.radius / st.radius;
				t_dust += st.peak * st.env * std::exp(-x * x) * (0.55 + 0.9 * nd);
				// cyclonic swirl, peaking at the radius, and the
				// outflow at the front
				const double hemi = st.centre.z >= 0.0 ? 1.0 : -1.0;
				const DVec3 tang = cross(st.centre, d) * hemi;
				const DVec3 out = d - st.centre * c;
				const double tl = length(tang), ol = length(out);
				const double swirl = 14.0 * st.env * x * std::exp(0.5 * (1.0 - x * x));
				const double gust = 6.0 * st.env * std::exp(-((x - 1.1) / 0.3) * ((x - 1.1) / 0.3));
				if (tl > 1e-9) {
					w3 = w3 + tang * (swirl / tl);
				}
				if (ol > 1e-9) {
					w3 = w3 + out * (gust / ol);
				}
			}
			for (size_t q = 0; q < plumes_.size(); ++q) {
				const double c = dot(d, plumes_[q].centre);
				if (c > plume_cos) {
					const double r = std::acos(std::min(c, 1.0)) * cfg_.radius / 120e3;
					t_dust += plume_k[q] * std::exp(-r * r);
				}
			}
			t_dust = std::max(t_dust, TAU_FLOOR);
			dust[k] = float((t_dust - TAU_FLOOR) / TAU_PER_DUST);
			// winds: zonal mean, the tide, slope flows, gusts
			double ue = u_zonal + 3.0 * std::cos(w_day) * clat;
			double vn = v_had + 2.0 * std::sin(w_day) * std::sin(lat);
			const double ge = slope_e_[k], gn = slope_n_[k];
			const double g = std::hypot(ge, gn);
			if (g > 1e-5) {
				// down the slope at night, up it by day
				const double day = std::cos(w_day);
				const double down = (day < 0.0 ? -9.0 * day : -5.0 * day) * std::tanh(g * 60.0);
				ue -= ge / g * down;
				vn -= gn / g * down;
			}
			ue += dot(w3, east) + 4.0 * (nd - 0.5);
			vn += dot(w3, north) + 4.0 * (nc - 0.5);
			su[k] = float(ue);
			sv[k] = float(-vn);
			lu[k] = float(2.2 * u_zonal + 0.5 * dot(w3, east));
			lv[k] = float(0.5 * v_had);
			// temperatures from the day's sun, damped by dust
			const double tm = t_mean(lat, dec, flux, alb_[k], cfg_.solar_flux);
			const double daylit = std::acos(clampd(-std::tan(lat) * std::tan(dec), -1.0, 1.0)) / PI;
			const double amp = 38.0 * daylit * std::exp(-0.35 * t_dust) * (1.4 - 2.0 * alb_[k]);
			const double shape = 0.75 * std::cos(w_day) + 0.25 * std::cos(2.0 * w_day);
			const double w_air = TAU / 24.0 * (lt - 15.0);
			const double p_here = p_datum * std::exp(-h / cfg_.scale_height);
			ps[k] = float(p_here);
			// caps: CO2 by season and latitude, a ragged edge
			const double lat_eff = std::fabs(latd) + 3.0 * (ns - 0.5);
			const double c2 = cap_mass(lat_eff, ls, latd > 0.0);
			co2[k] = float(c2);
			double fr = latd > 80.0 ? 1000.0 : 0.0;
			if (c2 <= 0.0 && cap_mass(lat_eff + 6.0, ls, latd > 0.0) > 0.0) {
				fr = std::max(fr, 0.05);
			}
			frost[k] = float(fr);
			double tg = tm + amp * shape;
			double ta = tm - 8.0 + 0.6 * amp * (0.75 * std::cos(w_air) + 0.25 * std::cos(2.0 * w_air)) +
					6.0 * (t_dust - 0.3) * daylit;
			if (c2 > 0.0) {
				tg = frost_point(p_here);
				ta = std::min(ta, tg + 25.0);
			}
			ts[k] = float(clampd(tg, 140.0, 300.0));
			tair[k] = float(clampd(ta, 140.0, 290.0));
			wv[k] = float(tes_column(latd, ls) * (0.8 + 0.4 * ns));
			// water ice clouds, each kind on its own levels
			const double c_acb = 0.6 * acb * std::exp(-((latd - 12.0) / 16.0) * ((latd - 12.0) / 16.0)) *
					(0.3 + 1.4 * nc);
			// year-round weather: patches over a third of the sky
			const double wet = 0.6 + 0.4 * clampd(wv[k] / 0.012, 0.0, 1.5);
			// a thin haze nearly everywhere, thicker in bands
			const double c_patch = 0.18 * wet * (0.45 + 0.9 * sstep(0.3, 0.75, nc)) * (0.7 + 0.6 * ns);
			// the basins fill with fog and haze, most in southern
			// autumn and winter (Hellas, Argyre)
			double c_basin = 0.0;
			for (size_t q = 0; q < basins.size(); ++q) {
				const double c = dot(d, basins[q]);
				if (c > basin_cos[q]) {
					const double x = std::acos(std::min(c, 1.0)) * cfg_.radius / BASINS[q].radius;
					c_basin += BASINS[q].weight * std::exp(-x * x);
				}
			}
			c_basin *= basin_season * (0.6 + 0.5 * sstep(4.0, 7.0, lt) * (1.0 - sstep(10.0, 13.0, lt))) *
					(0.7 + 0.6 * nc);
			double c_oro = 0.0;
			const double afternoon = sstep(10.0, 14.0, lt) * (1.0 - sstep(17.0, 20.0, lt));
			for (const DVec3 &pk : peaks) {
				const double c = dot(d, pk);
				if (c > peak_cos) {
					const double r = std::acos(std::min(c, 1.0)) * cfg_.radius / 350e3;
					c_oro += (0.15 + (0.25 + 0.6 * acb) * afternoon) * std::exp(-r * r);
				}
			}
			const double c_hood = 0.8 * (0.3 + 1.4 * nc) *
					(hood_n * std::exp(-((latd - 58.0) / 9.0) * ((latd - 58.0) / 9.0)) +
							hood_s * std::exp(-((latd + 58.0) / 9.0) * ((latd + 58.0) / 9.0)));
			const double morning = sstep(3.0, 6.0, lt) * (1.0 - sstep(8.0, 10.0, lt));
			const double c_fog = 0.25 * morning * sstep(-1000.0, -5000.0, h) * nc;
			const double c_high = 0.03 * nc * (1.0 - sstep(60.0, 75.0, std::fabs(latd)));
			double lo_sum = 0.0, hi_sum = 0.0;
			for (int l = 0; l < NL; ++l) {
				const double t = c_acb * W_ACB[l] + c_oro * W_ORO[l] + c_hood * W_HOOD[l] + c_fog * W_FOG[l] +
						c_high * W_HIGH[l] + c_patch * W_PATCH[l] + c_basin * W_BASIN[l];
				tau[size_t(l)][k] = float(t);
				ice[size_t(l)][k] = float(t * 0.008);
				(l < NL_LOW ? lo_sum : hi_sum) += t;
			}
			tau_low[k] = float(lo_sum);
			tau_high[k] = float(hi_sum);
		}
	}
}

void Climate::inject_dust(double p_px, double p_py, double p_time) {
	fold_px(p_px, p_py);
	const double lat = (90.0 - p_py / H_PX * 180.0) * DEG;
	const double lon = (p_px / W_PX * 360.0 - 180.0) * DEG;
	plumes_.push_back({ dir_of(lat, lon), p_time });
}

void Climate::fold_px(double &r_px, double &r_py) const {
	const double m = fposmod(r_py, 2.0 * H_PX);
	if (m >= H_PX) {
		r_px += W_PX * 0.5;
		r_py = 2.0 * H_PX - m;
	} else {
		r_py = m;
	}
}

double Climate::sample(const Field &p_f, double p_px, double p_py) const {
	fold_px(p_px, p_py);
	const double fx = fposmod(p_px / GCELL_PX - 0.5, double(GW));
	const double fy = clampd(p_py / GCELL_PX - 0.5, 0.0, double(GH) - 1.001);
	const int x0 = std::min(int(fx), GW - 1), y0 = int(fy);
	const int x1 = (x0 + 1) % GW, y1 = std::min(y0 + 1, GH - 1);
	return lerpd(lerpd(p_f[size_t(y0 * GW + x0)], p_f[size_t(y0 * GW + x1)], fx - x0),
			lerpd(p_f[size_t(y1 * GW + x0)], p_f[size_t(y1 * GW + x1)], fx - x0), fy - y0);
}

void Climate::pack_global(std::vector<uint8_t> &r_out) const {
	pack_fields(tau_low, tau_high, dust, size_t(N), r_out);
}

void Climate::pack_surface(std::vector<uint8_t> &r_out) const {
	r_out.resize(size_t(N) * 4);
	for (size_t k = 0; k < size_t(N); ++k) {
		r_out[k * 4] = uint8_t(clampd(co2[k] / 400.0, 0.0, 1.0) * 255.0);
		r_out[k * 4 + 1] = uint8_t(clampd(frost[k], 0.0, 1.0) * 255.0);
		r_out[k * 4 + 2] = uint8_t(clampd((ts[k] - 140.0) / 160.0, 0.0, 1.0) * 255.0);
		r_out[k * 4 + 3] = uint8_t(clampd((tair[k] - 140.0) / 160.0, 0.0, 1.0) * 255.0);
	}
}

} // namespace regolith

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

constexpr double CAP_FADE_DEG = 5.0; // a cap thins out over this much latitude at its edge

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

// p_bands' p_value at p_lat, linear between the bands (sorted by latitude).
double interp_band(const std::vector<CapBand> &p_bands, double CapBand::*p_value, double p_lat) {
	if (p_lat <= p_bands.front().lat_deg) {
		return p_bands.front().*p_value;
	}
	for (size_t b = 1; b < p_bands.size(); ++b) {
		if (p_lat <= p_bands[b].lat_deg) {
			return lerpd(p_bands[b - 1].*p_value, p_bands[b].*p_value,
					(p_lat - p_bands[b - 1].lat_deg) / (p_bands[b].lat_deg - p_bands[b - 1].lat_deg));
		}
	}
	return p_bands.back().*p_value;
}

double cap_mass(const ClimateModel &p_m, double p_lat_eff, double p_ls, bool p_north) {
	const std::vector<CapBand> &bands = p_north ? p_m.cap_north : p_m.cap_south;
	if (bands.empty()) {
		return 0.0;
	}
	const double lo = bands.front().lat_deg;
	if (p_lat_eff < lo - CAP_FADE_DEG) {
		return 0.0;
	}
	const double l = clampd(p_lat_eff, lo, 90.0);
	const double on = interp_band(bands, &CapBand::ls_on, l);
	const double pk = interp_band(bands, &CapBand::ls_peak, l);
	const double off = interp_band(bands, &CapBand::ls_off, l);
	const double m = interp_band(bands, &CapBand::kg_m2, l) *
			clampd((p_lat_eff - lo + CAP_FADE_DEG) / CAP_FADE_DEG, 0.0, 1.0);
	const double x = fposmod(p_ls - on, 360.0) + on;
	if (x > off) {
		return 0.0;
	}
	return x <= pk ? m * (x - on) / std::max(pk - on, 1.0) : m * (off - x) / std::max(off - pk, 1.0);
}

double frost_point(const ClimateModel &p_m, double p_p) {
	return p_m.frost_a_k / (p_m.frost_b - std::log(std::max(p_p, 1.0)));
}

// Diurnal-mean ground temperature from the day's insolation.
double t_mean(const ClimateModel &p_m, double p_lat, double p_dec, double p_flux, double p_alb, double p_solar) {
	const double h0 = std::acos(clampd(-std::tan(p_lat) * std::tan(p_dec), -1.0, 1.0));
	const double q = p_solar * p_flux / PI *
			(std::sin(p_lat) * std::sin(p_dec) * h0 + std::cos(p_lat) * std::cos(p_dec) * std::sin(h0));
	const double t4 = std::max((1.0 - p_alb) * p_m.absorbed * q, 2.0) / (p_m.emissivity * SIGMA);
	return clampd(std::pow(t4, 0.25), p_m.mean_ground_min_k, p_m.mean_ground_max_k);
}

double vapour_column(const ClimateModel &p_m, double p_lat, double p_ls) {
	const double s = fposmod(p_ls, 360.0) / 90.0;
	const int s0 = int(s) % 4, s1 = (s0 + 1) % 4;
	const double y = clampd((p_lat + 90.0) / 30.0, 0.0, 6.0);
	const int y0 = std::min(int(y), 5);
	const auto &table = p_m.vapour_column_pr_um;
	auto at = [&](int p_s) { return lerpd(table[size_t(p_s)][size_t(y0)], table[size_t(p_s)][size_t(y0 + 1)], y - y0); };
	return lerpd(at(s0), at(s1), s - std::floor(s)) * 1.0e-3; // kg/m2
}

// p_ls inside p_window (Ls degrees, wrapping past 360 when to < from).
bool in_window(double p_ls, const LsWindow &p_window) {
	return p_window.from <= p_window.to ? (p_ls > p_window.from && p_ls < p_window.to)
										: (p_ls > p_window.from || p_ls < p_window.to);
}

// p_scale outside p_ramp's rise and fall times 0, between them times 1.
double ramp(double p_ls, const LsRamp &p_ramp, double p_scale) {
	return p_scale * sstep(p_ramp.rise_from, p_ramp.rise_to, p_ls) * (1.0 - sstep(p_ramp.fall_from, p_ramp.fall_to, p_ls));
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
			alb_[k] = float(cfg_.model.albedo_min + cfg_.model.albedo_range * n);
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
	const ClimateModel &m = cfg_.model;
	if (m.storm_sites.empty()) {
		return;
	}
	const size_t n_sites = m.storm_sites.size();
	const int64_t today = int64_t(std::floor(p_time / cfg_.sol_s));
	for (int64_t s = today - m.storm_life_max_sols; s <= today; ++s) {
		const double t0 = double(s) * cfg_.sol_s;
		const double ls0 = solar_longitude(cfg_.orbit, cfg_.rotation, t0);
		// clim area of tau > 2 (1e6 km2); a dusty-season bump
		const double area = clim_.empty() ? ramp(ls0, m.storm_season_default, m.storm_area_default) : clim_at(6, ls0);
		for (int c = 0; c < m.storms_per_sol; ++c) {
			uint64_t h = mix(uint64_t(s) * 2654435761ull + uint64_t(c) * 40503ull + 0x5eedull);
			auto next = [&h] {
				h = mix(h);
				return unit(h);
			};
			if (next() > clampd(0.10 + 0.08 * area, 0.0, 0.8)) {
				continue;
			}
			const StormSite &site = m.storm_sites[size_t(next() * double(n_sites)) % n_sites];
			const bool in_season = in_window(ls0, site.south ? m.storm_season_south : m.storm_season_north);
			const double life = std::min(2.0 + next() * 6.0 + area * 0.8, double(m.storm_life_max_sols));
			const double age = (p_time - t0) / cfg_.sol_s - next();
			if (age < 0.0 || age > life) {
				continue;
			}
			const double a = age / life;
			const double u = next();
			const double radius = std::min((m.storm_radius_m + m.storm_radius_spread_m * u * u) *
							(1.0 + 0.25 * std::min(area, 8.0)),
					m.storm_radius_max_m);
			const double peak = (m.storm_peak_tau + m.storm_peak_spread_tau * next() + 0.1 * std::min(area, 4.0)) *
					(in_season ? 1.0 : m.storm_off_season);
			// eastward drift, easing toward the equator
			const double lat = (site.lat_deg + (next() - 0.5) * 8.0) * (1.0 - 0.25 * a);
			const double drift = m.storm_drift_ms * age * cfg_.sol_s / (cfg_.radius * std::max(std::cos(lat * DEG), 0.2));
			const double lon = site.lon_deg * DEG + (next() - 0.5) * 0.15 + drift;
			const double env = sstep(0.0, 0.2, a) * (1.0 - sstep(0.45, 1.0, a));
			r_out.push_back({ dir_of(lat * DEG, lon), radius, peak, env });
		}
	}
}

void Climate::update(double p_time) {
	const ClimateModel &m = cfg_.model;
	const double sol = cfg_.sol_s;
	ls = solar_longitude(cfg_.orbit, cfg_.rotation, p_time);
	const double lt0 = local_solar_time(sun_direction(cfg_.orbit, cfg_.rotation, p_time), 0.0);
	const double ls_r = ls * DEG;
	const double dec = std::asin(std::sin(cfg_.obliquity_deg * DEG) * std::sin(ls_r));
	const double ecc = cfg_.eccentricity;
	const double flux = std::pow((1.0 + ecc * std::cos(ls_r - cfg_.perihelion_ls * DEG)) / (1.0 - ecc * ecc), 2.0);
	// +1 in northern summer, -1 in southern
	const double tilt = std::sin(cfg_.obliquity_deg * DEG);
	const double sd = tilt != 0.0 ? std::sin(dec) / tilt : 0.0; // an untilted body has no summer
	// the datum pressure through the year
	const double p_datum = cfg_.datum_pressure * (1.0 + m.pressure_annual * std::cos((ls - m.pressure_annual_ls) * DEG) +
														 m.pressure_semiannual * std::cos(2.0 * (ls - m.pressure_semiannual_ls) * DEG));
	const double acb = ramp(ls, m.belt_season, 1.0);
	const double hood_n = clampd(0.5 + 0.7 * std::cos((ls - m.hood_peak_ls_north) * DEG), 0.0, 1.0);
	const double hood_s = clampd(0.5 + 0.7 * std::cos((ls - m.hood_peak_ls_south) * DEG), 0.0, 1.0);
	double base[3] = { m.dust_tau_default, m.dust_tau_default, m.dust_tau_default }; // south, global, north
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
						  [&](const Plume &p_p) { return p_time < p_p.t0 || p_time - p_p.t0 > m.plume_life_sols * sol; }),
			plumes_.end());
	std::vector<double> plume_k(plumes_.size());
	for (size_t q = 0; q < plumes_.size(); ++q) {
		plume_k[q] = m.plume_tau * std::exp(-(p_time - plumes_[q].t0) / (m.plume_decay_sols * sol));
	}
	std::vector<DVec3> peaks(m.peaks.size());
	for (size_t q = 0; q < peaks.size(); ++q) {
		peaks[q] = dir_of(m.peaks[q].lat_deg * DEG, m.peaks[q].lon_deg * DEG);
	}
	std::vector<DVec3> basins(m.basins.size());
	std::vector<double> basin_cos(m.basins.size());
	for (size_t q = 0; q < basins.size(); ++q) {
		basins[q] = dir_of(m.basins[q].lat_deg * DEG, m.basins[q].lon_deg * DEG);
		basin_cos[q] = std::cos(std::min(3.0 * m.basins[q].radius_m / cfg_.radius, PI));
	}
	// the basins' cloudy season, never fully clear
	const double basin_season = clampd(0.55 + 0.55 * std::cos((ls - m.basin_peak_ls) * DEG), 0.2, 1.0);
	const double days = p_time / sol;
	// dust patches evolve over sols; clouds ride the jet
	const double dust_k = cfg_.radius / m.dust_patch_m;
	const double cloud_k = cfg_.radius / m.cloud_patch_m;
	const double rot = std::fmod(p_time * m.cloud_drift_ms / cfg_.radius, TAU);
	const double cr = std::cos(rot), sr = std::sin(rot);
	const double peak_cos = std::cos(m.peak_cloud_radius_m * 2.5 / cfg_.radius);
	const double plume_cos = std::cos(m.plume_radius_m * 3.0 / cfg_.radius);
	for (int j = 0; j < GH; ++j) {
		const double lat = lat_[size_t(j)];
		const double latd = lat / DEG;
		const double clat = std::cos(lat);
		// the zonal mean: easterly trades, the winter jet
		const double winter = clampd(-sd * (lat >= 0.0 ? 1.0 : -1.0), 0.0, 1.0);
		const double u_zonal = -m.trade_ms * std::exp(-(latd / m.trade_width_deg) * (latd / m.trade_width_deg)) +
				(m.jet_ms + m.jet_winter_ms * winter) *
						std::exp(-((std::fabs(latd) - m.jet_lat_deg) / m.jet_width_deg) *
								((std::fabs(latd) - m.jet_lat_deg) / m.jet_width_deg));
		// the Hadley cell's surface branch crosses to summer
		const double v_had = m.hadley_ms * sd * std::exp(-(latd / m.hadley_width_deg) * (latd / m.hadley_width_deg));
		const double band = latd < 0.0 ? lerpd(base[0], base[1], clampd(latd / -45.0, 0.0, 1.0))
									   : lerpd(base[1], base[2], clampd(latd / 45.0, 0.0, 1.0));
		const DVec3 east_z(0.0, 0.0, 1.0);
		for (int i = 0; i < GW; ++i) {
			const size_t k = size_t(j) * GW + i;
			const DVec3 &d = dir_[k];
			const double h = topo[k];
			const double lt = fposmod(lt0 + lon_[size_t(i)] / DEG / 15.0, 24.0);
			const double w_day = TAU / 24.0 * (lt - m.ground_peak_hour);
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
				const double swirl = m.storm_swirl_ms * st.env * x * std::exp(0.5 * (1.0 - x * x));
				const double gust = m.storm_gust_ms * st.env * std::exp(-((x - 1.1) / 0.3) * ((x - 1.1) / 0.3));
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
					const double r = std::acos(std::min(c, 1.0)) * cfg_.radius / m.plume_radius_m;
					t_dust += plume_k[q] * std::exp(-r * r);
				}
			}
			t_dust = std::max(t_dust, m.dust_tau_floor);
			dust[k] = float((t_dust - m.dust_tau_floor) / m.tau_per_dust);
			// winds: zonal mean, the tide, slope flows, gusts
			double ue = u_zonal + m.tide_east_ms * std::cos(w_day) * clat;
			double vn = v_had + m.tide_north_ms * std::sin(w_day) * std::sin(lat);
			const double ge = slope_e_[k], gn = slope_n_[k];
			const double g = std::hypot(ge, gn);
			if (g > 1e-5) {
				// down the slope at night, up it by day
				const double day = std::cos(w_day);
				const double down = (day < 0.0 ? -m.slope_night_ms * day : -m.slope_day_ms * day) * std::tanh(g * 60.0);
				ue -= ge / g * down;
				vn -= gn / g * down;
			}
			ue += dot(w3, east) + m.gust_ms * (nd - 0.5);
			vn += dot(w3, north) + m.gust_ms * (nc - 0.5);
			su[k] = float(ue);
			sv[k] = float(-vn);
			lu[k] = float(2.2 * u_zonal + 0.5 * dot(w3, east));
			lv[k] = float(0.5 * v_had);
			// temperatures from the day's sun, damped by dust
			const double tm = t_mean(m, lat, dec, flux, alb_[k], cfg_.solar_flux);
			const double daylit = std::acos(clampd(-std::tan(lat) * std::tan(dec), -1.0, 1.0)) / PI;
			const double amp = m.diurnal_k * daylit * std::exp(-m.diurnal_dust_damping * t_dust) * (1.4 - 2.0 * alb_[k]);
			const double shape = 0.75 * std::cos(w_day) + 0.25 * std::cos(2.0 * w_day);
			const double w_air = TAU / 24.0 * (lt - m.air_peak_hour);
			const double p_here = p_datum * std::exp(-h / cfg_.scale_height);
			ps[k] = float(p_here);
			// caps: CO2 by season and latitude, a ragged edge
			const double lat_eff = std::fabs(latd) + 3.0 * (ns - 0.5);
			const double c2 = cap_mass(m, lat_eff, ls, latd > 0.0);
			co2[k] = float(c2);
			double fr = latd > m.perennial_frost_lat_deg ? m.perennial_frost_kg_m2 : 0.0;
			if (c2 <= 0.0 && cap_mass(m, lat_eff + m.frost_fringe_deg, ls, latd > 0.0) > 0.0) {
				fr = std::max(fr, m.frost_fringe_kg_m2);
			}
			frost[k] = float(fr);
			double tg = tm + amp * shape;
			double ta = tm - m.air_below_ground_k + 0.6 * amp * (0.75 * std::cos(w_air) + 0.25 * std::cos(2.0 * w_air)) +
					m.dust_warming_k * (t_dust - m.dust_tau_default) * daylit;
			if (c2 > 0.0) {
				tg = frost_point(m, p_here);
				ta = std::min(ta, tg + m.cap_air_above_frost_k);
			}
			ts[k] = float(clampd(tg, m.ground_min_k, m.ground_max_k));
			tair[k] = float(clampd(ta, m.air_min_k, m.air_max_k));
			wv[k] = float(vapour_column(m, latd, ls) * (0.8 + 0.4 * ns));
			// water ice clouds, each kind on its own levels
			const double c_acb = m.belt_tau * acb *
					std::exp(-((latd - m.belt_lat_deg) / m.belt_width_deg) * ((latd - m.belt_lat_deg) / m.belt_width_deg)) *
					(0.3 + 1.4 * nc);
			// year-round weather: patches over a third of the sky
			const double wet = 0.6 + 0.4 * clampd(wv[k] / m.wet_column_kg_m2, 0.0, 1.5);
			// a thin haze nearly everywhere, thicker in bands
			const double c_patch = m.patch_tau * wet * (0.45 + 0.9 * sstep(0.3, 0.75, nc)) * (0.7 + 0.6 * ns);
			// the basins fill with fog and haze, most in their season
			double c_basin = 0.0;
			for (size_t q = 0; q < basins.size(); ++q) {
				const double c = dot(d, basins[q]);
				if (c > basin_cos[q]) {
					const double x = std::acos(std::min(c, 1.0)) * cfg_.radius / m.basins[q].radius_m;
					c_basin += m.basins[q].weight * std::exp(-x * x);
				}
			}
			c_basin *= basin_season * (0.6 + 0.5 * sstep(4.0, 7.0, lt) * (1.0 - sstep(10.0, 13.0, lt))) *
					(0.7 + 0.6 * nc);
			double c_oro = 0.0;
			const double afternoon = sstep(10.0, 14.0, lt) * (1.0 - sstep(17.0, 20.0, lt));
			for (const DVec3 &pk : peaks) {
				const double c = dot(d, pk);
				if (c > peak_cos) {
					const double r = std::acos(std::min(c, 1.0)) * cfg_.radius / m.peak_cloud_radius_m;
					c_oro += (m.peak_cloud_tau + (m.peak_cloud_afternoon_tau + m.peak_cloud_belt_tau * acb) * afternoon) *
							std::exp(-r * r);
				}
			}
			const double c_hood = m.hood_tau * (0.3 + 1.4 * nc) *
					(hood_n * std::exp(-((latd - m.hood_lat_deg) / m.hood_width_deg) * ((latd - m.hood_lat_deg) / m.hood_width_deg)) +
							hood_s *
									std::exp(-((latd + m.hood_lat_deg) / m.hood_width_deg) *
											((latd + m.hood_lat_deg) / m.hood_width_deg)));
			const double morning = sstep(3.0, 6.0, lt) * (1.0 - sstep(8.0, 10.0, lt));
			const double c_fog = m.fog_tau * morning * sstep(m.fog_top_m, m.fog_full_m, h) * nc;
			const double c_high = m.high_tau * nc * (1.0 - sstep(60.0, 75.0, std::fabs(latd)));
			double lo_sum = 0.0, hi_sum = 0.0;
			for (int l = 0; l < NL; ++l) {
				const size_t lv = size_t(l);
				const double t = c_acb * m.belt_levels[lv] + c_oro * m.peak_levels[lv] + c_hood * m.hood_levels[lv] +
						c_fog * m.fog_levels[lv] + c_high * m.high_levels[lv] + c_patch * m.patch_levels[lv] +
						c_basin * m.basin_levels[lv];
				tau[lv][k] = float(t);
				ice[lv][k] = float(t * m.ice_per_tau);
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
	const ClimateModel &m = cfg_.model;
	const double span_k = m.ground_max_k - m.ground_min_k;
	r_out.resize(size_t(N) * 4);
	for (size_t k = 0; k < size_t(N); ++k) {
		r_out[k * 4] = uint8_t(clampd(co2[k] / m.cap_pack_kg_m2, 0.0, 1.0) * 255.0);
		r_out[k * 4 + 1] = uint8_t(clampd(frost[k], 0.0, 1.0) * 255.0);
		r_out[k * 4 + 2] = uint8_t(clampd((ts[k] - m.ground_min_k) / span_k, 0.0, 1.0) * 255.0);
		r_out[k * 4 + 3] = uint8_t(clampd((tair[k] - m.ground_min_k) / span_k, 0.0, 1.0) * 255.0);
	}
}

} // namespace regolith

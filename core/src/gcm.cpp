#include "regolith/gcm.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <thread>


// Line-for-line port of mors scripts/mars_atmosphere.gd; the
// comments there (and RESEARCH_ATMOSPHERE.md) carry the sources
// for every constant. Only what the sphere changes is new.

namespace regolith {

namespace {

constexpr double PI = std::numbers::pi;
constexpr double TAU = 2.0 * PI;
constexpr double DEG = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI; // Godot's rad_to_deg

// Body and grid constants are members, from GcmConfig; what
// stays here is Mars's column physics.
constexpr double DT = Gcm::DT;
constexpr double SURF_WIND = Gcm::SURF_WIND;
constexpr double TOPO_ALPHA = 0.02;
constexpr double TAU_FLOOR = Gcm::TAU_FLOOR;
constexpr double TAU_PER_DUST = Gcm::TAU_PER_DUST;
constexpr double PACK_DUST_MAX = 2.0;
constexpr double SIGMA = 5.670e-8;
constexpr double EMIS = 0.97;
constexpr double L_CO2 = 5.9e5;
constexpr double T_REF = 200.0;
constexpr double TIDE_GROUND_W = 0.09;
constexpr double TT_ANOM_MAX = 40.0;
constexpr double MEAN_STATE_W = 0.6;
constexpr double MEAN_STATE_MAX = 30.0;
constexpr double SW_K = 0.4;
constexpr double SW_ABS = 0.55;
constexpr double DUST_HEAT_K = 30.0;
constexpr double DUST_HEAT_MAX = 1.5;
constexpr double CH_BULK = 0.006;
constexpr double SLOPE_GAIN = 1440.0;
constexpr double SLOPE_MAX = 28.8;
constexpr double FRONT_K = 0.0;
constexpr double FRONT_MAX = 30.0;
constexpr double STAB_LO = 0.45;
constexpr double STAB_HI = 0.9;
constexpr double STRESS_RAIL_LO = 1.5;
constexpr double STRESS_RAIL_HI = 3.0;
constexpr double BC_K = 2.5e6;
constexpr double PAUSE_DEPTH = 0.6;
constexpr double PAUSE_IN = 25.0;
constexpr double PAUSE_OUT = 60.0;
constexpr double SOUTH_EDDY = 0.5;
constexpr double EDGE_K = 2.5e4;
constexpr double EDGE_MAX = 17.0;
constexpr double BL_DUST_W = 0.5;
constexpr double HAD_V_SURF = 7.5;
constexpr double HAD_JET = 12.8;
constexpr double HAD_JET_LAT = 30.0;
constexpr double HAD_EASTERLY = 5.1;
constexpr double HAD_WINTER_W = 7.0;
constexpr double HAD_EQUINOX = 0.075;
constexpr double HAD_DUST_K = 0.24;
constexpr double HAD_V_UPPER = 6.0;
constexpr double WBC_GAIN = 1.5;
constexpr double BC_AMP_MAX = 150.0;
constexpr double FRONT_WIND = 30.0;
constexpr double FRONT_PHASE_W = 0.2;
constexpr double TAU_STAR = 0.0225;
constexpr double KG_TO_DUST = 192.0;
constexpr double LIFT_ALPHA = 9.0e-6;
constexpr double DEVIL_ALPHA = 1.0e-10;
constexpr double LOG_LAW = 17.27;
constexpr double WEIBULL_K = 1.5;
constexpr double SED_FINE = 2.5e-7;
constexpr double SED_COARSE = 6.8e-7;
constexpr double COARSE_FRAC = 1.0;
constexpr double DUST_DIFF_K = 3.0e5;
constexpr double CAP_RAMP = 0.9;
constexpr double CAP_LAT_PER_K = 3.0;

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

// Water: Montmessin et al. 2004 (M04) cloud scheme, docs/water.md.
// Molecular constants as the LMD code (microphys_h.F90).
constexpr double M_RATIO = 18.015 / 43.34; // Viking 2 mean weight
constexpr double RHO_ICE = 917.0; // kg/m3
constexpr double MH2O = 18.0e-3, MCO2 = 44.0e-3; // kg/mol
constexpr double RGP = 8.3143, NAV = 6.023e23, KBZ = 1.381e-23;
constexpr double R_MCO2 = 1.97e-10, R_MH2O = 1.2e-10; // m
constexpr double SIG_H2O = 0.12; // N/m, ice and vapour
// K/m to 40 km, isothermal above (Seiff and Kirk 1977)
constexpr double LAPSE = 1.6e-3;
constexpr double LAPSE_TOP = 40000.0;
// eddy mixing, Montmessin 2002 para 33: 100 m2/s to 25 km,
// rising inversely with density above
constexpr double K_ZZ = 100.0;
constexpr double K_ZZ_TOP = 25000.0;
// nuclei, M04 para 29: N0 at tau_ref, radius r0 decaying
constexpr double CCN0 = 2.0e6; // per m3
constexpr double CCN_TAU_REF = 0.1;
constexpr double R0_SURF = 0.8e-6, R0_H = 18000.0; // m
// sedimentation radius factor f = exp(3 sigma^2), M04 para 31
constexpr double F_SED = 1.5;
constexpr double Q_EXT = 2.0; // visible, M04 para 85
// surface layer: first level at 4 m, z0 1 cm (M04 eq. 1)
constexpr double Z1_SURF = 4.0, Z0_SURF = 0.01;
constexpr double ALBEDO_ICE = 0.4; // over 5 um of ice, M04 para 35
constexpr double FROST_VISIBLE = 5.0e-3; // kg/m2 (5 um)
constexpr double NORTH_CAP_LAT = 80.0; // perennial ice, M04 para 34
constexpr double NORTH_CAP_ICE = 1000.0; // kg/m2, a reservoir
// start: TES zonal-mean column vapour (pr-um), read off M04
// Fig. 2 (top) at Ls 0, 90, 180, 270, every 30 deg from 90 S
constexpr double TES_COLUMN[4][7] = {
	{ 2.0, 6.0, 10.0, 12.0, 12.0, 10.0, 8.0 },
	{ 0.5, 2.0, 8.0, 15.0, 20.0, 30.0, 60.0 },
	{ 1.0, 5.0, 10.0, 12.0, 15.0, 15.0, 10.0 },
	{ 22.0, 15.0, 12.0, 12.0, 10.0, 5.0, 1.0 },
};

double tes_column(double p_lat, double p_ls) {
	const double s = std::fmod(p_ls + 360.0, 360.0) / 90.0;
	const int s0 = int(s) % 4, s1 = (s0 + 1) % 4;
	const double fs = s - std::floor(s);
	const double y = std::clamp((p_lat + 90.0) / 30.0, 0.0, 6.0);
	const int y0 = std::min(int(y), 5);
	const double fy = y - y0;
	auto at = [&](int p_s) {
		return TES_COLUMN[p_s][y0] + (TES_COLUMN[p_s][y0 + 1] - TES_COLUMN[p_s][y0]) * fy;
	};
	return (at(s0) + (at(s1) - at(s0)) * fs) * 1.0e-3; // kg/m2
}

// Vapour pressure over ice, Pa (Murphy and Koop 2005).
double e_ice(double p_t) {
	return std::exp(9.550426 - 5723.265 / p_t + 3.53068 * std::log(p_t) - 0.00728332 * p_t);
}

// CO2 gas mean free path, m (LMD newsedim, molrad 2.2e-10)
double free_path(double p_t, double p_p) {
	return 0.707 * 8.31 / (4.0 * 3.1416 * 2.2e-10 * 2.2e-10 * 6.023e23) * p_t / p_p;
}

// Stokes-Cunningham fall speed, m/s (Montmessin 2002 sec. 2.3,
// Fuchs 1964); viscosity 1e-5 Pa s as LMD newsedim
double fall_speed(double p_r, double p_t, double p_p, double p_g) {
	const double kn = free_path(p_t, p_p) / p_r;
	return 2.0 * p_g * RHO_ICE * p_r * p_r / (9.0 * 1.0e-5) * (1.0 + kn * (1.246 + 0.42 * std::exp(-0.87 / kn)));
}

// A column's levels: air mass (kg/m2), mid pressure and
// temperature (the Viking lapse about the mean T_a), the edge
// factors exp(-z / H); one exp per edge, one log per column
struct Column {
	double air[Gcm::NL], p[Gcm::NL], t[Gcm::NL], edge[Gcm::NL + 1];
	double sh;
};

void column_levels(double p_ps, double p_ta, double p_r, double p_g, Column &r_c) {
	r_c.sh = p_r * p_ta / p_g;
	for (int i = 0; i <= Gcm::NL; ++i) {
		r_c.edge[i] = std::exp(-Gcm::LEVEL_M[i] / r_c.sh);
	}
	const double log_ps = std::log(p_ps);
	for (int l = 0; l < Gcm::NL; ++l) {
		const double zm = 0.5 * (Gcm::LEVEL_M[l] + Gcm::LEVEL_M[l + 1]);
		r_c.air[l] = p_ps * (r_c.edge[l] - r_c.edge[l + 1]) / p_g;
		r_c.p[l] = p_ps * std::sqrt(r_c.edge[l] * r_c.edge[l + 1]);
		// CO2 frost point there, log p = log p_s - z / H
		const double t_fr = 3182.48 / (27.9546 - std::max(log_ps - zm / r_c.sh, std::log(1e-3)));
		r_c.t[l] = std::max(p_ta + LAPSE * r_c.sh - LAPSE * std::min(zm, LAPSE_TOP), t_fr);
	}
}

// Scalar exchange coefficient C_h and the wind it multiplies,
// LMD vdif_cd_mod.F90 (Colaitis 2013, England 1995 branch)
void surface_exchange(double p_t_air, double p_t_s, double p_wind, double p_wstar, double p_g, double &r_ch,
		double &r_u) {
	constexpr double karman = 0.41, nu = 0.001, bm = 16.0, bh = 16.0;
	constexpr double alphah = 1.0, betam = 5.0, betah = 5.0;
	const double lambda = std::sqrt(bh / bm) / alphah;
	const double ric = betah / (betam * betam);
	const double lz = std::log(Z1_SURF / Z0_SURF);
	const double cdn = (karman / lz) * (karman / lz);
	const double gust = std::log(1.0 + 0.7 * p_wstar + 2.3 * p_wstar * p_wstar);
	const double zu2 = std::max(p_wind * p_wind + gust * gust, 1e-6);
	double z0t_comp = 0.1 * Z0_SURF, chn = cdn, fh = 1.0;
	for (int ite = 0; ite < 10; ++ite) {
		const double z0t = z0t_comp;
		chn = cdn * lz / std::log(Z1_SURF / z0t);
		const double rib = p_g / p_t_s * std::sqrt(Z1_SURF * Z0_SURF) * lz * lz / std::log(Z1_SURF / z0t) *
				(p_t_air - p_t_s) / zu2;
		double fm, prandtl;
		if (rib > 0.0) {
			prandtl = 1.0;
			// past Ri_c the source's comment says no turbulent
			// mixing (its code sets 1; the comment is followed)
			fm = rib < ric ? ((ric - rib) / ric) * ((ric - rib) / ric) : 0.0;
			fh = fm;
		} else {
			fm = std::sqrt(1.0 - lambda * bm * rib);
			fh = (1.0 / alphah) * std::sqrt(1.0 - lambda * bh * rib) * std::pow(1.0 - lambda * bm * rib, 0.25);
			prandtl = alphah * std::pow(1.0 - lambda * bm * rib, 0.25) / std::sqrt(1.0 - lambda * bh * rib);
		}
		const double re = karman * std::sqrt(fm) * std::sqrt(zu2) * Z0_SURF / (lz * nu);
		z0t_comp = Z0_SURF * std::exp(-karman * 7.3 * std::pow(re, 0.25) * std::sqrt(prandtl) + 5.0 * karman);
		if (std::fabs(z0t - z0t_comp) <= 0.01 * Z0_SURF) {
			break;
		}
	}
	r_ch = chn * fh;
	r_u = std::sqrt(zu2);
}

// Water's condensation temperature (K) at pressure p (Pa) and
// mixing ratio q, LMD tcondwater.F90 (fit to watersat)
double t_cond_water(double p_p, double p_q) {
	const double x = std::log10(std::max(p_q, 1e-16) * p_p / (100.0 * 0.41));
	return 2.52846556e+02 + 2.39229653e+01 * x + 2.21333897e+00 * x * x + 1.79977992e-01 * x * x * x +
			1.00068175e-02 * x * x * x * x + 2.55145012e-04 * x * x * x * x * x;
}

// Growth resistance R_k + R_d (s/m2), LMD growthrate.F after
// Montmessin 2002 sec. 2.2: heat and vapour diffusion
double growth_resistance(double p_t, double p_p, double p_psat, double p_r) {
	const double pi = 3.14159265358979;
	const double k = (0.17913 * p_t - 13.9789) * 4.184e-4;
	const double lv = (2834.3 - 0.28 * (p_t - 273.15) - 0.004 * (p_t - 273.15) * (p_t - 273.15)) * 1.0e3;
	const double rr = R_MCO2 + R_MH2O;
	const double dv = 1.0 / 3.0 * std::sqrt(8.0 * KBZ * p_t / (pi * MH2O / NAV)) * KBZ * p_t /
			(pi * p_p * rr * rr * std::sqrt(1.0 + MH2O / MCO2));
	const double kn = p_t / p_p * (0.707 * RGP / (4.0 * pi * R_MCO2 * R_MCO2 * NAV)) / p_r;
	const double lambda = (1.333 + 0.71 / kn) / (1.0 + 1.0 / kn);
	const double rk = lv * lv * RHO_ICE * MH2O / (k * RGP * p_t * p_t);
	const double rd = RGP * p_t * RHO_ICE / (dv * p_psat * MH2O);
	return rk + rd * (1.0 + lambda * kn);
}

// Godot's helpers, same definitions
double clampd(double p_v, double p_lo, double p_hi) {
	return p_v < p_lo ? p_lo : (p_v > p_hi ? p_hi : p_v);
}
double sstep(double p_a, double p_b, double p_x) {
	if (p_a == p_b) {
		return p_a;
	}
	const double s = clampd((p_x - p_a) / (p_b - p_a), 0.0, 1.0);
	return s * s * (3.0 - 2.0 * s);
}
double lerpd(double p_a, double p_b, double p_t) {
	return p_a + (p_b - p_a) * p_t;
}
// Godot's wrapf: a result within CMP_EPSILON (relative) of the
// top snaps to the bottom; mors's seam behaviour depends on it.
double wrapd(double p_v, double p_min, double p_max) {
	const double range = p_max - p_min;
	if (range == 0.0) {
		return p_min;
	}
	const double r = p_v - range * std::floor((p_v - p_min) / range);
	const double tol = std::max(1e-5 * std::fabs(r), 1e-5);
	return std::fabs(r - p_max) < tol ? p_min : r;
}
double fposmod(double p_x, double p_y) {
	double v = std::fmod(p_x, p_y);
	if ((v < 0.0 && p_y > 0.0) || (v > 0.0 && p_y < 0.0)) {
		v += p_y;
	}
	return v;
}
int posmodi(int p_x, int p_y) {
	const int v = p_x % p_y;
	return v < 0 ? v + p_y : v;
}

double interp_band(const double *p_lats, const double *p_vals, int p_n, double p_lat) {
	if (p_lat <= p_lats[0]) {
		return p_vals[0];
	}
	for (int b = 1; b < p_n; ++b) {
		if (p_lat <= p_lats[b]) {
			const double t = (p_lat - p_lats[b - 1]) / (p_lats[b] - p_lats[b - 1]);
			return lerpd(p_vals[b - 1], p_vals[b], t);
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
	double m = interp_band(lats, p_north ? CAP_N_MASS : CAP_S_MASS, n, l);
	m *= clampd((p_lat_eff - lo + 5.0) / 5.0, 0.0, 1.0);
	const double x = fposmod(p_ls - on, 360.0) + on;
	if (x < on || x > off) {
		return 0.0;
	}
	if (x <= pk) {
		return m * (x - on) / std::max(pk - on, 1.0);
	}
	return m * (off - x) / std::max(off - pk, 1.0);
}

double frost_point(double p_p) {
	return 3182.48 / (27.9546 - std::log(std::max(p_p, 1.0)));
}

double white_flux(double p_tau_s, double p_rho, double p_grav) {
	if (p_tau_s <= TAU_STAR) {
		return 0.0;
	}
	const double r = std::sqrt(TAU_STAR / p_tau_s);
	return 2.61 * (p_tau_s / p_grav) * std::sqrt(p_tau_s / p_rho) * (1.0 - r) * (1.0 + r) * (1.0 + r);
}

double t_equilibrium(double p_lat, double p_dec, double p_flux, double p_alb, double p_solar) {
	const double cosh0 = clampd(-std::tan(p_lat) * std::tan(p_dec), -1.0, 1.0);
	const double h0 = std::acos(cosh0);
	const double q = p_solar * p_flux / PI *
			(std::sin(p_lat) * std::sin(p_dec) * h0 + std::cos(p_lat) * std::cos(p_dec) * std::sin(h0));
	const double t4 = std::max((1.0 - p_alb) * 0.88 * q, 2.0) / (0.85 * EMIS * SIGMA);
	return clampd(std::pow(t4, 0.25), 145.0, 265.0);
}

double clim_max_at(const std::vector<double> &p_clim, double p_ls) {
	if (p_clim.size() != 24) {
		return 1e9;
	}
	const double x = fposmod(p_ls, 360.0) / 15.0 - 0.5;
	const int i0 = posmodi(int(std::floor(x)), 24);
	const int i1 = (i0 + 1) % 24;
	return lerpd(p_clim[size_t(i0)], p_clim[size_t(i1)], x - std::floor(x));
}

} // namespace

template <typename F>
void Gcm::rows(F &&p_fn) {
	if (threads_ <= 1) {
		for (int j = 0; j < GH; ++j) {
			p_fn(j);
		}
		return;
	}
	std::vector<std::thread> pool;
	pool.reserve(size_t(threads_));
	for (int t = 0; t < threads_; ++t) {
		pool.emplace_back([&, t] {
			for (int j = t; j < GH; j += threads_) {
				p_fn(j);
			}
		});
	}
	for (std::thread &t : pool) {
		t.join();
	}
}

Gcm::Gcm(const GcmConfig &p_config) :
		cfg_(p_config),
		GW(p_config.gw),
		GH(p_config.gh),
		N(p_config.gw * p_config.gh),
		LN(p_config.ln),
		W_PX(p_config.px_per_turn),
		H_PX(p_config.px_per_turn * 0.5),
		GCELL_PX(p_config.px_per_turn / p_config.gw),
		// metres per cell along the equator
		GCELL_M(TAU * p_config.radius / p_config.gw),
		LCELL_PX(p_config.lcell_px),
		SOL_S(p_config.sol_s),
		SOL_HOURS(p_config.sol_hours),
		YEAR_SOLS(p_config.year_sols),
		OMEGA(p_config.omega),
		OMEGA_SOL(TAU / p_config.sol_s),
		GRAV(p_config.gravity),
		OBLIQUITY(p_config.obliquity_deg),
		ECC(p_config.eccentricity),
		PERI_LS(p_config.perihelion_ls),
		S_MARS(p_config.solar_flux),
		H_MEAN(p_config.equivalent_depth),
		SCALE_H(p_config.scale_height),
		P_DATUM_MAX(p_config.datum_pressure),
		R_GAS(p_config.r_gas),
		CP_AIR(p_config.cp_air),
		RHO_H(p_config.datum_pressure / p_config.gravity / p_config.equivalent_depth),
		R_DRAG(1.0 / (1.1 * p_config.sol_s)),
		R_HEAT(1.0 / (1.0 * p_config.sol_s)),
		C_DIURNAL(p_config.diurnal_skin),
		DEEP_RATIO(p_config.deep_ratio),
		TT_MEAN_TAU(3.0 * p_config.sol_s) {}

void Gcm::fold_px(double &r_px, double &r_py) const {
	const double m = fposmod(r_py, 2.0 * H_PX);
	if (m >= H_PX) {
		r_px += W_PX * 0.5;
		r_py = 2.0 * H_PX - m;
	} else {
		r_py = m;
	}
}

int Gcm::index(double p_px, double p_py) const {
	fold_px(p_px, p_py);
	const int i = posmodi(int(p_px / GCELL_PX), GW);
	const int j = std::clamp(int(p_py / GCELL_PX), 0, GH - 1);
	return j * GW + i;
}

double Gcm::sample(const Field &p_f, double p_px, double p_py) const {
	fold_px(p_px, p_py);
	const double fx = wrapd(p_px / GCELL_PX - 0.5, 0.0, double(GW));
	const double fy = clampd(p_py / GCELL_PX - 0.5, 0.0, double(GH) - 1.001);
	const int x0 = int(fx), y0 = int(fy);
	const int x1 = (x0 + 1) % GW;
	const int y1 = std::min(y0 + 1, GH - 1);
	return lerpd(lerpd(p_f[size_t(y0 * GW + x0)], p_f[size_t(y0 * GW + x1)], fx - x0),
			lerpd(p_f[size_t(y1 * GW + x0)], p_f[size_t(y1 * GW + x1)], fx - x0), fy - y0);
}

void Gcm::create(const float *p_topo, const float *p_lum, double p_ls, int p_threads) {
	threads_ = std::max(1, p_threads);
	for (Field *f : { &u, &v, &h, &wv, &dust, &dustc, &frost, &soil, &ts, &ta, &tdeep, &tmean, &ttm, &ttf, &co2, &wfr,
				 &wlf, &wdu, &wlc, &wdc, &psurf, &tair, &wsx, &wsy, &wconv, &topo, &pref, &alb, &cap, &wbc, &rel_, &nu_,
				 &nv_, &nh_, &tau_low, &tau_high, &cloud_frac, &d_slope, &d_edge, &d_had, &d_eddy, &d_stress, &d_lift_s,
				 &d_lift_d, &bl_x, &bl_y, &d_front, &d_tail }) {
		f->assign(N, 0.0f);
	}
	for (int l = 0; l < NL; ++l) {
		vap[size_t(l)].assign(N, 0.0f);
		ice[size_t(l)].assign(N, 0.0f);
		tau[size_t(l)].assign(N, 0.0f);
		frac[size_t(l)].assign(N, 0.0f);
	}
	for (int t = 0; t < NT; ++t) {
		adv_[size_t(t)].assign(N, 0.0f);
		nxt_[size_t(t)].assign(N, 0.0f);
	}
	for (Field &q : l_tau) {
		q.assign(size_t(LN) * LN, 0.0f);
	}
	l_dust.assign(size_t(LN) * LN, 0.0f);
	l_valid_ = false;
	lat_r_.assign(GH, 0.0);
	fcor_.assign(GH, 0.0);
	inv_dx_.assign(GH, 0.0);
	inv_dxt_.assign(GH, 0.0);
	cor_c_.assign(GH, 0.0);
	cor_s_.assign(GH, 0.0);
	carea_.assign(GH, 0.0);
	// The row and column tables hold float32 values, as mors's
	// PackedFloat32Array ones do (the cap edge is that sharp).
	auto f32 = [](double p_v) { return double(float(p_v)); };
	for (int j = 0; j < GH; ++j) {
		// derived from the double latitude, each stored float
		const double lat = (90.0 - (j + 0.5) / double(GH) * 180.0) * DEG;
		lat_r_[size_t(j)] = f32(lat);
		fcor_[size_t(j)] = f32(2.0 * OMEGA * std::sin(lat));
		inv_dx_[size_t(j)] = f32(1.0 / (GCELL_M * std::max(std::cos(lat), 0.25)));
		inv_dxt_[size_t(j)] = f32(1.0 / (GCELL_M * std::max(std::cos(lat), 0.008)));
		cor_c_[size_t(j)] = f32(std::cos(fcor_[size_t(j)] * DT));
		cor_s_[size_t(j)] = f32(std::sin(fcor_[size_t(j)] * DT));
		carea_[size_t(j)] = f32(std::max(std::cos(lat), 0.004));
	}
	cos_lon_.assign(GW, 0.0);
	sin_lon_.assign(GW, 0.0);
	for (int i = 0; i < GW; ++i) {
		const double lon = (i + 0.5) / double(GW) * TAU - PI;
		cos_lon_[size_t(i)] = f32(std::cos(lon));
		sin_lon_[size_t(i)] = f32(std::sin(lon));
	}
	for (int k = 0; k < N; ++k) {
		topo[size_t(k)] = p_topo ? p_topo[k] : 0.0f;
		pref[size_t(k)] = float(P_DATUM_MAX * std::exp(-double(topo[size_t(k)]) / SCALE_H));
	}
	// albedo and heat capacity from the mosaic luminance
	{
		double lo = 0.0, hi = 1.0;
		if (p_lum) {
			std::vector<float> sorted(p_lum, p_lum + N);
			std::sort(sorted.begin(), sorted.end());
			lo = sorted[size_t(N * 0.05)];
			hi = std::max(double(sorted[size_t(N * 0.95)]), lo + 1e-3);
		}
		for (int j = 0; j < GH; ++j) {
			const double latd = std::fabs(lat_r_[size_t(j)] * RAD2DEG);
			for (int i = 0; i < GW; ++i) {
				const int k = j * GW + i;
				const double n = p_lum ? clampd((p_lum[k] - lo) / (hi - lo), 0.0, 1.0) : 0.5;
				alb[size_t(k)] = float(0.11 + 0.20 * n);
				double inertia = lerpd(380.0, 70.0, n);
				inertia = lerpd(inertia, 600.0, sstep(75.0, 84.0, latd));
				cap[size_t(k)] = float(inertia * C_DIURNAL);
			}
		}
	}
	// deep mobile-dust reservoirs in the basins
	struct Basin {
		double lat, lon, r, depth;
	};
	const Basin basins[5] = { { -42.0, 70.0, 12.0, 2.4 }, { -50.0, -43.0, 9.0, 2.0 }, { 25.0, -40.0, 9.0, 1.8 },
		{ -26.0, -90.0, 10.0, 2.0 }, { 12.0, 87.0, 8.0, 1.6 } };
	const double ls_rad = p_ls * DEG;
	const double dec = std::asin(std::sin(OBLIQUITY * DEG) * std::sin(ls_rad));
	const double flux = std::pow((1.0 + ECC * std::cos(ls_rad - PERI_LS * DEG)) / (1.0 - ECC * ECC), 2.0);
	double msum = 0.0, asum = 0.0;
	for (int j = 0; j < GH; ++j) {
		const double latd = std::fabs(lat_r_[size_t(j)] * RAD2DEG);
		const double latv = lat_r_[size_t(j)] * RAD2DEG;
		double prow = 0.0;
		for (int i = 0; i < GW; ++i) {
			prow += pref[size_t(j * GW + i)];
		}
		const double tfr_row = frost_point(prow / GW);
		for (int i = 0; i < GW; ++i) {
			const size_t k = size_t(j * GW + i);
			// the perennial north cap's water ice, a reservoir
			frost[k] = latv > NORTH_CAP_LAT ? float(NORTH_CAP_ICE) : 0.0f;
			double s = 50.0;
			for (const Basin &b : basins) {
				const double bx = (b.lon + 180.0) / 360.0 * GW;
				const double by = (90.0 - b.lat) / 180.0 * GH;
				const double dx = std::min(std::fabs(i - bx), GW - std::fabs(i - bx));
				const double d2 = (dx * dx + (j - by) * (j - by)) / (b.r * b.r);
				s += 10.0 * b.depth * std::exp(-d2);
			}
			soil[k] = float(s);
			dust[k] = float(0.05 * (0.4 + 0.6 * std::cos(lat_r_[size_t(j)])));
			const double lat_eff = latd + CAP_LAT_PER_K * (frost_point(pref[k]) - tfr_row);
			co2[k] = float(cap_mass(lat_eff, p_ls, latv > 0.0));
			msum += co2[k] * carea_[size_t(j)];
			asum += carea_[size_t(j)];
			const double teq = t_equilibrium(lat_r_[size_t(j)], dec, flux, alb[k], S_MARS);
			ts[k] = float(teq);
			tdeep[k] = float(teq);
			tmean[k] = float(teq);
			ta[k] = float(teq - 12.0);
			ttm[k] = float(ta[k] + TIDE_GROUND_W * (ts[k] - ta[k]));
			ttf[k] = ttm[k];
		}
	}
	const double hbar0 = -msum / asum / RHO_H;
	for (int k = 0; k < N; ++k) {
		h[size_t(k)] = float(hbar0);
		if (co2[size_t(k)] > 0.0f) {
			const double t_fr = frost_point(pref[size_t(k)] * (1.0 + hbar0 / H_MEAN));
			ts[size_t(k)] = float(t_fr);
			tdeep[size_t(k)] = float(t_fr);
			tmean[size_t(k)] = float(t_fr);
			ta[size_t(k)] = float(std::min(double(ta[size_t(k)]), t_fr + 40.0));
			ttm[size_t(k)] = float(ta[size_t(k)] + TIDE_GROUND_W * (ts[size_t(k)] - ta[size_t(k)]));
			ttf[size_t(k)] = ttm[size_t(k)];
		}
	}
	// the observed column (TES, M04 Fig. 2) at this season, at
	// one mixing ratio, held to what each level holds
	for (int k = 0; k < N; ++k) {
		const double latv = lat_r_[size_t(k / GW)] * RAD2DEG;
		const double p_s = pref[size_t(k)] * (1.0 + hbar0 / H_MEAN);
		Column cl;
		column_levels(p_s, ta[size_t(k)], R_GAS, GRAV, cl);
		double air_tot = 0.0;
		for (int l = 0; l < NL; ++l) {
			air_tot += cl.air[l];
		}
		const double q0 = tes_column(latv, p_ls) / air_tot;
		double col = 0.0;
		for (int l = 0; l < NL; ++l) {
			const double held = std::min(q0, M_RATIO * e_ice(cl.t[l]) / cl.p[l]) * cl.air[l];
			vap[size_t(l)][size_t(k)] = float(held);
			col += held;
		}
		wv[size_t(k)] = float(col);
	}
	// western boundary currents (Joshi 1995; Newman 2002)
	for (int j = 0; j < GH; ++j) {
		double row_sum = 0.0;
		for (int i = 0; i < GW; ++i) {
			double hi_west = -1e9;
			for (int d = 1; d < 16; ++d) {
				hi_west = std::max(hi_west, double(topo[size_t(j * GW + posmodi(i - d, GW))]));
			}
			const double rise = hi_west - topo[size_t(j * GW + i)];
			wbc[size_t(j * GW + i)] = float(1.0 + WBC_GAIN * sstep(1000.0, 5000.0, rise));
			row_sum += wbc[size_t(j * GW + i)];
		}
		const double norm = double(GW) / std::max(row_sum, 1e-9);
		for (int i = 0; i < GW; ++i) {
			wbc[size_t(j * GW + i)] = float(wbc[size_t(j * GW + i)] * norm);
		}
	}
	for (std::vector<double> *r : { &row_h_, &row_tt_, &row_ts_, &row_tm_, &row_ms_, &row_dm_, &row_wlf_, &row_wlc_ }) {
		r->assign(GH, 0.0);
	}
	row_fix_.assign(size_t(GH) * 2 * NT, 0.0);
	bc_amp_.assign(GH, 0.0);
	hbar = hbar0;
	cap_scale = 1.0;
	ttbar = 0.0;
	ttmbar = 200.0;
	msbar = 0.0;
	ttbar_set = false;
	bc_phase = { 0.0, 2.0 };
	bc_period = { 4.0, 4.0 };
	bc_m = { 3, 2 };
	bc_timer = 0.0;
}

bool Gcm::finite() const {
	const Field *probes[] = { &u, &v, &h, &wv, &dust, &ice[0], &ice[NL - 1], &frost, &ts, &ta, &co2 };
	for (const Field *f : probes) {
		const double s = double((*f)[0]) + (*f)[N / 3] + (*f)[N / 2] + (*f)[2 * N / 3] + (*f)[N - 1];
		if (!std::isfinite(s)) {
			return false;
		}
	}
	return true;
}

void Gcm::reset_dynamics() {
	const double hb = std::isfinite(hbar) ? hbar : 0.0;
	for (int k = 0; k < N; ++k) {
		u[size_t(k)] = 0.0f;
		v[size_t(k)] = 0.0f;
		h[size_t(k)] = float(hb);
	}
}

void Gcm::inject_dust(double p_px, double p_py) {
	const int bi = std::clamp(int(p_px / GCELL_PX), 2, GW - 3);
	const int bj = std::clamp(int(p_py / GCELL_PX), 2, GH - 3);
	for (int dj = -2; dj <= 2; ++dj) {
		for (int di = -2; di <= 2; ++di) {
			const size_t kk = size_t((bj + dj) * GW + bi + di);
			const double want = std::max(1.1 - 0.2 * (std::abs(di) + std::abs(dj)) - dust[kk] - dustc[kk], 0.0);
			const double take = std::min(want, double(soil[kk]));
			soil[kk] = float(soil[kk] - take);
			dust[kk] = float(dust[kk] + take * (1.0 - COARSE_FRAC));
			dustc[kk] = float(dustc[kk] + take * COARSE_FRAC);
		}
	}
}

void Gcm::step(const Inputs &p_in, bool p_spinup) {
	step_waves(p_in);
	step_dynamics();
	step_tracers(p_in, p_spinup);
}

// baroclinic storm track (sec. 6.1)
void Gcm::step_waves(const Inputs &p_in) {
	bc_timer += DT;
	if (bc_timer > 5.0 * SOL_S) {
		bc_timer = 0.0;
		std::uniform_real_distribution<double> unit(0.0, 1.0);
		for (int hemi = 0; hemi < 2; ++hemi) {
			bc_period[size_t(hemi)] = 2.0 + 5.0 * unit(rng_);
			if (unit(rng_) < 0.3) {
				bc_m[size_t(hemi)] = bc_m[size_t(hemi)] == 2 ? 3 : 2;
			}
		}
	}
	for (int hemi = 0; hemi < 2; ++hemi) {
		bc_phase[size_t(hemi)] = fposmod(bc_phase[size_t(hemi)] - TAU * DT / (bc_period[size_t(hemi)] * SOL_S), TAU);
	}
	if (!ttbar_set) {
		return;
	}
	const double ls = p_in.ls;
	for (int j = 0; j < GH; ++j) {
		const int jm = std::max(j - 1, 0), jp = std::min(j + 1, GH - 1);
		double grad;
		if (p_in.wave_from_column) {
			const double tp = row_tm_[size_t(jp)] / (carea_[size_t(jp)] * GW);
			const double tm = row_tm_[size_t(jm)] / (carea_[size_t(jm)] * GW);
			grad = std::fabs(tp - tm) / (double(jp - jm) * GCELL_M);
		} else {
			grad = std::fabs(row_ts_[size_t(jp)] - row_ts_[size_t(jm)]) / (double(jp - jm) * GCELL_M);
		}
		const double latd = std::fabs(lat_r_[size_t(j)] * RAD2DEG);
		const double win = sstep(28.0, 42.0, latd) * (1.0 - sstep(70.0, 80.0, latd));
		const double solstice = lat_r_[size_t(j)] > 0.0 ? 270.0 : 90.0;
		const double dls = std::fabs(fposmod(ls - solstice + 180.0, 360.0) - 180.0);
		const double pause = 1.0 - PAUSE_DEPTH * (1.0 - sstep(PAUSE_IN, PAUSE_OUT, dls));
		const double hemi_k = lat_r_[size_t(j)] > 0.0 ? 1.0 : SOUTH_EDDY;
		bc_amp_[size_t(j)] = std::min(BC_K * grad, BC_AMP_MAX) * win * pause * hemi_k;
	}
}

// shallow water, forward-backward in time
void Gcm::step_dynamics() {
	const double inv_dy = 1.0 / GCELL_M;
	const double hb = hbar, ttb = ttbar, ttmb = ttmbar, msb = msbar;
	const bool tt_ok = ttbar_set;
	rows([&](int j) {
		const int jrm = std::max(j - 1, 0), jrp = std::min(j + 1, GH - 1);
		const int jm = jrm * GW, jp = jrp * GW;
		const int j0 = j * GW;
		const double idx = inv_dx_[size_t(j)];
		const double cm = carea_[size_t(jrm)] / carea_[size_t(j)];
		const double cp = carea_[size_t(jrp)] / carea_[size_t(j)];
		// flux form: face fluxes are the mean of the two rows'
		// v cos(lat); none crosses a pole, so mass is kept
		const double f_n = j > 0 ? 0.5 : 0.0, f_s = j < GH - 1 ? 0.5 : 0.0;
		const double taper = 1.0 - sstep(60.0, 85.0, std::fabs(lat_r_[size_t(j)] * RAD2DEG));
		const int hemi = lat_r_[size_t(j)] > 0.0 ? 0 : 1;
		const double amp = bc_amp_[size_t(j)];
		const double mw = bc_m[size_t(hemi)];
		const double ph = bc_phase[size_t(hemi)];
		for (int i = 0; i < GW; ++i) {
			const size_t k = size_t(j0 + i);
			const size_t im = size_t(j0 + (i + GW - 1) % GW);
			const size_t ip = size_t(j0 + (i + 1) % GW);
			double heq = hb;
			const double dtot = double(dust[k]) + dustc[k];
			if (tt_ok) {
				const double anom = clampd(double(ttf[k]) - ttm[k], -TT_ANOM_MAX, TT_ANOM_MAX) * taper;
				heq += H_MEAN * (anom - ttb) / T_REF;
				const double ms = clampd(ttm[k] - ttmb, -MEAN_STATE_MAX, MEAN_STATE_MAX) * taper;
				heq += MEAN_STATE_W * H_MEAN * (ms - msb) / T_REF;
			}
			if (amp > 0.0) {
				const double lon = (i + 0.5) / double(GW) * TAU;
				heq += amp * std::cos(mw * lon - ph);
			}
			const double vk = v[k];
			const double flux_s = f_s * (vk + double(v[size_t(jp + i)]) * cp);
			const double flux_n = f_n * (vk + double(v[size_t(jm + i)]) * cm);
			const double div = (double(u[ip]) - u[im]) * 0.5 * idx + (flux_s - flux_n) * inv_dy;
			const double rh = R_HEAT * (1.0 + 20.0 * std::min(dtot, 1.2));
			rel_[k] = float(-rh * (h[k] - heq));
			nh_[k] = float(h[k] + DT * (-H_MEAN * div));
		}
	});
	// The relaxation moves thickness toward the heating; its
	// rate varies with the dust, so its planet mean (a mass
	// source) is taken out.
	double rel_sum = 0.0, area_sum = 0.0;
	for (int j = 0; j < GH; ++j) {
		double s = 0.0;
		for (int i = 0; i < GW; ++i) {
			s += rel_[size_t(j * GW + i)];
		}
		rel_sum += s * carea_[size_t(j)];
		area_sum += carea_[size_t(j)] * GW;
	}
	const double rel_mean = rel_sum / area_sum;
	for (int k = 0; k < N; ++k) {
		nh_[size_t(k)] = float(nh_[size_t(k)] + DT * (rel_[size_t(k)] - rel_mean));
	}
	rows([&](int j) {
		const int jm = std::max(j - 1, 0) * GW, jp = std::min(j + 1, GH - 1) * GW;
		const double dyf = (j > 0 && j < GH - 1) ? 0.5 : 1.0;
		const int j0 = j * GW;
		const double idx = inv_dx_[size_t(j)];
		const double ca = cor_c_[size_t(j)], sa = cor_s_[size_t(j)];
		for (int i = 0; i < GW; ++i) {
			const size_t k = size_t(j0 + i);
			const size_t im = size_t(j0 + (i + GW - 1) % GW);
			const size_t ip = size_t(j0 + (i + 1) % GW);
			const double fx = wrapd(double(i) - u[k] * DT * idx, 0.0, double(GW));
			const double fy = clampd(double(j) - v[k] * DT * inv_dy, 0.0, double(GH) - 1.001);
			const int x0 = int(fx), y0 = int(fy);
			const double tx = fx - x0, ty = fy - y0;
			const int x1 = (x0 + 1) % GW, y1 = std::min(y0 + 1, GH - 1);
			const size_t a = size_t(y0 * GW + x0), b = size_t(y0 * GW + x1), c = size_t(y1 * GW + x0),
						 d = size_t(y1 * GW + x1);
			const double w00 = (1 - tx) * (1 - ty), w10 = tx * (1 - ty), w01 = (1 - tx) * ty, w11 = tx * ty;
			const double ub = u[a] * w00 + u[b] * w10 + u[c] * w01 + u[d] * w11;
			const double vb = v[a] * w00 + v[b] * w10 + v[c] * w01 + v[d] * w11;
			const double ur = ub * ca - vb * sa;
			const double vr = ub * sa + vb * ca;
			const double dtx = (double(topo[ip]) - topo[im]) * 0.5 * idx;
			const double dty = (double(topo[size_t(jp + i)]) - topo[size_t(jm + i)]) * dyf * inv_dy;
			const double gphix = GRAV * ((double(nh_[ip]) - nh_[im]) * 0.5 * idx + TOPO_ALPHA * dtx);
			const double gphiy = GRAV * ((double(nh_[size_t(jp + i)]) - nh_[size_t(jm + i)]) * dyf * inv_dy + TOPO_ALPHA * dty);
			const double spd = std::sqrt(ub * ub + vb * vb);
			const double r_eff = R_DRAG * (1.0 + spd / 45.0);
			nu_[k] = float(ur + DT * (-gphix - r_eff * ur));
			nv_[k] = float(vr + DT * (-gphiy - r_eff * vr));
		}
	});
	rows([&](int j) {
		const int j0 = j * GW;
		const double latd = std::fabs(lat_r_[size_t(j)] * RAD2DEG);
		if (latd > 78.0) {
			double mu = 0.0, mv = 0.0;
			for (int i = 0; i < GW; ++i) {
				mu += nu_[size_t(j0 + i)];
				mv += nv_[size_t(j0 + i)];
			}
			mu /= GW;
			mv /= GW;
			const double w = sstep(78.0, 89.0, latd);
			for (int i = 0; i < GW; ++i) {
				nu_[size_t(j0 + i)] = float(lerpd(nu_[size_t(j0 + i)], mu, w));
				nv_[size_t(j0 + i)] = float(lerpd(nv_[size_t(j0 + i)], mv, w));
			}
		}
		for (int i = 0; i < GW; ++i) {
			const size_t k = size_t(j0 + i);
			u[k] = float(clampd(nu_[k], -90.0, 90.0));
			v[k] = float(clampd(nv_[k], -90.0, 90.0));
			h[k] = nh_[k];
		}
	});
}

void Gcm::shift_profile(Field &r_arr, const std::vector<double> &p_had_up, double p_dust_k, double p_dt) {
	std::vector<double> prof(GH), shifted(GH);
	for (int j = 0; j < GH; ++j) {
		double m = 0.0;
		for (int i = 0; i < GW; ++i) {
			m += r_arr[size_t(j * GW + i)];
		}
		prof[size_t(j)] = m / GW;
	}
	double mass0 = 0.0, mass1 = 0.0;
	for (int j = 0; j < GH; ++j) {
		const double src = clampd(double(j) - p_had_up[size_t(j)] * (p_dust_k - 1.0) * p_dt / GCELL_M, 0.0, double(GH) - 1.001);
		const int j0 = int(src);
		const double t = src - j0;
		shifted[size_t(j)] = lerpd(prof[size_t(j0)], prof[size_t(std::min(j0 + 1, GH - 1))], t);
		mass0 += prof[size_t(j)] * carea_[size_t(j)];
		mass1 += shifted[size_t(j)] * carea_[size_t(j)];
	}
	const double norm = mass0 / std::max(mass1, 1e-12);
	for (int j = 0; j < GH; ++j) {
		const double delta = shifted[size_t(j)] * norm - prof[size_t(j)];
		if (std::fabs(delta) < 1e-9) {
			continue;
		}
		for (int i = 0; i < GW; ++i) {
			const size_t k = size_t(j * GW + i);
			r_arr[k] = float(std::max(r_arr[k] + delta, 0.0));
		}
	}
}

// tracer advection, surface energy balance, CO2 and water
// cycles, dust lifting and settling
void Gcm::step_tracers(const Inputs &p_in, bool p_spinup) {
	const double ls_rad = p_in.ls * DEG;
	const double subsolar_lat = std::asin(std::sin(OBLIQUITY * DEG) * std::sin(ls_rad));
	const double ssl = (0.5 - p_in.mtc / SOL_HOURS) * TAU;
	const double cs = std::cos(ssl), sn = std::sin(ssl);
	const double flux = std::pow((1.0 + ECC * std::cos(ls_rad - PERI_LS * DEG)) / (1.0 - ECC * ECC), 2.0);
	const double sdec = std::sin(subsolar_lat), cdec = std::cos(subsolar_lat);
	const double season_k = p_in.year_sols > 0.0 ? YEAR_SOLS / std::max(p_in.year_sols, 1.0) : 1.0;
	const double dust_k = p_in.dust_k >= 0.0 ? p_in.dust_k : season_k;
	const double edge_gain = p_in.edge_k >= 0.0 ? p_in.edge_k : EDGE_K;
	const double front_wind = p_in.front_wind >= 0.0 ? p_in.front_wind : FRONT_WIND;
	const double had_dust_k = p_in.had_dust_k >= 0.0 ? p_in.had_dust_k : HAD_DUST_K;
	const double had_v_upper = p_in.had_v_upper >= 0.0 ? p_in.had_v_upper : HAD_V_UPPER;
	const double had_surf = p_in.had_surf;
	const double wbc_scale = p_in.wbc_scale;
	const double wk = p_in.weibull_k > 0.0 ? p_in.weibull_k : WEIBULL_K;
	double wq[8];
	for (int q = 0; q < 8; ++q) {
		wq[q] = std::pow(-std::log(1.0 - (q + 0.5) / 8.0), 1.0 / wk);
	}
	const double wp99 = std::pow(-std::log(0.01), 1.0 / wk);
	const double dt = DT;
	// the advected tracers, in T_* order
	std::array<Field *, NT> tr;
	tr[T_DU] = &dust;
	tr[T_DC] = &dustc;
	for (int l = 0; l < NL; ++l) {
		tr[size_t(T_VAP + l)] = &vap[size_t(l)];
		tr[size_t(T_ICE + l)] = &ice[size_t(l)];
	}

	// area-weighted planet-mean dust, every 7th cell
	double dmean = 0.0, dmean_a = 0.0;
	for (int k = 0; k < N; k += 7) {
		const double wj = carea_[size_t(k / GW)];
		dmean += (double(dust[size_t(k)]) + dustc[size_t(k)]) * wj;
		dmean_a += wj;
	}
	dmean /= std::max(dmean_a, 1e-9);
	const double tau_glob = TAU_FLOOR + TAU_PER_DUST * dmean;
	const double obs_max = clim_max_at(clim_max, p_in.ls);
	cap_scale = 1.0 - sstep(obs_max * CAP_RAMP, obs_max, tau_glob);
	const double cap_sc = cap_scale;
	const double s_sum = subsolar_lat > 0.0 ? 1.0 : -1.0;
	const double peak_hemi = subsolar_lat > 0.0 ? 0.55 : 1.0;
	const double tilt = std::pow(std::fabs(std::sin(subsolar_lat)) / std::sin(OBLIQUITY * DEG), 1.5);
	const double had_s = (HAD_EQUINOX + (peak_hemi - HAD_EQUINOX) * tilt) * clampd(1.0 + had_dust_k * (tau_glob - 0.3), 1.0, 2.1);
	std::vector<double> ubar(GH), vbar(GH), had_u(GH), had_v(GH), had_up(GH);
	for (int j = 0; j < GH; ++j) {
		double su = 0.0, sv = 0.0;
		for (int i = 0; i < GW; ++i) {
			su += u[size_t(j * GW + i)];
			sv += v[size_t(j * GW + i)];
		}
		ubar[size_t(j)] = su / GW;
		vbar[size_t(j)] = sv / GW;
	}
	for (int j = 0; j < GH; ++j) {
		const double psi = s_sum * lat_r_[size_t(j)] * RAD2DEG;
		const double pv = sstep(-40.0, -20.0, psi) * (1.0 - sstep(20.0, 35.0, psi)) *
				(0.6 + 0.4 * std::exp(-std::pow((psi - 5.0) / 15.0, 2.0)));
		had_v[size_t(j)] = -s_sum * HAD_V_SURF * had_s * pv;
		had_up[size_t(j)] = s_sum * had_v_upper * had_s * std::exp(-std::pow((psi + 5.0) / 25.0, 2.0));
		had_u[size_t(j)] = had_s *
				(HAD_JET * std::exp(-std::pow((psi - HAD_JET_LAT) / 8.0, 2.0)) - HAD_EASTERLY * std::exp(-std::pow(psi / 20.0, 2.0)) +
						HAD_WINTER_W * std::exp(-std::pow((psi + 50.0) / 12.0, 2.0)));
	}
	const double ttmb = ttmbar;
	// The edge breeze reads neighbouring rows' sol-mean ground
	// temperature, which this pass rewrites: read last step's
	// (in mors the parallel rows race on it).
	const Field tmean_prev = tmean;
	rows([&](int j) {
		const int j0 = j * GW;
		const double clat = std::cos(lat_r_[size_t(j)]), slat = std::sin(lat_r_[size_t(j)]);
		const double latd = std::fabs(lat_r_[size_t(j)] * RAD2DEG);
		const double dyf = (j > 0 && j < GH - 1) ? 0.5 : 1.0;
		const double area = carea_[size_t(j)];
		const double taper = 1.0 - sstep(60.0, 85.0, latd);
		double sum_h = 0, sum_tt = 0, sum_ts = 0, sum_tm = 0, sum_ms = 0, sum_dm = 0, sum_wlf = 0, sum_wlc = 0;
		double pre[NT] = {}, post[NT] = {};
		const double idxt = inv_dxt_[size_t(j)];
		for (int i = 0; i < GW; ++i) {
			const size_t k = size_t(j0 + i);
			for (int t = 0; t < NT; ++t) {
				pre[t] += (*tr[size_t(t)])[k];
			}
			// backtrace, true metric; reflect over the poles
			auto trace = [&](double p_u, double p_v, size_t r_idx[4], double r_w[4]) {
				double fx = double(i) - p_u * dt * idxt;
				double fy = double(j) - p_v * dt / GCELL_M;
				if (fy < -0.5) {
					fy = -1.0 - fy;
					fx += GW * 0.5;
				} else if (fy > double(GH) - 0.5) {
					fy = 2.0 * double(GH) - 1.0 - fy;
					fx += GW * 0.5;
				}
				fx = wrapd(fx, 0.0, double(GW));
				fy = clampd(fy, 0.0, double(GH) - 1.001);
				const int x0 = int(fx), y0 = int(fy);
				const double tx = fx - x0, ty = fy - y0;
				const int x1 = (x0 + 1) % GW, y1 = std::min(y0 + 1, GH - 1);
				r_idx[0] = size_t(y0 * GW + x0);
				r_idx[1] = size_t(y0 * GW + x1);
				r_idx[2] = size_t(y1 * GW + x0);
				r_idx[3] = size_t(y1 * GW + x1);
				r_w[0] = (1 - tx) * (1 - ty);
				r_w[1] = tx * (1 - ty);
				r_w[2] = (1 - tx) * ty;
				r_w[3] = tx * ty;
			};
			auto bil = [](const Field &p_f, const size_t p_i[4], const double p_w[4]) {
				return p_f[p_i[0]] * p_w[0] + p_f[p_i[1]] * p_w[1] + p_f[p_i[2]] * p_w[2] + p_f[p_i[3]] * p_w[3];
			};
			size_t ia[4], id[4];
			double wa[4], wd[4];
			trace(u[k], v[k] + had_up[size_t(j)], ia, wa);
			// dust rides the boundary layer that lifted it [M]
			trace(u[k] + BL_DUST_W * bl_x[k], v[k] + had_up[size_t(j)] + BL_DUST_W * bl_y[k], id, wd);
			double a[NT];
			for (int t = 0; t < NT; ++t) {
				const bool dusty = t == T_DU || t == T_DC;
				a[t] = bil(*tr[size_t(t)], dusty ? id : ia, dusty ? wd : wa);
				post[t] += a[t];
				adv_[size_t(t)][k] = float(a[t]);
			}
			double &ad = a[T_DU], &ac = a[T_DC];
			// insolation: cos(solar zenith), orbit-scaled
			const double cosha = cos_lon_[size_t(i)] * cs + sin_lon_[size_t(i)] * sn;
			const double mu0 = std::max(slat * sdec + clat * cdec * cosha, 0.0);
			const double s_toa = S_MARS * flux * mu0;
			const double dtot = ad + ac;
			const double tau_v = TAU_FLOOR + TAU_PER_DUST * dtot;
			const double p_s = pref[k] * (H_MEAN + h[k] + TOPO_ALPHA * topo[k]) / H_MEAN;
			const double t_s = ts[k];
			const double t_a = ta[k];
			const double t_air = t_a + 0.55 * (t_s - t_a);
			const double rho = p_s / (R_GAS * t_air);
			const double t_sw = std::exp(-SW_K * tau_v / std::max(mu0, 0.15));
			const double sw_ground = s_toa * t_sw;
			const double sw_col = s_toa * (1.0 - t_sw) * SW_ABS;
			const double tau_r = (p_s / 610.0) * (0.11 + 0.5 * tau_v);
			const double tau_e = tau_r * clampd(1.50307 - 0.121687 * std::log(tau_r), 1.0, 2.0);
			const double beta_e = 1.0 - std::exp(-tau_e);
			const double ts4 = t_s * t_s * t_s * t_s;
			const double ta4 = t_a * t_a * t_a * t_a;
			const double r_down = SIGMA * ta4 * beta_e;
			double albedo = alb[k];
			if (co2[k] > 0.5f) {
				albedo = 0.60;
			} else if (frost[k] > FROST_VISIBLE) {
				albedo = std::max(albedo, ALBEDO_ICE);
			}
			// slope wind (sec. 6.2)
			const size_t im2 = size_t(j0 + (i + 1) % GW);
			const size_t im3 = size_t(j0 + (i + GW - 1) % GW);
			const size_t jpk = size_t(std::min(j + 1, GH - 1) * GW + i);
			const size_t jmk = size_t(std::max(j - 1, 0) * GW + i);
			const double gtx = (double(topo[im2]) - topo[im3]) * 0.5 * idxt;
			const double gty = (double(topo[jpk]) - topo[jmk]) * dyf / GCELL_M;
			const double buoy = SLOPE_GAIN * (t_s - tmean[k]) / T_REF;
			double sx = buoy * gtx, sy = buoy * gty;
			const double smag = std::sqrt(sx * sx + sy * sy);
			if (smag > SLOPE_MAX) {
				sx *= SLOPE_MAX / smag;
				sy *= SLOPE_MAX / smag;
			}
			// storm front: outflow down the dust gradient
			const double fk = -FRONT_K * (0.3 + 0.7 * mu0);
			double fx2 = fk * (double(dust[im2]) + dustc[im2] - dust[im3] - dustc[im3]) * 0.5;
			double fy2 = fk * (double(dust[jpk]) + dustc[jpk] - dust[jmk] - dustc[jmk]) * 0.5;
			const double fmag = std::sqrt(fx2 * fx2 + fy2 * fy2);
			if (fmag > FRONT_MAX) {
				fx2 *= FRONT_MAX / fmag;
				fy2 *= FRONT_MAX / fmag;
			}
			// cap-edge sea breeze on the sol-mean ground temperature
			const double tsk = tmean_prev[k];
			const double gx1 = tsk - tmean_prev[im3], gx2 = tmean_prev[im2] - tsk;
			const double gy1 = tsk - tmean_prev[jmk], gy2 = tmean_prev[jpk] - tsk;
			const double gx = std::fabs(gx1) > std::fabs(gx2) ? gx1 : gx2;
			const double gy = std::fabs(gy1) > std::fabs(gy2) ? gy1 : gy2;
			double ex = -edge_gain * gx * idxt;
			double ey = -edge_gain * gy * 2.0 * dyf / GCELL_M;
			const double emag = std::sqrt(ex * ex + ey * ey);
			if (emag > EDGE_MAX) {
				ex *= EDGE_MAX / emag;
				ey *= EDGE_MAX / emag;
			}
			const double day_f = sstep(0.15, 0.45, mu0);
			ex *= day_f;
			ey *= day_f;
			// the storm-track wave's cold front
			const int hemi_t = lat_r_[size_t(j)] > 0.0 ? 0 : 1;
			const double fr_amp = bc_amp_[size_t(j)];
			double fwx = 0.0, fwy = 0.0;
			if (fr_amp > 1.0) {
				const double lonr = (i + 0.5) / double(GW) * TAU;
				const double argw = double(bc_m[size_t(hemi_t)]) * lonr - bc_phase[size_t(hemi_t)];
				const double dph = fposmod(argw - PI * 0.5 + PI, TAU) - PI;
				const double fr = std::exp(-std::pow(dph / FRONT_PHASE_W, 2.0));
				const double fw = front_wind * (fr_amp / BC_AMP_MAX) * fr;
				const double eq_dir = lat_r_[size_t(j)] > 0.0 ? 1.0 : -1.0;
				fwx = fw * 0.5;
				fwy = fw * 0.85 * eq_dir;
			}
			const double wbc_k = 1.0 + (wbc[k] - 1.0) * wbc_scale;
			wsx[k] = float(sx + fx2 + ex + fwx + had_u[size_t(j)] * had_surf - SURF_WIND * ubar[size_t(j)]);
			wsy[k] = float(sy + fy2 + ey + fwy + had_v[size_t(j)] * wbc_k * had_surf - SURF_WIND * vbar[size_t(j)]);
			d_slope[k] = float(std::sqrt(sx * sx + sy * sy));
			d_edge[k] = float(std::sqrt(ex * ex + ey * ey));
			bl_x[k] = float(sx + ex + fwx);
			bl_y[k] = float(sy + ey + fwy);
			d_front[k] = float(std::sqrt(fwx * fwx + fwy * fwy));
			d_had[k] = float(had_surf * std::sqrt(had_u[size_t(j)] * had_u[size_t(j)] + had_v[size_t(j)] * wbc_k * had_v[size_t(j)] * wbc_k));
			const double eu = SURF_WIND * (u[k] - ubar[size_t(j)]) + fwx;
			const double ev = SURF_WIND * (v[k] - vbar[size_t(j)]) + fwy;
			d_eddy[k] = float(std::sqrt(eu * eu + ev * ev));
			sx = wsx[k];
			sy = wsy[k];
			// sensible heat, bulk with free convection
			const double dts = t_s - t_air;
			const double swx = u[k] * SURF_WIND + sx;
			const double swy = v[k] * SURF_WIND + sy;
			const double spd = std::sqrt(swx * swx + swy * swy);
			const double u_eff = spd + 2.0 * std::sqrt(std::max(dts, 0.0));
			const double f_s = rho * CP_AIR * CH_BULK * u_eff * dts;
			const double h_pbl = clampd(300.0 * dts, 0.0, 9000.0);
			const double wstar = std::pow(std::max(GRAV / t_air * f_s / (rho * CP_AIR) * h_pbl, 0.0), 1.0 / 3.0);
			wconv[k] = float(wstar);
			const double g_c = 0.7 * cap[k] * OMEGA_SOL;
			const double w = (1.0 - albedo) * sw_ground + EMIS * r_down - EMIS * SIGMA * ts4 - f_s - g_c * (t_s - tdeep[k]);
			const double t_fr = 3182.48 / (27.9546 - std::log(std::max(p_s, 1.0)));
			const bool frozen = co2[k] > 0.0f || t_s <= t_fr;
			if (frozen && (co2[k] > 0.0f || w < 0.0)) {
				wfr[k] = float(wfr[k] + (w - wfr[k]) * dt / SOL_S);
				double dm_fast = -w * dt / L_CO2;
				double dm_slow = -(season_k - 1.0) * wfr[k] * dt / L_CO2;
				if (dm_fast + dm_slow < -co2[k]) {
					const double scl = co2[k] / std::max(-(dm_fast + dm_slow), 1e-12);
					dm_fast *= scl;
					dm_slow *= scl;
				}
				// mors's order: a cap that sublimates whole lands on 0
				co2[k] = float(co2[k] + (dm_fast + dm_slow));
				h[k] = float(h[k] - dm_fast / RHO_H);
				sum_dm += dm_slow * area;
				ts[k] = float(t_fr);
			} else {
				wfr[k] = float(wfr[k] + (0.0 - wfr[k]) * dt / SOL_S);
				ts[k] = float(t_s + dt * w / (cap[k] + dt * (4.0 * EMIS * SIGMA * t_s * t_s * t_s + g_c)));
			}
			tdeep[k] = float(tdeep[k] + dt * season_k * g_c * (t_s - tdeep[k]) / (cap[k] * DEEP_RATIO));
			tmean[k] = float(tmean[k] + (ts[k] - tmean[k]) * dt / SOL_S);
			const double h_r = EMIS * SIGMA * ts4 * beta_e - 2.0 * r_down;
			ta[k] = float(std::max(t_a + dt * (h_r + sw_col + f_s) / (CP_AIR * p_s / GRAV), t_fr));
			psurf[k] = float(p_s);
			tair[k] = float(t_air);
			const double tt = ta[k] + TIDE_GROUND_W * (ts[k] - ta[k]) +
					DUST_HEAT_K * std::min(dtot, DUST_HEAT_MAX) * flux * (0.25 + 0.75 * mu0);
			ttf[k] = float(tt);
			ttm[k] = float(ttm[k] + (tt - ttm[k]) * dt / TT_MEAN_TAU);
			sum_h += h[k] * area;
			sum_tt += clampd(tt - ttm[k], -TT_ANOM_MAX, TT_ANOM_MAX) * taper * area;
			sum_tm += ttm[k] * area;
			sum_ms += clampd(ttm[k] - ttmb, -MEAN_STATE_MAX, MEAN_STATE_MAX) * taper * area;
			sum_ts += ts[k];
			// Water (docs/water.md). Levels: pressure from the
			// column scale height, temperature the Viking lapse
			// about the column's mass-weighted mean T_a
			{
				Column cl;
				column_levels(p_s, t_a, R_GAS, GRAV, cl);
				const double sh = cl.sh;
				const double *air = cl.air, *lt = cl.t, *lp = cl.p;
				double qsat[NL];
				for (int l = 0; l < NL; ++l) {
					qsat[l] = M_RATIO * e_ice(lt[l]) / lp[l];
				}
				// interface eddy conductance rho K / dz, shared
				// by vapour and ice
				double cond[NL - 1];
				const double k_top = std::exp(-K_ZZ_TOP / sh);
				for (int l = 0; l + 1 < NL; ++l) {
					const double dzc = 0.5 * (LEVEL_M[l + 2] - LEVEL_M[l]);
					const double rho_i = p_s * cl.edge[l + 1] / (R_GAS * 0.5 * (lt[l] + lt[l + 1]));
					const double kzz = K_ZZ * std::max(1.0, k_top / cl.edge[l + 1]);
					cond[l] = rho_i * kzz / dzc;
				}
				// surface flux, M04 eq. 1: rho C_h |U| (q_s(T_s)
				// - q), C_h cut by a stable surface layer (LMD
				// vdif_cd); sublimes ice, deposits on cold ground
				double &v0 = a[T_VAP];
				const double dq = M_RATIO * e_ice(t_s) / p_s - v0 / air[0];
				// bare ground drier-than-saturated air: no flux,
				// and C_h's solve is skipped
				if (dq < 0.0 || frost[k] > 0.0f) {
					double ch = 0.0, u_ch = 0.0;
					surface_exchange(t_air, t_s, spd, wstar, GRAV, ch, u_ch);
					double ex_w = rho * ch * u_ch * dq * dt;
					ex_w = ex_w > 0.0 ? std::min(ex_w, double(frost[k])) : std::max(ex_w, -v0);
					frost[k] = float(frost[k] - ex_w);
					v0 += ex_w;
				}
				// vapour and ice: convective adjustment mixes the
				// boundary layer (M04 para 32), eddy diffusion the
				// levels (Montmessin 2002 para 33)
				for (const int base : { T_VAP, T_ICE }) {
					int top = 0;
					while (top + 1 < NL && LEVEL_M[top + 1] < h_pbl) {
						++top;
					}
					double ms = 0.0, as = 0.0;
					for (int l = 0; l <= top; ++l) {
						ms += a[base + l];
						as += air[l];
					}
					for (int l = 0; l <= top; ++l) {
						a[base + l] = ms * air[l] / as;
					}
					for (int l = 0; l + 1 < NL; ++l) {
						const double q_lo = a[base + l] / air[l], q_hi = a[base + l + 1] / air[l + 1];
						double f = cond[l] * (q_lo - q_hi) * dt;
						// never past equal mixing ratios
						const double lim = (q_lo - q_hi) * air[l] * air[l + 1] / (air[l] + air[l + 1]);
						f = lim > 0.0 ? std::min(f, lim) : std::max(f, lim);
						a[base + l] -= f;
						a[base + l + 1] += f;
					}
				}
				// M04 sec. 2.2.2: crystals on N nuclei of radius
				// r0, grown by eq. 3 integrated implicitly (para
				// 28), then sedimented at f r_c (para 30)
				// exp(-sigma^2) with f = exp(3 sigma^2), and r0 per
				// level: constants
				static const double fall_k = std::exp(-std::log(F_SED) / 3.0);
				static const auto r0_of = [] {
					std::array<double, NL> r{};
					for (int i = 0; i < NL; ++i) {
						r[size_t(i)] = R0_SURF * std::exp(-0.5 * (LEVEL_M[i] + LEVEL_M[i + 1]) / R0_H);
					}
					return r;
				}();
				double low = 0.0, high = 0.0;
				for (int l = 0; l < NL; ++l) {
					double &vl = a[T_VAP + l];
					double &il = a[T_ICE + l];
					// no ice, subsaturated: growth is exactly a no-op
					if (il <= 0.0 && vl <= qsat[l] * air[l]) {
						tau[size_t(l)][k] = 0.0f;
						continue;
					}
					const double rho_l = lp[l] / (R_GAS * lt[l]);
					// exp(-z / H) at the level's middle is p / p_s
					const double n_kg = tau_v / CCN_TAU_REF * CCN0 * (lp[l] / p_s) / rho_l;
					const double r0 = r0_of[size_t(l)];
					auto radius = [&](double p_mc) {
						return std::cbrt(p_mc / (4.0 / 3.0 * PI * RHO_ICE * n_kg) + r0 * r0 * r0);
					};
					const double mv = vl / air[l], mc = il / air[l];
					const double r_c = radius(mc);
					const double s_eq = std::exp(2.0 * SIG_H2O * MH2O / (RHO_ICE * RGP * lt[l] * r_c));
					const double res = growth_resistance(lt[l], lp[l], e_ice(lt[l]), r_c);
					const double kk = 4.0 * PI * n_kg * RHO_ICE * r_c / res;
					double mv1 = (mv + dt * kk * s_eq) / (1.0 + dt * kk / qsat[l]);
					mv1 = clampd(mv1, 0.0, mv + mc);
					vl = mv1 * air[l];
					il = (mv + mc - mv1) * air[l];
					const double r_n = radius(il / air[l]);
					const double dz = LEVEL_M[l + 1] - LEVEL_M[l];
					const double w = fall_speed(F_SED * r_n, lt[l], lp[l], GRAV) * fall_k;
					const double out = il * std::min(w * dt / dz, 1.0);
					il -= out;
					if (l > 0) {
						a[T_ICE + l - 1] += out;
					} else {
						frost[k] = float(frost[k] + out);
					}
					const double t_l = 3.0 * Q_EXT * il / (4.0 * RHO_ICE * r_n);
					tau[size_t(l)][k] = float(t_l);
					(l < NL_LOW ? low : high) += t_l;
				}
				tau_low[k] = float(low);
				tau_high[k] = float(high);
				// LMD CLFvarying: a level's fraction from its
				// condensation temperature against T, +-3 K, at
				// least 0.1; the column's is the ice-weighted mean
				double cf_sum = 0.0, cf_norm = 0.0;
				for (int l = 0; l < NL; ++l) {
					const double il = a[T_ICE + l];
					if (il <= 0.0) {
						frac[size_t(l)][k] = 0.0f;
						continue;
					}
					const double tc = t_cond_water(lp[l], (a[T_VAP + l] + il) / air[l]);
					const double cf = std::clamp((tc - lt[l] + 3.0) / 6.0, 0.1, 1.0);
					frac[size_t(l)][k] = float(cf);
					cf_sum += cf * il;
					cf_norm += il;
				}
				cloud_frac[k] = float(cf_norm > 0.0 ? cf_sum / cf_norm : 0.0);
			}
			// stress lifting over the Weibull gusts (White 1979)
			double lift = 0.0;
			const double ust = std::sqrt(spd * spd + wstar * wstar) / LOG_LAW;
			if (rho * ust * ust * wp99 * wp99 > TAU_STAR) {
				double lift_kg = 0.0, tail_kg = 0.0;
				for (int q = 0; q < 8; ++q) {
					const double vq = ust * wq[q];
					const double fq = white_flux(rho * vq * vq, rho, GRAV);
					lift_kg += fq;
					if (q >= 6) {
						tail_kg += fq;
					}
				}
				lift_kg *= 0.125;
				d_tail[k] = float(tail_kg * 0.125 / std::max(lift_kg, 1e-30));
				lift = lift_kg * LIFT_ALPHA * dt * KG_TO_DUST * (1.0 - sstep(STRESS_RAIL_LO, STRESS_RAIL_HI, dtot));
			}
			// dust devils (Renno 1998)
			const double eta = 0.11 * h_pbl / SCALE_H;
			double devil = DEVIL_ALPHA * eta * std::max(f_s, 0.0) * dt * KG_TO_DUST * (1.0 - sstep(STAB_LO, STAB_HI, dtot));
			if (p_spinup || co2[k] > 0.0f) {
				lift = 0.0;
				devil = 0.0;
			}
			lift *= cap_sc;
			devil *= cap_sc;
			d_stress[k] = float(rho * ust * ust);
			d_lift_s[k] = float(lift);
			d_lift_d[k] = float(devil);
			const double take = std::min(lift + devil, double(soil[k]));
			const double fl = take * (lift * (1.0 - COARSE_FRAC) + devil) / std::max(lift + devil, 1e-12);
			soil[k] = float(soil[k] - take);
			ad += fl;
			ac += take - fl;
			// settling: haze over weeks, storm grains in sols
			const double sed_f = ad * SED_FINE * dt;
			const double sed_c = ac * SED_COARSE * dt;
			ad -= sed_f;
			ac -= sed_c;
			soil[k] = float(soil[k] + sed_f + sed_c);
			wlf[k] = float(wlf[k] + (fl / dt - wlf[k]) * dt / SOL_S);
			wdu[k] = float(wdu[k] + (ad - wdu[k]) * dt / SOL_S);
			wlc[k] = float(wlc[k] + ((take - fl) / dt - wlc[k]) * dt / SOL_S);
			wdc[k] = float(wdc[k] + (ac - wdc[k]) * dt / SOL_S);
			if (dust_k > 1.0) {
				const double slow_out = std::min((dust_k - 1.0) * SED_FINE * wdu[k] * dt, ad);
				ad -= slow_out;
				soil[k] = float(soil[k] + slow_out);
				sum_wlf += wlf[k];
				sum_wlc += wlc[k];
			}
			for (int t = 0; t < NT; ++t) {
				nxt_[size_t(t)][k] = float(std::max(a[t], 0.0));
			}
		}
		row_h_[size_t(j)] = sum_h;
		row_tt_[size_t(j)] = sum_tt;
		row_tm_[size_t(j)] = sum_tm;
		row_ms_[size_t(j)] = sum_ms;
		row_dm_[size_t(j)] = sum_dm;
		row_wlf_[size_t(j)] = sum_wlf / GW;
		row_wlc_[size_t(j)] = sum_wlc / GW;
		row_ts_[size_t(j)] = sum_ts / GW;
		for (int t = 0; t < NT; ++t) {
			row_fix_[size_t(j * 2 * NT + t)] = pre[t] * area;
			row_fix_[size_t(j * 2 * NT + NT + t)] = post[t] * area;
		}
	});
	// mass fixer: scale the advected part back to the old mass
	double fix[NT];
	for (int t = 0; t < NT; ++t) {
		double pre = 0.0, post = 0.0;
		for (int j = 0; j < GH; ++j) {
			pre += row_fix_[size_t(j * 2 * NT + t)];
			post += row_fix_[size_t(j * 2 * NT + NT + t)];
		}
		fix[t] = (post > 1e-12 ? clampd(pre / post, 0.9, 1.1) : 1.0) - 1.0;
	}
	// unresolved eddy mixing of dust [M]
	const double kdiff = DUST_DIFF_K * dt / (GCELL_M * GCELL_M);
	rows([&](int j) {
		const int jm = std::max(j - 1, 0) * GW, jp = std::min(j + 1, GH - 1) * GW;
		const int j0 = j * GW;
		for (int i = 0; i < GW; ++i) {
			const size_t k = size_t(j0 + i);
			const size_t im = size_t(j0 + (i + GW - 1) % GW);
			const size_t ip = size_t(j0 + (i + 1) % GW);
			for (int t = 0; t < NT; ++t) {
				const Field &n = nxt_[size_t(t)];
				// unresolved eddy mixing for the dust only
				double lap = 0.0;
				if (t == T_DU || t == T_DC) {
					lap = double(n[im]) + n[ip] + n[size_t(jm + i)] + n[size_t(jp + i)] - 4.0 * n[k];
				}
				(*tr[size_t(t)])[k] = float(std::max(n[k] + adv_[size_t(t)][k] * fix[t] + kdiff * lap, 0.0));
			}
			float col = 0.0f;
			for (int l = 0; l < NL; ++l) {
				col += vap[size_t(l)][k];
			}
			wv[k] = col;
		}
	});
	double th = 0, ttt = 0, ttm_s = 0, tms = 0, ta_sum = 0, tdm = 0;
	for (int j = 0; j < GH; ++j) {
		th += row_h_[size_t(j)];
		ttt += row_tt_[size_t(j)];
		ttm_s += row_tm_[size_t(j)];
		tms += row_ms_[size_t(j)];
		tdm += row_dm_[size_t(j)];
		ta_sum += carea_[size_t(j)] * GW;
	}
	// season-clock cap mass comes off the whole layer
	const double hshift = -tdm / ta_sum / RHO_H;
	if (hshift != 0.0) {
		for (float &x : h) {
			x = float(x + hshift);
		}
	}
	hbar = th / ta_sum + hshift;
	if (dust_k > 1.0) {
		double mean_f = 0.0, mean_c = 0.0;
		for (int j = 0; j < GH; ++j) {
			mean_f += row_wlf_[size_t(j)] * carea_[size_t(j)] * GW;
			mean_c += row_wlc_[size_t(j)] * carea_[size_t(j)] * GW;
		}
		mean_f /= ta_sum;
		mean_c /= ta_sum;
		const double add_f = (dust_k - 1.0) * mean_f * dt * cap_sc;
		const double add_c = (dust_k - 1.0) * mean_c * dt * cap_sc;
		if (add_f + add_c > 0.0) {
			for (int k = 0; k < N; ++k) {
				const double room = std::max(STRESS_RAIL_HI - dust[size_t(k)] - dustc[size_t(k)], 0.0);
				const double give = std::min(std::min(add_f + add_c, double(soil[size_t(k)])), room);
				const double ff = give * add_f / (add_f + add_c);
				dust[size_t(k)] = float(dust[size_t(k)] + ff);
				dustc[size_t(k)] = float(dustc[size_t(k)] + give - ff);
				soil[size_t(k)] = float(soil[size_t(k)] - give);
			}
		}
		shift_profile(dust, had_up, dust_k, dt);
		shift_profile(dustc, had_up, dust_k, dt);
	}
	ttbar = ttt / ta_sum;
	ttmbar = ttm_s / ta_sum;
	msbar = tms / ta_sum;
	ttbar_set = true;
}

// The fine window follows the player in whole cells; cells that
// enter it start from the coarse field.
void Gcm::local_follow(double p_px, double p_py) {
	const double want[2] = { std::round((p_px - LN * LCELL_PX * 0.5) / LCELL_PX) * LCELL_PX,
		std::round((p_py - LN * LCELL_PX * 0.5) / LCELL_PX) * LCELL_PX };
	Field dsum(N);
	for (int k = 0; k < N; ++k) {
		dsum[size_t(k)] = dust[size_t(k)] + dustc[size_t(k)];
	}
	const int dx = l_valid_ ? int(std::lround((want[0] - l_org_[0]) / LCELL_PX)) : LN;
	const int dy = l_valid_ ? int(std::lround((want[1] - l_org_[1]) / LCELL_PX)) : LN;
	if (dx != 0 || dy != 0) {
		for (int f = 0; f < 3; ++f) {
			Field &src = f == 2 ? l_dust : l_tau[size_t(f)];
			const Field &gref = f == 2 ? dsum : (f == 0 ? tau_low : tau_high);
			Field nq(size_t(LN) * LN);
			for (int j = 0; j < LN; ++j) {
				for (int i = 0; i < LN; ++i) {
					const int si = i + dx, sj = j + dy;
					if (si >= 0 && si < LN && sj >= 0 && sj < LN) {
						nq[size_t(j * LN + i)] = src[size_t(sj * LN + si)];
					} else {
						nq[size_t(j * LN + i)] = float(sample(gref, want[0] + (i + 0.5) * LCELL_PX, want[1] + (j + 0.5) * LCELL_PX));
					}
				}
			}
			src.swap(nq);
		}
	}
	l_org_[0] = want[0];
	l_org_[1] = want[1];
	l_valid_ = true;
}

void Gcm::local_tick(double p_dt) {
	Field dsum(N);
	for (int k = 0; k < N; ++k) {
		dsum[size_t(k)] = dust[size_t(k)] + dustc[size_t(k)];
	}
	for (int f = 0; f < 3; ++f) {
		Field &src = f == 2 ? l_dust : l_tau[size_t(f)];
		const Field &gref = f == 2 ? dsum : (f == 0 ? tau_low : tau_high);
		Field nq(size_t(LN) * LN);
		for (int j = 0; j < LN; ++j) {
			const double py = l_org_[1] + (j + 0.5) * LCELL_PX;
			const double wlat = (90.0 - py / H_PX * 180.0) * DEG;
			const double wcos = std::max(std::cos(wlat), 0.05);
			for (int i = 0; i < LN; ++i) {
				const double px = l_org_[0] + (i + 0.5) * LCELL_PX;
				const double wu = sample(u, px, py) / (200.0 * wcos);
				const double wv = sample(v, px, py) / 200.0;
				const double fx = (px - wu * p_dt - l_org_[0]) / LCELL_PX - 0.5;
				const double fy = (py - wv * p_dt - l_org_[1]) / LCELL_PX - 0.5;
				double val;
				if (fx < 0.0 || fy < 0.0 || fx >= LN - 1.001 || fy >= LN - 1.001) {
					val = sample(gref, px - wu * p_dt, py - wv * p_dt);
				} else {
					const int x0 = int(fx), y0 = int(fy);
					const double tx = fx - x0, ty = fy - y0;
					val = lerpd(lerpd(src[size_t(y0 * LN + x0)], src[size_t(y0 * LN + x0 + 1)], tx),
							lerpd(src[size_t((y0 + 1) * LN + x0)], src[size_t((y0 + 1) * LN + x0 + 1)], tx), ty);
				}
				// relax toward the coarse field
				val = lerpd(val, sample(gref, px, py), 0.04);
				nq[size_t(j * LN + i)] = float(val);
			}
		}
		src.swap(nq);
	}
}

namespace {

void pack_fields(const Gcm::Field &p_low, const Gcm::Field &p_high, const Gcm::Field &p_du, const Gcm::Field *p_du2,
		size_t p_n, std::vector<uint8_t> &r_out) {
	const double tau_s = 255.0 / Gcm::PACK_TAU_MAX;
	const double dust_s = 255.0 / PACK_DUST_MAX;
	r_out.resize(p_n * 4);
	for (size_t k = 0; k < p_n; ++k) {
		const double du = p_du[k] + (p_du2 ? (*p_du2)[k] : 0.0f);
		r_out[k * 4] = uint8_t(clampd(p_low[k], 0.0, Gcm::PACK_TAU_MAX) * tau_s);
		r_out[k * 4 + 1] = uint8_t(clampd(p_high[k], 0.0, Gcm::PACK_TAU_MAX) * tau_s);
		r_out[k * 4 + 2] = 0;
		r_out[k * 4 + 3] = uint8_t(clampd(du, 0.0, PACK_DUST_MAX) * dust_s);
	}
}

} // namespace

void Gcm::pack_local(std::vector<uint8_t> &r_out) const {
	pack_fields(l_tau[0], l_tau[1], l_dust, nullptr, size_t(LN) * LN, r_out);
}

void Gcm::pack_global(std::vector<uint8_t> &r_out) const {
	pack_fields(tau_low, tau_high, dust, &dustc, N, r_out);
}

// R CO2 ice / 400 kg/m2, G water ice / 1 kg/m2, B ground T and
// A air T as (T - 140) / 160
void Gcm::pack_surface(std::vector<uint8_t> &r_out) const {
	r_out.resize(size_t(N) * 4);
	for (size_t k = 0; k < size_t(N); ++k) {
		r_out[k * 4] = uint8_t(clampd(co2[k] / 400.0, 0.0, 1.0) * 255.0);
		r_out[k * 4 + 1] = uint8_t(clampd(frost[k] / 1.0, 0.0, 1.0) * 255.0);
		r_out[k * 4 + 2] = uint8_t(clampd((ts[k] - 140.0) / 160.0, 0.0, 1.0) * 255.0);
		r_out[k * 4 + 3] = uint8_t(clampd((tair[k] - 140.0) / 160.0, 0.0, 1.0) * 255.0);
	}
}

} // namespace regolith

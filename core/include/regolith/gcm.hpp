#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <vector>

namespace regolith {

// The body and its air, per planet. mars() is mors's model.
// Positions are "planet pixels": px_per_turn of them around
// the equator, half as many pole to pole.
struct GcmConfig {
	// body
	double radius = 106496.0 * 200.0 / 6.283185307179586; // m
	double gravity = 3.711; // m/s2
	double omega = 7.088e-5; // rad/s
	double sol_s = 88775.0; // s, mean solar day
	double sol_hours = 24.6597; // the clock's hours a sol (mors)
	double year_sols = 668.6;
	double obliquity_deg = 25.19;
	double eccentricity = 0.0934;
	double perihelion_ls = 251.0; // deg
	double solar_flux = 586.0; // W/m2 at mean distance
	// atmosphere
	double scale_height = 10800.0; // m
	double datum_pressure = 708.0; // Pa, with empty caps
	double equivalent_depth = 3000.0; // m, shallow water
	double r_gas = 191.8; // J/kg/K
	double cp_air = 744.0; // J/kg/K
	// soil: diurnal skin sqrt(sol / 2 pi) and the annual slab's
	// sqrt(year_sols), as mors rounded them for Mars
	double diurnal_skin = 118.9;
	double deep_ratio = 25.9;
	// grid
	int gw = 356;
	int gh = 178;
	int ln = 160; // fine window cells a side
	double px_per_turn = 106496.0;
	double lcell_px = 12.5; // fine window cell, planet px

	static GcmConfig mars() { return GcmConfig(); }
};

// mors's reduced Mars atmosphere (scripts/mars_atmosphere.gd,
// physics in its RESEARCH_ATMOSPHERE.md), ported step for step:
// forced shallow water on a lat-lon grid, advected vapour, three
// ice layers and two dust classes, a surface energy balance,
// the CO2 and water cycles, dust lifting and settling. The body
// comes from GcmConfig; the column physics is Mars's. Row j = 0
// is the north pole edge; column i = 0 is longitude -180.
// Winds: u east, v south (the grid is y-down).
class Gcm {
public:
	static constexpr double DT = 120.0; // s per step
	static constexpr double SURF_WIND = 0.55;
	static constexpr double TAU_FLOOR = 0.10;
	static constexpr double TAU_PER_DUST = 2.6;

	// Water in physical units on height levels above the ground
	// (docs/water.md): edges in metres.
	static constexpr int NL = 8;
	static constexpr double LEVEL_M[NL + 1] = { 0.0, 2000.0, 5000.0, 9000.0, 14000.0, 20000.0, 27000.0, 35000.0, 45000.0 };
	// the cloud packs split low (below 14 km) from high levels
	static constexpr int NL_LOW = 4;
	// advected tracers: fine dust, coarse dust, vapour and ice
	// per level
	static constexpr int T_DU = 0, T_DC = 1, T_VAP = 2, T_ICE = 2 + NL, NT = 2 + 2 * NL;

	explicit Gcm(const GcmConfig &p_config = GcmConfig::mars());
	const GcmConfig &config() const { return cfg_; }

	struct Inputs {
		double ls = 40.0; // degrees
		double mtc = 12.0; // hours at longitude 0
		double year_sols = -1.0; // the season clock; < 0: real
		// harness controls (mors's defaults)
		double dust_k = -1.0; // < 0: follow the season clock
		double edge_k = -1.0;
		double front_wind = -1.0;
		double had_dust_k = -1.0;
		double had_v_upper = -1.0;
		double had_surf = 1.0;
		double wbc_scale = 1.0;
		double weibull_k = -1.0;
		bool wave_from_column = false;
	};

	using Field = std::vector<float>;

	// p_topo: DEM height (m, over the areoid) at cell centres;
	// p_lum: mosaic luminance 0..1 per cell, or null.
	void create(const float *p_topo, const float *p_lum, double p_ls, int p_threads);
	void step(const Inputs &p_in, bool p_spinup);
	bool finite() const;
	void reset_dynamics();
	// F5 / scripted: dust raised around a planet pixel
	void inject_dust(double p_px, double p_py);

	// the fine window: a 160 x 160 grid of 2.5 km cells
	void local_follow(double p_px, double p_py);
	void local_tick(double p_dt);
	double local_org_x() const { return l_org_[0]; }
	double local_org_y() const { return l_org_[1]; }

	// bilinear sample of a grid field at a planet pixel
	double sample(const Field &p_f, double p_px, double p_py) const;
	void fold_px(double &r_px, double &r_py) const;
	int index(double p_px, double p_py) const;

	// RGBA8 packs the renderer decodes: R low cloud tau, G high
	// cloud tau (both over PACK_TAU_MAX), A dust (mors's scale)
	static constexpr double PACK_TAU_MAX = 3.0;
	void pack_local(std::vector<uint8_t> &r_out) const;
	void pack_global(std::vector<uint8_t> &r_out) const;
	void pack_surface(std::vector<uint8_t> &r_out) const;

	// state. vap[l] and ice[l] in kg/m2 per level, wv their
	// vapour column (a diagnostic); frost is surface water ice
	Field u, v, h, wv, dust, dustc, frost, soil, ts, ta, tdeep, tmean, ttm, ttf, co2, wfr, wlf, wdu, wlc, wdc,
			psurf, tair, wsx, wsy, wconv;
	std::array<Field, NL> vap, ice;
	// diagnostics: visible optical depth per level, and the
	// low and high sums the cloud packs carry
	std::array<Field, NL> tau;
	Field tau_low, tau_high;
	Field topo, pref, alb, cap, wbc;
	double hbar = 0.0, ttbar = 0.0, ttmbar = 200.0, msbar = 0.0, cap_scale = 1.0;
	bool ttbar_set = false;
	std::array<double, 2> bc_phase{ 0.0, 2.0 };
	std::array<double, 2> bc_period{ 4.0, 4.0 };
	std::array<int, 2> bc_m{ 3, 2 };
	double bc_timer = 0.0;
	std::vector<double> clim_max; // 24 bins of 15 deg Ls, or empty

	// diagnostics (attribution)
	Field d_slope, d_edge, d_had, d_eddy, d_stress, d_lift_s, d_lift_d, bl_x, bl_y, d_front, d_tail;

	// fine window state: low tau, high tau, dust
	std::array<Field, 2> l_tau;
	Field l_dust;

	// derived from the config (mors's names, now per body)
	const GcmConfig cfg_;
	const int GW, GH, N, LN;
	const double W_PX, H_PX, GCELL_PX, GCELL_M, LCELL_PX;
	const double SOL_S, SOL_HOURS, YEAR_SOLS, OMEGA, OMEGA_SOL, GRAV, OBLIQUITY, ECC, PERI_LS, S_MARS;
	const double H_MEAN, SCALE_H, P_DATUM_MAX, R_GAS, CP_AIR, RHO_H;
	const double R_DRAG, R_HEAT, C_DIURNAL, DEEP_RATIO, TT_MEAN_TAU;

private:
	void step_waves(const Inputs &p_in);
	void step_dynamics();
	void step_tracers(const Inputs &p_in, bool p_spinup);
	void shift_profile(Field &r_arr, const std::vector<double> &p_had_up, double p_dust_k, double p_dt);
	template <typename F>
	void rows(F &&p_fn);

	int threads_ = 1;
	std::vector<double> lat_r_, fcor_, inv_dx_, inv_dxt_, cor_c_, cor_s_, carea_, cos_lon_, sin_lon_;
	Field rel_; // relaxation tendency, per step
	Field nu_, nv_, nh_;
	// per tracer: the advected value and the value after sources
	std::array<Field, NT> adv_, nxt_;
	std::vector<double> row_h_, row_tt_, row_ts_, row_tm_, row_ms_, row_fix_, row_dm_, row_wlf_, row_wlc_;
	std::vector<double> bc_amp_;
	std::mt19937 rng_{ 1 };
	double l_org_[2] = { 0.0, 0.0 };
	bool l_valid_ = false;
};

} // namespace regolith

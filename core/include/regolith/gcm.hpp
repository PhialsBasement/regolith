#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <vector>

namespace regolith {

// mors's reduced Mars atmosphere (scripts/mars_atmosphere.gd,
// physics in its RESEARCH_ATMOSPHERE.md), ported step for step:
// forced shallow water on a 356 x 178 lat-lon grid, advected
// vapour, three ice layers and two dust classes, a surface
// energy balance, the CO2 and water cycles, dust lifting and
// settling. Row j = 0 is the north pole edge; column i = 0 is
// longitude -180. Winds: u east, v south (the grid is y-down).
class Gcm {
public:
	static constexpr int GW = 356;
	static constexpr int GH = 178;
	static constexpr int N = GW * GH;
	static constexpr int LN = 160; // fine window cells a side
	static constexpr double W_PX = 106496.0; // planet pixels (200 m)
	static constexpr double H_PX = 53248.0;
	static constexpr double GCELL_PX = W_PX / GW;
	static constexpr double LCELL_PX = 12.5;
	static constexpr double DT = 120.0; // s per step
	static constexpr double SOL_S = 88775.0;
	static constexpr double SOL_HOURS = 24.6597;
	static constexpr double YEAR_SOLS = 668.6;
	static constexpr double SURF_WIND = 0.55;
	static constexpr double TAU_FLOOR = 0.10;
	static constexpr double TAU_PER_DUST = 2.6;
	static constexpr double R_GAS = 191.8;

	struct Inputs {
		double ls = 40.0; // degrees
		double mtc = 12.0; // hours at longitude 0
		double year_sols = YEAR_SOLS; // the season clock
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
	static double sample(const Field &p_f, double p_px, double p_py);
	static void fold_px(double &r_px, double &r_py);
	static int index(double p_px, double p_py);

	// RGBA8 packs the renderer decodes (mors's scales)
	void pack_local(std::vector<uint8_t> &r_out) const;
	void pack_global(std::vector<uint8_t> &r_out) const;
	void pack_surface(std::vector<uint8_t> &r_out) const;
	const std::vector<uint8_t> &meso() const { return meso_; }

	// state
	Field u, v, h, qv, dust, dustc, frost, soil, ts, ta, tdeep, tmean, ttm, ttf, co2, wfr, wlf, wdu, wlc, wdc,
			psurf, tair, wsx, wsy, wconv;
	std::array<Field, 3> qc;
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

	// fine window state
	std::array<Field, 3> l_qc;
	Field l_dust;

private:
	void step_waves(const Inputs &p_in);
	void step_dynamics();
	void step_tracers(const Inputs &p_in, bool p_spinup);
	void shift_profile(Field &r_arr, const std::vector<double> &p_had_up, double p_dust_k, double p_dt);
	template <typename F>
	void rows(F &&p_fn);

	int threads_ = 1;
	std::vector<double> lat_r_, fcor_, inv_dx_, inv_dxt_, cor_c_, cor_s_, carea_, cos_lon_, sin_lon_;
	Field nu_, nv_, nh_, nqv_, ndu_, ndc_, nq0_, nq1_, nq2_, aqv_, adu_, adc_, aq0_, aq1_, aq2_;
	std::vector<double> row_h_, row_tt_, row_ts_, row_tm_, row_ms_, row_fix_, row_dm_, row_wlf_, row_wlc_;
	std::vector<double> bc_amp_;
	std::vector<uint8_t> meso_;
	std::mt19937 rng_{ 1 };
	double l_org_[2] = { 0.0, 0.0 };
	bool l_valid_ = false;
};

} // namespace regolith

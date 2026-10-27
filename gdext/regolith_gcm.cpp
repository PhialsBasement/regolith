#include "regolith_gcm.hpp"

#include <cstring>

#include <godot_cpp/core/class_db.hpp>

using namespace godot;
using regolith::Gcm;

void RegolithGcm::_bind_methods() {
	ClassDB::bind_method(D_METHOD("configure", "config"), &RegolithGcm::configure);
	ClassDB::bind_method(D_METHOD("create", "topo", "luminance", "ls", "threads"), &RegolithGcm::create);
	ClassDB::bind_method(D_METHOD("step", "inputs", "spinup"), &RegolithGcm::step);
	ClassDB::bind_method(D_METHOD("get_field", "name"), &RegolithGcm::get_field);
	ClassDB::bind_method(D_METHOD("set_field", "name", "value"), &RegolithGcm::set_field);
	ClassDB::bind_method(D_METHOD("get_scalars"), &RegolithGcm::get_scalars);
	ClassDB::bind_method(D_METHOD("set_scalars", "value"), &RegolithGcm::set_scalars);
	ClassDB::bind_method(D_METHOD("set_clim", "maxima"), &RegolithGcm::set_clim);
}

// "ice0".."ice7" and "tau0".."tau7" are the levels
Gcm::Field *RegolithGcm::field_of(Gcm &p_g, const String &p_name) {
	static const char *names[] = { "u", "v", "h", "wv", "dust", "dustc", "frost", "soil", "ts", "ta", "tdeep", "tmean",
		"ttm", "ttf", "co2", "wfr", "wlf", "wdu", "wlc", "wdc", "psurf", "tair", "wsx", "wsy", "wconv", "topo", "pref",
		"alb", "cap", "wbc", "tau_low", "tau_high", "bl_x", "bl_y", "d_slope", "d_edge", "d_had", "d_eddy",
		"d_stress", "d_front", "d_lift_s", "d_lift_d" };
	Gcm::Field *fields[] = { &p_g.u, &p_g.v, &p_g.h, &p_g.wv, &p_g.dust, &p_g.dustc, &p_g.frost, &p_g.soil, &p_g.ts,
		&p_g.ta, &p_g.tdeep, &p_g.tmean, &p_g.ttm, &p_g.ttf, &p_g.co2, &p_g.wfr, &p_g.wlf, &p_g.wdu, &p_g.wlc, &p_g.wdc,
		&p_g.psurf, &p_g.tair, &p_g.wsx, &p_g.wsy, &p_g.wconv, &p_g.topo, &p_g.pref, &p_g.alb, &p_g.cap, &p_g.wbc,
		&p_g.tau_low, &p_g.tau_high, &p_g.bl_x, &p_g.bl_y, &p_g.d_slope, &p_g.d_edge, &p_g.d_had, &p_g.d_eddy,
		&p_g.d_stress, &p_g.d_front, &p_g.d_lift_s, &p_g.d_lift_d };
	for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); ++i) {
		if (p_name == names[i]) {
			return fields[i];
		}
	}
	for (int l = 0; l < Gcm::NL; ++l) {
		if (p_name == String("ice") + String::num_int64(l)) {
			return &p_g.ice[size_t(l)];
		}
		if (p_name == String("tau") + String::num_int64(l)) {
			return &p_g.tau[size_t(l)];
		}
		if (p_name == String("vap") + String::num_int64(l)) {
			return &p_g.vap[size_t(l)];
		}
	}
	return nullptr;
}

regolith::GcmConfig RegolithGcm::config_from(const Dictionary &p_d) {
	regolith::GcmConfig c = regolith::GcmConfig::mars();
	c.radius = double(p_d.get("radius", c.radius));
	c.gravity = double(p_d.get("gravity", c.gravity));
	c.omega = double(p_d.get("omega", c.omega));
	c.sol_s = double(p_d.get("sol_s", c.sol_s));
	c.sol_hours = double(p_d.get("sol_hours", c.sol_s == 88775.0 ? c.sol_hours : c.sol_s / 3600.0));
	c.year_sols = double(p_d.get("year_sols", c.year_sols));
	c.obliquity_deg = double(p_d.get("obliquity_deg", c.obliquity_deg));
	c.eccentricity = double(p_d.get("eccentricity", c.eccentricity));
	c.perihelion_ls = double(p_d.get("perihelion_ls", c.perihelion_ls));
	c.solar_flux = double(p_d.get("solar_flux", c.solar_flux));
	c.scale_height = double(p_d.get("scale_height", c.scale_height));
	c.datum_pressure = double(p_d.get("datum_pressure", c.datum_pressure));
	c.equivalent_depth = double(p_d.get("equivalent_depth", c.equivalent_depth));
	c.r_gas = double(p_d.get("r_gas", c.r_gas));
	c.cp_air = double(p_d.get("cp_air", c.cp_air));
	c.diurnal_skin = double(p_d.get("diurnal_skin", c.diurnal_skin));
	c.deep_ratio = double(p_d.get("deep_ratio", c.deep_ratio));
	c.gw = int(p_d.get("gw", c.gw));
	c.gh = int(p_d.get("gh", c.gh));
	c.ln = int(p_d.get("ln", c.ln));
	c.px_per_turn = double(p_d.get("px_per_turn", c.px_per_turn));
	c.lcell_px = double(p_d.get("lcell_px", c.lcell_px));
	return c;
}

void RegolithGcm::configure(const Dictionary &p_config) {
	gcm_ = std::make_unique<Gcm>(config_from(p_config));
}

void RegolithGcm::create(const PackedFloat32Array &p_topo, const PackedFloat32Array &p_lum, double p_ls, int p_threads) {
	gcm_->create(p_topo.size() == gcm_->N ? p_topo.ptr() : nullptr, p_lum.size() == gcm_->N ? p_lum.ptr() : nullptr, p_ls,
			p_threads);
}

Gcm::Inputs RegolithGcm::inputs_from(const Dictionary &p_d) {
	Gcm::Inputs in;
	in.ls = double(p_d.get("ls", 40.0));
	in.mtc = double(p_d.get("mtc", 12.0));
	in.year_sols = double(p_d.get("year_sols", -1.0));
	in.dust_k = double(p_d.get("dust_k", -1.0));
	in.edge_k = double(p_d.get("edge_k", -1.0));
	in.front_wind = double(p_d.get("front_wind", -1.0));
	in.had_dust_k = double(p_d.get("had_dust_k", -1.0));
	in.had_v_upper = double(p_d.get("had_v_upper", -1.0));
	in.had_surf = double(p_d.get("had_surf", 1.0));
	in.wbc_scale = double(p_d.get("wbc_scale", 1.0));
	in.weibull_k = double(p_d.get("weibull_k", -1.0));
	in.wave_from_column = String(p_d.get("wave_src", "ts")) == "ta";
	return in;
}

void RegolithGcm::step(const Dictionary &p_inputs, bool p_spinup) {
	gcm_->step(inputs_from(p_inputs), p_spinup);
}

PackedFloat32Array RegolithGcm::get_field(const String &p_name) const {
	PackedFloat32Array out;
	const Gcm::Field *f = field_of(*gcm_, p_name);
	if (f) {
		out.resize(int64_t(f->size()));
		std::memcpy(out.ptrw(), f->data(), f->size() * sizeof(float));
	}
	return out;
}

void RegolithGcm::set_field(const String &p_name, const PackedFloat32Array &p_value) {
	Gcm::Field *f = field_of(*gcm_, p_name);
	if (f && size_t(p_value.size()) == f->size()) {
		std::memcpy(f->data(), p_value.ptr(), f->size() * sizeof(float));
	}
}

Dictionary RegolithGcm::get_scalars() const {
	Dictionary d;
	d["hbar"] = gcm_->hbar;
	d["ttbar"] = gcm_->ttbar;
	d["ttmbar"] = gcm_->ttmbar;
	d["msbar"] = gcm_->msbar;
	d["ttbar_set"] = gcm_->ttbar_set;
	d["cap_scale"] = gcm_->cap_scale;
	d["bc_timer"] = gcm_->bc_timer;
	return d;
}

void RegolithGcm::set_scalars(const Dictionary &p_d) {
	gcm_->hbar = double(p_d.get("hbar", gcm_->hbar));
	gcm_->ttbar = double(p_d.get("ttbar", gcm_->ttbar));
	gcm_->ttmbar = double(p_d.get("ttmbar", gcm_->ttmbar));
	gcm_->msbar = double(p_d.get("msbar", gcm_->msbar));
	gcm_->ttbar_set = bool(p_d.get("ttbar_set", gcm_->ttbar_set));
	gcm_->bc_timer = double(p_d.get("bc_timer", gcm_->bc_timer));
	const Array ph = p_d.get("bc_phase", Array());
	const Array pe = p_d.get("bc_period", Array());
	const Array mm = p_d.get("bc_m", Array());
	for (int k = 0; k < 2; ++k) {
		if (ph.size() == 2) {
			gcm_->bc_phase[size_t(k)] = double(ph[k]);
		}
		if (pe.size() == 2) {
			gcm_->bc_period[size_t(k)] = double(pe[k]);
		}
		if (mm.size() == 2) {
			gcm_->bc_m[size_t(k)] = int(mm[k]);
		}
	}
}

void RegolithGcm::set_clim(const PackedFloat32Array &p_value) {
	gcm_->clim_max.assign(p_value.ptr(), p_value.ptr() + p_value.size());
}

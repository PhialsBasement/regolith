// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith/orbit.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <numbers>

namespace regolith {

namespace {

constexpr double TWO_PI = 2.0 * std::numbers::pi;

struct Elements {
	double a, e, inc, node, peri, mean;
};

Elements to_elements(const OrbitState &p_s, double p_gm) {
	const DVec3 r = p_s.position, v = p_s.velocity;
	const DVec3 h = cross(r, v);
	const DVec3 nvec(-h.y, h.x, 0.0);
	const double rr = length(r);
	const DVec3 evec = (cross(v, h) / p_gm) - r / rr;
	Elements el;
	el.e = length(evec);
	el.a = 1.0 / (2.0 / rr - dot(v, v) / p_gm);
	el.inc = std::acos(h.z / length(h));
	el.node = std::atan2(nvec.y, nvec.x);
	const double n_len = length(nvec);
	el.peri = std::atan2(dot(cross(nvec, evec), h) / length(h), dot(nvec, evec));
	if (n_len < 1e-12) {
		el.node = 0.0;
		el.peri = std::atan2(evec.y, evec.x);
	}
	const double nu = std::atan2(dot(cross(evec, r), h) / length(h), dot(evec, r));
	const double ecc = 2.0 * std::atan(std::sqrt((1.0 - el.e) / (1.0 + el.e)) * std::tan(nu / 2.0));
	el.mean = ecc - el.e * std::sin(ecc);
	return el;
}

OrbitState from_elements(const Elements &p_el, double p_gm) {
	double ecc = p_el.mean;
	for (int k = 0; k < 12; ++k) {
		ecc -= (ecc - p_el.e * std::sin(ecc) - p_el.mean) / (1.0 - p_el.e * std::cos(ecc));
	}
	const double b = p_el.a * std::sqrt(1.0 - p_el.e * p_el.e);
	const double n = std::sqrt(p_gm / (p_el.a * p_el.a * p_el.a));
	const double edot = n / (1.0 - p_el.e * std::cos(ecc));
	const DVec3 pos(p_el.a * (std::cos(ecc) - p_el.e), b * std::sin(ecc), 0.0);
	const DVec3 vel(-p_el.a * std::sin(ecc) * edot, b * std::cos(ecc) * edot, 0.0);
	auto rotate = [&](const DVec3 &p_v) {
		const double cw = std::cos(p_el.peri), sw = std::sin(p_el.peri);
		const DVec3 a(cw * p_v.x - sw * p_v.y, sw * p_v.x + cw * p_v.y, 0.0);
		const double ci = std::cos(p_el.inc), si = std::sin(p_el.inc);
		const DVec3 b2(a.x, ci * a.y, si * a.y);
		const double co = std::cos(p_el.node), so = std::sin(p_el.node);
		return DVec3(co * b2.x - so * b2.y, so * b2.x + co * b2.y, b2.z);
	};
	return { rotate(pos), rotate(vel) };
}

template <typename T>
void put(std::vector<uint8_t> &r_out, T p_v) {
	const size_t at = r_out.size();
	r_out.resize(at + sizeof(T));
	std::memcpy(r_out.data() + at, &p_v, sizeof(T));
}

} // namespace

bool OrbitTable::open(const std::string &p_path) {
	states_.clear();
	FILE *f = std::fopen(p_path.c_str(), "rb");
	if (!f) {
		return false;
	}
	const bool ok = read(f);
	std::fclose(f);
	if (!ok) {
		states_.clear();
	}
	return ok;
}

bool OrbitTable::read(FILE *p_file) {
	constexpr size_t HEAD = 80, SAMPLE = 6 * sizeof(double);
	uint8_t head[HEAD];
	if (std::fread(head, 1, HEAD, p_file) != HEAD || std::memcmp(head, "RGE1", 4) != 0) {
		return false;
	}
	uint32_t version = 0, count = 0;
	std::memcpy(&version, head + 4, 4);
	std::memcpy(&count, head + 8, 4);
	std::memcpy(&t0_, head + 16, 8);
	std::memcpy(&step_, head + 24, 8);
	std::memcpy(&parent_gm_, head + 32, 8);
	std::memcpy(&parent_j2_, head + 40, 8);
	std::memcpy(&parent_radius_, head + 48, 8);
	std::memcpy(&pole_ra_, head + 56, 8);
	std::memcpy(&pole_dec_, head + 64, 8);
	// state() divides by step and indexes by the quotient; Kepler
	// outside the table needs a parent with mass
	if (version != 1 || count < 2 || !std::isfinite(t0_) || !std::isfinite(step_) || step_ <= 0.0 ||
			!std::isfinite(parent_gm_) || parent_gm_ <= 0.0 || !std::isfinite(parent_j2_) ||
			!std::isfinite(parent_radius_) || !std::isfinite(pole_ra_) || !std::isfinite(pole_dec_)) {
		return false;
	}
	// the samples must be in the file before anything is sized by count
#ifdef _WIN32
	const bool sized = _fseeki64(p_file, 0, SEEK_END) == 0;
	const int64_t end = sized ? _ftelli64(p_file) : -1;
	const bool back = _fseeki64(p_file, int64_t(HEAD), SEEK_SET) == 0;
#else
	const bool sized = fseeko(p_file, 0, SEEK_END) == 0;
	const int64_t end = sized ? int64_t(ftello(p_file)) : -1;
	const bool back = fseeko(p_file, off_t(HEAD), SEEK_SET) == 0;
#endif
	if (!back || end < int64_t(HEAD) || uint64_t(count) * SAMPLE > uint64_t(end) - HEAD) {
		return false;
	}
	std::vector<double> raw(size_t(count) * 6);
	if (std::fread(raw.data(), sizeof(double), raw.size(), p_file) != raw.size()) {
		return false;
	}
	for (double v : raw) {
		if (!std::isfinite(v)) {
			return false;
		}
	}
	states_.resize(count);
	for (size_t k = 0; k < count; ++k) {
		const double *s = raw.data() + k * 6;
		states_[k] = { DVec3(s[0], s[1], s[2]), DVec3(s[3], s[4], s[5]) };
	}
	return true;
}

bool OrbitTable::save(const std::string &p_path) const {
	std::vector<uint8_t> out;
	out.insert(out.end(), { 'R', 'G', 'E', '1' });
	put<uint32_t>(out, 1);
	put<uint32_t>(out, uint32_t(states_.size()));
	put<uint32_t>(out, 0);
	put<double>(out, t0_);
	put<double>(out, step_);
	put<double>(out, parent_gm_);
	put<double>(out, parent_j2_);
	put<double>(out, parent_radius_);
	put<double>(out, pole_ra_);
	put<double>(out, pole_dec_);
	out.resize(80, 0);
	for (const OrbitState &s : states_) {
		for (double v : { s.position.x, s.position.y, s.position.z, s.velocity.x, s.velocity.y, s.velocity.z }) {
			put<double>(out, v);
		}
	}
	FILE *f = std::fopen(p_path.c_str(), "wb");
	if (!f) {
		return false;
	}
	const bool ok = std::fwrite(out.data(), 1, out.size(), f) == out.size();
	return std::fclose(f) == 0 && ok;
}

// ICRF to the parent's equatorial frame (z on the pole, x
// on the ascending node of its equator on the ICRF one).
DVec3 OrbitTable::to_equator(const DVec3 &p_v) const {
	const double ra = pole_ra_ * std::numbers::pi / 180.0;
	const double dec = pole_dec_ * std::numbers::pi / 180.0;
	const double a = -(ra + std::numbers::pi / 2.0);
	const DVec3 z1(std::cos(a) * p_v.x - std::sin(a) * p_v.y, std::sin(a) * p_v.x + std::cos(a) * p_v.y, p_v.z);
	const double b = -(std::numbers::pi / 2.0 - dec);
	return { z1.x, std::cos(b) * z1.y - std::sin(b) * z1.z, std::sin(b) * z1.y + std::cos(b) * z1.z };
}

DVec3 OrbitTable::from_equator(const DVec3 &p_v) const {
	const double ra = pole_ra_ * std::numbers::pi / 180.0;
	const double dec = pole_dec_ * std::numbers::pi / 180.0;
	const double b = std::numbers::pi / 2.0 - dec;
	const DVec3 x1(p_v.x, std::cos(b) * p_v.y - std::sin(b) * p_v.z, std::sin(b) * p_v.y + std::cos(b) * p_v.z);
	const double a = ra + std::numbers::pi / 2.0;
	return { std::cos(a) * x1.x - std::sin(a) * x1.y, std::sin(a) * x1.x + std::cos(a) * x1.y, x1.z };
}

OrbitState OrbitTable::propagate(const OrbitState &p_from_icrf, double p_dt) const {
	const OrbitState p_from{ to_equator(p_from_icrf.position), to_equator(p_from_icrf.velocity) };
	Elements el = to_elements(p_from, parent_gm_);
	const double n = std::sqrt(parent_gm_ / (el.a * el.a * el.a));
	const double p = el.a * (1.0 - el.e * el.e);
	const double k = 1.5 * n * parent_j2_ * (parent_radius_ / p) * (parent_radius_ / p);
	el.node += -k * std::cos(el.inc) * p_dt;
	el.peri += 0.5 * k * (5.0 * std::cos(el.inc) * std::cos(el.inc) - 1.0) * p_dt;
	el.mean = std::fmod(el.mean + n * p_dt, TWO_PI);
	const OrbitState eq = from_elements(el, parent_gm_);
	return { from_equator(eq.position), from_equator(eq.velocity) };
}

OrbitState OrbitTable::state(double p_seconds) const {
	// open() guarantees two samples and a finite step above 0; a
	// table never opened has nothing to give
	if (states_.size() < 2 || !(step_ > 0.0)) {
		return {};
	}
	const double u = (p_seconds - t0_) / step_;
	if (u <= 0.0) {
		return propagate(states_.front(), p_seconds - t0_);
	}
	if (u >= double(states_.size() - 1)) {
		return propagate(states_.back(), p_seconds - end());
	}
	const size_t i = size_t(u);
	const double t = u - double(i);
	const OrbitState &a = states_[i], &b = states_[i + 1];
	// Cubic Hermite in position, its derivative for velocity.
	const double t2 = t * t, t3 = t2 * t;
	const double h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + t;
	const double h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;
	const double d00 = (6 * t2 - 6 * t) / step_, d10 = 3 * t2 - 4 * t + 1;
	const double d01 = (-6 * t2 + 6 * t) / step_, d11 = 3 * t2 - 2 * t;
	OrbitState s;
	s.position = a.position * h00 + a.velocity * (h10 * step_) + b.position * h01 + b.velocity * (h11 * step_);
	s.velocity = a.position * d00 + a.velocity * d10 + b.position * d01 + b.velocity * d11;
	return s;
}

void synchronous_axes(const OrbitState &p_s, DVec3 r_axes[3]) {
	const DVec3 x = normalize(p_s.position * -1.0);
	const DVec3 z = normalize(cross(p_s.position, p_s.velocity));
	r_axes[0] = x;
	r_axes[1] = cross(z, x);
	r_axes[2] = z;
}

} // namespace regolith

// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "regolith/climate.hpp"
#include "regolith/ephemeris.hpp"
#include "test.hpp"

using namespace regolith;

namespace {

// 2025-01-01 00:00 UTC, seconds since J2000
const double NEW_YEAR_2025_S = j2000_seconds_from_unix(1735689600.0);
// the Mars model's fingerprint below, from the climate as it was before
// its Mars numbers became the model's defaults (Regolith 0.5.0)
constexpr uint64_t MARS_FINGERPRINT = 0x2dbdf53067bd1593ull;

} // namespace

TEST_CASE(climate_takes_its_season_from_the_orbit_it_is_given) {
	// the same climate on Earth's orbit (Mars's pole kept) must be in
	// that orbit's season, not Mars's
	ClimateConfig earth_path;
	earth_path.orbit = EARTH_ORBIT;
	Climate climate(earth_path);
	climate.create(nullptr, nullptr, {});
	climate.update(NEW_YEAR_2025_S);
	const double expected_ls = solar_longitude(EARTH_ORBIT, MARS_ROTATION, NEW_YEAR_2025_S);
	const double mars_ls = solar_longitude(MARS_ORBIT, MARS_ROTATION, NEW_YEAR_2025_S);
	std::printf("  Ls %.4f deg, the orbit's %.4f deg (Mars's would be %.4f deg)\n", climate.ls, expected_ls, mars_ls);
	CHECK(std::abs(climate.ls - expected_ls) < 1e-9);
	CHECK(std::abs(climate.ls - mars_ls) > 1.0);
}

TEST_CASE(climate_defaults_to_mars_season) {
	Climate climate;
	climate.create(nullptr, nullptr, {});
	climate.update(NEW_YEAR_2025_S);
	const double mars_ls = mars_solar_longitude(NEW_YEAR_2025_S);
	std::printf("  Ls %.4f deg, Mars's %.4f deg\n", climate.ls, mars_ls);
	CHECK(std::abs(climate.ls - mars_ls) < 1e-9);
}

namespace {

// FNV-1a over the bits of every output field: two runs agree only if
// every value is identical.
uint64_t fingerprint(const Climate &p_c) {
	uint64_t h = 1469598103934665603ull;
	auto add = [&h](const void *p_data, size_t p_size) {
		const unsigned char *b = static_cast<const unsigned char *>(p_data);
		for (size_t i = 0; i < p_size; ++i) {
			h = (h ^ b[i]) * 1099511628211ull;
		}
	};
	for (const Climate::Field *f : { &p_c.dust, &p_c.su, &p_c.sv, &p_c.lu, &p_c.lv, &p_c.ps, &p_c.ts, &p_c.tair, &p_c.co2,
				 &p_c.frost, &p_c.wv, &p_c.tau_low, &p_c.tau_high }) {
		add(f->data(), f->size() * sizeof(float));
	}
	for (int l = 0; l < Climate::NL; ++l) {
		add(p_c.tau[size_t(l)].data(), p_c.tau[size_t(l)].size() * sizeof(float));
		add(p_c.ice[size_t(l)].data(), p_c.ice[size_t(l)].size() * sizeof(float));
	}
	std::vector<uint8_t> packed;
	p_c.pack_global(packed);
	add(packed.data(), packed.size());
	p_c.pack_surface(packed);
	add(packed.data(), packed.size());
	add(&p_c.ls, sizeof(double));
	return h;
}

} // namespace

// The climate over synthetic terrain, albedo and dust climatology, a
// plume raised, at three times of the year: its fields' fingerprint.
static uint64_t run_fingerprint(const ClimateConfig &p_config) {
	Climate climate(p_config);
	const int gw = climate.GW, gh = climate.GH;
	std::vector<float> topo(size_t(gw) * gh), lum(size_t(gw) * gh);
	for (int j = 0; j < gh; ++j) {
		for (int i = 0; i < gw; ++i) {
			topo[size_t(j) * gw + i] = float(6000.0 * std::sin(i * 0.11) * std::cos(j * 0.07) - 2500.0);
			lum[size_t(j) * gw + i] = float(0.5 + 0.4 * std::sin(i * 0.05 + j * 0.13));
		}
	}
	std::vector<std::array<double, 7>> clim(24);
	for (int r = 0; r < 24; ++r) {
		for (int c = 0; c < 7; ++c) {
			clim[size_t(r)][size_t(c)] = 0.2 + 0.05 * c + 0.3 * std::sin(r * 0.26);
		}
	}
	climate.create(topo.data(), lum.data(), clim);
	climate.inject_dust(1000.0, 500.0, NEW_YEAR_2025_S);
	uint64_t h = 0;
	for (const double days : { 0.0, 200.0, 450.0 }) {
		climate.update(NEW_YEAR_2025_S + days * 86400.0 + 3600.0);
		h = h * 31 + fingerprint(climate);
	}
	return h;
}

TEST_CASE(mars_model_is_the_default_bit_for_bit) {
	const uint64_t h = run_fingerprint(ClimateConfig());
	std::printf("  fingerprint %016llx, expected %016llx\n", (unsigned long long)h, (unsigned long long)MARS_FINGERPRINT);
	CHECK(h == MARS_FINGERPRINT);
}

TEST_CASE(a_body_without_seasonal_caps_has_no_cap_ice) {
	// no cap bands, no perennial frost: no cap ice anywhere, all year
	ClimateConfig config;
	config.model.cap_north.clear();
	config.model.cap_south.clear();
	config.model.perennial_frost_lat_deg = 90.0;
	Climate climate(config);
	climate.create(nullptr, nullptr, {});
	double most_cap_ice = 0.0, most_frost = 0.0;
	for (const double days : { 0.0, 120.0, 240.0, 360.0, 480.0, 600.0 }) {
		climate.update(NEW_YEAR_2025_S + days * 86400.0);
		for (size_t k = 0; k < climate.co2.size(); ++k) {
			most_cap_ice = std::max(most_cap_ice, double(climate.co2[k]));
			most_frost = std::max(most_frost, double(climate.frost[k]));
		}
	}
	std::printf("  most cap ice %.3f kg/m2, most frost %.3f kg/m2 (expected 0)\n", most_cap_ice, most_frost);
	CHECK(most_cap_ice == 0.0);
	CHECK(most_frost == 0.0);
}

TEST_CASE(an_untilted_body_has_finite_fields) {
	// no axial tilt: no summer anywhere, and nothing divides by zero
	ClimateConfig config;
	config.obliquity_deg = 0.0;
	Climate climate(config);
	climate.create(nullptr, nullptr, {});
	climate.update(NEW_YEAR_2025_S);
	size_t not_finite = 0;
	for (const Climate::Field *f : { &climate.dust, &climate.su, &climate.sv, &climate.ps, &climate.ts, &climate.tair,
				 &climate.co2, &climate.tau_low, &climate.tau_high }) {
		for (const float v : *f) {
			not_finite += std::isfinite(v) ? 0 : 1;
		}
	}
	std::printf("  %zu values not finite (expected 0)\n", not_finite);
	CHECK(not_finite == 0);
}

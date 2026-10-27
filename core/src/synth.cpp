#include "regolith/synth.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "FastNoiseLite.h"
#include "regolith/cube.hpp"

namespace regolith {

namespace {

using fastnoiselite::FastNoiseLite;

constexpr double PI = std::numbers::pi;
constexpr double DEG = PI / 180.0;

// planet_streamer.gd constants, metres unless noted.
constexpr double DETAIL_AMP = 26.0;
constexpr double DETAIL_MIN = 0.35;
constexpr double DETAIL_MAX = 2.2;
constexpr double DUNE_AMP = 22.0;
constexpr double DUNE_MASK_EDGE = 0.62;
constexpr double ROUGH_AMP = 1.6;
constexpr double ROUGH_SLOPE = 6.0;
constexpr double TAR_AMP = 1.8;
constexpr double HUM_AMP = 0.6;
constexpr double TAR_WL = 55.0;
constexpr double STRATA_L = 70.0;
constexpr double STRATA_HARD = 0.35;
constexpr double STRATA_LO_DEG = 6.0;
constexpr double STRATA_HI_DEG = 16.0;
constexpr double REPOSE_SOFT_DEG = 31.0;
constexpr double REPOSE_HARD_DEG = 72.0;
constexpr int THERMAL_ITERS = 40;
constexpr double THERMAL_RATE = 0.5;
constexpr double FINE_PX = 12.5; // terra_fine's grid pitch
constexpr double HOLE_LAT = 3.20;
constexpr double HOLE_LON = 20.20;
constexpr double HOLE_R = 600.0;
constexpr double HOLE_DEPTH = 500.0;
constexpr double ICE_N_START = 76.0, ICE_N_FULL = 82.0;
constexpr double ICE_S_START = 78.0, ICE_S_FULL = 84.0;
constexpr int64_t SC_SEED = 0xC7A73B;

struct Band {
	double cell; // metres
	double d_min;
	double d_max;
	int64_t kb;
};
// SC_BANDS: 32 px and 4 px cells at 200 m/px.
constexpr Band BANDS[2] = { { 6400.0, 250.0, 1000.0, 0 }, { 800.0, 40.0, 250.0, 100 } };

double sstep(double p_a, double p_b, double p_x) {
	const double t = std::clamp((p_x - p_a) / (p_b - p_a), 0.0, 1.0);
	return t * t * (3.0 - 2.0 * t);
}

// terra_fine.gd's hash and gradient noise, on the CPU in double.
uint32_t uhash(uint32_t p_x, uint32_t p_y) {
	p_x = p_x * 1664525u + 1013904223u;
	p_y = p_y * 1664525u + 1013904223u;
	p_x += p_y * 1664525u;
	p_y += p_x * 1664525u;
	p_x ^= p_x >> 16u;
	p_y ^= p_y >> 16u;
	p_x += p_y * 1664525u;
	p_y += p_x * 1664525u;
	p_x ^= p_x >> 16u;
	p_y ^= p_y >> 16u;
	return p_x ^ p_y;
}

// The hashed angle has 16 bits, so its cos and sin (and the 3D
// noise's sqrt(1 - z^2)) come from tables: same values, no trig.
struct GradTables {
	std::vector<double> c, s, r;
	GradTables() :
			c(65536), s(65536), r(65536) {
		for (int k = 0; k < 65536; ++k) {
			const double a = double(k) * (2.0 * PI / 65535.0);
			c[size_t(k)] = std::cos(a);
			s[size_t(k)] = std::sin(a);
			const double z = double(k) / 32767.5 - 1.0;
			r[size_t(k)] = std::sqrt(std::max(0.0, 1.0 - z * z));
		}
	}
};

const GradTables &grad_tables() {
	static const GradTables t;
	return t;
}

double quintic(double p_f) {
	return p_f * p_f * p_f * (p_f * (p_f * 6.0 - 15.0) + 10.0);
}

double gnoise2(double p_x, double p_y, uint32_t p_seed) {
	const double ix = std::floor(p_x), iy = std::floor(p_y);
	const double fx = p_x - ix, fy = p_y - iy;
	const int32_t cx = int32_t(int64_t(ix)), cy = int32_t(int64_t(iy));
	const GradTables &tab = grad_tables();
	auto grad = [&](int32_t p_a, int32_t p_b, double p_dx, double p_dy) {
		const uint32_t h = uhash(uint32_t(p_a) ^ p_seed, uint32_t(p_b) ^ (p_seed * 7u)) & 0xFFFFu;
		return tab.c[h] * p_dx + tab.s[h] * p_dy;
	};
	const double ux = quintic(fx), uy = quintic(fy);
	const double a = grad(cx, cy, fx, fy);
	const double b = grad(cx + 1, cy, fx - 1.0, fy);
	const double c = grad(cx, cy + 1, fx, fy - 1.0);
	const double d = grad(cx + 1, cy + 1, fx - 1.0, fy - 1.0);
	return ((a + (b - a) * ux) + ((c + (d - c) * ux) - (a + (b - a) * ux)) * uy) * 1.4;
}

// The 3D extension of gnoise2: the sphere has no global plane,
// so fields mors drew in chart pixels are drawn in space.
double gnoise3(const DVec3 &p_p, uint32_t p_seed) {
	const double ix = std::floor(p_p.x), iy = std::floor(p_p.y), iz = std::floor(p_p.z);
	const double f[3] = { p_p.x - ix, p_p.y - iy, p_p.z - iz };
	const int32_t c[3] = { int32_t(int64_t(ix)), int32_t(int64_t(iy)), int32_t(int64_t(iz)) };
	const GradTables &tab = grad_tables();
	double corner[8];
	for (int k = 0; k < 8; ++k) {
		const int32_t o[3] = { k & 1, (k >> 1) & 1, (k >> 2) & 1 };
		const uint32_t h = uhash(uhash(uint32_t(c[0] + o[0]) ^ p_seed, uint32_t(c[1] + o[1])), uint32_t(c[2] + o[2]) ^ (p_seed * 7u));
		const uint32_t ht = h & 0xFFFFu, hz = (h >> 16) & 0xFFFFu;
		const double z = double(hz) / 32767.5 - 1.0;
		const double s = tab.r[hz];
		corner[k] = tab.c[ht] * s * (f[0] - o[0]) + tab.s[ht] * s * (f[1] - o[1]) + z * (f[2] - o[2]);
	}
	const double u = quintic(f[0]), v = quintic(f[1]), w = quintic(f[2]);
	auto lerp = [](double p_a, double p_b, double p_t) { return p_a + (p_b - p_a) * p_t; };
	const double x00 = lerp(corner[0], corner[1], u), x10 = lerp(corner[2], corner[3], u);
	const double x01 = lerp(corner[4], corner[5], u), x11 = lerp(corner[6], corner[7], u);
	return lerp(lerp(x00, x10, v), lerp(x01, x11, v), w) * 1.6;
}

double fbm3(const DVec3 &p_p, uint32_t p_seed) {
	return gnoise3(p_p, p_seed) * 0.57 + gnoise3(p_p * 2.03 + DVec3(11.0, 11.0, 11.0), p_seed + 1u) * 0.29 +
			gnoise3(p_p * 4.11 + DVec3(23.0, 23.0, 23.0), p_seed + 2u) * 0.14;
}

// planet_streamer.gd _sch: 64-bit wrap-around arithmetic.
int64_t sch(int64_t p_x, int64_t p_y, int64_t p_k) {
	uint64_t h = uint64_t(p_x) * 0x9E3779B1ull + uint64_t(p_y) * 0x85EBCA77ull + uint64_t(p_k) * 0xC2B2AE3Dull + uint64_t(SC_SEED);
	auto shr = [](uint64_t p_v, int p_s) { return uint64_t(int64_t(p_v) >> p_s); };
	h = (h ^ shr(h, 16)) * 0x7FEB352Dull;
	h = (h ^ shr(h, 15)) * 0x846CA68Bull;
	return int64_t((h ^ shr(h, 16)) & 0x7FFFFFFFull);
}

DVec3 dir_of_latlon(double p_lat, double p_lon) {
	return { std::cos(p_lat * DEG) * std::cos(p_lon * DEG), std::cos(p_lat * DEG) * std::sin(p_lon * DEG), std::sin(p_lat * DEG) };
}

double arc(const DVec3 &p_a, const DVec3 &p_b) {
	return std::atan2(length(cross(p_a, p_b)), dot(p_a, p_b));
}

void tangent_axes(const DVec3 &p_d, DVec3 &r_east, DVec3 &r_north) {
	DVec3 e = cross(DVec3(0, 0, 1), p_d);
	r_east = length(e) > 1e-9 ? normalize(e) : DVec3(0, 1, 0);
	r_north = cross(p_d, r_east);
}

// Slope (mean |gradient|) of a tile sample, one-sided at the
// outer ring.
double grade_at(const HeightTile &p_t, const TileLayout &p_lay, const std::vector<float> &p_src, int p_i, int p_j, double p_spacing) {
	const int lo = -p_lay.border, hi = p_lay.intervals + p_lay.border;
	auto at = [&](int p_a, int p_b) { return double(p_src[size_t(p_lay.index(std::clamp(p_a, lo, hi), std::clamp(p_b, lo, hi)))]); };
	const double sx = (p_i > lo && p_i < hi) ? 2.0 : 1.0;
	const double sy = (p_j > lo && p_j < hi) ? 2.0 : 1.0;
	const double gx = (at(p_i + 1, p_j) - at(p_i - 1, p_j)) / sx;
	const double gy = (at(p_i, p_j + 1) - at(p_i, p_j - 1)) / sy;
	return (std::fabs(gx) + std::fabs(gy)) * 0.5 / p_spacing;
}

FastNoiseLite make_noise(int p_seed, FastNoiseLite::FractalType p_fractal, int p_octaves) {
	FastNoiseLite n(p_seed);
	n.SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2);
	n.SetFractalType(p_fractal);
	n.SetFractalOctaves(p_octaves);
	// Coordinates arrive pre-scaled in double; see sample().
	n.SetFrequency(1.0f);
	return n;
}

} // namespace

// The planet_streamer.gd noises, frequencies converted from
// "per 200 m (or 50 m) source pixel" to per metre.
struct Synth::Noises {
	FastNoiseLite dn = make_noise(4242, FastNoiseLite::FractalType_FBm, 4);
	FastNoiseLite dune = make_noise(5151, FastNoiseLite::FractalType_Ridged, 2);
	FastNoiseLite dmask = make_noise(6262, FastNoiseLite::FractalType_None, 1);
	FastNoiseLite rough = make_noise(7373, FastNoiseLite::FractalType_FBm, 3);
	FastNoiseLite prov = make_noise(808, FastNoiseLite::FractalType_FBm, 2);
	FastNoiseLite patch = make_noise(909, FastNoiseLite::FractalType_FBm, 2);
	FastNoiseLite ice = make_noise(111, FastNoiseLite::FractalType_None, 1);

	static double sample(const FastNoiseLite &p_n, const DVec3 &p_metres, double p_freq) {
		const DVec3 p = p_metres * p_freq;
		return double(p_n.GetNoise(float(p.x), float(p.y), float(p.z)));
	}
};

float DarkField::sample(double p_lat, double p_lon) const {
	if (v.empty()) {
		return 0.0f;
	}
	const double fx = std::fmod((p_lon + 180.0) / 360.0 * w - 0.5 + w, double(w));
	const double fy = std::clamp((90.0 - p_lat) / 180.0 * h - 0.5, 0.0, double(h) - 1.001);
	const int x0 = int(fx), y0 = int(fy);
	const int x1 = (x0 + 1) % w, y1 = std::min(y0 + 1, h - 1);
	const double tx = fx - x0, ty = fy - y0;
	auto px = [&](int p_x, int p_y) { return double(v[size_t(p_y) * w + p_x]); };
	return float((px(x0, y0) * (1 - tx) + px(x1, y0) * tx) * (1 - ty) + (px(x0, y1) * (1 - tx) + px(x1, y1) * tx) * ty);
}

Synth::Synth(const SynthConfig &p_config) :
		config_(p_config), noise_(std::make_unique<Noises>()) {}

Synth::~Synth() = default;

double Synth::spacing(int p_level) const {
	return PI / 2.0 * config_.r_ref / double(int64_t(config_.intervals) << p_level);
}

DVec3 Synth::sample_dir(TileKey p_key, const TileLayout &p_lay, int p_i, int p_j) {
	const int64_t m = int64_t(p_lay.intervals) << p_key.level();
	const IVec3 p = fold_lattice(p_key.face(), int64_t(p_key.x()) * p_lay.intervals + p_i,
			int64_t(p_key.y()) * p_lay.intervals + p_j, m);
	return lattice_dir(p, m);
}

// planet_streamer.gd PASS 3: slope-conditioned FBM detail plus
// ridged dunes inside their mask, at the 200 m level.
void Synth::detail(HeightTile &r_tile, const TileLayout &p_lay) const {
	if (config_.profile != 1) {
		return;
	}
	const std::vector<float> src = r_tile.rho;
	const double sp = spacing(r_tile.key.level());
	for (int j = -p_lay.border; j <= p_lay.intervals + p_lay.border; ++j) {
		for (int i = -p_lay.border; i <= p_lay.intervals + p_lay.border; ++i) {
			const DVec3 pm = sample_dir(r_tile.key, p_lay, i, j) * config_.r_ref;
			const double grade = grade_at(r_tile, p_lay, src, i, j, sp);
			const double amp = std::clamp(DETAIL_MIN + grade * 7.0, DETAIL_MIN, DETAIL_MAX);
			const double n = Noises::sample(noise_->dn, pm, 0.08 / 200.0);
			const double r01 = (Noises::sample(noise_->dune, pm, 0.1 / 200.0) + 1.0) * 0.5;
			const double mask = (Noises::sample(noise_->dmask, pm, 0.004 / 200.0) + 1.0) * 0.5;
			const double m01 = std::clamp((mask - DUNE_MASK_EDGE) * 5.0, 0.0, 1.0);
			r_tile.rho[size_t(p_lay.index(i, j))] += float(n * DETAIL_AMP * amp + r01 * r01 * DUNE_AMP * m01);
		}
	}
	r_tile.update_bounds(p_lay);
}

// STAGE 1's roughness band: 400, 200 and 100 m octaves, more on
// slopes, at the 50 m level.
void Synth::roughness(HeightTile &r_tile, const TileLayout &p_lay) const {
	if (config_.profile != 1) {
		return;
	}
	const std::vector<float> src = r_tile.rho;
	const double sp = spacing(r_tile.key.level());
	for (int j = -p_lay.border; j <= p_lay.intervals + p_lay.border; ++j) {
		for (int i = -p_lay.border; i <= p_lay.intervals + p_lay.border; ++i) {
			const DVec3 pm = sample_dir(r_tile.key, p_lay, i, j) * config_.r_ref;
			const double grade = grade_at(r_tile, p_lay, src, i, j, sp);
			const double n = Noises::sample(noise_->rough, pm, 0.125 / 50.0);
			r_tile.rho[size_t(p_lay.index(i, j))] += float(n * ROUGH_AMP * (1.0 + grade * ROUGH_SLOPE));
		}
	}
	r_tile.update_bounds(p_lay);
}

// terra_fine.gd RELIEF: ripples, hummocks and strata, at 12.5 m.
void Synth::relief(HeightTile &r_tile, const TileLayout &p_lay) const {
	r_tile.repose.assign(r_tile.rho.size(), 1.0f);
	if (config_.profile != 1) {
		return;
	}
	const std::vector<float> src = r_tile.rho;
	const double sp = spacing(r_tile.key.level());
	const double lo_tan = std::tan(STRATA_LO_DEG * DEG), hi_tan = std::tan(STRATA_HI_DEG * DEG);
	const double soft_tan = std::tan(REPOSE_SOFT_DEG * DEG), hard_tan = std::tan(REPOSE_HARD_DEG * DEG);
	for (int j = -p_lay.border; j <= p_lay.intervals + p_lay.border; ++j) {
		for (int i = -p_lay.border; i <= p_lay.intervals + p_lay.border; ++i) {
			const size_t k = size_t(p_lay.index(i, j));
			const DVec3 d = sample_dir(r_tile.key, p_lay, i, j);
			// P in terra_fine's fine pixels, but in space
			const DVec3 p = d * (config_.r_ref / FINE_PX);
			const double h = src[k];
			const double grade = grade_at(r_tile, p_lay, src, i, j, sp);
			// the DEM's own slope is never relaxed
			const double keep = grade * 1.15 + 0.05;
			r_tile.repose[k] = float(std::max(1.0, keep));
			// ripples: wind drifts over ~20 km, patchy at ~5 km
			DVec3 east, north;
			tangent_axes(d, east, north);
			const double ang = fbm3(p / 1600.0, 90u) * PI;
			const DVec3 wd = east * std::cos(ang) + north * std::sin(ang);
			const DVec3 wq = north * std::cos(ang) - east * std::sin(ang);
			const double cover = sstep(0.30, 0.62, 0.5 + 0.5 * gnoise3(p / 400.0 + DVec3(17.0, 17.0, 17.0), 91u));
			const double wl = TAR_WL / FINE_PX;
			double r = 1.0 - std::fabs(gnoise2(dot(p, wd) / wl + 3.1, dot(p, wq) / (wl * 3.0) + 3.1, 92u));
			r = r * r;
			const double tar = (r - 0.42) * TAR_AMP * cover * std::clamp(1.0 - grade * 4.0, 0.0, 1.0);
			// hummocks: everywhere, rougher on slopes
			const double hum = fbm3(p / 6.4, 93u) * HUM_AMP * (1.0 + grade * ROUGH_SLOPE);
			double out = h + tar + hum;
			// strata: hard layers at fixed elevations on slopes
			const double ll = STRATA_L * (1.0 + 0.4 * (gnoise3(p / 800.0, 41u) * 0.67 + gnoise3(p / 400.0 + DVec3(5, 5, 5), 42u) * 0.33));
			const double t = h / ll;
			const double layer = std::floor(t);
			const double fr = t - layer;
			const DVec3 pl = p + DVec3(layer * 137.0, -layer * 291.0, layer * 183.0);
			const double sel = gnoise3(pl / 320.0, 43u) * 0.45 + gnoise3(pl / 160.0 + DVec3(7, 7, 7), 44u) * 0.25 +
					gnoise3(pl / 48.0 + DVec3(13, 13, 13), 45u) * 0.19 + gnoise3(pl / 24.0 + DVec3(19, 19, 19), 46u) * 0.11;
			const double thr = 0.55 - 1.1 * STRATA_HARD;
			const double hard = sstep(thr - 0.06, thr + 0.06, sel);
			const double gate = sstep(lo_tan, hi_tan, grade);
			// bench, then the cliff: flat for the lower 55 %
			const double b = 0.55;
			const double gg = fr < b ? 0.0 : std::pow((fr - b) / (1.0 - b), 1.3);
			out += (ll * (layer + gg) - h) * gate * hard;
			const double lip = sstep(0.55, 0.8, fr) * hard;
			const double soft = std::max(soft_tan, keep);
			const double base = std::max(1.0, keep);
			r_tile.repose[k] = float(base + ((soft + (hard_tan - soft) * lip) - base) * gate);
			r_tile.rho[k] = float(out);
		}
	}
	r_tile.update_bounds(p_lay);
}

// terra_fine.gd THERMAL: material slides off slopes steeper
// than the repose limit, 40 short steps.
void Synth::thermal(std::vector<float> &r_h, const std::vector<float> &p_rep, int p_n, double p_spacing) const {
	if (config_.profile != 1) {
		return;
	}
	std::vector<float> next(r_h.size());
	for (int it = 0; it < THERMAL_ITERS; ++it) {
		for (int y = 0; y < p_n; ++y) {
			for (int x = 0; x < p_n; ++x) {
				const size_t k = size_t(y) * p_n + x;
				const double h = r_h[k];
				const double rk = p_rep[k];
				double acc = 0.0;
				for (int dy = -1; dy <= 1; ++dy) {
					for (int dx = -1; dx <= 1; ++dx) {
						const int nx = std::clamp(x + dx, 0, p_n - 1);
						const int ny = std::clamp(y + dy, 0, p_n - 1);
						if (nx == x && ny == y) {
							continue;
						}
						const size_t kn = size_t(ny) * p_n + nx;
						const double hn = r_h[kn];
						const double dist = p_spacing * ((dx != 0 && dy != 0) ? 1.4142136 : 1.0);
						const double out_f = std::max((h - hn) - rk * dist, 0.0);
						const double in_f = std::max((hn - h) - double(p_rep[kn]) * dist, 0.0);
						acc += in_f - out_f;
					}
				}
				next[k] = float(h + acc * THERMAL_RATE * 0.125);
			}
		}
		r_h.swap(next);
	}
}

void Synth::borehole(HeightTile &r_tile, const TileLayout &p_lay) const {
	if (config_.profile != 1) {
		return;
	}
	const DVec3 c = dir_of_latlon(HOLE_LAT, HOLE_LON);
	// Skip tiles nowhere near it.
	const DVec3 mid = sample_dir(r_tile.key, p_lay, p_lay.intervals / 2, p_lay.intervals / 2);
	const double half = spacing(r_tile.key.level()) * p_lay.intervals;
	if (arc(mid, c) * config_.r_ref > half + 2.0 * HOLE_R) {
		return;
	}
	for (int j = -p_lay.border; j <= p_lay.intervals + p_lay.border; ++j) {
		for (int i = -p_lay.border; i <= p_lay.intervals + p_lay.border; ++i) {
			const double r = arc(sample_dir(r_tile.key, p_lay, i, j), c) * config_.r_ref / HOLE_R;
			if (r < 1.0) {
				r_tile.rho[size_t(p_lay.index(i, j))] -= float(HOLE_DEPTH * std::clamp(1.0 - std::pow(r, 6.0), 0.0, 1.0));
			}
		}
	}
	r_tile.update_bounds(p_lay);
}

// _stamp_small_fine: hashed sub-catalogue craters, power-law
// sizes, Garvin depth times a hashed degradation state. Cells
// are squares of face metres, keyed by face, so every crater
// belongs to one face and is found from any side of an edge.
void Synth::small_craters_near(const DVec3 &p_dir, double p_radius, int p_band, std::vector<SmallCrater> &r_out) const {
	const Band &band = BANDS[p_band];
	const double edge = PI / 2.0 * config_.r_ref;
	const int64_t cells = int64_t(std::ceil(edge / band.cell));
	// face metres stretch up to ~2x against true metres
	const double reach = (band.d_max * 0.5 * 1.5 + p_radius) * 2.0;
	const double span = 1.0 - (band.d_min / band.d_max) * (band.d_min / band.d_max);
	for (int face = 0; face < FACE_COUNT; ++face) {
		const FaceAxes &ax = face_axes(face);
		const double w = dot(cast<double>(ax.f), p_dir);
		if (w < 0.3) {
			continue;
		}
		const double x = (unwarp(dot(cast<double>(ax.u), p_dir) / w) + 1.0) * 0.5 * edge;
		const double y = (unwarp(dot(cast<double>(ax.v), p_dir) / w) + 1.0) * 0.5 * edge;
		const int64_t x0 = std::max<int64_t>(0, int64_t(std::floor((x - reach) / band.cell)));
		const int64_t x1 = std::min<int64_t>(cells - 1, int64_t(std::floor((x + reach) / band.cell)));
		const int64_t y0 = std::max<int64_t>(0, int64_t(std::floor((y - reach) / band.cell)));
		const int64_t y1 = std::min<int64_t>(cells - 1, int64_t(std::floor((y + reach) / band.cell)));
		for (int64_t cy = y0; cy <= y1; ++cy) {
			for (int64_t cx = x0; cx <= x1; ++cx) {
				const int64_t key_x = cx + int64_t(face) * (int64_t(1) << 24);
				const int64_t n = sch(key_x, cy, band.kb) % 3;
				for (int64_t k = 0; k < n; ++k) {
					const int64_t h1 = sch(key_x, cy, band.kb + 1 + k * 4);
					const int64_t h2 = sch(key_x, cy, band.kb + 2 + k * 4);
					const int64_t h3 = sch(key_x, cy, band.kb + 3 + k * 4);
					const int64_t hf = sch(key_x, cy, band.kb + 4 + k * 4);
					const double px = (double(cx) + double(h1 % 1024) / 1024.0) * band.cell;
					const double py = (double(cy) + double(h2 % 1024) / 1024.0) * band.cell;
					if (px >= edge || py >= edge) {
						continue;
					}
					const double u = double(h3 % 1024) / 1024.0;
					const double d_m = band.d_min / std::sqrt(1.0 - u * span);
					SmallCrater c;
					c.ry = d_m * 0.5;
					c.dir = face_dir({ face, px / edge * 2.0 - 1.0, py / edge * 2.0 - 1.0 });
					if (arc(c.dir, p_dir) * config_.r_ref > p_radius + c.ry * 1.5) {
						continue;
					}
					// most are shallow dishes, ~1 in 14 young and sharp
					const double f = (hf % 14 == 0) ? (0.8 + 0.2 * double(hf % 256) / 256.0)
													: (0.08 + 0.32 * double(hf % 256) / 256.0);
					c.depth = f * 210.0 * std::pow(d_m / 1000.0, 0.81);
					c.cos_reach = std::cos(c.ry * 1.5 / config_.r_ref);
					r_out.push_back(c);
				}
			}
		}
	}
}

double Synth::small_crater_sum(const DVec3 &p_dir, const std::vector<SmallCrater> &p_list) const {
	double sum = 0.0;
	for (const SmallCrater &c : p_list) {
		if (dot(c.dir, p_dir) < c.cos_reach) {
			continue;
		}
		const double rr = arc(c.dir, p_dir) * config_.r_ref / c.ry;
		if (rr >= 1.5) {
			continue;
		}
		double dh = rr < 1.0 ? -c.depth * (1.0 - rr * rr) : 0.0;
		const double re = (rr - 1.0) / 0.16;
		dh += 0.25 * c.depth * std::exp(-re * re);
		sum += dh * (1.0 - sstep(1.1, 1.5, rr));
	}
	return sum;
}

void Synth::craters(HeightTile &r_tile, const TileLayout &p_lay, int p_band) const {
	if (config_.profile != 1) {
		return;
	}
	// Every crater that can reach the tile, found once.
	const int n = p_lay.intervals;
	const DVec3 mid = sample_dir(r_tile.key, p_lay, n / 2, n / 2);
	double radius = 0.0;
	for (int c = 0; c < 4; ++c) {
		const int i = (c & 1) ? n + p_lay.border : -p_lay.border;
		const int j = (c & 2) ? n + p_lay.border : -p_lay.border;
		radius = std::max(radius, arc(mid, sample_dir(r_tile.key, p_lay, i, j)) * config_.r_ref);
	}
	std::vector<SmallCrater> list;
	small_craters_near(mid, radius, p_band, list);
	if (list.empty()) {
		return;
	}
	for (int j = -p_lay.border; j <= n + p_lay.border; ++j) {
		for (int i = -p_lay.border; i <= n + p_lay.border; ++i) {
			r_tile.rho[size_t(p_lay.index(i, j))] += float(small_crater_sum(sample_dir(r_tile.key, p_lay, i, j), list));
		}
	}
	r_tile.update_bounds(p_lay);
}

// _build_color_map: R gravel province, G bedrock patch, B ice,
// A Viking darkness; the borehole rim is glassed to bedrock.
void Synth::geology(TileKey p_key, const TileLayout &p_lay, std::vector<uint8_t> &r_rgba) const {
	const int side = geology_side(p_lay);
	r_rgba.assign(size_t(side) * side * 4, 0);
	if (config_.profile == 0) {
		return;
	}
	const DVec3 hole = dir_of_latlon(HOLE_LAT, HOLE_LON);
	auto byte = [](double p_n) { return uint8_t(std::clamp(p_n * 127.5 + 127.5, 0.0, 255.0)); };
	for (int qj = -1; qj < side - 1; ++qj) {
		for (int qi = -1; qi < side - 1; ++qi) {
			const DVec3 d = sample_dir(p_key, p_lay, qi * 4, qj * 4);
			const DVec3 pm = d * config_.r_ref;
			uint8_t *o = &r_rgba[(size_t(qj + 1) * side + size_t(qi + 1)) * 4];
			o[0] = byte(Noises::sample(noise_->prov, pm, 0.0042 / 200.0));
			o[1] = byte(Noises::sample(noise_->patch, pm, 0.019 / 200.0));
			if (config_.profile == 2) {
				o[3] = 255; // Phobos: dark regolith everywhere
				continue;
			}
			const double lat = std::asin(std::clamp(d.z, -1.0, 1.0)) / DEG;
			const double lon = std::atan2(d.y, d.x) / DEG;
			const double edge = Noises::sample(noise_->ice, pm, 0.01 / 200.0) * 2.5;
			const double ice = lat >= 0.0 ? sstep(ICE_N_START, ICE_N_FULL, lat + edge)
										  : sstep(ICE_S_START, ICE_S_FULL, -lat + edge);
			o[2] = uint8_t(ice * 255.0);
			o[3] = dark_ ? uint8_t(std::clamp(double(dark_->sample(lat, lon)), 0.0, 1.0) * 255.0) : 0;
			if (arc(d, hole) * config_.r_ref <= HOLE_R * 1.6) {
				o[1] = 255;
			}
		}
	}
}

} // namespace regolith

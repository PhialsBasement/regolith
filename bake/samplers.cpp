#include "samplers.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

namespace regolith::bake {

namespace {

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
constexpr double DEG = 180.0 / std::numbers::pi;
constexpr double DEM_RADIUS = 3396190.0;
constexpr double POLE_BLEND_M = 12000.0;

struct LatLon {
	double lat;
	double lon;
};

LatLon lat_lon(const DVec3 &p_d) {
	return { std::atan2(p_d.z, std::hypot(p_d.x, p_d.y)) * DEG, std::atan2(p_d.y, p_d.x) * DEG };
}

// Subsample (a, b) of the canonical sample, a and b in -1..1.
FaceCoord subsample_coord(const FaceSample &p_c, int64_t p_m, int p_a, int p_b) {
	const double m = double(p_m);
	const double s = (double(2 * p_c.gi - p_m) + (2.0 / 3.0) * p_a) / m;
	const double t = (double(2 * p_c.gj - p_m) + (2.0 / 3.0) * p_b) / m;
	return { p_c.face, s, t };
}

DVec3 subsample_dir(const FaceSample &p_c, int64_t p_m, int p_a, int p_b) {
	return face_dir(subsample_coord(p_c, p_m, p_a, p_b));
}

} // namespace

double MarsHeightSampler::rho(int p_level, const FaceSample &p_canonical, int64_t p_m, const IVec3 &p_point) const {
	FaceCoord coords[9];
	DVec3 dirs[9];
	LatLon lls[9];
	for (int k = 0; k < 9; ++k) {
		coords[k] = subsample_coord(p_canonical, p_m, k % 3 - 1, k / 3 - 1);
		dirs[k] = face_dir(coords[k]);
		lls[k] = lat_lon(dirs[k]);
	}
	const int first = std::max(0, DATA_LEVEL - p_level);
	double h = NaN;
	for (int mip = first; mip < dem_.mip_count() && std::isnan(h); ++mip) {
		double sum = 0.0;
		int n = 0;
		for (const LatLon &ll : lls) {
			const double v = dem_.sample(mip, ll.lat, ll.lon);
			if (!std::isnan(v)) {
				sum += v;
				++n;
			}
		}
		h = n ? sum / n : NaN;
		if (std::isnan(h)) {
			++fallbacks_;
		}
	}
	if (std::isnan(h)) {
		return NaN;
	}
	const LatLon ll = lat_lon(lattice_dir(p_point, p_m));
	const double colat_m = (90.0 - std::abs(ll.lat)) / DEG * DEM_RADIUS;
	const double u = colat_m / POLE_BLEND_M;
	if (u < 1.0) {
		const double w = 1.0 - u * u * (3.0 - 2.0 * u);
		const double cap = ll.lat > 0.0 ? dem_.north_cap() : dem_.south_cap();
		h = h * (1.0 - w) + cap * w;
	}
	if (catalog_) {
		// Features narrower than this level's texel are left
		// to the finer levels.
		const double texel = std::numbers::pi / 2.0 * Catalog::BODY_RADIUS / double(int64_t(INTERVALS) << p_level);
		double stamps = 0.0;
		for (int k = 0; k < 9; ++k) {
			stamps += catalog_->stamp(coords[k].face, coords[k].s, coords[k].t, dirs[k], texel);
		}
		h += stamps / 9.0;
	}
	return h + areoid_.radius(ll.lat, ll.lon) - R_REF;
}

double MarsAreoidSampler::rho(int p_level, const FaceSample &p_canonical, int64_t p_m, const IVec3 &p_point) const {
	const LatLon ll = lat_lon(lattice_dir(p_point, p_m));
	return areoid_.radius(ll.lat, ll.lon) - MarsHeightSampler::R_REF;
}

double PhobosHeightSampler::rho(int p_level, const FaceSample &p_canonical, int64_t p_m, const IVec3 &p_point) const {
	double sum = 0.0;
	int n = 0;
	for (int b = -1; b <= 1; ++b) {
		for (int a = -1; a <= 1; ++a) {
			int hits = 0;
			const double r = shape_.radius(subsample_dir(p_canonical, p_m, a, b), &hits);
			if (hits > 1) {
				++multi_hits_;
			}
			if (std::isnan(r)) {
				++misses_;
				continue;
			}
			sum += r;
			++n;
		}
	}
	return n ? sum / n - R_REF : NaN;
}

} // namespace regolith::bake

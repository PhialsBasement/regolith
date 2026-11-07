#pragma once

#include <atomic>
#include <cstdint>

#include "areoid.hpp"
#include "catalog.hpp"
#include "mars_dem.hpp"
#include "phobos_icq.hpp"
#include "raster_dem.hpp"
#include "tile_bake.hpp"

namespace regolith::bake {

// Mars height: 3x3 supersampled DEM at mip (7 - level), pole
// caps converged, catalogue stamped, areoid added.
class MarsHeightSampler : public LevelSampler {
public:
	static constexpr double R_REF = 3389500.0;
	static constexpr int DATA_LEVEL = 7;
	static constexpr int INTERVALS = 208;

	MarsHeightSampler(const MarsDem &p_dem, const Areoid &p_areoid, const Catalog *p_catalog) :
			dem_(p_dem), areoid_(p_areoid), catalog_(p_catalog) {}
	double rho(int p_level, const FaceSample &p_canonical, int64_t p_m, const IVec3 &p_point) const override;
	uint64_t mip_fallbacks() const { return fallbacks_; }

private:
	const MarsDem &dem_;
	const Areoid &areoid_;
	const Catalog *catalog_;
	mutable std::atomic<uint64_t> fallbacks_{ 0 };
};

class MarsAreoidSampler : public LevelSampler {
public:
	explicit MarsAreoidSampler(const Areoid &p_areoid) :
			areoid_(p_areoid) {}
	double rho(int p_level, const FaceSample &p_canonical, int64_t p_m, const IVec3 &p_point) const override;

private:
	const Areoid &areoid_;
};

// Any body from a global raster of heights over its datum: a
// sphere of datum_radius, or per point the radius in a datum
// raster. 3x3 supersampled at mip (data_level - level), pole
// caps converged, as Mars without its catalogue.
class DemHeightSampler : public LevelSampler {
public:
	DemHeightSampler(const RasterDem &p_dem, const RasterDem *p_datum, double p_datum_radius, double p_r_ref,
			int p_data_level) :
			dem_(p_dem), datum_(p_datum), datum_radius_(p_datum_radius), r_ref_(p_r_ref), data_level_(p_data_level) {}
	double rho(int p_level, const FaceSample &p_canonical, int64_t p_m, const IVec3 &p_point) const override;
	uint64_t mip_fallbacks() const { return fallbacks_; }
	double datum(double p_lat, double p_lon) const;

private:
	const RasterDem &dem_;
	const RasterDem *datum_;
	double datum_radius_;
	double r_ref_;
	int data_level_;
	mutable std::atomic<uint64_t> fallbacks_{ 0 };
};

class DemDatumSampler : public LevelSampler {
public:
	DemDatumSampler(const DemHeightSampler &p_height, double p_r_ref) :
			height_(p_height), r_ref_(p_r_ref) {}
	double rho(int p_level, const FaceSample &p_canonical, int64_t p_m, const IVec3 &p_point) const override;

private:
	const DemHeightSampler &height_;
	double r_ref_;
};

// Phobos: 3x3 supersampled ray casts against the ICQ mesh.
class PhobosHeightSampler : public LevelSampler {
public:
	static constexpr double R_REF = 11080.0;

	explicit PhobosHeightSampler(const IcqShape &p_shape) :
			shape_(p_shape) {}
	double rho(int p_level, const FaceSample &p_canonical, int64_t p_m, const IVec3 &p_point) const override;
	uint64_t multi_hits() const { return multi_hits_; }
	uint64_t misses() const { return misses_; }

private:
	const IcqShape &shape_;
	mutable std::atomic<uint64_t> multi_hits_{ 0 };
	mutable std::atomic<uint64_t> misses_{ 0 };
};

} // namespace regolith::bake

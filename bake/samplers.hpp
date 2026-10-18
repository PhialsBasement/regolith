#pragma once

#include <atomic>
#include <cstdint>

#include "areoid.hpp"
#include "catalog.hpp"
#include "mars_dem.hpp"
#include "phobos_icq.hpp"
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

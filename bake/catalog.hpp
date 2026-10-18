#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "regolith/vec.hpp"

namespace regolith::bake {

// Catalogued Mars features stamped into the DEM, with the
// profiles mors uses (see docs/rgp1.md, "Catalogue stamps").
class Catalog {
public:
	static constexpr double BODY_RADIUS = 3389500.0;
	static constexpr int GRID = 512;

	bool load_craters(const std::string &p_path); // crater_restore.bin
	bool load_pits(const std::string &p_path); // cave_sites.bin
	void build_index();
	// Summed profiles of features at least p_min_diameter wide
	// at p_dir, which is (p_face, p_s, p_t); s, t may overshoot.
	double stamp(int p_face, double p_s, double p_t, const DVec3 &p_dir, double p_min_diameter) const;
	size_t crater_count() const { return craters_.size(); }
	size_t pit_count() const { return pits_.size(); }

private:
	struct Feature {
		DVec3 dir;
		double radius = 0.0; // metres
		double amount = 0.0; // crater deepen or pit depth
		uint32_t flags = 0; // 1 complex, 2 peak, 4 pit
		double cos_reach = 0.0; // cheap reject before any trig
	};

	double profile(const Feature &p_f, double p_rr) const;

	std::vector<Feature> features_;
	std::vector<std::vector<uint32_t>> cells_;
	std::vector<Feature> craters_;
	std::vector<Feature> pits_;
};

} // namespace regolith::bake

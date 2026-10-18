#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "regolith/vec.hpp"

namespace regolith::bake {

// Gaskell ICQ shape model as a triangle mesh in metres, with
// triangles bucketed by our cube-face coordinates for rays.
class IcqShape {
public:
	static constexpr int GRID = 512;

	bool open(const std::string &p_path);
	// Distance along p_dir from the origin to the surface.
	// r_hits gets the number of distinct crossings (1 if
	// star-shaped). NaN when the ray misses.
	double radius(const DVec3 &p_dir, int *r_hits) const;
	size_t triangle_count() const { return tris_.size(); }

private:
	int q_ = 0;
	std::vector<DVec3> verts_;
	std::vector<std::array<uint32_t, 3>> tris_;
	std::vector<std::vector<uint32_t>> buckets_;
};

} // namespace regolith::bake

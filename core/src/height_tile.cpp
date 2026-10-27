#include "regolith/height_tile.hpp"

#include <algorithm>
#include <limits>

#include "regolith/codec.hpp"

namespace regolith {

namespace {

void interior_bounds(HeightTile &r_tile, const TileLayout &p_layout) {
	float lo = std::numeric_limits<float>::infinity();
	float hi = -lo;
	for (int j = 0; j <= p_layout.intervals; ++j) {
		for (int i = 0; i <= p_layout.intervals; ++i) {
			const float v = r_tile.at(p_layout, i, j);
			lo = std::min(lo, v);
			hi = std::max(hi, v);
		}
	}
	r_tile.min = lo;
	r_tile.max = hi;
}

// Midpoint of p1 and p2; written symmetrically so a face
// walking the edge backwards gets the same bits.
float midpoint(float p_a0, float p_a1, float p_a2, float p_a3) {
	return (9.0f * (p_a1 + p_a2) - (p_a0 + p_a3)) * 0.0625f;
}

} // namespace

void HeightTile::update_bounds(const TileLayout &p_layout) {
	interior_bounds(*this, p_layout);
}

bool PackTileSource::open(const std::string &p_path) {
	return reader_.open(p_path);
}

bool PackTileSource::bounds(TileKey p_key, float &r_min, float &r_max) const {
	const PackEntry *e = reader_.find(p_key);
	if (!e) {
		return false;
	}
	r_min = e->min;
	r_max = e->max;
	return true;
}

bool PackTileSource::load(TileKey p_key, HeightTile &r_tile) const {
	const PackEntry *e = reader_.find(p_key);
	if (!e) {
		return false;
	}
	std::vector<uint8_t> blob;
	if (!reader_.read_blob_concurrent(*e, blob)) {
		return false;
	}
	const TileLayout lay = layout();
	std::vector<int32_t> v(size_t(lay.count()));
	if (!decode_tile(blob.data(), blob.size(), lay.side(), Codec(header().codec), v.data())) {
		return false;
	}
	const double step = header().step;
	r_tile.key = p_key;
	r_tile.rho.resize(v.size());
	for (size_t k = 0; k < v.size(); ++k) {
		r_tile.rho[k] = float(double(v[k]) * step);
	}
	r_tile.min = e->min;
	r_tile.max = e->max;
	return true;
}

void subdivide(const HeightTile &p_parent, const TileLayout &p_layout, int p_quadrant, HeightTile &r_child) {
	const int n = p_layout.intervals;
	const int b = p_layout.border;
	const int qx = p_quadrant & 1;
	const int qy = p_quadrant >> 1;
	const int last = (1 << p_parent.key.level()) - 1;
	const bool face_lo_x = p_parent.key.x() == 0;
	const bool face_hi_x = int(p_parent.key.x()) == last;
	const bool face_lo_y = p_parent.key.y() == 0;
	const bool face_hi_y = int(p_parent.key.y()) == last;
	auto parent = [&](int p_i, int p_j) { return p_parent.at(p_layout, p_i, p_j); };
	// A stencil for a point inside the face never reads past the
	// face edge: the fold there differs per face near corners.
	auto mid4 = [&](float p_a0, float p_a1, float p_a2, float p_a3, int p_a, bool p_lo, bool p_hi) {
		if (p_a == 0 && p_lo) {
			p_a0 = 2.0f * p_a1 - p_a2;
		}
		if (p_a + 1 == n && p_hi) {
			p_a3 = 2.0f * p_a2 - p_a1;
		}
		return midpoint(p_a0, p_a1, p_a2, p_a3);
	};
	// Value at half-index x2 (parent units * 2) along row p_j.
	auto along_x = [&](int p_x2, int p_j) {
		if ((p_x2 & 1) == 0) {
			return parent(p_x2 >> 1, p_j);
		}
		const int a = (p_x2 - 1) >> 1;
		const bool inside = a >= 0 && a < n;
		return mid4(parent(a - 1, p_j), parent(a, p_j), parent(a + 1, p_j), parent(a + 2, p_j), a,
				inside && face_lo_x, inside && face_hi_x);
	};
	r_child.key = p_parent.key.child(p_quadrant);
	r_child.rho.resize(size_t(p_layout.count()));
	for (int j = -b; j <= n + b; ++j) {
		const int y2 = qy * n + j;
		for (int i = -b; i <= n + b; ++i) {
			const int x2 = qx * n + i;
			float v;
			if ((y2 & 1) == 0) {
				v = along_x(x2, y2 >> 1);
			} else {
				const int c = (y2 - 1) >> 1;
				const bool inside = c >= 0 && c < n;
				v = mid4(along_x(x2, c - 1), along_x(x2, c), along_x(x2, c + 1), along_x(x2, c + 2), c,
						inside && face_lo_y, inside && face_hi_y);
			}
			r_child.rho[size_t(p_layout.index(i, j))] = v;
		}
	}
	interior_bounds(r_child, p_layout);
}

} // namespace regolith

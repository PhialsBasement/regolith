#pragma once

namespace regolith {

// A tile's samples run from -border to intervals + border on
// both axes, stored row-major (j outer, i inner).
struct TileLayout {
	int intervals = 256;
	int border = 2;

	constexpr int side() const { return intervals + 1 + 2 * border; }
	constexpr int count() const { return side() * side(); }
	constexpr int index(int p_i, int p_j) const { return (p_j + border) * side() + (p_i + border); }
};

} // namespace regolith

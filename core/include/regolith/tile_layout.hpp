// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

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

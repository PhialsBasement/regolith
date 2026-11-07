// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <vector>

#include "regolith/vec.hpp"

namespace regolith {

struct OrbitState {
	DVec3 position; // metres, parent-centred, ICRF (J2000)
	DVec3 velocity; // metres per second
};

// A satellite's state at a fixed step (from JPL Horizons):
// Hermite between samples, Kepler with J2 drift outside them.
// Layout in docs/rge1.md.
class OrbitTable {
public:
	bool open(const std::string &p_path);
	bool save(const std::string &p_path) const;
	OrbitState state(double p_seconds) const;
	double start() const { return t0_; }
	double end() const { return t0_ + step_ * double(states_.size() - 1); }

	double t0_ = 0.0;
	double step_ = 0.0;
	double parent_gm_ = 0.0;
	double parent_j2_ = 0.0;
	double parent_radius_ = 0.0;
	double pole_ra_ = 0.0; // degrees, ICRF
	double pole_dec_ = 90.0;
	std::vector<OrbitState> states_;

private:
	OrbitState propagate(const OrbitState &p_from_icrf, double p_dt) const;
	DVec3 to_equator(const DVec3 &p_v) const;
	DVec3 from_equator(const DVec3 &p_v) const;
};

// Body-fixed axes (columns, ICRF) of a synchronously rotating
// moon: +X toward its parent, +Z along the orbit normal.
void synchronous_axes(const OrbitState &p_s, DVec3 r_axes[3]);

} // namespace regolith

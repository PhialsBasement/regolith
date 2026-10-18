#include "regolith/select.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "regolith/cube.hpp"

namespace regolith {

double Selector::node_edge(double p_r_ref, int p_level) {
	return std::numbers::pi / 2.0 * p_r_ref / double(uint64_t(1) << p_level);
}

double Selector::range(const SelectInput &p_in, int p_level) {
	return p_in.range_factor * node_edge(p_in.r_ref, p_level);
}

NodeSphere Selector::bounds(const SelectInput &p_in, TileKey p_key) const {
	// Unknown tiles borrow the nearest known ancestor's range,
	// padded for the overshoot of the 4-point subdivision.
	float lo = 0.0f, hi = 0.0f;
	TileKey k = p_key;
	bool known = rho_bounds && rho_bounds(k, lo, hi);
	while (!known && k.level() > 0) {
		k = k.parent();
		known = rho_bounds && rho_bounds(k, lo, hi);
	}
	double pad = 2.0;
	if (k.level() != p_key.level()) {
		pad += 0.15 * double(hi - lo);
	}
	const double r_lo = p_in.r_ref + double(lo) - pad;
	const double r_hi = p_in.r_ref + double(hi) + pad;

	const double tiles = double(uint64_t(1) << p_key.level());
	const double s0 = -1.0 + 2.0 * p_key.x() / tiles;
	const double t0 = -1.0 + 2.0 * p_key.y() / tiles;
	const double span = 2.0 / tiles;
	DVec3 pts[18];
	int n = 0;
	for (int b = 0; b <= 2; ++b) {
		for (int a = 0; a <= 2; ++a) {
			const DVec3 d = face_dir({ p_key.face(), s0 + span * 0.5 * a, t0 + span * 0.5 * b });
			pts[n++] = d * r_lo;
			pts[n++] = d * r_hi;
		}
	}
	DVec3 lo_v = pts[0], hi_v = pts[0];
	for (const DVec3 &p : pts) {
		lo_v = { std::min(lo_v.x, p.x), std::min(lo_v.y, p.y), std::min(lo_v.z, p.z) };
		hi_v = { std::max(hi_v.x, p.x), std::max(hi_v.y, p.y), std::max(hi_v.z, p.z) };
	}
	NodeSphere s;
	s.center = (lo_v + hi_v) * 0.5;
	for (const DVec3 &p : pts) {
		s.radius = std::max(s.radius, length(p - s.center));
	}
	// The surface bulges between the 3x3 sample directions.
	const double alpha = std::numbers::pi / 2.0 / tiles;
	s.radius += r_hi * alpha * alpha / 16.0;
	return s;
}

bool Selector::culled(const SelectInput &p_in, const NodeSphere &p_s) const {
	for (const std::array<double, 4> &pl : p_in.planes) {
		if (pl[0] * p_s.center.x + pl[1] * p_s.center.y + pl[2] * p_s.center.z - pl[3] > p_s.radius) {
			return true;
		}
	}
	const double big_r = p_in.occluder_radius;
	const double d = length(p_in.camera);
	if (big_r <= 0.0 || d <= big_r) {
		return false;
	}
	// Hidden when inside the occluder's shadow cone and past the
	// plane through its tangent points.
	const DVec3 u = p_in.camera * (-1.0 / d);
	const DVec3 q = p_s.center - p_in.camera;
	const double along = dot(q, u);
	const double perp = length(q - u * along);
	const double sin_t = big_r / d;
	const double cos_t = std::sqrt(1.0 - sin_t * sin_t);
	const double inside = along * sin_t - perp * cos_t;
	return inside > p_s.radius && along - p_s.radius > d - big_r * big_r / d;
}

void Selector::visit(const SelectInput &p_in, TileKey p_key, std::vector<SelectedNode> &r_nodes,
		std::vector<WantedTile> &r_wanted) const {
	const NodeSphere s = bounds(p_in, p_key);
	if (culled(p_in, s)) {
		return;
	}
	const double dist = std::max(0.0, length(p_in.camera - s.center) - s.radius);
	const int level = p_key.level();
	if (level < p_in.max_level && dist < range(p_in, level + 1)) {
		bool all_ready = true;
		for (int q = 0; q < 4; ++q) {
			const TileKey c = p_key.child(q);
			if (!ready(c)) {
				all_ready = false;
				r_wanted.push_back({ c, float(1.0 / (1.0 + dist)) });
			}
		}
		if (all_ready) {
			for (int q = 0; q < 4; ++q) {
				visit(p_in, p_key.child(q), r_nodes, r_wanted);
			}
			return;
		}
	}
	SelectedNode n;
	n.key = p_key;
	n.sphere = s;
	if (level == 0) {
		n.morph_start = n.morph_end = std::numeric_limits<double>::infinity();
	} else {
		n.morph_end = range(p_in, level);
		n.morph_start = p_in.morph_start * n.morph_end;
	}
	r_nodes.push_back(n);
}

void Selector::select(const SelectInput &p_in, std::vector<SelectedNode> &r_nodes, std::vector<WantedTile> &r_wanted) const {
	r_nodes.clear();
	r_wanted.clear();
	for (int face = 0; face < FACE_COUNT; ++face) {
		visit(p_in, TileKey::make(face, 0, 0, 0), r_nodes, r_wanted);
	}
}

} // namespace regolith

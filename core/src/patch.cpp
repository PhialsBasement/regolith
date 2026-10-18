#include "regolith/patch.hpp"

#include <cmath>
#include <numbers>

#include "regolith/cube.hpp"

namespace regolith {

namespace {

constexpr double QUARTER_PI = std::numbers::pi / 4.0;

int64_t face_intervals(const PatchSpec &p_spec) {
	return int64_t(p_spec.intervals) << p_spec.key.level();
}

} // namespace

RenderFrame RenderFrame::at(const DVec3 &p_anchor) {
	RenderFrame f;
	f.anchor = p_anchor;
	f.up = normalize(p_anchor);
	const DVec3 e = cross(DVec3(0.0, 0.0, 1.0), f.up);
	const double e_len = length(e);
	// At a pole any horizontal axis will do; pick +Y.
	f.east = e_len > 1e-12 ? e / e_len : DVec3(0.0, 1.0, 0.0);
	f.south = cross(f.east, f.up);
	return f;
}

DVec3 RenderFrame::to_local_vec(const DVec3 &p_v) const {
	return { dot(p_v, east), dot(p_v, up), dot(p_v, south) };
}

DVec3 RenderFrame::to_local_point(const DVec3 &p_p) const {
	return to_local_vec(p_p - anchor);
}

NodeConstants node_constants(const PatchSpec &p_spec, double p_r_center, const RenderFrame &p_frame) {
	const int64_t m = face_intervals(p_spec);
	const int64_t half = p_spec.intervals / 2;
	const int64_t gc = int64_t(p_spec.key.x()) * p_spec.intervals + half;
	const int64_t gt = int64_t(p_spec.key.y()) * p_spec.intervals + half;
	const double s_c = double(2 * gc - m) / double(m);
	const double t_c = double(2 * gt - m) / double(m);

	const FaceAxes &a = face_axes(p_spec.key.face());
	const DVec3 u = cast<double>(a.u);
	const DVec3 v = cast<double>(a.v);
	const DVec3 q = cast<double>(a.f) + u * warp(s_c) + v * warp(t_c);
	const DVec3 center = q / length(q) * p_r_center;

	NodeConstants nc;
	nc.center = cast<float>(p_frame.to_local_point(center));
	nc.axis_u = cast<float>(p_frame.to_local_vec(u));
	nc.axis_v = cast<float>(p_frame.to_local_vec(v));
	nc.q_center = cast<float>(p_frame.to_local_vec(q));
	nc.q_len = float(length(q));
	nc.s_c = float(s_c);
	nc.t_c = float(t_c);
	nc.cos_sc = float(std::cos(QUARTER_PI * s_c));
	nc.cos_tc = float(std::cos(QUARTER_PI * t_c));
	nc.inv_m = float(1.0 / double(m));
	nc.r_center = float(p_r_center);
	return nc;
}

FVec3 vertex_float(const NodeConstants &p_nc, int p_i, int p_j, int p_intervals, float p_dh) {
	const float qp = float(QUARTER_PI);
	const float ds = float(2 * p_i - p_intervals) * p_nc.inv_m;
	const float dt = float(2 * p_j - p_intervals) * p_nc.inv_m;

	// tan(a) - tan(b) = sin(a - b) / (cos a cos b): the
	// difference never cancels against a large value.
	const float dx = std::sin(qp * ds) / (std::cos(qp * (p_nc.s_c + ds)) * p_nc.cos_sc);
	const float dy = std::sin(qp * dt) / (std::cos(qp * (p_nc.t_c + dt)) * p_nc.cos_tc);

	const FVec3 dq = p_nc.axis_u * dx + p_nc.axis_v * dy;
	const FVec3 q = p_nc.q_center + dq;
	const float n = p_nc.q_len;
	const float n2 = length(q);
	const float k = (2.0f * dot(p_nc.q_center, dq) + dot(dq, dq)) / (n * n2 * (n + n2));
	const FVec3 dir_delta = dq / n2 - p_nc.q_center * k;
	return p_nc.center + dir_delta * p_nc.r_center + (q / n2) * p_dh;
}

DVec3 vertex_reference(const PatchSpec &p_spec, int p_i, int p_j, double p_r, const RenderFrame &p_frame) {
	const int64_t m = face_intervals(p_spec);
	const FaceSample sample{
		p_spec.key.face(),
		int64_t(p_spec.key.x()) * p_spec.intervals + p_i,
		int64_t(p_spec.key.y()) * p_spec.intervals + p_j,
	};
	return p_frame.to_local_point(lattice_dir(lattice_point(sample, m), m) * p_r);
}

} // namespace regolith

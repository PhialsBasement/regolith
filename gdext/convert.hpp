#pragma once

#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "regolith/patch.hpp"
#include "regolith/vec.hpp"

namespace regolith {

inline DVec3 to_d(const godot::Vector3 &p_v) {
	return { double(p_v.x), double(p_v.y), double(p_v.z) };
}

inline godot::Vector3 to_v(const DVec3 &p_v) {
	return godot::Vector3(real_t(p_v.x), real_t(p_v.y), real_t(p_v.z));
}

// Body-fixed vector from a render-frame vector (the inverse
// of RenderFrame::to_local_vec).
inline DVec3 to_body_vec(const RenderFrame &p_f, const DVec3 &p_v) {
	return p_f.east * p_v.x + p_f.up * p_v.y + p_f.south * p_v.z;
}

inline DVec3 to_body_point(const RenderFrame &p_f, const DVec3 &p_v) {
	return p_f.anchor + to_body_vec(p_f, p_v);
}

// Render-frame basis whose columns are the body-fixed axes
// of p_local (east/up/south) expressed in p_frame.
inline godot::Basis basis_between(const RenderFrame &p_frame, const RenderFrame &p_local) {
	const DVec3 x = p_frame.to_local_vec(p_local.east);
	const DVec3 y = p_frame.to_local_vec(p_local.up);
	const DVec3 z = p_frame.to_local_vec(p_local.south);
	godot::Basis b;
	b.set_column(0, to_v(x));
	b.set_column(1, to_v(y));
	b.set_column(2, to_v(z));
	return b;
}

} // namespace regolith

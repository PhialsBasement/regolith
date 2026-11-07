// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vector>

#include <godot_cpp/classes/rd_uniform.hpp>
#include <godot_cpp/classes/rendering_device.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/projection.hpp>
#include <godot_cpp/variant/rid.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "regolith/vec.hpp"

// RenderingDevice helpers shared by Regolith's compute effects.
namespace godot::rdu {

using RD = RenderingDevice;

PackedByteArray bytes_of(const void *p_data, size_t p_size);
// a sampled texture filled with p_data, updatable
RID make_texture(RD *p_rd, RD::DataFormat p_format, int p_w, int p_h, int p_d, const PackedByteArray &p_data);
// a storage and sampled texture; 6 layers make a cube map
RID make_storage(RD *p_rd, RD::DataFormat p_format, int p_w, int p_h, int p_d, int p_mips = 1, int p_layers = 1);
// res://addons/regolith/NAME.glsl, #include lines inlined; the
// importer's SPIR-V in an exported game. Invalid on error.
RID compile(RD *p_rd, const String &p_name);
RID make_sampler(RD *p_rd, bool p_linear, RD::SamplerRepeatMode p_u, RD::SamplerRepeatMode p_vw);
Ref<RDUniform> uniform(RD::UniformType p_type, int p_binding, RID p_a, RID p_b = RID());
Ref<RDUniform> image(int p_binding, RID p_tex);
Ref<RDUniform> sampled(int p_binding, RID p_sampler, RID p_tex);
// a cached uniform set for set 0 of p_shader
RID set_of(RID p_shader, const std::vector<Ref<RDUniform>> &p_list);
void push4(std::vector<float> &r_v, double p_x, double p_y, double p_z, double p_w);
void push3(std::vector<float> &r_v, const Vector3 &p_v, double p_w);
void push3(std::vector<float> &r_v, const regolith::DVec3 &p_v, double p_w);
void push_mat(std::vector<float> &r_v, const Projection &p_m);
uint32_t groups(int p_n, int p_local);

} // namespace godot::rdu

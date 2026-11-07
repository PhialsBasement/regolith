#include "rd_util.hpp"

#include <cstring>

#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/rd_sampler_state.hpp>
#include <godot_cpp/classes/rd_shader_file.hpp>
#include <godot_cpp/classes/rd_shader_source.hpp>
#include <godot_cpp/classes/rd_shader_spirv.hpp>
#include <godot_cpp/classes/rd_texture_format.hpp>
#include <godot_cpp/classes/rd_texture_view.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/uniform_set_cache_rd.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

namespace godot::rdu {

namespace {

// a C string: a global godot::String would be built before
// the extension interface exists
constexpr const char *DIR = "res://addons/regolith/";

// The source with its #include lines inlined, from the same
// directory; empty when the file is absent (an export).
String source_of(const String &p_path) {
	const String code = FileAccess::get_file_as_string(p_path);
	if (code.is_empty()) {
		return code;
	}
	String out;
	for (const String &line : code.split("\n")) {
		const String t = line.strip_edges();
		if (t.begins_with("#[")) {
			continue;
		}
		if (t.begins_with("#include")) {
			out += source_of(String(DIR) + t.get_slice("\"", 1));
		} else {
			out += line;
		}
		out += String("\n");
	}
	return out;
}

} // namespace

PackedByteArray bytes_of(const void *p_data, size_t p_size) {
	PackedByteArray a;
	a.resize(int64_t(p_size));
	std::memcpy(a.ptrw(), p_data, p_size);
	return a;
}

RID make_texture(RD *p_rd, RD::DataFormat p_format, int p_w, int p_h, int p_d, const PackedByteArray &p_data) {
	Ref<RDTextureFormat> f;
	f.instantiate();
	f->set_format(p_format);
	f->set_width(uint32_t(p_w));
	f->set_height(uint32_t(p_h));
	f->set_depth(uint32_t(p_d));
	f->set_texture_type(p_d > 1 ? RD::TEXTURE_TYPE_3D : RD::TEXTURE_TYPE_2D);
	f->set_usage_bits(RD::TEXTURE_USAGE_SAMPLING_BIT | RD::TEXTURE_USAGE_CAN_UPDATE_BIT);
	Ref<RDTextureView> v;
	v.instantiate();
	TypedArray<PackedByteArray> data;
	data.push_back(p_data);
	return p_rd->texture_create(f, v, data);
}

RID make_storage(RD *p_rd, RD::DataFormat p_format, int p_w, int p_h, int p_d, int p_mips, int p_layers) {
	Ref<RDTextureFormat> f;
	f.instantiate();
	f->set_format(p_format);
	f->set_width(uint32_t(p_w));
	f->set_height(uint32_t(p_h));
	f->set_depth(uint32_t(p_d));
	f->set_mipmaps(uint32_t(p_mips));
	f->set_array_layers(uint32_t(p_layers));
	f->set_texture_type(p_layers == 6 ? RD::TEXTURE_TYPE_CUBE :
			(p_d > 1 ? RD::TEXTURE_TYPE_3D : RD::TEXTURE_TYPE_2D));
	f->set_usage_bits(RD::TEXTURE_USAGE_STORAGE_BIT | RD::TEXTURE_USAGE_SAMPLING_BIT |
			RD::TEXTURE_USAGE_CAN_COPY_FROM_BIT);
	Ref<RDTextureView> v;
	v.instantiate();
	return p_rd->texture_create(f, v);
}

RID compile(RD *p_rd, const String &p_name) {
	const String path = String(DIR) + p_name + ".glsl";
	const String code = source_of(path);
	Ref<RDShaderSPIRV> spirv;
	if (code.is_empty()) {
		// an exported game ships the importer's SPIR-V only
		Ref<RDShaderFile> file = ResourceLoader::get_singleton()->load(path);
		if (file.is_valid()) {
			spirv = file->get_spirv();
		}
	} else {
		Ref<RDShaderSource> src;
		src.instantiate();
		src->set_stage_source(RD::SHADER_STAGE_COMPUTE, code);
		spirv = p_rd->shader_compile_spirv_from_source(src);
	}
	const String err = spirv.is_valid() ? spirv->get_stage_compile_error(RD::SHADER_STAGE_COMPUTE) : "no SPIR-V";
	if (!err.is_empty()) {
		UtilityFunctions::push_error("Regolith: ", p_name, " failed: ", err);
		return RID();
	}
	return p_rd->shader_create_from_spirv(spirv, p_name);
}

RID make_sampler(RD *p_rd, bool p_linear, RD::SamplerRepeatMode p_u, RD::SamplerRepeatMode p_vw) {
	Ref<RDSamplerState> s;
	s.instantiate();
	const RD::SamplerFilter f = p_linear ? RD::SAMPLER_FILTER_LINEAR : RD::SAMPLER_FILTER_NEAREST;
	s->set_min_filter(f);
	s->set_mag_filter(f);
	s->set_mip_filter(f);
	s->set_repeat_u(p_u);
	s->set_repeat_v(p_vw);
	s->set_repeat_w(p_vw);
	return p_rd->sampler_create(s);
}

Ref<RDUniform> uniform(RD::UniformType p_type, int p_binding, RID p_a, RID p_b) {
	Ref<RDUniform> u;
	u.instantiate();
	u->set_uniform_type(p_type);
	u->set_binding(p_binding);
	u->add_id(p_a);
	if (p_b.is_valid()) {
		u->add_id(p_b);
	}
	return u;
}

Ref<RDUniform> image(int p_binding, RID p_tex) {
	return uniform(RD::UNIFORM_TYPE_IMAGE, p_binding, p_tex);
}

Ref<RDUniform> sampled(int p_binding, RID p_sampler, RID p_tex) {
	return uniform(RD::UNIFORM_TYPE_SAMPLER_WITH_TEXTURE, p_binding, p_sampler, p_tex);
}

RID set_of(RID p_shader, const std::vector<Ref<RDUniform>> &p_list) {
	TypedArray<Ref<RDUniform>> s;
	for (const Ref<RDUniform> &x : p_list) {
		s.push_back(x);
	}
	return UniformSetCacheRD::get_cache(p_shader, 0, s);
}

void push4(std::vector<float> &r_v, double p_x, double p_y, double p_z, double p_w) {
	r_v.insert(r_v.end(), { float(p_x), float(p_y), float(p_z), float(p_w) });
}

void push3(std::vector<float> &r_v, const Vector3 &p_v, double p_w) {
	push4(r_v, p_v.x, p_v.y, p_v.z, p_w);
}

void push3(std::vector<float> &r_v, const regolith::DVec3 &p_v, double p_w) {
	push4(r_v, p_v.x, p_v.y, p_v.z, p_w);
}

void push_mat(std::vector<float> &r_v, const Projection &p_m) {
	for (int c = 0; c < 4; ++c) {
		push4(r_v, p_m.columns[c].x, p_m.columns[c].y, p_m.columns[c].z, p_m.columns[c].w);
	}
}

uint32_t groups(int p_n, int p_local) {
	return uint32_t((p_n + p_local - 1) / p_local);
}

} // namespace godot::rdu

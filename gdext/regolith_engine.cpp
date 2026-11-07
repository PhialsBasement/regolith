// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith_engine.hpp"

#include <atomic>
#include <filesystem>

#include <godot_cpp/classes/dir_access.hpp>
#include <godot_cpp/classes/file_access.hpp>
#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/classes/resource_loader.hpp>
#include <godot_cpp/classes/shader_include.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

namespace godot {

// The engine registers the setting with a default; a project
// saved by the patched editor holds it too, but with none.
bool regolith_engine_patched() {
	const Variant d = ProjectSettings::get_singleton()->property_get_revert("rendering/forest/gpu_occlusion");
	return d.get_type() == Variant::BOOL;
}

bool regolith_engine_double() {
	// the library the engine loaded matches its precision
	return sizeof(real_t) == sizeof(double);
}

String regolith_terrain_shader(const String &p_name) {
	const String suffix = regolith_engine_patched() ? String() : String("_stock");
	return String("res://addons/regolith/") + p_name + suffix + String(".gdshader");
}

namespace {

String include_code(const String &p_path) {
	Ref<ShaderInclude> inc = ResourceLoader::get_singleton()->load(p_path);
	if (inc.is_null()) {
		UtilityFunctions::push_error("Regolith: cannot load the shader include ", p_path);
		return String();
	}
	return inc->get_code();
}

// Each call to p_name(...) becomes p_with; with p_statement the
// whole statement goes, up to its ");".
String replace_calls(const String &p_code, const String &p_name, const String &p_with, bool p_statement) {
	String out = p_code;
	const String open = p_name + String("(");
	int64_t at = out.find(open);
	while (at >= 0) {
		const int64_t end = p_statement ? out.find(");", at) : out.find(")", at);
		if (end < 0) {
			break;
		}
		const int64_t stop = end + (p_statement ? 2 : 1);
		out = out.substr(0, at) + p_with + out.substr(stop);
		at = out.find(open, at + p_with.length());
	}
	return out;
}

// Stock Godot lacks the patched engine's render mode, depth-pass
// flag and section profiler: the same shader without them.
String for_stock(const String &p_code) {
	String c = p_code.replace(", motion_vectors_rigid", "").replace("IN_DEPTH_PREPASS", "false");
	c = replace_calls(c, "profile_clock", "0u", false);
	return replace_calls(c, "profile_add", ";", true);
}

} // namespace

std::string regolith_disk_path(const String &p_path) {
	ProjectSettings *ps = ProjectSettings::get_singleton();
	const String g = ps->globalize_path(p_path);
	std::error_code ec;
	if (std::filesystem::exists(g.utf8().get_data(), ec) || !p_path.begins_with("res://")) {
		return g.utf8().get_data();
	}
	// exported: inside the .pck, so copied out to user:// once
	Ref<FileAccess> in = FileAccess::open(p_path, FileAccess::READ);
	if (in.is_null()) {
		return g.utf8().get_data();
	}
	const String dst = String("user://regolith_data/") + p_path.substr(6);
	const std::string out_path = ps->globalize_path(dst).utf8().get_data();
	const uint64_t len = in->get_length();
	if (std::filesystem::exists(out_path, ec) && std::filesystem::file_size(out_path, ec) == len) {
		return out_path;
	}
	DirAccess::make_dir_recursive_absolute(ps->globalize_path(dst).get_base_dir());
	const String part = dst + String(".part");
	Ref<FileAccess> out = FileAccess::open(part, FileAccess::WRITE);
	if (out.is_null()) {
		UtilityFunctions::push_error("Regolith: cannot write ", dst);
		return g.utf8().get_data();
	}
	UtilityFunctions::print("Regolith: unpacking ", p_path, " (", int64_t(len / 1000000), " MB) to ", dst);
	for (uint64_t done = 0; done < len;) {
		const PackedByteArray chunk = in->get_buffer(int64_t(std::min<uint64_t>(len - done, 8u << 20)));
		if (chunk.is_empty()) {
			break;
		}
		out->store_buffer(chunk);
		done += uint64_t(chunk.size());
	}
	out->close();
	// whole or nothing: a half copy never passes for the pack
	std::filesystem::rename(ps->globalize_path(part).utf8().get_data(), out_path, ec);
	return out_path;
}

Ref<Shader> regolith_clipmap_shader(const String &p_surface_path) {
	const String core = include_code("res://addons/regolith/regolith_clipmap_core.gdshaderinc");
	const String path = p_surface_path.is_empty() ? String("res://addons/regolith/regolith_surface_default.gdshaderinc")
												  : p_surface_path;
	const String surface = include_code(path);
	if (core.is_empty() || surface.is_empty()) {
		return Ref<Shader>();
	}
	String code = core.replace("// @surface", surface);
	if (!regolith_engine_patched()) {
		code = for_stock(code);
	}
	Ref<Shader> s;
	s.instantiate();
	s->set_code(code);
	return s;
}

String regolith_engine_warning() {
	String s;
	if (!regolith_engine_double()) {
		s += String("SINGLE PRECISION: REGOLITH IS BROKEN IN THIS GODOT.\n\n") +
				String("This Godot uses single-precision floats. At planet scale positions far from the ") +
				String("origin fall apart: terrain cracks and swims, physics and orbits go wrong. Regolith ") +
				String("does not support this. Use a double-precision Godot (precision=double), as ") +
				String("regolith/engine/build_godot.sh (Linux) or build_godot.ps1 (Windows) builds it, ") +
				String("and the matching double export templates.");
	}
	if (!regolith_engine_patched()) {
		if (!s.is_empty()) {
			s += String("\n\n");
		}
		s += String("Regolith is running on STOCK Godot.\n\n") +
				String("Its terrain falls back to slow shaders: expect FAR lower frame rates than Regolith is made ") +
				String("for. Build the patched engine with regolith/engine/build_godot.sh (Linux, macOS) or ") +
				String("build_godot.ps1 (Windows) and open the project with that editor.");
	}
	return s;
}

void regolith_engine_warn_once() {
	static std::atomic<bool> done{ false };
	if (done.exchange(true)) {
		return;
	}
	const String w = regolith_engine_warning();
	if (!w.is_empty()) {
		UtilityFunctions::push_error(String("REGOLITH ENGINE WARNING\n") + w);
	}
}

} // namespace godot

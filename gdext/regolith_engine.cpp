#include "regolith_engine.hpp"

#include <atomic>

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
	if (!regolith_engine_patched()) {
		s += String("Regolith is running on STOCK Godot.\n\n") +
				String("Its terrain falls back to slow shaders: expect FAR lower frame rates than Regolith is made ") +
				String("for. Build the patched engine with regolith/engine/build_godot.sh (Linux, macOS) or ") +
				String("build_godot.ps1 (Windows) and open the project with that editor.");
	}
	if (!regolith_engine_double()) {
		if (!s.is_empty()) {
			s += String("\n\n");
		}
		s += String("This Godot uses single precision: at planet scale, positions far from the origin jitter. ") +
				String("Regolith is made for double precision (precision=double), as build_godot builds it.");
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
		UtilityFunctions::push_error(String("REGOLITH PERFORMANCE WARNING\n") + w);
	}
}

} // namespace godot

#include "regolith_engine.hpp"

#include <atomic>

#include <godot_cpp/classes/project_settings.hpp>
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

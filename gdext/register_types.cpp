#include <gdextension_interface.h>

#include <godot_cpp/classes/editor_plugin_registration.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "regolith_atmosphere.hpp"
#include "regolith_body.hpp"
#include "regolith_clouds.hpp"
#include "regolith_editor.hpp"
#include "regolith_weather.hpp"
#include "regolith_collider.hpp"
#include "regolith_system.hpp"
#include "regolith_vessel.hpp"

using namespace godot;

namespace {

void initialize_regolith(ModuleInitializationLevel p_level) {
	if (p_level == MODULE_INITIALIZATION_LEVEL_EDITOR) {
		GDREGISTER_INTERNAL_CLASS(RegolithEditorPlugin);
		EditorPlugins::add_by_type<RegolithEditorPlugin>();
		return;
	}
	if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	GDREGISTER_CLASS(RegolithSystem);
	GDREGISTER_CLASS(RegolithBody);
	GDREGISTER_CLASS(RegolithCollider);
	GDREGISTER_CLASS(RegolithVessel);
	GDREGISTER_CLASS(RegolithAirEffect);
	GDREGISTER_CLASS(RegolithAtmosphere);
	GDREGISTER_CLASS(RegolithWeather);
	GDREGISTER_CLASS(RegolithCloudEffect);
	GDREGISTER_CLASS(RegolithClouds);
}

void uninitialize_regolith(ModuleInitializationLevel p_level) {
}

} // namespace

extern "C" {

GDExtensionBool GDE_EXPORT regolith_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address, GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
	init_obj.register_initializer(initialize_regolith);
	init_obj.register_terminator(uninitialize_regolith);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
	return init_obj.init();
}

}

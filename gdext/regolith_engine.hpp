// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <godot_cpp/classes/shader.hpp>
#include <godot_cpp/variant/string.hpp>

namespace godot {

// Which Godot Regolith runs on. The patched engine (engine/
// build_godot.sh) adds rendering/forest/* settings and the
// terrain shaders' fast paths; stock Godot gets slow variants.
bool regolith_engine_patched();
bool regolith_engine_double();
// the terrain shader for this engine: name without extension
String regolith_terrain_shader(const String &p_name);
// The clipmap terrain: the core with a surface (.gdshaderinc,
// empty: Regolith's default) inlined, fitted to this engine.
Ref<Shader> regolith_clipmap_shader(const String &p_surface_path);
// One loud error per run when the engine is stock or single
// precision: performance is far below what Regolith is made for.
void regolith_engine_warn_once();
// the warning's text, shared with the editor dialog
String regolith_engine_warning();

} // namespace godot

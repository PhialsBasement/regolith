// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <godot_cpp/classes/editor_plugin.hpp>

namespace godot {

class Label;

// The editor side: on a stock or single-precision Godot, a big
// warning when the project opens (dismissable per project) and
// a red notice in the 3D toolbar that stays.
class RegolithEditorPlugin : public EditorPlugin {
	GDCLASS(RegolithEditorPlugin, EditorPlugin)

public:
	void _enter_tree() override;
	void _exit_tree() override;

protected:
	static void _bind_methods();

private:
	void show_warning();
	void on_action(const StringName &p_action, Object *p_dialog);

	Label *notice_ = nullptr;
};

} // namespace godot

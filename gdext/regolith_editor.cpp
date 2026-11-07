// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "regolith_editor.hpp"

#include <godot_cpp/classes/accept_dialog.hpp>
#include <godot_cpp/classes/button.hpp>
#include <godot_cpp/classes/editor_interface.hpp>
#include <godot_cpp/classes/editor_settings.hpp>
#include <godot_cpp/classes/label.hpp>
#include <godot_cpp/core/class_db.hpp>

#include "regolith_engine.hpp"

using namespace godot;

namespace {

const char *META_SECTION = "regolith";
const char *META_KEY = "engine_warning_dismissed";
const char *DISMISS = "dismiss";

} // namespace

void RegolithEditorPlugin::_bind_methods() {
	ClassDB::bind_method(D_METHOD("show_warning"), &RegolithEditorPlugin::show_warning);
	ClassDB::bind_method(D_METHOD("on_action", "action", "dialog"), &RegolithEditorPlugin::on_action);
}

void RegolithEditorPlugin::_enter_tree() {
	if (regolith_engine_warning().is_empty()) {
		return;
	}
	notice_ = memnew(Label);
	notice_->set_text(regolith_engine_patched() ? String("Regolith: single precision")
												: String("Regolith: STOCK GODOT, SLOW TERRAIN"));
	notice_->add_theme_color_override("font_color", Color(1.0, 0.25, 0.2));
	notice_->set_tooltip_text(regolith_engine_warning());
	notice_->set_mouse_filter(Control::MOUSE_FILTER_PASS);
	add_control_to_container(CONTAINER_SPATIAL_EDITOR_MENU, notice_);
	regolith_engine_warn_once();
	// once the editor has laid itself out
	call_deferred("show_warning");
}

void RegolithEditorPlugin::_exit_tree() {
	if (notice_) {
		remove_control_from_container(CONTAINER_SPATIAL_EDITOR_MENU, notice_);
		notice_->queue_free();
		notice_ = nullptr;
	}
}

void RegolithEditorPlugin::show_warning() {
	EditorInterface *ei = EditorInterface::get_singleton();
	if (bool(ei->get_editor_settings()->get_project_metadata(META_SECTION, META_KEY, false))) {
		return;
	}
	AcceptDialog *d = memnew(AcceptDialog);
	d->set_title("Regolith: slow engine");
	d->set_text(regolith_engine_warning());
	d->set_autowrap(true);
	d->get_label()->add_theme_font_size_override("font_size", 22);
	d->get_label()->add_theme_color_override("font_color", Color(1.0, 0.45, 0.35));
	d->get_ok_button()->set_text("I understand");
	d->add_button("Don't show again for this project", true, DISMISS);
	d->connect("custom_action", Callable(this, "on_action").bind(d));
	d->connect("confirmed", Callable(d, "queue_free"));
	d->connect("canceled", Callable(d, "queue_free"));
	ei->popup_dialog_centered(d, Vector2i(760, 360));
}

void RegolithEditorPlugin::on_action(const StringName &p_action, Object *p_dialog) {
	if (String(p_action) == String(DISMISS)) {
		EditorInterface::get_singleton()->get_editor_settings()->set_project_metadata(META_SECTION, META_KEY, true);
	}
	if (AcceptDialog *d = Object::cast_to<AcceptDialog>(p_dialog)) {
		d->queue_free();
	}
}

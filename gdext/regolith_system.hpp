#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/string.hpp>

namespace godot {

// Owns the clock, the bodies and the floating origin. In M0
// it only proves the core library is linked and sane.
class RegolithSystem : public Node {
	GDCLASS(RegolithSystem, Node)

protected:
	static void _bind_methods();

public:
	String get_version() const;
	Dictionary self_test() const;
};

} // namespace godot

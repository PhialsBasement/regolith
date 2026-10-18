#include "regolith_system.hpp"

#include <cmath>
#include <cstdio>
#include <random>

#include <godot_cpp/core/class_db.hpp>

#include "regolith/cube.hpp"

using namespace godot;

void RegolithSystem::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_version"), &RegolithSystem::get_version);
	ClassDB::bind_method(D_METHOD("self_test"), &RegolithSystem::self_test);
}

String RegolithSystem::get_version() const {
	return "0.0.1-m0";
}

Dictionary RegolithSystem::self_test() const {
	std::mt19937_64 rng(7);
	std::normal_distribution<double> n(0.0, 1.0);
	double worst = 0.0;
	for (int i = 0; i < 100000; ++i) {
		const regolith::DVec3 d = regolith::normalize(regolith::DVec3(n(rng), n(rng), n(rng)));
		const regolith::DVec3 back = regolith::face_dir(regolith::dir_to_face(d));
		worst = std::fmax(worst, regolith::length(back - d));
	}
	// Godot prints floats with fixed decimals, so 4e-16 would
	// show as 0.0; hand back a scientific string as well.
	char text[32];
	std::snprintf(text, sizeof(text), "%.3e", worst);
	Dictionary result;
	result["dir_round_trip_worst_rad"] = worst;
	result["dir_round_trip_worst_text"] = String(text);
	result["ok"] = worst < 1e-15;
	return result;
}

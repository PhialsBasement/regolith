// This Source Code Form is subject to the terms of the
// Mozilla Public License, v. 2.0. If a copy of the MPL was
// not distributed with this file, You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <godot_cpp/variant/dictionary.hpp>

#include "regolith/climate.hpp"

namespace godot {

// The climate a weather config describes: each key is a ClimateConfig
// or ClimateModel field name (the lapse profile's as lapse_low,
// lapse_high, lapse_turn_m, air_floor_k). A key left out keeps Mars's
// value; one that does not fit (wrong type or shape, out of range) or
// that is not a key at all is reported, and the value stays Mars's.
regolith::ClimateConfig climate_config_from(const Dictionary &p_config);

} // namespace godot

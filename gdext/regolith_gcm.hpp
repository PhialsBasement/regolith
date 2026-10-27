#pragma once

#include <memory>

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/string.hpp>

#include "regolith/gcm.hpp"

namespace godot {

// The GCM itself, for scripts and harnesses: RegolithWeather
// drives one on its own thread for the game.
class RegolithGcm : public RefCounted {
	GDCLASS(RegolithGcm, RefCounted)

public:
	// Body and grid parameters (GcmConfig names); Mars when unset.
	void configure(const Dictionary &p_config);
	static regolith::GcmConfig config_from(const Dictionary &p_config);
	void create(const PackedFloat32Array &p_topo, const PackedFloat32Array &p_lum, double p_ls, int p_threads);
	void step(const Dictionary &p_inputs, bool p_spinup);
	PackedFloat32Array get_field(const String &p_name) const;
	void set_field(const String &p_name, const PackedFloat32Array &p_value);
	Dictionary get_scalars() const;
	void set_scalars(const Dictionary &p_value);
	void set_clim(const PackedFloat32Array &p_value);
	PackedByteArray get_meso() const;

	regolith::Gcm &model() { return *gcm_; }
	static regolith::Gcm::Inputs inputs_from(const Dictionary &p_inputs);
	static regolith::Gcm::Field *field_of(regolith::Gcm &p_gcm, const String &p_name);

protected:
	static void _bind_methods();

private:
	std::unique_ptr<regolith::Gcm> gcm_ = std::make_unique<regolith::Gcm>();
};

} // namespace godot

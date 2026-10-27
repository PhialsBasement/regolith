shader_type canvas_item;

// Cloud transmittance of the sun around the eye (mors's
// cloud_sun_od and shadow decal): each texel is a ground point
// on the tangent plane; the sun's ray crosses the ice slabs.
#include "res://addons/regolith/regolith_clouds.gdshaderinc"

uniform vec3 shadow_origin; // render space, under the eye
uniform vec3 shadow_e; // render-space axes of the map
uniform vec3 shadow_s;
uniform float shadow_size = 100000.0;
const float EXT[3] = float[](0.0018, 0.0009, 0.0009);

void fragment() {
	vec2 off = (UV - 0.5) * shadow_size;
	vec3 w = shadow_origin + shadow_e * off.x + shadow_s * off.y;
	vec3 q = u_to_body * (w - u_center);
	q = normalize(q) * (u_datum_r + ground_alt);
	float sun_up = dot(sun_body, normalize(q));
	float od = 0.0;
	if (sun_up > 0.02) {
		for (int l = 0; l < 3; l++) {
			float b;
			float tp;
			slab_bounds(l, b, tp);
			float run0 = (b - ground_alt) / sun_up;
			float run1 = (tp - ground_alt) / sun_up;
			float seg = (run1 - run0) / 4.0;
			float acc = 0.0;
			for (int i = 0; i < 4; i++) {
				acc += layer_density(q + sun_body * (run0 + seg * (float(i) + 0.5)), l);
			}
			od += acc * seg * EXT[l];
		}
	}
	float r = length(UV - 0.5) * 2.0;
	float t = mix(1.0, exp(-od), 1.0 - smoothstep(0.7, 1.0, r));
	COLOR = vec4(t, t, t, 1.0);
}

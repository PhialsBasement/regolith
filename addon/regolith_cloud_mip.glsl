#[compute]
#version 450

// One mip of the shape noise: the box mean of the level above,
// so a coarse cascade sees the noise filtered, never aliased.

layout(local_size_x = 4, local_size_y = 4, local_size_z = 4) in;

layout(rgba8, set = 0, binding = 0) uniform writeonly image3D dst;
layout(set = 0, binding = 1) uniform sampler3D src;

void main() {
	ivec3 v = ivec3(gl_GlobalInvocationID);
	if (any(greaterThanEqual(v, imageSize(dst)))) {
		return;
	}
	vec4 s = vec4(0.0);
	for (int i = 0; i < 8; i++) {
		s += texelFetch(src, v * 2 + ivec3(i & 1, (i >> 1) & 1, i >> 2), 0);
	}
	imageStore(dst, v, s * 0.125);
}

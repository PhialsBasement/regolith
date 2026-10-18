#pragma once

#include <cmath>
#include <cstdint>

namespace regolith {

template <typename T>
struct Vec3 {
	T x{};
	T y{};
	T z{};

	constexpr Vec3() = default;
	constexpr Vec3(T p_x, T p_y, T p_z) :
			x(p_x), y(p_y), z(p_z) {}

	constexpr Vec3 operator+(const Vec3 &p_o) const { return { x + p_o.x, y + p_o.y, z + p_o.z }; }
	constexpr Vec3 operator-(const Vec3 &p_o) const { return { x - p_o.x, y - p_o.y, z - p_o.z }; }
	constexpr Vec3 operator*(T p_s) const { return { x * p_s, y * p_s, z * p_s }; }
	constexpr Vec3 operator/(T p_s) const { return { x / p_s, y / p_s, z / p_s }; }
	constexpr bool operator==(const Vec3 &) const = default;
};

using DVec3 = Vec3<double>;
using FVec3 = Vec3<float>;
using IVec3 = Vec3<int64_t>;

template <typename T>
constexpr T dot(const Vec3<T> &p_a, const Vec3<T> &p_b) {
	return p_a.x * p_b.x + p_a.y * p_b.y + p_a.z * p_b.z;
}

template <typename T>
constexpr Vec3<T> cross(const Vec3<T> &p_a, const Vec3<T> &p_b) {
	return { p_a.y * p_b.z - p_a.z * p_b.y, p_a.z * p_b.x - p_a.x * p_b.z, p_a.x * p_b.y - p_a.y * p_b.x };
}

template <typename T>
T length(const Vec3<T> &p_a) {
	return std::sqrt(dot(p_a, p_a));
}

template <typename T>
Vec3<T> normalize(const Vec3<T> &p_a) {
	return p_a / length(p_a);
}

template <typename To, typename From>
constexpr Vec3<To> cast(const Vec3<From> &p_a) {
	return { static_cast<To>(p_a.x), static_cast<To>(p_a.y), static_cast<To>(p_a.z) };
}

} // namespace regolith

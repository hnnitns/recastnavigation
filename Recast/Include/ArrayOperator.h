#pragma once

#include <array>
#include <type_traits>

template <class _Ty, size_t _Size>
static inline void operator+=(std::array<_Ty, _Size>& v1, const std::array<_Ty, _Size>& v2)
{
	for (size_t i = 0; i < _Size; i++)
	{
		v1[i] += v2[i];
	}
}

template <class _Ty, size_t _Size>
static inline void operator-=(std::array<_Ty, _Size>& v1, const std::array<_Ty, _Size>& v2)
{
	for (size_t i = 0; i < _Size; i++)
	{
		v1[i] -= v2[i];
	}
}

template <class _Ty, size_t _Size>
static inline void operator/=(std::array<_Ty, _Size>& v1, const std::array<_Ty, _Size>& v2)
{
	for (size_t i = 0; i < _Size; i++)
	{
		v1[i] /= v2[i];
	}
}

template <class _Ty, size_t _Size>
static inline void operator*=(std::array<_Ty, _Size>& v1, const std::array<_Ty, _Size>& v2)
{
	for (size_t i = 0; i < _Size; i++)
	{
		v1[i] *= v2[i];
	}
}

template <class _Ty, size_t _Size>
[[nodiscard]] static inline constexpr auto operator+(const std::array<_Ty, _Size>& v1, const std::array<_Ty, _Size>& v2)
{
	std::remove_const<decltype(v1)> rv{};

	for (size_t i = 0; i < _Size; i++)
	{
		rv[i] = v1[i] + v2[i];
	}

	return rv;
}

template <class _Ty, size_t _Size>
[[nodiscard]] static inline constexpr auto operator-(const std::array<_Ty, _Size>& v1, const std::array<_Ty, _Size>& v2)
{
	std::remove_const<decltype(v1)> rv{};

	for (size_t i = 0; i < _Size; i++)
	{
		rv[i] = v1[i] - v2[i];
	}

	return rv;
}

template <class _Ty, size_t _Size>
[[nodiscard]] static inline constexpr auto operator*(const std::array<_Ty, _Size>& v1, const std::array<_Ty, _Size>& v2)
{
	std::remove_const<decltype(v1)> rv{};

	for (size_t i = 0; i < _Size; i++)
	{
		rv[i] = v1[i] * v2[i];
	}

	return rv;
}

template <class _Ty, size_t _Size>
[[nodiscard]] static inline constexpr auto operator/(const std::array<_Ty, _Size>& v1, const std::array<_Ty, _Size>& v2)
{
	std::remove_const<decltype(v1)> rv{};

	for (size_t i = 0; i < _Size; i++)
	{
		rv[i] = v1[i] / v2[i];
	}

	return rv;
}

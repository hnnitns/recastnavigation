//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef DETOURCOMMON_H
#define DETOURCOMMON_H

#include "DetourMath.h"
#include <cstddef>
#include <array>

/**
@defgroup detour Detour

Members in this module are used to create, manipulate, and query navigation meshes.
このモジュールのメンバーは、ナビゲーションメッシュの作成、操作、およびクエリに使用されます。

@note This is a summary list of members.  Use the index or search feature to find minor members.
	これはメンバーの要約リストです。 インデックスまたは検索機能を使用して、マイナーメンバーを見つけます。

*/

// @name General helper functions
// @{

// Used to ignore a function parameter.  VS complains about unused parameters and this silences the warning.
// 関数パラメーターを無視するために使用されます。 VSは未使用のパラメーターについて不平を言っており、これは警告を黙らせます。
//  @param [in] _ Unused parameter
template<class T> void inline constexpr dtIgnoreUnused(const T&) { }

// Swaps the values of the two parameters.
// 2つのパラメーターの値を交換します。
//  @param[in,out]	a	Value A
//  @param[in,out]	b	Value B
template<class T> inline constexpr void dtSwap(T& a, T& b) { T t = a; a = b; b = t; }

// Returns the minimum of two values.
// 2つの値の最小値を返します。
//  @param[in]		a	Value A
//  @param[in]		b	Value B
//  @return The minimum of the two values.
template<class T> inline constexpr T dtMin(T a, T b) { return a < b ? a : b; }

// Returns the maximum of two values.
// 最大2つの値を返します。
//  @param[in]		a	Value A
//  @param[in]		b	Value B
//  @return The maximum of the two values.
template<class T> inline constexpr T dtMax(T a, T b) { return a > b ? a : b; }

// Returns the absolute value.
// 絶対値を返します。
//  @param[in]		a	The value.
//  @return The absolute value of the specified value.
template<class T> inline constexpr T dtAbs(T a) { return a < 0 ? -a : a; }

// Returns the square of the value.
// 値の二乗を返します。
//  @param[in]		a	The value.
//  @return The square of the value.
template<class T> inline constexpr T dtSqr(T a) { return a * a; }

// Clamps the value to the specified range.
// 指定された範囲に値をクランプします。
//  @param[in]		v	The value to clamp.
//  @param[in]		mn	The minimum permitted return value.
//  @param[in]		mx	The maximum permitted return value.
//  @return The value, clamped to the specified range.
template<class T> inline constexpr T dtClamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

// @}
// @name Vector helper functions.
// @{

// Derives the cross product of two vectors. (@p v1 x @p v2)
// 2つのベクトルの外積を導出します。 （v1 x v2）
//  @param[out]	dest	The cross product. [(x, y, z)]
//  @param[in]		v1		A Vector [(x, y, z)]
//  @param[in]		v2		A vector [(x, y, z)]
inline constexpr void dtVcross(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
	dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
	dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// Derives the cross product of two vectors. (@p v1 x @p v2)
// 2つのベクトルの外積を導出します。 （v1 x v2）
//  @param[out]	dest	The cross product. [(x, y, z)]
//  @param[in]		v1		A Vector [(x, y, z)]
//  @param[in]		v2		A vector [(x, y, z)]
inline constexpr void dtVcross(
	std::array<float, 3>* dest, const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	dest->at(0) = v1[1] * v2[2] - v1[2] * v2[1];
	dest->at(1) = v1[2] * v2[0] - v1[0] * v2[2];
	dest->at(2) = v1[0] * v2[1] - v1[1] * v2[0];
}

// Derives the dot product of two vectors. (@p v1 . @p v2)
// 2つのベクトルのドット積を導出します。 （v1 . v2）
//  @param[in]		v1	A Vector [(x, y, z)]
//  @param[in]		v2	A vector [(x, y, z)]
// @return The dot product.
inline constexpr float dtVdot(const float* v1, const float* v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// Derives the dot product of two vectors. (@p v1 . @p v2)
// 2つのベクトルのドット積を導出します。 （v1 . v2）
//  @param[in]		v1	A Vector [(x, y, z)]
//  @param[in]		v2	A vector [(x, y, z)]
// @return The dot product.
inline constexpr float dtVdot(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
// スケーリングされたベクトルの加算を実行します。（v1 +（v2 * s））
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  @param[in]		v2		The vector to scale and add to @p v1. [(x, y, z)]
//  @param[in]		s		The amount to scale @p v2 by before adding to @p v1.
inline constexpr void dtVmad(float* dest, const float* v1, const float* v2, const float s)
{
	dest[0] = v1[0] + v2[0] * s;
	dest[1] = v1[1] + v2[1] * s;
	dest[2] = v1[2] + v2[2] * s;
}

// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
// スケーリングされたベクトルの加算を実行します。（v1 +（v2 * s））
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  @param[in]		v2		The vector to scale and add to @p v1. [(x, y, z)]
//  @param[in]		s		The amount to scale @p v2 by before adding to @p v1.
inline constexpr void dtVmad(
	std::array<float, 3>* dest, const std::array<float, 3>& v1, const std::array<float, 3>& v2, const float s)
{
	dest->at(0) = v1[0] + v2[0] * s;
	dest->at(1) = v1[1] + v2[1] * s;
	dest->at(2) = v1[2] + v2[2] * s;
}

// Performs a linear interpolation between two vectors. (@p v1 toward @p v2)
// 2つのベクトル間で線形補間を実行します。 （v1からv2へ）
//  @param[out]	dest	The result vector. [(x, y, x)]
//  @param[in]		v1		The starting vector.
//  @param[in]		v2		The destination vector.
//	 @param[in]		t		The interpolation factor. [Limits: 0 <= value <= 1.0]
inline constexpr void dtVlerp(float* dest, const float* v1, const float* v2, const float t)
{
	dest[0] = v1[0] + (v2[0] - v1[0]) * t;
	dest[1] = v1[1] + (v2[1] - v1[1]) * t;
	dest[2] = v1[2] + (v2[2] - v1[2]) * t;
}

// Performs a linear interpolation between two vectors. (@p v1 toward @p v2)
// 2つのベクトル間で線形補間を実行します。 （v1からv2へ）
//  @param[out]	dest	The result vector. [(x, y, x)]
//  @param[in]		v1		The starting vector.
//  @param[in]		v2		The destination vector.
//	 @param[in]		t		The interpolation factor. [Limits: 0 <= value <= 1.0]
inline constexpr void dtVlerp(
	std::array<float, 3>* dest, const std::array<float, 3>& v1, const std::array<float, 3>& v2, const float t)
{
	dest->at(0) = v1[0] + (v2[0] - v1[0]) * t;
	dest->at(1) = v1[1] + (v2[1] - v1[1]) * t;
	dest->at(2) = v1[2] + (v2[2] - v1[2]) * t;
}

// Performs a vector addition. (@p v1 + @p v2)
// ベクトルの加算を実行します。 （v1 + v2）
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  @param[in]		v2		The vector to add to @p v1. [(x, y, z)]
inline constexpr void dtVadd(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0] + v2[0];
	dest[1] = v1[1] + v2[1];
	dest[2] = v1[2] + v2[2];
}

// Performs a vector subtraction. (@p v1 - @p v2)
// ベクトル減算を実行します。（v1 - v2）
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  @param[in]		v2		The vector to subtract from @p v1. [(x, y, z)]
inline constexpr void dtVsub(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0] - v2[0];
	dest[1] = v1[1] - v2[1];
	dest[2] = v1[2] - v2[2];
}

namespace DtOperator
{
	using ArrayF = std::array<float, 3>;

	// Performs a vector addition. (@p v1 + @p v2)
	// ベクトルの加算を実行します。 （v1 + v2）
	//  @param[out]	dest	The result vector. [(x, y, z)]
	//  @param[in]		v1		The base vector. [(x, y, z)]
	//  @param[in]		v2		The vector to add to @p v1. [(x, y, z)]
	inline auto operator+(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		std::array<float, 3> dest;

		dest[0] = v1[0] + v2[0];
		dest[1] = v1[1] + v2[1];
		dest[2] = v1[2] + v2[2];

		return dest;
	}

	inline void operator+=(std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		v1[0] += v2[0];
		v1[1] += v2[1];
		v1[2] += v2[2];
	}

	// Performs a vector subtraction. (@p v1 - @p v2)
	// ベクトル減算を実行します。（v1 - v2）
	//  @param[out]	dest	The result vector. [(x, y, z)]
	//  @param[in]		v1		The base vector. [(x, y, z)]
	//  @param[in]		v2		The vector to subtract from @p v1. [(x, y, z)]
	inline auto operator-(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		std::array<float, 3> dest;

		dest[0] = v1[0] - v2[0];
		dest[1] = v1[1] - v2[1];
		dest[2] = v1[2] - v2[2];

		return dest;
	}

	inline void operator-=(std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		v1[0] -= v2[0];
		v1[1] -= v2[1];
		v1[2] -= v2[2];
	}

	inline auto operator*(const std::array<float, 3>& v1, const float num)
	{
		std::array<float, 3> dest;

		dest[0] = v1[0] * num;
		dest[1] = v1[1] * num;
		dest[2] = v1[2] * num;

		return dest;
	}

	inline void operator*=(std::array<float, 3>& v1, const float num)
	{
		v1[0] *= num;
		v1[1] *= num;
		v1[2] *= num;
	}
}

// Scales the vector by the specified value. (@p v * @p t)
// 指定された値でベクトルをスケーリングします。（v * t）
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		v		The vector to scale. [(x, y, z)]
//  @param[in]		t		The scaling factor.
inline constexpr void dtVscale(float* dest, const float* v, const float t)
{
	dest[0] = v[0] * t;
	dest[1] = v[1] * t;
	dest[2] = v[2] * t;
}

// Selects the minimum value of each element from the specified vectors.
// 指定されたベクトルから各要素の最小値を選択します。
//  @param[in,out]	mn	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]	v	A vector. [(x, y, z)]
inline constexpr void dtVmin(float* mn, const float* v)
{
	mn[0] = dtMin(mn[0], v[0]);
	mn[1] = dtMin(mn[1], v[1]);
	mn[2] = dtMin(mn[2], v[2]);
}

// Selects the minimum value of each element from the specified vectors.
// 指定されたベクトルから各要素の最小値を選択します。
//  @param[in,out]	mn	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]	v	A vector. [(x, y, z)]
inline constexpr void dtVmin(std::array<float, 3>* mn, const std::array<float, 3>& v)
{
	mn->at(0) = dtMin(mn->at(0), v[0]);
	mn->at(1) = dtMin(mn->at(1), v[1]);
	mn->at(2) = dtMin(mn->at(2), v[2]);
}

// Selects the maximum value of each element from the specified vectors.
// 指定されたベクトルから各要素の最大値を選択します。
//  @param[in,out]	mx	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]		v	A vector. [(x, y, z)]
inline constexpr void dtVmax(float* mx, const float* v)
{
	mx[0] = dtMax(mx[0], v[0]);
	mx[1] = dtMax(mx[1], v[1]);
	mx[2] = dtMax(mx[2], v[2]);
}

// Selects the maximum value of each element from the specified vectors.
// 指定されたベクトルから各要素の最大値を選択します。
//  @param[in,out]	mx	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]		v	A vector. [(x, y, z)]
inline constexpr void dtVmax(std::array<float, 3>* mx, const std::array<float, 3>& v)
{
	mx->at(0) = dtMax(mx->at(0), v[0]);
	mx->at(1) = dtMax(mx->at(1), v[1]);
	mx->at(2) = dtMax(mx->at(2), v[2]);
}

// Sets the vector elements to the specified values.
// ベクトル要素を指定された値に設定します。
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		x		The x-value of the vector.
//  @param[in]		y		The y-value of the vector.
//  @param[in]		z		The z-value of the vector.
inline constexpr void dtVset(float* dest, const float x, const float y, const float z)
{
	dest[0] = x; dest[1] = y; dest[2] = z;
}

// Sets the vector elements to the specified values.
// ベクトル要素を指定された値に設定します。
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		x		The x-value of the vector.
//  @param[in]		y		The y-value of the vector.
//  @param[in]		z		The z-value of the vector.
inline constexpr void dtVset(std::array<float, 3>* dest, const float x, const float y, const float z)
{
	dest->at(0) = x; dest->at(1) = y; dest->at(2) = z;
}

// Performs a vector copy.
// ベクターコピーを実行します。
//  @param[out]	dest	The result. [(x, y, z)]
//  @param[in]		a		The vector to copy. [(x, y, z)]
inline constexpr void dtVcopy(float* dest, const float* a)
{
	dest[0] = a[0];
	dest[1] = a[1];
	dest[2] = a[2];
}

// Derives the scalar length of the vector.
// ベクトルのスカラー長を導出します。
//  @param[in]		v The vector. [(x, y, z)]
// @return The scalar length of the vector.
inline float dtVlen(const float* v)
{
	return dtMathSqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Derives the scalar length of the vector.
// ベクトルのスカラー長を導出します。
//  @param[in]		v The vector. [(x, y, z)]
// @return The scalar length of the vector.
inline float dtVlen(const std::array<float, 3>& v)
{
	return dtMathSqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Derives the square of the scalar length of the vector. (len * len)
// ベクトルのスカラー長の二乗を導出します。（len * len）
//  @param[in]		v The vector. [(x, y, z)]
// @return The square of the scalar length of the vector.
inline constexpr float dtVlenSqr(const float* v)
{
	return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

// Derives the square of the scalar length of the vector. (len * len)
// ベクトルのスカラー長の二乗を導出します。（len * len）
//  @param[in]		v The vector. [(x, y, z)]
// @return The square of the scalar length of the vector.
inline constexpr float dtVlenSqr(const std::array<float, 3>& v)
{
	return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

// Returns the distance between two points.
// 2点間の距離を返します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The distance between the two points.
inline float dtVdist(const float* v1, const float* v2)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return dtMathSqrtf(dx * dx + dy * dy + dz * dz);
}

// Returns the distance between two points.
// 2点間の距離を返します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The distance between the two points.
inline float dtVdist(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return dtMathSqrtf(dx * dx + dy * dy + dz * dz);
}

// Returns the square of the distance between two points.
// 2点間の距離の2乗を返します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The square of the distance between the two points.
inline constexpr float dtVdistSqr(const float* v1, const float* v2)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return dx * dx + dy * dy + dz * dz;
}

// Returns the square of the distance between two points.
// 2点間の距離の2乗を返します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The square of the distance between the two points.
inline constexpr float dtVdistSqr(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	const float dx = v2[0] - v1[0];
	const float dy = v2[1] - v1[1];
	const float dz = v2[2] - v1[2];
	return dx * dx + dy * dy + dz * dz;
}

// Derives the distance between the specified points on the xz-plane.
// xz平面上の指定されたポイント間の距離を導出します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The distance between the point on the xz-plane.
//
// The vectors are projected onto the xz-plane, so the y-values are ignored.
// ベクトルはxz平面に投影されるため、y値は無視されます。
inline float dtVdist2D(const float* v1, const float* v2)
{
	const float dx = v2[0] - v1[0];
	const float dz = v2[2] - v1[2];
	return dtMathSqrtf(dx * dx + dz * dz);
}

// Derives the distance between the specified points on the xz-plane.
// xz平面上の指定されたポイント間の距離を導出します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The distance between the point on the xz-plane.
//
// The vectors are projected onto the xz-plane, so the y-values are ignored.
// ベクトルはxz平面に投影されるため、y値は無視されます。
inline float dtVdist2D(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	const float dx = v2[0] - v1[0];
	const float dz = v2[2] - v1[2];
	return dtMathSqrtf(dx * dx + dz * dz);
}

// Derives the square of the distance between the specified points on the xz-plane.
// xz平面上の指定されたポイント間の距離の2乗を導出します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The square of the distance between the point on the xz-plane.
inline constexpr float dtVdist2DSqr(const float* v1, const float* v2)
{
	const float dx = v2[0] - v1[0];
	const float dz = v2[2] - v1[2];
	return dx * dx + dz * dz;
}

// Derives the square of the distance between the specified points on the xz-plane.
// xz平面上の指定されたポイント間の距離の2乗を導出します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The square of the distance between the point on the xz-plane.
inline constexpr float dtVdist2DSqr(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	const float dx = v2[0] - v1[0];
	const float dz = v2[2] - v1[2];
	return dx * dx + dz * dz;
}

// Normalizes the vector.
// ベクトルを正規化します。
//  @param[in,out]	v	The vector to normalize. [(x, y, z)]
inline void dtVnormalize(float* v)
{
	float d = 1.f / dtMathSqrtf(dtSqr(v[0]) + dtSqr(v[1]) + dtSqr(v[2]));
	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

// Normalizes the vector.
// ベクトルを正規化します。
//  @param[in,out]	v	The vector to normalize. [(x, y, z)]
inline void dtVnormalize(std::array<float, 3>* v)
{
	float d = 1.f / dtMathSqrtf(dtSqr(v->at(0)) + dtSqr(v->at(1)) + dtSqr(v->at(2)));
	v->at(0) *= d;
	v->at(1) *= d;
	v->at(2) *= d;
}

// Performs a 'sloppy' colocation check of the specified points.
// 指定されたポイントの「ずさんな」コロケーションチェックを実行します。
//  @param[in]		p0	A point. [(x, y, z)]
//  @param[in]		p1	A point. [(x, y, z)]
// @return True if the points are considered to be at the same location.
// ポイントが同じ場所にあると見なされる場合はTrue。
//
// Basically, this function will return true if the specified points are close enough to eachother to be considered colocated.
// 基本的に、この関数は、指定されたポイントが同じ場所にあると見なされるほど互いに近い場合にtrueを返します。
inline constexpr bool dtVequal(const float* p0, const float* p1)
{
	constexpr float thr = dtSqr(1.f / 16384.0f);
	const float d = dtVdistSqr(p0, p1);
	return d < thr;
}

// Performs a 'sloppy' colocation check of the specified points.
// 指定されたポイントの「ずさんな」コロケーションチェックを実行します。
//  @param[in]		p0	A point. [(x, y, z)]
//  @param[in]		p1	A point. [(x, y, z)]
// @return True if the points are considered to be at the same location.
// ポイントが同じ場所にあると見なされる場合はTrue。
//
// Basically, this function will return true if the specified points are close enough to eachother to be considered colocated.
// 基本的に、この関数は、指定されたポイントが同じ場所にあると見なされるほど互いに近い場合にtrueを返します。
inline constexpr bool dtVequal(const std::array<float, 3>& p0, const std::array<float, 3>& p1)
{
	constexpr float thr = dtSqr(1.f / 16384.0f);
	const float d = dtVdistSqr(p0, p1);
	return d < thr;
}

// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
// xz平面上の2つのベクトルのドット積を導出します。 （u .v）
//  @param[in]		u		A vector [(x, y, z)]
//  @param[in]		v		A vector [(x, y, z)]
// @return The dot product on the xz-plane.
//
// The vectors are projected onto the xz-plane, so the y-values are ignored.
// ベクトルはxz平面に投影されるため、y値は無視されます。
inline constexpr float dtVdot2D(const float* u, const float* v)
{
	return u[0] * v[0] + u[2] * v[2];
}

// Derives the dot product of two vectors on the xz-plane. (@p u . @p v)
// xz平面上の2つのベクトルのドット積を導出します。 （u .v）
//  @param[in]		u		A vector [(x, y, z)]
//  @param[in]		v		A vector [(x, y, z)]
// @return The dot product on the xz-plane.
//
// The vectors are projected onto the xz-plane, so the y-values are ignored.
// ベクトルはxz平面に投影されるため、y値は無視されます。
inline constexpr float dtVdot2D(const std::array<float, 3>& u, const std::array<float, 3>& v)
{
	return u[0] * v[0] + u[2] * v[2];
}

// Derives the xz-plane 2D perp product of the two vectors. (uz * vx - ux * vz)
// 2つのベクトルのxz平面2D perp積を導出します。 （uz * vx - ux * vz）
//  @param[in]		u		The LHV vector [(x, y, z)]
//  @param[in]		v		The RHV vector [(x, y, z)]
// @return The dot product on the xz-plane.
//
// The vectors are projected onto the xz-plane, so the y-values are ignored.
//ベクトルはxz平面に投影されるため、y値は無視されます。
inline constexpr float dtVperp2D(const float* u, const float* v)
{
	return u[2] * v[0] - u[0] * v[2];
}

// Derives the xz-plane 2D perp product of the two vectors. (uz * vx - ux * vz)
// 2つのベクトルのxz平面2D perp積を導出します。 （uz * vx - ux * vz）
//  @param[in]		u		The LHV vector [(x, y, z)]
//  @param[in]		v		The RHV vector [(x, y, z)]
// @return The dot product on the xz-plane.
//
// The vectors are projected onto the xz-plane, so the y-values are ignored.
//ベクトルはxz平面に投影されるため、y値は無視されます。
inline constexpr float dtVperp2D(const std::array<float, 3>& u, const std::array<float, 3>& v)
{
	return u[2] * v[0] - u[0] * v[2];
}

// @}
// @name Computational geometry helper functions.
// @name 計算幾何学ヘルパー関数。
// @{

// Derives the signed xz-plane area of the triangle ABC, or the relationship of line AB to point C.
// 三角形ABCの符号付きxz平面領域、またはラインABとポイントCの関係を導出します。
//  @param[in]		a		Vertex A. [(x, y, z)]
//  @param[in]		b		Vertex B. [(x, y, z)]
//  @param[in]		c		Vertex C. [(x, y, z)]
// @return The signed xz-plane area of the triangle.
// @return 三角形の符号付きxz平面領域。
inline constexpr float dtTriArea2D(const float* a, const float* b, const float* c)
{
	const float abx = b[0] - a[0];
	const float abz = b[2] - a[2];
	const float acx = c[0] - a[0];
	const float acz = c[2] - a[2];
	return acx * abz - abx * acz;
}

// Derives the signed xz-plane area of the triangle ABC, or the relationship of line AB to point C.
// 三角形ABCの符号付きxz平面領域、またはラインABとポイントCの関係を導出します。
//  @param[in]		a		Vertex A. [(x, y, z)]
//  @param[in]		b		Vertex B. [(x, y, z)]
//  @param[in]		c		Vertex C. [(x, y, z)]
// @return The signed xz-plane area of the triangle.
// @return 三角形の符号付きxz平面領域。
inline constexpr float dtTriArea2D(
	const std::array<float, 3>& a, const std::array<float, 3>& b, const std::array<float, 3>& c)
{
	const float abx = b[0] - a[0];
	const float abz = b[2] - a[2];
	const float acx = c[0] - a[0];
	const float acz = c[2] - a[2];
	return acx * abz - abx * acz;
}

// Determines if two axis-aligned bounding boxes overlap.
// 2つの軸に沿った境界ボックスが重なるかどうかを決定します。
//  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
//  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
//  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
//  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
// @return True if the two AABB's overlap.
//	2つのAABBが重複している場合はTrue。
// @see dtOverlapBounds
inline constexpr bool dtOverlapQuantBounds(const uint16_t amin[3], const uint16_t amax[3],
	const uint16_t bmin[3], const uint16_t bmax[3])
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

// Determines if two axis-aligned bounding boxes overlap.
// 2つの軸に沿った境界ボックスが重なるかどうかを決定します。
//  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
//  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
//  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
//  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
// @return True if the two AABB's overlap.
//	2つのAABBが重複している場合はTrue。
// @see dtOverlapBounds
inline constexpr bool dtOverlapQuantBounds(
	const std::array<uint16_t, 3>& amin, const std::array<uint16_t, 3>& amax,
	const std::array<uint16_t, 3>& bmin, const std::array<uint16_t, 3>& bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

// Determines if two axis-aligned bounding boxes overlap.
// 2つの軸に沿った境界ボックスが重なるかどうかを決定します。
//  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
//  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
//  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
//  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
// @return True if the two AABB's overlap.
//	2つのAABBが重複している場合はTrue。
// @see dtOverlapQuantBounds
inline constexpr bool dtOverlapBounds(const float* amin, const float* amax,
	const float* bmin, const float* bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

// Determines if two axis-aligned bounding boxes overlap.
// 2つの軸に沿った境界ボックスが重なるかどうかを決定します。
//  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
//  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
//  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
//  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
// @return True if the two AABB's overlap.
//	2つのAABBが重複している場合はTrue。
// @see dtOverlapQuantBounds
inline constexpr bool dtOverlapBounds(const std::array<float, 3>& amin, const std::array<float, 3>& amax,
	const std::array<float, 3>& bmin, const std::array<float, 3>& bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

// Derives the closest point on a triangle from the specified reference point.
//  @param[out]	closest	The closest point on the triangle.
//  @param[in]		p		The reference point from which to test. [(x, y, z)]
//  @param[in]		a		Vertex A of triangle ABC. [(x, y, z)]
//  @param[in]		b		Vertex B of triangle ABC. [(x, y, z)]
//  @param[in]		c		Vertex C of triangle ABC. [(x, y, z)]
void dtClosestPtPointTriangle(float* closest, const float* p,
	const float* a, const float* b, const float* c);

// Derives the y-axis height of the closest point on the triangle from the specified reference point.
//  @param[in]		p		The reference point from which to test. [(x, y, z)]
//  @param[in]		a		Vertex A of triangle ABC. [(x, y, z)]
//  @param[in]		b		Vertex B of triangle ABC. [(x, y, z)]
//  @param[in]		c		Vertex C of triangle ABC. [(x, y, z)]
//  @param[out]	h		The resulting height.
bool dtClosestHeightPointTriangle(const float* p, const float* a, const float* b, const float* c, float& h);

bool dtIntersectSegmentPoly2D(const float* p0, const float* p1,
	const float* verts, int nverts,
	float& tmin, float& tmax,
	int& segMin, int& segMax);

bool dtIntersectSegSeg2D(const float* ap, const std::array<float, 3>& aq,
	const std::array<float, 3>& bp, const std::array<float, 3>& bq,
	float& s, float& t);

// Determines if the specified point is inside the convex polygon on the xz-plane.
//  @param[in]		pt		The point to check. [(x, y, z)]
//  @param[in]		verts	The polygon vertices. [(x, y, z) * @p nverts]
//  @param[in]		nverts	The number of vertices. [Limit: >= 3]
// @return True if the point is inside the polygon.
bool dtPointInPolygon(const float* pt, const float* verts, const int nverts);

bool dtDistancePtPolyEdgesSqr(const float* pt, const float* verts, const int nverts,
	float* ed, float* et);

float dtDistancePtSegSqr2D(const float* pt, const float* p, const float* q, float& t);

// Derives the centroid of a convex polygon.
//  @param[out]	tc		The centroid of the polgyon. [(x, y, z)]
//  @param[in]		idx		The polygon indices. [(vertIndex) * @p nidx]
//  @param[in]		nidx	The number of indices in the polygon. [Limit: >= 3]
//  @param[in]		verts	The polygon vertices. [(x, y, z) * vertCount]
void dtCalcPolyCenter(float* tc, const uint16_t* idx, int nidx, const float* verts);

// Determines if the two convex polygons overlap on the xz-plane.
//  @param[in]		polya		Polygon A vertices.	[(x, y, z) * @p npolya]
//  @param[in]		npolya		The number of vertices in polygon A.
//  @param[in]		polyb		Polygon B vertices.	[(x, y, z) * @p npolyb]
//  @param[in]		npolyb		The number of vertices in polygon B.
// @return True if the two polygons overlap.
bool dtOverlapPolyPoly2D(const float* polya, const int npolya,
	const float* polyb, const int npolyb);

// @}
// @name Miscellanious functions. // その他の機能。
// @{
inline constexpr uint32_t dtNextPow2(uint32_t v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline uint32_t dtIlog2(uint32_t v)
{
	uint32_t r;
	uint32_t shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}

inline constexpr int dtAlign4(int x) { return (x + 3) & ~3; }

inline constexpr int dtOppositeTile(int side) { return (side + 4) & 0x7; }

inline constexpr void dtSwapByte(uint8_t* a, uint8_t* b)
{
	uint8_t tmp = *a;
	*a = *b;
	*b = tmp;
}

inline constexpr void dtSwapEndian(uint16_t* v)
{
	uint8_t* x = (uint8_t*)v;
	dtSwapByte(x + 0, x + 1);
}

inline constexpr void dtSwapEndian(short* v)
{
	uint8_t* x = (uint8_t*)v;
	dtSwapByte(x + 0, x + 1);
}

inline constexpr void dtSwapEndian(uint32_t* v)
{
	uint8_t* x = (uint8_t*)v;
	dtSwapByte(x + 0, x + 3);
	dtSwapByte(x + 1, x + 2);
}

inline void dtSwapEndian(int* v)
{
	uint8_t* x = (uint8_t*)v;
	dtSwapByte(x + 0, x + 3); dtSwapByte(x + 1, x + 2);
}

inline constexpr void dtSwapEndian(float* v)
{
	uint8_t* x = (uint8_t*)v;
	dtSwapByte(x + 0, x + 3);
	dtSwapByte(x + 1, x + 2);
}

// Returns a random point in a convex polygon.
// 凸多角形のランダムポイントを返します。
// Adapted from Graphics Gems article.
// Graphics Gemsの記事から改編。
void dtRandomPointInConvexPoly(const float* pts, const int npts, float* areas,
	const float s, const float t, float* out);

// アドバンスバッファーポインターを取得する
template<typename TypeToRetrieveAs>
TypeToRetrieveAs* dtGetThenAdvanceBufferPointer(const uint8_t*& buffer, const size_t distanceToAdvance)
{
	TypeToRetrieveAs* returnPointer = reinterpret_cast<TypeToRetrieveAs*>(buffer);
	buffer += distanceToAdvance;
	return returnPointer;
}

// アドバンスバッファーポインターを取得する
template<typename TypeToRetrieveAs>
TypeToRetrieveAs* dtGetThenAdvanceBufferPointer(uint8_t*& buffer, const size_t distanceToAdvance)
{
	TypeToRetrieveAs* returnPointer = reinterpret_cast<TypeToRetrieveAs*>(buffer);
	buffer += distanceToAdvance;
	return returnPointer;
}

// @}

#endif // DETOURCOMMON_H

//////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

/**

@fn float dtTriArea2D(const float* a, const float* b, const float* c)
@par

The vertices are projected onto the xz-plane, so the y-values are ignored.

This is a low cost function than can be used for various purposes.  Its main purpose
is for point/line relationship testing.

In all cases: A value of zero indicates that all vertices are collinear or represent the same point.
(On the xz-plane.)

When used for point/line relationship tests, AB usually represents a line against which
the C point is to be tested.  In this case:

A positive value indicates that point C is to the left of line AB, looking from A toward B.<br/>
A negative value indicates that point C is to the right of lineAB, looking from A toward B.

When used for evaluating a triangle:

The absolute value of the return value is two times the area of the triangle when it is
projected onto the xz-plane.

A positive return value indicates:

<ul>
<li>The vertices are wrapped in the normal Detour wrap direction.</li>
<li>The triangle's 3D face normal is in the general up direction.</li>
</ul>

A negative return value indicates:

<ul>
<li>The vertices are reverse wrapped. (Wrapped opposite the normal Detour wrap direction.)</li>
<li>The triangle's 3D face normal is in the general down direction.</li>
</ul>

*/

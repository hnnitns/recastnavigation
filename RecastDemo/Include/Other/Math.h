//==================================================
//
//!+++ Math.h
//
//==================================================
#pragma once

// プロパティーページ\C++/C\コマンドラインに「/Zc:__cplusplus」を指定する必要がある
#if (__cplusplus < 201703L)
#error "コマンドラインに「/Zc:__cplusplus」を定義していない、もしくは「-std=c++17」をコマンドオプションに指定してください。"
#endif

//---------------------------------------------------------
//!+++  インクルード
//---------------------------------------------------------
#include	<cmath>
#include	<cassert>
#include	<type_traits>
#include	<algorithm>
#include	<limits>
#include	<DirectXMath.h>

#define HAS_VECTOR3 (__has_include("Vector3.h"))
#define HAS_VECTOR2 (__has_include("Vector2.h"))

#if HAS_VECTOR3
#include "Vector3.h"
#endif

#if HAS_VECTOR2
#include "Vector2.h"
#endif

//---------------------------------------------------------
//!+++  関数
//---------------------------------------------------------

//!+++ 数学関係
namespace Math
{
	template <class _Ty = double>
	static constexpr _Ty _180{ static_cast<_Ty>(180.0) };  // 関数内で使用
	template <class _Ty = double>
	static constexpr _Ty _2{ static_cast<_Ty>(2.0) };  // 関数内で使用
	template <class _Ty = double>
	static constexpr _Ty _05{ static_cast<_Ty>(0.5) };  // 関数内で使用
	template <class _Ty = double>
	static constexpr _Ty PAI{ static_cast<_Ty>(3.141592653589793) }; // 円周率
	template <class _Ty = double>
	static constexpr _Ty _0{ static_cast<_Ty>(0.0) }; // 関数内で使用

	//!++ 偶数・奇数判断
	// 引数↓
	// int factor : 判断したい値
	// 戻り値↓
	// 引数の型：１は奇数、０は偶数
	template<class _Ty>
	_NODISCARD static inline constexpr int OddEvenJudge(const _Ty number)
	{
		static_assert(std::is_integral<_Ty>(), "This type isn't integral");

		return (number & 1);
	}

	//!++ 線形補間
	template<typename _Ty>
	_NODISCARD static inline constexpr _Ty Leap(const _Ty num1, const _Ty num2, const _Ty time)
	{
		return num1 + time * (num2 - num1);
	}

	//!++ 値が誤差を含んだ上で同じかどうか
	// 引数↓
	// 小数 epsilon_num : 誤差を含む数値
	// 小数 num : epsilon_numと比べたい数値
	// 戻り値↓
	// 引数の型：範囲内の数値
	template<typename _Ty>
	_NODISCARD static inline constexpr bool AdjEqual(const _Ty epsilon_num, const _Ty num)
	{
		static_assert(std::is_floating_point<_Ty>(), "This type isn't floating point");

		constexpr auto Fabs{ [](const _Ty num) constexpr {
			if (num > _0<_Ty>) return num; else return -num;
		} };

		static constexpr auto Epsilon{ std::numeric_limits<_Ty>::epsilon() };
		auto dis{ Fabs(epsilon_num - num) };

		return (dis <= Epsilon);
	}

	//!++ 値が誤差を含んだ上で同じかどうか
	// 引数↓
	// 小数 epsilon_num : 誤差を含む数値
	// 小数 num : epsilon_numと比べたい数値
	// 小数 pow_num : 実際の誤差の何倍まで許容するか(1.19209e-07)
	// 戻り値↓
	// 引数の型：範囲内の数値
	template<typename _Ty>
	_NODISCARD static inline constexpr bool NearyEqual(const _Ty epsilon_num, const _Ty num, const _Ty pow_num)
	{
		static_assert(std::is_floating_point<_Ty>(), "This type isn't floating point");

		constexpr auto Fabs{ [](const _Ty num) constexpr {
			if (num > _0<_Ty>) return num; else return -num;
		} };

		const auto Epsilon{ std::numeric_limits<_Ty>::epsilon() * pow_num };
		auto dis{ Fabs(epsilon_num - num) };

		return (dis <= Epsilon);
	}

	//!++ A数値とB数値の間に検証したい数値があるかどうか
	// 引数↓
	// 小数 first_element : A数値
	// 小数 second_element : B数値
	// 小数 verification_elemet : 検証したい数値
	// 戻り値↓
	// 引数の型：AとBの間に検証数値が存在するかどうか
	template<bool IsIncludeEqual = true, typename _Ty = float>
	_NODISCARD static inline constexpr bool IsBetweenNumber(const _Ty first_element, const _Ty second_element,
		const _Ty verification_elemet)
	{
		if constexpr (IsIncludeEqual)
		{
			if constexpr (std::is_floating_point<_Ty>())
			{
				// 検証値と同じかどうか調べる
				if (AdjEqual(verification_elemet, first_element) || AdjEqual(verification_elemet, second_element))
					return true;
			}
			else
			{
				if (verification_elemet == first_element || verification_elemet == second_element)
					return true;
			}
		}

		const _Ty
			min_num{ (std::min)(first_element, second_element) },
			max_num{ (std::max)(first_element, second_element) };

		return (verification_elemet > min_num&& verification_elemet < max_num);
	}

	//!++ 角度の正規化（ラジアン）
	// 引数↓
	// 小数 radian : 正規化した角度（ラジアン）
	// 戻り値↓
	// 引数の型 ： -PI ～ +PI
	template<typename _Ty>
	_NODISCARD static inline _Ty RadiansNormalize(const _Ty radian)
	{
		static_assert(std::is_floating_point<_Ty>::value, "This type isn't floating point.");

		return (std::atan2(std::sin(radian), std::cos(radian)));
	}

	//!++ 角度の正規化（デグリー）
	// 引数↓
	// 小数 degrees : 正規化した角度（デグリー）
	// 戻り値↓
	// 引数の型：
	template<typename _Ty>
	_NODISCARD static inline _Ty DegreesNormalize(const _Ty degrees)
	{
		static_assert(std::is_floating_point<_Ty>::value, "This type isn't floating point.");

		return (ToDegree(RadiansNormalize(ToRadian(degrees))));
	}

	//!++ 二乗する
	// 引数↓
	// factor : 二乗をする値
	// 戻り値↓
	// 引数の型：二乗値
	template<class _Ty>
	_NODISCARD static inline constexpr _Ty Pow2(const _Ty factor)
	{
		return (factor * factor);
	}

	//!++ 角度からラジアンへ
	// 引数↓
	// float, double angle  : 角度
	// 戻り値↓
	// float, double：ラジアン
	template<class _Ty>
	_NODISCARD static inline constexpr _Ty ToRadian(const _Ty angle)
	{
		// 小数でなければならない
		static_assert(std::is_floating_point<_Ty>::value == true,
			"Have to be deciaml.");

		return static_cast<_Ty>(angle * PAI<_Ty> / _180<_Ty>);
	}

	//!++ ラジアンから角度へ
	// 引数↓
	// float radian : ラジアン
	// 戻り値↓
	// float：角度
	template<class _Ty>
	_NODISCARD static inline constexpr _Ty ToDegree(const _Ty radian)
	{
		// 小数でなければならない
		static_assert(std::is_floating_point<_Ty>::value == true,
			"Have to be deciaml.");

		return static_cast<_Ty>(radian * _180<_Ty> / PAI<_Ty>);
	}

	//!++ 高さと底辺から角度を求める
	// 引数↓
	// width : 底辺
	// height : 対辺
	// 戻り値↓
	// float：ラジアン
	template<class _Ty>
	_NODISCARD static inline _Ty Atan2(const _Ty width, const _Ty hight)
	{
		// 小数でなければならない
		static_assert(std::is_floating_point<_Ty>::value == true,
			"Have to be deciaml.");

		return static_cast<_Ty>(std::atan2<_Ty, _Ty>(hight, width));
	}

#if HAS_VECTOR2
	//!++ 基準座標から他の座標への角度を求める
	// 引数↓
	// Vector2 base_pos : 求めたい角度元である座標
	// Vector2 another_pos : 求めたい角度先である座標
	// 戻り値↓
	// float：ラジアン
	template<class _Ty>
	_NODISCARD static inline _Ty PosToRadian(const Vector2<_Ty>& base_pos,
		const Vector2<_Ty>& another_pos)
	{
		// 小数でなければならない
		static_assert(std::is_floating_point<_Ty>::value == true,
			"Have to be deciaml.");

		return Atan2<_Ty>(another_pos.x - base_pos.x, another_pos.y - base_pos.y);
	}
#endif

	//!++ 基準座標から他の座標への角度を求める
	// 引数↓
	// DirectX::XMFLOAT2 base_pos : 求めたい角度元である座標
	// DirectX::XMFLOAT2 another_pos : 求めたい角度先である座標
	// 戻り値↓
	// float：ラジアン
	_NODISCARD static inline float PosToRadian(const DirectX::XMFLOAT2& base_pos,
		const DirectX::XMFLOAT2& another_pos)
	{
		return Atan2<float>(another_pos.x - base_pos.x, another_pos.y - base_pos.y);
	}

	//!++ サインカーブを求める
	// 引数↓
	// int amolitude : 振幅（～のカウントで(1秒60カウント)）
	// float period : 周期（±～の範囲を遷移する）
	// int count : カウント
	// int base : 基準点（0から～だけ基準点を変える）(初期は0)
	// 戻り値↓
	// float：数値
	template<class _Ty>
	_NODISCARD static inline _Ty SinCurve(const int amolitude, const int period, const int count, const int base = 0)
	{
		// 小数でなければならない
		static_assert(std::is_floating_point<_Ty>::value == true,
			"Have to be deciaml.");

		return static_cast<_Ty>(base + period + std::sin(PAI<_Ty> * _2<_Ty> / period * count) * amolitude);
	}

#if HAS_VECTOR2
	//!++ 二点間の距離を求める（二次元）
	// 引数↓
	// Vector2 : 第１座標
	// Vector2 : 第２座標
	// 戻り値↓
	// 引数の型：距離
	template<class _Ty>
	_NODISCARD static inline _Ty Distance(const Vector2<_Ty>& pos1, const Vector2<_Ty>& pos2)
	{
		// 小数でなければならない
		static_assert(std::is_floating_point<_Ty>::value == true,
			"Have to be deciaml.");

		return static_cast<_Ty>(std::pow<_Ty, _Ty>(
			Pow2<_Ty>(pos2.x - pos1.x) + Pow2<_Ty>(pos2.y - pos1.y), _05<_Ty>
			));
	}
#endif

	//!++ 二点間の距離を求める（二次元）
	// 引数↓
	// DirectX::XMFLOAT2 : 第１座標
	// DirectX::XMFLOAT2 : 第２座標
	// 戻り値↓
	// 引数の型：距離
	_NODISCARD static inline float Distance(const DirectX::XMFLOAT2& pos1, const DirectX::XMFLOAT2& pos2)
	{
		return std::pow(Pow2<float>(pos2.x - pos1.x) + Pow2<float>(pos2.y - pos1.y), _05<float>);
	}

#if HAS_VECTOR3
	//!++ 二点間の距離を求める（三次元）
	// 引数↓
	// Vector3 : 第１座標
	// Vector3 : 第２座標
	// 戻り値↓
	// 引数の型：距離
	template<class _Ty>
	_NODISCARD static inline _Ty Distance(const Vector3<_Ty>& pos1, const Vector3<_Ty>& pos2)
	{
		// 小数でなければならない
		static_assert(std::is_floating_point<_Ty>::value == true,
			"Have to be deciaml.");

		return static_cast<_Ty>(std::pow<_Ty, _Ty>(
			Pow2<_Ty>(pos2.x - pos1.x) + Pow2<_Ty>(pos2.y - pos1.y) + Pow2<_Ty>(pos2.z - pos1.z), _05<_Ty>
			));
}
#endif

	//!++ 二点間の距離を求める（三次元）
	// 引数↓
	// DirectX::XMFLOAT3 : 第１座標
	// DirectX::XMFLOAT3 : 第２座標
	// 戻り値↓
	// 引数の型：距離
	_NODISCARD static inline float Distance(const DirectX::XMFLOAT3& pos1, const DirectX::XMFLOAT3& pos2)
	{
		return (std::pow(Pow2<float>(pos2.x - pos1.x) + Pow2<float>(pos2.y - pos1.y) + Pow2<float>(pos2.z - pos1.z), _05<float>));
	}

	//!++ 二点間の距離を求める（二乗されたまま）（三次元）
	// 引数↓
	// Vector2 : 第１座標
	// Vector2 : 第２座標
	// 戻り値↓
	// 引数の型：距離の二乗
	_NODISCARD static inline float DistanceSq(const DirectX::XMFLOAT3& pos1, const DirectX::XMFLOAT3& pos2)
	{
		return (Pow2<float>(pos2.x - pos1.x) + Pow2<float>(pos2.y - pos1.y) + Pow2<float>(pos2.z - pos1.z));
	}

#if HAS_VECTOR2
	//!++ 二点間の距離を求める（簡易版）
	// 正確な距離は求まらない。距離を比較したい時など大体の距離でOKな時など（簡易的な分、Distance関数と比べるととてもかるい）
	// 引数↓
	// Vector2 : 第１座標
	// Vector2 : 第２座標
	// 戻り値↓
	// 引数の型：距離（不正確：ただし、比べる程度なら使える）
	template<class _Ty>
	_NODISCARD static inline _Ty EasyDistance(const Vector2<_Ty>& pos1, const Vector2<_Ty>& pos2)
	{
		const auto dis_vec2{ (pos1 - pos2).Fabsf() };

		return (dis_vec2.x + dis_vec2.y);
	}
#endif

	//!++ 二点間の距離を求める（簡易版）
	// 正確な距離は求まらない。距離を比較したい時など大体の距離でOKな時など（簡易的な分、Distance関数と比べるととてもかるい）
	// 引数↓
	// Vector2 : 第１座標
	// Vector2 : 第２座標
	// 戻り値↓
	// 引数の型：距離（不正確：ただし、比べる程度なら使える）
	_NODISCARD static inline float EasyDistance(const DirectX::XMFLOAT3& pos1, const DirectX::XMFLOAT3& pos2)
	{
		auto FAbs{ [](const auto& vf3, const auto& vf3b) {
			DirectX::XMFLOAT3 temp;

			temp.x = fabs(vf3.x - vf3b.x);
			temp.y = fabs(vf3.y - vf3b.y);
			temp.z = fabs(vf3.z - vf3b.z);

			return temp;
		} };

		const auto dis_vec3{ FAbs(pos2, pos1) };

		return (dis_vec3.x + dis_vec3.y + dis_vec3.z);
	}

#if HAS_VECTOR2
	//!++ 二点間の中点を求める
	// 引数↓
	// Vector2 : 第１座標
	// Vector2 : 第２座標
	// 戻り値↓
	// 引数の型：二点間の中点
	template<class _Ty>
	_NODISCARD static inline constexpr decltype(auto) MiddlePoint(const Vector2<_Ty>& pos1, const Vector2<_Ty>& pos2)
	{
		return ((pos1 + pos2) / _2<_Ty>);
	}
#endif

	//!++ 二点間の中点を求める
	// 引数↓
	// DirectX::XMFLOAT2 : 第１座標
	// DirectX::XMFLOAT2 : 第２座標
	// 戻り値↓
	// 引数の型：二点間の中点
	_NODISCARD static inline DirectX::XMFLOAT2 MiddlePoint(const DirectX::XMFLOAT2& pos1, const DirectX::XMFLOAT2& pos2)
	{
		return DirectX::XMFLOAT2{ ((pos1.x + pos2.x) / 2.f), ((pos1.y + pos2.y) / 2.f) };
	}

	//!++ 三点間の中点を求める
	// 引数↓
	// DirectX::XMFLOAT2 : 第１座標
	// DirectX::XMFLOAT2 : 第２座標
	// 戻り値↓
	// 引数の型：二点間の中点
	_NODISCARD static inline DirectX::XMFLOAT3 MiddlePoint(const DirectX::XMFLOAT3& pos1, const DirectX::XMFLOAT3& pos2)
	{
		return 	DirectX::XMFLOAT3{ ((pos1.x + pos2.x) / 2.f), ((pos1.y + pos2.y) / 2.f), ((pos1.x + pos2.x) / 2.f) };
	}

	//!++ Xの速度
	// 引数↓
	// float speed : 速度
	// float ラジアン : 角度
	// 戻り値↓
	// 引数の型：速度X
	template<class _Ty>
	_NODISCARD static _Ty Xspeed(const _Ty speed, const _Ty rad)
	{
		// 小数でなければならない
		static_assert(std::is_floating_point<_Ty>::value == true,
			"Have to be deciaml.");

		return static_cast<_Ty>(speed * std::cos(rad));
	}

	//!++ Yの速度
	// 引数↓
	// float speed : 速度
	// float ラジアン : 角度
	// 戻り値↓
	// 引数の型：速度Y
	template<class _Ty>
	_NODISCARD static _Ty Yspeed(const _Ty speed, const _Ty rad)
	{
		// 小数でなければならない
		static_assert(std::is_floating_point<_Ty>::value == true,
			"Have to be deciaml.");

		return static_cast<_Ty>(speed * std::sin(rad));
	}
		};
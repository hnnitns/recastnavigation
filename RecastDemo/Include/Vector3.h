//==================================================
//
//!+++ Vector3.h
//
//==================================================
#pragma once

// プロパティーページ\C++/C\コマンドラインに「/Zc:__cplusplus」を指定する必要がある
#if (__cplusplus < 201703L)
#error "コマンドラインに「/Zc:__cplusplus」を定義していない、もしくは「-std=c++17」をコマンドオプションに指定してください。"
#endif

#include <stdexcept>
#include <cassert>
#include <cmath>
#include <type_traits>
#include <cstring>
#include <limits>
#include <initializer_list>
#include <DirectXMath.h>

//!++ Vector3 (普通版)
#if 1
template<class T>
class Vector3 final
{
private:
	template<typename _Ty = T>
	static constexpr _Ty _0{ static_cast<_Ty>(0) };

public:  //+ コンストラクタ系

	// デフォルトコンストラクタ
	template<class _Ty = T>
	explicit constexpr Vector3() noexcept
		: x(std::forward<_Ty>(static_cast<_Ty>(0))), y(std::forward<_Ty>(static_cast<_Ty>(0))), z(std::forward<_Ty>(static_cast<_Ty>(0)))
	{};

	// コンストラクタ（別々指定、右辺値参照）
	template<class _Ty = T>
	explicit constexpr Vector3(_Ty&& x, _Ty&& y, _Ty&& z) noexcept
			: x(x), y(y), z(z) { };

	// コンストラクタ（別々指定）
	template<class _Ty = T>
	explicit constexpr Vector3(const _Ty& x, const _Ty& y, const _Ty& z) noexcept
			: x(x), y(y), z(z) { };

	// イニシャライザーリスト代入演算子
	template<class _Ty = T>
	constexpr Vector3(const std::initializer_list<T> & list)
	{
		if (list.size() != 3)
			throw std::invalid_argument("不正なイニシャライザーリスト");

		auto itr{ list.begin() };

		x = *itr;
		y = *(++itr);
		z = *(++itr);
	}

	//. XMFLOAT3用コピーコンストラクタ
	constexpr Vector3(const DirectX::XMFLOAT3& _Right) noexcept
		: x(_Right.x), y(_Right.y), z(_Right.z) { }

	//. XMFLOAT3用ムーブコンストラクタ
	constexpr Vector3(DirectX::XMFLOAT3&& _Right) noexcept
		: x(std::forward<float>(_Right.x)), y(std::forward<float>(_Right.y)), z(std::forward<float>(_Right.z)) { }

	// コピーコンストラクタ
	template<class _Ty = T>
	explicit constexpr Vector3(const Vector3<_Ty>& v) noexcept
			: x(v.x), y(v.y), z(v.z) { }

	// ムーブコンストラクタ
	template<class _Ty = T>
	explicit constexpr Vector3(Vector3<_Ty>&& _Right) noexcept
		: x(std::forward<_Ty>(_Right.x)), y(std::forward<_Ty>(_Right.y)), z(std::forward<_Ty>(_Right.z)) { }

	//. XMFLOAT3用コピー代入演算子
	decltype(auto) operator= (const DirectX::XMFLOAT3& xmfloat3)
	{
		static_assert(std::is_floating_point<T>::value, "This type isn't Float.");

		x = xmfloat3.x;
		y = xmfloat3.y;
		z = xmfloat3.z;
	}

	// ムーブ代入演算子
	template<class _Ty = T>
	decltype(auto) operator= (Vector3<_Ty> &&_Right) noexcept
	{
		using std::forward;

		if(this != &_Right)
		{
			Clear();

			x = forward<_Ty>(_Right.x);
			y = forward<_Ty>(_Right.y);
			z = forward<_Ty>(_Right.z);
		}

		return (*this);
	}

	//. XMFLOAT3用ムーブ代入演算子
	decltype(auto) operator= (DirectX::XMFLOAT3&& xmfloat3) noexcept
	{
		static_assert(std::is_floating_point<T>::value, "This type isn't Float.");

		x = std::forward<float>(xmfloat3.x);
		y = std::forward<float>(xmfloat3.y);
		z = std::forward<float>(xmfloat3.z);
	}

	// イニシャライザーリスト代入演算子
	template<class _Ty = T>
	decltype(auto) operator= (const std::initializer_list<T>& list)
	{
		if (list.size() != 3)
			throw std::invalid_argument("不正なイニシャライザーリスト");

		auto itr{ list.begin() };

		x = *itr;
		y = *(++itr);
		z = *(++itr);
	}

	// コピー代入演算子
	template<class _Ty = T>
	decltype(auto) operator=(const Vector3<_Ty>& v) noexcept
	{
		x = v.x;
		y = v.y;
		z = v.z;

		return (*this);
	}

	// 代入演算子（まとめて指定）
	template<class _Ty = T>
	decltype(auto) operator=(const _Ty& _Right) noexcept
	{
		x = (_Right);
		y = (_Right);
		z = (_Right);

		return (*this);
	}

	// デストラクタ
	~Vector3() noexcept = default;

public:
	static constexpr auto NumMaxLimit { (std::numeric_limits<std::remove_reference<T>>::max)() };
	static constexpr auto NumMinLimit { (std::numeric_limits<std::remove_reference<T>>::lowest)() };

private:  //+ エイリアステンプレート

	template<class _Ty = T>
	using V3 = Vector3<_Ty>;

public:  //+ 演算子オーバーロード

	// Vector2同士

	constexpr decltype(auto) operator+ (const V3<> &v) const noexcept;
	constexpr decltype(auto) operator- (const V3<> &v) const noexcept;
	constexpr decltype(auto) operator* (const V3<> &v) const noexcept;
	constexpr decltype(auto) operator/ (const V3<> &v) const;
	constexpr decltype(auto) operator% (const V3<> &v) const;

	decltype(auto) operator+= (const V3<> &v) noexcept;
	decltype(auto) operator-= (const V3<> &v) noexcept;
	decltype(auto) operator*= (const V3<> &v) noexcept;
	decltype(auto) operator/= (const V3<> &v);
	decltype(auto) operator%= (const V3<> &v);

	constexpr bool operator== (const V3<> &v) const noexcept;
	constexpr bool operator!= (const V3<> &v) const noexcept;

	constexpr bool operator< (const V3<> &v) const noexcept;
	constexpr bool operator> (const V3<> &v) const noexcept;
	constexpr bool operator<= (const V3<> &v) const noexcept;
	constexpr bool operator>= (const V3<> &v) const noexcept;

	auto operator++(int);
	auto operator--(int);
	decltype(auto) operator++();
	decltype(auto) operator--();

	decltype(auto) operator-() const;
	decltype(auto) operator+() const;

	// Vector3と数値

	constexpr decltype(auto) operator+ (const T n) const noexcept;
	constexpr decltype(auto) operator- (const T n) const noexcept;
	constexpr decltype(auto) operator* (const T n) const noexcept;
	constexpr decltype(auto) operator/ (const T n) const;
	constexpr decltype(auto) operator% (const T n) const;

	decltype(auto) operator+= (const T n) noexcept;
	decltype(auto) operator-= (const T n) noexcept;
	decltype(auto) operator*= (const T n) noexcept;
	decltype(auto) operator/= (const T n);
	decltype(auto) operator%= (const T n);

	constexpr bool operator== (const T n) const noexcept;
	constexpr bool operator!= (const T n) const noexcept;

	constexpr bool operator> (const T n) const noexcept;
	constexpr bool operator< (const T n) const noexcept;
	constexpr bool operator>= (const T n) const noexcept;
	constexpr bool operator<= (const T n) const noexcept;

public: //+ 関数

	template<class _Ty = T>
	_NODISCARD constexpr decltype(auto) Fill(const _Ty num)
	{
		x = y = z = num;

		return (*this);
	}
	template<class _Ty>
	_NODISCARD constexpr decltype(auto) ToChangeType() const;  // 型変換関数
	void Swap(V3<>& _Right);  // 中身交換関数
	void Clear();  // 中身の初期化
	_NODISCARD decltype(auto) Fabsf();  // 絶対値にする
	_NODISCARD constexpr bool isEmpty() const;  // 中身が０かどうか判断する
	_NODISCARD constexpr auto ToXMFLOAT3() const
	{
		static_assert(std::is_floating_point<T>::value, "This type isn't Float.");

		return (DirectX::XMFLOAT3{ x, y, z });
	}

private:
	template<class _Ty>  // std::swapを呼び出す
	void _Swap_adl(_Ty& _Left, _Ty& _Right);
	_NODISCARD constexpr bool isInclude0(const V3<>& _Right) const;  // 要素が一つでも０かどうか判断する
	template<typename _Ty>
	_NODISCARD constexpr bool AdjEqual(const _Ty epsilon_num, const _Ty num) const
	{
		static_assert(std::is_floating_point<_Ty>(), "This type isn't floating point");

		constexpr auto Fabs{ [] (const _Ty num) constexpr {
			if (num > _0<_Ty>) return num; else return -num;
		} };

		static constexpr auto Epsilon{ std::numeric_limits<_Ty>::epsilon() };
		auto dis{ Fabs(epsilon_num - num) };

		return (dis <= Epsilon);
	}

public:  //++ メンバ変数

	T x, y, z;
};

//+ 関数
#if 1
// 型変換関数
template<class T> template<class _Ty>
inline constexpr decltype(auto) Vector3<T>::ToChangeType() const
{
	using std::forward;

	// 違う型でなければならない
	static_assert(!std::is_same<T, _Ty>::value, "This is same type. This have to be different type.");

	return (V3<_Ty> { forward<_Ty>(static_cast<_Ty>(x)), forward<_Ty>(static_cast<_Ty>(y)), forward<_Ty>(static_cast<_Ty>(z)) });
}

// std::swapを呼び出す
template<class T> template<class _Ty>
inline void Vector3<T>::_Swap_adl(_Ty & _Left, _Ty & _Right)
{
	std::swap<_Ty, _Ty>(_Left, _Right);
}

// 別のVector2と中身を交換（同じ型でする事）
template<class T>
inline void Vector3<T>::Swap(V3<>& _Right)
{
	if (this != &_Right)
	{
		_Swap_adl(x, _Right.x);
		_Swap_adl(y, _Right.y);
		_Swap_adl(z, _Right.z);
	}
}

// 中身の初期化
template<class T>
inline void Vector3<T>::Clear()
{
	using std::forward;

	x = _0<>;
	y = _0<>;
	z = _0<>;
}

// 絶対値にする
template<class T>
inline decltype(auto) Vector3<T>::Fabsf()
{
	using std::forward;

	// 符号ありでなければならない
	static_assert(std::is_signed<T>::value, "This type is unsigned. This have to be signed.");

	x = forward<T>(static_cast<T>(fabs(x)));
	y = forward<T>(static_cast<T>(fabs(y)));
	z = forward<T>(static_cast<T>(fabs(z)));

	return (*this);
}

// 中身が０かどうか判断する
template<class T>
inline constexpr bool Vector3<T>::isEmpty() const
{
	if constexpr (std::is_floating_point<T>())
	{
		return AdjEqual(x, _0<>) && AdjEqual(y, _0<>) && AdjEqual(z, _0<>);
	}
	else
	{
		return (x == _0<> && y == _0<> && z == _0<>);
	}
}

// 要素が一つでも０かどうか判断する
template<class T>
inline constexpr bool Vector3<T>::isInclude0(const V3<>& Right) const
{
	if constexpr (std::is_floating_point<T>())
	{
		return AdjEqual(Right.x * Right.y * Right.z, _0<>);
	}
	else
	{
		return (Right.x * Right.y * Right.z == _0<>);
	}
}

#endif

//+ 演算子オーバーロード（Vector2同士）
#if 1
template<class T>
inline constexpr decltype(auto) Vector3<T>::operator+(const V3<>& v) const noexcept
{
	using std::forward;

	return  V3<>{ forward<T>(x + v.x), forward<T>(y + v.y), forward<T>(z + v.z) };
}

template<class T>
inline constexpr decltype(auto) Vector3<T>::operator-(const V3<>& v) const noexcept
{
	using std::forward;

	return  V3<>{ forward<T>(x - v.x), forward<T>(y - v.y), forward<T>(z - v.z) };
}

template<class T>
inline constexpr decltype(auto) Vector3<T>::operator*(const V3<>& v) const noexcept
{
	using std::forward;

	return  V3<>{ forward<T>(x * v.x), forward<T>(y * v.y), forward<T>(z * v.z) };
}

template<class T>
inline constexpr decltype(auto) Vector3<T>::operator/(const V3<>& v) const
{
	assert(!isInclude0(v) && "0で割ろうとしている");

	using std::forward;

	return  V3<>{ forward<T>(x / v.x), forward<T>(y / v.y), forward<T>(z / v.z) };
}

template<class T>
inline constexpr decltype(auto) Vector3<T>::operator%(const V3<>& v) const
{
	assert(!isInclude0(v) && "0で割ろうとしている");

	using std::forward;

	if constexpr (std::is_floating_point<T>())
	{
		return  V3<>{ forward<T>(std::fmod<T>(x, v.x)), forward<T>(std::fmod<T>(y, v.y)), forward<T>(std::fmod<T>(z, v.z)) };
	}
	else
	{
		return  V3<>{ forward<T>(x% v.x), forward<T>(y% v.y), forward<T>(z% v.z) };
	}
}

template<class T>
inline decltype(auto) Vector3<T>::operator+=(const V3<>& v) noexcept
{
	x += (v.x);
	y += (v.y);
	z += (v.z);

	return (*this);
}

template<class T>
inline decltype(auto) Vector3<T>::operator-=(const V3<>& v) noexcept
{
	x -= (v.x);
	y -= (v.y);
	z -= (v.z);

	return (*this);
}

template<class T>
inline decltype(auto) Vector3<T>::operator*=(const V3<>& v) noexcept
{
	x *= (v.x);
	y *= (v.y);
	z *= (v.z);

	return (*this);
}

template<class T>
inline decltype(auto) Vector3<T>::operator/=(const V3<>& v)
{
	assert(!isInclude0(v) && "0で割ろうとしている");

	x /= (v.x);
	y /= (v.y);
	z /= (v.z);

	return (*this);
}

template<class T>
inline decltype(auto) Vector3<T>::operator%=(const V3<>& v)
{
	assert(!isInclude0(v) && "0で割ろうとしている");

	if constexpr (std::is_floating_point<T>())
	{
		x = std::fmod<T>(x, v.x);
		y = std::fmod<T>(y, v.y);
		z = std::fmod<T>(z, v.z);
	}
	else
	{
		x %= (v.x);
		y %= (v.y);
		z %= (v.z);
	}

	return (*this);
}

template<class T>
inline constexpr bool Vector3<T>::operator==(const V3<> & v) const noexcept
{
	if constexpr (std::is_floating_point<T>())
	{
		return (AdjEqual(x, v.x) && AdjEqual(y, v.y)&& AdjEqual(z, v.z));
	}
	else
	{
		return (x == v.x) && (y == v.y) && (z == v.z);
	}
}

template<class T>
inline constexpr bool Vector3<T>::operator!=(const V3<> & v) const noexcept
{
	if constexpr (std::is_floating_point<T>())
	{
		return !(AdjEqual(x, v.x) && AdjEqual(y, v.y) && AdjEqual(z, v.z));
	}
	else
	{
		return (x != v.x) || (y != v.y) || (z != v.z);
	}
}

template<class T>
inline constexpr bool Vector3<T>::operator<(const V3<>& v) const noexcept
{
	return (x < v.x) && (y < v.y);
}

template<class T>
inline constexpr bool Vector3<T>::operator>(const V3<>& v) const noexcept
{
	return (x > v.x) && (y > v.y);
}

template<class T>
inline constexpr bool Vector3<T>::operator<=(const V3<>& v) const noexcept
{
	return (x <= v.x) && (y <= v.y);
}

template<class T>
inline constexpr bool Vector3<T>::operator>=(const V3<>& v) const noexcept
{
	return (x >= v.x) && (y >= v.y);
}

template<class T>
inline auto Vector3<T>::operator++(int)
{
	// 整数型でなければならない
	static_assert(std::is_integral<T>::value, "This type isn't integral. This have to be integral");

	assert(x != NumMaxLimit || y != NumMaxLimit && "この型の最大値・最大値を超えている");

	Vector3<T> temp { *this };

	x++;
	y++;
	z++;

	return (temp);
}

template<class T>
inline auto Vector3<T>::operator--(int)
{
	// 整数型でなければならない
	static_assert(std::is_integral<T>::value, "This type isn't integral. This have to be integral");

	assert(x != NumMaxLimit || y != NumMaxLimit && "この型の最大値・最大値を超えている");

	Vector3<T> temp { *this };

	x--;
	y--;
	z--;

	return (temp);
}

template<class T>
inline decltype(auto) Vector3<T>::operator++()
{
	// 整数型でなければならない
	static_assert(std::is_integral<T>::value, "This type isn't integral. This have to be integral");

	assert(x != NumMaxLimit || y != NumMaxLimit && "この型の最大値・最大値を超えている");

	x++;
	y++;
	z++;

	return (*this);
}

template<class T>
inline decltype(auto) Vector3<T>::operator--()
{
	// 整数型でなければならない
	static_assert(std::is_integral<T>::value, "This type isn't integral. This have to be integral");

	assert(x != NumMaxLimit || y != NumMaxLimit && "この型の最大値・最大値を超えている");

	x--;
	y--;
	z--;

	return (*this);
}

template<class T>
inline decltype(auto) Vector3<T>::operator-() const
{
	// 符号ありでなければならない
	static_assert(std::is_signed<T>::value,
				  "This type is unsigned. This have to be signed.");

	return  V3<T>(-x, -y, -z);
}

template<class T>
inline decltype(auto) Vector3<T>::operator+() const
{
	return  V3<T>(+x, +y, +z);
}

#endif

//+ 演算子オーバーロード（Vector2と数値）
#if 1
template<class T>
inline constexpr decltype(auto) Vector3<T>::operator+(const T n) const noexcept
{
	using std::forward;

	return V3<> { forward<T>(x + n), forward<T>(y + n), forward<T>(z + n) };
}

template<class T>
inline constexpr decltype(auto) Vector3<T>::operator-(const T n) const noexcept
{
	using std::forward;

	return V3<> { forward<T>(x - n), forward<T>(y - n), forward<T>(z - n) };
}

template<class T>
inline constexpr decltype(auto) Vector3<T>::operator*(const T n) const noexcept
{
	using std::forward;

	return V3<> { forward<T>(x * n), forward<T>(y * n), forward<T>(z * n) };
}

template<class T>
inline constexpr decltype(auto) Vector3<T>::operator/(const T n) const
{
	assert(n != 0 && "0で割ろうとしている");

	using std::forward;

	return V3<> { forward<T>(x / n), forward<T>(y / n), forward<T>(z / n) };
}

template<class T>
inline constexpr decltype(auto) Vector3<T>::operator%(const T n) const
{
	assert(n != 0 && "0で割ろうとしている");

	using std::forward;

	if constexpr (std::is_floating_point<T>())
	{
		return  V3<>{ forward<T>(std::fmod<T>(x, n)), forward<T>(std::fmod<T>(y, n)), forward<T>(std::fmod<T>(z, n)) };
	}
	else
	{
		return V3<> { forward<T>(x % n), forward<T>(y % n), forward<T>(z % n) };
	}
}

template<class T>
inline decltype(auto) Vector3<T>::operator+=(const T n) noexcept
{
	x += (n);
	y += (n);
	z += (n);

	return (*this);
}

template<class T>
inline decltype(auto) Vector3<T>::operator-=(const T n) noexcept
{
	x -= (n);
	y -= (n);
	z -= (n);

	return (*this);
}

template<class T>
inline decltype(auto) Vector3<T>::operator*=(const T n) noexcept
{
	x *= (n);
	y *= (n);
	z *= (n);

	return (*this);
}

template<class T>
inline decltype(auto) Vector3<T>::operator/=(const T n)
{
	assert(n != static_cast<T>(0) && "0で割ろうとしている");

	x /= (n);
	y /= (n);
	z /= (n);

	return (*this);
}

template<class T>
inline decltype(auto) Vector3<T>::operator%=(const T n)
{
	assert(n != 0 && "0で割ろうとしている");

	if constexpr (std::is_floating_point<T>())
	{
		x = std::fmod<T>(x, n);
		y = std::fmod<T>(y, n);
		z = std::fmod<T>(z, n);
	}
	else
	{
		x %= (n);
		y %= (n);
		z %= (n);
	}

	return (*this);
}

template<class T>
inline constexpr bool Vector3<T>::operator==(const T n) const noexcept
{
	if constexpr (std::is_floating_point<T>())
	{
		return (AdjEqual(x, n) && AdjEqual(y, n) && AdjEqual(z, n));
	}
	else
	{
		return (x == n) && (y == n) && (z == n);
	}
}

template<class T>
inline constexpr bool Vector3<T>::operator!=(const T n) const noexcept
{
	if constexpr (std::is_floating_point<T>())
	{
		return !(AdjEqual(x, n) && AdjEqual(y, n) && AdjEqual(z, n));
	}
	else
	{
		return (x != n) || (y != n) || (z != n);
	}
}

template<class T>
inline constexpr bool Vector3<T>::operator>(const T n) const noexcept
{
	return (x > n) && (y > n);
}

template<class T>
inline constexpr bool Vector3<T>::operator<(const T n) const noexcept
{
	return (x < n) && (y < n);
}

template<class T>
inline constexpr bool Vector3<T>::operator>=(const T n) const noexcept
{
	return (x >= n) && (y >= n);
}

template<class T>
inline constexpr bool Vector3<T>::operator<=(const T n) const noexcept
{
	return (x <= n) && (y <= n);
}
#endif
#endif

//!+++ Vector2クラスの補助関数
namespace Vec3sub
{
	//!++ Vector2の簡易作成版
	// 宣言時にautoを組み合わせると楽
	template<class _Ty> inline
		constexpr decltype(auto)
		MakeVector3(_Ty&& content1, _Ty&& content2, _Ty&& content3) noexcept
	{
		using std::forward;

		return (Vector3<_Ty>(forward<_Ty>(content1), forward<_Ty>(content2), forward<_Ty>(content3)));
	}

	//!++ Vector2の簡易作成版
	// 宣言時にautoを組み合わせると楽
	template<class _Ty> inline
		constexpr decltype(auto)
		MakeVector3(const _Ty& content1, const _Ty& content2, const _Ty& content3) noexcept
	{
		using std::forward;

		return (Vector3<_Ty>((content1), (content2), (content3)));
	}

	//!++ Vector2の右辺値参照の簡易作成版
	template<class _Ty> inline
		constexpr decltype(auto)
		ForwardAsVector3(_Ty&& content1, _Ty&& content2, _Ty&& content3) noexcept
	{
		using std::forward;

		return (Vector3<_Ty&&>(forward<_Ty>(content1), forward<_Ty>(content2), forward<_Ty>(content3)));
	}

	//!++ Vector2の絶対値を取りたい場合
	template<class _Ty> inline
		constexpr decltype(auto)
		MakeFabsVector3(Vector3<_Ty>& v3) noexcept
	{
		using std::forward;

		// 符号ありでなければならない
		static_assert(std::is_signed<_Ty>::value, "This type is unsigned. This have to be signed.");

		return (Vector3<_Ty>(forward<_Ty>(static_cast<_Ty>(fabs(v3.x))), forward<_Ty>(static_cast<_Ty>(fabs(v3.y))), forward<_Ty>(static_cast<_Ty>(fabs(v3.z)))));
	}

	//!++ Vector2のXY逆転させた数値を取りたい場合
	template<class _Ty> inline
		constexpr decltype(auto)
		MakeSwapXYVector3(Vector3<_Ty>& v3) noexcept
	{
		using std::forward;

		return (Vector3<_Ty>(forward<_Ty>(static_cast<_Ty>(fabs(v3.y))), forward<_Ty>(static_cast<_Ty>(fabs(v3.x))), forward<_Ty>(static_cast<_Ty>(fabs(v3.z)))));
	}

	//!++ 各数値を絶対値にした上でVector2を取りたい場合
	template<class _Ty> inline
		constexpr decltype(auto)
		MakeFabsVector3(_Ty&& content1, _Ty&& content2, _Ty&& content3) noexcept
	{
		using std::forward;

		// 符号ありでなければならない
		static_assert(std::is_signed<_Ty>::value, "This type is unsigned. This have to be signed.");

		return (Vector3<_Ty>(forward<_Ty>(static_cast<_Ty>(fabs(content1))), forward<_Ty>(static_cast<_Ty>(fabs(content2))), forward<_Ty>(static_cast<_Ty>(fabs(content3)))));
	}
}

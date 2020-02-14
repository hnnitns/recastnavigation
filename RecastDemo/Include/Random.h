#pragma once

//--------------------------------
//!++	インクルード
//--------------------------------
#include <random>
#include <type_traits>
#include <limits>

//--------------------------------
//!+++	クラス
//--------------------------------

//++ 乱数生成クラス（整数版）
class RndIntMaker final
{
public:  // コンストラクタ・デストラクタ
	RndIntMaker(const int max = (std::numeric_limits<int>::max)(),
		const int min = (std::numeric_limits<int>::min)()) :
		mt { device() }, random{ min, max }, max { max }, min { min }
	{ }
	~RndIntMaker() = default;
	RndIntMaker(const RndIntMaker&) = delete;
	RndIntMaker(RndIntMaker&&) noexcept = delete;
	auto operator=(const RndIntMaker&) = delete;
	auto operator=(RndIntMaker&&) noexcept = delete;

private:  // 変数
	std::random_device device;  // ランダム生成器
	std::mt19937 mt;  // メルセンヌ・ツイスタ生成器
	std::uniform_int_distribution<> random;  // 範囲指定乱数生成（int版）
public:
	const int max;  // 乱数最大値
	const int min;  // 乱数最小値

public:  // 関数

	// ランダム値取得（整数版）
	template<typename _Ty = int>
	[[nodiscard]] _Ty GetRnd()
	{
		static_assert(std::is_integral<_Ty>::value, "This type isn't integral. It has to be integral.");
		return static_cast<_Ty>(random(mt));
	}
};

//++ 乱数生成クラス（浮動小数版）
class RndDoubleMaker final
{
public:  // コンストラクタ・デストラクタ
	RndDoubleMaker(const double max = (std::numeric_limits<double>::max)(),
		const double min = (std::numeric_limits<double>::min)()) :
		mt { device() }, random{ min, max }, max { max }, min { min }
	{ }
	~RndDoubleMaker() = default;
	RndDoubleMaker(const RndDoubleMaker&) = delete;
	RndDoubleMaker(RndDoubleMaker&&) noexcept = delete;
	auto operator=(const RndDoubleMaker&) = delete;
	auto operator=(RndDoubleMaker&&) noexcept = delete;

private:  // 変数
	std::random_device device;  // ランダム生成器
	std::mt19937 mt;  // メルセンヌ・ツイスタ生成器
	std::uniform_real_distribution<> random; // 範囲指定乱数生成（double版）
public:
	const double max;  // 乱数最大値
	const double min;  // 乱数最小値

public:  // 関数

	// ランダム値取得（浮動小数点版）
	template<typename _Ty = double>
	[[nodiscard]] _Ty GetRnd()
	{
		static_assert(std::is_floating_point<_Ty>::value, "This type isn't floating_point. It has to be floating_point.");
		return static_cast<_Ty>(random(mt));
	}
};
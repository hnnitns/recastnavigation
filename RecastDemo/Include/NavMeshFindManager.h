#pragma once

#include <memory>
#include <vector>
#include <array>

class NavMeshFindManager final
{
private:
	static constexpr int MAX_POLYS{ 256 };
	static constexpr int MAX_SMOOTH{ 2048 };

public:
	NavMeshFindManager();
	~NavMeshFindManager() = default;
	NavMeshFindManager(const NavMeshFindManager&) = delete;
	auto& operator=(const NavMeshFindManager&) = delete;
	NavMeshFindManager(NavMeshFindManager&&) = delete;
	auto& operator=(NavMeshFindManager&&) = delete;

public:
	bool Init(class Sample* sample);

	// start_posからend_posへ経路探索を行う
	// 滑らかな経路になるが低速
	bool FindSmoothPath(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
		std::vector<std::array<float, 3>>* result_path, const size_t max_polygon_count = MAX_POLYS, const size_t max_smooth_count = MAX_SMOOTH / 3);

	// start_posからend_posへ経路探索を行う
	// 直線的な経路になるが高速
	bool FindStraightPath(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
		std::vector<std::array<float, 3>>* result_straight_path, const size_t max_path_count = MAX_POLYS / 3);

	// ナビメッシュ上でのレイキャストを行う
	// Y軸を無視する代わりに、かなり高速なレイキャスト
	bool RayCast(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos, bool* is_hit,
		std::array<float, 3>* hit_position = nullptr, float* distance = nullptr, std::array<float, 3>* hit_normal = nullptr,
		const size_t max_path_count = MAX_POLYS / 3);

	// 検索サイズの変更
	void SetSearchSize(const std::array<float, 3>& search_size) noexcept;

private:
	class Sample* m_sample;
	std::unique_ptr<class NavMeshTesterTool> m_state;
};


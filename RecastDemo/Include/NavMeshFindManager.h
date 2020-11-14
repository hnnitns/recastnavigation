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

	// start_pos����end_pos�֌o�H�T�����s��
	// ���炩�Ȍo�H�ɂȂ邪�ᑬ
	bool FindSmoothPath(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
		std::vector<std::array<float, 3>>* result_path, const size_t max_polygon_count = MAX_POLYS, const size_t max_smooth_count = MAX_SMOOTH / 3);

	// start_pos����end_pos�֌o�H�T�����s��
	// �����I�Ȍo�H�ɂȂ邪����
	bool FindStraightPath(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
		std::vector<std::array<float, 3>>* result_straight_path, const size_t max_path_count = MAX_POLYS / 3);

	// �i�r���b�V����ł̃��C�L���X�g���s��
	// Y���𖳎��������ɁA���Ȃ荂���ȃ��C�L���X�g
	bool RayCast(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos, bool* is_hit,
		std::array<float, 3>* hit_position = nullptr, float* distance = nullptr, std::array<float, 3>* hit_normal = nullptr,
		const size_t max_path_count = MAX_POLYS / 3);

	// �����T�C�Y�̕ύX
	void SetSearchSize(const std::array<float, 3>& search_size) noexcept;

private:
	class Sample* m_sample;
	std::unique_ptr<class NavMeshTesterTool> m_state;
};


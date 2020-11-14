#include "NavMeshFindManager.h"
#include "NavMeshTesterTool.h"

NavMeshFindManager::NavMeshFindManager()
	: m_sample()
{}

NavMeshFindManager::~NavMeshFindManager()
{
}

bool NavMeshFindManager::Init(Sample* sample)
{
	if (m_sample != sample)
	{
		m_sample = sample;
	}

	if (!sample) return false;

	if (!m_state) m_state = std::make_unique<NavMeshTesterTool>();

	m_state->init(sample);

	return true;
}

bool NavMeshFindManager::FindSmoothPath(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
	std::vector<std::array<float, 3>>* result_path, const size_t max_polygon_count, const size_t max_smooth_count)
{
	if (m_state)	return false;

	return dtStatusFailed(m_state->FindSmoothPath(start_pos, end_pos, result_path, max_polygon_count, max_smooth_count));
}

bool NavMeshFindManager::FindStraightPath(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
	std::vector<std::array<float, 3>>* result_straight_path, const size_t max_path_count)
{
	if (m_state)	return false;

	return dtStatusFailed(m_state->FindStraightPath(start_pos, end_pos, result_straight_path, max_path_count));
}

bool NavMeshFindManager::RayCast(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
	bool* is_hit, std::array<float, 3>* hit_position, float* distance, std::array<float, 3>* hit_normal,
	const size_t max_path_count)
{
	if (m_state)	return false;

	return dtStatusFailed(m_state->RayCast(start_pos, end_pos, is_hit, hit_position, distance, hit_normal, max_path_count));
}

void NavMeshFindManager::SetSearchSize(const std::array<float, 3>& search_size) noexcept
{
	if (m_state)	return;

	m_state->SetSearchSize(search_size);
}

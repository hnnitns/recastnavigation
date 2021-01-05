#include "OffMeshLinkManager.h"
#include "Sample.h"
#include "DetourDebugDraw.h"

OffMeshLinkManager::OffMeshLinkManager()
	: m_sample(), m_oldFlags(), link_count()
{}

OffMeshLinkManager::~OffMeshLinkManager()
{}

bool OffMeshLinkManager::Init(Sample * sample)
{
	if (m_sample != sample)
	{
		m_sample = sample;
		m_oldFlags = m_sample->getNavMeshDrawFlags();
		m_sample->setNavMeshDrawFlags(m_oldFlags & ~DU_DRAWNAVMESH_OFFMESHCONS);
	}

	return true;
}

void OffMeshLinkManager::Draw()
{
	duDebugDraw& dd = m_sample->getDebugDraw();
	const float s = m_sample->getAgentRadius();

	auto& geom = m_sample->getInputGeom();
	if (!geom)	return;

	geom->drawOffMeshConnections(&dd, true);
}

int OffMeshLinkManager::AddLink(
	const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos, const bool is_bidirectional)
{
	constexpr unsigned char area = SAMPLE_POLYAREA_JUMP;
	constexpr unsigned short flags = SAMPLE_POLYFLAGS_JUMP;

	auto& geom = m_sample->getInputGeom();

	if (!geom)	return -1;

	return geom->addOffMeshConnection(start_pos.data(), end_pos.data(), m_sample->getAgentRadius(),
		is_bidirectional ? 1 : 0, area, flags);
}

void OffMeshLinkManager::DeleteLink(const int index)
{
	auto& geom = m_sample->getInputGeom();

	if (!geom)	return;

	geom->deleteOffMeshConnection(index);
}

bool OffMeshLinkManager::HitLinkPoint(const std::array<float, 3>& point, const int index)
{
	auto& geom = m_sample->getInputGeom();

	// 不正なインデックス
	if (!geom || index < 0 || index >= geom->getOffMeshConnectionCount())	return;

	const auto& verts{ geom->getOffMeshConnectionVerts() };

	const float dist{ rcVdistSqr(point.data(), &verts[index * 3]) };

	// エンドポイントが十分に近い場合
	return (::sqrtf(dist) < m_sample->getAgentRadius());
}

bool OffMeshLinkManager::HitLinksPoint(const std::array<float, 3>& point, int* index)
{
	auto& geom = m_sample->getInputGeom();

	if (!(geom && index))	return;

	// 最も近いリンクのエンドポイントを見つける
	float nearestDist{ (std::numeric_limits<float>::max)() };
	int nearestIndex{ -1 };

	const auto& verts{ geom->getOffMeshConnectionVerts() };

	for (int i = 0; i < geom->getOffMeshConnectionCount() * 2; ++i)
	{
		const float d{ rcVdistSqr(point.data(), &verts[i * 3]) };

		if (d < nearestDist)
		{
			nearestDist = d;
			nearestIndex = i / 2; // 各リンクには2つの頂点があります。
		}
	}

	if (nearestIndex == -1)	return false;

	*index = -1;

	// エンドポイントが十分に近い場合
	if (::sqrtf(nearestDist) < m_sample->getAgentRadius())
	{
		*index = nearestIndex;
		return true;
	}

	return false;
}

int OffMeshLinkManager::GetLinkSize() const noexcept
{
	auto& geom = m_sample->getInputGeom();

	if (!geom)	return 0;

	return geom->getOffMeshConnectionCount();
}

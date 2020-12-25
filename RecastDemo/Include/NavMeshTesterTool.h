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

#ifndef NAVMESHTESTERTOOL_H
#define NAVMESHTESTERTOOL_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

class NavMeshTesterTool : public SampleTool
{
private:
	static constexpr int MAX_POLYS{ 256 };
	static constexpr int MAX_SMOOTH{ 2048 };
	static constexpr int MAX_RAND_POINTS{ 64 };
	static constexpr int MAX_STEER_POINTS{ 10 };

	Sample* m_sample;

	dtNavMesh* m_navMesh;
	dtNavMeshQuery* m_navQuery;

	dtQueryFilter m_filter;

	dtStatus m_pathFindStatus;

	enum class ToolMode
	{
		PATHFIND_FOLLOW,
		PATHFIND_STRAIGHT,
		PATHFIND_SLICED,
		RAYCAST,
		DISTANCE_TO_WALL,
		FIND_POLYS_IN_CIRCLE,
		FIND_POLYS_IN_SHAPE,
		FIND_LOCAL_NEIGHBOURHOOD,
	};

	ToolMode m_toolMode;

	int m_straightPathOptions;

	dtPolyRef m_startRef;
	dtPolyRef m_endRef;
	std::array<dtPolyRef, MAX_POLYS> m_polys;
	std::array<dtPolyRef, MAX_POLYS> m_parent;
	int m_npolys;
	std::array<float, MAX_POLYS * 3> m_straightPath;
	std::array<unsigned char, MAX_POLYS> m_straightPathFlags;
	std::array<dtPolyRef, MAX_POLYS> m_straightPathPolys;
	int m_nstraightPath;
	std::array<float, 3> m_polyPickExt;
	std::array<float, MAX_SMOOTH * 3> m_smoothPath;
	int m_nsmoothPath;
	std::array<float, 4 * 3> m_queryPoly;

	std::array<float, MAX_RAND_POINTS * 3> m_randPoints;
	int m_nrandPoints;
	bool m_randPointsInCircle;

	std::array<float, 3> m_spos, m_epos, m_hitPos, m_hitNormal;
	bool m_hitResult;
	float m_distanceToWall;
	float m_neighbourhoodRadius;
	float m_randomRadius;
	bool m_sposSet;
	bool m_eposSet;

	int m_pathIterNum;
	std::array<dtPolyRef, MAX_POLYS> m_pathIterPolys;
	int m_pathIterPolyCount;
	std::array<float, 3> m_prevIterPos, m_iterPos, m_steerPos, m_targetPos;

	std::array<float, MAX_STEER_POINTS * 3> m_steerPoints;
	int m_steerPointCount;

	std::array<float, 3> search_size;

public:
	NavMeshTesterTool();

	int type() override { return TOOL_NAVMESH_TESTER; }
	void init(Sample* sample) override;
	void reset() override;
	void handleMenu() override;
	void handleClickDown(const float* s, const float* p, bool shift) override;
	void handleClickUp(const float* /*s*/, const float* /*p*/) override {}
	void handleClick(const float* /*s*/, const float* /*p*/) override {}

	// パスの追跡処理を行う
	void handleToggle() override;
	void handleStep() override;
	void handleUpdate(const float dt) override;
	void handleRender() override;
	void handleRenderOverlay(double* proj, double* model, int* view) override;

	// 経路探索などを行う
	void recalc();
	void drawAgent(const float* pos, float r, float h, float c, const unsigned int col);

	// 以降はオリジナル関数

	// start_posからend_posへ経路探索を行う
	// 滑らかな経路になるが低速
	dtStatus FindSmoothPath(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
		std::vector<std::array<float, 3>>* result_path, const size_t max_polygon_count = MAX_POLYS,
		const size_t max_smooth_count = MAX_SMOOTH / 3) const;

	// start_posからend_posへ経路探索を行う
	// 直線的な経路になるが高速
	dtStatus FindStraightPath(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
		std::vector<std::array<float, 3>>* result_straight_path, const size_t max_path_count = MAX_POLYS / 3) const;

	// ナビメッシュ上でのレイキャストを行う
	// Y軸を無視する代わりに、かなり高速なレイキャスト
	dtStatus RayCast(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos, bool* is_hit,
		std::array<float, 3>* hit_position = nullptr, float* distance = nullptr, std::array<float, 3>* hit_normal = nullptr,
		const size_t max_path_count = MAX_POLYS / 3) const;

	// 検索サイズの変更
	void SetSearchSize(const std::array<float, 3>& _search_size) noexcept { this->search_size = _search_size; };
};

#endif // NAVMESHTESTERTOOL_H
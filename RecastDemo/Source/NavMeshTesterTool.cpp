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

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include "SDL.h"
#include "SDL_opengl.h"
#include <GL/glu.h>
#include "imgui.h"
#include "NavMeshTesterTool.h"
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "DetourCommon.h"
#include "Random.h"

#if _WIN32
#define printf	printf_s
#endif

#pragma warning (disable : 4702)

using namespace DtOperator;

// Uncomment this to dump all the requests in stdout.
// これのコメントを外して、すべてのリクエストを標準出力にダンプします。
#define DUMP_REQS

namespace
{
	// Returns a random number [0..1]
	// 乱数[0..1]を返します
	float frand()
	{
		static RndDoubleMaker random{ 1.0, 0.0 };

		return random.GetRnd<float>();
	}

	inline constexpr bool inRange(const float* v1, const ArrayF& v2, const float r, const float h)
	{
		const float dx = v2[0] - v1[0];
		const float dy = v2[1] - v1[1];
		const float dz = v2[2] - v1[2];
		return (dx * dx + dz * dz) < r * r && fabsf(dy) < h;
	}

	inline constexpr bool inRange(const ArrayF& v1, const ArrayF& v2, const float r, const float h)
	{
		return inRange(v1.data(), v2, r, h);
	}

	template<size_t PolysSize = MAX_POLYS, size_t VisitSize = 16>
	int fixupCorridor(std::array<dtPolyRef, PolysSize>* path, const int npath, const int maxPath,
		const std::array<dtPolyRef, VisitSize>& visited, const int nvisited)
	{
		int furthestPath = -1;
		int furthestVisited = -1;

		// Find furthest common polygon.
		for (int i = npath - 1; i >= 0; --i)
		{
			bool found = false;
			for (int j = nvisited - 1; j >= 0; --j)
			{
				if (path->at(i) == visited[j])
				{
					furthestPath = i;
					furthestVisited = j;
					found = true;
				}
			}
			if (found)
				break;
		}

		// If no intersection found just return current path.
		// 交差点が見つからない場合は、現在のパスを返します。
		if (furthestPath == -1 || furthestVisited == -1)
			return npath;

		// Concatenate paths.
		//パスを連結します。

		// Adjust beginning of the buffer to include the visited.
		// バッファの先頭を調整して、訪問先を含めます。
		const int req = nvisited - furthestVisited;
		const int orig = rcMin(furthestPath + 1, npath);
		int size = rcMax(0, npath - orig);

		if (req + size > maxPath)
			size = maxPath - req;

		if (size != 0)
		{
			size_t i{};

			std::for_each_n(path->begin() + req, size, [&](dtPolyRef& pt) { pt = path->at(orig + i++); });
		}

		// Store visited
		for (int i = 0; i < req; ++i)
			path->at(i) = visited[(nvisited - 1) - i];

		return req + size;
	}

	// This function checks if the path has a small U-turn,
	// この関数は、パスに小さなUターンがあるかどうか、
	// that is, a polygon further in the path is adjacent to the first polygon in the path.
	// つまり、パス内のポリゴンがパス内の最初のポリゴンに隣接しているかどうかを確認します。
	// If that happens, a shortcut is taken.
	// その場合、ショートカットが使用されます。
	// This can happen if the target (T) location is at tile boundary, and we're (S) approaching it parallel to the tile edge.
	// これは、ターゲット（T）の位置がタイルの境界にあり、タイルの端に平行に（S）接近している場合に発生する可能性があります。
	// The choice at the vertex can be arbitrary,
	// 頂点での選択は任意です。
	//  +---+---+
	//  |:::|:::|
	//  +-S-+-T-+
	//  |:::|   | <-- the step can end up in here, resulting U-turn path. //ステップはここに到達し、結果としてUターンパスになります。
	//  +---+---+
	template<size_t PolySize = MAX_POLYS>
	int fixupShortcuts(std::array<dtPolyRef, PolySize>* path, int npath, dtNavMeshQuery* navQuery)
	{
		if (npath < 3)
			return npath;

		// Get connected polygons
		constexpr int maxNeis = 16;

		std::array<dtPolyRef, maxNeis> neis{};
		int nneis{};
		const dtMeshTile* tile{};
		const dtPoly* poly{};

		if (dtStatusFailed(navQuery->getAttachedNavMesh()->getTileAndPolyByRef(path->at(0), &tile, &poly)))
			return npath;

		for (uint32_t k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
		{
			const dtLink& link = tile->links[k];

			if (link.ref != 0)
			{
				if (nneis < maxNeis)
					neis[nneis++] = link.ref;
			}
		}

		// If any of the neighbour polygons is within the next few polygons
		// in the path, short cut to that polygon directly.
		constexpr int maxLookAhead = 6;
		int cut{};

		for (int i = dtMin(maxLookAhead, npath) - 1; i > 1 && cut == 0; i--)
		{
			for (int j = 0; j < nneis; j++)
			{
				if (path->at(i) == neis[j])
				{
					cut = i;
					break;
				}
			}
		}

		if (cut > 1)
		{
			int offset = cut - 1;

			npath -= offset;

			for (int i = 1; i < npath; i++)
				path->at(i) = path->at(i + offset);
		}

		return npath;
	}

	template<size_t Size = 0>
	bool getSteerTarget(dtNavMeshQuery* navQuery, const ArrayF& startPos, const ArrayF& endPos,
		const float minTargetDist, const std::array<dtPolyRef, MAX_POLYS>& path, const int pathSize,
		ArrayF* steerPos, uint8_t& steerPosFlag, dtPolyRef& steerPosRef,
		std::array<float, Size>* outPoints = nullptr, int* outPointCount = nullptr)
	{
		// Find steer target.
		// ステアターゲットを見つけます。
		constexpr int MAX_STEER_POINTS = 3;

		std::array<float, MAX_STEER_POINTS * 3> steerPath{};
		std::array<uint8_t, MAX_STEER_POINTS> steerPathFlags{};
		std::array<dtPolyRef, MAX_STEER_POINTS> steerPathPolys{};
		int nsteerPath{};

		navQuery->findStraightPath(startPos, endPos, path.data(), pathSize,
			steerPath.data(), steerPathFlags.data(), steerPathPolys.data(), &nsteerPath, MAX_STEER_POINTS);

		if (!nsteerPath) return false;

		if (outPoints && outPointCount)
		{
			*outPointCount = nsteerPath;

			for (int i = 0; i < nsteerPath; ++i)
				dtVcopy(&outPoints->at(i * 3), &steerPath[i * 3]);
		}

		// Find vertex far enough to steer to.
		// ステアするのに十分な頂点を見つけます。
		int ns{};

		while (ns < nsteerPath)
		{
			// Stop at Off-Mesh link or when point is further than slop away.
			// オフメッシュリンクで停止するか、ポイントが傾斜よりも遠くにあるときに停止します。
			if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
				!inRange(&steerPath[ns * 3], startPos, minTargetDist, 1000.0f))
				break;

			ns++;
		}

		// Failed to find good point to steer to.
		// 適切なステアリングポイントを見つけることができませんでした。
		if (ns >= nsteerPath) return false;

		dtVcopy(steerPos->data(), &steerPath[ns * 3]);
		steerPos->at(1) = startPos[1];
		steerPosFlag = steerPathFlags[ns];
		steerPosRef = steerPathPolys[ns];

		return true;
	}

	inline void getPolyCenter(dtNavMesh* navMesh, dtPolyRef ref, ArrayF* center)
	{
		center->fill(0.f);

		const dtMeshTile* tile = 0;
		const dtPoly* poly = 0;
		dtStatus status = navMesh->getTileAndPolyByRef(ref, &tile, &poly);

		if (dtStatusFailed(status)) return;

		for (int i = 0; i < (int)poly->vertCount; ++i)
		{
			const float* v = &tile->verts[poly->verts[i] * 3];

			center->at(0) += v[0];
			center->at(1) += v[1];
			center->at(2) += v[2];
		}

		*center *= (1.f / poly->vertCount);
	}
}

NavMeshTesterTool::NavMeshTesterTool()
	: m_sample(nullptr), m_navMesh(nullptr), m_navQuery(nullptr), m_filter{}, m_pathFindStatus(DT_FAILURE),
	m_toolMode(ToolMode::TOOLMODE_PATHFIND_FOLLOW), m_straightPathOptions(0), m_startRef(0), m_endRef(0),
	m_polys{}, m_parent{}, m_npolys(0), m_straightPath{}, m_straightPathFlags{}, m_straightPathPolys{},
	m_nstraightPath(0), m_polyPickExt{}, m_smoothPath{}, m_nsmoothPath(0), m_queryPoly{}, m_randPoints{},
	m_nrandPoints(0), m_randPointsInCircle(false), m_spos{}, m_epos{}, m_hitPos{}, m_hitNormal{},
	m_hitResult(false), m_distanceToWall(0), m_neighbourhoodRadius(0), m_randomRadius(0),
	m_sposSet(false), m_eposSet(false), m_pathIterNum(0), m_pathIterPolyCount(0), m_steerPointCount(0)
{
	m_filter.setIncludeFlags(SAMPLE_POLYFLAGS_ALL ^ SAMPLE_POLYFLAGS_DISABLED);
	m_filter.setExcludeFlags(0);

	m_polyPickExt[0] = 2;
	m_polyPickExt[1] = 4;
	m_polyPickExt[2] = 2;

	m_neighbourhoodRadius = 2.5f;
	m_randomRadius = 5.0f;
}

void NavMeshTesterTool::init(Sample* sample)
{
	m_sample = sample;
	m_navMesh = sample->getNavMesh();
	m_navQuery = sample->getNavMeshQuery();
	recalc();

	if (m_navQuery)
	{
		// Change costs.
		m_filter.setAreaCost(SAMPLE_POLYAREA_TYPE_GROUND, 1.f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_TYPE_WATER, 10.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_TYPE_ROAD, 1.f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_FLAG_DOOR, 1.f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_TYPE_GRASS, 2.0f);
		m_filter.setAreaCost(SAMPLE_POLYAREA_FLAG_JUMP, 1.5f);
	}

	m_neighbourhoodRadius = sample->getAgentRadius() * 20.0f;
	m_randomRadius = sample->getAgentRadius() * 30.0f;
}

void NavMeshTesterTool::handleMenu()
{
	if (imguiCheck("Pathfind Follow", m_toolMode == ToolMode::TOOLMODE_PATHFIND_FOLLOW))
	{
		m_toolMode = ToolMode::TOOLMODE_PATHFIND_FOLLOW;
		recalc();
	}
	if (imguiCheck("Pathfind Straight", m_toolMode == ToolMode::TOOLMODE_PATHFIND_STRAIGHT))
	{
		m_toolMode = ToolMode::TOOLMODE_PATHFIND_STRAIGHT;
		recalc();
	}
	if (m_toolMode == ToolMode::TOOLMODE_PATHFIND_STRAIGHT)
	{
		imguiIndent();
		imguiLabel("Vertices at crossings");

		if (imguiCheck("None", m_straightPathOptions == 0))
		{
			m_straightPathOptions = 0;
			recalc();
		}
		if (imguiCheck("Area", m_straightPathOptions == DT_STRAIGHTPATH_AREA_CROSSINGS))
		{
			m_straightPathOptions = DT_STRAIGHTPATH_AREA_CROSSINGS;
			recalc();
		}
		if (imguiCheck("All", m_straightPathOptions == DT_STRAIGHTPATH_ALL_CROSSINGS))
		{
			m_straightPathOptions = DT_STRAIGHTPATH_ALL_CROSSINGS;
			recalc();
		}

		imguiUnindent();
	}
	if (imguiCheck("Pathfind Sliced", m_toolMode == ToolMode::TOOLMODE_PATHFIND_SLICED))
	{
		m_toolMode = ToolMode::TOOLMODE_PATHFIND_SLICED;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Distance to Wall", m_toolMode == ToolMode::TOOLMODE_DISTANCE_TO_WALL))
	{
		m_toolMode = ToolMode::TOOLMODE_DISTANCE_TO_WALL;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Raycast", m_toolMode == ToolMode::TOOLMODE_RAYCAST))
	{
		m_toolMode = ToolMode::TOOLMODE_RAYCAST;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Find Polys in Circle", m_toolMode == ToolMode::TOOLMODE_FIND_POLYS_IN_CIRCLE))
	{
		m_toolMode = ToolMode::TOOLMODE_FIND_POLYS_IN_CIRCLE;
		recalc();
	}
	if (imguiCheck("Find Polys in Shape", m_toolMode == ToolMode::TOOLMODE_FIND_POLYS_IN_SHAPE))
	{
		m_toolMode = ToolMode::TOOLMODE_FIND_POLYS_IN_SHAPE;
		recalc();
	}

	imguiSeparator();

	if (imguiCheck("Find Local Neighbourhood", m_toolMode == ToolMode::TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD))
	{
		m_toolMode = ToolMode::TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD;
		recalc();
	}

	imguiSeparator();

	if (imguiButton("Set Random Start"))
	{
		dtStatus status = m_navQuery->findRandomPoint(&m_filter, frand, &m_startRef, &m_spos);
		if (dtStatusSucceed(status))
		{
			m_sposSet = true;
			recalc();
		}
	}
	if (imguiButton("Set Random End", m_sposSet))
	{
		if (m_sposSet)
		{
			dtStatus status = m_navQuery->findRandomPointAroundCircle(m_startRef, m_spos.data(), m_randomRadius, &m_filter, frand, &m_endRef, &m_epos);
			if (dtStatusSucceed(status))
			{
				m_eposSet = true;
				recalc();
			}
		}
	}

	imguiSeparator();

	if (imguiButton("Make Random Points"))
	{
		m_randPointsInCircle = false;
		m_nrandPoints = 0;

		for (int i = 0; i < MAX_RAND_POINTS; i++)
		{
			ArrayF pt{};
			dtPolyRef ref{};
			dtStatus status = m_navQuery->findRandomPoint(&m_filter, frand, &ref, &pt);

			if (dtStatusSucceed(status))
			{
				dtVcopy(&m_randPoints[m_nrandPoints * 3], pt.data());
				m_nrandPoints++;
			}
		}
	}
	if (imguiButton("Make Random Points Around", m_sposSet))
	{
		if (m_sposSet)
		{
			m_nrandPoints = 0;
			m_randPointsInCircle = true;

			for (int i = 0; i < MAX_RAND_POINTS; i++)
			{
				ArrayF pt{};
				dtPolyRef ref{};
				dtStatus status = m_navQuery->findRandomPointAroundCircle(m_startRef, m_spos.data(), m_randomRadius, &m_filter, frand, &ref, &pt);

				if (dtStatusSucceed(status))
				{
					dtVcopy(&m_randPoints[m_nrandPoints * 3], pt.data());
					m_nrandPoints++;
				}
			}
		}
	}

	imguiSeparator();

	imguiLabel("Include Flags");

	imguiIndent();
	if (imguiCheck("Walk", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_WALK) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_WALK);
		recalc();
	}
	if (imguiCheck("Swim", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_SWIM) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_SWIM);
		recalc();
	}
	if (imguiCheck("Door", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_DOOR) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_DOOR);
		recalc();
	}
	if (imguiCheck("Jump", (m_filter.getIncludeFlags() & SAMPLE_POLYFLAGS_JUMP) != 0))
	{
		m_filter.setIncludeFlags(m_filter.getIncludeFlags() ^ SAMPLE_POLYFLAGS_JUMP);
		recalc();
	}
	imguiUnindent();

	imguiSeparator();
	imguiLabel("Exclude Flags");

	imguiIndent();
	if (imguiCheck("Walk", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_WALK) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_WALK);
		recalc();
	}
	if (imguiCheck("Swim", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_SWIM) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_SWIM);
		recalc();
	}
	if (imguiCheck("Door", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_DOOR) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_DOOR);
		recalc();
	}
	if (imguiCheck("Jump", (m_filter.getExcludeFlags() & SAMPLE_POLYFLAGS_JUMP) != 0))
	{
		m_filter.setExcludeFlags(m_filter.getExcludeFlags() ^ SAMPLE_POLYFLAGS_JUMP);
		recalc();
	}
	imguiUnindent();

	imguiSeparator();
}

void NavMeshTesterTool::handleClick(const float* /*s*/, const float* p, bool shift)
{
	// スタート地点設定
	if (shift)
	{
		m_sposSet = true;
		dtVcopy(m_spos.data(), p);
	}
	// ゴール地点設定
	else
	{
		m_eposSet = true;
		dtVcopy(m_epos.data(), p);
	}
	recalc();
}

void NavMeshTesterTool::handleStep()
{
}

void NavMeshTesterTool::handleToggle()
{
	// TODO: merge separate to a path iterator. Use same code in recalc() too. // 個別にパスイテレータにマージします。 recalc（）でも同じコードを使用します。
	if (m_toolMode != ToolMode::TOOLMODE_PATHFIND_FOLLOW) return;

	if (!m_sposSet || !m_eposSet || !m_startRef || !m_endRef) return;

	constexpr float STEP_SIZE = 0.5f;
	constexpr float SLOP = 0.01f;

	if (m_pathIterNum == 0)
	{
		m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, &m_polys, &m_npolys, MAX_POLYS);
		m_nsmoothPath = 0;

		m_pathIterPolyCount = m_npolys;

		if (m_pathIterPolyCount)
		{
			size_t i{};
			std::for_each_n(std::begin(m_pathIterPolys), m_pathIterPolyCount,
				[&](dtPolyRef& poly) { poly = m_polys[i++]; });
		}

		if (m_pathIterPolyCount)
		{
			// Iterate over the path to find smooth path on the detail mesh surface.
			// パスを反復処理して、詳細メッシュサーフェス上の滑らかなパスを見つけます。
			m_navQuery->closestPointOnPoly(m_startRef, m_spos.data(), m_iterPos.data(), 0);
			m_navQuery->closestPointOnPoly(m_pathIterPolys[m_pathIterPolyCount - 1], m_epos.data(), m_targetPos.data(), 0);

			m_nsmoothPath = 0;

			dtVcopy(&m_smoothPath[m_nsmoothPath * 3], m_iterPos.data());
			m_nsmoothPath++;
		}
	}

	m_prevIterPos = m_iterPos;
	m_pathIterNum++;

	if (!m_pathIterPolyCount) return;
	if (m_nsmoothPath >= MAX_SMOOTH) return;

	// Move towards target a small advancement at a time until target reached or
	// when ran out of memory to store the path.
	// ターゲットに到達するまで、またはパスを保存するためにメモリを使い果たしたときに、少しずつターゲットに向かって移動します。

	// Find location to steer towards.
	// 操縦する場所を見つけます。
	ArrayF steerPos{};
	uint8_t steerPosFlag{};
	dtPolyRef steerPosRef{};

	if (!getSteerTarget(m_navQuery, m_iterPos, m_targetPos, SLOP,
		m_pathIterPolys, m_pathIterPolyCount, &steerPos, steerPosFlag, steerPosRef,
		&m_steerPoints, &m_steerPointCount))
		return;

	m_steerPos = steerPos;

	bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
	bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;

	// Find movement delta.
	ArrayF delta{ steerPos - m_iterPos };
	float len{ sqrtf(dtVdot(delta, delta)) };

	// If the steer target is end of path or off-mesh link, do not move past the location.
	if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
		len = 1;
	else
		len = STEP_SIZE / len;

	ArrayF moveTgt{};
	dtVmad(&moveTgt, m_iterPos, delta, len);

	// Move
	ArrayF result{};
	std::array<dtPolyRef, 16> visited{};
	int nvisited{};
	float h{};

	m_navQuery->moveAlongSurface(m_pathIterPolys[0], m_iterPos.data(), moveTgt.data(), &m_filter,
		result.data(), visited.data(), &nvisited, 16);

	m_pathIterPolyCount = fixupCorridor(&m_pathIterPolys, m_pathIterPolyCount, MAX_POLYS, visited, nvisited);
	m_pathIterPolyCount = fixupShortcuts(&m_pathIterPolys, m_pathIterPolyCount, m_navQuery);

	m_navQuery->getPolyHeight(m_pathIterPolys[0], result.data(), &h);

	result[1] = h;
	m_iterPos = result;

	// Handle end of path and off-mesh links when close enough.
	if (endOfPath && inRange(m_iterPos, steerPos, SLOP, 1.f))
	{
		// Reached end of path.
		m_iterPos = m_targetPos;

		if (m_nsmoothPath < MAX_SMOOTH)
		{
			dtVcopy(&m_smoothPath[m_nsmoothPath * 3], m_iterPos.data());
			m_nsmoothPath++;
		}
		return;
	}
	else if (offMeshConnection && inRange(m_iterPos, steerPos, SLOP, 1.f))
	{
		// Reached off-mesh connection.
		ArrayF startPos{}, endPos{};

		// Advance the path up to and over the off-mesh connection.
		dtPolyRef prevRef{}, polyRef = m_pathIterPolys[0];
		int npos{};

		while (npos < m_pathIterPolyCount && polyRef != steerPosRef)
		{
			prevRef = polyRef;
			polyRef = m_pathIterPolys[npos];
			npos++;
		}

		for (int i = npos; i < m_pathIterPolyCount; ++i)
			m_pathIterPolys[i - npos] = m_pathIterPolys[i];

		m_pathIterPolyCount -= npos;

		// Handle the connection.
		dtStatus status = m_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos.data(), endPos.data());

		if (dtStatusSucceed(status))
		{
			if (m_nsmoothPath < MAX_SMOOTH)
			{
				dtVcopy(&m_smoothPath[m_nsmoothPath * 3], startPos.data());
				m_nsmoothPath++;
				// Hack to make the dotted path not visible during off-mesh connection.
				if (m_nsmoothPath & 1)
				{
					dtVcopy(&m_smoothPath[m_nsmoothPath * 3], startPos.data());
					m_nsmoothPath++;
				}
			}
			// Move position at the other side of the off-mesh link.
			m_iterPos = endPos;

			float eh{};

			m_navQuery->getPolyHeight(m_pathIterPolys[0], m_iterPos.data(), &eh);
			m_iterPos[1] = eh;
		}
	}

	// Store results.
	if (m_nsmoothPath < MAX_SMOOTH)
	{
		dtVcopy(&m_smoothPath[m_nsmoothPath * 3], m_iterPos.data());
		m_nsmoothPath++;
	}
}

void NavMeshTesterTool::handleUpdate(const float /*dt*/)
{
	// Sliceモードのみ
	if (m_toolMode == ToolMode::TOOLMODE_PATHFIND_SLICED)
	{
		// パスの探索順に探索する
		if (dtStatusInProgress(m_pathFindStatus))
		{
			m_pathFindStatus = m_navQuery->updateSlicedFindPath(1, 0);
		}

		// パスの探索を完了
		if (dtStatusSucceed(m_pathFindStatus))
		{
			m_navQuery->finalizeSlicedFindPath(m_polys.data(), &m_npolys, MAX_POLYS);
			m_nstraightPath = 0;

			if (m_npolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				// 部分パスの場合、終点が最後のポリゴンに固定されていることを確認してください。
				ArrayF epos{ m_epos };

				if (m_polys[m_npolys - 1] != m_endRef)
					m_navQuery->closestPointOnPoly(m_polys[m_npolys - 1], m_epos.data(), epos.data(), 0);

				m_navQuery->findStraightPath(m_spos, epos, m_polys.data(), m_npolys,
					m_straightPath.data(), m_straightPathFlags.data(),
					m_straightPathPolys.data(), &m_nstraightPath, MAX_POLYS, DT_STRAIGHTPATH_ALL_CROSSINGS);
			}

			m_pathFindStatus = DT_FAILURE;
		}
	}
}

void NavMeshTesterTool::reset()
{
	m_startRef = 0;
	m_endRef = 0;
	m_npolys = 0;
	m_nstraightPath = 0;
	m_nsmoothPath = 0;
	m_hitPos.fill(0.f);
	m_hitNormal.fill(0.f);
	m_distanceToWall = 0;
}

void NavMeshTesterTool::recalc()
{
	// ナビメッシュが存在しない
	if (!m_navMesh) return;

	// スタート地点が設定されている
	if (m_sposSet)
		m_navQuery->findNearestPoly(m_spos.data(), m_polyPickExt.data(), &m_filter, &m_startRef, 0);
	else
		m_startRef = 0;

	// ゴール地点が設定されている
	if (m_eposSet)
		m_navQuery->findNearestPoly(m_epos.data(), m_polyPickExt.data(), &m_filter, &m_endRef, 0);
	else
		m_endRef = 0;

	m_pathFindStatus = DT_FAILURE;

	if (m_toolMode == ToolMode::TOOLMODE_PATHFIND_FOLLOW)
	{
		m_pathIterNum = 0;
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			printf("pi  %f %f %f  %f %f %f  0x%x 0x%x\n",
				m_spos[0], m_spos[1], m_spos[2], m_epos[0], m_epos[1], m_epos[2],
				m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif

			m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, &m_polys, &m_npolys, MAX_POLYS);

			m_nsmoothPath = 0;

			if (m_npolys)
			{
				// Iterate over the path to find smooth path on the detail mesh surface.
				std::array<dtPolyRef, MAX_POLYS> polys{};

				{
					size_t i{};
					std::for_each_n(polys.begin(), m_npolys, [&](dtPolyRef& poly) { poly = m_polys[i++]; });
				}
				int npolys = m_npolys;

				ArrayF iterPos{}, targetPos{};
				m_navQuery->closestPointOnPoly(m_startRef, m_spos.data(), iterPos.data(), 0);
				m_navQuery->closestPointOnPoly(polys[npolys - 1], m_epos.data(), targetPos.data(), 0);

				constexpr float STEP_SIZE = 0.5f;
				constexpr float SLOP = 0.01f;

				m_nsmoothPath = 0;

				dtVcopy(&m_smoothPath[m_nsmoothPath * 3], iterPos.data());
				m_nsmoothPath++;

				// Move towards target a small advancement at a time until target reached or
				// when ran out of memory to store the path.
				while (npolys && m_nsmoothPath < MAX_SMOOTH)
				{
					// Find location to steer towards.
					ArrayF steerPos{};
					uint8_t steerPosFlag;
					dtPolyRef steerPosRef;

					if (!getSteerTarget(m_navQuery, iterPos, targetPos, SLOP, polys, npolys, &steerPos,
						steerPosFlag, steerPosRef))
						break;

					bool endOfPath = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
					bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;

					// Find movement delta.
					ArrayF delta{ steerPos - iterPos };
					float len{};

					len = dtMathSqrtf(dtVdot(delta, delta));
					// If the steer target is end of path or off-mesh link, do not move past the location.
					if ((endOfPath || offMeshConnection) && len < STEP_SIZE)
						len = 1;
					else
						len = STEP_SIZE / len;
					ArrayF moveTgt{};
					dtVmad(&moveTgt, iterPos, delta, len);

					// Move
					ArrayF result{};
					std::array<dtPolyRef, 16> visited{};
					int nvisited{};
					float h{};

					m_navQuery->moveAlongSurface(polys[0], iterPos.data(), moveTgt.data(), &m_filter,
						result.data(), visited.data(), &nvisited, 16);

					npolys = fixupCorridor(&polys, npolys, MAX_POLYS, visited, nvisited);
					npolys = fixupShortcuts(&polys, npolys, m_navQuery);

					m_navQuery->getPolyHeight(polys[0], result.data(), &h);
					result[1] = h;
					iterPos = result;

					// Handle end of path and off-mesh links when close enough.
					if (endOfPath && inRange(iterPos, steerPos, SLOP, 1.f))
					{
						// Reached end of path.
						iterPos = targetPos;

						if (m_nsmoothPath < MAX_SMOOTH)
						{
							dtVcopy(&m_smoothPath[m_nsmoothPath * 3], iterPos.data());
							m_nsmoothPath++;
						}
						break;
					}
					else if (offMeshConnection && inRange(iterPos, steerPos, SLOP, 1.f))
					{
						// Reached off-mesh connection.
						ArrayF startPos, endPos;

						// Advance the path up to and over the off-mesh connection.
						dtPolyRef prevRef{}, polyRef = polys[0];
						int npos{};

						while (npos < npolys && polyRef != steerPosRef)
						{
							prevRef = polyRef;
							polyRef = polys[npos];
							npos++;
						}

						for (int i = npos; i < npolys; ++i)
							polys[i - npos] = polys[i];

						npolys -= npos;

						// Handle the connection.
						dtStatus status = m_navMesh->getOffMeshConnectionPolyEndPoints(prevRef, polyRef, startPos.data(), endPos.data());

						if (dtStatusSucceed(status))
						{
							if (m_nsmoothPath < MAX_SMOOTH)
							{
								dtVcopy(&m_smoothPath[m_nsmoothPath * 3], startPos.data());
								m_nsmoothPath++;
								// Hack to make the dotted path not visible during off-mesh connection.
								if (m_nsmoothPath & 1)
								{
									dtVcopy(&m_smoothPath[m_nsmoothPath * 3], startPos.data());
									m_nsmoothPath++;
								}
							}
							// Move position at the other side of the off-mesh link.
							iterPos = endPos;

							float eh{};
							m_navQuery->getPolyHeight(polys[0], iterPos.data(), &eh);
							iterPos[1] = eh;
						}
					}

					// Store results.
					if (m_nsmoothPath < MAX_SMOOTH)
					{
						dtVcopy(&m_smoothPath[m_nsmoothPath * 3], iterPos.data());
						m_nsmoothPath++;
					}
				}
			}
		}
		else
		{
			m_npolys = 0;
			m_nsmoothPath = 0;
		}
	}
	else if (m_toolMode == ToolMode::TOOLMODE_PATHFIND_STRAIGHT)
	{
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
				m_spos[0], m_spos[1], m_spos[2], m_epos[0], m_epos[1], m_epos[2],
				m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			m_navQuery->findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, &m_polys, &m_npolys,
				MAX_POLYS);

			m_nstraightPath = 0;

			if (m_npolys)
			{
				// In case of partial path, make sure the end point is clamped to the last polygon.
				ArrayF epos{ m_epos };

				if (m_polys[m_npolys - 1] != m_endRef)
					m_navQuery->closestPointOnPoly(m_polys[m_npolys - 1], m_epos.data(), epos.data(), 0);

				m_navQuery->findStraightPath(m_spos, epos, m_polys.data(), m_npolys,
					m_straightPath.data(), m_straightPathFlags.data(),
					m_straightPathPolys.data(), &m_nstraightPath, MAX_POLYS, m_straightPathOptions);
			}
		}
		else
		{
			m_npolys = 0;
			m_nstraightPath = 0;
		}
	}
	else if (m_toolMode == ToolMode::TOOLMODE_PATHFIND_SLICED)
	{
		if (m_sposSet && m_eposSet && m_startRef && m_endRef)
		{
#ifdef DUMP_REQS
			printf("ps  %f %f %f  %f %f %f  0x%x 0x%x\n",
				m_spos[0], m_spos[1], m_spos[2], m_epos[0], m_epos[1], m_epos[2],
				m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			m_npolys = 0;
			m_nstraightPath = 0;

			m_pathFindStatus = m_navQuery->initSlicedFindPath(m_startRef, m_endRef, m_spos.data(), m_epos.data(), &m_filter, DT_FINDPATH_ANY_ANGLE);
		}
		else
		{
			m_npolys = 0;
			m_nstraightPath = 0;
		}
	}
	else if (m_toolMode == ToolMode::TOOLMODE_RAYCAST)
	{
		m_nstraightPath = 0;
		if (m_sposSet && m_eposSet && m_startRef)
		{
#ifdef DUMP_REQS
			printf("rc  %f %f %f  %f %f %f  0x%x 0x%x\n",
				m_spos[0], m_spos[1], m_spos[2], m_epos[0], m_epos[1], m_epos[2],
				m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			float t{};

			m_npolys = 0;
			m_nstraightPath = 2;
			m_straightPath[0] = m_spos[0];
			m_straightPath[1] = m_spos[1];
			m_straightPath[2] = m_spos[2];

			m_navQuery->raycast(m_startRef, m_spos.data(), m_epos.data(), &m_filter, &t, m_hitNormal.data(), m_polys.data(), &m_npolys, MAX_POLYS);

			if (t > 1)
			{
				// No hit
				m_hitPos = m_epos;
				m_hitResult = false;
			}
			else
			{
				// Hit
				dtVlerp(&m_hitPos, m_spos, m_epos, t);
				m_hitResult = true;
			}
			// Adjust height.
			if (m_npolys > 0)
			{
				float h{};

				m_navQuery->getPolyHeight(m_polys[m_npolys - 1], m_hitPos.data(), &h);
				m_hitPos[1] = h;
			}

			dtVcopy(&m_straightPath[3], m_hitPos.data());
		}
	}
	else if (m_toolMode == ToolMode::TOOLMODE_DISTANCE_TO_WALL)
	{
		m_distanceToWall = 0;
		if (m_sposSet && m_startRef)
		{
#ifdef DUMP_REQS
			printf("dw  %f %f %f  %f  0x%x 0x%x\n",
				m_spos[0], m_spos[1], m_spos[2], 100.0f,
				m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			m_distanceToWall = 0.0f;
			m_navQuery->findDistanceToWall(m_startRef, m_spos.data(), 100.0f, &m_filter, &m_distanceToWall, m_hitPos.data(), m_hitNormal.data());
		}
	}
	else if (m_toolMode == ToolMode::TOOLMODE_FIND_POLYS_IN_CIRCLE)
	{
		if (m_sposSet && m_startRef && m_eposSet)
		{
			const float dx = m_epos[0] - m_spos[0];
			const float dz = m_epos[2] - m_spos[2];
			float dist = sqrtf(dx * dx + dz * dz);
#ifdef DUMP_REQS
			printf("fpc  %f %f %f  %f  0x%x 0x%x\n",
				m_spos[0], m_spos[1], m_spos[2], dist,
				m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			m_navQuery->findPolysAroundCircle(m_startRef, m_spos.data(), dist, &m_filter,
				m_polys.data(), m_parent.data(), 0, &m_npolys, MAX_POLYS);
		}
	}
	else if (m_toolMode == ToolMode::TOOLMODE_FIND_POLYS_IN_SHAPE)
	{
		if (m_sposSet && m_startRef && m_eposSet)
		{
			const float nx = (m_epos[2] - m_spos[2]) * 0.25f;
			const float nz = -(m_epos[0] - m_spos[0]) * 0.25f;
			const float agentHeight = m_sample ? m_sample->getAgentHeight() : 0;

			m_queryPoly[0] = m_spos[0] + nx * 1.2f;
			m_queryPoly[1] = m_spos[1] + agentHeight / 2;
			m_queryPoly[2] = m_spos[2] + nz * 1.2f;

			m_queryPoly[3] = m_spos[0] - nx * 1.3f;
			m_queryPoly[4] = m_spos[1] + agentHeight / 2;
			m_queryPoly[5] = m_spos[2] - nz * 1.3f;

			m_queryPoly[6] = m_epos[0] - nx * 0.8f;
			m_queryPoly[7] = m_epos[1] + agentHeight / 2;
			m_queryPoly[8] = m_epos[2] - nz * 0.8f;

			m_queryPoly[9] = m_epos[0] + nx;
			m_queryPoly[10] = m_epos[1] + agentHeight / 2;
			m_queryPoly[11] = m_epos[2] + nz;

#ifdef DUMP_REQS
			printf("fpp  %f %f %f  %f %f %f  %f %f %f  %f %f %f  0x%x 0x%x\n",
				m_queryPoly[0], m_queryPoly[1], m_queryPoly[2],
				m_queryPoly[3], m_queryPoly[4], m_queryPoly[5],
				m_queryPoly[6], m_queryPoly[7], m_queryPoly[8],
				m_queryPoly[9], m_queryPoly[10], m_queryPoly[11],
				m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			m_navQuery->findPolysAroundShape(m_startRef, m_queryPoly.data(), 4, &m_filter,
				m_polys.data(), m_parent.data(), 0, &m_npolys, MAX_POLYS);
		}
	}
	else if (m_toolMode == ToolMode::TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD)
	{
		if (m_sposSet && m_startRef)
		{
#ifdef DUMP_REQS
			printf("fln  %f %f %f  %f  0x%x 0x%x\n",
				m_spos[0], m_spos[1], m_spos[2], m_neighbourhoodRadius,
				m_filter.getIncludeFlags(), m_filter.getExcludeFlags());
#endif
			m_navQuery->findLocalNeighbourhood(m_startRef, m_spos.data(), m_neighbourhoodRadius, &m_filter,
				m_polys.data(), m_parent.data(), &m_npolys, MAX_POLYS);
		}
	}
}

void NavMeshTesterTool::handleRender()
{
	duDebugDraw& dd = m_sample->getDebugDraw();

	constexpr uint32_t startCol = duRGBA(128, 25, 0, 192);
	constexpr uint32_t endCol = duRGBA(51, 102, 0, 129);
	constexpr uint32_t pathCol = duRGBA(0, 0, 0, 64);

	const float agentRadius = m_sample->getAgentRadius();
	const float agentHeight = m_sample->getAgentHeight();
	const float agentClimb = m_sample->getAgentClimb();

	dd.depthMask(false);

	if (m_sposSet) drawAgent(m_spos.data(), agentRadius, agentHeight, agentClimb, startCol);

	if (m_eposSet) drawAgent(m_epos.data(), agentRadius, agentHeight, agentClimb, endCol);

	dd.depthMask(true);

	if (!m_navMesh)	return;

	switch (m_toolMode)
	{
		case ToolMode::TOOLMODE_PATHFIND_FOLLOW:
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_endRef, endCol);

			if (m_npolys)
			{
				for (int i = 0; i < m_npolys; ++i)
				{
					if (m_polys[i] == m_startRef || m_polys[i] == m_endRef)
						continue;
					duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
				}
			}

			if (m_nsmoothPath)
			{
				dd.depthMask(false);
				constexpr uint32_t spathCol = duRGBA(0, 0, 0, 220);
				dd.begin(DU_DRAW_LINES, 3.0f);
				for (int i = 0; i < m_nsmoothPath; ++i)
					dd.vertex(m_smoothPath[i * 3], m_smoothPath[i * 3 + 1] + 0.1f, m_smoothPath[i * 3 + 2], spathCol);
				dd.end();
				dd.depthMask(true);
			}

			if (m_pathIterNum)
			{
				duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_pathIterPolys[0], duRGBA(255, 255, 255, 128));

				dd.depthMask(false);
				dd.begin(DU_DRAW_LINES, 1.f);

				constexpr uint32_t prevCol = duRGBA(255, 192, 0, 220);
				constexpr uint32_t curCol = duRGBA(255, 255, 255, 220);
				constexpr uint32_t steerCol = duRGBA(0, 192, 255, 220);

				dd.vertex(m_prevIterPos[0], m_prevIterPos[1] - 0.3f, m_prevIterPos[2], prevCol);
				dd.vertex(m_prevIterPos[0], m_prevIterPos[1] + 0.3f, m_prevIterPos[2], prevCol);

				dd.vertex(m_iterPos[0], m_iterPos[1] - 0.3f, m_iterPos[2], curCol);
				dd.vertex(m_iterPos[0], m_iterPos[1] + 0.3f, m_iterPos[2], curCol);

				dd.vertex(m_prevIterPos[0], m_prevIterPos[1] + 0.3f, m_prevIterPos[2], prevCol);
				dd.vertex(m_iterPos[0], m_iterPos[1] + 0.3f, m_iterPos[2], prevCol);

				dd.vertex(m_prevIterPos[0], m_prevIterPos[1] + 0.3f, m_prevIterPos[2], steerCol);
				dd.vertex(m_steerPos[0], m_steerPos[1] + 0.3f, m_steerPos[2], steerCol);

				for (int i = 0; i < m_steerPointCount - 1; ++i)
				{
					dd.vertex(m_steerPoints[i * 3 + 0], m_steerPoints[i * 3 + 1] + 0.2f, m_steerPoints[i * 3 + 2], duDarkenCol(steerCol));
					dd.vertex(m_steerPoints[(i + 1) * 3 + 0], m_steerPoints[(i + 1) * 3 + 1] + 0.2f, m_steerPoints[(i + 1) * 3 + 2], duDarkenCol(steerCol));
				}

				dd.end();
				dd.depthMask(true);
			}

			break;
		}
		case ToolMode::TOOLMODE_PATHFIND_STRAIGHT:
		case ToolMode::TOOLMODE_PATHFIND_SLICED:
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_endRef, endCol);

			if (m_npolys)
			{
				for (int i = 0; i < m_npolys; ++i)
				{
					if (m_polys[i] == m_startRef || m_polys[i] == m_endRef)
						continue;
					duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
				}
			}

			if (m_nstraightPath)
			{
				dd.depthMask(false);

				constexpr uint32_t spathCol = duRGBA(64, 16, 0, 220);
				constexpr uint32_t offMeshCol = duRGBA(128, 96, 0, 220);

				dd.begin(DU_DRAW_LINES, 2.0f);

				for (int i = 0; i < m_nstraightPath - 1; ++i)
				{
					uint32_t col{};

					if (m_straightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
						col = offMeshCol;
					else
						col = spathCol;

					dd.vertex(m_straightPath[i * 3], m_straightPath[i * 3 + 1] + 0.4f, m_straightPath[i * 3 + 2], col);
					dd.vertex(m_straightPath[(i + 1) * 3], m_straightPath[(i + 1) * 3 + 1] + 0.4f, m_straightPath[(i + 1) * 3 + 2], col);
				}

				dd.end();
				dd.begin(DU_DRAW_POINTS, 6.0f);

				for (int i = 0; i < m_nstraightPath; ++i)
				{
					uint32_t col{};

					if (m_straightPathFlags[i] & DT_STRAIGHTPATH_START)
						col = startCol;
					else if (m_straightPathFlags[i] & DT_STRAIGHTPATH_END)
						col = endCol;
					else if (m_straightPathFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
						col = offMeshCol;
					else
						col = spathCol;

					dd.vertex(m_straightPath[i * 3], m_straightPath[i * 3 + 1] + 0.4f, m_straightPath[i * 3 + 2], col);
				}

				dd.end();
				dd.depthMask(true);
			}

			break;
		}
		case ToolMode::TOOLMODE_RAYCAST:
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);

			if (m_nstraightPath)
			{
				for (int i = 1; i < m_npolys; ++i)
					duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);

				dd.depthMask(false);
				const uint32_t spathCol = m_hitResult ? duRGBA(64, 16, 0, 220) : duRGBA(240, 240, 240, 220);
				dd.begin(DU_DRAW_LINES, 2.0f);
				for (int i = 0; i < m_nstraightPath - 1; ++i)
				{
					dd.vertex(m_straightPath[i * 3], m_straightPath[i * 3 + 1] + 0.4f, m_straightPath[i * 3 + 2], spathCol);
					dd.vertex(m_straightPath[(i + 1) * 3], m_straightPath[(i + 1) * 3 + 1] + 0.4f, m_straightPath[(i + 1) * 3 + 2], spathCol);
				}
				dd.end();
				dd.begin(DU_DRAW_POINTS, 4.0f);

				for (int i = 0; i < m_nstraightPath; ++i)
				{
					dd.vertex(m_straightPath[i * 3], m_straightPath[i * 3 + 1] + 0.4f, m_straightPath[i * 3 + 2], spathCol);
				}

				dd.end();

				if (m_hitResult)
				{
					constexpr uint32_t hitCol = duRGBA(0, 0, 0, 128);

					dd.begin(DU_DRAW_LINES, 2.0f);
					dd.vertex(m_hitPos[0], m_hitPos[1] + 0.4f, m_hitPos[2], hitCol);
					dd.vertex(m_hitPos[0] + m_hitNormal[0] * agentRadius,
						m_hitPos[1] + 0.4f + m_hitNormal[1] * agentRadius,
						m_hitPos[2] + m_hitNormal[2] * agentRadius, hitCol);
					dd.end();
				}
				dd.depthMask(true);
			}
			break;
		}
		case ToolMode::TOOLMODE_DISTANCE_TO_WALL:
		{
			duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_startRef, startCol);
			dd.depthMask(false);
			duDebugDrawCircle(&dd, m_spos[0], m_spos[1] + agentHeight / 2, m_spos[2], m_distanceToWall, duRGBA(64, 16, 0, 220), 2.0f);
			dd.begin(DU_DRAW_LINES, 3.0f);
			dd.vertex(m_hitPos[0], m_hitPos[1] + 0.02f, m_hitPos[2], duRGBA(0, 0, 0, 192));
			dd.vertex(m_hitPos[0], m_hitPos[1] + agentHeight, m_hitPos[2], duRGBA(0, 0, 0, 192));
			dd.end();
			dd.depthMask(true);

			break;
		}
		case ToolMode::TOOLMODE_FIND_POLYS_IN_CIRCLE:
		{
			for (int i = 0; i < m_npolys; ++i)
			{
				duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
				dd.depthMask(false);

				if (m_parent[i])
				{
					ArrayF p0{}, p1{};

					dd.depthMask(false);
					getPolyCenter(m_navMesh, m_parent[i], &p0);
					getPolyCenter(m_navMesh, m_polys[i], &p1);
					duDebugDrawArc(&dd, p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0, 0, 0, 128), 2.0f);
					dd.depthMask(true);
				}

				dd.depthMask(true);
			}

			if (m_sposSet && m_eposSet)
			{
				dd.depthMask(false);

				const float dx = m_epos[0] - m_spos[0];
				const float dz = m_epos[2] - m_spos[2];
				const float dist = sqrtf(dx * dx + dz * dz);

				duDebugDrawCircle(&dd, m_spos[0], m_spos[1] + agentHeight / 2, m_spos[2], dist, duRGBA(64, 16, 0, 220), 2.0f);
				dd.depthMask(true);
			}

			break;
		}
		case ToolMode::TOOLMODE_FIND_POLYS_IN_SHAPE:
		{
			for (int i = 0; i < m_npolys; ++i)
			{
				duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
				dd.depthMask(false);

				if (m_parent[i])
				{
					ArrayF p0{}, p1{};

					dd.depthMask(false);
					getPolyCenter(m_navMesh, m_parent[i], &p0);
					getPolyCenter(m_navMesh, m_polys[i], &p1);
					duDebugDrawArc(&dd, p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0, 0, 0, 128), 2.0f);
					dd.depthMask(true);
				}

				dd.depthMask(true);
			}

			if (m_sposSet && m_eposSet)
			{
				dd.depthMask(false);

				constexpr uint32_t col = duRGBA(64, 16, 0, 220);

				dd.begin(DU_DRAW_LINES, 2.0f);

				for (int i = 0, j = 3; i < 4; j = i++)
				{
					const float* p0 = &m_queryPoly[j * 3];
					const float* p1 = &m_queryPoly[i * 3];

					dd.vertex(p0, col);
					dd.vertex(p1, col);
				}

				dd.end();
				dd.depthMask(true);
			}

			break;
		}
		case ToolMode::TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD:
		{
			for (int i = 0; i < m_npolys; ++i)
			{
				duDebugDrawNavMeshPoly(&dd, *m_navMesh, m_polys[i], pathCol);
				dd.depthMask(false);

				if (m_parent[i])
				{
					ArrayF p0{}, p1{};

					dd.depthMask(false);
					getPolyCenter(m_navMesh, m_parent[i], &p0);
					getPolyCenter(m_navMesh, m_polys[i], &p1);
					duDebugDrawArc(&dd, p0[0], p0[1], p0[2], p1[0], p1[1], p1[2], 0.25f, 0.0f, 0.4f, duRGBA(0, 0, 0, 128), 2.0f);
					dd.depthMask(true);
				}

				constexpr int MAX_SEGS = DT_VERTS_PER_POLYGON * 4;

				std::array<float, MAX_SEGS * 6> segs{};
				std::array<dtPolyRef, MAX_SEGS> refs{};
				int nsegs{};

				m_navQuery->getPolyWallSegments(m_polys[i], &m_filter, segs.data(), refs.data(), &nsegs, MAX_SEGS);
				dd.begin(DU_DRAW_LINES, 2.0f);

				for (int j = 0; j < nsegs; ++j)
				{
					const float* s = &segs[j * 6];

					// Skip too distant segments.
					float tseg;
					float distSqr = dtDistancePtSegSqr2D(m_spos.data(), s, s + 3, tseg);

					if (distSqr > dtSqr(m_neighbourhoodRadius))
						continue;

					ArrayF delta{}, norm{}, p0{}, p1{};

					dtVsub(delta.data(), s + 3, s);
					dtVmad(p0.data(), s, delta.data(), 0.5f);

					norm[0] = delta[2];
					norm[1] = 0;
					norm[2] = -delta[0];

					dtVnormalize(&norm);
					dtVmad(&p1, p0, norm, agentRadius * 0.5f);

					// Skip backfacing segments.
					if (refs[j])
					{
						constexpr uint32_t col = duRGBA(255, 255, 255, 32);

						dd.vertex(s[0], s[1] + agentClimb, s[2], col);
						dd.vertex(s[3], s[4] + agentClimb, s[5], col);
					}
					else
					{
						uint32_t col = duRGBA(192, 32, 16, 192);

						if (dtTriArea2D(m_spos.data(), s, s + 3) < 0.0f)
							col = duRGBA(96, 32, 16, 192);

						dd.vertex(p0[0], p0[1] + agentClimb, p0[2], col);
						dd.vertex(p1[0], p1[1] + agentClimb, p1[2], col);

						dd.vertex(s[0], s[1] + agentClimb, s[2], col);
						dd.vertex(s[3], s[4] + agentClimb, s[5], col);
					}
				}
				dd.end();

				dd.depthMask(true);
			}

			if (m_sposSet)
			{
				dd.depthMask(false);
				duDebugDrawCircle(&dd, m_spos[0], m_spos[1] + agentHeight / 2, m_spos[2], m_neighbourhoodRadius, duRGBA(64, 16, 0, 220), 2.0f);
				dd.depthMask(true);
			}

			break;
		}
	}

	if (m_nrandPoints > 0)
	{
		dd.begin(DU_DRAW_POINTS, 6.0f);

		for (int i = 0; i < m_nrandPoints; i++)
		{
			const float* p = &m_randPoints[i * 3];
			dd.vertex(p[0], p[1] + 0.1f, p[2], duRGBA(220, 32, 16, 192));
		}
		dd.end();

		if (m_randPointsInCircle && m_sposSet)
		{
			duDebugDrawCircle(&dd, m_spos[0], m_spos[1] + agentHeight / 2, m_spos[2], m_randomRadius, duRGBA(64, 16, 0, 220), 2.0f);
		}
	}
}

void NavMeshTesterTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;

	// Draw start and end point labels
	if (m_sposSet && gluProject((GLdouble)m_spos[0], (GLdouble)m_spos[1], (GLdouble)m_spos[2],
		model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y - 25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0, 0, 0, 220));
	}
	if (m_eposSet && gluProject((GLdouble)m_epos[0], (GLdouble)m_epos[1], (GLdouble)m_epos[2],
		model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y - 25), IMGUI_ALIGN_CENTER, "End", imguiRGBA(0, 0, 0, 220));
	}

	// Tool help
	const int h = view[3];

	imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB+SHIFT: Set start location  LMB: Set end location", imguiRGBA(255, 255, 255, 192));
}

void NavMeshTesterTool::drawAgent(const float* pos, float r, float h, float c, const uint32_t col)
{
	duDebugDraw& dd = m_sample->getDebugDraw();

	dd.depthMask(false);

	// Agent dimensions.
	duDebugDrawCylinderWire(&dd, pos[0] - r, pos[1] + 0.02f, pos[2] - r, pos[0] + r, pos[1] + h, pos[2] + r, col, 2.0f);

	duDebugDrawCircle(&dd, pos[0], pos[1] + c, pos[2], r, duRGBA(0, 0, 0, 64), 1.f);

	constexpr uint32_t colb = duRGBA(0, 0, 0, 196);

	dd.begin(DU_DRAW_LINES);
	dd.vertex(pos[0], pos[1] - c, pos[2], colb);
	dd.vertex(pos[0], pos[1] + c, pos[2], colb);
	dd.vertex(pos[0] - r / 2, pos[1] + 0.02f, pos[2], colb);
	dd.vertex(pos[0] + r / 2, pos[1] + 0.02f, pos[2], colb);
	dd.vertex(pos[0], pos[1] + 0.02f, pos[2] - r / 2, colb);
	dd.vertex(pos[0], pos[1] + 0.02f, pos[2] + r / 2, colb);
	dd.end();

	dd.depthMask(true);
}
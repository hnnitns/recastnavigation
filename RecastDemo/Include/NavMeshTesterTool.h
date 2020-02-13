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
	Sample* m_sample;

	dtNavMesh* m_navMesh;
	dtNavMeshQuery* m_navQuery;

	dtQueryFilter m_filter;

	dtStatus m_pathFindStatus;

	enum ToolMode
	{
		TOOLMODE_PATHFIND_FOLLOW,
		TOOLMODE_PATHFIND_STRAIGHT,
		TOOLMODE_PATHFIND_SLICED,
		TOOLMODE_RAYCAST,
		TOOLMODE_DISTANCE_TO_WALL,
		TOOLMODE_FIND_POLYS_IN_CIRCLE,
		TOOLMODE_FIND_POLYS_IN_SHAPE,
		TOOLMODE_FIND_LOCAL_NEIGHBOURHOOD,
	};

	ToolMode m_toolMode;

	int m_straightPathOptions;

	static constexpr int MAX_SMOOTH = 2048;

	dtPolyRef m_startRef;
	dtPolyRef m_endRef;
	std::array<dtPolyRef, MAX_POLYS> m_polys;
	std::array<dtPolyRef, MAX_POLYS> m_parent;
	int m_npolys;
	std::array<float, MAX_POLYS * 3> m_straightPath;
	std::array<uint8_t, MAX_POLYS> m_straightPathFlags;
	std::array<dtPolyRef, MAX_POLYS> m_straightPathPolys;
	int m_nstraightPath;
	std::array<float, 3> m_polyPickExt;
	std::array<float, MAX_SMOOTH * 3> m_smoothPath;
	int m_nsmoothPath;
	std::array<float, 4 * 3> m_queryPoly;

	static constexpr int MAX_RAND_POINTS = 64;
	float m_randPoints[MAX_RAND_POINTS * 3];
	int m_nrandPoints;
	bool m_randPointsInCircle;

	float m_spos[3];
	float m_epos[3];
	float m_hitPos[3];
	float m_hitNormal[3];
	bool m_hitResult;
	float m_distanceToWall;
	float m_neighbourhoodRadius;
	float m_randomRadius;
	bool m_sposSet;
	bool m_eposSet;

	int m_pathIterNum;
	dtPolyRef m_pathIterPolys[MAX_POLYS];
	int m_pathIterPolyCount;
	float m_prevIterPos[3], m_iterPos[3], m_steerPos[3], m_targetPos[3];

	static const int MAX_STEER_POINTS = 10;
	float m_steerPoints[MAX_STEER_POINTS * 3];
	int m_steerPointCount;

public:
	NavMeshTesterTool();

	virtual int type() { return TOOL_NAVMESH_TESTER; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* s, const float* p, bool shift);
	// �p�X�̒ǐՏ������s��
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);

	// �o�H�T���Ȃǂ��s��
	void recalc();
	void drawAgent(const float* pos, float r, float h, float c, const uint32_t col);
};

#endif // NAVMESHTESTERTOOL_H
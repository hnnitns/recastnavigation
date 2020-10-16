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
#include <cstring>
#include <cfloat>
#include <cstdlib>
#include <new>
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAssert.h"
#include "DetourAlloc.h"

dtCrowd* dtAllocCrowd()
{
	void* mem = dtAlloc(sizeof(dtCrowd), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtCrowd;
}

void dtFreeCrowd(dtCrowd* ptr)
{
	if (!ptr) return;
	ptr->~dtCrowd();
	dtFree(ptr);
}

namespace
{
	const int MAX_ITERS_PER_UPDATE = 100;

	const int MAX_PATHQUEUE_NODES = 4096;
	const int MAX_COMMON_NODES = 512;

	inline float tween(const float t, const float t0, const float t1)
	{
		return dtClamp((t - t0) / (t1 - t0), 0.0f, 1.f);
	}

	void integrate(dtCrowdAgent* ag, const float dt)
	{
		// Fake dynamic constraint.
		const float maxDelta = ag->params.maxAcceleration * dt;
		float dv[3];
		dtVsub(dv, ag->nvel, ag->vel);
		float ds = dtVlen(dv);
		if (ds > maxDelta)
			dtVscale(dv, dv, maxDelta / ds);
		dtVadd(ag->vel, ag->vel, dv);

		// Integrate
		if (dtVlen(ag->vel) > 0.0001f)
			dtVmad(ag->npos, ag->npos, ag->vel, dt);
		else
			dtVset(ag->vel, 0, 0, 0);
	}

	bool overOffmeshConnection(const dtCrowdAgent* ag, const float radius)
	{
		if (!ag->ncorners)
			return false;

		const bool offMeshConnection = (ag->cornerFlags[ag->ncorners - 1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
		if (offMeshConnection)
		{
			const float distSq = dtVdist2DSqr(ag->npos, &ag->cornerVerts[(ag->ncorners - 1) * 3]);
			if (distSq < radius * radius)
				return true;
		}

		return false;
	}

	float getDistanceToGoal(const dtCrowdAgent* ag, const float range)
	{
		if (!ag->ncorners)
			return range;

		const bool endOfPath = (ag->cornerFlags[ag->ncorners - 1] & DT_STRAIGHTPATH_END) ? true : false;
		if (endOfPath)
			return dtMin(dtVdist2D(ag->npos, &ag->cornerVerts[(ag->ncorners - 1) * 3]), range);

		return range;
	}

	void calcSmoothSteerDirection(const dtCrowdAgent* ag, float* dir)
	{
		if (!ag->ncorners)
		{
			dtVset(dir, 0, 0, 0);
			return;
		}

		const int ip0 = 0;
		const int ip1 = dtMin(1, ag->ncorners - 1);
		const float* p0 = &ag->cornerVerts[ip0 * 3];
		const float* p1 = &ag->cornerVerts[ip1 * 3];

		float dir0[3], dir1[3];
		dtVsub(dir0, p0, ag->npos);
		dtVsub(dir1, p1, ag->npos);
		dir0[1] = 0;
		dir1[1] = 0;

		float len0 = dtVlen(dir0);
		float len1 = dtVlen(dir1);
		if (len1 > 0.001f)
			dtVscale(dir1, dir1, 1.f / len1);

		dir[0] = dir0[0] - dir1[0] * len0 * 0.5f;
		dir[1] = 0;
		dir[2] = dir0[2] - dir1[2] * len0 * 0.5f;

		dtVnormalize(dir);
	}

	void calcStraightSteerDirection(const dtCrowdAgent* ag, float* dir)
	{
		if (!ag->ncorners)
		{
			dtVset(dir, 0, 0, 0);
			return;
		}
		dtVsub(dir, &ag->cornerVerts[0], ag->npos);
		dir[1] = 0;
		dtVnormalize(dir);
	}

	int addNeighbour(const int idx, const float dist,
		dtCrowdNeighbour* neis, const int nneis, const int maxNeis)
	{
		// Insert neighbour based on the distance.
		dtCrowdNeighbour* nei = 0;
		if (!nneis)
		{
			nei = &neis[nneis];
		}
		else if (dist >= neis[nneis - 1].dist)
		{
			if (nneis >= maxNeis)
				return nneis;
			nei = &neis[nneis];
		}
		else
		{
			int i;
			for (i = 0; i < nneis; ++i)
				if (dist <= neis[i].dist)
					break;

			const int tgt = i + 1;
			const int n = dtMin(nneis - i, maxNeis - tgt);

			dtAssert(tgt + n <= maxNeis);

			if (n > 0)
				memmove(&neis[tgt], &neis[i], sizeof(dtCrowdNeighbour) * n);
			nei = &neis[i];
		}

		memset(nei, 0, sizeof(dtCrowdNeighbour));

		nei->idx = idx;
		nei->dist = dist;

		return dtMin(nneis + 1, maxNeis);
	}

	int getNeighbours(const float* pos, const float height, const float range,
		const dtCrowdAgent* skip, dtCrowdNeighbour* result, const int maxResult,
		dtCrowdAgent** agents, const int /*nagents*/, dtProximityGrid* grid)
	{
		int n = 0;

		constexpr int MAX_NEIS = 32;
		unsigned short ids[MAX_NEIS];
		int nids = grid->queryItems(pos[0] - range, pos[2] - range,
			pos[0] + range, pos[2] + range,
			ids, MAX_NEIS);

		for (int i = 0; i < nids; ++i)
		{
			const dtCrowdAgent* ag = agents[ids[i]];

			if (ag == skip) continue;

			// Check for overlap.
			float diff[3];
			dtVsub(diff, pos, ag->npos);
			if (dtMathFabsf(diff[1]) >= (height + ag->params.height) / 2.0f)
				continue;
			diff[1] = 0;
			const float distSqr = dtVlenSqr(diff);
			if (distSqr > dtSqr(range))
				continue;

			n = addNeighbour(ids[i], distSqr, result, n, maxResult);
		}
		return n;
	}

	int addToOptQueue(dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents)
	{
		// Insert neighbour based on greatest time.
		// 最大時間に基づいて付近のエージェントを挿入します。
		int slot = 0;
		if (!nagents)
		{
			slot = nagents;
		}
		else if (newag->topologyOptTime <= agents[nagents - 1]->topologyOptTime)
		{
			if (nagents >= maxAgents)
				return nagents;
			slot = nagents;
		}
		else
		{
			int i;
			for (i = 0; i < nagents; ++i)
				if (newag->topologyOptTime >= agents[i]->topologyOptTime)
					break;

			const int tgt = i + 1;
			const int n = dtMin(nagents - i, maxAgents - tgt);

			dtAssert(tgt + n <= maxAgents);

			if (n > 0)
				memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*) * n);
			slot = i;
		}

		agents[slot] = newag;

		return dtMin(nagents + 1, maxAgents);
	}

	int addToPathQueue(dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents)
	{
		// Insert neighbour based on greatest time.
		int slot = 0;
		if (!nagents)
		{
			slot = nagents;
		}
		else if (newag->targetReplanTime <= agents[nagents - 1]->targetReplanTime)
		{
			if (nagents >= maxAgents)
				return nagents;
			slot = nagents;
		}
		else
		{
			int i;
			for (i = 0; i < nagents; ++i)
				if (newag->targetReplanTime >= agents[i]->targetReplanTime)
					break;

			const int tgt = i + 1;
			const int n = dtMin(nagents - i, maxAgents - tgt);

			dtAssert(tgt + n <= maxAgents);

			if (n > 0)
				memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*) * n);
			slot = i;
		}

		agents[slot] = newag;

		return dtMin(nagents + 1, maxAgents);
	}
}

/**
@class dtCrowd
@par

This is the core class of the @ref crowd module.  See the @ref crowd documentation for a summary
of the crowd features.

A common method for setting up the crowd is as follows:

-# Allocate the crowd using #dtAllocCrowd.
-# Initialize the crowd using #init().
-# Set the avoidance configurations using #setObstacleAvoidanceParams().
-# Add agents using #addAgent() and make an initial movement request using #requestMoveTarget().

A common process for managing the crowd is as follows:

-# Call #update() to allow the crowd to manage its agents.
-# Retrieve agent information using #getActiveAgents().
-# Make movement requests using #requestMoveTarget() when movement goal changes.
-# Repeat every frame.

Some agent configuration settings can be updated using #updateAgentParameters().  But the crowd owns the
agent position.  So it is not possible to update an active agent's position.  If agent position
must be fed back into the crowd, the agent must be removed and re-added.

Notes:

- Path related information is available for newly added agents only after an #update() has been
  performed.
- Agent objects are kept in a pool and re-used.  So it is important when using agent objects to check the value of
  #dtCrowdAgent::active to determine if the agent is actually in use or not.
- This class is meant to provide 'local' movement. There is a limit of 256 polygons in the path corridor.
  So it is not meant to provide automatic pathfinding services over long distances.

@see dtAllocCrowd(), dtFreeCrowd(), init(), dtCrowdAgent

*/

dtCrowd::dtCrowd() :
	m_maxAgents(0),
	m_agents(0),
	m_activeAgents(0),
	m_agentAnims(0),
	m_obstacleQuery(0),
	m_grid(0),
	m_pathResult(0),
	m_maxPathResult(0),
	m_maxAgentRadius(0),
	m_velocitySampleCount(0),
	m_navquery(0)
{
}

dtCrowd::~dtCrowd()
{
	purge();
}

void dtCrowd::purge()
{
	for (int i = 0; i < m_maxAgents; ++i)
		m_agents[i].~dtCrowdAgent(); // delete は使えないため、デストラクタを直接呼び出す
	dtFree(m_agents);
	m_agents = 0;
	m_maxAgents = 0;

	dtFree(m_activeAgents);
	m_activeAgents = 0;

	dtFree(m_agentAnims);
	m_agentAnims = 0;

	dtFree(m_pathResult);
	m_pathResult = 0;

	dtFreeProximityGrid(m_grid);
	m_grid = 0;

	dtFreeObstacleAvoidanceQuery(m_obstacleQuery);
	m_obstacleQuery = 0;

	dtFreeNavMeshQuery(m_navquery);
	m_navquery = 0;
}

/// @par
///
/// May be called more than once to purge and re-initialize the crowd.
/// クラウドをパージして再初期化するために複数回呼び出される場合があります。
bool dtCrowd::init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav)
{
	purge();

	m_maxAgents = maxAgents;
	m_maxAgentRadius = maxAgentRadius;

	// Larger than agent radius because it is also used for agent recovery.
	// エージェントの復旧にも使用されるため、エージェントの半径よりも大きい。
	dtVset(m_agentPlacementHalfExtents, m_maxAgentRadius*2.0f, m_maxAgentRadius*1.5f, m_maxAgentRadius*2.0f);

	m_grid = dtAllocProximityGrid();

	if (!m_grid) return false;

	if (!m_grid->init(m_maxAgents * 4, maxAgentRadius * 3))
		return false;

	m_obstacleQuery = dtAllocObstacleAvoidanceQuery();
	if (!m_obstacleQuery)
		return false;
	if (!m_obstacleQuery->init(6, 8))
		return false;

	// Init obstacle query params.
	// 障害物クエリパラメータを初期化します。
	memset(m_obstacleQueryParams, 0, sizeof(m_obstacleQueryParams));
	for (int i = 0; i < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS; ++i)
	{
		dtObstacleAvoidanceParams* params = &m_obstacleQueryParams[i];
		params->velBias = 0.4f;
		params->weightDesVel = 2.0f;
		params->weightCurVel = 0.75f;
		params->weightSide = 0.75f;
		params->weightToi = 2.5f;
		params->horizTime = 2.5f;
		params->gridSize = 33;
		params->adaptiveDivs = 7;
		params->adaptiveRings = 2;
		params->adaptiveDepth = 5;
	}

	// Allocate temp buffer for merging paths.
	// パスをマージするための一時バッファを割り当てます。
	m_maxPathResult = 256;
	m_pathResult = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef) * m_maxPathResult, DT_ALLOC_PERM);
	if (!m_pathResult)
		return false;

	if (!m_pathq.init(m_maxPathResult, MAX_PATHQUEUE_NODES, nav))
		return false;

	m_agents = (dtCrowdAgent*)dtAlloc(sizeof(dtCrowdAgent) * m_maxAgents, DT_ALLOC_PERM);
	if (!m_agents) return false;

	m_activeAgents = (dtCrowdAgent**)dtAlloc(sizeof(dtCrowdAgent*) * m_maxAgents, DT_ALLOC_PERM);
	if (!m_activeAgents) return false;

	m_agentAnims = (dtCrowdAgentAnimation*)dtAlloc(sizeof(dtCrowdAgentAnimation) * m_maxAgents, DT_ALLOC_PERM);
	if (!m_agentAnims) return false;

	for (int i = 0; i < m_maxAgents; ++i)
	{
		auto& agent{ m_agents[i] };

		// https://cpprefjp.github.io/reference/new/op_new.html
		/*
		この形式は実質何もしていない。
		この形式は、記憶域を確保した上でそこに新たなオブジェクトを構築するのではなく、
		あらかじめ確保されている記憶域上に新たなオブジェクトを構築するのに用いられる。
		一般に、プログラム実行中の記憶域の動的確保は、処理系が OS からヒープを確保するのに対し、
		この形式では、既にプログラムに確保済みの任意の記憶域上にオブジェクトを構築するため、
		上手く使った場合には new / delete を大量に繰り返す必要のある処理を高速に実現しうる。
		*/
		new(&agent) dtCrowdAgent();

		agent.active = false;
		agent.is_run = false;

		if (!agent.corridor.init(m_maxPathResult))
			return false;
	}

	for (int i = 0; i < m_maxAgents; ++i)
	{
		m_agentAnims[i].active = false;
	}

	// The navquery is mostly used for local searches, no need for large node pool.
	// navqueryは主にローカル検索に使用され、大きなノードプールは必要ありません。
	m_navquery = dtAllocNavMeshQuery();

	if (!m_navquery) return false;

	if (dtStatusFailed(m_navquery->init(nav, MAX_COMMON_NODES)))
		return false;

	return true;
}

void dtCrowd::setObstacleAvoidanceParams(const int idx, const dtObstacleAvoidanceParams* params)
{
	if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
		memcpy(&m_obstacleQueryParams[idx], params, sizeof(dtObstacleAvoidanceParams));
}

const dtObstacleAvoidanceParams* dtCrowd::getObstacleAvoidanceParams(const int idx) const
{
	if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
		return &m_obstacleQueryParams[idx];

	return nullptr;
}

/// @par
///
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
const dtCrowdAgent* dtCrowd::getAgentAt(const int idx) const
{
	if (idx < 0 || idx >= m_maxAgents)
		return 0;
	return &m_agents[idx];
}

///
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
dtCrowdAgent* dtCrowd::getEditableAgentAt(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return 0;
	return &m_agents[idx];
}

void dtCrowd::updateAgentParameters(const int idx, const dtCrowdAgentParams* params)
{
	if (idx < 0 || idx >= m_maxAgents)
		return;
	memcpy(&m_agents[idx].params, params, sizeof(dtCrowdAgentParams));
}

/// @par
///
/// The agent's position will be constrained to the surface of the navigation mesh.
int dtCrowd::addAgent(const std::array<float, 3>& pos, const dtCrowdAgentParams* params)
{
	// Find empty slot.
	int idx = -1;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active)
		{
			idx = i;
			break;
		}
	}
	if (idx == -1)
		return -1;

	dtCrowdAgent* ag = &m_agents[idx];

	updateAgentParameters(idx, params);

	// Find nearest position on navmesh and place the agent there.
	// navmeshの最も近い位置を見つけて、そこにエージェントを配置します。
	std::array<float, 3> nearest{ pos };
	dtPolyRef ref = 0;
	dtStatus status = m_navquery->findNearestPoly(
		pos.data(), m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ref, nearest.data());

	if (dtStatusFailed(status))
	{
		nearest = pos;
		ref = 0;
	}

	ag->corridor.reset(ref, nearest.data());
	ag->boundary.reset();
	ag->partial = false;

	ag->topologyOptTime = 0;
	ag->targetReplanTime = 0;
	ag->nneis = 0;

	dtVset(ag->dvel, 0, 0, 0);
	dtVset(ag->nvel, 0, 0, 0);
	dtVset(ag->vel, 0, 0, 0);
	dtVcopy(ag->npos, nearest.data());

	ag->desiredSpeed = 0;

	if (ref)
		ag->state = DT_CROWDAGENT_STATE_WALKING;
	else
		ag->state = DT_CROWDAGENT_STATE_INVALID;

	ag->targetState = DT_CROWDAGENT_TARGET_NONE;

	ag->active = true;
	ag->is_run = true;

	return idx;
}

/// @par
///
/// The agent is deactivated and will no longer be processed.  Its #dtCrowdAgent object
/// is not removed from the pool.  It is marked as inactive so that it is available for reuse.
void dtCrowd::removeAgent(const int idx) noexcept
{
	if (idx >= 0 && idx < m_maxAgents)
	{
		auto& agent{ m_agents[idx] };

		agent.active = false;
		agent.is_run = false;
	}
}

void dtCrowd::SetRunning(const int idx, const bool is_running) noexcept
{
	if (idx >= 0 && idx < m_maxAgents)
	{
		auto& agent{ m_agents[idx] };

		if (agent.active)	agent.is_run = is_running;
	}
}

bool dtCrowd::IsRunning(const int idx) const noexcept
{
	if (idx >= 0 && idx < m_maxAgents)
	{
		auto& agent{ m_agents[idx] };

		return (agent.active && agent.is_run);
	}
	else
		return false;
}

bool dtCrowd::requestMoveTargetReplan(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	// Initialize request.
	ag->targetRef = ref;
	dtVcopy(ag->targetPos, pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = true;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
}

/// @par
///
/// This method is used when a new target is set.
///
/// The position will be constrained to the surface of the navigation mesh.
///
/// The request will be processed during the next #update().
bool dtCrowd::requestMoveTarget(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;
	if (!ref)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	// Initialize request.
	ag->targetRef = ref;
	dtVcopy(ag->targetPos, pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
}

bool dtCrowd::requestMoveVelocity(const int idx, const float* vel)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	// Initialize request.
	ag->targetRef = 0;
	dtVcopy(ag->targetPos, vel);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	ag->targetState = DT_CROWDAGENT_TARGET_VELOCITY;

	return true;
}

bool dtCrowd::resetMoveTarget(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	// Initialize request.
	ag->targetRef = 0;
	dtVset(ag->targetPos, 0, 0, 0);
	dtVset(ag->dvel, 0, 0, 0);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	ag->targetState = DT_CROWDAGENT_TARGET_NONE;

	return true;
}

int dtCrowd::getActiveAgents(dtCrowdAgent** agents, const int maxAgents)
{
	int n = 0;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		auto& agent{ m_agents[i] };

		if (!(agent.active && agent.is_run)) continue;

		if (n < maxAgents) agents[n++] = &agent;
	}
	return n;
}

void dtCrowd::updateMoveRequest(const float /*dt*/)
{
	const int PATH_MAX_AGENTS = 8;
	dtCrowdAgent* queue[PATH_MAX_AGENTS];
	int nqueue = 0;

	// Fire off new requests.
	// 新しいリクエストを開始します。
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgent* ag = &m_agents[i];
		if (!(ag->active && ag->is_run))
			continue;
		if (ag->state == DT_CROWDAGENT_STATE_INVALID)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING)
		{
			const dtPolyRef* path = ag->corridor.getPath();
			const int npath = ag->corridor.getPathCount();
			dtAssert(npath);

			constexpr int MAX_RES = 32;
			float reqPos[3];
			dtPolyRef reqPath[MAX_RES];	// The path to the request location //リクエストの場所へのパス
			int reqPathCount = 0;

			// Quick search towards the goal.
			// 目標に向かってすばやく検索します。
			constexpr int MAX_ITER = 20;
			m_navquery->initSlicedFindPath(path[0], ag->targetRef, ag->npos, ag->targetPos, &m_filters[ag->params.queryFilterType]);
			m_navquery->updateSlicedFindPath(MAX_ITER, 0);

			dtStatus status = 0;
			if (ag->targetReplan) // && npath > 10)
			{
				// Try to use existing steady path during replan if possible.
				// 可能であれば、再計画中に既存の安定したパスを使用してみてください。
				status = m_navquery->finalizeSlicedFindPathPartial(path, npath, reqPath, &reqPathCount, MAX_RES);
			}
			else
			{
				// Try to move towards target when goal changes.
				// 目標が変わったときに目標に向かって移動しようとします。
				status = m_navquery->finalizeSlicedFindPath(reqPath, &reqPathCount, MAX_RES);
			}

			if (!dtStatusFailed(status) && reqPathCount > 0)
			{
				// In progress or succeed.
				// 進行中または成功。
				if (reqPath[reqPathCount - 1] != ag->targetRef)
				{
					// Partial path, constrain target position inside the last polygon.
					// 部分パス。最後のポリゴン内のターゲット位置を制限します。
					status = m_navquery->closestPointOnPoly(reqPath[reqPathCount - 1], ag->targetPos, reqPos, 0);
					if (dtStatusFailed(status))
						reqPathCount = 0;
				}
				else
				{
					dtVcopy(reqPos, ag->targetPos);
				}
			}
			else
			{
				reqPathCount = 0;
			}

			if (!reqPathCount)
			{
				// Could not find path, start the request from current location.
				// パスが見つかりませんでした。現在の場所からリクエストを開始します
				dtVcopy(reqPos, ag->npos);
				reqPath[0] = path[0];
				reqPathCount = 1;
			}

			ag->corridor.setCorridor(reqPos, reqPath, reqPathCount);
			ag->boundary.reset();
			ag->partial = false;

			if (reqPath[reqPathCount - 1] == ag->targetRef)
			{
				ag->targetState = DT_CROWDAGENT_TARGET_VALID;
				ag->targetReplanTime = 0.0;
			}
			else
			{
				// The path is longer or potentially unreachable, full plan.
				// パスが長いか、到達できない可能性があるため、完全な計画です。
				ag->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE;
			}
		}

		if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
		{
			nqueue = addToPathQueue(ag, queue, nqueue, PATH_MAX_AGENTS);
		}
	}

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];
		ag->targetPathqRef = m_pathq.request(ag->corridor.getLastPoly(), ag->targetRef,
			ag->corridor.getTarget(), ag->targetPos, &m_filters[ag->params.queryFilterType]);
		if (ag->targetPathqRef != DT_PATHQ_INVALID)
			ag->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_PATH;
	}

	// Update requests.
	// リクエストを更新します。
	m_pathq.update(MAX_ITERS_PER_UPDATE);

	dtStatus status;

	// Process path results.
	// パスの結果を処理します。
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgent* ag = &m_agents[i];
		if (!ag->active)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
		{
			// Poll path queue.
			status = m_pathq.getRequestStatus(ag->targetPathqRef);

			if (dtStatusFailed(status))
			{
				// Path find failed, retry if the target location is still valid.
				// パスの検索に失敗しました。ターゲットの場所がまだ有効な場合は再試行してください。
				ag->targetPathqRef = DT_PATHQ_INVALID;
				if (ag->targetRef)
					ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
				else
					ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
				ag->targetReplanTime = 0.0;
			}
			else if (dtStatusSucceed(status))
			{
				const dtPolyRef* path = ag->corridor.getPath();
				const int npath = ag->corridor.getPathCount();
				dtAssert(npath);

				// Apply results.
				// 結果を適用します。
				float targetPos[3];
				dtVcopy(targetPos, ag->targetPos);

				dtPolyRef* res = m_pathResult;
				bool valid = true;
				int nres = 0;
				status = m_pathq.getPathResult(ag->targetPathqRef, res, &nres, m_maxPathResult);
				if (dtStatusFailed(status) || !nres)
					valid = false;

				if (dtStatusDetail(status, DT_PARTIAL_RESULT))
					ag->partial = true;
				else
					ag->partial = false;

				// Merge result and existing path.
				// The agent might have moved whilst the request is being processed, so the path may have changed.
				// We assume that the end of the path is at the same location where the request was issued.
				//結果と既存のパスをマージします。
				//リクエストの処理中にエージェントが移動した可能性があるため、パスが変更された可能性があります。
				//パスの終わりは、リクエストが発行されたのと同じ場所にあると想定しています。

				// The last ref in the old path should be the same as the location where the request was issued.
				//古いパスの最後の参照は、リクエストが発行された場所と同じである必要があります。
				if (valid && path[npath - 1] != res[0])
					valid = false;

				if (valid)
				{
					// Put the old path infront of the old path.
					// 古いパスを古いパスの前に置きます。
					if (npath > 1)
					{
						// Make space for the old path.
						if ((npath - 1) + nres > m_maxPathResult)
							nres = m_maxPathResult - (npath - 1);

						memmove(res + npath - 1, res, sizeof(dtPolyRef) * nres);
						// Copy old path in the beginning.
						// 最初に古いパスをコピーします。
						memcpy(res, path, sizeof(dtPolyRef) * (npath - 1));
						nres += npath - 1;

						// Remove trackbacks
						// トラックバックを削除します
						for (int j = 0; j < nres; ++j)
						{
							if (j - 1 >= 0 && j + 1 < nres)
							{
								if (res[j - 1] == res[j + 1])
								{
									memmove(res + (j - 1), res + (j + 1), sizeof(dtPolyRef) * (nres - (j + 1)));
									nres -= 2;
									j -= 2;
								}
							}
						}
					}

					// Check for partial path.
					// パスの一部を確認します。
					if (res[nres - 1] != ag->targetRef)
					{
						// Partial path, constrain target position inside the last polygon.
						// 部分パス。最後のポリゴン内のターゲット位置を制限します。
						float nearest[3];
						status = m_navquery->closestPointOnPoly(res[nres - 1], targetPos, nearest, 0);
						if (dtStatusSucceed(status))
							dtVcopy(targetPos, nearest);
						else
							valid = false;
					}
				}

				if (valid)
				{
					// Set current corridor.
					// 現在のコリドーを設定します。
					ag->corridor.setCorridor(targetPos, res, nres);
					// Force to update boundary.
					// 境界を強制的に更新します。
					ag->boundary.reset();
					ag->targetState = DT_CROWDAGENT_TARGET_VALID;
				}
				else
				{
					// Something went wrong.
					// 問題が発生しました。
					ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
				}

				ag->targetReplanTime = 0.0;
			}
		}
	}
}

void dtCrowd::updateTopologyOptimization(dtCrowdAgent** agents, const int nagents, const float dt)
{
	if (!nagents)
		return;

	const float OPT_TIME_THR = 0.5f; // seconds
	const int OPT_MAX_AGENTS = 1;
	dtCrowdAgent* queue[OPT_MAX_AGENTS];
	int nqueue = 0;

	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;
		if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_TOPO) == 0)
			continue;
		ag->topologyOptTime += dt;
		if (ag->topologyOptTime >= OPT_TIME_THR)
			nqueue = addToOptQueue(ag, queue, nqueue, OPT_MAX_AGENTS);
	}

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];
		ag->corridor.optimizePathTopology(m_navquery, &m_filters[ag->params.queryFilterType]);
		ag->topologyOptTime = 0;
	}
}

void dtCrowd::checkPathValidity(dtCrowdAgent** agents, const int nagents, const float dt)
{
	constexpr int CHECK_LOOKAHEAD = 10;
	constexpr float TARGET_REPLAN_DELAY = 1.0; // seconds

	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag{ agents[i] };

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		ag->targetReplanTime += dt;

		bool replan = false;

		// First check that the current location is valid.
		const int idx = getAgentIndex(ag);
		float agentPos[3];
		dtPolyRef agentRef = ag->corridor.getFirstPoly();
		dtVcopy(agentPos, ag->npos);
		if (!m_navquery->isValidPolyRef(agentRef, &m_filters[ag->params.queryFilterType]))
		{
			// Current location is not valid, try to reposition.
			// TODO: this can snap agents, how to handle that?
			float nearest[3]{};
			dtVcopy(nearest, agentPos);
			agentRef = 0;
			m_navquery->findNearestPoly(ag->npos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &agentRef, nearest);
			dtVcopy(agentPos, nearest);

			if (!agentRef)
			{
				// Could not find location in navmesh, set state to invalid.
				ag->corridor.reset(0, agentPos);
				ag->partial = false;
				ag->boundary.reset();
				ag->state = DT_CROWDAGENT_STATE_INVALID;
				continue;
			}

			// Make sure the first polygon is valid, but leave other valid
			// polygons in the path so that replanner can adjust the path better.
			ag->corridor.fixPathStart(agentRef, agentPos);
			//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
			ag->boundary.reset();
			dtVcopy(ag->npos, agentPos);

			replan = true;
		}

		// If the agent does not have move target or is controlled by velocity, no need to recover the target nor replan.
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		// Try to recover move request position.
		if (ag->targetState != DT_CROWDAGENT_TARGET_NONE && ag->targetState != DT_CROWDAGENT_TARGET_FAILED)
		{
			if (!m_navquery->isValidPolyRef(ag->targetRef, &m_filters[ag->params.queryFilterType]))
			{
				// Current target is not valid, try to reposition.
				float nearest[3];
				dtVcopy(nearest, ag->targetPos);
				ag->targetRef = 0;
				m_navquery->findNearestPoly(ag->targetPos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ag->targetRef, nearest);
				dtVcopy(ag->targetPos, nearest);
				replan = true;
			}
			if (!ag->targetRef)
			{
				// Failed to reposition target, fail moverequest.
				ag->corridor.reset(agentRef, agentPos);
				ag->partial = false;
				ag->targetState = DT_CROWDAGENT_TARGET_NONE;
			}
		}

		// If nearby corridor is not valid, replan.
		if (!ag->corridor.isValid(CHECK_LOOKAHEAD, m_navquery, &m_filters[ag->params.queryFilterType]))
		{
			// Fix current path.
//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
//			ag->boundary.reset();
			replan = true;
		}

		// If the end of the path is near and it is not the requested location, replan.
		if (ag->targetState == DT_CROWDAGENT_TARGET_VALID)
		{
			if (ag->targetReplanTime > TARGET_REPLAN_DELAY&&
				ag->corridor.getPathCount() < CHECK_LOOKAHEAD&&
				ag->corridor.getLastPoly() != ag->targetRef)
				replan = true;
		}

		// Try to replan path to goal.
		if (replan)
		{
			if (ag->targetState != DT_CROWDAGENT_TARGET_NONE)
			{
				requestMoveTargetReplan(idx, ag->targetRef, ag->targetPos);
			}
		}
	}
}

void dtCrowd::update(const float dt, dtCrowdAgentDebugInfo* debug)
{
	m_velocitySampleCount = 0;

	const int debugIdx = debug ? debug->idx : -1;

	dtCrowdAgent** agents = m_activeAgents;
	int nagents = getActiveAgents(agents, m_maxAgents); // エージェントの有効数

	// Check that all agents still have valid paths.
	// すべてのエージェントにまだ有効なパスがあることを確認します。
	checkPathValidity(agents, nagents, dt);

	// Update async move request and path finder.
	// 非同期移動リクエストとパスファインダーを更新します。
	updateMoveRequest(dt);

	// Optimize path topology.
	// パストポロジを最適化します。
	updateTopologyOptimization(agents, nagents, dt);

	// Register agents to proximity grid.
	// エージェントをプロキシミティグリッドに登録します。
	m_grid->clear();
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		const float* p = ag->npos;
		const float r = ag->params.radius;
		m_grid->addItem((unsigned short)i, p[0] - r, p[2] - r, p[0] + r, p[2] + r);
	}

	// Get nearby navmesh segments and agents to collide with.
	// 衝突する近くのナビメッシュセグメントとエージェントを取得します。
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		// Update the collision boundary after certain distance has been passed or if it has become invalid.
		// 特定の距離が経過した後、または無効になった場合に、衝突境界を更新します。
		const float updateThr = ag->params.collisionQueryRange * 0.25f;
		if (dtVdist2DSqr(ag->npos, ag->boundary.getCenter()) > dtSqr(updateThr) ||
			!ag->boundary.isValid(m_navquery, &m_filters[ag->params.queryFilterType]))
		{
			ag->boundary.update(ag->corridor.getFirstPoly(), ag->npos, ag->params.collisionQueryRange,
				m_navquery, &m_filters[ag->params.queryFilterType]);
		}

		// Query neighbour agents
		// 近隣エージェントをクエリします
		ag->nneis = getNeighbours(ag->npos, ag->params.height, ag->params.collisionQueryRange,
			ag, ag->neis, DT_CROWDAGENT_MAX_NEIGHBOURS,
			agents, nagents, m_grid);
		for (int j = 0; j < ag->nneis; j++)
			ag->neis[j].idx = getAgentIndex(agents[ag->neis[j].idx]);
	}

	// Find next corner to steer to.
	// 操縦する次のコーナーを見つけます。
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		// Find corners for steering
		// ステアリングのコーナーを見つける
		ag->ncorners = ag->corridor.findCorners(ag->cornerVerts, ag->cornerFlags, ag->cornerPolys,
			DT_CROWDAGENT_MAX_CORNERS, m_navquery, &m_filters[ag->params.queryFilterType]);

		// Check to see if the corner after the next corner is directly visible, and short cut to there.
		// 次のコーナーの後のコーナーが直接表示されるかどうかを確認し、そこへのショートカット。
		if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_VIS) && ag->ncorners > 0)
		{
			const float* target = &ag->cornerVerts[dtMin(1, ag->ncorners - 1) * 3];
			ag->corridor.optimizePathVisibility(target, ag->params.pathOptimizationRange, m_navquery, &m_filters[ag->params.queryFilterType]);

			// Copy data for debug purposes.
			// デバッグのためにデータをコピーします。
			if (debugIdx == i)
			{
				dtVcopy(debug->optStart, ag->corridor.getPos());
				dtVcopy(debug->optEnd, target);
			}
		}
		else
		{
			// Copy data for debug purposes.
			// デバッグのためにデータをコピーします。
			if (debugIdx == i)
			{
				dtVset(debug->optStart, 0, 0, 0);
				dtVset(debug->optEnd, 0, 0, 0);
			}
		}
	}

	// Trigger off-mesh connections (depends on corners).
	// オフメッシュ接続をトリガーします（コーナーによって異なります）。
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		// Check
		const float triggerRadius = ag->params.radius * 2.25f;
		if (overOffmeshConnection(ag, triggerRadius))
		{
			// Prepare to off-mesh connection.
			// オフメッシュ接続の準備をします。
			const int idx = (int)(ag - m_agents);
			dtCrowdAgentAnimation* anim = &m_agentAnims[idx];

			// Adjust the path over the off-mesh connection.
			// オフメッシュ接続のパスを調整します。
			dtPolyRef refs[2];
			if (ag->corridor.moveOverOffmeshConnection(ag->cornerPolys[ag->ncorners - 1], refs,
				anim->startPos, anim->endPos, m_navquery))
			{
				dtVcopy(anim->initPos, ag->npos);
				anim->polyRef = refs[1];
				anim->active = true;
				anim->t = 0.0f;
				anim->tmax = (dtVdist2D(anim->startPos, anim->endPos) / ag->params.maxSpeed) * 0.5f;

				ag->state = DT_CROWDAGENT_STATE_OFFMESH;
				ag->ncorners = 0;
				ag->nneis = 0;
				continue;
			}
			else
			{
				// Path validity check will ensure that bad/blocked connections will be replanned.
				// パスの有効性チェックにより、不良またはブロックされた接続が確実に再計画されます。
			}
		}
	}

	// Calculate steering.
	// ステアリングを計算します。
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag{ agents[i] };

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE)
			continue;

		float dvel[3]{};

		if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			dtVcopy(dvel, ag->targetPos);
			ag->desiredSpeed = dtVlen(ag->targetPos);
		}
		else
		{
			// Calculate steering direction.
			// ステアリング方向を計算します。
			if (ag->params.updateFlags & DT_CROWD_ANTICIPATE_TURNS)
				calcSmoothSteerDirection(ag, dvel);
			else
				calcStraightSteerDirection(ag, dvel);

			// Calculate speed scale, which tells the agent to slowdown at the end of the path.
			// 速度スケールを計算します。これは、パスの最後で減速するようにエージェントに指示します。
			const float slowDownRadius = ag->params.radius * 2;	// TODO: make less hacky.
			const float speedScale = getDistanceToGoal(ag, slowDownRadius) / slowDownRadius;

			ag->desiredSpeed = ag->params.maxSpeed;
			dtVscale(dvel, dvel, ag->desiredSpeed * speedScale);
		}

		// Separation // 分離
		if (ag->params.updateFlags & DT_CROWD_SEPARATION)
		{
			const float separationDist = ag->params.collisionQueryRange;
			const float invSeparationDist = 1.f / separationDist;
			const float separationWeight = ag->params.separationWeight;

			float w = 0;
			float disp[3] = { 0,0,0 };

			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];

				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				diff[1] = 0;

				const float distSqr = dtVlenSqr(diff);
				if (distSqr < 0.00001f)
					continue;
				if (distSqr > dtSqr(separationDist))
					continue;
				const float dist = dtMathSqrtf(distSqr);
				const float weight = separationWeight * (1.f - dtSqr(dist * invSeparationDist));

				dtVmad(disp, disp, diff, weight / dist);
				w += 1.f;
			}

			if (w > 0.0001f)
			{
				// Adjust desired velocity.
				// 希望の速度を調整します。
				dtVmad(dvel, dvel, disp, 1.f / w);
				// Clamp desired velocity to desired speed.
				// 希望の速度を希望の速度にクランプします。
				const float speedSqr = dtVlenSqr(dvel);
				const float desiredSqr = dtSqr(ag->desiredSpeed);
				if (speedSqr > desiredSqr)
					dtVscale(dvel, dvel, desiredSqr / speedSqr);
			}
		}

		// Set the desired velocity.
		// 希望の速度を設定します。
		dtVcopy(ag->dvel, dvel);
	}

	// Velocity planning.
	// 速度計画。
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		if (ag->params.updateFlags & DT_CROWD_OBSTACLE_AVOIDANCE)
		{
			m_obstacleQuery->reset();

			// Add neighbours as obstacles.
			// 付近のエージェントを障害物として追加します。
			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				m_obstacleQuery->addCircle(nei->npos, nei->params.radius, nei->vel, nei->dvel);
			}

			// Append neighbour segments as obstacles.
			// 隣接するセグメントを障害物として追加します。
			for (int j = 0; j < ag->boundary.getSegmentCount(); ++j)
			{
				const float* s = ag->boundary.getSegment(j);
				if (dtTriArea2D(ag->npos, s, s + 3) < 0.0f)
					continue;
				m_obstacleQuery->addSegment(s, s + 3);
			}

			dtObstacleAvoidanceDebugData* vod = 0;
			if (debugIdx == i)
				vod = debug->vod;

			// Sample new safe velocity.
			// 新しい安全速度をサンプリングします。
			bool adaptive = true;
			int ns = 0;

			const dtObstacleAvoidanceParams* params = &m_obstacleQueryParams[ag->params.obstacleAvoidanceType];

			//! 群衆を動かしている関数（メイン）
			if (adaptive)
			{
				ns = m_obstacleQuery->sampleVelocityAdaptive(ag->npos, ag->params.radius, ag->desiredSpeed,
					ag->vel, ag->dvel, ag->nvel, params, vod);
			}
			else
			{
				ns = m_obstacleQuery->sampleVelocityGrid(ag->npos, ag->params.radius, ag->desiredSpeed,
					ag->vel, ag->dvel, ag->nvel, params, vod);
			}
			m_velocitySampleCount += ns;
		}
		else
		{
			// If not using velocity planning, new velocity is directly the desired velocity.
			// 速度計画を使用しない場合、新しい速度は直接望ましい速度です。
			dtVcopy(ag->nvel, ag->dvel);
		}
	}

	// Integrate.
	// 統合。
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		integrate(ag, dt);
	}

	// Handle collisions.
	// 衝突を処理します。
	// 設置時にエージェント同士が重なり合うのを防ぐ（それ以外はあまり関係ないはず）
	for (int iter = 0; iter < 4; ++iter)
	{
		for (int i = 0; i < nagents; ++i)
		{
			dtCrowdAgent* ori_ag = agents[i]; // 基準のエージェント
			const int ori_idx = getAgentIndex(ori_ag); // 基準のエージェントの添え値

			if (ori_ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			dtVset(ori_ag->disp, 0, 0, 0);

			float w = 0;

			for (int j = 0; j < ori_ag->nneis; ++j)
			{
				const dtCrowdAgent* oth_ag_nei{ &m_agents[ori_ag->neis[j].idx] }; // 他のエージェント（付近のみ）
				const int oth_idx{ getAgentIndex(oth_ag_nei) }; // 他のエージェントの添え値

				float ag_vec[3]; // 付近のエージェントから基準のエージェントへのベクトル
				dtVsub(ag_vec, ori_ag->npos, oth_ag_nei->npos);
				ag_vec[1] = 0; // Yは無視

				float dist = dtVlenSqr(ag_vec); // 距離の二乗を求める

				// 円と円の判定を行い衝突していないなら何もしない
				if (dist > dtSqr(ori_ag->params.radius + oth_ag_nei->params.radius))
					continue;

				dist = dtMathSqrtf(dist); // 実際の距離を求める

				// めり込んでいる距離を計算
				float pen = (ori_ag->params.radius + oth_ag_nei->params.radius) - dist;

				// お互いがほぼ同じ位置にいる
				if (dist < 0.0001f)
				{
					// Agents on top of each other, try to choose diverging separation directions.
					// エージェントを互いの上に置き、発散する分離方向を選択してみてください。
					if (ori_idx > oth_idx)
						dtVset(ag_vec, -ori_ag->dvel[2], 0, ori_ag->dvel[0]);
					else
						dtVset(ag_vec, ori_ag->dvel[2], 0, -ori_ag->dvel[0]);

					pen = 0.01f;
				}
				// 1フレームで離れないように徐々に離していく
				else
				{
					constexpr float COLLISION_RESOLVE_FACTOR = 0.5f; // 離す際の加速度

					pen = (1.f / dist) * (pen * 0.5f) * COLLISION_RESOLVE_FACTOR;
				}

				// 徐々にベクトル方向に加速させる（離す）
				dtVmad(ori_ag->disp, ori_ag->disp, ag_vec, pen);

				w += 1.f;
			}

			// 急激に離れないようにする為、付近のエージェント数に応じて速度を緩める
			if (w > 0.0001f)
			{
				const float iw = 1.f / w;
				dtVscale(ori_ag->disp, ori_ag->disp, iw);
			}
		}

		// 位置に加算する
		for (int i = 0; i < nagents; ++i)
		{
			dtCrowdAgent* ag = agents[i];
			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			dtVadd(ag->npos, ag->npos, ag->disp);
		}
	}

	// ナビメッシュの角などを綺麗に移動できるようにする所（移動のサブ担当）
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		// Move along navmesh.
		// navmeshに沿って移動します。
		ag->corridor.movePosition(ag->npos, m_navquery, &m_filters[ag->params.queryFilterType]);
		// Get valid constrained position back.
		// 有効な拘束位置を取得します。
		dtVcopy(ag->npos, ag->corridor.getPos());

		// If not using path, truncate the corridor to just one poly.
		// パスを使用しない場合は、コリドーを1つのポリゴンに切り詰めます。
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			ag->corridor.reset(ag->corridor.getFirstPoly(), ag->npos);
			ag->partial = false;
		}
	}

	// Update agents using off-mesh connection.
	// オフメッシュ接続を使用してエージェントを更新します。
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgentAnimation* anim = &m_agentAnims[i];
		if (!anim->active)
			continue;
		dtCrowdAgent* ag = agents[i];

		anim->t += dt;
		if (anim->t > anim->tmax)
		{
			// Reset animation
			// アニメーションをリセットします
			anim->active = false;
			// Prepare agent for walking.
			// ウォーキングのためにエージェントを準備します。
			ag->state = DT_CROWDAGENT_STATE_WALKING;
			continue;
		}

		// Update position
		// 位置を更新します
		const float ta = anim->tmax * 0.15f;
		const float tb = anim->tmax;
		if (anim->t < ta)
		{
			const float u = tween(anim->t, 0.0, ta);
			dtVlerp(ag->npos, anim->initPos, anim->startPos, u);
		}
		else
		{
			const float u = tween(anim->t, ta, tb);
			dtVlerp(ag->npos, anim->startPos, anim->endPos, u);
		}

		// Update velocity.
		// 速度を更新します。
		dtVset(ag->vel, 0, 0, 0);
		dtVset(ag->dvel, 0, 0, 0);
	}
}
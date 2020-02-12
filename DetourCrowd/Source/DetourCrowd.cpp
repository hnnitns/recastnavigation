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

using namespace DtOperator;

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
		ArrayF dv{ ag->nvel - ag->vel };
		float ds = dtVlen(dv);

		if (ds > maxDelta)
			dv *= maxDelta / ds;

		ag->vel += dv;

		// Integrate
		if (dtVlen(ag->vel) > 0.0001f)
			dtVmad(&ag->npos, ag->npos, ag->vel, dt);
		else
			ag->vel.fill(0.f);
	}

	bool overOffmeshConnection(const dtCrowdAgent* ag, const float radius)
	{
		if (!ag->ncorners)
			return false;

		const bool offMeshConnection = (ag->cornerFlags[ag->ncorners - 1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
		if (offMeshConnection)
		{
			const float distSq = dtVdist2DSqr(ag->npos.data(), &ag->cornerVerts[(ag->ncorners - 1) * 3]);
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
			return dtMin(dtVdist2D(ag->npos.data(), &ag->cornerVerts[(ag->ncorners - 1) * 3]), range);

		return range;
	}

	void calcSmoothSteerDirection(const dtCrowdAgent* ag, ArrayF* dir)
	{
		if (!ag->ncorners)
		{
			dir->fill(0.f);

			return;
		}

		const int ip0 = 0;
		const int ip1 = dtMin(1, ag->ncorners - 1);
		const float* p0 = &ag->cornerVerts[ip0 * 3];
		const float* p1 = &ag->cornerVerts[ip1 * 3];

		ArrayF dir0{}, dir1{};
		dtVsub(dir0.data(), p0, ag->npos.data());
		dtVsub(dir1.data(), p1, ag->npos.data());
		dir0[1] = 0;
		dir1[1] = 0;

		float len0 = dtVlen(dir0);
		float len1 = dtVlen(dir1);
		if (len1 > 0.001f)
			dir1 *= (1.f / len1);

		dir->at(0) = dir0[0] - dir1[0] * len0 * 0.5f;
		dir->at(1) = 0;
		dir->at(2) = dir0[2] - dir1[2] * len0 * 0.5f;

		dtVnormalize(dir);
	}

	void calcStraightSteerDirection(const dtCrowdAgent* ag, ArrayF* dir)
	{
		if (!ag->ncorners)
		{
			dir->fill(0.f);

			return;
		}
		dtVsub(dir->data(), &ag->cornerVerts[0], ag->npos.data());
		dir->at(1) = 0;
		dtVnormalize(dir);
	}

	int addNeighbour(const int idx, const float dist,
		std::array<dtCrowdNeighbour, DT_CROWDAGENT_MAX_NEIGHBOURS>* neis, const int nneis, const int maxNeis)
	{
		// Insert neighbour based on the distance.
		dtCrowdNeighbour* nei{};

		if (!nneis)
		{
			nei = &neis->at(nneis);
		}
		else if (dist >= neis->at(nneis - 1).dist)
		{
			if (nneis >= maxNeis)
				return nneis;
			nei = &neis->at(nneis);
		}
		else
		{
			int i;
			for (i = 0; i < nneis; ++i)
				if (dist <= neis->at(i).dist)
					break;

			const int tgt = i + 1;
			const int n = dtMin(nneis - i, maxNeis - tgt);

			dtAssert(tgt + n <= maxNeis);

			if (n > 0)
				memmove(&neis->at(tgt), &neis->at(i), sizeof(dtCrowdNeighbour) * n);
			nei = &neis->at(i);
		}

		memset(nei, 0, sizeof(dtCrowdNeighbour));

		nei->idx = idx;
		nei->dist = dist;

		return dtMin(nneis + 1, maxNeis);
	}

	int getNeighbours(const ArrayF& pos, const float height, const float range, const dtCrowdAgent* skip,
		std::array<dtCrowdNeighbour, DT_CROWDAGENT_MAX_NEIGHBOURS>* result, const int maxResult,
		dtCrowdAgent** agents, [[maybe_unused]]const int nagents, dtProximityGrid* grid)
	{
		int n = 0;

		static const int MAX_NEIS = 32;
		unsigned short ids[MAX_NEIS];
		int nids = grid->queryItems(pos[0] - range, pos[2] - range,
			pos[0] + range, pos[2] + range,
			ids, MAX_NEIS);

		for (int i = 0; i < nids; ++i)
		{
			const dtCrowdAgent* ag = agents[ids[i]];

			if (ag == skip) continue;

			// Check for overlap.
			ArrayF diff{ pos - ag->npos };

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
		m_agents[i].~dtCrowdAgent();
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
/// �N���E�h���p�[�W���čď��������邽�߂ɕ�����Ăяo�����ꍇ������܂��B
bool dtCrowd::init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav)
{
	purge();

	m_maxAgents = maxAgents;
	m_maxAgentRadius = maxAgentRadius;

	dtVset(&m_ext, m_maxAgentRadius * 2.0f, m_maxAgentRadius * 1.5f, m_maxAgentRadius * 2.0f);

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
	// ��Q���N�G���p�����[�^�����������܂��B
	m_obstacleQueryParams.fill({});

	for (auto& params : m_obstacleQueryParams)
	{
		params.velBias = 0.4f;
		params.weightDesVel = 2.0f;
		params.weightCurVel = 0.75f;
		params.weightSide = 0.75f;
		params.weightToi = 2.5f;
		params.horizTime = 2.5f;
		params.gridSize = 33;
		params.adaptiveDivs = 7;
		params.adaptiveRings = 2;
		params.adaptiveDepth = 5;
	}

	// Allocate temp buffer for merging paths.
	// �p�X���}�[�W���邽�߂̈ꎞ�o�b�t�@�����蓖�Ă܂��B
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
		new(&m_agents[i]) dtCrowdAgent();
		m_agents[i].active = false;

		if (!m_agents[i].corridor.init(m_maxPathResult))
			return false;
	}

	for (int i = 0; i < m_maxAgents; ++i)
	{
		m_agentAnims[i].active = false;
	}

	// The navquery is mostly used for local searches, no need for large node pool.
	// navquery�͎�Ƀ��[�J�������Ɏg�p����A�傫�ȃm�[�h�v�[���͕K�v����܂���B
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

int dtCrowd::getAgentCount() const
{
	return m_maxAgents;
}

/// @par
///
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
const dtCrowdAgent* dtCrowd::getAgent(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return 0;
	return &m_agents[idx];
}

///
/// Agents in the pool may not be in use.  Check #dtCrowdAgent.active before using the returned object.
dtCrowdAgent* dtCrowd::getEditableAgent(const int idx)
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
int dtCrowd::addAgent(const float* pos, const dtCrowdAgentParams* params)
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
	ArrayF nearest;
	dtPolyRef ref = 0;
	dtVcopy(nearest.data(), pos);
	dtStatus status =
		m_navquery->findNearestPoly(
			pos, m_ext.data(), &m_filters[ag->params.queryFilterType], &ref, nearest.data());

	if (dtStatusFailed(status))
	{
		dtVcopy(nearest.data(), pos);
		ref = 0;
	}

	ag->corridor.reset(ref, nearest.data());
	ag->boundary.reset();
	ag->partial = false;

	ag->topologyOptTime = 0;
	ag->targetReplanTime = 0;
	ag->nneis = 0;

	ag->dvel.fill(0.f);
	ag->nvel.fill(0.f);
	ag->vel.fill(0.f);
	ag->npos = nearest;

	ag->desiredSpeed = 0;

	if (ref)
		ag->state = DT_CROWDAGENT_STATE_WALKING;
	else
		ag->state = DT_CROWDAGENT_STATE_INVALID;

	ag->targetState = DT_CROWDAGENT_TARGET_NONE;

	ag->active = true;

	return idx;
}

/// @par
///
/// The agent is deactivated and will no longer be processed.  Its #dtCrowdAgent object
/// is not removed from the pool.  It is marked as inactive so that it is available for reuse.
void dtCrowd::removeAgent(const int idx)
{
	if (idx >= 0 && idx < m_maxAgents)
	{
		m_agents[idx].active = false;
	}
}

bool dtCrowd::requestMoveTargetReplan(const int idx, dtPolyRef ref, const std::array<float, 3>& pos)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	// Initialize request.
	ag->targetRef = ref;
	ag->targetPos = pos;
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
	dtVcopy(ag->targetPos.data(), pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
}

bool dtCrowd::requestMoveVelocity(const int idx, const std::array<float, 3>& vel)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	// Initialize request.
	ag->targetRef = 0;
	ag->targetPos = vel;
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
	ag->targetPos.fill(0.f);
	ag->dvel.fill(0.f);
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
		if (!m_agents[i].active) continue;
		if (n < maxAgents)
			agents[n++] = &m_agents[i];
	}
	return n;
}

void dtCrowd::updateMoveRequest(const float /*dt*/)
{
	const int PATH_MAX_AGENTS = 8;
	dtCrowdAgent* queue[PATH_MAX_AGENTS];
	int nqueue = 0;

	// Fire off new requests.
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgent* ag = &m_agents[i];

		if (!ag->active)
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
			ArrayF reqPos;
			std::array<dtPolyRef, MAX_RES> reqPath;	// The path to the request location
			int reqPathCount{};

			// Quick search towards the goal.
			constexpr int MAX_ITER = 20;

			m_navquery->initSlicedFindPath(path[0], ag->targetRef, ag->npos.data(), ag->targetPos.data(), &m_filters[ag->params.queryFilterType]);
			m_navquery->updateSlicedFindPath(MAX_ITER, 0);
			dtStatus status{};

			if (ag->targetReplan) // && npath > 10)
			{
				// Try to use existing steady path during replan if possible.
				status = m_navquery->finalizeSlicedFindPathPartial(path, npath, reqPath.data(), &reqPathCount, MAX_RES);
			}
			else
			{
				// Try to move towards target when goal changes.
				status = m_navquery->finalizeSlicedFindPath(reqPath.data(), &reqPathCount, MAX_RES);
			}

			if (!dtStatusFailed(status) && reqPathCount > 0)
			{
				// In progress or succeed.
				if (reqPath[reqPathCount - 1] != ag->targetRef)
				{
					// Partial path, constrain target position inside the last polygon.
					status = m_navquery->closestPointOnPoly(reqPath[reqPathCount - 1], ag->targetPos.data(),
						reqPos.data(), 0);

					if (dtStatusFailed(status))
						reqPathCount = 0;
				}
				else
				{
					reqPos = ag->targetPos;
				}
			}
			else
			{
				reqPathCount = 0;
			}

			if (!reqPathCount)
			{
				// Could not find path, start the request from current location.
				reqPos = ag->npos;
				reqPath[0] = path[0];
				reqPathCount = 1;
			}

			ag->corridor.setCorridor(reqPos, reqPath.data(), reqPathCount);
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
	m_pathq.update(MAX_ITERS_PER_UPDATE);

	dtStatus status;

	// Process path results.
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
				ArrayF targetPos{ ag->targetPos };
				dtPolyRef* res = m_pathResult;
				bool valid = true;
				int nres{};

				status = m_pathq.getPathResult(ag->targetPathqRef, res, &nres, m_maxPathResult);
				if (dtStatusFailed(status) || !nres)
					valid = false;

				if (dtStatusDetail(status, DT_PARTIAL_RESULT))
					ag->partial = true;
				else
					ag->partial = false;

				// Merge result and existing path.
				// The agent might have moved whilst the request is
				// being processed, so the path may have changed.
				// We assume that the end of the path is at the same location
				// where the request was issued.

				// The last ref in the old path should be the same as
				// the location where the request was issued..
				if (valid && path[npath - 1] != res[0])
					valid = false;

				if (valid)
				{
					// Put the old path infront of the old path.
					if (npath > 1)
					{
						// Make space for the old path.
						if ((npath - 1) + nres > m_maxPathResult)
							nres = m_maxPathResult - (npath - 1);

						memmove(res + npath - 1, res, sizeof(dtPolyRef) * nres);
						// Copy old path in the beginning.
						memcpy(res, path, sizeof(dtPolyRef) * (npath - 1));
						nres += npath - 1;

						// Remove trackbacks
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
					if (res[nres - 1] != ag->targetRef)
					{
						// Partial path, constrain target position inside the last polygon.
						ArrayF nearest{};
						status =
							m_navquery->closestPointOnPoly(res[nres - 1], targetPos.data(), nearest.data(), 0);

						if (dtStatusSucceed(status))
							targetPos = nearest;
						else
							valid = false;
					}
				}

				if (valid)
				{
					// Set current corridor.
					ag->corridor.setCorridor(targetPos, res, nres);
					// Force to update boundary.
					ag->boundary.reset();
					ag->targetState = DT_CROWDAGENT_TARGET_VALID;
				}
				else
				{
					// Something went wrong.
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
	constexpr float TARGET_REPLAN_DELAY = 1.f; // seconds

	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		ag->targetReplanTime += dt;

		bool replan = false;

		// First check that the current location is valid.
		const int idx = getAgentIndex(ag);
		ArrayF agentPos{ ag->npos };
		dtPolyRef agentRef = ag->corridor.getFirstPoly();

		if (!m_navquery->isValidPolyRef(agentRef, &m_filters[ag->params.queryFilterType]))
		{
			// Current location is not valid, try to reposition.
			// TODO: this can snap agents, how to handle that?
			ArrayF nearest{ agentPos };

			agentRef = 0;
			m_navquery->
				findNearestPoly(ag->npos.data(), m_ext.data(), &m_filters[ag->params.queryFilterType],
					&agentRef, nearest.data());

			agentPos = nearest;

			if (!agentRef)
			{
				// Could not find location in navmesh, set state to invalid.
				ag->corridor.reset(0, agentPos.data());
				ag->partial = false;
				ag->boundary.reset();
				ag->state = DT_CROWDAGENT_STATE_INVALID;
				continue;
			}

			// Make sure the first polygon is valid, but leave other valid
			// polygons in the path so that replanner can adjust the path better.
			ag->corridor.fixPathStart(agentRef, agentPos.data());
			//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
			ag->boundary.reset();
			ag->npos = agentPos;

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
				ArrayF nearest{ ag->targetPos };

				ag->targetRef = 0;
				m_navquery->findNearestPoly(
					ag->targetPos.data(), m_ext.data(), &m_filters[ag->params.queryFilterType], &ag->targetRef,
					nearest.data());

				ag->targetPos = nearest;
				replan = true;
			}
			if (!ag->targetRef)
			{
				// Failed to reposition target, fail moverequest.
				ag->corridor.reset(agentRef, agentPos.data());
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
	int nagents = getActiveAgents(agents, m_maxAgents);

	// Check that all agents still have valid paths.
	// ���ׂẴG�[�W�F���g���܂��L���ȃp�X�������Ă��邱�Ƃ��m�F���܂��B
	checkPathValidity(agents, nagents, dt);

	// Update async move request and path finder.
	// �񓯊��ړ��v���ƃp�X�t�@�C���_�[���X�V���܂��B
	updateMoveRequest(dt);

	// Optimize path topology.
	// �p�X�g�|���W���œK�����܂��B
	updateTopologyOptimization(agents, nagents, dt);

	// Register agents to proximity grid.
	// �G�[�W�F���g���ߐڃO���b�h�ɓo�^���܂��B
	m_grid->clear();
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		const auto& p = ag->npos;
		const float r = ag->params.radius;
		m_grid->addItem((unsigned short)i, p[0] - r, p[2] - r, p[0] + r, p[2] + r);
	}

	// Get nearby navmesh segments and agents to collide with.
	// �Փ˂���߂���navmesh�Z�O�����g�ƃG�[�W�F���g���擾���܂��B
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		// Update the collision boundary after certain distance has been passed or if it has become invalid.
		// ����̋������n���ꂽ��A�܂��͖����ɂȂ����ꍇ�A�Փˋ��E���X�V���܂��B
		const float updateThr = ag->params.collisionQueryRange * 0.25f;
		if (dtVdist2DSqr(ag->npos.data(), ag->boundary.getCenter()) > dtSqr(updateThr) ||
			!ag->boundary.isValid(m_navquery, &m_filters[ag->params.queryFilterType]))
		{
			ag->boundary.update(ag->corridor.getFirstPoly(), ag->npos.data(), ag->params.collisionQueryRange,
				m_navquery, &m_filters[ag->params.queryFilterType]);
		}

		// Query neighbour agents
		// �l�C�o�[�G�[�W�F���g���Ɖ�܂�
		ag->nneis = getNeighbours(ag->npos, ag->params.height, ag->params.collisionQueryRange,
			ag, &ag->neis, DT_CROWDAGENT_MAX_NEIGHBOURS,
			agents, nagents, m_grid);

		for (int j = 0; j < ag->nneis; j++)
			ag->neis[j].idx = getAgentIndex(agents[ag->neis[j].idx]);
	}

	// Find next corner to steer to.
	// ���c���鎟�̃R�[�i�[�������܂��B
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		// Find corners for steering
		// �X�e�A�����O�̃R�[�i�[��������
		ag->ncorners = ag->corridor.findCorners(ag->cornerVerts.data(), ag->cornerFlags.data(),
			ag->cornerPolys.data(), DT_CROWDAGENT_MAX_CORNERS, m_navquery,
			&m_filters[ag->params.queryFilterType]);

		// Check to see if the corner after the next corner is directly visible, and short cut to there.
		// ���̃R�[�i�[�̌�̃R�[�i�[�����ڌ����邩�ǂ������m�F���A�����ւ̃V���[�g�J�b�g�B
		if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_VIS) && ag->ncorners > 0)
		{
			const float* target = &ag->cornerVerts[dtMin(1, ag->ncorners - 1) * 3];
			ag->corridor.optimizePathVisibility(target, ag->params.pathOptimizationRange, m_navquery, &m_filters[ag->params.queryFilterType]);

			// Copy data for debug purposes.
			// �f�o�b�O�̂��߂Ƀf�[�^���R�s�[���܂��B
			if (debugIdx == i)
			{
				debug->optStart = ag->corridor.getPos();
				dtVcopy(debug->optEnd.data(), target);
			}
		}
		else
		{
			// Copy data for debug purposes.
			// �f�o�b�O�̂��߂Ƀf�[�^���R�s�[���܂��B
			if (debugIdx == i)
			{
				debug->optStart.fill(0.f);
				debug->optEnd.fill(0.f);
			}
		}
	}

	// Trigger off-mesh connections (depends on corners).
	// �I�t���b�V���ڑ����g���K�[���܂��i�R�[�i�[�Ɉˑ��j�B
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
			// �I�t���b�V���ڑ��̏��������܂��B
			const int idx = (int)(ag - m_agents);
			dtCrowdAgentAnimation* anim = &m_agentAnims[idx];

			// Adjust the path over the off-mesh connection.
			// �I�t���b�V���ڑ���̃p�X�𒲐����܂��B
			std::array<dtPolyRef, 2> refs{};

			if (ag->corridor.moveOverOffmeshConnection(ag->cornerPolys[ag->ncorners - 1], &refs,
				&anim->startPos, &anim->endPos, m_navquery))
			{
				anim->initPos = ag->npos;
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
				// �p�X�̗L�����`�F�b�N�ɂ��A�s��/�u���b�N���ꂽ�ڑ����Čv�悳��܂�
			}
		}
	}

	// Calculate steering.
	// �X�e�A�����O���v�Z���܂��B
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE)
			continue;

		ArrayF dvel{};

		if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			dvel = ag->targetPos;
			ag->desiredSpeed = dtVlen(ag->targetPos);
		}
		else
		{
			// Calculate steering direction.
			// �X�e�A�����O�̕������v�Z���܂��B
			if (ag->params.updateFlags & DT_CROWD_ANTICIPATE_TURNS)
				calcSmoothSteerDirection(ag, &dvel);
			else
				calcStraightSteerDirection(ag, &dvel);

			// Calculate speed scale, which tells the agent to slowdown at the end of the path.
			// ���x�̃X�P�[�����v�Z���܂��B����ɂ��A�p�X�̏I���Ō�������悤�G�[�W�F���g�Ɏw�����܂��B
			const float slowDownRadius = ag->params.radius * 2;	// TODO: make less hacky.
			const float speedScale = getDistanceToGoal(ag, slowDownRadius) / slowDownRadius;

			ag->desiredSpeed = ag->params.maxSpeed;
			dvel *= (ag->desiredSpeed * speedScale);
		}

		//����
		if (ag->params.updateFlags & DT_CROWD_SEPARATION)
		{
			const float separationDist = ag->params.collisionQueryRange;
			const float invSeparationDist = 1.f / separationDist;
			const float separationWeight = ag->params.separationWeight;

			float w = 0;
			ArrayF disp{};

			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];

				ArrayF diff{ ag->npos - nei->npos };

				diff[1] = 0;

				const float distSqr = dtVlenSqr(diff);
				if (distSqr < 0.00001f)
					continue;
				if (distSqr > dtSqr(separationDist))
					continue;
				const float dist = dtMathSqrtf(distSqr);
				const float weight = separationWeight * (1.f - dtSqr(dist * invSeparationDist));

				dtVmad(&disp, disp, diff, weight / dist);
				w += 1.f;
			}

			if (w > 0.0001f)
			{
				// Adjust desired velocity.
				// ��]�̑��x�𒲐����܂��B
				dtVmad(&dvel, dvel, disp, 1.f / w);
				// Clamp desired velocity to desired speed.
				// ��]�̑��x����]�̑��x�ɌŒ肵�܂��B
				const float speedSqr = dtVlenSqr(dvel);
				const float desiredSqr = dtSqr(ag->desiredSpeed);
				if (speedSqr > desiredSqr)
					dvel *= (desiredSqr / speedSqr);
			}
		}

		// Set the desired velocity.
		// ��]�̑��x��ݒ肵�܂��B
		ag->dvel = dvel;
	}

	// Velocity planning.
	// ���x�v��B
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		if (ag->params.updateFlags & DT_CROWD_OBSTACLE_AVOIDANCE)
		{
			m_obstacleQuery->reset();

			// Add neighbours as obstacles.
			// �אl����Q���Ƃ��Ēǉ����܂��B
			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				m_obstacleQuery->addCircle(nei->npos, nei->params.radius, nei->vel, nei->dvel);
			}

			// Append neighbour segments as obstacles.
			// �אڃZ�O�����g����Q���Ƃ��Ēǉ����܂��B
			for (int j = 0; j < ag->boundary.getSegmentCount(); ++j)
			{
				const float* s = ag->boundary.getSegment(j);
				if (dtTriArea2D(ag->npos.data(), s, s + 3) < 0.0f)
					continue;
				m_obstacleQuery->addSegment(s, s + 3);
			}

			dtObstacleAvoidanceDebugData* vod = 0;
			if (debugIdx == i)
				vod = debug->vod;

			// Sample new safe velocity.
			// �V�������S�ȑ��x���T���v�����O���܂��B
			bool adaptive = true;
			int ns = 0;

			const dtObstacleAvoidanceParams* params = &m_obstacleQueryParams[ag->params.obstacleAvoidanceType];

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
			// ���x�v����g�p���Ȃ��ꍇ�A�V�������x�͒��ږړI�̑��x�ł��B
			ag->nvel = ag->dvel;
		}
	}

	//�������܂��B
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		integrate(ag, dt);
	}

	// Handle collisions.
	// �Փ˂��������܂��B
	static const float COLLISION_RESOLVE_FACTOR = 0.7f;

	for (int iter = 0; iter < 4; ++iter)
	{
		for (int i = 0; i < nagents; ++i)
		{
			dtCrowdAgent* ag = agents[i];
			const int idx0 = getAgentIndex(ag);

			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			ag->disp.fill(0.f);

			float w = 0;

			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				const int idx1 = getAgentIndex(nei);

				ArrayF diff{ ag->npos - nei->npos };

				diff[1] = 0;

				float dist = dtVlenSqr(diff);
				if (dist > dtSqr(ag->params.radius + nei->params.radius))
					continue;
				dist = dtMathSqrtf(dist);
				float pen = (ag->params.radius + nei->params.radius) - dist;
				if (dist < 0.0001f)
				{
					// Agents on top of each other, try to choose diverging separation directions.
					// �݂��̏�ɂ���G�[�W�F���g�́A���U���镪��������I�����悤�Ƃ��܂��B
					if (idx0 > idx1)
						dtVset(&diff, -ag->dvel[2], 0, ag->dvel[0]);
					else
						dtVset(&diff, ag->dvel[2], 0, -ag->dvel[0]);
					pen = 0.01f;
				}
				else
				{
					pen = (1.f / dist) * (pen * 0.5f) * COLLISION_RESOLVE_FACTOR;
				}

				dtVmad(&ag->disp, ag->disp, diff, pen);

				w += 1.f;
			}

			if (w > 0.0001f) ag->disp *= (1.f / w);
		}

		for (int i = 0; i < nagents; ++i)
		{
			dtCrowdAgent* ag = agents[i];
			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			ag->npos += ag->disp;
		}
	}

	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		// Move along navmesh.
		// �i�r���b�V���ɉ����Ĉړ����܂��B
		ag->corridor.movePosition(ag->npos, m_navquery, &m_filters[ag->params.queryFilterType]);
		// Get valid constrained position back.
		// �L���Ȑ���ʒu���擾���܂��B
		ag->npos = ag->corridor.getPos();

		// If not using path, truncate the corridor to just one poly.
		// �p�X���g�p���Ȃ��ꍇ�A�R���h�[��1�̃|���S���ɐ؂�̂Ă܂��B
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			ag->corridor.reset(ag->corridor.getFirstPoly(), ag->npos.data());
			ag->partial = false;
		}
	}

	// Update agents using off-mesh connection.
	// �I�t���b�V���ڑ����g�p���ăG�[�W�F���g���X�V���܂��B
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
			// �A�j���[�V���������Z�b�g���܂�
			anim->active = false;
			// Prepare agent for walking.
			// ���s�̂��߂ɃG�[�W�F���g���������܂��B
			ag->state = DT_CROWDAGENT_STATE_WALKING;
			continue;
		}

		// Update position
		// �ʒu���X�V���܂�
		const float ta = anim->tmax * 0.15f;
		const float tb = anim->tmax;

		if (anim->t < ta)
		{
			const float u = tween(anim->t, 0.0, ta);
			dtVlerp(&ag->npos, anim->initPos, anim->startPos, u);
		}
		else
		{
			const float u = tween(anim->t, ta, tb);
			dtVlerp(&ag->npos, anim->startPos, anim->endPos, u);
		}

		// Update velocity.
		// ���x���X�V���܂��B
		ag->vel.fill(0.f);
		ag->dvel.fill(0.f);
	}
}
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

#ifndef DETOURCROWD_H
#define DETOURCROWD_H

#include <array>
#include <deque>
#include "DetourNavMeshQuery.h"
#include "DetourObstacleAvoidance.h"
#include "DetourLocalBoundary.h"
#include "DetourPathCorridor.h"
#include "DetourProximityGrid.h"
#include "DetourPathQueue.h"
#include "Common.h"

// The maximum number of neighbors that a crowd agent can take into account for steering decisions.
// �N���E�h�G�[�W�F���g���X�e�A�����O����̂��߂ɍl���ł���t�߂̃G�[�W�F���g�̍ő吔�B
// @ingroup crowd
constexpr int DT_CROWDAGENT_MAX_NEIGHBOURS = 6;

// The maximum number of corners a crowd agent will look ahead in the path.
// �N���E�h�G�[�W�F���g���p�X�Ő������R�[�i�[�̍ő吔�B
// This value is used for sizing the crowd agent corner buffers.
// ���̒l�́A�N���E�h�G�[�W�F���g�R�[�i�[�o�b�t�@�[�̃T�C�Y�ݒ�Ɏg�p����܂��B
// Due to the behavior of the crowd manager, the actual number of useful corners will be one less than this number.
// �N���E�h�}�l�[�W���[�̓���ɂ��A�L�p�ȃR�[�i�[�̎��ۂ̐��͂��̐�����1���Ȃ��Ȃ�܂��B
// @ingroup crowd
constexpr int DT_CROWDAGENT_MAX_CORNERS = 4;

// The maximum number of crowd avoidance configurations supported by the crowd manager.
// �N���E�h�}�l�[�W���[���T�|�[�g����N���E�h����\���̍ő吔�B
// @ingroup crowd
// @see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), dtCrowd::getObstacleAvoidanceParams(),
//		 dtCrowdAgentParams::obstacleAvoidanceType
constexpr int DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS = 8;

// The maximum number of query filter types supported by the crowd manager.
// �N���E�h�}�l�[�W���[���T�|�[�g����N�G���t�B���^�[�^�C�v�̍ő吔�B
// @ingroup crowd
// @see dtQueryFilter, dtCrowd::getFilter() dtCrowd::getEditableFilter(),
//		dtCrowdAgentParams::queryFilterType
constexpr int DT_CROWD_MAX_QUERY_FILTER_TYPE = 16;

// Provides neighbor data for agents managed by the crowd.
// �Q�O���Ǘ�����G�[�W�F���g�̋ߗ׃f�[�^��񋟂��܂��B
// @ingroup crowd
// @see dtCrowdAgent::neis, dtCrowd
struct dtCrowdNeighbour
{
	int idx;		// The index of the neighbor in the crowd. // �Q�O�̒��ׂ̗̃C���f�b�N�X�B
	float dist;		// The distance between the current agent and the neighbor. // ���݂̃G�[�W�F���g�Ɨאl�̊Ԃ̋����B
};

// Configuration parameters for a crowd agent.
// @ingroup crowd
struct dtCrowdAgentParams
{
	float radius;			//< Agent radius. [Limit: >= 0]
	float height;			//< Agent height. [Limit: > 0]
	float maxAcceleration;	//< Maximum allowed acceleration. [Limit: >= 0]
	float maxSpeed;			//< Maximum allowed speed. [Limit: >= 0]

	// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
	// �X�e�A�����O����ƌ��Ȃ����O�ɏՓ˗v�f���ǂꂾ���߂��Ȃ���΂Ȃ�Ȃ������`���܂��B [�����F> 0]
	float collisionQueryRange;

	//< The path visibility optimization range. [Limit: > 0]
	// �p�X�̉����̍œK���͈́B[�����F > 0]
	float pathOptimizationRange;

	// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
	// �G�[�W�F���g�}�l�[�W�������̃G�[�W�F���g�Ƃ̏Փ˂��������̂ɂǂꂾ���ϋɓI�ɂȂ邩�B [�����F> = 0]
	float separationWeight;

	// Flags that impact steering behavior. (See: #UpdateFlags)
	// �X�e�A�����O����ɉe����^����t���O�B �i�Q�ƁF#UpdateFlags�j
	unsigned char updateFlags;

	// The index of the avoidance configuration to use for the agent.
	// �G�[�W�F���g�Ɏg�p�������\���̃C���f�b�N�X�B
	// [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	unsigned char obstacleAvoidanceType;

	// The index of the query filter used by this agent.
	// ���̃G�[�W�F���g���g�p����N�G���t�B���^�[�̃C���f�b�N�X�B
	unsigned char queryFilterType;

	// User defined data attached to the agent.
	// �G�[�W�F���g�ɓY�t���ꂽ���[�U�[��`�f�[�^�B
	void* userData;
};

enum MoveRequestState
{
	DT_CROWDAGENT_TARGET_NONE = 0,
	DT_CROWDAGENT_TARGET_FAILED,
	DT_CROWDAGENT_TARGET_VALID,
	DT_CROWDAGENT_TARGET_REQUESTING,
	DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE,
	DT_CROWDAGENT_TARGET_WAITING_FOR_PATH,
	DT_CROWDAGENT_TARGET_VELOCITY,
};

// Represents an agent managed by a #dtCrowd object.
// @ingroup crowd
struct dtCrowdAgent
{
	// True if the agent is active, false if the agent is in an unused slot in the agent pool.
	// �G�[�W�F���g���A�N�e�B�u�ȏꍇ��true�A�G�[�W�F���g���G�[�W�F���g�v�[���̖��g�p�̃X���b�g�ɂ���ꍇ��false�B
	bool active;

	// �G�[�W�F���g�������Ԃɂ���ꍇ��true�A����s�̏ꍇ��false
	bool is_run;

	// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
	// �G�[�W�F���g���ʉ߂��郁�b�V���|���S���̃^�C�v�B �i�Q�ƁF#CrowdAgentState�j
	unsigned char state;

	// True if the agent has valid path (targetState == DT_CROWDAGENT_TARGET_VALID) and the path does not lead to the requested position, else false.
	// �G�[�W�F���g�ɗL���ȃp�X�itargetState == DT_CROWDAGENT_TARGET_VALID�j������A�p�X���v�����ꂽ�ʒu�ɂȂ���Ȃ��ꍇ��true�A����ȊO�̏ꍇ��false�B
	bool partial;

	// The path corridor the agent is using.
	// �G�[�W�F���g���g�p���Ă���p�X�R���h�[�B
	dtPathCorridor corridor;

	// The local boundary data for the agent.
	// �G�[�W�F���g�̃��[�J�����E�f�[�^�B
	dtLocalBoundary boundary;

	// Time since the agent's path corridor was optimized.
	// �G�[�W�F���g�̃p�X�R���h�[���œK������Ă���̎��ԁB
	float topologyOptTime;

	// The known neighbors of the agent.
	// �G�[�W�F���g�̊��m�̕t�߂̃G�[�W�F���g�B
	dtCrowdNeighbour neis[DT_CROWDAGENT_MAX_NEIGHBOURS];

	// The number of neighbors.
	// �t�߂̃G�[�W�F���g�̐��B
	int nneis;

	// The desired speed.
	// �]�܂������x�B
	float desiredSpeed;

	// The current agent position. [(x, y, z)]
	// ���݂̃G�[�W�F���g�̈ʒu�B [�ix�Ay�Az�j]
	float npos[3];

	//< A temporary value used to accumulate agent displacement during iterative collision resolution. [(x, y, z)]
	// �����I�ȏՓˉ������ɃG�[�W�F���g�̕ψʂ�ݐς��邽�߂Ɏg�p�����ꎞ�I�Ȓl�B [�ix�Ay�Az�j]
	float disp[3];

	//< The desired velocity of the agent. Based on the current path, calculated from scratch each frame. [(x, y, z)]
	// �G�[�W�F���g�̖]�܂������x�B�e�t���[�����[������v�Z�������݂̃p�X�Ɋ�Â��܂��B [�ix�Ay�Az�j]
	float dvel[3];

	//< The desired velocity adjusted by obstacle avoidance, calculated from scratch each frame. [(x, y, z)]
	// �e�t���[�����[������v�Z�����A��Q������ɂ���Ē������ꂽ�ړI�̑��x�B [�ix�Ay�Az�j]
	float nvel[3];

	//< The actual velocity of the agent. The change from nvel -> vel is constrained by max acceleration. [(x, y, z)]
	// �G�[�W�F���g�̎��ۂ̑��x�B nvel-> vel����̕ύX�́A�ő�����x�ɂ���Đ��񂳂�܂��B [�ix�Ay�Az�j]
	float vel[3];

	// The agent's configuration parameters.
	// �G�[�W�F���g�̍\���p�����[�^�B
	dtCrowdAgentParams params;

	// The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
	// �G�[�W�F���g�̃��[�J���p�X�R���h�[�R�[�i�[�B �i���������B�j[�ix�Ay�Az�j* #ncorners]
	float cornerVerts[DT_CROWDAGENT_MAX_CORNERS * 3];

	// The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
	// ���[�J���p�X�̃R���h�[�R�[�i�[�t���O�B �i�Q�ƁF#dtStraightPathFlags�j[�i�t���O�j* #ncorners]
	unsigned char cornerFlags[DT_CROWDAGENT_MAX_CORNERS];

	// The reference id of the polygon being entered at the corner. [(polyRef) * #ncorners]
	// �R�[�i�[�ɓ��͂����|���S���̎Q��ID�B [�ipolyRef�j* #ncorners]
	dtPolyRef cornerPolys[DT_CROWDAGENT_MAX_CORNERS];

	// The number of corners.
	// �R�[�i�[�̐��B
	int ncorners;

	//< State of the movement request.
	// �ړ����N�G�X�g�̏�ԁB
	unsigned char targetState;

	//< Target polyref of the movement request.
	// �ړ����N�G�X�g�̃^�[�Q�b�g�|�����t�@�����X�B
	dtPolyRef targetRef;

	//< Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).
	// �ړ����N�G�X�g�̃^�[�Q�b�g�ʒu�iDT_CROWDAGENT_TARGET_VELOCITY�̏ꍇ�͑��x�j
	float targetPos[3];

	//< Path finder ref.
	// �p�X�t�@�C���_�[�̎Q�ƁB
	dtPathQueueRef targetPathqRef;

	//< Flag indicating that the current path is being replanned.
	// ���݂̃p�X���Čv�悳��Ă��邱�Ƃ������t���O�B
	bool targetReplan;

	// <Time since the agent's target was replanned.
	// �G�[�W�F���g�̃^�[�Q�b�g���Čv�悳��Ă���̎��ԁB
	float targetReplanTime;
};

struct dtCrowdAgentAnimation
{
	bool active;
	float initPos[3], startPos[3], endPos[3];
	dtPolyRef polyRef;
	float t, tmax;
};

// Crowd agent update flags.
// �Q�O�G�[�W�F���g�X�V�t���O�B
// @ingroup crowd
// @see dtCrowdAgentParams::updateFlags
enum UpdateFlags
{
	DT_CROWD_ANTICIPATE_TURNS = 1,
	DT_CROWD_OBSTACLE_AVOIDANCE = 2,
	DT_CROWD_SEPARATION = 4,
	// Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
	// #dtPathCorridor :: optimizePathVisibility�i�j���g�p���āA�G�[�W�F���g�p�X���œK�����܂��B
	DT_CROWD_OPTIMIZE_VIS = 8,
	// Use dtPathCorridor::optimizePathTopology() to optimize the agent path.
	// dtPathCorridor :: optimizePathTopology�i�j���g�p���āA�G�[�W�F���g�p�X���œK�����܂��B
	DT_CROWD_OPTIMIZE_TOPO = 16,
};

struct dtCrowdAgentDebugInfo
{
	int idx;
	float optStart[3], optEnd[3];
	dtObstacleAvoidanceDebugData* vod;
};

// Provides local steering behaviors for a group of agents.
// �G�[�W�F���g�̃O���[�v�Ƀ��[�J���X�e�A�����O�����񋟂��܂��B
// @ingroup crowd
class dtCrowd
{
	int m_maxAgents;
	dtCrowdAgent* m_agents;
	dtCrowdAgent** m_activeAgents;
	dtCrowdAgentAnimation* m_agentAnims;

	dtPathQueue m_pathq;

	dtObstacleAvoidanceParams m_obstacleQueryParams[DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS];
	dtObstacleAvoidanceQuery* m_obstacleQuery;

	dtProximityGrid* m_grid;

	dtPolyRef* m_pathResult;
	int m_maxPathResult;

	float m_agentPlacementHalfExtents[3];

	dtQueryFilter m_filters[DT_CROWD_MAX_QUERY_FILTER_TYPE];

	float m_maxAgentRadius;

	int m_velocitySampleCount;

	dtNavMeshQuery* m_navquery;

	void updateTopologyOptimization(dtCrowdAgent** agents, const int nagents, const float dt);
	void updateMoveRequest(const float dt);
	void checkPathValidity(dtCrowdAgent** agents, const int nagents, const float dt);

	inline int getAgentIndex(const dtCrowdAgent* agent) const { return (int)(agent - m_agents); }

	bool requestMoveTargetReplan(const int idx, dtPolyRef ref, const float* pos);

	void purge();

public:
	dtCrowd();
	~dtCrowd();

	// Initializes the crowd.
	// �Q�W�����������܂��B
	//  @param[in] maxAgents	The maximum number of agents the crowd can manage. [Limit: >= 1]
	//  �Q�W���Ǘ��ł���G�[�W�F���g�̍ő吔�B
	//  @param[in] maxAgentRadius	The maximum radius of any agent that will be added to the crowd. [Limit: > 0]
	//  �Q�W�ɒǉ������G�[�W�F���g�̍ő唼�a�B
	//  @param[in] nav		The navigation mesh to use for planning.
	//  �v��Ɏg�p����i�r�Q�[�V�������b�V���B
	// @return	True if the initialization succeeded.
	// �����������������ꍇ��True�B
	bool init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav);

	// Sets the shared avoidance configuration for the specified index.
	//�w�肳�ꂽ�C���f�b�N�X�̋��L����\����ݒ肵�܂��B
	//  @param[in] idx	The index. [Limits: 0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	//  �C���f�b�N�X�B
	//  @param[in] params	The new configuration.
	//  �V�����ݒ�B
	void setObstacleAvoidanceParams(const int idx, const dtObstacleAvoidanceParams* params);

	// Gets the shared avoidance configuration for the specified index.
	//  @param[in]		idx		The index of the configuration to retreive.
	//							[Limits:  0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	// @return The requested configuration.
	const dtObstacleAvoidanceParams* getObstacleAvoidanceParams(const int idx) const;

	// Gets the specified agent from the pool.
	//	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	// @return The requested agent.
	const dtCrowdAgent* getAgentAt(const int idx) const;

	const dtCrowdAgent* getAgent() const noexcept { return m_agents; }

	// Gets the specified agent from the pool.
	//	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	// @return The requested agent.
	dtCrowdAgent* getEditableAgentAt(const int idx);

	dtCrowdAgent* getEditableAgent() noexcept { return m_agents; }

	// The maximum number of agents that can be managed by the object.
	// @return The maximum number of agents.
	int getAgentCount() const noexcept { return m_maxAgents; };

	// Adds a new agent to the crowd.
	//  @param[in]		pos		The requested position of the agent. [(x, y, z)]
	//  @param[in]		params	The configutation of the agent.
	// @return The index of the agent in the agent pool. Or -1 if the agent could not be added.
	int addAgent(const std::array<float, 3>& pos, const dtCrowdAgentParams* params);

	// Updates the specified agent's configuration.
	//  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	//  @param[in]		params	The new agent configuration.
	void updateAgentParameters(const int idx, const dtCrowdAgentParams* params);

	// Removes the agent from the crowd.
	//  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	void removeAgent(const int idx) noexcept;

	void SetRunning(const int idx, const bool is_running) noexcept;
	bool IsRunning(const int idx) const noexcept;

	// Submits a new move request for the specified agent.
	//  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	//  @param[in]		ref		The position's polygon reference.
	//  @param[in]		pos		The position within the polygon. [(x, y, z)]
	// @return True if the request was successfully submitted.
	bool requestMoveTarget(const int idx, dtPolyRef ref, const float* pos);

	// Submits a new move request for the specified agent.
	//  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	//  @param[in]		vel		The movement velocity. [(x, y, z)]
	// @return True if the request was successfully submitted.
	bool requestMoveVelocity(const int idx, const float* vel);

	// Resets any request for the specified agent.
	//  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	// @return True if the request was successfully reseted.
	bool resetMoveTarget(const int idx);

	// Gets the active agents int the agent pool.
	//  @param[out]	agents		An array of agent pointers. [(#dtCrowdAgent *) * maxAgents]
	//  @param[in]		maxAgents	The size of the crowd agent array.
	// @return The number of agents returned in @p agents.
	int getActiveAgents(dtCrowdAgent** agents, const int maxAgents);

	// Updates the steering and positions of all agents.
	//  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
	//  @param[out]	debug	A debug object to load with debug information. [Opt]
	void update(const float dt, dtCrowdAgentDebugInfo* debug);

	// Gets the filter used by the crowd.
	// @return The filter used by the crowd.
	inline const dtQueryFilter* getFilter(const int i) const { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }

	// Gets the filter used by the crowd.
	// @return The filter used by the crowd.
	inline dtQueryFilter* getEditableFilter(const int i) { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }

	/// Gets the search halfExtents [(x, y, z)] used by the crowd for query operations.
	/// @return The search halfExtents used by the crowd. [(x, y, z)]
	const float* getQueryHalfExtents() const { return m_agentPlacementHalfExtents; }

	/// Same as getQueryHalfExtents. Left to maintain backwards compatibility.
	/// @return The search halfExtents used by the crowd. [(x, y, z)]
	const float* getQueryExtents() const { return m_agentPlacementHalfExtents; }

	/// Gets the velocity sample count.
	/// @return The velocity sample count.
	inline int getVelocitySampleCount() const { return m_velocitySampleCount; }

	// Gets the crowd's proximity grid.
	// @return The crowd's proximity grid.
	const dtProximityGrid* getGrid() const { return m_grid; }

	// Gets the crowd's path request queue.
	// @return The crowd's path request queue.
	const dtPathQueue* getPathQueue() const { return &m_pathq; }

	// Gets the query object used by the crowd.
	const dtNavMeshQuery* getNavMeshQuery() const { return m_navquery; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtCrowd(const dtCrowd&) = delete;
	dtCrowd& operator=(const dtCrowd&) = delete;
};

// Allocates a crowd object using the Detour allocator.
// @return A crowd object that is ready for initialization, or null on failure.
//  @ingroup crowd
dtCrowd* dtAllocCrowd();

// Frees the specified crowd object using the Detour allocator.
//  @param[in]		ptr		A crowd object allocated using #dtAllocCrowd
//  @ingroup crowd
void dtFreeCrowd(dtCrowd* ptr);

#endif // DETOURCROWD_H

//////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

/**

@defgroup crowd Crowd

Members in this module implement local steering and dynamic avoidance features.

The crowd is the big beast of the navigation features. It not only handles a
lot of the path management for you, but also local steering and dynamic
avoidance between members of the crowd. I.e. It can keep your agents from
running into each other.

Main class: #dtCrowd

The #dtNavMeshQuery and #dtPathCorridor classes provide perfectly good, easy
to use path planning features. But in the end they only give you points that
your navigation client should be moving toward. When it comes to deciding things
like agent velocity and steering to avoid other agents, that is up to you to
implement. Unless, of course, you decide to use #dtCrowd.

Basically, you add an agent to the crowd, providing various configuration
settings such as maximum speed and acceleration. You also provide a local
target to more toward. The crowd manager then provides, with every update, the
new agent position and velocity for the frame. The movement will be
constrained to the navigation mesh, and steering will be applied to ensure
agents managed by the crowd do not collide with each other.

This is very powerful feature set. But it comes with limitations.

The biggest limitation is that you must give control of the agent's position
completely over to the crowd manager. You can update things like maximum speed
and acceleration. But in order for the crowd manager to do its thing, it can't
allow you to constantly be giving it overrides to position and velocity. So
you give up direct control of the agent's movement. It belongs to the crowd.

The second biggest limitation revolves around the fact that the crowd manager
deals with local planning. So the agent's target should never be more than
256 polygons aways from its current position. If it is, you risk
your agent failing to reach its target. So you may still need to do long
distance planning and provide the crowd manager with intermediate targets.

Other significant limitations:

- All agents using the crowd manager will use the same #dtQueryFilter.
- Crowd management is relatively expensive. The maximum agents under crowd
  management at any one time is between 20 and 30.  A good place to start
  is a maximum of 25 agents for 0.5ms per frame.

@note This is a summary list of members.  Use the index or search
feature to find minor members.

@struct dtCrowdAgentParams
@see dtCrowdAgent, dtCrowd::addAgent(), dtCrowd::updateAgentParameters()

@var dtCrowdAgentParams::obstacleAvoidanceType
@par

#dtCrowd permits agents to use different avoidance configurations.  This value
is the index of the #dtObstacleAvoidanceParams within the crowd.

@see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(),
	 dtCrowd::getObstacleAvoidanceParams()

@var dtCrowdAgentParams::collisionQueryRange
@par

Collision elements include other agents and navigation mesh boundaries.

This value is often based on the agent radius and/or maximum speed. E.g. radius * 8

@var dtCrowdAgentParams::pathOptimizationRange
@par

Only applicalbe if #updateFlags includes the #DT_CROWD_OPTIMIZE_VIS flag.

This value is often based on the agent radius. E.g. radius * 30

@see dtPathCorridor::optimizePathVisibility()

@var dtCrowdAgentParams::separationWeight
@par

A higher value will result in agents trying to stay farther away from each other at
the cost of more difficult steering in tight spaces.

*/

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
// クラウドエージェントがステアリング決定のために考慮できる付近のエージェントの最大数。
// @ingroup crowd
constexpr int DT_CROWDAGENT_MAX_NEIGHBOURS = 6;

// The maximum number of corners a crowd agent will look ahead in the path.
// クラウドエージェントがパスで先を見るコーナーの最大数。
// This value is used for sizing the crowd agent corner buffers.
// この値は、クラウドエージェントコーナーバッファーのサイズ設定に使用されます。
// Due to the behavior of the crowd manager, the actual number of useful corners will be one less than this number.
// クラウドマネージャーの動作により、有用なコーナーの実際の数はこの数よりも1つ少なくなります。
// @ingroup crowd
constexpr int DT_CROWDAGENT_MAX_CORNERS = 4;

// The maximum number of crowd avoidance configurations supported by the crowd manager.
// クラウドマネージャーがサポートするクラウド回避構成の最大数。
// @ingroup crowd
// @see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(), dtCrowd::getObstacleAvoidanceParams(),
//		 dtCrowdAgentParams::obstacleAvoidanceType
constexpr int DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS = 8;

// The maximum number of query filter types supported by the crowd manager.
// クラウドマネージャーがサポートするクエリフィルタータイプの最大数。
// @ingroup crowd
// @see dtQueryFilter, dtCrowd::getFilter() dtCrowd::getEditableFilter(),
//		dtCrowdAgentParams::queryFilterType
constexpr int DT_CROWD_MAX_QUERY_FILTER_TYPE = 16;

// Provides neighbor data for agents managed by the crowd.
// 群衆が管理するエージェントの近隣データを提供します。
// @ingroup crowd
// @see dtCrowdAgent::neis, dtCrowd
struct dtCrowdNeighbour
{
	int idx;		// The index of the neighbor in the crowd. // 群衆の中の隣のインデックス。
	float dist;		// The distance between the current agent and the neighbor. // 現在のエージェントと隣人の間の距離。
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
	// ステアリング動作と見なされる前に衝突要素がどれだけ近くなければならないかを定義します。 [制限：> 0]
	float collisionQueryRange;

	//< The path visibility optimization range. [Limit: > 0]
	// パスの可視化の最適化範囲。[制限： > 0]
	float pathOptimizationRange;

	// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
	// エージェントマネージャがこのエージェントとの衝突を回避するのにどれだけ積極的になるか。 [制限：> = 0]
	float separationWeight;

	// Flags that impact steering behavior. (See: #UpdateFlags)
	// ステアリング動作に影響を与えるフラグ。 （参照：#UpdateFlags）
	unsigned char updateFlags;

	// The index of the avoidance configuration to use for the agent.
	// エージェントに使用する回避構成のインデックス。
	// [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	unsigned char obstacleAvoidanceType;

	// The index of the query filter used by this agent.
	// このエージェントが使用するクエリフィルターのインデックス。
	unsigned char queryFilterType;

	// User defined data attached to the agent.
	// エージェントに添付されたユーザー定義データ。
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
	// エージェントがアクティブな場合はtrue、エージェントがエージェントプールの未使用のスロットにある場合はfalse。
	bool active;

	// エージェントが動作状態にある場合はtrue、動作不可の場合はfalse
	bool is_run;

	// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
	// エージェントが通過するメッシュポリゴンのタイプ。 （参照：#CrowdAgentState）
	unsigned char state;

	// True if the agent has valid path (targetState == DT_CROWDAGENT_TARGET_VALID) and the path does not lead to the requested position, else false.
	// エージェントに有効なパス（targetState == DT_CROWDAGENT_TARGET_VALID）があり、パスが要求された位置につながらない場合はtrue、それ以外の場合はfalse。
	bool partial;

	// The path corridor the agent is using.
	// エージェントが使用しているパスコリドー。
	dtPathCorridor corridor;

	// The local boundary data for the agent.
	// エージェントのローカル境界データ。
	dtLocalBoundary boundary;

	// Time since the agent's path corridor was optimized.
	// エージェントのパスコリドーが最適化されてからの時間。
	float topologyOptTime;

	// The known neighbors of the agent.
	// エージェントの既知の付近のエージェント。
	dtCrowdNeighbour neis[DT_CROWDAGENT_MAX_NEIGHBOURS];

	// The number of neighbors.
	// 付近のエージェントの数。
	int nneis;

	// The desired speed.
	// 望ましい速度。
	float desiredSpeed;

	// The current agent position. [(x, y, z)]
	// 現在のエージェントの位置。 [（x、y、z）]
	float npos[3];

	//< A temporary value used to accumulate agent displacement during iterative collision resolution. [(x, y, z)]
	// 反復的な衝突解決中にエージェントの変位を累積するために使用される一時的な値。 [（x、y、z）]
	float disp[3];

	//< The desired velocity of the agent. Based on the current path, calculated from scratch each frame. [(x, y, z)]
	// エージェントの望ましい速度。各フレームをゼロから計算した現在のパスに基づきます。 [（x、y、z）]
	float dvel[3];

	//< The desired velocity adjusted by obstacle avoidance, calculated from scratch each frame. [(x, y, z)]
	// 各フレームをゼロから計算した、障害物回避によって調整された目的の速度。 [（x、y、z）]
	float nvel[3];

	//< The actual velocity of the agent. The change from nvel -> vel is constrained by max acceleration. [(x, y, z)]
	// エージェントの実際の速度。 nvel-> velからの変更は、最大加速度によって制約されます。 [（x、y、z）]
	float vel[3];

	// The agent's configuration parameters.
	// エージェントの構成パラメータ。
	dtCrowdAgentParams params;

	// The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
	// エージェントのローカルパスコリドーコーナー。 （正しい道。）[（x、y、z）* #ncorners]
	float cornerVerts[DT_CROWDAGENT_MAX_CORNERS * 3];

	// The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
	// ローカルパスのコリドーコーナーフラグ。 （参照：#dtStraightPathFlags）[（フラグ）* #ncorners]
	unsigned char cornerFlags[DT_CROWDAGENT_MAX_CORNERS];

	// The reference id of the polygon being entered at the corner. [(polyRef) * #ncorners]
	// コーナーに入力されるポリゴンの参照ID。 [（polyRef）* #ncorners]
	dtPolyRef cornerPolys[DT_CROWDAGENT_MAX_CORNERS];

	// The number of corners.
	// コーナーの数。
	int ncorners;

	//< State of the movement request.
	// 移動リクエストの状態。
	unsigned char targetState;

	//< Target polyref of the movement request.
	// 移動リクエストのターゲットポリリファレンス。
	dtPolyRef targetRef;

	//< Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).
	// 移動リクエストのターゲット位置（DT_CROWDAGENT_TARGET_VELOCITYの場合は速度）
	float targetPos[3];

	//< Path finder ref.
	// パスファインダーの参照。
	dtPathQueueRef targetPathqRef;

	//< Flag indicating that the current path is being replanned.
	// 現在のパスが再計画されていることを示すフラグ。
	bool targetReplan;

	// <Time since the agent's target was replanned.
	// エージェントのターゲットが再計画されてからの時間。
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
// 群衆エージェント更新フラグ。
// @ingroup crowd
// @see dtCrowdAgentParams::updateFlags
enum UpdateFlags
{
	DT_CROWD_ANTICIPATE_TURNS = 1,
	DT_CROWD_OBSTACLE_AVOIDANCE = 2,
	DT_CROWD_SEPARATION = 4,
	// Use #dtPathCorridor::optimizePathVisibility() to optimize the agent path.
	// #dtPathCorridor :: optimizePathVisibility（）を使用して、エージェントパスを最適化します。
	DT_CROWD_OPTIMIZE_VIS = 8,
	// Use dtPathCorridor::optimizePathTopology() to optimize the agent path.
	// dtPathCorridor :: optimizePathTopology（）を使用して、エージェントパスを最適化します。
	DT_CROWD_OPTIMIZE_TOPO = 16,
};

struct dtCrowdAgentDebugInfo
{
	int idx;
	float optStart[3], optEnd[3];
	dtObstacleAvoidanceDebugData* vod;
};

// Provides local steering behaviors for a group of agents.
// エージェントのグループにローカルステアリング動作を提供します。
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
	// 群集を初期化します。
	//  @param[in] maxAgents	The maximum number of agents the crowd can manage. [Limit: >= 1]
	//  群集が管理できるエージェントの最大数。
	//  @param[in] maxAgentRadius	The maximum radius of any agent that will be added to the crowd. [Limit: > 0]
	//  群集に追加されるエージェントの最大半径。
	//  @param[in] nav		The navigation mesh to use for planning.
	//  計画に使用するナビゲーションメッシュ。
	// @return	True if the initialization succeeded.
	// 初期化が成功した場合はTrue。
	bool init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav);

	// Sets the shared avoidance configuration for the specified index.
	//指定されたインデックスの共有回避構成を設定します。
	//  @param[in] idx	The index. [Limits: 0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	//  インデックス。
	//  @param[in] params	The new configuration.
	//  新しい設定。
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

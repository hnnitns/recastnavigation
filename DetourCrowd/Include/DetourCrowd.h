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

#include "DetourNavMeshQuery.h"
#include "DetourObstacleAvoidance.h"
#include "DetourLocalBoundary.h"
#include "DetourPathCorridor.h"
#include "DetourProximityGrid.h"
#include "DetourPathQueue.h"

// The maximum number of neighbors that a crowd agent can take into account for steering decisions.
// クラウドエージェントがステアリング決定のために考慮できるネイバーの最大数。
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

// The type of navigation mesh polygon the agent is currently traversing.
// エージェントが現在通過しているナビゲーションメッシュポリゴンのタイプ。
// @ingroup crowd
enum CrowdAgentState
{
	// The agent is not in a valid state.
	// エージェントは有効な状態ではありません。
	DT_CROWDAGENT_STATE_INVALID,
	// The agent is traversing a normal navigation mesh polygon.
	// エージェントは通常のナビゲーションメッシュポリゴンを通過します。
	DT_CROWDAGENT_STATE_WALKING,
	// The agent is traversing an off-mesh connection.
	// エージェントはオフメッシュ接続を通過します。
	DT_CROWDAGENT_STATE_OFFMESH,
};

// Configuration parameters for a crowd agent.
// @ingroup crowd
struct dtCrowdAgentParams
{
	float radius;						//< Agent radius. [Limit: >= 0]
	float height;						//< Agent height. [Limit: > 0]
	float maxAcceleration;				//< Maximum allowed acceleration. [Limit: >= 0]
	float maxSpeed;						//< Maximum allowed speed. [Limit: >= 0]

	// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
	float collisionQueryRange;

	float pathOptimizationRange;		//< The path visibility optimization range. [Limit: > 0]

	// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
	float separationWeight;

	// Flags that impact steering behavior. (See: #UpdateFlags)
	uint8_t updateFlags;

	// The index of the avoidance configuration to use for the agent.
	// [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	uint8_t obstacleAvoidanceType;

	// The index of the query filter used by this agent.
	uint8_t queryFilterType;

	// User defined data attached to the agent.
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
	bool active;

	// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)
	uint8_t state;

	// True if the agent has valid path (targetState == DT_CROWDAGENT_TARGET_VALID) and the path does not lead to the requested position, else false.
	bool partial;

	// The path corridor the agent is using.
	dtPathCorridor corridor;

	// The local boundary data for the agent.
	dtLocalBoundary boundary;

	// Time since the agent's path corridor was optimized.
	float topologyOptTime;

	// The known neighbors of the agent.
	std::array<dtCrowdNeighbour, DT_CROWDAGENT_MAX_NEIGHBOURS> neis;

	// The number of neighbors.
	int nneis;

	// The desired speed.
	float desiredSpeed;

	std::array<float, 3> npos;		//< The current agent position. [(x, y, z)]
	std::array<float, 3> disp;		//< A temporary value used to accumulate agent displacement during iterative collision resolution. [(x, y, z)]
	std::array<float, 3> dvel;		//< The desired velocity of the agent. Based on the current path, calculated from scratch each frame. [(x, y, z)]
	std::array<float, 3> nvel;		//< The desired velocity adjusted by obstacle avoidance, calculated from scratch each frame. [(x, y, z)]
	std::array<float, 3> vel;		//< The actual velocity of the agent. The change from nvel -> vel is constrained by max acceleration. [(x, y, z)]

	// The agent's configuration parameters.
	dtCrowdAgentParams params;

	// The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
	std::array<float, DT_CROWDAGENT_MAX_CORNERS * 3> cornerVerts;

	// The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
	std::array<uint8_t, DT_CROWDAGENT_MAX_CORNERS> cornerFlags;

	// The reference id of the polygon being entered at the corner. [(polyRef) * #ncorners]
	std::array<dtPolyRef, DT_CROWDAGENT_MAX_CORNERS> cornerPolys;

	// The number of corners.
	int ncorners;

	uint8_t targetState;			//< State of the movement request.
	dtPolyRef targetRef;			//< Target polyref of the movement request.
	std::array<float, 3> targetPos;	//< Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).
	dtPathQueueRef targetPathqRef;		//< Path finder ref.
	bool targetReplan;					//< Flag indicating that the current path is being replanned.
	float targetReplanTime;				// <Time since the agent's target was replanned.
};

struct dtCrowdAgentAnimation
{
	bool active;
	std::array<float, 3> initPos, startPos, endPos;
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
	std::array<float, 3> optStart, optEnd;
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

	std::array<dtObstacleAvoidanceParams, DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS> m_obstacleQueryParams;
	dtObstacleAvoidanceQuery* m_obstacleQuery;

	dtProximityGrid* m_grid;

	dtPolyRef* m_pathResult;
	int m_maxPathResult;

	std::array<float, 3> m_ext;

	std::array<dtQueryFilter, DT_CROWD_MAX_QUERY_FILTER_TYPE> m_filters;

	float m_maxAgentRadius;

	int m_velocitySampleCount;

	dtNavMeshQuery* m_navquery;

	void updateTopologyOptimization(dtCrowdAgent** agents, const int nagents, const float dt);
	void updateMoveRequest(const float dt);
	void checkPathValidity(dtCrowdAgent** agents, const int nagents, const float dt);

	inline int getAgentIndex(const dtCrowdAgent* agent) const { return (int)(agent - m_agents); }

	bool requestMoveTargetReplan(const int idx, dtPolyRef ref, const std::array<float, 3>& pos);

	void purge();

public:
	dtCrowd();
	~dtCrowd();

	// Initializes the crowd.
	// 群集を初期化します。
	//  @param[in] maxAgents : The maximum number of agents the crowd can manage. [Limit: >= 1]
	//  クラウドが管理できるエージェントの最大数。 [制限：> = 1]
	//  @param[in] maxAgentRadius : The maximum radius of any agent that will be added to the crowd. [Limit: > 0]
	//  群衆に追加されるエージェントの最大半径。 [制限：> 0]
	//  @param[in] nav : The navigation mesh to use for planning.
	//  計画に使用するナビゲーションメッシュ。
	// @return True if the initialization succeeded.
	// 初期化が成功した場合はtrue。
	bool init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav);

	// Sets the shared avoidance configuration for the specified index.
	// 指定されたインデックスの共有回避設定を設定します。
	//  @param[in]		idx		The index. [Limits: 0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	//  @param[in]		params	The new configuration.
	void setObstacleAvoidanceParams(const int idx, const dtObstacleAvoidanceParams* params);

	// Gets the shared avoidance configuration for the specified index.
	// 指定されたインデックスの共有回避設定を取得します。
	//  @param[in]		idx		The index of the configuration to retreive.
	//							[Limits:  0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	// @return The requested configuration.
	const dtObstacleAvoidanceParams* getObstacleAvoidanceParams(const int idx) const;

	// Gets the specified agent from the pool.
	// 指定されたエージェントをプールから取得します。
	//	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	// @return The requested agent.
	const dtCrowdAgent* getAgent(const int idx);

	// Gets the specified agent from the pool.
	// 指定されたエージェントをプールから取得します。
	//	 @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	// @return The requested agent.
	dtCrowdAgent* getEditableAgent(const int idx);

	// The maximum number of agents that can be managed by the object.
	// オブジェクトで管理できるエージェントの最大数。
	// @return The maximum number of agents.
	int getAgentCount() const;

	// Adds a new agent to the crowd.
	// 群衆に新しいエージェントを追加します。
	//  @param[in]		pos		The requested position of the agent. [(x, y, z)]
	//  @param[in]		params	The configutation of the agent.
	// @return The index of the agent in the agent pool. Or -1 if the agent could not be added.
	int addAgent(const float* pos, const dtCrowdAgentParams* params);

	// Updates the specified agent's configuration.
	// 指定されたエージェントの構成を更新します。
	//  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	//  @param[in]		params	The new agent configuration.
	void updateAgentParameters(const int idx, const dtCrowdAgentParams* params);

	// Removes the agent from the crowd.
	// 群衆からエージェントを削除します。
	//  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	void removeAgent(const int idx);

	// Submits a new move request for the specified agent.
	// 指定されたエージェントの新しい移動要求を送信します。
	//  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	//  @param[in]		ref		The position's polygon reference.
	//  @param[in]		pos		The position within the polygon. [(x, y, z)]
	// @return True if the request was successfully submitted.
	bool requestMoveTarget(const int idx, dtPolyRef ref, const float* pos);

	// Submits a new move request for the specified agent.
	// 指定されたエージェントの新しい移動要求を送信します。
	//  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	//  @param[in]		vel		The movement velocity. [(x, y, z)]
	// @return True if the request was successfully submitted.
	bool requestMoveVelocity(const int idx, const std::array<float, 3>& vel);

	// Resets any request for the specified agent.
	// 指定されたエージェントの要求をリセットします。
	//  @param[in]		idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	// @return True if the request was successfully reseted.
	bool resetMoveTarget(const int idx);

	// Gets the active agents int the agent pool.
	// エージェントプール内のアクティブなエージェントを取得します。
	//  @param[out]	agents		An array of agent pointers. [(#dtCrowdAgent *) * maxAgents]
	//  @param[in]		maxAgents	The size of the crowd agent array.
	// @return The number of agents returned in @p agents.
	int getActiveAgents(dtCrowdAgent** agents, const int maxAgents);

	// Updates the steering and positions of all agents.
	// すべてのエージェントのステアリングと位置を更新します。
	//  @param[in]		dt		The time, in seconds, to update the simulation. [Limit: > 0]
	//  @param[out]	debug	A debug object to load with debug information. [Opt]
	void update(const float dt, dtCrowdAgentDebugInfo* debug);

	// Gets the filter used by the crowd.
	// 群衆が使用するフィルターを取得します。
	// @return The filter used by the crowd.
	inline const dtQueryFilter* getFilter(const int i) const { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }

	// Gets the filter used by the crowd.
	// 群衆が使用するフィルターを取得します。
	// @return The filter used by the crowd.
	inline dtQueryFilter* getEditableFilter(const int i) { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }

	// Gets the search extents [(x, y, z)] used by the crowd for query operations.
	// 群衆がクエリ操作に使用する検索範囲[（x、y、z）]を取得します。
	// @return The search extents used by the crowd. [(x, y, z)]
	//	群衆が使用する検索範囲。 [（x、y、z）]
	const auto& getQueryExtents() const { return m_ext; }

	// Gets the velocity sample count.
	// 速度のサンプルカウントを取得します。
	// @return The velocity sample count.
	inline int getVelocitySampleCount() const { return m_velocitySampleCount; }

	// Gets the crowd's proximity grid.
	// 群衆の近接グリッドを取得します。
	// @return The crowd's proximity grid.
	const dtProximityGrid* getGrid() const { return m_grid; }

	// Gets the crowd's path request queue.
	// 群衆のパスリクエストキューを取得します。
	// @return The crowd's path request queue.
	const dtPathQueue* getPathQueue() const { return &m_pathq; }

	// Gets the query object used by the crowd.
	// 群衆が使用するクエリオブジェクトを取得します。
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
このモジュールのメンバーは、ローカルステアリングおよび動的回避機能を実装します。

The crowd is the big beast of the navigation features. It not only handles a
lot of the path management for you, but also local steering and dynamic
avoidance between members of the crowd. I.e. It can keep your agents from
running into each other.
群衆はナビゲーション機能の大物です。
それはあなたのために多くの経路管理を処理するだけでなく、
群衆のメンバーの間のローカルステアリングと動的な回避も処理します。
つまりエージェントがお互いに実行されないようにすることができます。

Main class: #dtCrowd

The #dtNavMeshQuery and #dtPathCorridor classes provide perfectly good, easy
to use path planning features. But in the end they only give you points that
your navigation client should be moving toward. When it comes to deciding things
like agent velocity and steering to avoid other agents, that is up to you to
implement. Unless, of course, you decide to use #dtCrowd.
#dtNavMeshQueryおよび#dtPathCorridorクラスは、完全に優れた使いやすいパス計画機能を提供します。
ただし、最終的には、ナビゲーションクライアントが移動すべきポイントのみを提供します。
エージェントの速度や他のエージェントを避けるためのステアリングなどを決定する場合、実装するのはあなた次第です。
もちろん、＃dtCrowdを使用することに決めた場合を除きます。

Basically, you add an agent to the crowd, providing various configuration
settings such as maximum speed and acceleration. You also provide a local
target to more toward. The crowd manager then provides, with every update, the
new agent position and velocity for the frame. The movement will be
constrained to the navigation mesh, and steering will be applied to ensure
agents managed by the crowd do not collide with each other.
基本的に、エージェントを群衆に追加し、最大速度や加速などのさまざまな構成設定を提供します。
また、より多くの方にローカルターゲットを提供します。
クラウドマネージャーは、更新のたびに、フレームの新しいエージェントの位置と速度を提供します。
移動はナビゲーションメッシュに制限され、
群衆が管理するエージェントが互いに衝突しないようにステアリングが適用されます。

This is very powerful feature set. But it comes with limitations.
これは非常に強力な機能セットです。ただし、制限があります。

The biggest limitation is that you must give control of the agent's position
completely over to the crowd manager. You can update things like maximum speed
and acceleration. But in order for the crowd manager to do its thing, it can't
allow you to constantly be giving it overrides to position and velocity. So
you give up direct control of the agent's movement. It belongs to the crowd.
最大の制限は、エージェントの位置を完全にクラウドマネージャーに制御する必要があることです。
最大速度や加速などを更新できます。
ただし、クラウドマネージャーがその処理を行うためには、位置と速度のオーバーライドを常に与えることはできません。
したがって、エージェントの動きの直接制御を放棄します。それは群衆に属します。

The second biggest limitation revolves around the fact that the crowd manager
deals with local planning. So the agent's target should never be more than
256 polygons aways from its current position. If it is, you risk
your agent failing to reach its target. So you may still need to do long
distance planning and provide the crowd manager with intermediate targets.
2番目に大きい制限は、クラウドマネージャーがローカル計画を扱うという事実を中心に展開します。
したがって、エージェントのターゲットは、現在の位置から256ポリゴン以上離れてはなりません。
そうである場合、エージェントがターゲットに到達できないリスクがあります。
そのため、長距離の計画を立て、クラウドマネージャーに中間ターゲットを提供する必要があります。

Other significant limitations:
その他の重要な制限：

- All agents using the crowd manager will use the same #dtQueryFilter.
- Crowd management is relatively expensive. The maximum agents under crowd
  management at any one time is between 20 and 30.  A good place to start
  is a maximum of 25 agents for 0.5ms per frame.
- クラウドマネージャーを使用するすべてのエージェントは、同じ#dtQueryFilterを使用します。
- 群衆管理は比較的高価です。クラウド管理下のエージェントの最大数は、常に20〜30です。開始するのに適した場所は、フレームあたり0.5ミリ秒で最大25エージェントです。

@note This is a summary list of members.  Use the index or search
feature to find minor members.
これはメンバーの要約リストです。インデックスまたは検索機能を使用して、マイナーメンバーを見つけます。

@struct dtCrowdAgentParams
@see dtCrowdAgent, dtCrowd::addAgent(), dtCrowd::updateAgentParameters()

@var dtCrowdAgentParams::obstacleAvoidanceType
@par

#dtCrowd permits agents to use different avoidance configurations.  This value
is the index of the #dtObstacleAvoidanceParams within the crowd.
#dtCrowdは、エージェントが異なる回避設定を使用できるようにします。
この値は、クラウド内の#dtObstacleAvoidanceParamsのインデックスです。

@see dtObstacleAvoidanceParams, dtCrowd::setObstacleAvoidanceParams(),
	 dtCrowd::getObstacleAvoidanceParams()

@var dtCrowdAgentParams::collisionQueryRange
@par

Collision elements include other agents and navigation mesh boundaries.
衝突要素には、他のエージェントとナビゲーションメッシュ境界が含まれます。

This value is often based on the agent radius and/or maximum speed. E.g. radius * 8
多くの場合、この値はエージェントの半径や最大速度に基づいています。例えば。半径* 8

@var dtCrowdAgentParams::pathOptimizationRange
@par

Only applicalbe if #updateFlags includes the #DT_CROWD_OPTIMIZE_VIS flag.
#updateFlagsに#DT_CROWD_OPTIMIZE_VISフラグが含まれている場合にのみ適用されます。

This value is often based on the agent radius. E.g. radius * 30
多くの場合、この値はエージェントの半径に基づいています。例えば。半径* 30

@see dtPathCorridor::optimizePathVisibility()

@var dtCrowdAgentParams::separationWeight
@par

A higher value will result in agents trying to stay farther away from each other at
the cost of more difficult steering in tight spaces.
値を大きくすると、エージェントは、狭いスペースでの操縦が難しくなりますが、互いから遠く離れようとします。

*/

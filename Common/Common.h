#pragma once

#include <array>

struct RaycastMeshHitInfo
{
	float dis{}; // 衝突地点までの距離
	std::array<float, 3> pos{}; // 衝突地点
	std::array<float, 3> vec{}; // 衝突へのベクトル
};

struct dtObstacleCylinder
{
	float pos[3], before_pos[3];
	float radius;
	float height;
};

struct dtObstacleBox
{
	float bmin[3], bmax[3];
	float before_bmin[3], before_bmax[3];
};

struct dtObstacleOrientedBox
{
	float center[3];
	float halfExtents[3];
	float rotAux[2]; //{ cos(0.5f*angle)*sin(-0.5f*angle); cos(0.5f*angle)*cos(0.5f*angle) - 0.5 }
	float y_radian;
};

enum ObstacleType
{
	DT_OBSTACLE_CYLINDER,
	DT_OBSTACLE_BOX, // AABB
	DT_OBSTACLE_ORIENTED_BOX, // OBB
};

struct AddObstacleData
{
	dtObstacleCylinder cylinder;
	dtObstacleBox box;
	dtObstacleOrientedBox oriented_box;
	ObstacleType type;
};

struct AddAgentStruct
{
	std::array<float, 3> pos{}; // 現在座標
	float radius{}; // 半径
	float height{}; // 高さ
	float max_accele{ 8.f }; // 最大加速度
	float max_speed{ 3.5f }; // 最大速度
	float collision_range{ 12.f }; // ステアリング動作と見なされる前に衝突要素がどれだけ近くなければならないか
	float path_optimization_range{ 30.f }; // パスの可視化の最適化範囲
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
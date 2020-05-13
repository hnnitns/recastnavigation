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

#ifndef CROWDTOOL_H
#define CROWDTOOL_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourObstacleAvoidance.h"
#include "ValueHistory.h"
#include "DetourCrowd.h"

// Tool to create crowds.

struct CrowdToolParams
{
	bool m_expandSelectedDebugDraw;
	bool m_showCorners;
	bool m_showCollisionSegments;
	bool m_showPath;
	bool m_showVO;
	bool m_showOpt;
	bool m_showNeis;

	bool m_expandDebugDraw;
	bool m_showLabels;
	bool m_showGrid;
	bool m_showNodes;
	bool m_showPerfGraph;
	bool m_showDetailAll;

	bool m_expandOptions;
	bool m_anticipateTurns;
	bool m_optimizeVis;
	bool m_optimizeTopo;
	bool m_obstacleAvoidance;
	float m_obstacleAvoidanceType;
	bool m_separation;
	float m_separationWeight;
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

class CrowdToolState : public SampleToolState
{
	Sample* m_sample;
	dtNavMesh* m_nav;
	dtCrowd* m_crowd;

	float m_targetPos[3];
	dtPolyRef m_targetRef;

	dtCrowdAgentDebugInfo m_agentDebug;
	dtObstacleAvoidanceDebugData* m_vod;

	static constexpr int AGENT_MAX_TRAIL = 64;
	static constexpr int MAX_AGENTS = 128;
	struct AgentTrail
	{
		std::array<float, AGENT_MAX_TRAIL * 3> trail;
		int htrail;
	};
	AgentTrail m_trails[MAX_AGENTS];

	ValueHistory m_crowdTotalTime;
	ValueHistory m_crowdSampleCount;

	CrowdToolParams m_toolParams;

	bool is_all_run;

public:
	CrowdToolState();
	virtual ~CrowdToolState();

	virtual void init(class Sample* sample);
	virtual void reset();
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleUpdate(const float dt);

	inline bool IsAllRunning() const { return is_all_run; }
	inline void SetAllRunning(const bool s) { is_all_run = s; }
	inline bool IsRunning(const int idx) noexcept { return m_crowd->IsRunning(idx); }
	inline void SetRunning(const int idx, const bool is_running) const noexcept
	{ m_crowd->SetRunning(idx, is_running); }

	int AddAgent(const AddAgentStruct& add_data);
	void RemoveAgent(const int idx);
	void hilightAgent(const int idx);
	void updateAgentParams();
	int hitTestAgents(const float* s, const float* p);
	void setMoveTarget(const float* pos, bool adjust);
	bool SetMoveTargetAt(const std::array<float, 3>& tgt_pos, const int idx, bool is_velocity_move = false);
	void updateTick(const float dt);

	inline CrowdToolParams* getToolParams() { return &m_toolParams; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	CrowdToolState(const CrowdToolState&) = delete;
	CrowdToolState& operator=(const CrowdToolState&) = delete;
};

class CrowdManager
{
	Sample* m_sample;
	std::unique_ptr<CrowdToolState> m_state;

public:
	CrowdManager(Sample* sample);
	~CrowdManager() = default;

	void Update(const float dt) { m_state->handleUpdate(dt); }
};

class CrowdTool : public SampleTool
{
	Sample* m_sample;
	CrowdToolState* m_state;

	enum ToolMode
	{
		TOOLMODE_CREATE,
		TOOLMODE_MOVE_TARGET,
		TOOLMODE_SELECT,
		TOOLMODE_TOGGLE_POLYS,
	};
	ToolMode m_mode;

public:
	CrowdTool();

	virtual int type() { return TOOL_CROWD; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
};

#endif // CROWDTOOL_H

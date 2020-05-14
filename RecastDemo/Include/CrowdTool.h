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
	std::array<float, 3> pos{}; // Œ»İÀ•W
	float radius{}; // ”¼Œa
	float height{}; // ‚‚³
	float max_accele{ 8.f }; // Å‘å‰Á‘¬“x
	float max_speed{ 3.5f }; // Å‘å‘¬“x
	float collision_range{ 12.f }; // ƒXƒeƒAƒŠƒ“ƒO“®ì‚ÆŒ©‚È‚³‚ê‚é‘O‚ÉÕ“Ë—v‘f‚ª‚Ç‚ê‚¾‚¯‹ß‚­‚È‚¯‚ê‚Î‚È‚ç‚È‚¢‚©
	float path_optimization_range{ 30.f }; // ƒpƒX‚Ì‰Â‹‰»‚ÌÅ“K‰»”ÍˆÍ
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

	inline bool IsAllRunning() const noexcept { return is_all_run; }
	inline void SetAllRunning(const bool s) noexcept { is_all_run = s; }
	inline bool IsRunning(const int idx) noexcept { return m_crowd->IsRunning(idx); }
	inline void SetRunning(const int idx, const bool is_running) const noexcept
	{ m_crowd->SetRunning(idx, is_running); }
	auto* GetAgent(const int idx) const noexcept { return m_crowd->getAgentAt(idx); }
	auto* GetEditableAgent(const int idx) const noexcept { return m_crowd->getEditableAgentAt(idx); }

	int AddAgent(const AddAgentStruct& add_data);
	void RemoveAgent(const int idx);
	void ClearAgent();
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
	CrowdManager(const CrowdManager&) = delete;
	auto& operator=(const CrowdManager&) = delete;
	CrowdManager(CrowdManager&&) = delete;
	auto& operator=(CrowdManager&&) = delete;

	// XV
	void Update(const float dt) { m_state->handleUpdate(dt); }
	// ŒQO‚É’Ç‰Ái’Ç‰Á‚³‚ê‚½ƒG[ƒWƒFƒ“ƒg‚Ì”Ô†F‚±‚Ì”Ô†‚ğIndex‚Æ‚µ‚Äg‚¤‚Ì‚Å•Û‚µ‚Ä‚¨‚­•K—v‚ª‚ ‚éj
	_NODISCARD int AddAgent(const AddAgentStruct& add_data) { return (m_state->AddAgent(add_data)); }
	// ŒQO‚©‚çíœ
	void RemoveAgent(const int index) { m_state->RemoveAgent(index); }
	// ŒQO‚©‚ç‘S‚Ä‚ğíœ
	void ClearAgent() { m_state->ClearAgent(); };
	// ŒQO‚Ìƒ^[ƒQƒbƒgÀ•W‚ğİ’è
	void SetMoveTarget(const int index, const std::array<float, 3>& tgt_pos, const bool is_moove_velocity = false)
	{ m_state->SetMoveTargetAt(tgt_pos, index, is_moove_velocity); }
	// ŒQO‘S‘Ì‚ª“®‚¢‚Ä‚¢‚é‚©‚Ç‚¤‚©
	_NODISCARD bool IsAllRunning() const noexcept { return m_state->IsAllRunning(); }
	// ŒQO‘S‘Ì‚Ì“®‚«‚ğİ’è‚·‚é
	void SetAllRunning(const bool is_run) noexcept { m_state->SetAllRunning(is_run); }
	// ŒQO‚Ìˆê•”‚ª“®‚¢‚Ä‚¢‚é‚©‚Ç‚¤‚©
	_NODISCARD bool IsRunning(const int index) noexcept { return m_state->IsRunning(index); }
	// ŒQO‚Ìˆê•”‚Ì“®‚«‚ğİ’è‚·‚é
	void SetRunning(const int index, const bool is_run) const noexcept { m_state->SetRunning(index, is_run); }
	// ŒQO‚Ìˆê•”‚ÌÀ•W‚ÌÄİ’è
	void SetAgentRePosition(const int index, const std::array<float, 3>& agent_pos) noexcept;
	// ŒQO‚Ìˆê•”‚ÌÀ•W‚Ìæ“¾
	_NODISCARD std::array<float, 3> GetAgentPosition(const int index) const noexcept;
	// ŒQO‚Ìˆê•”‚Ì‘¬“x‚Ìæ“¾
	_NODISCARD std::array<float, 3> GetAgentVelocity(const int index) const noexcept;
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

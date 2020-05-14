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
	std::array<float, 3> pos{}; // ���ݍ��W
	float radius{}; // ���a
	float height{}; // ����
	float max_accele{ 8.f }; // �ő�����x
	float max_speed{ 3.5f }; // �ő呬�x
	float collision_range{ 12.f }; // �X�e�A�����O����ƌ��Ȃ����O�ɏՓ˗v�f���ǂꂾ���߂��Ȃ���΂Ȃ�Ȃ���
	float path_optimization_range{ 30.f }; // �p�X�̉����̍œK���͈�
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

	// �X�V
	void Update(const float dt) { m_state->handleUpdate(dt); }
	// �Q�O�ɒǉ��i�ǉ����ꂽ�G�[�W�F���g�̔ԍ��F���̔ԍ���Index�Ƃ��Ďg���̂ŕێ����Ă����K�v������j
	_NODISCARD int AddAgent(const AddAgentStruct& add_data) { return (m_state->AddAgent(add_data)); }
	// �Q�O����폜
	void RemoveAgent(const int index) { m_state->RemoveAgent(index); }
	// �Q�O����S�Ă��폜
	void ClearAgent() { m_state->ClearAgent(); };
	// �Q�O�̃^�[�Q�b�g���W��ݒ�
	void SetMoveTarget(const int index, const std::array<float, 3>& tgt_pos, const bool is_moove_velocity = false)
	{ m_state->SetMoveTargetAt(tgt_pos, index, is_moove_velocity); }
	// �Q�O�S�̂������Ă��邩�ǂ���
	_NODISCARD bool IsAllRunning() const noexcept { return m_state->IsAllRunning(); }
	// �Q�O�S�̂̓�����ݒ肷��
	void SetAllRunning(const bool is_run) noexcept { m_state->SetAllRunning(is_run); }
	// �Q�O�̈ꕔ�������Ă��邩�ǂ���
	_NODISCARD bool IsRunning(const int index) noexcept { return m_state->IsRunning(index); }
	// �Q�O�̈ꕔ�̓�����ݒ肷��
	void SetRunning(const int index, const bool is_run) const noexcept { m_state->SetRunning(index, is_run); }
	// �Q�O�̈ꕔ�̍��W�̍Đݒ�
	void SetAgentRePosition(const int index, const std::array<float, 3>& agent_pos) noexcept;
	// �Q�O�̈ꕔ�̍��W�̎擾
	_NODISCARD std::array<float, 3> GetAgentPosition(const int index) const noexcept;
	// �Q�O�̈ꕔ�̑��x�̎擾
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

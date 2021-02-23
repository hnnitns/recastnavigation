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

#ifndef RECASTSAMPLE_H
#define RECASTSAMPLE_H

#include <optional>

#include "InputGeom.h"
#include "Recast.h"
#include "SampleInterfaces.h"

// Tool types. // ツールタイプ。
enum SampleToolType
{
	TOOL_NONE = 0,	         // なし
	TOOL_TILE_EDIT,	         // タイル編集
	TOOL_TILE_HIGHLIGHT,     // タイルのハイライト
	TOOL_TEMP_OBSTACLE,	     // 一時的な障害
	TOOL_NAVMESH_TESTER,     // ナビメッシュテスター
	TOOL_NAVMESH_PRUNE,	     // ナビメッシュプルーン
	TOOL_OFFMESH_CONNECTION, // オフメッシュ接続
	TOOL_CONVEX_VOLUME,	     // 凸ボリューム
	TOOL_CROWD,	             // 群集
	MAX_TOOLS
};

/// These are just sample areas to use consistent values across the samples.
/// The use should specify these base on his needs.
/// これらは、サンプル全体で一貫した値を使用するための単なるサンプル領域です。
/// 使用法は、必要に応じてこれらのベースを指定する必要があります。
enum SamplePolyAreas
{
	SAMPLE_POLYAREA_GROUND,
	SAMPLE_POLYAREA_WATER,
	SAMPLE_POLYAREA_ROAD,
	SAMPLE_POLYAREA_DOOR,
	SAMPLE_POLYAREA_GRASS,
	SAMPLE_POLYAREA_JUMP,
};

enum SamplePolyFlags
{
	// Ability to walk (ground, grass, road)
	// 歩く機能（地面、草、道路）
	SAMPLE_POLYFLAGS_WALK = 0x01,

	// Ability to swim (water).
	// 泳ぐ機能（水）。
	SAMPLE_POLYFLAGS_SWIM = 0x02,

	// Ability to move through doors.
	// ドアを通って移動する機能。
	SAMPLE_POLYFLAGS_DOOR = 0x04,

	// Ability to jump.
	// ジャンプする機能。
	SAMPLE_POLYFLAGS_JUMP = 0x08,

	// Disabled polygon
	// 無効なポリゴン
	SAMPLE_POLYFLAGS_DISABLED = 0x10,

	// All abilities.
	// すべての能力。
	SAMPLE_POLYFLAGS_ALL = 0xffff
};

class SampleDebugDraw : public DebugDrawGL
{
public:
	virtual unsigned int areaToCol(unsigned int area);
};

enum SamplePartitionType
{
	SAMPLE_PARTITION_WATERSHED, // 分水界分割
	SAMPLE_PARTITION_MONOTONE,  // モノトーン分割
	SAMPLE_PARTITION_LAYERS,    // レイヤー分割
};

struct SampleTool
{
	virtual ~SampleTool() {}
	virtual int type() = 0;
	virtual void init(class Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleMenu() = 0;
	virtual void handleClickDown(const float* s, const float* p, bool shift) = 0;
	virtual void handleClickUp(const float* s, const float* p) = 0;
	virtual void handleClick(const float* s, const float* p) = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleToggle() = 0;
	virtual void handleStep() = 0;
	virtual void handleUpdate(const float dt) = 0;

	bool mouse_middle_push{};
};

struct SampleToolState {
	virtual ~SampleToolState() {}
	virtual void init(class Sample* sample) = 0;
	virtual void reset() = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleUpdate(const float dt) = 0;
};

class Sample
{
protected:
	std::optional<class InputGeom> m_geom;
	class dtNavMesh* m_navMesh{};
	class dtNavMeshQuery* m_navQuery{};
	class dtCrowd* m_crowd{};
	std::optional<rcConfig> generation_params;
	unsigned char m_navMeshDrawFlags;

	float m_cellSize;
	float m_cellHeight;
	float m_agentHeight;
	float m_agentRadius;
	float m_agentMaxClimb;
	float m_agentMaxSlope;
	float m_regionMinSize;
	float m_regionMergeSize;
	float m_edgeMaxLen;
	float m_edgeMaxError;
	float m_vertsPerPoly;
	float m_detailSampleDist;
	float m_detailSampleMaxError;
	SamplePartitionType m_partitionType;

	bool m_filterLowHangingObstacles;
	bool m_filterLedgeSpans;
	bool m_filterWalkableLowHeightSpans;

	std::unique_ptr<SampleTool> m_tool;
	std::array<std::shared_ptr<SampleToolState>, MAX_TOOLS> m_toolStates;

	BuildContext* m_ctx{};

	SampleDebugDraw m_dd;

	dtNavMesh* loadAll(const char* path);
	void saveAll(const char* path, const dtNavMesh* mesh);

public:
	Sample();
	virtual ~Sample();

	void setContext(BuildContext* ctx) { m_ctx = ctx; }
	BuildContext* GetContext() noexcept { return m_ctx; }

	void setTool(std::unique_ptr<SampleTool>&& tool);
	auto& getToolState(int type) { return m_toolStates[type]; }
	void setToolState(int type, std::shared_ptr<SampleToolState>&& s) { m_toolStates[type] = s; }

	SampleDebugDraw& getDebugDraw() { return m_dd; }

	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	virtual void handleClickDown(const float* s, const float* p, bool shift);
	virtual void handleClickUp(const float* s = nullptr, const float* p = nullptr);
	virtual void handleClick(const float* s = nullptr, const float* p = nullptr);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged();
	virtual bool handleBuild();
	virtual void handleUpdate(const float dt);
	virtual void collectSettings(struct BuildSettings& settings);

	std::optional<class InputGeom>& getInputGeom() noexcept { return m_geom; }
	class dtNavMesh* getNavMesh() noexcept { return m_navMesh; }
	class dtNavMeshQuery* getNavMeshQuery() noexcept { return m_navQuery; }
	class dtCrowd* getCrowd() noexcept { return m_crowd; }
	float getAgentRadius() const noexcept { return m_agentRadius; }
	float getAgentHeight() const noexcept { return m_agentHeight; }
	float getAgentClimb() const noexcept { return m_agentMaxClimb; }
	const auto& GetGenerationParams() const noexcept { return generation_params; }

	unsigned char getNavMeshDrawFlags() const { return m_navMeshDrawFlags; }
	void setNavMeshDrawFlags(unsigned char flags) { m_navMeshDrawFlags = flags; }

	void updateToolStates(const float dt);
	void initToolStates(Sample* sample);
	void resetToolStates();
	void renderToolStates();
	void renderOverlayToolStates(double* proj, double* model, int* view);

	void resetCommonSettings();
	void handleCommonSettings(); // 共通の設定(Imgui上での変更)

	auto& GetTool() noexcept { return m_tool; }
	float GetCellSize() const noexcept { return m_cellSize; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Sample(const Sample&) = delete;
	Sample& operator=(const Sample&) = delete;
};

#endif // RECASTSAMPLE_H

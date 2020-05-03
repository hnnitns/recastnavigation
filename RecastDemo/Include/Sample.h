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

// Mask of the ceil part of the area id (3 lower bits)
// エリアid（下位3ビット）のceil部分のマスク、値0（RC_NULL_AREA）は未使用のまま
// the 0 value (RC_NULL_AREA) is left unused
// 0の値（RC_NULL_AREA）は未使用のままになります
constexpr unsigned char SAMPLE_POLYAREA_TYPE_MASK = 0x07;

// Value for the kind of ceil "ground"
// 「グランド」の種類の値
constexpr unsigned char SAMPLE_POLYAREA_TYPE_GROUND = 0x1;

// Value for the kind of ceil "water"
// 「水」の種類の値
constexpr unsigned char SAMPLE_POLYAREA_TYPE_WATER = 0x2;

// Value for the kind of ceil "road"
// 「道路」の種類の値
constexpr unsigned char SAMPLE_POLYAREA_TYPE_ROAD = 0x3;

// Value for the kind of ceil "grass"
// 「草」の種類の値
constexpr unsigned char SAMPLE_POLYAREA_TYPE_GRASS = 0x4;

// Flag for door area. Can be combined with area types and jump flag.
// ドア領域のフラグ。 領域タイプおよびジャンプフラグと組み合わせることができます。
constexpr unsigned char SAMPLE_POLYAREA_FLAG_DOOR = 0x08;

// Flag for jump area. Can be combined with area types and door flag.
// ジャンプ領域のフラグ。 領域タイプおよびドアフラグと組み合わせることができます。
constexpr unsigned char SAMPLE_POLYAREA_FLAG_JUMP = 0x10;

extern rcAreaModification const SAMPLE_AREAMOD_GROUND;

enum SamplePolyFlags
{
	// Ability to walk (ground, grass, road)
	// 歩く能力（地面、草、道路）
	SAMPLE_POLYFLAGS_WALK = 0x01,

	// Ability to swim (water).
	// 泳ぐ能力（水）。
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

unsigned short sampleAreaToFlags(unsigned char area);

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
	virtual void handleClick(const float* s, const float* p, bool shift) = 0;
	virtual void handleRender() = 0;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) = 0;
	virtual void handleToggle() = 0;
	virtual void handleStep() = 0;
	virtual void handleUpdate(const float dt) = 0;
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
	class dtNavMesh* m_navMesh;
	class dtNavMeshQuery* m_navQuery;
	class dtCrowd* m_crowd;

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
	int m_partitionType;

	bool m_filterLowHangingObstacles;
	bool m_filterLedgeSpans;
	bool m_filterWalkableLowHeightSpans;

	SampleTool* m_tool;
	SampleToolState* m_toolStates[MAX_TOOLS];

	BuildContext* m_ctx;

	SampleDebugDraw m_dd;

public:
	Sample();
	virtual ~Sample();

	void setContext(BuildContext* ctx) { m_ctx = ctx; }

	void setTool(SampleTool* tool);
	SampleToolState* getToolState(int type) { return m_toolStates[type]; }
	void setToolState(int type, SampleToolState* s) { m_toolStates[type] = s; }

	SampleDebugDraw& getDebugDraw() { return m_dd; }

	virtual void handleSettings();
	virtual void handleTools();
	virtual void handleDebugMode();
	virtual void handleClick(const float* s, const float* p, bool shift);
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);
	virtual void handleMeshChanged();
	virtual bool handleBuild();
	virtual void handleUpdate(const float dt);
	virtual void collectSettings(struct BuildSettings& settings);

	std::optional<class InputGeom>& getInputGeom() { return m_geom; }
	class dtNavMesh* getNavMesh() { return m_navMesh; }
	class dtNavMeshQuery* getNavMeshQuery() { return m_navQuery; }
	class dtCrowd* getCrowd() { return m_crowd; }
	float getAgentRadius() { return m_agentRadius; }
	float getAgentHeight() { return m_agentHeight; }
	float getAgentClimb() { return m_agentMaxClimb; }

	unsigned char getNavMeshDrawFlags() const { return m_navMeshDrawFlags; }
	void setNavMeshDrawFlags(unsigned char flags) { m_navMeshDrawFlags = flags; }

	void updateToolStates(const float dt);
	void initToolStates(Sample* sample);
	void resetToolStates();
	void renderToolStates();
	void renderOverlayToolStates(double* proj, double* model, int* view);

	void resetCommonSettings();
	void handleCommonSettings(); // 共通の設定(Imgui上での変更)

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Sample(const Sample&);
	Sample& operator=(const Sample&);
};

#endif // RECASTSAMPLE_H

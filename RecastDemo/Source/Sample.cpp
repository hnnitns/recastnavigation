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
#include <cmath>
#include <cstdio>
#include <array>
#include "Sample.h"
#include "InputGeom.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourCrowd.h"
#include "imgui.h"
#include "SDL.h"
#include "SDL_opengl.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

uint16_t sampleAreaToFlags(uint8_t area)
{
	uint8_t areaType = (area & SAMPLE_POLYAREA_TYPE_MASK);
	uint16_t flags = (uint16_t)((areaType == SAMPLE_POLYAREA_TYPE_WATER) ? SAMPLE_POLYFLAGS_SWIM : SAMPLE_POLYFLAGS_WALK);
	if ((areaType & SAMPLE_POLYAREA_FLAG_DOOR) != 0)
	{
		flags |= SAMPLE_POLYFLAGS_DOOR;
	}
	if ((areaType & SAMPLE_POLYAREA_FLAG_JUMP) != 0)
	{
		flags |= SAMPLE_POLYFLAGS_JUMP;
	}
	return flags;
}

uint32_t SampleDebugDraw::areaToCol(uint32_t area)
{
	uint32_t col;

	uint8_t ceil = (area & SAMPLE_POLYAREA_TYPE_MASK);
	switch (ceil)
	{
		// Ground : light blue
		case SAMPLE_POLYAREA_TYPE_GROUND: col = duRGBA(0, 192, 255, 255); break;
			// Water : blue
		case SAMPLE_POLYAREA_TYPE_WATER: col = duRGBA(0, 0, 255, 255); break;
			// Road : brown
		case SAMPLE_POLYAREA_TYPE_ROAD: col = duRGBA(50, 20, 12, 255); break;
			// Grass : green
		case SAMPLE_POLYAREA_TYPE_GRASS: col = duRGBA(0, 255, 0, 255); break;
			// Unexpected ceil : red
		default: col = duRGBA(255, 0, 0, 255); break;
	}

	if (area & SAMPLE_POLYAREA_FLAG_DOOR)
	{
		// Door : cyan
		col = duLerpCol(col, duRGBA(0, 255, 255, 255), 127);
	}
	if (area & SAMPLE_POLYAREA_FLAG_JUMP)
	{
		// Jump : yellow
		col = duLerpCol(col, duRGBA(255, 255, 0, 255), 127);
	}

	return col;
}

Sample::Sample() :
	m_geom(0),
	m_navMesh(0),
	m_navQuery(0),
	m_crowd(0),
	m_navMeshDrawFlags(DU_DRAWNAVMESH_OFFMESHCONS | DU_DRAWNAVMESH_CLOSEDLIST),
	m_filterLowHangingObstacles(true),
	m_filterLedgeSpans(true),
	m_filterWalkableLowHeightSpans(true),
	m_tool(0),
	m_ctx(0)
{
	resetCommonSettings();
	m_navQuery = dtAllocNavMeshQuery();
	m_crowd = dtAllocCrowd();

	m_toolStates.fill(nullptr);
}

Sample::~Sample()
{
	dtFreeNavMeshQuery(m_navQuery);
	dtFreeNavMesh(m_navMesh);
	dtFreeCrowd(m_crowd);

	delete m_tool;

	for (auto& tool : m_toolStates)
	{
		delete tool;
		tool = nullptr;
	}
}

void Sample::setTool(SampleTool* tool)
{
	delete m_tool;
	m_tool = tool;

	if (tool) m_tool->init(this);
}

void Sample::handleSettings() { }

void Sample::handleTools() { }

void Sample::handleDebugMode() { }

void Sample::handleRender()
{
	if (!m_geom) return;

	// Draw mesh // メッシュを描画します
	duDebugDrawTriMesh(&m_dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
		m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(), 0, 1.f);

	/// Draw bounds // 境界を描く
	constexpr uint32_t Color{ duRGBA(255, 255, 255, 128) };
	const auto& bmin = m_geom->getMeshBoundsMin();
	const auto& bmax = m_geom->getMeshBoundsMax();

	duDebugDrawBoxWire(&m_dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], Color, 1.f);
}

void Sample::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/) { }

void Sample::handleMeshChanged(InputGeom* geom)
{
	m_geom = geom;

	const BuildSettings* buildSettings{ geom->getBuildSettings() };

	if (buildSettings)
	{
		m_cellSize             = buildSettings->cellSize;
		m_cellHeight           = buildSettings->cellHeight;
		m_agentHeight          = buildSettings->agentHeight;
		m_agentRadius          = buildSettings->agentRadius;
		m_agentMaxClimb        = buildSettings->agentMaxClimb;
		m_agentMaxSlope        = buildSettings->agentMaxSlope;
		m_regionMinSize        = buildSettings->regionMinSize;
		m_regionMergeSize      = buildSettings->regionMergeSize;
		m_edgeMaxLen           = buildSettings->edgeMaxLen;
		m_edgeMaxError         = buildSettings->edgeMaxError;
		m_vertsPerPoly         = buildSettings->vertsPerPoly;
		m_detailSampleDist     = buildSettings->detailSampleDist;
		m_detailSampleMaxError = buildSettings->detailSampleMaxError;
		m_partitionType        = buildSettings->partitionType;
	}
}

void Sample::collectSettings(BuildSettings& settings)
{
	settings.cellSize             = m_cellSize;
	settings.cellHeight           = m_cellHeight;
	settings.agentHeight          = m_agentHeight;
	settings.agentRadius          = m_agentRadius;
	settings.agentMaxClimb        = m_agentMaxClimb;
	settings.agentMaxSlope        = m_agentMaxSlope;
	settings.regionMinSize        = m_regionMinSize;
	settings.regionMergeSize      = m_regionMergeSize;
	settings.edgeMaxLen           = m_edgeMaxLen;
	settings.edgeMaxError         = m_edgeMaxError;
	settings.vertsPerPoly         = m_vertsPerPoly;
	settings.detailSampleDist     = m_detailSampleDist;
	settings.detailSampleMaxError = m_detailSampleMaxError;
	settings.partitionType        = m_partitionType;
}

void Sample::resetCommonSettings()
{
	m_cellSize             = 0.3f;
	m_cellHeight           = 0.2f;
	m_agentHeight          = 2.f;
	m_agentRadius          = 0.6f;
	m_agentMaxClimb        = 0.9f;
	m_agentMaxSlope        = 45.f;
	m_regionMinSize        = 8;
	m_regionMergeSize      = 20;
	m_edgeMaxLen           = 12.f;
	m_edgeMaxError         = 1.3f;
	m_vertsPerPoly         = 6.f;
	m_detailSampleDist     = 6.f;
	m_detailSampleMaxError = 1.f;
	m_partitionType        = SAMPLE_PARTITION_WATERSHED;
}

void Sample::handleCommonSettings()
{
	// 分類：ラスタライズ----------------------------------------------------------------------
	imguiLabel("Rasterization");

	imguiSlider("Cell Size"/* セルのサイズ */, &m_cellSize, 0.1f, 1.f, 0.01f);
	imguiSlider("Cell Height"/* セルの高さ */, &m_cellHeight, 0.1f, 1.f, 0.01f);

	// 地形メッシュが存在する
	if (m_geom)
	{
		const auto& bmin = m_geom->getNavMeshBoundsMin();
		const auto& bmax = m_geom->getNavMeshBoundsMax();
		int gw{}, gh{};
		std::array<char, 64u> text{};

		// グリットサイズの計算
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);

		snprintf(text.data(), text.size(), "Voxels  %d x %d", gw, gh);
		imguiValue(text.data());
	}

	// 分類：エージェント（経路探索を行う対象）情報---------------------------------------------
	// ※ ナビメッシュの生成に影響を及ぼす
	imguiSeparator();
	imguiLabel("Agent");

	imguiSlider("Height"/* 高さ */, &m_agentHeight, 0.1f, 5.0f, 0.1f);
	imguiSlider("Radius"/* 半径 */, &m_agentRadius, 0.0f, 5.0f, 0.1f);
	imguiSlider("Max Climb"/* 壁を登れる高さ */, &m_agentMaxClimb, 0.1f, 5.0f, 0.1f);
	imguiSlider("Max Slope"/* 歩ける最大勾配 */, &m_agentMaxSlope, 0.0f, 90.0f, 1.f);

	// 分類：ナビメッシュ領域-------------------------------------------------------------------
	imguiSeparator();
	imguiLabel("Region");

	// ナビメッシュ領域の最低サイズ（不必要に大きくすると必要な領域が削除される可能性がある）
	imguiSlider("Min Region Size", &m_regionMinSize, 0.0f, 150.0f, 1.f);

	// 付近のナビメッシュ領域とマージするサイズ（不必要に小さい領域を減らす為）(ただ、値を大きくすればするほど ナビメッシュ生成に時間がかかる)
	imguiSlider("Merged Region Size", &m_regionMergeSize, 0.0f, 150.0f, 1.f);

	// 分類：分割-------------------------------------------------------------------------------
	imguiSeparator();
	imguiLabel("Partitioning");

	// 分水界分割
	if (imguiCheck("Watershed", m_partitionType == SAMPLE_PARTITION_WATERSHED))
		m_partitionType = SAMPLE_PARTITION_WATERSHED;

	// モノトーン分割
	if (imguiCheck("Monotone", m_partitionType == SAMPLE_PARTITION_MONOTONE))
		m_partitionType = SAMPLE_PARTITION_MONOTONE;

	// レイヤー分割
	if (imguiCheck("Layers", m_partitionType == SAMPLE_PARTITION_LAYERS))
		m_partitionType = SAMPLE_PARTITION_LAYERS;

	// 分類：歩行可能な面のフィルター ---------------------------------------------------------
	// 不要なオーバーハングと、キャラが立つことができないフィルタスパンを削除 → いわゆるナビメッシュを綺麗にする機能
	imguiSeparator();
	imguiLabel("Filtering");

	// 縁石などの低層の物体や階段などの構造物の上を歩行可能領域の形成を可能にする（低層の物体などの構造物が多いほど、生成時間に影響が出てくる）
	if (imguiCheck("Low Hanging Obstacles", m_filterLowHangingObstacles))
		m_filterLowHangingObstacles = !m_filterLowHangingObstacles;

	// 説明を訳すなら出張り部分を削る（主にエージェントの幅によって大幅に変わる・ナビメッシュ生成時間にそこそこ影響あり）
	if (imguiCheck("Ledge Spans", m_filterLedgeSpans))
		m_filterLedgeSpans = !m_filterLedgeSpans;

	// 指定された高さよりも小さい場合、ウォーク可能スパンをウォーク不可としてマークする（ナビメッシュ生成時間に僅かに影響あり）
	if (imguiCheck("Walkable Low Height Spans", m_filterWalkableLowHeightSpans))
		m_filterWalkableLowHeightSpans = !m_filterWalkableLowHeightSpans;

	// 分類：ポリゴン化------------------------------------------------------------------------
	imguiSeparator();
	imguiLabel("Polygonization");
	/// 圧倒的語彙力不足！

	// メッシュの境界に沿った輪郭エッジの最大許容長(短くすればするほど端が「ガクガク」になる)
	imguiSlider("Max Edge Length", &m_edgeMaxLen, 0.0f, 50.0f, 1.f);

	// 輪郭のエッジが元の輪郭から逸脱する最大距離（ナビメッシュをどれだけ地形に合わせるか（短くすればするほど端が「カクカク」になる））
	imguiSlider("Max Edge Error", &m_edgeMaxError, 0.1f, 3.0f, 0.1f);

	// 輪郭からポリゴンへの変換時に生成する頂点の最大許可数（一定数を超えると全くナビメッシュが生成されなくなる）
	imguiSlider("Verts Per Poly", &m_vertsPerPoly, 3.0f, 12.0f, 1.f);

	// 分類：詳細メッシュ----------------------------------------------------------------------
	imguiSeparator();
	imguiLabel("Detail Mesh");
	// 地形をサンプリングするときに使用する距離(少なければ少ないほど生成時間が増加し、ナビメッシュが綺麗になる)※0だと生成途中でお亡くなりになるので注意
	imguiSlider("Sample Distance", &m_detailSampleDist, 0.1f, 16.0f, 1.f);

	// ナビメッシュ表面が地形データから逸脱する最大距離(少なければ少ないほど生成時間が増加し、ナビメッシュが綺麗になる)※多くするとナビメッシュから地形がはみ出てしまう
	imguiSlider("Max Sample Error", &m_detailSampleMaxError, 0.0f, 16.0f, 1.f);

	imguiSeparator();
}

void Sample::handleClick(const float* s, const float* p, bool shift)
{
	if (m_tool)
		m_tool->handleClick(s, p, shift);
}

void Sample::handleToggle()
{
	if (m_tool)
		m_tool->handleToggle();
}

void Sample::handleStep()
{
	if (m_tool)
		m_tool->handleStep();
}

bool Sample::handleBuild()
{
	return true;
}

void Sample::handleUpdate(const float dt)
{
	if (m_tool)
		m_tool->handleUpdate(dt);

	updateToolStates(dt);
}

void Sample::updateToolStates(const float dt)
{
	for (auto* tool : m_toolStates)
	{
		if (tool) tool->handleUpdate(dt);
	}
}

void Sample::initToolStates(Sample* sample)
{
	for (auto* tool : m_toolStates)
	{
		if (tool) tool->init(sample);
	}
}

void Sample::resetToolStates()
{
	for (auto* tool : m_toolStates)
	{
		if (tool) tool->reset();
	}
}

void Sample::renderToolStates()
{
	for (auto* tool : m_toolStates)
	{
		if (tool) tool->handleRender();
	}
}

void Sample::renderOverlayToolStates(double* proj, double* model, int* view)
{
	for (auto* tool : m_toolStates)
	{
		if (tool) tool->handleRenderOverlay(proj, model, view);
	}
}
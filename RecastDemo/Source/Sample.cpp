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

	// Draw mesh // ���b�V����`�悵�܂�
	duDebugDrawTriMesh(&m_dd, m_geom->getMesh()->getVerts(), m_geom->getMesh()->getVertCount(),
		m_geom->getMesh()->getTris(), m_geom->getMesh()->getNormals(), m_geom->getMesh()->getTriCount(), 0, 1.f);

	/// Draw bounds // ���E��`��
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
	// ���ށF���X�^���C�Y----------------------------------------------------------------------
	imguiLabel("Rasterization");

	imguiSlider("Cell Size"/* �Z���̃T�C�Y */, &m_cellSize, 0.1f, 1.f, 0.01f);
	imguiSlider("Cell Height"/* �Z���̍��� */, &m_cellHeight, 0.1f, 1.f, 0.01f);

	// �n�`���b�V�������݂���
	if (m_geom)
	{
		const auto& bmin = m_geom->getNavMeshBoundsMin();
		const auto& bmax = m_geom->getNavMeshBoundsMax();
		int gw{}, gh{};
		std::array<char, 64u> text{};

		// �O���b�g�T�C�Y�̌v�Z
		rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);

		snprintf(text.data(), text.size(), "Voxels  %d x %d", gw, gh);
		imguiValue(text.data());
	}

	// ���ށF�G�[�W�F���g�i�o�H�T�����s���Ώہj���---------------------------------------------
	// �� �i�r���b�V���̐����ɉe�����y�ڂ�
	imguiSeparator();
	imguiLabel("Agent");

	imguiSlider("Height"/* ���� */, &m_agentHeight, 0.1f, 5.0f, 0.1f);
	imguiSlider("Radius"/* ���a */, &m_agentRadius, 0.0f, 5.0f, 0.1f);
	imguiSlider("Max Climb"/* �ǂ�o��鍂�� */, &m_agentMaxClimb, 0.1f, 5.0f, 0.1f);
	imguiSlider("Max Slope"/* ������ő���z */, &m_agentMaxSlope, 0.0f, 90.0f, 1.f);

	// ���ށF�i�r���b�V���̈�-------------------------------------------------------------------
	imguiSeparator();
	imguiLabel("Region");

	// �i�r���b�V���̈�̍Œ�T�C�Y�i�s�K�v�ɑ傫������ƕK�v�ȗ̈悪�폜�����\��������j
	imguiSlider("Min Region Size", &m_regionMinSize, 0.0f, 150.0f, 1.f);

	// �t�߂̃i�r���b�V���̈�ƃ}�[�W����T�C�Y�i�s�K�v�ɏ������̈�����炷�ׁj(�����A�l��傫������΂���ق� �i�r���b�V�������Ɏ��Ԃ�������)
	imguiSlider("Merged Region Size", &m_regionMergeSize, 0.0f, 150.0f, 1.f);

	// ���ށF����-------------------------------------------------------------------------------
	imguiSeparator();
	imguiLabel("Partitioning");

	// �����E����
	if (imguiCheck("Watershed", m_partitionType == SAMPLE_PARTITION_WATERSHED))
		m_partitionType = SAMPLE_PARTITION_WATERSHED;

	// ���m�g�[������
	if (imguiCheck("Monotone", m_partitionType == SAMPLE_PARTITION_MONOTONE))
		m_partitionType = SAMPLE_PARTITION_MONOTONE;

	// ���C���[����
	if (imguiCheck("Layers", m_partitionType == SAMPLE_PARTITION_LAYERS))
		m_partitionType = SAMPLE_PARTITION_LAYERS;

	// ���ށF���s�\�Ȗʂ̃t�B���^�[ ---------------------------------------------------------
	// �s�v�ȃI�[�o�[�n���O�ƁA�L�����������Ƃ��ł��Ȃ��t�B���^�X�p�����폜 �� ������i�r���b�V�����Y��ɂ���@�\
	imguiSeparator();
	imguiLabel("Filtering");

	// ���΂Ȃǂ̒�w�̕��̂�K�i�Ȃǂ̍\�����̏����s�\�̈�̌`�����\�ɂ���i��w�̕��̂Ȃǂ̍\�����������قǁA�������Ԃɉe�����o�Ă���j
	if (imguiCheck("Low Hanging Obstacles", m_filterLowHangingObstacles))
		m_filterLowHangingObstacles = !m_filterLowHangingObstacles;

	// ������󂷂Ȃ�o���蕔�������i��ɃG�[�W�F���g�̕��ɂ���đ啝�ɕς��E�i�r���b�V���������Ԃɂ��������e������j
	if (imguiCheck("Ledge Spans", m_filterLedgeSpans))
		m_filterLedgeSpans = !m_filterLedgeSpans;

	// �w�肳�ꂽ���������������ꍇ�A�E�H�[�N�\�X�p�����E�H�[�N�s�Ƃ��ă}�[�N����i�i�r���b�V���������Ԃɋ͂��ɉe������j
	if (imguiCheck("Walkable Low Height Spans", m_filterWalkableLowHeightSpans))
		m_filterWalkableLowHeightSpans = !m_filterWalkableLowHeightSpans;

	// ���ށF�|���S����------------------------------------------------------------------------
	imguiSeparator();
	imguiLabel("Polygonization");
	/// ���|�I��b�͕s���I

	// ���b�V���̋��E�ɉ������֊s�G�b�W�̍ő勖�e��(�Z������΂���قǒ[���u�K�N�K�N�v�ɂȂ�)
	imguiSlider("Max Edge Length", &m_edgeMaxLen, 0.0f, 50.0f, 1.f);

	// �֊s�̃G�b�W�����̗֊s�����E����ő勗���i�i�r���b�V�����ǂꂾ���n�`�ɍ��킹�邩�i�Z������΂���قǒ[���u�J�N�J�N�v�ɂȂ�j�j
	imguiSlider("Max Edge Error", &m_edgeMaxError, 0.1f, 3.0f, 0.1f);

	// �֊s����|���S���ւ̕ϊ����ɐ������钸�_�̍ő勖���i��萔�𒴂���ƑS���i�r���b�V������������Ȃ��Ȃ�j
	imguiSlider("Verts Per Poly", &m_vertsPerPoly, 3.0f, 12.0f, 1.f);

	// ���ށF�ڍ׃��b�V��----------------------------------------------------------------------
	imguiSeparator();
	imguiLabel("Detail Mesh");
	// �n�`���T���v�����O����Ƃ��Ɏg�p���鋗��(���Ȃ���Ώ��Ȃ��قǐ������Ԃ��������A�i�r���b�V�����Y��ɂȂ�)��0���Ɛ����r���ł��S���Ȃ�ɂȂ�̂Œ���
	imguiSlider("Sample Distance", &m_detailSampleDist, 0.1f, 16.0f, 1.f);

	// �i�r���b�V���\�ʂ��n�`�f�[�^�����E����ő勗��(���Ȃ���Ώ��Ȃ��قǐ������Ԃ��������A�i�r���b�V�����Y��ɂȂ�)����������ƃi�r���b�V������n�`���͂ݏo�Ă��܂�
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
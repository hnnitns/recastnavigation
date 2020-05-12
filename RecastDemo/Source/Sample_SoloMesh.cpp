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
#include <string>
#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_SoloMesh.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "RecastDump.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "NavMeshTesterTool.h"
#include "NavMeshPruneTool.h"
#include "OffMeshConnectionTool.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

#ifdef _DEBUG
#define   new	new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

Sample_SoloMesh::Sample_SoloMesh() :
	m_keepInterResults(true),
	m_totalBuildTimeMs(0),
	m_triareas(0),
	m_solid(0),
	m_chf(0),
	m_cset(0),
	m_pmesh(0),
	m_dmesh(0),
	m_drawMode(DRAWMODE_NAVMESH)
{
	setTool(new NavMeshTesterTool);
}

Sample_SoloMesh::~Sample_SoloMesh()
{
	cleanup();
}

void Sample_SoloMesh::cleanup()
{
	delete[] m_triareas;

	m_triareas = nullptr;

	rcFreeHeightField(m_solid);
	m_solid = nullptr;

	rcFreeCompactHeightfield(m_chf);
	m_chf = nullptr;

	rcFreeContourSet(m_cset);
	m_cset = nullptr;

	rcFreePolyMesh(m_pmesh);
	m_pmesh = nullptr;

	rcFreePolyMeshDetail(m_dmesh);
	m_dmesh = nullptr;

	dtFreeNavMesh(m_navMesh);
	m_navMesh = nullptr;
}

void Sample_SoloMesh::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_keepInterResults))
		m_keepInterResults = !m_keepInterResults;

	imguiSeparator();

	char msg[64]{};
	snprintf(msg, 64, "Build Time: %.1fms", m_totalBuildTimeMs);
	imguiLabel(msg);

	imguiSeparator();
}

void Sample_SoloMesh::handleTools()
{
	int type = !m_tool ? TOOL_NONE : m_tool->type();

	if (imguiCheck("Test Navmesh", type == TOOL_NAVMESH_TESTER))
	{
		setTool(new NavMeshTesterTool);
	}
	if (imguiCheck("Prune Navmesh", type == TOOL_NAVMESH_PRUNE))
	{
		setTool(new NavMeshPruneTool);
	}
	if (imguiCheck("Create Off-Mesh Connections", type == TOOL_OFFMESH_CONNECTION))
	{
		setTool(new OffMeshConnectionTool);
	}
	if (imguiCheck("Create Convex Volumes", type == TOOL_CONVEX_VOLUME))
	{
		setTool(new ConvexVolumeTool);
	}
	if (imguiCheck("Create Crowds", type == TOOL_CROWD))
	{
		setTool(new CrowdTool);
	}

	imguiSeparatorLine();

	imguiIndent();

	if (m_tool)
		m_tool->handleMenu();

	imguiUnindent();
}

void Sample_SoloMesh::handleDebugMode()
{
	// Check which modes are valid.
	// �L���ȃ��[�h���m�F���܂��B
	bool valid[MAX_DRAWMODE]{};

	if (m_geom)
	{
		valid[DRAWMODE_NAVMESH]            = m_navMesh  != 0;
		valid[DRAWMODE_NAVMESH_TRANS]      = m_navMesh  != 0;
		valid[DRAWMODE_NAVMESH_BVTREE]     = m_navMesh  != 0;
		valid[DRAWMODE_NAVMESH_NODES]      = m_navQuery != 0;
		valid[DRAWMODE_NAVMESH_INVIS]      = m_navMesh  != 0;
		valid[DRAWMODE_MESH]               = true;
		valid[DRAWMODE_VOXELS]             = m_solid    != 0;
		valid[DRAWMODE_VOXELS_WALKABLE]    = m_solid    != 0;
		valid[DRAWMODE_COMPACT]            = m_chf      != 0;
		valid[DRAWMODE_COMPACT_DISTANCE]   = m_chf      != 0;
		valid[DRAWMODE_COMPACT_REGIONS]    = m_chf      != 0;
		valid[DRAWMODE_REGION_CONNECTIONS] = m_cset     != 0;
		valid[DRAWMODE_RAW_CONTOURS]       = m_cset     != 0;
		valid[DRAWMODE_BOTH_CONTOURS]      = m_cset     != 0;
		valid[DRAWMODE_CONTOURS]           = m_cset     != 0;
		valid[DRAWMODE_POLYMESH]           = m_pmesh    != 0;
		valid[DRAWMODE_POLYMESH_DETAIL]    = m_dmesh    != 0;
	}

	int unavail{};

	for (int i = 0; i < MAX_DRAWMODE; ++i)
	{
		if (!valid[i]) unavail++;
	}

	if (unavail == MAX_DRAWMODE) return;

	imguiLabel("Draw");

	if (imguiCheck("Input Mesh", m_drawMode         == DRAWMODE_MESH, valid[DRAWMODE_MESH]))
		m_drawMode = DRAWMODE_MESH;

	if (imguiCheck("Navmesh", m_drawMode            == DRAWMODE_NAVMESH, valid[DRAWMODE_NAVMESH]))
		m_drawMode = DRAWMODE_NAVMESH;

	if (imguiCheck("Navmesh Invis", m_drawMode      == DRAWMODE_NAVMESH_INVIS, valid[DRAWMODE_NAVMESH_INVIS]))
		m_drawMode = DRAWMODE_NAVMESH_INVIS;

	if (imguiCheck("Navmesh Trans", m_drawMode      == DRAWMODE_NAVMESH_TRANS, valid[DRAWMODE_NAVMESH_TRANS]))
		m_drawMode = DRAWMODE_NAVMESH_TRANS;

	if (imguiCheck("Navmesh BVTree", m_drawMode     == DRAWMODE_NAVMESH_BVTREE, valid[DRAWMODE_NAVMESH_BVTREE]))
		m_drawMode = DRAWMODE_NAVMESH_BVTREE;

	if (imguiCheck("Navmesh Nodes", m_drawMode      == DRAWMODE_NAVMESH_NODES, valid[DRAWMODE_NAVMESH_NODES]))
		m_drawMode = DRAWMODE_NAVMESH_NODES;

	if (imguiCheck("Voxels", m_drawMode             == DRAWMODE_VOXELS, valid[DRAWMODE_VOXELS]))
		m_drawMode = DRAWMODE_VOXELS;

	if (imguiCheck("Walkable Voxels", m_drawMode    == DRAWMODE_VOXELS_WALKABLE, valid[DRAWMODE_VOXELS_WALKABLE]))
		m_drawMode = DRAWMODE_VOXELS_WALKABLE;

	if (imguiCheck("Compact", m_drawMode            == DRAWMODE_COMPACT, valid[DRAWMODE_COMPACT]))
		m_drawMode = DRAWMODE_COMPACT;

	if (imguiCheck("Compact Distance", m_drawMode   == DRAWMODE_COMPACT_DISTANCE, valid[DRAWMODE_COMPACT_DISTANCE]))
		m_drawMode = DRAWMODE_COMPACT_DISTANCE;

	if (imguiCheck("Compact Regions", m_drawMode    == DRAWMODE_COMPACT_REGIONS, valid[DRAWMODE_COMPACT_REGIONS]))
		m_drawMode = DRAWMODE_COMPACT_REGIONS;

	if (imguiCheck("Region Connections", m_drawMode == DRAWMODE_REGION_CONNECTIONS, valid[DRAWMODE_REGION_CONNECTIONS]))
		m_drawMode = DRAWMODE_REGION_CONNECTIONS;

	if (imguiCheck("Raw Contours", m_drawMode       == DRAWMODE_RAW_CONTOURS, valid[DRAWMODE_RAW_CONTOURS]))
		m_drawMode = DRAWMODE_RAW_CONTOURS;

	if (imguiCheck("Both Contours", m_drawMode      == DRAWMODE_BOTH_CONTOURS, valid[DRAWMODE_BOTH_CONTOURS]))
		m_drawMode = DRAWMODE_BOTH_CONTOURS;

	if (imguiCheck("Contours", m_drawMode           == DRAWMODE_CONTOURS, valid[DRAWMODE_CONTOURS]))
		m_drawMode = DRAWMODE_CONTOURS;

	if (imguiCheck("Poly Mesh", m_drawMode          == DRAWMODE_POLYMESH, valid[DRAWMODE_POLYMESH]))
		m_drawMode = DRAWMODE_POLYMESH;

	if (imguiCheck("Poly Mesh Detail", m_drawMode   == DRAWMODE_POLYMESH_DETAIL, valid[DRAWMODE_POLYMESH_DETAIL]))
		m_drawMode = DRAWMODE_POLYMESH_DETAIL;

	if (unavail)
	{
		// [���Ԍ��ʂ�ێ�]�Ƀ`�F�b�N�}�[�N��t���āA�f�o�b�O���[�h�I�v�V����������ɕ\�����܂��B
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("to see more debug mode options.");
	}
}

void Sample_SoloMesh::handleRender()
{
	if (m_geom->isLoadGeomMeshEmpty()) return;

	glEnable(GL_FOG);
	glDepthMask(GL_TRUE);

	const float texScale = 1.f / (m_cellSize * 10.0f);

	if (m_drawMode != DRAWMODE_NAVMESH_TRANS)
	{
		// Draw mesh
		for (auto& geom : m_geom->getLoadGeomMesh())
		{
			const auto& mesh{ geom.m_mesh };

			duDebugDrawTriMeshSlope(&m_dd, mesh->getVerts(), mesh->getVertCount(),
				mesh->getTris(), mesh->getNormals(), mesh->getTriCount(),
				m_agentMaxSlope, texScale);
		}

		m_geom->drawOffMeshConnections(&m_dd);
	}

	glDisable(GL_FOG);
	glDepthMask(GL_FALSE);

	// Draw bounds
	const auto& bmin = m_geom->getNavMeshBoundsMin();
	const auto& bmax = m_geom->getNavMeshBoundsMax();

	duDebugDrawBoxWire(&m_dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2],
		duRGBA(255, 255, 255, 128), 1.f);

	m_dd.begin(DU_DRAW_POINTS, 5.0f);
	m_dd.vertex(bmin[0], bmin[1], bmin[2], duRGBA(255, 255, 255, 128));
	m_dd.end();

	if (m_navMesh && m_navQuery &&
		(m_drawMode == DRAWMODE_NAVMESH ||
			m_drawMode == DRAWMODE_NAVMESH_TRANS ||
			m_drawMode == DRAWMODE_NAVMESH_BVTREE ||
			m_drawMode == DRAWMODE_NAVMESH_NODES ||
			m_drawMode == DRAWMODE_NAVMESH_INVIS))
	{
		if (m_drawMode != DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&m_dd, *m_navMesh, *m_navQuery, m_navMeshDrawFlags);
		if (m_drawMode == DRAWMODE_NAVMESH_BVTREE)
			duDebugDrawNavMeshBVTree(&m_dd, *m_navMesh);
		if (m_drawMode == DRAWMODE_NAVMESH_NODES)
			duDebugDrawNavMeshNodes(&m_dd, *m_navQuery);
		duDebugDrawNavMeshPolysWithFlags(&m_dd, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0, 0, 0, 128));
	}

	glDepthMask(GL_TRUE);

	if (m_chf && m_drawMode == DRAWMODE_COMPACT)
		duDebugDrawCompactHeightfieldSolid(&m_dd, *m_chf);

	if (m_chf && m_drawMode == DRAWMODE_COMPACT_DISTANCE)
		duDebugDrawCompactHeightfieldDistance(&m_dd, *m_chf);

	if (m_chf && m_drawMode == DRAWMODE_COMPACT_REGIONS)
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);

	if (m_solid && m_drawMode == DRAWMODE_VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&m_dd, *m_solid);
		glDisable(GL_FOG);
	}
	if (m_solid && m_drawMode == DRAWMODE_VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&m_dd, *m_solid);
		glDisable(GL_FOG);
	}
	if (m_cset && m_drawMode == DRAWMODE_RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_cset && m_drawMode == DRAWMODE_BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_cset, 0.5f);
		duDebugDrawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_cset && m_drawMode == DRAWMODE_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_chf && m_cset && m_drawMode == DRAWMODE_REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);

		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_pmesh && m_drawMode == DRAWMODE_POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&m_dd, *m_pmesh);
		glDepthMask(GL_TRUE);
	}
	if (m_dmesh && m_drawMode == DRAWMODE_POLYMESH_DETAIL)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMeshDetail(&m_dd, *m_dmesh);
		glDepthMask(GL_TRUE);
	}

	m_geom->drawConvexVolumes(&m_dd);

	if (m_tool)
		m_tool->handleRender();
	renderToolStates();

	glDepthMask(GL_TRUE);
}

void Sample_SoloMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
	if (m_tool)
		m_tool->handleRenderOverlay(proj, model, view);
	renderOverlayToolStates(proj, model, view);
}

void Sample_SoloMesh::handleMeshChanged()
{
	Sample::handleMeshChanged();

	dtFreeNavMesh(m_navMesh);
	m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

bool Sample_SoloMesh::handleBuild()
{
	if (m_geom->isLoadGeomMeshEmpty())
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified."); // ���̓��b�V�����w�肳��Ă��܂���B
		return false;
	}

	cleanup();

	const auto& bmin{ m_geom->getNavMeshBoundsMin() };
	const auto& bmax{ m_geom->getNavMeshBoundsMax() };
	const auto& mesh{ m_geom->getMeshAt(0) };

	const auto& verts{ mesh->getVerts() };
	const int nverts{ mesh->getVertCount() };
	const auto& tris{ mesh->getTris() };
	const int ntris{ mesh->getTriCount() };

	//
	// Step 1. Initialize build config. �r���h�\�������������܂��B
	//

	// Init build configuration from GUI
	// GUI�����Init�r���h�\��
	memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.cs = m_cellSize;
	m_cfg.ch = m_cellHeight;
	m_cfg.walkableSlopeAngle = m_agentMaxSlope;
	m_cfg.walkableHeight = (int)ceilf(m_agentHeight / m_cfg.ch);
	m_cfg.walkableClimb = (int)floorf(m_agentMaxClimb / m_cfg.ch);
	m_cfg.walkableRadius = (int)ceilf(m_agentRadius / m_cfg.cs);
	m_cfg.maxEdgeLen = (int)(m_edgeMaxLen / m_cellSize);
	m_cfg.maxSimplificationError = m_edgeMaxError;
	m_cfg.minRegionArea = (int)rcSqr(m_regionMinSize);		// Note: area = size*size
	m_cfg.mergeRegionArea = (int)rcSqr(m_regionMergeSize);	// Note: area = size*size
	m_cfg.maxVertsPerPoly = (int)m_vertsPerPoly;
	m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

	// Set the area where the navigation will be build.
	// �i�r�Q�[�V�������\�z����G���A��ݒ肵�܂��B
	// Here the bounds of the input mesh are used, but the area could be specified by an user defined box, etc.
	// �����ł́A���̓��b�V���̋��E���g�p����܂����A�̈�̓��[�U�[��`�̃{�b�N�X�ȂǂŎw��ł��܂��B
	rcVcopy(m_cfg.bmin, bmin.data()); // �R�s�[
	rcVcopy(m_cfg.bmax, bmax.data()); // �R�s�[

	// �O���b�g�T�C�Y�̌v�Z
	rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);

	// Reset build times gathering.
	// �r���h���Ԃ̎��W�����Z�b�g���܂��B
	m_ctx->resetTimers();

	// Start the build process.
	// �r���h�v���Z�X���J�n���܂��B
	m_ctx->startTimer(RC_TIMER_TOTAL);

	m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
	m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);
	m_ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f, ntris / 1000.0f);

	//
	// Step 2. Rasterize input polygon soup. ���̓|���S���X�[�v�����X�^���C�Y���܂��B
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	// ���̓f�[�^�����X�^���C�Y����{�N�Z���n�C�g�t�B�[���h�����蓖�Ă܂��B
	m_solid = rcAllocHeightfield();
	if (!m_solid)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'."); // �������[�s���usolid�v
		return false;
	}
	if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield."); // �\���b�h�n�`���쐬�ł��܂���ł���
		return false;
	}

	// Allocate array that can hold triangle area types.
	// �O�p�`�̈�^�C�v��ێ��ł���z������蓖�Ă܂��B
	// If you have multiple meshes you need to process, allocate and array which can hold the max number of triangles you need to process.
	// ��������K�v�����郁�b�V������������ꍇ�́A��������K�v������O�p�`�̍ő吔��ێ��ł���z��Ɗ��蓖�Ă��s���܂��B
	m_triareas = new unsigned char[ntris];
	if (!m_triareas)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", ntris); // �������[�s���um_triareas�v
		return false;
	}

	// Find triangles which are walkable based on their slope and rasterize them.
	// �X�΂Ɋ�Â��ĕ��s�\�ȎO�p�`�������A���X�^���C�Y���܂��B
	// If your input data is multiple meshes, you can transform them here, calculate the are type for each of the meshes and rasterize them.
	// ���̓f�[�^�������̃��b�V���ł���ꍇ�A�����ł�����ϊ����A�e���b�V����are�^�C�v���v�Z���āA���������X�^���C�Y�ł��܂��B
	memset(m_triareas, 0, ntris * sizeof(unsigned char));
	rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, verts.data(), nverts, tris.data(), ntris, m_triareas, SAMPLE_AREAMOD_GROUND);

	if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris.data(), m_triareas, ntris, *m_solid, m_cfg.walkableClimb))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles."); // �O�p�`�����X�^���C�Y�ł��܂���ł����B
		return false;
	}

	if (!m_keepInterResults)
	{
		delete[] m_triareas;
		m_triareas = 0;
	}

	//
	// Step 3. Filter walkables surfaces. ���s�\�Ȗʂ��t�B���^�[���܂��B
	//

	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	// ���ׂẴW�I���g�������X�^���C�Y�����ƁA�t�B���^�����O�̏����p�X�����s���āA
	// �ێ�I�ȃ��X�^���C�Y�ɂ���Ĉ����N�������s�v�ȃI�[�o�[�n���O�ƁA�L�����������Ƃ��ł��Ȃ��t�B���^�X�p�����폜���܂�
	if (m_filterLowHangingObstacles)
		// ���΂Ȃǂ̒�w�̕��̂�K�i�Ȃǂ̍\�����̏����s�\�̈�̌`�����\�ɂ���
		rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);

	if (m_filterLedgeSpans)
		// ������󂷂Ȃ�o���蕔�������i��Ƀv���[���[�̕��ɂ���đ啝�ɕς��E�i�r���b�V���������Ԃɂ��������e������j
		rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);

	if (m_filterWalkableLowHeightSpans)
		// �w�肳�ꂽ���������������ꍇ�A�E�H�[�N�\�X�p�����E�H�[�N�s�Ƃ��ă}�[�N����
		rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);

	//
	// Step 4. Partition walkable surface to simple regions. // ���s�\�ȕ\�ʂ�P���ȗ̈�ɕ������܂��B
	//

	// Compact the heightfield so that it is faster to handle from now on.
	//�@�n�C�g�t�B�[���h�����k���āA����̏����������ɂȂ�悤�ɂ��܂��B
	// This will result more cache coherent data as well as the neighbours between walkable cells will be calculated.
	//�@����ɂ��A��葽���̃L���b�V���R�q�[�����g�f�[�^����������A�E�H�[�N�\�Z���Ԃ̗אڃZ�����v�Z����܂��B
	m_chf = rcAllocCompactHeightfield();

	if (!m_chf)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'."); // �������[�s���uchf�v
		return false;
	}

	// �R���p�N�g�Ȓn�`�̐���
	if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data."); // �R���p�N�g�ȃf�[�^���\�z�ł��܂���ł����B
		return false;
	}

	if (!m_keepInterResults)
	{
		rcFreeHeightField(m_solid);
		m_solid = 0;
	}

	// Erode the walkable area by agent radius.
	// �G�[�W�F���g���a���Ƃɕ��s�\�G���A��N�H���܂��B
	if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode."); // �N�H�ł��܂���ł����B
		return false;
	}

	// (Optional) Mark areas.
	// �i�I�v�V�����j�G���A���}�[�N���܂��B
	const auto* vols = m_geom->getConvexVolumes(); // �ʃ{�����[�����擾

	for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i) // �ʃ{�����[�������擾
		rcMarkConvexPolyArea(m_ctx, vols->at(i).verts.data(), vols->at(i).nverts, vols->at(i).hmin, vols->at(i).hmax, vols->at(i).areaMod, *m_chf); // �ʑ��p�`�̈���}�[�N

	// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
	// There are 3 martitioning methods, each with some pros and cons:
	// �n�C�g�t�B�[���h�𕪊����āA��ŊȒP�ȃA���S���Y�����g�p���ĕ��s�\�G���A���O�p���ʂł���悤�ɂ��܂��B
	// �p�[�e�B�V���������ɂ�3�̕��@������A���ꂼ�꒷���ƒZ��������܂��B

	// 1) Watershed partitioning
	//   - the classic Recast partitioning
	//   - creates the nicest tessellation
	//   - usually slowest
	//   - partitions the heightfield into nice regions without holes or overlaps
	//   - the are some corner cases where this method creates produces holes and overlaps
	//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
	//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
	//   * generally the best choice if you precompute the navmesh, use this if you have large open areas
	// 1�j�����E����
	//   -�N���V�b�N�ȃ��L���X�g�p�[�e�B�V����
	//   -�ł��ǂ��e�b�Z���[�V�������쐬���܂�
	//   -�ʏ�ł��x��
	//   -�n�`�����Ȃǂ��Ȃ��A�Y��ɕ����ł��܂�
	//   -���̕��@���쐬���邢�����̃R�[�i�[�P�[�X�́A���ƃI�[�o�[���b�v�𐶐����܂�
	//      -�����ȏ�Q�����傫�ȃI�[�v���G���A�ɋ߂��ꍇ�A�����\�������ꍇ������܂��i�O�p���ʂł���������ł��܂��j
	//      -����������̘L���i�K�i�Ȃǁj������ꍇ�A�d������������\��������A����ɂ��O�p���ʂ����s���܂�
	// * navmesh�����O�v�Z����ꍇ�͈�ʓI�ɍŗǂ̑I���ł��B�傫�ȋ󂫗̈悪����ꍇ�͂�����g�p���܂�

	// 2) Monotone partioning
	//   - fastest
	//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
	//   - creates long thin polygons, which sometimes causes paths with detours
	//   * use this if you want fast navmesh generation
	// 2�j���m�g�[������
	//   -�ő�
	//   -�n�`������d���̂Ȃ��̈�ɕ������܂��i�ۏ؁j
	//   -�I��̂���p�X�������N�������Ƃ�����A�ג����|���S�����쐬���܂�
	//   *����navmesh�������K�v�ȏꍇ�͂�����g�p���܂�

	// 3) Layer partitoining
	//   - quite fast
	//   - partitions the heightfield into non-overlapping regions
	//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
	//   - produces better triangles than monotone partitioning
	//   - does not have the corner cases of watershed partitioning
	//   - can be slow and create a bit ugly tessellation (still better than monotone)
	//     if you have large open areas with small obstacles (not a problem if you use tiles)
	//   * good choice to use for tiled navmesh with medium and small sized tiles
	// 3�j���C���[����
	//   - ���Ȃ葬��
	//   -�n�`���d�����Ȃ��̈�ɕ������܂�
	//   -���ɑΏ����邽�߂ɎO�p���ʃR�[�h�Ɉˑ����Ă��܂��i���������āA�P���ȕ��������x���j
	//   -���m�g�[�����������D�ꂽ�O�p�`�𐶐����܂�
	//   -���敪���̃R�[�i�[�P�[�X�͂���܂���
	//   -�����ȏ�Q���̂���傫�ȃI�[�v���G���A������ꍇ�A�x�����Ƃ�����A�������e�b�Z���[�V�������쐬�ł��܂��i���m�g�[�������D��Ă��܂��j�i�^�C�����g�p����ꍇ�͖�肠��܂���j
	//   *���^����я��^�̃^�C���Ń^�C�����肳�ꂽnavmesh�Ɏg�p����̂ɓK�����I����

	switch (m_partitionType)
	{
		case SAMPLE_PARTITION_WATERSHED: // �����E����
		{
			// Prepare for region partitioning, by calculating distance field along the walkable surface.
			// ���s�\�ȕ\�ʂɉ����ċ����t�B�[���h���v�Z���āA�̈敪���̏��������܂��B
			if (!rcBuildDistanceField(m_ctx, *m_chf))
			{
				m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field."); // �����t�B�[���h���\�z�ł��܂���ł���
				return false;
			}

			// Partition the walkable surface into simple regions without holes.
			// ���s�\�ȕ\�ʂ��A���̂Ȃ��P���ȗ̈�ɕ������܂��B
			if (!rcBuildRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
			{
				m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions."); // ������\�z�ł��܂���ł���
				return false;
			}

			break;
		}
		case SAMPLE_PARTITION_MONOTONE: // ���m�g�[������
		{
			// Partition the walkable surface into simple regions without holes.
			// Monotone partitioning does not need distancefield.
			//���s�\�ȃT�[�t�F�X���A���̂Ȃ��P���ȗ̈�ɕ������܂��B
			//�P���ȕ����͋����t�B�[���h��K�v�Ƃ��܂���B
			if (!rcBuildRegionsMonotone(m_ctx, *m_chf, 0, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
			{
				m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions."); // ���m�g�[���̈���\�z�ł��܂���ł����B
				return false;
			}

			break;
		}
		case SAMPLE_PARTITION_LAYERS:
		{
			// Partition the walkable surface into simple regions without holes.
			// ���s�\�ȃT�[�t�F�X���A���̂Ȃ��P���ȗ̈�ɕ������܂��B
			if (!rcBuildLayerRegions(m_ctx, *m_chf, 0, m_cfg.minRegionArea)) //
			{
				m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions."); // ���C���[�̈���\�z�ł��܂���ł���
				return false;
			}

			break;
		}
	}

	//
	// Step 5. Trace and simplify region contours. �̈�̗֊s���g���[�X���Ċȑf�����܂��B
	//

	// Create contours.
	// �֊s���쐬���܂��B
	m_cset = rcAllocContourSet();
	if (!m_cset)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'."); // �������[�s���ucset�v
		return false;
	}

	if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours."); // �֊s���쐬�ł��܂���ł����B
		return false;
	}

	//
	// Step 6. Build polygons mesh from contours. �֊s����|���S�����b�V�����쐬���܂��B
	//

	// Build polygon navmesh from the contours.
	// ����������|���S���i�u���b�V�����쐬���܂��B
	m_pmesh = rcAllocPolyMesh();
	if (!m_pmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'."); // �������[�s���upmesh�v
		return false;
	}
	if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours."); // �֊s���O�p���ʂł��܂���ł����B
		return false;
	}

	//
	// Step 7. Create detail mesh which allows to access approximate height on each polygon.
	//         �e�|���S���̂��悻�̍����ɃA�N�Z�X�ł���ڍ׃��b�V�����쐬���܂��B
	//

	m_dmesh = rcAllocPolyMeshDetail();
	if (!m_dmesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'."); // �������[�s���upmdtl�v
		return false;
	}

	if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, *m_dmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh."); // �ڍ׃��b�V�����\�z�ł��܂���ł����B
		return false;
	}

	if (!m_keepInterResults)
	{
		rcFreeCompactHeightfield(m_chf);
		m_chf = 0;
		rcFreeContourSet(m_cset);
		m_cset = 0;
	}

	// At this point the navigation mesh data is ready, you can access it from m_pmesh.
	// ���̎��_�ŁA�i�r�Q�[�V�������b�V���f�[�^�̏������ł��܂����Bm_pmesh����A�N�Z�X�ł��܂��B
	// See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.
	// �f�[�^�ɃA�N�Z�X������@�̗�Ƃ��āAduDebugDrawPolyMesh�܂���dtCreateNavMeshData���Q�Ƃ��Ă��������B

	//
	// (Optional) Step 8. Create Detour data from Recast poly mesh. �X�e�b�v8. Recast poly mesh����I��f�[�^���쐬���܂��B
	//

	// The GUI may allow more max points per polygon than Detour can handle.
	// GUI�ł́ADetour�ŏ����ł�����������̃|���S��������̍ő�|�C���g���������ꍇ������܂��B
	// Only build the detour navmesh if we do not exceed the limit.
	// �����𒴂��Ȃ��ꍇ�ɂ̂݁A�I��navmesh���\�z���܂��B

	if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		unsigned char* navData = 0;
		int navDataSize = 0;

		// Update poly flags from areas.
		// �G���A����|���S���t���O���X�V���܂��B
		for (int i = 0; i < m_pmesh->npolys; ++i)
		{
			m_pmesh->flags[i] = sampleAreaToFlags(m_pmesh->areas[i]);
		}

		dtNavMeshCreateParams params;
		memset(&params, 0, sizeof(params));
		params.verts = m_pmesh->verts;
		params.vertCount = m_pmesh->nverts;
		params.polys = m_pmesh->polys;
		params.polyAreas = m_pmesh->areas;
		params.polyFlags = m_pmesh->flags;
		params.polyCount = m_pmesh->npolys;
		params.nvp = m_pmesh->nvp;
		params.detailMeshes = m_dmesh->meshes;
		params.detailVerts = m_dmesh->verts;
		params.detailVertsCount = m_dmesh->nverts;
		params.detailTris = m_dmesh->tris;
		params.detailTriCount = m_dmesh->ntris;
		params.offMeshConVerts = m_geom->getOffMeshConnectionVerts().data();
		params.offMeshConRad = m_geom->getOffMeshConnectionRads().data();
		params.offMeshConDir = m_geom->getOffMeshConnectionDirs().data();
		params.offMeshConAreas = m_geom->getOffMeshConnectionAreas().data();
		params.offMeshConFlags = m_geom->getOffMeshConnectionFlags().data();
		params.offMeshConUserID = m_geom->getOffMeshConnectionId().data();
		params.offMeshConCount = m_geom->getOffMeshConnectionCount();
		params.walkableHeight = m_agentHeight;
		params.walkableRadius = m_agentRadius;
		params.walkableClimb = m_agentMaxClimb;
		rcVcopy(params.bmin, m_pmesh->bmin);
		rcVcopy(params.bmax, m_pmesh->bmax);
		params.cs = m_cfg.cs;
		params.ch = m_cfg.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return false;
		}

		m_navMesh = dtAllocNavMesh();
		if (!m_navMesh)
		{
			dtFree(navData);
			m_ctx->log(RC_LOG_ERROR, "Could not create Detour navmesh");
			return false;
		}

		dtStatus status;

		status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh");
			return false;
		}

		status = m_navQuery->init(m_navMesh, 2048);
		if (dtStatusFailed(status))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not init Detour navmesh query");
			return false;
		}
	}

	m_ctx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	// �p�t�H�[�}���X���v��\�����܂��B
	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pmesh->nverts, m_pmesh->npolys);

	m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;

	if (m_tool)
		m_tool->init(this);
	initToolStates(this);

	return true;
}
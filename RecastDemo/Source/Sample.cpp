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

unsigned int SampleDebugDraw::areaToCol(unsigned int area)
{
	switch (area)
	{
		// Ground (0) : light blue
	case SAMPLE_POLYAREA_GROUND: return duRGBA(0, 192, 255, 255);
		// Water : blue
	case SAMPLE_POLYAREA_WATER: return duRGBA(0, 0, 255, 255);
		// Road : brown
	case SAMPLE_POLYAREA_ROAD: return duRGBA(50, 20, 12, 255);
		// Door : cyan
	case SAMPLE_POLYAREA_DOOR: return duRGBA(0, 255, 255, 255);
		// Grass : green
	case SAMPLE_POLYAREA_GRASS: return duRGBA(0, 255, 0, 255);
		// Jump : yellow
	case SAMPLE_POLYAREA_JUMP: return duRGBA(255, 255, 0, 255);
		// Unexpected : red
	default: return duRGBA(255, 0, 0, 255);
	}
}

Sample::Sample() :
	m_navMeshDrawFlags(DU_DRAWNAVMESH_OFFMESHCONS | DU_DRAWNAVMESH_CLOSEDLIST),
	m_filterLowHangingObstacles(true), m_filterLedgeSpans(true), m_filterWalkableLowHeightSpans(true)
{
	resetCommonSettings();
	m_navQuery = dtAllocNavMeshQuery();
	m_crowd = dtAllocCrowd();
	m_geom.emplace();

	m_toolStates.fill(nullptr);
}

Sample::~Sample()
{
	dtFreeNavMeshQuery(m_navQuery);
	dtFreeNavMesh(m_navMesh);
	dtFreeCrowd(m_crowd);
	m_tool = nullptr;
	m_toolStates.fill(nullptr);
}

void Sample::setTool(std::unique_ptr<SampleTool>&& tool)
{
	m_tool = std::move(tool);

	if (m_tool) m_tool->init(this);
}

void Sample::handleSettings()
{
}

void Sample::handleTools()
{
}

void Sample::handleDebugMode()
{
}

void Sample::handleRender()
{
	if (!m_geom)
		return;

	// Draw mesh
	for (auto& geom : m_geom->getLoadGeomMesh())
	{
		const auto& mesh{ geom.m_mesh };

		duDebugDrawTriMesh(&m_dd, mesh->getVerts(), mesh->getVertCount(), mesh->getTris(), mesh->getNormals(),
			mesh->getTriCount(), 0, 1.f);
	}

	/// Draw bounds // ���E��`��
	constexpr auto Color{ duRGBA(255, 255, 255, 128) };
	const auto& bmin = m_geom->getMeshBoundsMin();
	const auto& bmax = m_geom->getMeshBoundsMax();

	duDebugDrawBoxWire(&m_dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2], Color, 1.f);
}

void Sample::handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/)
{
}

void Sample::handleMeshChanged()
{
	const BuildSettings* buildSettings{ m_geom->getBuildSettings() };

	if (buildSettings)
	{
		m_cellSize = buildSettings->cellSize;
		m_cellHeight = buildSettings->cellHeight;
		m_agentHeight = buildSettings->agentHeight;
		m_agentRadius = buildSettings->agentRadius;
		m_agentMaxClimb = buildSettings->agentMaxClimb;
		m_agentMaxSlope = buildSettings->agentMaxSlope;
		m_regionMinSize = buildSettings->regionMinSize;
		m_regionMergeSize = buildSettings->regionMergeSize;
		m_edgeMaxLen = buildSettings->edgeMaxLen;
		m_edgeMaxError = buildSettings->edgeMaxError;
		m_vertsPerPoly = buildSettings->vertsPerPoly;
		m_detailSampleDist = buildSettings->detailSampleDist;
		m_detailSampleMaxError = buildSettings->detailSampleMaxError;
		m_partitionType = static_cast<SamplePartitionType>(buildSettings->partitionType);
	}
}

void Sample::collectSettings(BuildSettings& settings)
{
	settings.cellSize = m_cellSize;
	settings.cellHeight = m_cellHeight;
	settings.agentHeight = m_agentHeight;
	settings.agentRadius = m_agentRadius;
	settings.agentMaxClimb = m_agentMaxClimb;
	settings.agentMaxSlope = m_agentMaxSlope;
	settings.regionMinSize = m_regionMinSize;
	settings.regionMergeSize = m_regionMergeSize;
	settings.edgeMaxLen = m_edgeMaxLen;
	settings.edgeMaxError = m_edgeMaxError;
	settings.vertsPerPoly = m_vertsPerPoly;
	settings.detailSampleDist = m_detailSampleDist;
	settings.detailSampleMaxError = m_detailSampleMaxError;
	settings.partitionType = m_partitionType;

	if (m_geom)
	{
		for (const auto& src : m_geom->getLoadGeomMesh())
		{
			auto& dest{ settings.meshes.emplace_back() };

			dest.pos = src.pos;
			dest.rotate = src.rotate;
			dest.scale = src.scale;
		}
	}
}

void Sample::resetCommonSettings()
{
	m_cellSize = 0.3f;
	m_cellHeight = 0.2f;
	m_agentHeight = 2.0f;
	m_agentRadius = 0.6f;
	m_agentMaxClimb = 0.9f;
	m_agentMaxSlope = 45.0f;
	m_regionMinSize = 8;
	m_regionMergeSize = 20;
	m_edgeMaxLen = 12.0f;
	m_edgeMaxError = 1.3f;
	m_vertsPerPoly = 6.0f;
	m_detailSampleDist = 6.0f;
	m_detailSampleMaxError = 1.f;
	m_partitionType = SAMPLE_PARTITION_WATERSHED;
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
		rcCalcGridSize(bmin.data(), bmax.data(), m_cellSize, &gw, &gh);

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
	imguiSeparator();

	if (!m_geom->isLoadGeomMeshEmpty())
	{
		imguiLabel("Geometry deformation"); // �n�`�ύX

		for (auto& geom : m_geom->getEditableLoadGeomMesh())
		{
			if (!geom.is_selected)	continue;

			constexpr std::array<char*, 3> Contents{ "X", "Y", "Z" };

			bool is_changed{};
			std::string text{ "Position " };

			for (size_t i = 0; i < Contents.size(); i++)
			{
				is_changed |= imguiSlider((text + Contents[i]).c_str(), &geom.pos[i], -100.f, 100.f, 1.f);
			}

			imguiSeparator();
			text = "Scale";

			for (size_t i = 0; i < Contents.size(); i++)
			{
				is_changed |= imguiSlider((text + Contents[i]).c_str(), &geom.scale[i], 0.1f, 10.f, 0.1f);
			}

			imguiSeparator();
			text = "Rotate";

			for (size_t i = 0; i < Contents.size(); i++)
			{
				is_changed |= imguiSlider((text + Contents[i]).c_str(), &geom.rotate[i], -180.f, 180.f, 1.f);
			}

			geom.is_changed = is_changed;
		}
	}

	imguiSeparator();
	imguiSeparator();
}

void Sample::handleClickDown(const float* s, const float* p, bool shift)
{
	if (m_tool)
		m_tool->handleClickDown(s, p, shift);
}

void Sample::handleClickUp(const float* s, const float* p)
{
	if (m_tool)
		m_tool->handleClickUp(s, p);
}

void Sample::handleClick(const float* s, const float* p)
{
	if (m_tool)
		m_tool->handleClick(s, p);
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
	if (m_geom)
	{
		for (auto& geom : m_geom->getEditableLoadGeomMesh())
		{
			if (geom.is_changed && geom.is_selected)
			{
				geom.Update();

				m_geom->CalcAllMeshBounds();
			}
		}
	}

	if (m_tool)
		m_tool->handleUpdate(dt);
	updateToolStates(dt);
}

void Sample::updateToolStates(const float dt)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleUpdate(dt);
	}
}

void Sample::initToolStates(Sample* sample)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->init(sample);
	}
}

void Sample::resetToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->reset();
	}
}

void Sample::renderToolStates()
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRender();
	}
}

void Sample::renderOverlayToolStates(double* proj, double* model, int* view)
{
	for (int i = 0; i < MAX_TOOLS; i++)
	{
		if (m_toolStates[i])
			m_toolStates[i]->handleRenderOverlay(proj, model, view);
	}
}

static const int NAVMESHSET_MAGIC = 'M'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'MSET';
static const int NAVMESHSET_VERSION = 1;

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams params;
};

struct NavMeshTileHeader
{
	dtTileRef tileRef;
	int dataSize;
};

dtNavMesh* Sample::loadAll(const char* path)
{
	FILE* fp = fopen(path, "rb");
	if (!fp) return 0;

	// Read header.
	NavMeshSetHeader header;
	size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, fp);
	if (readLen != 1)
	{
		fclose(fp);
		return 0;
	}
	if (header.magic != NAVMESHSET_MAGIC)
	{
		fclose(fp);
		return 0;
	}
	if (header.version != NAVMESHSET_VERSION)
	{
		fclose(fp);
		return 0;
	}

	dtNavMesh* mesh = dtAllocNavMesh();
	if (!mesh)
	{
		fclose(fp);
		return 0;
	}
	dtStatus status = mesh->init(&header.params);
	if (dtStatusFailed(status))
	{
		fclose(fp);
		return 0;
	}

	// Read tiles.
	for (int i = 0; i < header.numTiles; ++i)
	{
		NavMeshTileHeader tileHeader;
		readLen = fread(&tileHeader, sizeof(tileHeader), 1, fp);
		if (readLen != 1)
		{
			fclose(fp);
			return 0;
		}

		if (!tileHeader.tileRef || !tileHeader.dataSize)
			break;

		unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
		if (!data) break;
		memset(data, 0, tileHeader.dataSize);
		readLen = fread(data, tileHeader.dataSize, 1, fp);
		if (readLen != 1)
		{
			dtFree(data);
			fclose(fp);
			return 0;
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);

	return mesh;
}

void Sample::saveAll(const char* path, const dtNavMesh* mesh)
{
	if (!mesh) return;

	FILE* fp = fopen(path, "wb");
	if (!fp)
		return;

	// Store header.
	NavMeshSetHeader header;
	header.magic = NAVMESHSET_MAGIC;
	header.version = NAVMESHSET_VERSION;
	header.numTiles = 0;
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;
		header.numTiles++;
	}
	memcpy(&header.params, mesh->getParams(), sizeof(dtNavMeshParams));
	fwrite(&header, sizeof(NavMeshSetHeader), 1, fp);

	// Store tiles.
	for (int i = 0; i < mesh->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = mesh->getTile(i);
		if (!tile || !tile->header || !tile->dataSize) continue;

		NavMeshTileHeader tileHeader;
		tileHeader.tileRef = mesh->getTileRef(tile);
		tileHeader.dataSize = tile->dataSize;
		fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

		fwrite(tile->data, tile->dataSize, 1, fp);
	}

	fclose(fp);
}

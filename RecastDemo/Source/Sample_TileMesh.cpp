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
#include <array>
#include <unordered_map>
#include <atomic>

#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu>
#else
#	include <GL/glu.h>
#endif
#include "imgui.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Sample_TileMesh.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourDebugDraw.h"
#include "NavMeshTesterTool.h"
#include "NavMeshPruneTool.h"
#include "OffMeshConnectionTool.h"
#include "ConvexVolumeTool.h"
#include "CrowdTool.h"
#include "AlgorithmHelper.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

namespace
{
	namespace exec = std::execution;

	constexpr int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
	constexpr int NAVMESHSET_VERSION = 1;

	inline unsigned int nextPow2(unsigned int v)
	{
		v--;
		v |= v >> 1;
		v |= v >> 2;
		v |= v >> 4;
		v |= v >> 8;
		v |= v >> 16;
		v++;
		return v;
	}

	inline unsigned int ilog2(unsigned int v)
	{
		unsigned int r{}, shift{};

		r = (v > 0xffff) << 4; v >>= r;
		shift = (v > 0xff) << 3; v >>= shift; r |= shift;
		shift = (v > 0xf) << 2; v >>= shift; r |= shift;
		shift = (v > 0x3) << 1; v >>= shift; r |= shift;
		r |= (v >> 1);

		return r;
	}

	template<class _Ty>
	constexpr inline size_t S_Cast(const _Ty draw_mode) { return (static_cast<size_t>(draw_mode)); }

	class NavMeshTileTool : public SampleTool
	{
		Sample_TileMesh* m_sample;
		float m_hitPos[3];
		bool m_hitPosSet;

	public:

		NavMeshTileTool() :
			m_sample(0), m_hitPosSet(false)
		{
			m_hitPos[0] = m_hitPos[1] = m_hitPos[2] = 0;
		}

		~NavMeshTileTool() = default;

		int type() override { return TOOL_TILE_EDIT; }

		void init(Sample* sample) override
		{
			m_sample = (Sample_TileMesh*)sample;
		}

		void reset() override {}

		void handleMenu() override
		{
			imguiLabel("Create Tiles");
			if (imguiButton("Create All"))
			{
				if (m_sample)
					m_sample->buildAllTiles();
			}
			if (imguiButton("Remove All"))
			{
				if (m_sample)
					m_sample->removeAllTiles();
			}
		}

		void handleClick(const float* /*s*/, const float* p, bool shift) override
		{
			m_hitPosSet = true;
			rcVcopy(m_hitPos, p);
			if (m_sample)
			{
				if (shift)
					m_sample->removeTile(m_hitPos);
				else
					m_sample->buildTile(m_hitPos);
			}
		}

		void handleToggle() override {}

		void handleStep() override {}

		void handleUpdate(const float /*dt*/) override {}

		void handleRender() override
		{
			if (m_hitPosSet)
			{
				const float s = m_sample->getAgentRadius();
				glColor4ub(0, 0, 0, 128);
				glLineWidth(2.0f);
				glBegin(GL_LINES);
				glVertex3f(m_hitPos[0] - s, m_hitPos[1] + 0.1f, m_hitPos[2]);
				glVertex3f(m_hitPos[0] + s, m_hitPos[1] + 0.1f, m_hitPos[2]);
				glVertex3f(m_hitPos[0], m_hitPos[1] - s + 0.1f, m_hitPos[2]);
				glVertex3f(m_hitPos[0], m_hitPos[1] + s + 0.1f, m_hitPos[2]);
				glVertex3f(m_hitPos[0], m_hitPos[1] + 0.1f, m_hitPos[2] - s);
				glVertex3f(m_hitPos[0], m_hitPos[1] + 0.1f, m_hitPos[2] + s);
				glEnd();
				glLineWidth(1.f);
			}
		}

		void handleRenderOverlay(double* proj, double* model, int* view) override
		{
			GLdouble x, y, z;
			if (m_hitPosSet && gluProject((GLdouble)m_hitPos[0], (GLdouble)m_hitPos[1], (GLdouble)m_hitPos[2],
				model, proj, view, &x, &y, &z))
			{
				int tx = 0, ty = 0;
				m_sample->getTilePos(m_hitPos, tx, ty);
				char text[32];
				snprintf(text, 32, "(%d,%d)", tx, ty);
				imguiDrawText((int)x, (int)y - 25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0, 0, 0, 220));
			}

			// Tool help
			const int h = view[3];
			imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Rebuild hit tile.  Shift+LMB: Clear hit tile.", imguiRGBA(255, 255, 255, 192));
		}
	};
}

Sample_TileMesh::Sample_TileMesh() :
	m_keepInterResults{}, m_buildAll(true), m_totalBuildTimeMs{}, m_drawMode(DrawMode::DRAWMODE_NAVMESH),
	m_maxTiles{}, m_maxPolysPerTile{}, m_tileSize(32), m_tileCol(duRGBA(0, 0, 0, 32)), m_tileBuildTime{},
	m_tileMemUsage{}, m_tileTriCount{}, m_lastBuiltTileBmin{}, m_lastBuiltTileBmax{}
{
	resetCommonSettings();
	setTool(new NavMeshTileTool);
}

Sample_TileMesh::~Sample_TileMesh()
{
	CleanUp();
	dtFreeNavMesh(m_navMesh);
	m_navMesh = nullptr;
}

void Sample_TileMesh::CleanUp()
{
	m_triareas.clear();
	m_solid = nullptr;
	rcFreeCompactHeightfield(m_chf.release());
	m_chf = nullptr;
	rcFreeContourSet(m_cset.release());
	m_cset = nullptr;
	rcFreePolyMesh(m_pmesh.release());
	m_pmesh = nullptr;
	rcFreePolyMeshDetail(m_dmesh.release());
	m_dmesh = nullptr;
}

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

void Sample_TileMesh::saveAll(const char* path, const dtNavMesh* mesh)
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

dtNavMesh* Sample_TileMesh::loadAll(const char* path)
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
			fclose(fp);
			return 0;
		}

		mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
	}

	fclose(fp);

	return mesh;
}

void Sample_TileMesh::handleSettings()
{
	Sample::handleCommonSettings();

	if (imguiCheck("Keep Itermediate Results", m_keepInterResults))
		m_keepInterResults = !m_keepInterResults;

	if (imguiCheck("Build All Tiles", m_buildAll))
		m_buildAll = !m_buildAll;

	imguiLabel("Tiling");
	imguiSlider("TileSize", &m_tileSize, 16.0f, 1024.0f, 16.0f);

	if (m_geom)
	{
		char text[64];
		int gw{}, gh{};
		const auto& bmin = m_geom->getNavMeshBoundsMin();
		const auto& bmax = m_geom->getNavMeshBoundsMax();

		rcCalcGridSize(bmin.data(), bmax.data(), m_cellSize, &gw, &gh);

		const int ts = (int)m_tileSize;
		const int tw = (gw + ts - 1) / ts;
		const int th = (gh + ts - 1) / ts;

		snprintf(text, 64, "Tiles  %d x %d", tw, th);
		imguiValue(text);

		// Max tiles and max polys affect how the tile IDs are caculated.
		// There are 22 bits available for identifying a tile and a polygon.
		int tileBits = rcMin((int)ilog2(nextPow2(tw * th)), 14);
		if (tileBits > 14) tileBits = 14;
		int polyBits = 22 - tileBits;
		m_maxTiles = 1 << tileBits;
		m_maxPolysPerTile = 1 << polyBits;
		snprintf(text, 64, "Max Tiles  %d", m_maxTiles);
		imguiValue(text);
		snprintf(text, 64, "Max Polys  %d", m_maxPolysPerTile);
		imguiValue(text);
	}
	else
	{
		m_maxTiles = 0;
		m_maxPolysPerTile = 0;
	}

	imguiSeparator();

	imguiIndent();
	imguiIndent();

	if (imguiButton("Save"))
	{
		saveAll("all_tiles_navmesh.bin", m_navMesh);
	}

	if (imguiButton("Load"))
	{
		dtFreeNavMesh(m_navMesh);
		m_navMesh = loadAll("all_tiles_navmesh.bin");
		m_navQuery->init(m_navMesh, 2048);
	}

	imguiUnindent();
	imguiUnindent();

	char msg[64];
	snprintf(msg, 64, "Build Time: %.1fms", m_totalBuildTimeMs);
	imguiLabel(msg);

	imguiSeparator();

	imguiSeparator();
}

void Sample_TileMesh::handleTools()
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
	if (imguiCheck("Create Tiles", type == TOOL_TILE_EDIT))
	{
		setTool(new NavMeshTileTool);
	}
	if (imguiCheck("Create Off-Mesh Links", type == TOOL_OFFMESH_CONNECTION))
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

void Sample_TileMesh::handleDebugMode()
{
	// Check which modes are valids.
	std::unordered_map<DrawMode, bool> valids{};

	if (m_geom)
	{
		valids[DrawMode::DRAWMODE_NAVMESH] = m_navMesh != 0;
		valids[DrawMode::DRAWMODE_NAVMESH_TRANS] = m_navMesh != 0;
		valids[DrawMode::DRAWMODE_NAVMESH_BVTREE] = m_navMesh != 0;
		valids[DrawMode::DRAWMODE_NAVMESH_NODES] = m_navQuery != 0;
		valids[DrawMode::DRAWMODE_NAVMESH_PORTALS] = m_navMesh != 0;
		valids[DrawMode::DRAWMODE_NAVMESH_INVIS] = m_navMesh != 0;
		valids[DrawMode::DRAWMODE_MESH] = true;
		valids[DrawMode::DRAWMODE_VOXELS] = m_solid != 0;
		valids[DrawMode::DRAWMODE_VOXELS_WALKABLE] = m_solid != 0;
		valids[DrawMode::DRAWMODE_COMPACT] = m_chf != 0;
		valids[DrawMode::DRAWMODE_COMPACT_DISTANCE] = m_chf != 0;
		valids[DrawMode::DRAWMODE_COMPACT_REGIONS] = m_chf != 0;
		valids[DrawMode::DRAWMODE_REGION_CONNECTIONS] = m_cset != 0;
		valids[DrawMode::DRAWMODE_RAW_CONTOURS] = m_cset != 0;
		valids[DrawMode::DRAWMODE_BOTH_CONTOURS] = m_cset != 0;
		valids[DrawMode::DRAWMODE_CONTOURS] = m_cset != 0;
		valids[DrawMode::DRAWMODE_POLYMESH] = m_pmesh != 0;
		valids[DrawMode::DRAWMODE_POLYMESH_DETAIL] = m_dmesh != 0;
	}

	std::atomic_int unavail{};

	For_Each(valids, [&](const auto& vaild) { if (!vaild.second) unavail++; }, exec::par);
	//for (int i = 0; i < MAX_DRAWMODE; ++i)
	//	if (!valids[i]) unavail++;

	if (unavail == S_Cast(DrawMode::MAX_DRAWMODE)) return;

	imguiLabel("Draw");

	if (auto dm = DrawMode::DRAWMODE_MESH; imguiCheck("Input Mesh", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_NAVMESH; imguiCheck("Navmesh", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_NAVMESH_INVIS; imguiCheck("Navmesh Invis", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_NAVMESH_TRANS; imguiCheck("Navmesh Trans", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_NAVMESH_BVTREE; imguiCheck("Navmesh BVTree", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_NAVMESH_NODES; imguiCheck("Navmesh Nodes", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_NAVMESH_PORTALS; imguiCheck("Navmesh Portals", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_VOXELS; imguiCheck("Voxels", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_VOXELS_WALKABLE; imguiCheck("Walkable Voxels", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_COMPACT; imguiCheck("Compact", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_COMPACT_DISTANCE; imguiCheck("Compact Distance", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_COMPACT_REGIONS; imguiCheck("Compact Regions", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_REGION_CONNECTIONS; imguiCheck("Region Connections", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_RAW_CONTOURS; imguiCheck("Raw Contours", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_BOTH_CONTOURS; imguiCheck("Both Contours", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_CONTOURS; imguiCheck("Contours", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_POLYMESH; imguiCheck("Poly Mesh", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;
	if (auto dm = DrawMode::DRAWMODE_POLYMESH_DETAIL; imguiCheck("Poly Mesh Detail", m_drawMode == dm, valids[dm]))
		m_drawMode = dm;

	if (unavail)
	{
		imguiValue("Tick 'Keep Itermediate Results'");
		imguiValue("rebuild some tiles to see");
		imguiValue("more debug mode options.");
	}
}

void Sample_TileMesh::handleRender()
{
	if (m_geom->isLoadGeomMeshEmpty()) return;

	const float texScale = 1.f / (m_cellSize * 10.0f);

	// Draw mesh // メッシュを描画
	if (m_drawMode != DrawMode::DRAWMODE_NAVMESH_TRANS)
	{
		for (auto& geom : m_geom->getLoadGeomMesh())
		{
			const auto& mesh{ geom.m_mesh };

			// Draw mesh // メッシュを描画
			duDebugDrawTriMeshSlope(&m_dd, mesh->getVerts(), mesh->getVertCount(),
				mesh->getTris(), mesh->getNormals(), mesh->getTriCount(),
				m_agentMaxSlope, texScale, geom.is_selected);
		}

		m_geom->drawOffMeshConnections(&m_dd);
	}

	glDepthMask(GL_FALSE);

	// Draw bounds // 境界を描く
	const auto& bmin = m_geom->getNavMeshBoundsMin();
	const auto& bmax = m_geom->getNavMeshBoundsMax();
	duDebugDrawBoxWire(&m_dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2],
		duRGBA(255, 255, 255, 128), 1.f);

	// Tiling grid.
	int gw{}, gh{};
	rcCalcGridSize(bmin.data(), bmax.data(), m_cellSize, &gw, &gh);
	const int tw = (gw + (int)m_tileSize - 1) / (int)m_tileSize;
	const int th = (gh + (int)m_tileSize - 1) / (int)m_tileSize;
	const float s = m_tileSize * m_cellSize;
	duDebugDrawGridXZ(&m_dd, bmin[0], bmin[1], bmin[2], tw, th, s, duRGBA(0, 0, 0, 64), 1.f);

	// Draw active tile
	duDebugDrawBoxWire(&m_dd, m_lastBuiltTileBmin[0], m_lastBuiltTileBmin[1], m_lastBuiltTileBmin[2],
		m_lastBuiltTileBmax[0], m_lastBuiltTileBmax[1], m_lastBuiltTileBmax[2], m_tileCol, 1.f);

	if (m_navMesh && m_navQuery &&
		(m_drawMode == DrawMode::DRAWMODE_NAVMESH || m_drawMode == DrawMode::DRAWMODE_NAVMESH_TRANS ||
			m_drawMode == DrawMode::DRAWMODE_NAVMESH_BVTREE || m_drawMode == DrawMode::DRAWMODE_NAVMESH_NODES ||
			m_drawMode == DrawMode::DRAWMODE_NAVMESH_PORTALS || m_drawMode == DrawMode::DRAWMODE_NAVMESH_INVIS))
	{
		if (m_drawMode != DrawMode::DRAWMODE_NAVMESH_INVIS)
			duDebugDrawNavMeshWithClosedList(&m_dd, *m_navMesh, *m_navQuery, m_navMeshDrawFlags);
		if (m_drawMode == DrawMode::DRAWMODE_NAVMESH_BVTREE)
			duDebugDrawNavMeshBVTree(&m_dd, *m_navMesh);
		if (m_drawMode == DrawMode::DRAWMODE_NAVMESH_PORTALS)
			duDebugDrawNavMeshPortals(&m_dd, *m_navMesh);
		if (m_drawMode == DrawMode::DRAWMODE_NAVMESH_NODES)
			duDebugDrawNavMeshNodes(&m_dd, *m_navQuery);

		duDebugDrawNavMeshPolysWithFlags(&m_dd, *m_navMesh, SAMPLE_POLYFLAGS_DISABLED, duRGBA(0, 0, 0, 128));
	}

	glDepthMask(GL_TRUE);

	if (m_chf && m_drawMode == DrawMode::DRAWMODE_COMPACT)
		duDebugDrawCompactHeightfieldSolid(&m_dd, *m_chf);

	if (m_chf && m_drawMode == DrawMode::DRAWMODE_COMPACT_DISTANCE)
		duDebugDrawCompactHeightfieldDistance(&m_dd, *m_chf);
	if (m_chf && m_drawMode == DrawMode::DRAWMODE_COMPACT_REGIONS)
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);
	if (m_solid && m_drawMode == DrawMode::DRAWMODE_VOXELS)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldSolid(&m_dd, *m_solid);
		glDisable(GL_FOG);
	}
	if (m_solid && m_drawMode == DrawMode::DRAWMODE_VOXELS_WALKABLE)
	{
		glEnable(GL_FOG);
		duDebugDrawHeightfieldWalkable(&m_dd, *m_solid);
		glDisable(GL_FOG);
	}

	if (m_cset && m_drawMode == DrawMode::DRAWMODE_RAW_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}

	if (m_cset && m_drawMode == DrawMode::DRAWMODE_BOTH_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawRawContours(&m_dd, *m_cset, 0.5f);
		duDebugDrawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_cset && m_drawMode == DrawMode::DRAWMODE_CONTOURS)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawContours(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_chf && m_cset && m_drawMode == DrawMode::DRAWMODE_REGION_CONNECTIONS)
	{
		duDebugDrawCompactHeightfieldRegions(&m_dd, *m_chf);

		glDepthMask(GL_FALSE);
		duDebugDrawRegionConnections(&m_dd, *m_cset);
		glDepthMask(GL_TRUE);
	}
	if (m_pmesh && m_drawMode == DrawMode::DRAWMODE_POLYMESH)
	{
		glDepthMask(GL_FALSE);
		duDebugDrawPolyMesh(&m_dd, *m_pmesh);
		glDepthMask(GL_TRUE);
	}
	if (m_dmesh && m_drawMode == DrawMode::DRAWMODE_POLYMESH_DETAIL)
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

void Sample_TileMesh::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x{}, y{}, z{};

	// Draw start and end point labels
	if (m_tileBuildTime > 0.0f && gluProject((GLdouble)(m_lastBuiltTileBmin[0] + m_lastBuiltTileBmax[0]) / 2, (GLdouble)(m_lastBuiltTileBmin[1] + m_lastBuiltTileBmax[1]) / 2, (GLdouble)(m_lastBuiltTileBmin[2] + m_lastBuiltTileBmax[2]) / 2,
		model, proj, view, &x, &y, &z))
	{
		char text[32];
		snprintf(text, 32, "%.3fms / %dTris / %.1fkB", m_tileBuildTime, m_tileTriCount, m_tileMemUsage);
		imguiDrawText((int)x, (int)y - 25, IMGUI_ALIGN_CENTER, text, imguiRGBA(0, 0, 0, 220));
	}

	if (m_tool)
		m_tool->handleRenderOverlay(proj, model, view);
	renderOverlayToolStates(proj, model, view);
}

void Sample_TileMesh::handleMeshChanged()
{
	Sample::handleMeshChanged();

	const BuildSettings* buildSettings = m_geom->getBuildSettings();
	if (buildSettings && buildSettings->tileSize > 0)
		m_tileSize = buildSettings->tileSize;

	CleanUp();

	dtFreeNavMesh(m_navMesh);
	m_navMesh = nullptr;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

bool Sample_TileMesh::handleBuild()
{
	if (m_geom->isLoadGeomMeshEmpty())
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles."); // 頂点と三角形はありません。
		return false;
	}

	dtFreeNavMesh(m_navMesh);

	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh."); // navmeshを割り当てることができませんでした。
		return false;
	}

	dtNavMeshParams params{};
	dtStatus status{};

	rcVcopy(params.orig, m_geom->getNavMeshBoundsMin().data());
	params.tileWidth = m_tileSize * m_cellSize;
	params.tileHeight = m_tileSize * m_cellSize;
	params.maxTiles = m_maxTiles;
	params.maxPolys = m_maxPolysPerTile;

	status = m_navMesh->init(&params);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh."); // ナビメッシュを初期化できませんでした。
		return false;
	}

	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query"); // Detour navmeshクエリを初期化できませんでした
		return false;
	}

	if (m_buildAll) buildAllTiles();

	if (m_tool) m_tool->init(this);

	initToolStates(this);

	return true;
}

void Sample_TileMesh::collectSettings(BuildSettings& settings)
{
	Sample::collectSettings(settings);

	settings.tileSize = m_tileSize;
}

void Sample_TileMesh::buildTile(const float* pos)
{
	if (!m_geom) return;
	if (!m_navMesh) return;

	const auto& bmin = m_geom->getNavMeshBoundsMin();
	const auto& bmax = m_geom->getNavMeshBoundsMax();

	const float ts = m_tileSize * m_cellSize;
	const int tx = (int)((pos[0] - bmin[0]) / ts);
	const int ty = (int)((pos[2] - bmin[2]) / ts);

	m_lastBuiltTileBmin[0] = bmin[0] + tx * ts;
	m_lastBuiltTileBmin[1] = bmin[1];
	m_lastBuiltTileBmin[2] = bmin[2] + ty * ts;

	m_lastBuiltTileBmax[0] = bmin[0] + (tx + 1) * ts;
	m_lastBuiltTileBmax[1] = bmax[1];
	m_lastBuiltTileBmax[2] = bmin[2] + (ty + 1) * ts;

	m_tileCol = duRGBA(255, 255, 255, 64);

	m_ctx->resetLog();

	int dataSize = 0;
	unsigned char* data = buildTileMesh(tx, ty, m_lastBuiltTileBmin, m_lastBuiltTileBmax, dataSize);

	// Remove any previous data (navmesh owns and deletes the data).
	// 以前のデータを削除します（navmeshはデータを所有および削除します）。
	m_navMesh->removeTile(m_navMesh->getTileRefAt(tx, ty, 0), 0, 0);

	// Add tile, or leave the location empty.
	// タイルを追加するか、場所を空のままにします。
	if (data)
	{
		// Let the navmesh own the data.
		// ナビメッシュにデータを所有させます。
		dtStatus status = m_navMesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, 0);

		if (dtStatusFailed(status)) dtFree(data);
	}

	m_ctx->dumpLog("Build Tile (%d,%d):", tx, ty);
}

void Sample_TileMesh::getTilePos(const float* pos, int& tx, int& ty)
{
	if (!m_geom) return;

	const auto& bmin = m_geom->getNavMeshBoundsMin();
	const float ts = m_tileSize * m_cellSize;

	tx = (int)((pos[0] - bmin[0]) / ts);
	ty = (int)((pos[2] - bmin[2]) / ts);
}

void Sample_TileMesh::removeTile(const float* pos)
{
	if (!m_geom) return;
	if (!m_navMesh) return;

	const auto& bmin = m_geom->getNavMeshBoundsMin();
	const auto& bmax = m_geom->getNavMeshBoundsMax();

	const float ts = m_tileSize * m_cellSize;
	const int tx = (int)((pos[0] - bmin[0]) / ts);
	const int ty = (int)((pos[2] - bmin[2]) / ts);

	m_lastBuiltTileBmin[0] = bmin[0] + tx * ts;
	m_lastBuiltTileBmin[1] = bmin[1];
	m_lastBuiltTileBmin[2] = bmin[2] + ty * ts;

	m_lastBuiltTileBmax[0] = bmin[0] + (tx + 1) * ts;
	m_lastBuiltTileBmax[1] = bmax[1];
	m_lastBuiltTileBmax[2] = bmin[2] + (ty + 1) * ts;

	m_tileCol = duRGBA(128, 32, 16, 64);

	m_navMesh->removeTile(m_navMesh->getTileRefAt(tx, ty, 0), 0, 0);
}

void Sample_TileMesh::buildAllTiles()
{
	if (!m_geom) return;
	if (!m_navMesh) return;

	const auto& bmin = m_geom->getNavMeshBoundsMin();
	const auto& bmax = m_geom->getNavMeshBoundsMax();
	int gw{}, gh{};

	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);

	const int ts = (int)m_tileSize;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;
	const float tcs = m_tileSize * m_cellSize;

	// Start the build process.
	// ビルドプロセスを開始します。
	m_ctx->startTimer(RC_TIMER_TEMP);

	for (int y = 0; y < th; ++y)
	{
		for (int x = 0; x < tw; ++x)
		{
			m_lastBuiltTileBmin[0] = bmin[0] + x * tcs;
			m_lastBuiltTileBmin[1] = bmin[1];
			m_lastBuiltTileBmin[2] = bmin[2] + y * tcs;

			m_lastBuiltTileBmax[0] = bmin[0] + (x + 1) * tcs;
			m_lastBuiltTileBmax[1] = bmax[1];
			m_lastBuiltTileBmax[2] = bmin[2] + (y + 1) * tcs;

			int dataSize{};
			unsigned char* data = buildTileMesh(x, y, m_lastBuiltTileBmin, m_lastBuiltTileBmax, dataSize);

			if (data)
			{
				// Remove any previous data (navmesh owns and deletes the data).
				// 以前のデータを削除します（navmeshはデータを所有および削除します）。
				m_navMesh->removeTile(m_navMesh->getTileRefAt(x, y, 0), 0, 0);

				// Let the navmesh own the data.
				// navmeshにデータを所有させます。
				dtStatus status = m_navMesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, 0);

				if (dtStatusFailed(status)) dtFree(data);
			}
		}
	}

	// Stop the build process.
	// ビルドプロセスを停止します。
	m_ctx->stopTimer(RC_TIMER_TEMP);

	m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TEMP) / 1000.0f;
}

void Sample_TileMesh::removeAllTiles()
{
	if (!m_geom || !m_navMesh)
		return;

	const auto& bmin = m_geom->getNavMeshBoundsMin();
	const auto& bmax = m_geom->getNavMeshBoundsMax();
	int gw{}, gh{};

	rcCalcGridSize(bmin.data(), bmax.data(), m_cellSize, &gw, &gh);

	const int ts = (int)m_tileSize;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;

	for (int y = 0; y < th; ++y)
		for (int x = 0; x < tw; ++x)
			m_navMesh->removeTile(m_navMesh->getTileRefAt(x, y, 0), 0, 0);
}

unsigned char* Sample_TileMesh::buildTileMesh(const int tx, const int ty, const std::array<float, 3>& bmin,
	const std::array<float, 3>& bmax, int& dataSize)
{
	if (m_geom->isLoadGeomMeshEmpty())
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");  // 入力メッシュが指定されていません。
		return nullptr;
	}

	m_tileMemUsage = 0;
	m_tileBuildTime = 0;

	CleanUp();

	// Init build configuration from GUI
	// GUIからのビルド構成の初期化
	m_cfg = {};
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
	m_cfg.tileSize = (int)m_tileSize;
	m_cfg.borderSize = m_cfg.walkableRadius + 3; // Reserve enough padding. // 十分なパディングを予約します。
	m_cfg.width = m_cfg.tileSize + m_cfg.borderSize * 2;
	m_cfg.height = m_cfg.tileSize + m_cfg.borderSize * 2;
	m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

	// Expand the heighfield bounding box by border size to find the extents of geometry we need to build this tile.
	// このタイルを構築するために必要なジオメトリの範囲を見つけるために、境界線サイズで地形バウンディングボックスを展開します。
	//
	// This is done in order to make sure that the navmesh tiles connect correctly at the borders,
	// and the obstacles close to the border work correctly with the dilation process.
	// これは、ナビメッシュタイルが境界で正しく接続され、境界に近い障害物が膨張プロセスで正しく機能することを確認するために行われます。
	// No polygons (or contours) will be created on the border area.
	// 境界領域にポリゴン（または輪郭）は作成されません。
	//
	// IMPORTANT! // 重要！
	//
	//   :''''''''':
	//   : +-----+ :
	//   : |     | :
	//   : |     |<--- tile to build // 構築するタイル
	//   : |     | :
	//   : +-----+ :<-- geometry needed // ジオメトリが必要
	//   :.........:
	//
	// You should use this bounding box to query your input geometry.
	// この境界ボックスを使用して、入力ジオメトリを照会する必要があります。
	//
	// For example if you build a navmesh for terrain, and want the navmesh tiles to match the terrain tile size
	// you will need to pass in data from neighbour terrain tiles too!
	// たとえば、地形のnavmeshを構築し、navmeshタイルを地形タイルのサイズに一致させたい場合、
	// 隣接する地形タイルからもデータを渡す必要があります！
	// In a simple case, just pass in all the 8 neighbours,
	// or use the bounding box below to only pass in a sliver of each of the 8 neighbours.
	// 単純な場合、8つの隣すべてを渡すか、下の境界ボックスを使用して、8つの隣人それぞれのスライバーだけを渡します。
	rcVcopy(m_cfg.bmin, bmin.data());
	rcVcopy(m_cfg.bmax, bmax.data());
	m_cfg.bmin[0] -= m_cfg.borderSize * m_cfg.cs;
	m_cfg.bmin[2] -= m_cfg.borderSize * m_cfg.cs;
	m_cfg.bmax[0] += m_cfg.borderSize * m_cfg.cs;
	m_cfg.bmax[2] += m_cfg.borderSize * m_cfg.cs;

	// Reset build times gathering.
	// ビルド時間の収集をリセットします。
	m_ctx->resetTimers();

	// Start the build process.
	// ビルドプロセスを開始します。
	m_ctx->startTimer(RC_TIMER_TOTAL);

	if (!m_pmesh)	m_pmesh.reset(rcAllocPolyMesh());
	if (!m_dmesh)	m_dmesh.reset(rcAllocPolyMeshDetail());

	std::vector<rcPolyMesh*> poly_meshes;
	std::vector<rcPolyMeshDetail*> detail_meshes;

	for (const auto& geom : m_geom->getLoadGeomMesh())
	{
		const auto& mesh{ geom.m_mesh };
		const auto& chunkyMesh = geom.m_chunkyMesh;

		const float* verts = mesh->getVerts();
		const int nverts = mesh->getVertCount();
		const int ntris = mesh->getTriCount();

		m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
		m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);
		m_ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f, ntris / 1000.0f);

		// Allocate voxel heightfield where we rasterize our input data to.
		// 入力データをラスタライズするボクセルハイトフィールドを割り当てます。
		try
		{
			m_solid.reset(rcAllocHeightfield());
		}
		catch (const std::exception&)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'."); // メモリー不足「solid」
			return nullptr;
		}

		if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield."); // ソリッドハイトフィールドを作成できませんでした。
			return nullptr;
		}

		// Allocate array that can hold triangle flags.
		// If you have multiple meshes you need to process, allocate
		// and array which can hold the max number of triangles you need to process.
		// 三角形のフラグを保持できる配列を割り当てます。
		// 処理する必要のあるメッシュが複数ある場合、処理する必要のある三角形の最大数を保持できる配列および割り当てと配列。
		try
		{
			m_triareas.resize(chunkyMesh->maxTrisPerChunk, 0);
		}
		catch (const std::exception&)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk); // メモリー不足「m_triareas」
			return nullptr;
		}

		float tbmin[2]{}, tbmax[2]{};

		tbmin[0] = m_cfg.bmin[0];
		tbmin[1] = m_cfg.bmin[2];
		tbmax[0] = m_cfg.bmax[0];
		tbmax[1] = m_cfg.bmax[2];

		std::array<int, 512> cid{}; // TODO: Make grow when returning too many items. // 戻るアイテムが多すぎる場合はサイズを大きくさせる。
		const int ncid = rcGetChunksOverlappingRect(&(*chunkyMesh), tbmin, tbmax, cid.data(), 512);

		if (!ncid) continue;

		m_tileTriCount = 0;

		for (int i = 0; i < ncid; ++i)
		{
			const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
			const int* ctris = &chunkyMesh->tris[node.i * 3];
			const int nctris = node.n;

			m_tileTriCount += nctris;

			Fill(m_triareas, 0, exec::par);
			rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle,
				verts, nverts, ctris, nctris, m_triareas.data(), SAMPLE_AREAMOD_GROUND);

			if (!rcRasterizeTriangles(m_ctx, verts, nverts, ctris, m_triareas.data(), nctris, *m_solid, m_cfg.walkableClimb))
				return 0;
		}

		if (!m_keepInterResults)
		{
			m_triareas.clear();
		}

		// Once all geometry is rasterized, we do initial pass of filtering to
		// remove unwanted overhangs caused by the conservative rasterization
		// as well as filter spans where the character cannot possibly stand.
		// すべてのジオメトリがラスタライズされると、最初のフィルタリングパスを実行して、
		// 保守的なラスタライゼーションによって引き起こされる不要なオーバーハングと、
		// 文字が耐えられない可能性があるフィルタスパンを削除します。
		if (m_filterLowHangingObstacles)
			rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);

		if (m_filterLedgeSpans)
			rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);

		if (m_filterWalkableLowHeightSpans)
			rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);

		// Compact the heightfield so that it is faster to handle from now on.
		// This will result more cache coherent data as well as the neighbours
		// between walkable cells will be calculated.
		// ハイトフィールドを圧縮して、今後の処理が高速になるようにします。
		// これにより、より多くのキャッシュコヒーレントデータが生成され、ウォーク可能セル間の隣接セルが計算されます。
		try
		{
			m_chf.reset(rcAllocCompactHeightfield());
		}
		catch (const std::exception&)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'."); // メモリー不足「chf」
			return nullptr;
		}

		if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data."); // コンパクトなデータを構築できませんでした。
			return nullptr;
		}

		if (!m_keepInterResults)
		{
			m_solid = nullptr;
		}

		// Erode the walkable area by agent radius.
		// エージェントの半径ごとに歩行可能エリアを侵食します。
		if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");  // 侵食できませんでした。
			return nullptr;
		}

		// (Optional) Mark areas.
		//（オプション）エリアをマークします。
		const auto* vols = m_geom->getConvexVolumes();

		for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
			rcMarkConvexPolyArea(m_ctx, vols->at(i).verts.data(), vols->at(i).nverts, vols->at(i).hmin, vols->at(i).hmax,
				vols->at(i).areaMod, *m_chf);

		// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
		// There are 3 martitioning methods, each with some pros and cons:
		// 高さフィールドを分割して、後で簡単なアルゴリズムを使用して歩行可能エリアを三角測量できるようにします。
		// それぞれ3つの長所と短所がある3つのマトリションメソッドがあります。
		// 1) Watershed partitioning
		//   - the classic Recast partitioning
		//   - creates the nicest tessellation
		//   - usually slowest
		//   - partitions the heightfield into nice regions without holes or overlaps
		//   - the are some corner cases where this method creates produces holes and overlaps
		//      - holes may appear when a small obstacles is close to large open area (triangulation can handle this)
		//      - overlaps may occur if you have narrow spiral corridors (i.e stairs), this make triangulation to fail
		//   * generally the best choice if you precompute the nacmesh, use this if you have large open areas
		// 2) Monotone partioning
		//   - fastest
		//   - partitions the heightfield into regions without holes and overlaps (guaranteed)
		//   - creates long thin polygons, which sometimes causes paths with detours
		//   * use this if you want fast navmesh generation
		// 3) Layer partitoining
		//   - quite fast
		//   - partitions the heighfield into non-overlapping regions
		//   - relies on the triangulation code to cope with holes (thus slower than monotone partitioning)
		//   - produces better triangles than monotone partitioning
		//   - does not have the corner cases of watershed partitioning
		//   - can be slow and create a bit ugly tessellation (still better than monotone)
		//     if you have large open areas with small obstacles (not a problem if you use tiles)
		//   * good choice to use for tiled navmesh with medium and small sized tiles
		//
		// 高さフィールドを分割して、後で簡単なアルゴリズムを使用して歩行可能エリアを三角測量できるようにします。
		// それぞれ3つの長所と短所がある3つのマトリションメソッドがあります。
		//  1）分水界分割
		//		-従来のリキャストパーティション
		//		-最も良いテッセレーションを作成します
		//		-通常は最も遅い
		//		-高さフィールドを、穴や重なりのない素敵な領域に分割します
		//		-このメソッドが作成するいくつかのコーナーケースは、穴とオーバーラップを生成します
		//			-小さな障害物が大きな開口部に近い場合に穴が現れることがあります（三角測量でこれを処理できます）
		//			-狭い螺旋状の通路（階段など）がある場合、重複が発生する可能性があり、これにより三角測量が失敗します
		//		*ナクメッシュを事前計算する場合は一般的に最良の選択、大きな空き領域がある場合はこれを使用する
		//
		//  2）モノトーン分割
		//		-最速
		//		-高さフィールドを穴や重複のない領域に分割します（保証）
		//		-長くて細いポリゴンを作成します
		//			* navmeshの高速生成が必要な場合はこれを使用します
		//
		//  3）レイヤー分割
		//		- かなり速いです
		//		-重なった領域を重複しない領域に分割します
		//		-穴に対処するために三角形分割コードに依存します（したがって、単調な分割よりも遅い）
		//		-モノトーン分割よりも優れた三角形を生成します
		//		-流域分割のコーナーケースはありません
		//		-遅く、少しいテッセレーションを作成できます（モノトーンよりも優れています）
		//		 小さな障害物のある大きな空き領域がある場合（タイルを使用する場合は問題ありません）
		//		*中サイズと小サイズのタイルでタイル張りされたnavmeshに使用するのに良い選択

		switch (m_partitionType)
		{
			case SAMPLE_PARTITION_WATERSHED:
			{
				// Prepare for region partitioning, by calculating distance field along the walkable surface.
				// 歩行可能な表面に沿った距離フィールドを計算して、領域分割の準備をします。
				if (!rcBuildDistanceField(m_ctx, *m_chf))
				{
					m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field."); // 距離フィールドを構築できませんでした。
					return nullptr;
				}

				// Partition the walkable surface into simple regions without holes.
				//　歩行可能な表面を穴のない単純な領域に分割します。
				if (!rcBuildRegions(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
				{
					m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions."); // 流域を構築できませんでした。
					return nullptr;
				}

				break;
			}
			case SAMPLE_PARTITION_MONOTONE:
			{
				// Partition the walkable surface into simple regions without holes.
				// Monotone partitioning does not need distancefield.
				// 歩行可能な表面を穴のない単純な領域に分割します。
				// モノトーン分割は距離フィールドを必要としません。
				if (!rcBuildRegionsMonotone(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
				{
					m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions."); // モノトーン領域を構築できませんでした。
					return nullptr;
				}

				break;
			}
			case SAMPLE_PARTITION_LAYERS:
			{
				// Partition the walkable surface into simple regions without holes.
				// 歩行可能な表面を穴のない単純な領域に分割します。
				if (!rcBuildLayerRegions(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea))
				{
					m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions."); // レイヤー領域を構築できませんでした。
					return nullptr;
				}

				break;
			}
		}

		// Create contours.
		// 輪郭を作成します。
		m_cset.reset(rcAllocContourSet());
		if (!m_cset)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'."); // メモリー不足「cset」
			return nullptr;
		}

		if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours."); // 輪郭を作成できませんでした。
			return nullptr;
		}

		if (m_cset->nconts == 0) continue;

		// Build polygon navmesh from the contours.
		// 輪郭からポリゴンナビメッシュを作成します。
		try
		{
			poly_meshes.emplace_back(rcAllocPolyMesh());
		}
		catch (const std::exception&)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'."); // メモリー不足「pmesh」
			return nullptr;
		}

		if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *poly_meshes.back()))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours."); // 輪郭を三角測量できませんでした。
			return nullptr;
		}

		// Build detail mesh.
		// 詳細メッシュを作成します。
		try
		{
			detail_meshes.emplace_back(rcAllocPolyMeshDetail());
		}
		catch (const std::exception&)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'dmesh'."); // メモリー不足「dmesh」
			return nullptr;
		}

		if (!rcBuildPolyMeshDetail(m_ctx, *poly_meshes.back(), *m_chf,
			m_cfg.detailSampleDist, m_cfg.detailSampleMaxError,
			*detail_meshes.back()))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build polymesh detail."); // ポリゴンメッシュの詳細を作成できませんでした。（コメントまさかのnotつけ忘れ(笑)）
			return nullptr;
		}

		if (!m_keepInterResults)
		{
			rcFreeCompactHeightfield(m_chf.release());
			m_chf = nullptr;
			rcFreeContourSet(m_cset.release());
			m_cset = nullptr;
		}
	}

	// poly_meshをm_pmeshへマージする
	if (!rcMergePolyMeshes(m_ctx, poly_meshes, *m_pmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not merge polymesh detail."); // ポリゴンメッシュのマージに失敗
		return nullptr;
	}

	// detail_meshをm_dmeshへマージする
	if (!rcMergePolyMeshDetails(m_ctx, detail_meshes, *m_dmesh))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not merge polymesh detail."); // ポリゴンメッシュのマージに失敗
		return nullptr;
	}

	For_Each(poly_meshes, [](auto* pmesh) { rcFreePolyMesh(pmesh); pmesh = nullptr; }, exec::par);
	For_Each(detail_meshes, [](auto* dmesh) { rcFreePolyMeshDetail(dmesh); dmesh = nullptr; }, exec::par);

	unsigned char* navData{};
	int navDataSize{};

	if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (m_pmesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			// 頂点インデックスはushortsであり、0xffffを超える頂点を指すことはできません。
			m_ctx->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", m_pmesh->nverts, 0xffff);
			return nullptr;
		}

		// Update poly flags from areas.
		// エリアからポリゴンフラグを更新します。
		for (int i = 0; i < m_pmesh->npolys; ++i)
		{
			m_pmesh->flags[i] = sampleAreaToFlags(m_pmesh->areas[i]);
		}

		dtNavMeshCreateParams params{};

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
		params.tileX = tx;
		params.tileY = ty;
		params.tileLayer = 0;
		rcVcopy(params.bmin, m_pmesh->bmin);
		rcVcopy(params.bmax, m_pmesh->bmax);
		params.cs = m_cfg.cs;
		params.ch = m_cfg.ch;
		params.buildBvTree = true;

		if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		{
			m_ctx->log(RC_LOG_ERROR, "Could not build Detour navmesh.");
			return nullptr;
		}
	}
	m_tileMemUsage = navDataSize / 1024.0f;

	m_ctx->stopTimer(RC_TIMER_TOTAL);

	// Show performance stats.
	// パフォーマンスの統計を表示します。
	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_pmesh->nverts, m_pmesh->npolys);

	m_tileBuildTime = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;
	dataSize = navDataSize;

	return navData;
}
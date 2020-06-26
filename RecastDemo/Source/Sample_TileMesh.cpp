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
#include "DetourCommon.h"
#include "fastlz.h"
#include "DetourTileCache.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

#ifdef _DEBUG
#define   new	new(_NORMAL_BLOCK, __FILE__, __LINE__)
#endif

#define _ENABLE_ATOMIC_ALIGNMENT_FIX

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
	constexpr inline int S_Cast(const _Ty draw_mode) { return (static_cast<int>(draw_mode)); }

	class NavMeshTileTool : public SampleTool
	{
		Sample_TileMesh* m_sample;
		float m_hitPos[3];
		bool m_hitPosSet;
		int m_drawType;

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

		void handleClickDown(const float* /*s*/, const float* p, bool shift) override
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

		void handleClickUp(const float* /*s*/, const float* /*p*/) override {}
		void handleClick(const float* /*s*/, const float* /*p*/) override {}

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

	// This value specifies how many layers (or "floors") each navmesh tile is expected to have.
	//���̒l�́A�enavmesh�^�C���ɕK�v�ȃ��C���[�i�܂��́u�t���A�v�j�̐����w�肵�܂��B
	constexpr int EXPECTED_LAYERS_PER_TILE = 4;

	bool isectSegAABB(const float* sp, const float* sq,
		const float* amin, const float* amax,
		float& tmin, float& tmax)
	{
		constexpr float EPS = 1e-6f;

		float d[3];
		rcVsub(d, sq, sp);
		tmin = 0;  // set to -FLT_MAX to get first hit on line
		// -FLT_MAX�ɐݒ肵�āA����ōŏ��̃q�b�g���擾���܂�
		tmax = FLT_MAX;		// set to max distance ray can travel (for segment)
		// �������ړ��ł���ő勗���ɐݒ肵�܂��i�Z�O�����g�p�j

		// For all three slabs
		// 3�̃X���u���ׂ�
		for (int i = 0; i < 3; i++)
		{
			if (fabsf(d[i]) < EPS)
			{
				// Ray is parallel to slab. No hit if origin not within slab
				// �����̓X���u�ɕ��s�ł��B ���_���X���u���ɂȂ��ꍇ�̓q�b�g�Ȃ�
				if (sp[i] < amin[i] || sp[i] > amax[i])
					return false;
			}
			else
			{
				// Compute intersection t value of ray with near and far plane of slab
				//�X���u�̃j�A����уt�@�[�v���[���ƃ��C�̌���t�l���v�Z���܂�
				const float ood = 1.f / d[i];
				float t1 = (amin[i] - sp[i]) * ood;
				float t2 = (amax[i] - sp[i]) * ood;

				// Make t1 be intersection with near plane, t2 with far plane
				// t1���߂��̕��ʂƌ��������At2�������̕��ʂƌ��������܂�
				if (t1 > t2) rcSwap(t1, t2);

				// Compute the intersection of slab intersections intervals
				// �X���u�̌����Ԋu�Ƃ̌������v�Z
				if (t1 > tmin) tmin = t1;
				if (t2 < tmax) tmax = t2;

				// Exit with no collision as soon as slab intersection becomes empty
				// �X���u�̌����_�������Ȃ�Ƃ����ɏՓ˂Ȃ��ŏI��
				if (tmin > tmax) return false;
			}
		}

		return true;
	}

	inline constexpr int calcLayerBufferSize(const int gridWidth, const int gridHeight)
	{
		constexpr int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
		const int gridSize = gridWidth * gridHeight;
		return headerSize + gridSize * 4;
	}

	constexpr int MAX_LAYERS = 32;
}

struct FastLZCompressor : public dtTileCacheCompressor
{
	virtual int maxCompressedSize(const int bufferSize)
	{
		return (int)(bufferSize * 1.05f);
	}

	virtual dtStatus compress(const unsigned char* buffer, const int bufferSize,
		unsigned char* compressed, const int /*maxCompressedSize*/, int* compressedSize)
	{
		*compressedSize = fastlz_compress((const void* const)buffer, bufferSize, compressed);
		return DT_SUCCESS;
	}

	virtual dtStatus decompress(const unsigned char* compressed, const int compressedSize,
		unsigned char* buffer, const int maxBufferSize, int* bufferSize)
	{
		*bufferSize = fastlz_decompress(compressed, compressedSize, buffer, maxBufferSize);
		return *bufferSize < 0 ? DT_FAILURE : DT_SUCCESS;
	}
};

struct LinearAllocator : public dtTileCacheAlloc
{
	unsigned char* buffer;
	size_t capacity;
	size_t top;
	size_t high;

	LinearAllocator(const size_t cap) : buffer(0), capacity(0), top(0), high(0)
	{
		resize(cap);
	}

	~LinearAllocator()
	{
		dtFree(buffer);
	}

	void resize(const size_t cap)
	{
		if (buffer) dtFree(buffer);
		buffer = (unsigned char*)dtAlloc(cap, DT_ALLOC_PERM);
		capacity = cap;
	}

	virtual void reset()
	{
		high = dtMax(high, top);
		top = 0;
	}

	virtual void* alloc(const size_t size)
	{
		if (!buffer)
			return 0;
		if (top + size > capacity)
			return 0;
		unsigned char* mem = &buffer[top];
		top += size;
		return mem;
	}

	virtual void free(void* /*ptr*/)
	{
		// Empty
	}
};

struct MeshProcess : public dtTileCacheMeshProcess
{
	InputGeom* m_geom;

	inline MeshProcess() : m_geom(0)
	{
	}

	inline void init(InputGeom* geom)
	{
		m_geom = geom;
	}

	virtual void process(struct dtNavMeshCreateParams* params,
		unsigned char* polyAreas, unsigned short* polyFlags)
	{
		// Update poly flags from areas.
		for (int i = 0; i < params->polyCount; ++i)
		{
			polyFlags[i] = sampleAreaToFlags(polyAreas[i]);
		}

		// Pass in off-mesh connections.
		if (m_geom)
		{
			params->offMeshConVerts = m_geom->getOffMeshConnectionVerts().data();
			params->offMeshConRad = m_geom->getOffMeshConnectionRads().data();
			params->offMeshConDir = m_geom->getOffMeshConnectionDirs().data();
			params->offMeshConAreas = m_geom->getOffMeshConnectionAreas().data();
			params->offMeshConFlags = m_geom->getOffMeshConnectionFlags().data();
			params->offMeshConUserID = m_geom->getOffMeshConnectionId().data();
			params->offMeshConCount = m_geom->getOffMeshConnectionCount();
		}
	}
};

struct TileCacheData
{
	unsigned char* data;
	int dataSize;
};

struct RasterizationContext
{
	RasterizationContext() :
		solid(0),
		lset(0),
		chf(0),
		ntiles(0)
	{
		tiles.fill({});
	}

	~RasterizationContext()
	{
		rcFreeHeightField(solid);
		triareas.clear();
		rcFreeHeightfieldLayerSet(lset);
		rcFreeCompactHeightfield(chf);
		for (auto& tile : tiles)
		{
			dtFree(tile.data);
			tile.data = nullptr;
		}
	}

	rcHeightfield* solid;
	std::vector<UINT8> triareas;
	rcHeightfieldLayerSet* lset;
	rcCompactHeightfield* chf;
	std::array<TileCacheData, MAX_LAYERS> tiles;
	int ntiles;
};

Sample_TileMesh::Sample_TileMesh() :
	m_keepInterResults{}, m_buildAll(true), m_totalBuildTimeMs{}, m_drawMode(DrawMode::DRAWMODE_NAVMESH),
	m_maxTiles{}, m_maxPolysPerTile{}, m_tileSize(32), m_tileCol(duRGBA(0, 0, 0, 32)), m_tileBuildTime{},
	m_tileMemUsage{}, m_tileTriCount{}, m_lastBuiltTileBmin{}, m_lastBuiltTileBmax{}
{
	resetCommonSettings();

	m_pmesh.reset(rcAllocPolyMesh());
	m_dmesh.reset(rcAllocPolyMeshDetail());

	setTool(new NavMeshTileTool);
}

Sample_TileMesh::~Sample_TileMesh()
{
	CleanUp();
	dtFreeNavMesh(m_navMesh);
	m_navMesh = nullptr;
	rcFreePolyMesh(m_pmesh.release());
	m_pmesh = nullptr;
	rcFreePolyMeshDetail(m_dmesh.release());
	m_dmesh = nullptr;
}

void Sample_TileMesh::CleanUp()
{
	m_triareas.clear();
	m_solid = nullptr;
	rcFreeCompactHeightfield(m_chf.release());
	m_chf = nullptr;
	rcFreeContourSet(m_cset.release());
	m_cset = nullptr;
	rcFreePolyMesh(m_last_pmesh.release());
	m_last_pmesh = nullptr;
	rcFreePolyMeshDetail(m_last_dmesh.release());
	m_last_dmesh = nullptr;
}

struct NavMeshSetHeader
{
	int magic;
	int version;
	int numTiles;
	dtNavMeshParams meshParams;
	dtTileCacheParams cacheParams;
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
	memcpy(&header.meshParams, m_navMesh->getParams(), sizeof(dtNavMeshParams));
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
		// Error or early EOF
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
	dtStatus status = mesh->init(&header.meshParams);
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
			// Error or early EOF
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
			// Error or early EOF
			dtFree(data);
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

	int gridSize{ 1 };
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

		gridSize = tw * th;
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

	// Draw mesh1 // ���b�V����`��
	if (m_drawMode != DrawMode::DRAWMODE_NAVMESH_TRANS)
	{
		for (auto& geom : m_geom->getLoadGeomMesh())
		{
			const auto& mesh{ geom.m_mesh };

			// Draw mesh1 // ���b�V����`��
			duDebugDrawTriMeshSlope(&m_dd, mesh->getVerts(), mesh->getVertCount(),
				mesh->getTris(), mesh->getNormals(), mesh->getTriCount(),
				m_agentMaxSlope, texScale, geom.is_selected);
		}

		m_geom->drawOffMeshConnections(&m_dd);
	}

	glDepthMask(GL_FALSE);

	// Draw bounds // ���E��`��
	const auto& bmin = m_geom->getNavMeshBoundsMin();
	const auto& bmax = m_geom->getNavMeshBoundsMax();
	duDebugDrawBoxWire(&m_dd, bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2],
		duRGBA(255, 255, 255, 128), 1.f);

	// Tiling grid. // �ŉ����ɂ���O���b�g��`��
	int gw{}, gh{};
	rcCalcGridSize(bmin.data(), bmax.data(), m_cellSize, &gw, &gh);
	const int tw = (gw + (int)m_tileSize - 1) / (int)m_tileSize;
	const int th = (gh + (int)m_tileSize - 1) / (int)m_tileSize;
	const float s = m_tileSize * m_cellSize;
	duDebugDrawGridXZ(&m_dd, bmin[0], bmin[1], bmin[2], tw, th, s, duRGBA(0, 0, 0, 64), 1.f);

	// Draw active tile // �L���ȃ^�C����̒����́i��j��`��
	duDebugDrawBoxWire(&m_dd, m_lastBuiltTileBmin[0], m_lastBuiltTileBmin[1], m_lastBuiltTileBmin[2],
		m_lastBuiltTileBmax[0], m_lastBuiltTileBmax[1], m_lastBuiltTileBmax[2], m_tileCol, 1.f);

	if (m_navMesh && m_navQuery &&
		(m_drawMode == DrawMode::DRAWMODE_NAVMESH || m_drawMode == DrawMode::DRAWMODE_NAVMESH_TRANS ||
			m_drawMode == DrawMode::DRAWMODE_NAVMESH_BVTREE || m_drawMode == DrawMode::DRAWMODE_NAVMESH_NODES ||
			m_drawMode == DrawMode::DRAWMODE_NAVMESH_PORTALS || m_drawMode == DrawMode::DRAWMODE_NAVMESH_INVIS))
	{
		// �i�r���b�V���̕`��
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
	if (m_tileBuildTime > 0.0f &&
		gluProject((GLdouble)(m_lastBuiltTileBmin[0] + m_lastBuiltTileBmax[0]) / 2,
			(GLdouble)(m_lastBuiltTileBmin[1] + m_lastBuiltTileBmax[1]) / 2,
			(GLdouble)(m_lastBuiltTileBmin[2] + m_lastBuiltTileBmax[2]) / 2, model, proj, view, &x, &y, &z))
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
	m_navMesh = 0;

	if (m_tool)
	{
		m_tool->reset();
		m_tool->init(this);
	}
	resetToolStates();
	initToolStates(this);
}

int Sample_TileMesh::rasterizeTileLayers(
	const int tx, const int ty, const rcConfig& cfg, TileCacheData* tiles, const int maxTiles)
{
	if (m_geom->isLoadGeomMeshEmpty())
	{
		m_ctx->log(RC_LOG_ERROR, "buildTile: Input mesh is not specified."); // ���̓��b�V�����w�肳��Ă��܂���B
		return 0;
	}

	FastLZCompressor comp{};
	RasterizationContext rc{};

	const auto& mesh{ m_geom->getMeshAt(0) };
	const auto& verts = mesh->getVerts();
	const int nverts = mesh->getVertCount();
	const auto& chunkyMesh = m_geom->getChunkyMeshAt(0);

	// Tile bounds.
	// �^�C�����E
	const float tcs = cfg.tileSize * cfg.cs;

	rcConfig tcfg;
	memcpy(&tcfg, &cfg, sizeof(tcfg));

	tcfg.bmin[0] = cfg.bmin[0] + tx * tcs;
	tcfg.bmin[1] = cfg.bmin[1];
	tcfg.bmin[2] = cfg.bmin[2] + ty * tcs;
	tcfg.bmax[0] = cfg.bmin[0] + (tx + 1) * tcs;
	tcfg.bmax[1] = cfg.bmax[1];
	tcfg.bmax[2] = cfg.bmin[2] + (ty + 1) * tcs;
	tcfg.bmin[0] -= tcfg.borderSize * tcfg.cs;
	tcfg.bmin[2] -= tcfg.borderSize * tcfg.cs;
	tcfg.bmax[0] += tcfg.borderSize * tcfg.cs;
	tcfg.bmax[2] += tcfg.borderSize * tcfg.cs;

	// Allocate voxel heightfield where we rasterize our input data to.
	// ���̓f�[�^�����X�^���C�Y����{�N�Z���n�`�����蓖�Ă܂��B
	rc.solid = rcAllocHeightfield();
	if (!rc.solid)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'."); // �������[�s���usolid�v
		return 0;
	}

	if (!rcCreateHeightfield(m_ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield."); // �\���b�h�n�`���쐬�ł��܂���ł����B
		return 0;
	}

	// Allocate array that can hold triangle flags.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	// �O�p�`�̃t���O��ێ��ł���z������蓖�Ă܂��B
	// ��������K�v������O�p�`�̍ő吔��ێ��ł��镡���̃��b�V���������A���蓖�āA�z�񂷂�K�v������ꍇ�B
	try
	{
		rc.triareas.resize(chunkyMesh->maxTrisPerChunk, 0);
	}
	catch (const std::exception&)
	{
		// �������[�s���um_triareas�v
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk); // �������[�s���um_triareas�v
		return 0;
	}

	float tbmin[2]{}, tbmax[2]{};
	tbmin[0] = tcfg.bmin[0];
	tbmin[1] = tcfg.bmin[2];
	tbmax[0] = tcfg.bmax[0];
	tbmax[1] = tcfg.bmax[2];

	int cid[512]{}; // TODO: Make grow when returning too many items.  // �ԕi����A�C�e������������ꍇ�͐��������܂��B
	const int ncid = rcGetChunksOverlappingRect(&(*chunkyMesh), tbmin, tbmax, cid, 512);

	if (!ncid) return 0; // empty

	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
		const int* tris = &chunkyMesh->tris[node.i * 3];
		const int ntris = node.n;

		std::fill_n(rc.triareas.begin(), ntris, 0);

		rcMarkWalkableTriangles(m_ctx, tcfg.walkableSlopeAngle,
			verts.data(), nverts, tris, ntris, rc.triareas.data(),
			SAMPLE_AREAMOD_GROUND);

		if (!rcRasterizeTriangles(m_ctx, verts, nverts, tris, rc.triareas.data(), ntris, *rc.solid, tcfg.walkableClimb))
			return 0;
	}

	// Once all geometry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	// ���ׂẴW�I���g�������X�^���C�Y�����ƁA�ŏ��̃t�B���^�����O�p�X�����s���āA
	// �ێ�I�ȃ��X�^���C�[�[�V�����ɂ���Ĉ����N�������s�v�ȃI�[�o�[�n���O�ƁA
	// �������ς����Ȃ��\��������t�B���^�X�p�����폜���܂��B
	if (m_filterLowHangingObstacles)
		rcFilterLowHangingWalkableObstacles(m_ctx, tcfg.walkableClimb, *rc.solid);

	if (m_filterLedgeSpans)
		rcFilterLedgeSpans(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);

	if (m_filterWalkableLowHeightSpans)
		rcFilterWalkableLowHeightSpans(m_ctx, tcfg.walkableHeight, *rc.solid);

	rc.chf = rcAllocCompactHeightfield();
	if (!rc.chf)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'."); // �������[�s���uchf�v
		return 0;
	}

	if (!rcBuildCompactHeightfield(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid, *rc.chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data."); // �R���p�N�g�ȃf�[�^���\�z�ł��܂���ł����B
		return 0;
	}

	// Erode the walkable area by agent radius.
	//�@�G�[�W�F���g�̔��a���Ƃɕ��s�\�G���A��N�H���܂��B
	if (!rcErodeWalkableArea(m_ctx, tcfg.walkableRadius, *rc.chf))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode."); // �N�H�ł��܂���ł����B
		return 0;
	}

	// (Optional) Mark areas.
	//�i�I�v�V�����j�G���A���}�[�N���܂��B
	const auto* vols{ m_geom->getConvexVolumes() };

	for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
	{
		rcMarkConvexPolyArea(m_ctx, vols->at(i).verts.data(), vols->at(i).nverts,
			vols->at(i).hmin, vols->at(i).hmax,
			vols->at(i).areaMod, *rc.chf);
	}

	// Recast�A���P�[�^�[���g�p���āA�n�`���C���[�Z�b�g�����蓖�Ă�
	rc.lset = rcAllocHeightfieldLayerSet();
	if (!rc.lset)
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'lset'."); // �������[�s���ulset�v
		return 0;
	}
	// �R���p�N�g�Ȓn�`���烌�C���[�Z�b�g���\�z
	if (!rcBuildHeightfieldLayers(m_ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
	{
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build heighfield layers.");
		return 0;
	}

	rc.ntiles = 0;
	for (int i = 0; i < rcMin(rc.lset->nlayers, MAX_LAYERS); ++i)
	{
		TileCacheData* tile = &rc.tiles[rc.ntiles++];
		const rcHeightfieldLayer* layer = &rc.lset->layers[i];

		// Store header
		// �w�b�_�[��ۑ�
		dtTileCacheLayerHeader header;
		header.magic = DT_TILECACHE_MAGIC;
		header.version = DT_TILECACHE_VERSION;

		// Tile layer location in the navmesh.
		// navmesh�̃^�C�����C���[�̏ꏊ�B
		header.tx = tx;
		header.ty = ty;
		header.tlayer = i;
		dtVcopy(header.bmin, layer->bmin);
		dtVcopy(header.bmax, layer->bmax);

		// Tile info.
		header.width = (unsigned char)layer->width;
		header.height = (unsigned char)layer->height;
		header.minx = (unsigned char)layer->minx;
		header.maxx = (unsigned char)layer->maxx;
		header.miny = (unsigned char)layer->miny;
		header.maxy = (unsigned char)layer->maxy;
		header.hmin = (unsigned short)layer->hmin;
		header.hmax = (unsigned short)layer->hmax;

		dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons,
			&tile->data, &tile->dataSize);
		if (dtStatusFailed(status))
		{
			return 0;
		}
	}

	// Transfer ownsership of tile data from build context to the caller.
	// �^�C���f�[�^�̏��L�����r���h�R���e�L�X�g����Ăяo�����ɓ]�����܂��B
	int n = 0;
	for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
	{
		tiles[n++] = rc.tiles[i];
		rc.tiles[i].data = 0;
		rc.tiles[i].dataSize = 0;
	}

	return n;
}

bool Sample_TileMesh::handleBuild()
{
	if (m_geom->isLoadGeomMeshEmpty())
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: No vertices and triangles."); // ���_�ƎO�p�`�͂���܂���B
		return false;
	}

	dtFreeNavMesh(m_navMesh);

	m_navMesh = dtAllocNavMesh();
	if (!m_navMesh)
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not allocate navmesh."); // navmesh�����蓖�Ă邱�Ƃ��ł��܂���ł����B
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
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init navmesh."); // �i�r���b�V�����������ł��܂���ł����B
		return false;
	}

	status = m_navQuery->init(m_navMesh, 2048);
	if (dtStatusFailed(status))
	{
		m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not init Detour navmesh query"); // Detour navmesh�N�G�����������ł��܂���ł���
		return false;
	}

	if (m_buildAll)
	{
		if (!buildAllTiles())
		{
			m_ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not build all tiles"); // �S�Ẵ^�C���𐶐��ł��܂���ł����B
			return false;
		}
	}

	if (m_tool) m_tool->init(this);

	initToolStates(this);

	return true;
}

void Sample_TileMesh::handleUpdate(const float dt)
{
	Sample::handleUpdate(dt);
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
	// �ȑO�̃f�[�^���폜���܂��inavmesh�̓f�[�^�����L����э폜���܂��j�B
	m_navMesh->removeTile(m_navMesh->getTileRefAt(tx, ty, 0), 0, 0);

	// Add tile, or leave the location empty.
	// �^�C����ǉ����邩�A�ꏊ����̂܂܂ɂ��܂��B
	if (data)
	{
		// Let the navmesh own the data.
		// �i�r���b�V���Ƀf�[�^�����L�����܂��B
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

bool Sample_TileMesh::buildAllTiles()
{
	if (!m_geom) return false;
	if (!m_navMesh) return false;

	// Init cache // �L���b�V���̏�����
	const auto& bmin = m_geom->getNavMeshBoundsMin();
	const auto& bmax = m_geom->getNavMeshBoundsMax();
	int gw{}, gh{};

	rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);

	const int ts = (int)m_tileSize;
	const int tw = (gw + ts - 1) / ts;
	const int th = (gh + ts - 1) / ts;
	const float tcs = m_tileSize * m_cellSize;

	// Start the build process.
	// �r���h�v���Z�X���J�n���܂��B
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
				// �ȑO�̃f�[�^���폜���܂��inavmesh�̓f�[�^�����L����э폜���܂��j�B
				m_navMesh->removeTile(m_navMesh->getTileRefAt(x, y, 0), 0, 0);

				// Let the navmesh own the data.
				// navmesh�Ƀf�[�^�����L�����܂��B
				dtStatus status = m_navMesh->addTile(data, dataSize, DT_TILE_FREE_DATA, 0, 0);

				if (dtStatusFailed(status)) dtFree(data);
			}
		}
	}

	// Stop the build process.
	// �r���h�v���Z�X���~���܂��B
	m_ctx->stopTimer(RC_TIMER_TEMP);

	m_totalBuildTimeMs = m_ctx->getAccumulatedTime(RC_TIMER_TEMP) / 1000.0f;

	const dtNavMesh* nav = m_navMesh;
	int navmeshMemUsage{};

	for (int i = 0; i < nav->getMaxTiles(); ++i)
	{
		const dtMeshTile* tile = nav->getTile(i);
		if (tile->header)
			navmeshMemUsage += tile->dataSize;
	}

	printf("navmeshMemUsage = %.1f kB", navmeshMemUsage / 1024.0f);

	return true;
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
		m_ctx->log(RC_LOG_ERROR, "buildNavigation: Input mesh is not specified.");  // ���̓��b�V�����w�肳��Ă��܂���B
		return nullptr;
	}

	m_tileMemUsage = 0;
	m_tileBuildTime = 0;

	CleanUp();

	// Init build configuration from GUI
	// GUI����̃r���h�\���̏�����
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
	m_cfg.borderSize = m_cfg.walkableRadius + 3; // Reserve enough padding. // �\���ȃp�f�B���O��\�񂵂܂��B
	m_cfg.width = m_cfg.tileSize + m_cfg.borderSize * 2;
	m_cfg.height = m_cfg.tileSize + m_cfg.borderSize * 2;
	m_cfg.detailSampleDist = m_detailSampleDist < 0.9f ? 0 : m_cellSize * m_detailSampleDist;
	m_cfg.detailSampleMaxError = m_cellHeight * m_detailSampleMaxError;

	// Expand the heighfield bounding box by border size to find the extents of geometry we need to build this tile.
	// ���̃^�C�����\�z���邽�߂ɕK�v�ȃW�I���g���͈̔͂������邽�߂ɁA���E���T�C�Y�Œn�`�o�E���f�B���O�{�b�N�X��W�J���܂��B
	//
	// This is done in order to make sure that the navmesh tiles connect correctly at the borders,
	// and the obstacles close to the border work correctly with the dilation process.
	// ����́A�i�r���b�V���^�C�������E�Ő������ڑ�����A���E�ɋ߂���Q�����c���v���Z�X�Ő������@�\���邱�Ƃ��m�F���邽�߂ɍs���܂��B
	// No polygons (or contours) will be created on the border area.
	// ���E�̈�Ƀ|���S���i�܂��͗֊s�j�͍쐬����܂���B
	//
	// IMPORTANT! // �d�v�I
	//
	//   :''''''''':
	//   : +-----+ :
	//   : |     | :
	//   : |     |<--- tile to build // �\�z����^�C��
	//   : |     | :
	//   : +-----+ :<-- geometry needed // �W�I���g�����K�v
	//   :.........:
	//
	// You should use this bounding box to query your input geometry.
	// ���̋��E�{�b�N�X���g�p���āA���̓W�I���g�����Ɖ��K�v������܂��B
	//
	// For example if you build a navmesh for terrain, and want the navmesh tiles to match the terrain tile size
	// you will need to pass in data from neighbour terrain tiles too!
	// ���Ƃ��΁A�n�`��navmesh���\�z���Anavmesh�^�C����n�`�^�C���̃T�C�Y�Ɉ�v���������ꍇ�A
	// �אڂ���n�`�^�C��������f�[�^��n���K�v������܂��I
	// In a simple case, just pass in all the 8 neighbours,
	// or use the bounding box below to only pass in a sliver of each of the 8 neighbours.
	// �P���ȏꍇ�A8�ׂ̗��ׂĂ�n�����A���̋��E�{�b�N�X���g�p���āA8�̗אl���ꂼ��̃X���C�o�[������n���܂��B
	rcVcopy(m_cfg.bmin, bmin.data());
	rcVcopy(m_cfg.bmax, bmax.data());
	m_cfg.bmin[0] -= m_cfg.borderSize * m_cfg.cs;
	m_cfg.bmin[2] -= m_cfg.borderSize * m_cfg.cs;
	m_cfg.bmax[0] += m_cfg.borderSize * m_cfg.cs;
	m_cfg.bmax[2] += m_cfg.borderSize * m_cfg.cs;

	// Reset build times gathering.
	// �r���h���Ԃ̎��W�����Z�b�g���܂��B
	m_ctx->resetTimers();

	// Start the build process.
	// �r���h�v���Z�X���J�n���܂��B
	m_ctx->startTimer(RC_TIMER_TOTAL);

	std::vector<rcPolyMesh*> poly_meshes;
	std::vector<rcPolyMeshDetail*> detail_meshes;

	if (!m_last_pmesh) m_last_pmesh.reset(rcAllocPolyMesh());
	if (!m_last_dmesh) m_last_dmesh.reset(rcAllocPolyMeshDetail());

	for (const auto& geom : m_geom->getLoadGeomMesh())
	{
		const auto& mesh{ geom.m_mesh };
		const auto& chunkyMesh{ geom.m_chunkyMesh };

		const auto& verts = mesh->getVerts();
		const int nverts = mesh->getVertCount();
		const int ntris = mesh->getTriCount();

		m_ctx->log(RC_LOG_PROGRESS, "Building navigation:");
		m_ctx->log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);
		m_ctx->log(RC_LOG_PROGRESS, " - %.1fK verts, %.1fK tris", nverts / 1000.0f, ntris / 1000.0f);

		// Allocate voxel heightfield where we rasterize our input data to.
		// ���̓f�[�^�����X�^���C�Y����{�N�Z���n�C�g�t�B�[���h�����蓖�Ă܂��B
		try
		{
			m_solid.reset(rcAllocHeightfield());
		}
		catch (const std::exception&)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'."); // �������[�s���usolid�v
			return nullptr;
		}

		if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield."); // �\���b�h�n�C�g�t�B�[���h���쐬�ł��܂���ł����B
			return nullptr;
		}

		// Allocate array that can hold triangle flags.
		// If you have multiple meshes you need to process, allocate
		// and array which can hold the max number of triangles you need to process.
		// �O�p�`�̃t���O��ێ��ł���z������蓖�Ă܂��B
		// ��������K�v�̂��郁�b�V������������ꍇ�A��������K�v�̂���O�p�`�̍ő吔��ێ��ł���z�񂨂�ъ��蓖�ĂƔz��B
		try
		{
			m_triareas.resize(chunkyMesh->maxTrisPerChunk, 0);
		}
		catch (const std::exception&)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", chunkyMesh->maxTrisPerChunk); // �������[�s���um_triareas�v
			return nullptr;
		}

		float tbmin[2]{}, tbmax[2]{};

		tbmin[0] = m_cfg.bmin[0];
		tbmin[1] = m_cfg.bmin[2];
		tbmax[0] = m_cfg.bmax[0];
		tbmax[1] = m_cfg.bmax[2];

		std::array<int, 512> cid{}; // TODO: Make grow when returning too many items. // �߂�A�C�e������������ꍇ�̓T�C�Y��傫��������B
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

			// �\�ʂ����s�\���ǂ������m�F
			rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle,
				verts.data(), nverts, ctris, nctris, m_triareas.data(), SAMPLE_AREAMOD_GROUND);

			if (!rcRasterizeTriangles(m_ctx, verts, nverts, ctris, m_triareas.data(), nctris, *m_solid, m_cfg.walkableClimb))
				return nullptr;
		}

		if (!m_keepInterResults)
		{
			m_triareas.clear();
		}

		// Once all geometry is rasterized, we do initial pass of filtering to
		// remove unwanted overhangs caused by the conservative rasterization
		// as well as filter spans where the character cannot possibly stand.
		// ���ׂẴW�I���g�������X�^���C�Y�����ƁA�ŏ��̃t�B���^�����O�p�X�����s���āA
		// �ێ�I�ȃ��X�^���C�[�[�V�����ɂ���Ĉ����N�������s�v�ȃI�[�o�[�n���O�ƁA
		// �������ς����Ȃ��\��������t�B���^�X�p�����폜���܂��B
		if (m_filterLowHangingObstacles)
			rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);

		if (m_filterLedgeSpans)
			rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);

		if (m_filterWalkableLowHeightSpans)
			rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);

		// Compact the heightfield so that it is faster to handle from now on.
		// This will result more cache coherent data as well as the neighbours
		// between walkable cells will be calculated.
		// �n�C�g�t�B�[���h�����k���āA����̏����������ɂȂ�悤�ɂ��܂��B
		// ����ɂ��A��葽���̃L���b�V���R�q�[�����g�f�[�^����������A�E�H�[�N�\�Z���Ԃ̗אڃZ�����v�Z����܂��B
		try
		{
			m_chf = std::make_unique<rcCompactHeightfield>();
		}
		catch (const std::exception&)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'."); // �������[�s���uchf�v
			return nullptr;
		}

		if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data."); // �R���p�N�g�ȃf�[�^���\�z�ł��܂���ł����B
			return nullptr;
		}

		if (!m_keepInterResults)
		{
			m_solid = nullptr;
		}

		// Erode the walkable area by agent radius.
		// �G�[�W�F���g�̔��a���Ƃɕ��s�\�G���A��N�H���܂��B
		if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");  // �N�H�ł��܂���ł����B
			return nullptr;
		}

		// (Optional) Mark areas.
		//�i�I�v�V�����j�G���A���}�[�N���܂��B
		const auto* vols = m_geom->getConvexVolumes();

		for (int i = 0; i < m_geom->getConvexVolumeCount(); ++i)
			rcMarkConvexPolyArea(m_ctx, vols->at(i).verts.data(), vols->at(i).nverts, vols->at(i).hmin, vols->at(i).hmax,
				vols->at(i).areaMod, *m_chf);

		// Partition the heightfield so that we can use simple algorithm later to triangulate the walkable areas.
		// There are 3 martitioning methods, each with some pros and cons:
		// �����t�B�[���h�𕪊����āA��ŊȒP�ȃA���S���Y�����g�p���ĕ��s�\�G���A���O�p���ʂł���悤�ɂ��܂��B
		// ���ꂼ��3�̒����ƒZ��������3�̃}�g���V�������\�b�h������܂��B
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
		// �����t�B�[���h�𕪊����āA��ŊȒP�ȃA���S���Y�����g�p���ĕ��s�\�G���A���O�p���ʂł���悤�ɂ��܂��B
		// ���ꂼ��3�̒����ƒZ��������3�̃}�g���V�������\�b�h������܂��B
		//  1�j�����E����
		//		-�]���̃��L���X�g�p�[�e�B�V����
		//		-�ł��ǂ��e�b�Z���[�V�������쐬���܂�
		//		-�ʏ�͍ł��x��
		//		-�����t�B�[���h���A����d�Ȃ�̂Ȃ��f�G�ȗ̈�ɕ������܂�
		//		-���̃��\�b�h���쐬���邢�����̃R�[�i�[�P�[�X�́A���ƃI�[�o�[���b�v�𐶐����܂�
		//			-�����ȏ�Q�����傫�ȊJ�����ɋ߂��ꍇ�Ɍ�������邱�Ƃ�����܂��i�O�p���ʂł���������ł��܂��j
		//			-����������̒ʘH�i�K�i�Ȃǁj������ꍇ�A�d������������\��������A����ɂ��O�p���ʂ����s���܂�
		//		*�i�N���b�V�������O�v�Z����ꍇ�͈�ʓI�ɍŗǂ̑I���A�傫�ȋ󂫗̈悪����ꍇ�͂�����g�p����
		//
		//  2�j���m�g�[������
		//		-�ő�
		//		-�����t�B�[���h������d���̂Ȃ��̈�ɕ������܂��i�ۏ؁j
		//		-�����čׂ��|���S�����쐬���܂�
		//			* navmesh�̍����������K�v�ȏꍇ�͂�����g�p���܂�
		//
		//  3�j���C���[����
		//		- ���Ȃ葬���ł�
		//		-�d�Ȃ����̈���d�����Ȃ��̈�ɕ������܂�
		//		-���ɑΏ����邽�߂ɎO�p�`�����R�[�h�Ɉˑ����܂��i���������āA�P���ȕ��������x���j
		//		-���m�g�[�����������D�ꂽ�O�p�`�𐶐����܂�
		//		-���敪���̃R�[�i�[�P�[�X�͂���܂���
		//		-�x���A�������e�b�Z���[�V�������쐬�ł��܂��i���m�g�[�������D��Ă��܂��j
		//		 �����ȏ�Q���̂���傫�ȋ󂫗̈悪����ꍇ�i�^�C�����g�p����ꍇ�͖�肠��܂���j
		//		*���T�C�Y�Ə��T�C�Y�̃^�C���Ń^�C�����肳�ꂽnavmesh�Ɏg�p����̂ɗǂ��I��

		switch (m_partitionType)
		{
			case SAMPLE_PARTITION_WATERSHED:
			{
				// Prepare for region partitioning, by calculating distance field along the walkable surface.
				// ���s�\�ȕ\�ʂɉ����������t�B�[���h���v�Z���āA�̈敪���̏��������܂��B
				if (!rcBuildDistanceField(m_ctx, *m_chf))
				{
					m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field."); // �����t�B�[���h���\�z�ł��܂���ł����B
					return nullptr;
				}

				// Partition the walkable surface into simple regions without holes.
				//�@���s�\�ȕ\�ʂ����̂Ȃ��P���ȗ̈�ɕ������܂��B
				if (!rcBuildRegions(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
				{
					m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions."); // ������\�z�ł��܂���ł����B
					return nullptr;
				}

				break;
			}
			case SAMPLE_PARTITION_MONOTONE:
			{
				// Partition the walkable surface into simple regions without holes.
				// Monotone partitioning does not need distancefield.
				// ���s�\�ȕ\�ʂ����̂Ȃ��P���ȗ̈�ɕ������܂��B
				// ���m�g�[�������͋����t�B�[���h��K�v�Ƃ��܂���B
				if (!rcBuildRegionsMonotone(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
				{
					m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build monotone regions."); // ���m�g�[���̈���\�z�ł��܂���ł����B
					return nullptr;
				}

				break;
			}
			case SAMPLE_PARTITION_LAYERS:
			{
				// Partition the walkable surface into simple regions without holes.
				// ���s�\�ȕ\�ʂ����̂Ȃ��P���ȗ̈�ɕ������܂��B
				if (!rcBuildLayerRegions(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea))
				{
					m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build layer regions."); // ���C���[�̈���\�z�ł��܂���ł����B
					return nullptr;
				}

				break;
			}
		}

		// Create contours.
		// �֊s���쐬���܂��B
		m_cset.reset(rcAllocContourSet());
		if (!m_cset)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'."); // �������[�s���ucset�v
			return nullptr;
		}

		if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours."); // �֊s���쐬�ł��܂���ł����B
			return nullptr;
		}

		if (m_cset->nconts == 0) continue;

		// Build polygon navmesh from the contours.
		// �֊s����|���S���i�r���b�V�����쐬���܂��B
		try
		{
			poly_meshes.emplace_back(rcAllocPolyMesh());
		}
		catch (const std::exception&)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'."); // �������[�s���upmesh�v
			return nullptr;
		}

		if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *poly_meshes.back()))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours."); // �֊s���O�p���ʂł��܂���ł����B
			return nullptr;
		}

		// Build detail mesh1.
		// �ڍ׃��b�V�����쐬���܂��B
		try
		{
			detail_meshes.emplace_back(rcAllocPolyMeshDetail());
		}
		catch (const std::exception&)
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'dmesh'."); // �������[�s���udmesh�v
			return nullptr;
		}

		if (!rcBuildPolyMeshDetail(m_ctx, *poly_meshes.back(), *m_chf,
			m_cfg.detailSampleDist, m_cfg.detailSampleMaxError,
			*detail_meshes.back()))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build polymesh detail."); // �|���S�����b�V���̏ڍׂ��쐬�ł��܂���ł����B�i�R�����g�܂�����not���Y��(��)�j
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

	// �}�[�W
	{
		// poly_mesh
		if (!rcMergePolyMeshes(m_ctx, poly_meshes, m_last_pmesh.get()))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not merge polymesh."); // �|���S�����b�V���̃}�[�W�Ɏ��s
			return nullptr;
		}

		// detail_mesh
		if (!rcMergePolyMeshDetails(m_ctx, detail_meshes, m_last_dmesh.get()))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not merge polymesh detail."); // �ڍ׃��b�V���̃}�[�W�Ɏ��s
			return nullptr;
		}
	}

	// ��U���Z�b�g
	For_Each(poly_meshes, [](auto* pmesh) { rcFreePolyMesh(pmesh); }, exec::par);
	For_Each(detail_meshes, [](auto* dmesh) { rcFreePolyMeshDetail(dmesh); }, exec::par);

	// ��U������
	poly_meshes.clear();
	detail_meshes.clear();

	// �������[�Ċm��
	for (size_t i = 0; i < 2; i++)
	{
		poly_meshes.emplace_back(rcAllocPolyMesh());
		detail_meshes.emplace_back(rcAllocPolyMeshDetail());
	}

	// �R�s�[
	{
		// poly_mesh
		if (!(rcCopyPolyMesh(m_ctx, *m_last_pmesh, poly_meshes[0]) &&
			rcCopyPolyMesh(m_ctx, *m_pmesh, poly_meshes[1])))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not copy polymesh."); // �|���S�����b�V���̃}�[�W�Ɏ��s
			return nullptr;
		}

		// detail_mesh
		if (!(rcCopyPolyMeshDetail(m_ctx, *m_last_dmesh, detail_meshes[0]) &&
			rcCopyPolyMeshDetail(m_ctx, *m_dmesh, detail_meshes[1])))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not copy polymesh detail."); // �ڍ׃��b�V���̃}�[�W�Ɏ��s
			return nullptr;
		}
	}

	// �ă}�[�W
	{
		// poly_mesh
		if (!rcMergePolyMeshes(m_ctx, poly_meshes, m_pmesh.get()))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not merge polymesh."); // �|���S�����b�V���̃}�[�W�Ɏ��s
			return nullptr;
		}

		// detail_mesh
		if (!rcMergePolyMeshDetails(m_ctx, detail_meshes, m_dmesh.get()))
		{
			m_ctx->log(RC_LOG_ERROR, "buildNavigation: Could not merge polymesh detail."); // �ڍ׃��b�V���̃}�[�W�Ɏ��s
			return nullptr;
		}
	}

	unsigned char* navData{};
	int navDataSize{};

	if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
	{
		if (m_last_pmesh->nverts >= 0xffff)
		{
			// The vertex indices are ushorts, and cannot point to more than 0xffff vertices.
			// ���_�C���f�b�N�X��ushorts�ł���A0xffff�𒴂��钸�_���w�����Ƃ͂ł��܂���B
			m_ctx->log(RC_LOG_ERROR, "Too many vertices per tile %d (max: %d).", m_last_pmesh->nverts, 0xffff);
			return nullptr;
		}

		// Update poly flags from areas.
		// �G���A����|���S���t���O���X�V���܂��B
		for (int i = 0; i < m_last_pmesh->npolys; ++i)
		{
			m_last_pmesh->flags[i] = sampleAreaToFlags(m_last_pmesh->areas[i]);
		}

		dtNavMeshCreateParams params{};

		params.verts = m_last_pmesh->verts;
		params.vertCount = m_last_pmesh->nverts;
		params.polys = m_last_pmesh->polys;
		params.polyAreas = m_last_pmesh->areas;
		params.polyFlags = m_last_pmesh->flags;
		params.polyCount = m_last_pmesh->npolys;
		params.nvp = m_last_pmesh->nvp;
		params.detailMeshes = m_last_dmesh->meshes;
		params.detailVerts = m_last_dmesh->verts;
		params.detailVertsCount = m_last_dmesh->nverts;
		params.detailTris = m_last_dmesh->tris;
		params.detailTriCount = m_last_dmesh->ntris;
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
		rcVcopy(params.bmin, m_last_pmesh->bmin);
		rcVcopy(params.bmax, m_last_pmesh->bmax);
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
	// �p�t�H�[�}���X�̓��v��\�����܂��B
	duLogBuildTimes(*m_ctx, m_ctx->getAccumulatedTime(RC_TIMER_TOTAL));
	m_ctx->log(RC_LOG_PROGRESS, ">> Polymesh: %d vertices  %d polygons", m_last_pmesh->nverts, m_last_pmesh->npolys);

	m_tileBuildTime = m_ctx->getAccumulatedTime(RC_TIMER_TOTAL) / 1000.0f;
	dataSize = navDataSize;

	For_Each(poly_meshes, [](auto* pmesh) { rcFreePolyMesh(pmesh); pmesh = nullptr; }, exec::par);
	For_Each(detail_meshes, [](auto* dmesh) { rcFreePolyMeshDetail(dmesh); dmesh = nullptr; }, exec::par);

	return navData;
}
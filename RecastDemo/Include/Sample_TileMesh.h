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

#ifndef RECASTSAMPLETILEMESH_H
#define RECASTSAMPLETILEMESH_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "Recast.h"
#include "ChunkyTriMesh.h"

enum class DrawDetailType
{
	DRAWDETAIL_AREAS,
	DRAWDETAIL_REGIONS,
	DRAWDETAIL_CONTOURS,
	DRAWDETAIL_MESH,
};

class Sample_TileMesh : public Sample
{
private:
	bool m_keepInterResults;
	bool m_buildAll;
	float m_totalBuildTimeMs;

	std::vector<unsigned char> m_triareas;
	std::unique_ptr<rcHeightfield> m_solid;
	std::unique_ptr<rcCompactHeightfield> m_chf;
	std::unique_ptr<rcContourSet> m_cset;
	std::unique_ptr<rcPolyMesh> m_pmesh, m_last_pmesh;
	std::unique_ptr<rcPolyMeshDetail> m_dmesh, m_last_dmesh;
	rcConfig m_cfg;

	enum class DrawMode
	{
		DRAWMODE_NAVMESH,
		DRAWMODE_NAVMESH_TRANS,
		DRAWMODE_NAVMESH_BVTREE,
		DRAWMODE_NAVMESH_NODES,
		DRAWMODE_NAVMESH_PORTALS,
		DRAWMODE_NAVMESH_INVIS,
		DRAWMODE_MESH,
		DRAWMODE_VOXELS,
		DRAWMODE_VOXELS_WALKABLE,
		DRAWMODE_COMPACT,
		DRAWMODE_COMPACT_DISTANCE,
		DRAWMODE_COMPACT_REGIONS,
		DRAWMODE_REGION_CONNECTIONS,
		DRAWMODE_RAW_CONTOURS,
		DRAWMODE_BOTH_CONTOURS,
		DRAWMODE_CONTOURS,
		DRAWMODE_POLYMESH,
		DRAWMODE_POLYMESH_DETAIL,
		MAX_DRAWMODE
	};

	DrawMode m_drawMode;

	int m_maxTiles;
	int m_maxPolysPerTile;
	float m_tileSize;

	uint32_t m_tileCol;
	std::array<float, 3> m_lastBuiltTileBmin, m_lastBuiltTileBmax;
	float m_tileBuildTime;
	float m_tileMemUsage;
	int m_tileTriCount;

	unsigned char* buildTileMesh(const int tx, const int ty, const std::array<float, 3>& bmin,
		const std::array<float, 3>& bmax, int& dataSize);

	void CleanUp();

	void saveAll(const char* path, const dtNavMesh* mesh);
	dtNavMesh* loadAll(const char* path);

public:
	Sample_TileMesh();
	virtual ~Sample_TileMesh();

	void handleSettings() override;
	void handleTools() override;
	void handleDebugMode() override;
	void handleRender() override;
	void handleRenderOverlay(double* proj, double* model, int* view) override;
	void handleMeshChanged() override;
	bool handleBuild() override;
	void handleUpdate(const float dt) override;
	void collectSettings(struct BuildSettings& settings) override;

	void getTilePos(const float* pos, int& tx, int& ty);

	void buildTile(const float* pos);
	void removeTile(const float* pos);
	bool buildAllTiles();
	void removeAllTiles();

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Sample_TileMesh(const Sample_TileMesh&) = delete;
	Sample_TileMesh& operator=(Sample_TileMesh&) = delete;
	Sample_TileMesh(const Sample_TileMesh&&) = delete;
	Sample_TileMesh& operator=(Sample_TileMesh&&) = delete;

	int rasterizeTileLayers(const int tx, const int ty, const rcConfig& cfg, struct TileCacheData* tiles, const int maxTiles);
};

#endif // RECASTSAMPLETILEMESH_H

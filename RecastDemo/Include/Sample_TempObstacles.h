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

#ifndef RECASTSAMPLETEMPOBSTACLE_H
#define RECASTSAMPLETEMPOBSTACLE_H

#include "Sample.h"
#include "Common.h"
#include "DetourNavMesh.h"
#include "Recast.h"
#include "ChunkyTriMesh.h"
#include "DetourTileCache.h"

class Sample_TempObstacles : public Sample
{
protected:
	bool m_keepInterResults;

	std::unique_ptr<struct LinearAllocator> m_talloc;
	std::unique_ptr<struct FastLZCompressor> m_tcomp;
	std::unique_ptr<struct MeshProcess> m_tmproc;

	class dtTileCache* m_tileCache;

	float m_cacheBuildTimeMs;
	int m_cacheCompressedSize;
	int m_cacheRawSize;
	int m_cacheLayerCount;
	unsigned int m_cacheBuildMemUsage;
	int m_maxTiles;
	int m_maxPolysPerTile;
	float m_tileSize;

	enum DrawMode
	{
		DRAWMODE_NAVMESH,
		DRAWMODE_NAVMESH_TRANS,
		DRAWMODE_NAVMESH_BVTREE,
		DRAWMODE_NAVMESH_NODES,
		DRAWMODE_NAVMESH_PORTALS,
		DRAWMODE_NAVMESH_INVIS,
		DRAWMODE_MESH,
		DRAWMODE_CACHE_BOUNDS,
		MAX_DRAWMODE
	};

	DrawMode m_drawMode;

public:
	Sample_TempObstacles();
	~Sample_TempObstacles();

	void handleSettings() override;
	void handleTools() override;
	void handleDebugMode() override;
	void handleRender() override;
	void handleRenderOverlay(double* proj, double* model, int* view) override;
	void handleMeshChanged() override;
	bool handleBuild() override;
	void handleUpdate(const float dt) override;

	void getTilePos(const float* pos, int& tx, int& ty);

	void renderCachedTile(const int tx, const int ty, const int type);
	void renderCachedTileOverlay(const int tx, const int ty, double* proj, double* model, int* view);

	void addTempObstacle(const AddObstacleData& add_data);
	void removeTempObstacle(const float* sp, const float* sq);
	void clearAllTempObstacles();
	void MoveTempObstacle(const dtObstacleRef ref, const float* move_pos);
	void CalcBoxPos(const float* middle_pos, const float* box_size, dtObstacleBox* box) const noexcept
	{
		m_tileCache->CalcBoxPos(middle_pos, box_size, box);
	}
	void CalcBoxPos(const float* middle_pos, dtObstacleOrientedBox* box)
	{
		m_tileCache->CalcBoxPos(middle_pos, box);
	}
	dtObstacleRef HitTestObstacle(const float* sp, const float* sq) const;

	void StartMoveObstacles() noexcept;
	dtStatus EndMoveObstacles();

	int GetObstacleRequestCount() const noexcept;
	int GetObstacleUpdateCount() const noexcept;

	void buildTile(const float* pos);
	bool removeTile(const float* pos);
	bool buildAllTiles();
	void removeAllTiles();

	void saveAll(const char* path);
	void loadAll(const char* path);

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Sample_TempObstacles(const Sample_TempObstacles&) = delete;
	Sample_TempObstacles& operator=(const Sample_TempObstacles&) = delete;

	int rasterizeTileLayers(const int tx, const int ty, const rcConfig& cfg, struct TileCacheData* tiles, const int maxTiles);
	void CleanUp();
	void buildTileMeshLayer(
		const int tx, const int ty, const rcConfig& cfg, const struct dtTileCacheParams& tcparams);
	dtStatus buildTileMesh(const int tx, const int ty);
};

#endif // RECASTSAMPLETEMPOBSTACLE_H

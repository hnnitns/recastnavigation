#ifndef DETOURTILECACHE_H
#define DETOURTILECACHE_H

#include "DetourStatus.h"
#include <array>

typedef unsigned int dtObstacleRef;

typedef unsigned int dtCompressedTileRef;

constexpr int MaxObstacleNum{ 128 };

// Flags for addTile // addTileのフラグ
enum dtCompressedTileFlags
{
	// Navmesh owns the tile memory and should free it.
	// ナビメッシュはタイルメモリを所有しており、解放する必要があります。
	DT_COMPRESSEDTILE_FREE_DATA = 0x01,
};

struct dtCompressedTile
{
	unsigned int salt; // Counter describing modifications to the tile. // タイルの変更を説明するカウンター。
	struct dtTileCacheLayerHeader* header;
	unsigned char* compressed;
	int compressedSize;
	unsigned char* data;
	int dataSize;
	unsigned int flags;
	dtCompressedTile* next;
};

enum ObstacleState
{
	DT_OBSTACLE_EMPTY,
	DT_OBSTACLE_PROCESSING,
	DT_OBSTACLE_PROCESSED,
	DT_OBSTACLE_REMOVING,
};

enum ObstacleType
{
	DT_OBSTACLE_CYLINDER,
	DT_OBSTACLE_BOX,
};

struct dtObstacleCylinder
{
	float pos[3];
	float radius;
	float height;
};

struct dtObstacleBox
{
	float bmin[3];
	float bmax[3];
};

constexpr int DT_MAX_TOUCHED_TILES = 8;

struct dtTileCacheObstacle
{
	union
	{
		dtObstacleCylinder cylinder;
		dtObstacleBox box;
	};

	std::array<dtCompressedTileRef, DT_MAX_TOUCHED_TILES> touched;
	std::array<dtCompressedTileRef, DT_MAX_TOUCHED_TILES> pending;
	unsigned short salt;
	ObstacleType type;
	ObstacleState state;
	unsigned char ntouched;
	unsigned char npending;
	dtTileCacheObstacle* next;
};

struct dtTileCacheParams
{
	float orig[3];
	float cs, ch;
	int width, height;
	float walkableHeight;
	float walkableRadius;
	float walkableClimb;
	float maxSimplificationError;
	int maxTiles;
	int maxObstacles;
};

struct dtTileCacheMeshProcess
{
	virtual ~dtTileCacheMeshProcess() { }

	virtual void process(struct dtNavMeshCreateParams* params,
		unsigned char* polyAreas, unsigned short* polyFlags) = 0;
};

class dtTileCache
{
public:
	dtTileCache();
	~dtTileCache();

	struct dtTileCacheAlloc* getAlloc() { return m_talloc; }
	struct dtTileCacheCompressor* getCompressor() { return m_tcomp; }
	const dtTileCacheParams* getParams() const { return &m_params; }

	inline int getTileCount() const { return m_params.maxTiles; }
	inline const dtCompressedTile* getTile(const int i) const { return &m_tiles[i]; }

	inline int getObstacleCount() const { return m_params.maxObstacles; }
	inline const dtTileCacheObstacle* getObstacleAt(const int i) const { return &m_obstacles[i]; }
	inline const std::array<dtTileCacheObstacle, MaxObstacleNum>& getObstacle() const noexcept { return m_obstacles; }

	const dtTileCacheObstacle* getObstacleByRef(dtObstacleRef ref);

	dtObstacleRef getObstacleRef(const dtTileCacheObstacle* obmin) const;

	dtStatus init(const dtTileCacheParams* params,
		struct dtTileCacheAlloc* talloc,
		struct dtTileCacheCompressor* tcomp,
		struct dtTileCacheMeshProcess* tmproc);

	int getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const;

	dtCompressedTile* getTileAt(const int tx, const int ty, const int tlayer);
	dtCompressedTileRef getTileRef(const dtCompressedTile* tile) const;
	dtCompressedTileRef getTileRefAt(const int tx, const int ty) const;
	const dtCompressedTile* getTileByRef(dtCompressedTileRef ref) const;

	dtStatus addTile(unsigned char* data, const int dataSize, unsigned char flags, dtCompressedTileRef* result);

	dtStatus removeTile(dtCompressedTileRef ref, unsigned char** data, int* dataSize);

	dtStatus addCylinderObstacle(const float* pos, const float radius, const float height, dtObstacleRef* result);
	dtStatus addBoxObstacle(const float* bmin, const float* bmax, dtObstacleRef* result);

	dtStatus removeObstacle(const dtObstacleRef ref);

	dtStatus queryTiles(const float* bmin, const float* bmax,
		dtCompressedTileRef* results, int* resultCount, const int maxResults) const;

	// Updates the tile cache by rebuilding tiles touched by unfinished obstacle requests.
	// 未完成の障害物リクエストが接触したタイルを再構築して、タイルキャッシュを更新します。
	//  @param[in] dt : The time step size. Currently not used.
	// タイムステップサイズ。 現在使用されていません。
	//  @param[in] navmesh : The mesh to affect when rebuilding tiles.
	// タイルを再構築するときに影響するメッシュ。
	//  @param[out] upToDate	Whether : the tile cache is fully up to date with obstacle requests and tile rebuilds.
	// タイルキャッシュが障害要求とタイルの再構築によって完全に最新であるかどうか。
	// If the tile cache is up to date another (immediate) call to update will have no effect;
	// タイルキャッシュが最新の場合、更新の別の（即時）呼び出しは効果がありません。
	// otherwise another call will continue processing obstacle requests and tile rebuilds.
	// それ以外の場合は、別の呼び出しが障害要求の処理とタイルの再構築を続行します。
	dtStatus update(const float dt, class dtNavMesh* navmesh, bool* upToDate = 0);

	dtStatus buildNavMeshTilesAt(const int tx, const int ty, class dtNavMesh* navmesh);

	dtStatus buildNavMeshTile(const dtCompressedTileRef ref, class dtNavMesh* navmesh);

	void calcTightTileBounds(const struct dtTileCacheLayerHeader* header, float* bmin, float* bmax) const;

	void getObstacleBounds(const struct dtTileCacheObstacle* ob, float* bmin, float* bmax) const;

	// Encodes a tile id.
	// タイルIDをエンコードします。
	inline dtCompressedTileRef encodeTileId(unsigned int salt, unsigned int it) const
	{
		return ((dtCompressedTileRef)salt << m_tileBits) | (dtCompressedTileRef)it;
	}

	// Decodes a tile salt.
	// タイルソルトをデコードします。
	inline unsigned int decodeTileIdSalt(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef saltMask = ((dtCompressedTileRef)1 << m_saltBits) - 1;
		return (unsigned int)((ref >> m_tileBits)& saltMask);
	}

	// Decodes a tile id.
	// タイルIDをデコードします。
	inline unsigned int decodeTileIdTile(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef tileMask = ((dtCompressedTileRef)1 << m_tileBits) - 1;
		return (unsigned int)(ref & tileMask);
	}

	// Encodes an obstacle id.
	// 障害物IDをエンコードします。
	inline dtObstacleRef encodeObstacleId(unsigned int salt, unsigned int it) const
	{
		return ((dtObstacleRef)salt << 16) | (dtObstacleRef)it;
	}

	// Decodes an obstacle salt.
	// 障害物ソルトをデコードします。
	inline unsigned int decodeObstacleIdSalt(dtObstacleRef ref) const
	{
		const dtObstacleRef saltMask = ((dtObstacleRef)1 << 16) - 1;
		return (unsigned int)((ref >> 16)& saltMask);
	}

	// Decodes an obstacle id.
	// 障害物IDをデコードします。
	inline unsigned int decodeObstacleIdObstacle(dtObstacleRef ref) const
	{
		const dtObstacleRef tileMask = ((dtObstacleRef)1 << 16) - 1;
		return (unsigned int)(ref & tileMask);
	}

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtTileCache(const dtTileCache&) = delete;
	dtTileCache& operator=(const dtTileCache&) = delete;

	enum ObstacleRequestAction
	{
		REQUEST_ADD,
		REQUEST_REMOVE,
	};

	struct ObstacleRequest
	{
		int action;
		dtObstacleRef ref;
	};

	int m_tileLutSize; // Tile hash lookup size (must be pot). // タイルハッシュルックアップサイズ（ポットである必要があります）。
	int m_tileLutMask; // Tile hash lookup mask.               // タイルハッシュルックアップマスク。

	dtCompressedTile** m_posLookup;	  // Tile hash lookup.  // タイルハッシュルックアップ。
	dtCompressedTile* m_nextFreeTile; // Freelist of tiles. // タイルのフリーリスト。
	dtCompressedTile* m_tiles;		  // List of tiles.     // タイルのリスト。

	unsigned int m_saltBits; // Number of salt bits in the tile ID. // タイルIDのソルトビットの数。
	unsigned int m_tileBits; // Number of tile bits in the tile ID. // タイルIDのタイルビット数。

	dtTileCacheParams m_params;

	dtTileCacheAlloc* m_talloc;
	dtTileCacheCompressor* m_tcomp;
	dtTileCacheMeshProcess* m_tmproc;

	std::array<dtTileCacheObstacle, MaxObstacleNum> m_obstacles;
	dtTileCacheObstacle* m_nextFreeObstacle;

	static const int MAX_REQUESTS = 64;
	std::array<ObstacleRequest, MAX_REQUESTS> m_reqs;
	int m_nreqs;

	static const int MAX_UPDATE = 64;
	dtCompressedTileRef m_update[MAX_UPDATE];
	int m_nupdate;
};

dtTileCache* dtAllocTileCache();
void dtFreeTileCache(dtTileCache* tc);

#endif

#ifndef DETOURTILECACHE_H
#define DETOURTILECACHE_H

#include <array>
#include "DetourStatus.h"

typedef uint32_t dtObstacleRef;

typedef uint32_t dtCompressedTileRef;

// Flags for addTile // addTileのフラグ
enum dtCompressedTileFlags
{
	// Navmesh owns the tile memory and should free it.
	// ナビメッシュはタイルメモリを所有しており、解放する必要があります。
	DT_COMPRESSEDTILE_FREE_DATA = 0x01,
};

struct dtCompressedTile
{
	uint32_t salt; // Counter describing modifications to the tile. // タイルの変更を説明するカウンター。
	struct dtTileCacheLayerHeader* header;
	uint8_t* compressed;
	int compressedSize;
	uint8_t* data;
	int dataSize;
	uint32_t flags;
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
	std::array<float, 3> pos;
	float radius;
	float height;
};

struct dtObstacleBox
{
	std::array<float, 3> bmin, bmax;
};

constexpr int DT_MAX_TOUCHED_TILES = 8;

struct dtTileCacheObstacle
{
	union
	{
		dtObstacleCylinder cylinder;
		dtObstacleBox box;
	};

	dtCompressedTileRef touched[DT_MAX_TOUCHED_TILES];
	dtCompressedTileRef pending[DT_MAX_TOUCHED_TILES];
	uint16_t salt;
	uint8_t type;
	uint8_t state;
	uint8_t ntouched;
	uint8_t npending;
	dtTileCacheObstacle* next;
};

struct dtTileCacheParams
{
	std::array<float, 3> orig;
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
		uint8_t* polyAreas, uint16_t* polyFlags) = 0;
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
	inline const dtTileCacheObstacle* getObstacle(const int i) const { return &m_obstacles[i]; }

	const dtTileCacheObstacle* getObstacleByRef(dtObstacleRef ref);

	dtObstacleRef getObstacleRef(const dtTileCacheObstacle* obmin) const;

	dtStatus init(const dtTileCacheParams* params,
		struct dtTileCacheAlloc* talloc,
		struct dtTileCacheCompressor* tcomp,
		struct dtTileCacheMeshProcess* tmproc);

	int getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const;

	dtCompressedTile* getTileAt(const int tx, const int ty, const int tlayer);
	dtCompressedTileRef getTileRef(const dtCompressedTile* tile) const;
	const dtCompressedTile* getTileByRef(dtCompressedTileRef ref) const;

	dtStatus addTile(uint8_t* data, const int dataSize, uint8_t flags, dtCompressedTileRef* result);

	dtStatus removeTile(dtCompressedTileRef ref, uint8_t** data, int* dataSize);

	dtStatus addObstacle(const float* pos, const float radius, const float height, dtObstacleRef* result);
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
	inline dtCompressedTileRef encodeTileId(uint32_t salt, uint32_t it) const
	{
		return ((dtCompressedTileRef)salt << m_tileBits) | (dtCompressedTileRef)it;
	}

	// Decodes a tile salt.
	// タイルソルトをデコードします。
	inline uint32_t decodeTileIdSalt(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef saltMask = ((dtCompressedTileRef)1 << m_saltBits) - 1;
		return (uint32_t)((ref >> m_tileBits)& saltMask);
	}

	// Decodes a tile id.
	// タイルIDをデコードします。
	inline uint32_t decodeTileIdTile(dtCompressedTileRef ref) const
	{
		const dtCompressedTileRef tileMask = ((dtCompressedTileRef)1 << m_tileBits) - 1;
		return (uint32_t)(ref & tileMask);
	}

	// Encodes an obstacle id.
	// 障害物IDをエンコードします。
	inline dtObstacleRef encodeObstacleId(uint32_t salt, uint32_t it) const
	{
		return ((dtObstacleRef)salt << 16) | (dtObstacleRef)it;
	}

	// Decodes an obstacle salt.
	// 障害物ソルトをデコードします。
	inline uint32_t decodeObstacleIdSalt(dtObstacleRef ref) const
	{
		const dtObstacleRef saltMask = ((dtObstacleRef)1 << 16) - 1;
		return (uint32_t)((ref >> 16)& saltMask);
	}

	// Decodes an obstacle id.
	// 障害物IDをデコードします。
	inline uint32_t decodeObstacleIdObstacle(dtObstacleRef ref) const
	{
		const dtObstacleRef tileMask = ((dtObstacleRef)1 << 16) - 1;
		return (uint32_t)(ref & tileMask);
	}

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtTileCache(const dtTileCache&);
	dtTileCache& operator=(const dtTileCache&);

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

	uint32_t m_saltBits; // Number of salt bits in the tile ID. // タイルIDのソルトビットの数。
	uint32_t m_tileBits; // Number of tile bits in the tile ID. // タイルIDのタイルビット数。

	dtTileCacheParams m_params;

	dtTileCacheAlloc* m_talloc;
	dtTileCacheCompressor* m_tcomp;
	dtTileCacheMeshProcess* m_tmproc;

	dtTileCacheObstacle* m_obstacles;
	dtTileCacheObstacle* m_nextFreeObstacle;

	static const int MAX_REQUESTS = 64;
	ObstacleRequest m_reqs[MAX_REQUESTS];
	int m_nreqs;

	static const int MAX_UPDATE = 64;
	dtCompressedTileRef m_update[MAX_UPDATE];
	int m_nupdate;
};

dtTileCache* dtAllocTileCache();
void dtFreeTileCache(dtTileCache* tc);

#endif

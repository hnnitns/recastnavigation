#ifndef DETOURTILECACHE_H
#define DETOURTILECACHE_H

#include "DetourStatus.h"
#include "Common.h"
#include <array>
#include <unordered_set>

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

enum ObstacleState // 障害物の状態
{
	DT_OBSTACLE_EMPTY, // 空
	DT_OBSTACLE_PROCESSING, // 処理中
	DT_OBSTACLE_PROCESSED, // 処理完了
	DT_OBSTACLE_REMOVING, // 削除
	DT_OBSTACLE_MOVING, // 移動
};

constexpr int DT_MAX_TOUCHED_TILES = 8;

struct dtTileCacheObstacle
{
	union
	{
		dtObstacleCylinder cylinder;
		dtObstacleBox box;
		dtObstacleOrientedBox orientedBox;
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

	// Aabb obstacle.
	dtStatus addBoxObstacle(const float* bmin, const float* bmax, dtObstacleRef* result);

	// Box obstacle: can be rotated in Y.
	dtStatus addBoxObstacle(const float* center, const float* halfExtents, const float yRadians, dtObstacleRef* result);

	dtStatus removeObstacle(const dtObstacleRef ref);

	dtStatus MoveObstacle(const dtObstacleRef ref, const float* move_pos);

	void AdjPosCylinderObstacle(float* out_pos, const dtObstacleCylinder& cylinder) const;
	void AdjPosBoxObstacle(float* out_pos_min, float* out_pos_max, const dtObstacleBox& box) const;
	void AdjPosBoxObstacle(float* out_pos, const dtObstacleOrientedBox& box) const;
	void CalcBoxPos(const float* middle_pos, const float* box_size, dtObstacleBox* box);
	void CalcBoxPos(const float* middle_pos, dtObstacleOrientedBox* box);

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

	void StartMoveObstacles() noexcept;
	dtStatus EndMoveObstacles(class dtNavMesh* navmesh);

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
	void AdjPosCylinder(float* pos) const noexcept { pos[1] -= 0.5f; }
	void AdjPosBox(float* p_min, float* p_max) const noexcept { p_min[1] -= 0.25f; p_max[1] -= 0.25f; };
	void AdjPosBox(float* center) const noexcept { center[1] -= 0.25f; };

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtTileCache(const dtTileCache&) = delete;
	dtTileCache& operator=(const dtTileCache&) = delete;

	enum ObstacleRequestAction // 障害物の処理リクエストの種類
	{
		REQUEST_ADD, // 追加
		REQUEST_REMOVE, // 削除
		REQUEST_MOVE, // 移動
	};

	struct ObstacleRequest
	{
		ObstacleRequestAction action; // 処理リクエストの種類
		dtObstacleRef ref;            // 処理リクエストの障害物リストの添え値
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

	std::array<dtTileCacheObstacle, MaxObstacleNum> m_obstacles; // 障害物リスト
	dtTileCacheObstacle* m_nextFreeObstacle;

	static constexpr int MAX_REQUESTS = 64;
	std::array<ObstacleRequest, MAX_REQUESTS> m_reqs;
	int m_nreqs; // 障害物の処理リクエスト数

	static constexpr int MAX_UPDATE = 64;
	std::array<dtCompressedTileRef, MAX_UPDATE> m_update;
	int m_nupdate; // 障害物の更新リクエスト数

	// ナビメッシュ生成の為に障害物がどのタイルを通ったかを保存する
	std::unordered_set<dtCompressedTileRef> m_update_cache; /// 重複させないためにunordered_setを使用
	bool is_update_chace;
};

dtTileCache* dtAllocTileCache();
void dtFreeTileCache(dtTileCache* tc);

#endif

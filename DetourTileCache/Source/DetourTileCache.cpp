#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "AlgorithmHelper.h"

#include <cstring>
#include <new>

dtTileCache* dtAllocTileCache()
{
	void* mem = dtAlloc(sizeof(dtTileCache), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtTileCache;
}

void dtFreeTileCache(dtTileCache* tc)
{
	if (!tc) return;
	tc->~dtTileCache();
	dtFree(tc);
}

namespace
{
	namespace exec = std::execution;

	template<size_t Size>
	inline bool contains(const std::array<dtCompressedTileRef, Size>& ref, const int n, const dtCompressedTileRef v)
	{
		return (Any_Of_N(ref, n, [=](auto rf) { return rf == v; }, exec::par));
	}

	struct NavMeshTileBuildContext
	{
		inline NavMeshTileBuildContext(struct dtTileCacheAlloc* a) : layer(0), lcset(0), lmesh(0), alloc(a) {}
		inline ~NavMeshTileBuildContext() { purge(); }
		void purge()
		{
			dtFreeTileCacheLayer(alloc, layer);
			layer = nullptr;
			dtFreeTileCacheContourSet(alloc, lcset);
			lcset = nullptr;
			dtFreeTileCachePolyMesh(alloc, lmesh);
			lmesh = nullptr;
		}
		struct dtTileCacheLayer* layer;
		struct dtTileCacheContourSet* lcset;
		struct dtTileCachePolyMesh* lmesh;
		struct dtTileCacheAlloc* alloc;
	};

	inline int computeTileHash(int x, int y, const int mask)
	{
		constexpr unsigned int h1 = 0x8da6b343; // Large multiplicative constants; // 大きな乗法定数。
		constexpr unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes // ここで任意に選んだ素数

		unsigned int n = h1 * x + h2 * y;
		return (int)(n & mask);
	}
}

dtTileCache::dtTileCache() :
	m_tileLutSize(0), m_tileLutMask(0), m_posLookup(0), m_nextFreeTile(0), m_tiles(0), m_saltBits(0), m_tileBits(0),
	m_talloc(0), m_tcomp(0), m_tmproc(0), m_nextFreeObstacle(0), m_nreqs(0), m_nupdate(0), is_update_chace()
{
	memset(&m_params, 0, sizeof(m_params));
	m_reqs.fill({});
}

dtTileCache::~dtTileCache()
{
	for (int i = 0; i < m_params.maxTiles; ++i)
	{
		if (m_tiles[i].flags & DT_COMPRESSEDTILE_FREE_DATA)
		{
			dtFree(m_tiles[i].data);
			m_tiles[i].data = nullptr;
		}
	}
	dtFree(m_posLookup);
	m_posLookup = 0;
	dtFree(m_tiles);
	m_tiles = 0;
	m_nreqs = 0;
	m_nupdate = 0;
}

const dtCompressedTile* dtTileCache::getTileByRef(dtCompressedTileRef ref) const
{
	if (!ref)
		return 0;
	unsigned int tileIndex = decodeTileIdTile(ref);
	unsigned int tileSalt = decodeTileIdSalt(ref);
	if ((int)tileIndex >= m_params.maxTiles)
		return 0;
	const dtCompressedTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return 0;
	return tile;
}

dtStatus dtTileCache::init(const dtTileCacheParams* params,
	dtTileCacheAlloc* talloc,
	dtTileCacheCompressor* tcomp,
	dtTileCacheMeshProcess* tmproc)
{
	m_talloc = talloc;
	m_tcomp = tcomp;
	m_tmproc = tmproc;
	m_nreqs = 0;
	memcpy(&m_params, params, sizeof(m_params));

	m_obstacles.fill({});
	m_nextFreeObstacle = nullptr;

	for (int i = m_params.maxObstacles - 1; i >= 0; --i)
	{
		m_obstacles[i].salt = 1;
		m_obstacles[i].next = m_nextFreeObstacle;
		m_nextFreeObstacle = &m_obstacles[i];
	}

	// Init tiles // タイルの初期化
	m_tileLutSize = dtNextPow2(m_params.maxTiles / 4);

	if (!m_tileLutSize) m_tileLutSize = 1;

	m_tileLutMask = m_tileLutSize - 1;
	m_tiles = (dtCompressedTile*)dtAlloc(sizeof(dtCompressedTile) * m_params.maxTiles, DT_ALLOC_PERM);

	if (!m_tiles) return DT_FAILURE | DT_OUT_OF_MEMORY;

	m_posLookup = (dtCompressedTile**)dtAlloc(sizeof(dtCompressedTile*) * m_tileLutSize, DT_ALLOC_PERM);

	if (!m_posLookup) return DT_FAILURE | DT_OUT_OF_MEMORY;

	memset(m_tiles, 0, sizeof(dtCompressedTile) * m_params.maxTiles);
	memset(m_posLookup, 0, sizeof(dtCompressedTile*) * m_tileLutSize);
	m_nextFreeTile = nullptr;

	for (int i = m_params.maxTiles - 1; i >= 0; --i)
	{
		m_tiles[i].salt = 1;
		m_tiles[i].next = m_nextFreeTile;
		m_nextFreeTile = &m_tiles[i];
	}

	// Init ID generator values.
	// Init IDジェネレーターの値。
	m_tileBits = dtIlog2(dtNextPow2((unsigned int)m_params.maxTiles));

	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	// ソルトマスクは32ビットuintを使用して計算され、オーバーフローするため、31ソルトビットのみを許可します。
	m_saltBits = dtMin((unsigned int)31, 32 - m_tileBits);

	if (m_saltBits < 10) return DT_FAILURE | DT_INVALID_PARAM;

	return DT_SUCCESS;
}

int dtTileCache::getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const
{
	int n{};

	// Find tile based on hash.
	// ハッシュに基づいてタイルを見つけます。
	int h = computeTileHash(tx, ty, m_tileLutMask);
	dtCompressedTile* tile = m_posLookup[h];

	while (tile)
	{
		if (tile->header && tile->header->tx == tx && tile->header->ty == ty)
		{
			if (n < maxTiles)
				tiles[n++] = getTileRef(tile);
		}
		tile = tile->next;
	}

	return n;
}

dtCompressedTile* dtTileCache::getTileAt(const int tx, const int ty, const int tlayer)
{
	// Find tile based on hash.
	int h = computeTileHash(tx, ty, m_tileLutMask);
	dtCompressedTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->tx == tx &&
			tile->header->ty == ty &&
			tile->header->tlayer == tlayer)
		{
			return tile;
		}
		tile = tile->next;
	}
	return 0;
}

dtCompressedTileRef dtTileCache::getTileRef(const dtCompressedTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return (dtCompressedTileRef)encodeTileId(tile->salt, it);
}

dtCompressedTileRef dtTileCache::getTileRefAt(const int tx, const int ty) const
{
	// Find tile based on hash.
	int h = computeTileHash(tx, ty, m_tileLutMask);
	dtCompressedTile* tile = m_posLookup[h];

	return getTileRef(tile);
}

dtObstacleRef dtTileCache::getObstacleRef(const dtTileCacheObstacle* ob) const
{
	if (!ob) return 0;
	const unsigned int idx = (unsigned int)(ob - m_obstacles.data());
	return encodeObstacleId(ob->salt, idx);
}

const dtTileCacheObstacle* dtTileCache::getObstacleByRef(dtObstacleRef ref)
{
	if (!ref)
		return 0;
	unsigned int idx = decodeObstacleIdObstacle(ref);
	if ((int)idx >= m_params.maxObstacles)
		return 0;
	const dtTileCacheObstacle* ob = &m_obstacles[idx];
	unsigned int salt = decodeObstacleIdSalt(ref);
	if (ob->salt != salt)
		return 0;
	return ob;
}

dtStatus dtTileCache::addTile(unsigned char* data, const int dataSize, unsigned char flags, dtCompressedTileRef* result)
{
	// Make sure the data is in right format.
	dtTileCacheLayerHeader* header = (dtTileCacheLayerHeader*)data;
	if (header->magic != DT_TILECACHE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_TILECACHE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;

	// Make sure the location is free.
	if (getTileAt(header->tx, header->ty, header->tlayer))
		return DT_FAILURE;

	// Allocate a tile.
	dtCompressedTile* tile = 0;
	if (m_nextFreeTile)
	{
		tile = m_nextFreeTile;
		m_nextFreeTile = tile->next;
		tile->next = 0;
	}

	// Make sure we could allocate a tile.
	if (!tile)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	// Insert tile into the position lut.
	int h = computeTileHash(header->tx, header->ty, m_tileLutMask);
	tile->next = m_posLookup[h];
	m_posLookup[h] = tile;

	// Init tile.
	const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
	tile->header = (dtTileCacheLayerHeader*)data;
	tile->data = data;
	tile->dataSize = dataSize;
	tile->compressed = tile->data + headerSize;
	tile->compressedSize = tile->dataSize - headerSize;
	tile->flags = flags;

	if (result)
		*result = getTileRef(tile);

	return DT_SUCCESS;
}

dtStatus dtTileCache::removeTile(dtCompressedTileRef ref, unsigned char** data, int* dataSize)
{
	if (!ref)
		return DT_FAILURE | DT_INVALID_PARAM;
	unsigned int tileIndex = decodeTileIdTile(ref);
	unsigned int tileSalt = decodeTileIdSalt(ref);
	if ((int)tileIndex >= m_params.maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	dtCompressedTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return DT_FAILURE | DT_INVALID_PARAM;

	// Remove tile from hash lookup.
	const int h = computeTileHash(tile->header->tx, tile->header->ty, m_tileLutMask);
	dtCompressedTile* prev = 0;
	dtCompressedTile* cur = m_posLookup[h];
	while (cur)
	{
		if (cur == tile)
		{
			if (prev)
				prev->next = cur->next;
			else
				m_posLookup[h] = cur->next;
			break;
		}
		prev = cur;
		cur = cur->next;
	}

	// Reset tile.
	if (tile->flags & DT_COMPRESSEDTILE_FREE_DATA)
	{
		// Owns data
		dtFree(tile->data);
		tile->data = 0;
		tile->dataSize = 0;
		if (data) *data = 0;
		if (dataSize) *dataSize = 0;
	}
	else
	{
		if (data) *data = tile->data;
		if (dataSize) *dataSize = tile->dataSize;
	}

	tile->header = 0;
	tile->data = 0;
	tile->dataSize = 0;
	tile->compressed = 0;
	tile->compressedSize = 0;
	tile->flags = 0;

	// Update salt, salt should never be zero.
	tile->salt = (tile->salt + 1) & ((1 << m_saltBits) - 1);
	if (tile->salt == 0)
		tile->salt++;

	// Add to free list.
	tile->next = m_nextFreeTile;
	m_nextFreeTile = tile;

	return DT_SUCCESS;
}

dtStatus dtTileCache::addCylinderObstacle(const float* pos, const float radius, const float height, dtObstacleRef* result)
{
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;

	dtTileCacheObstacle* ob{};
	if (m_nextFreeObstacle)
	{
		ob = m_nextFreeObstacle;
		m_nextFreeObstacle = ob->next;
		ob->next = 0;
	}
	if (!ob)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	unsigned short salt = ob->salt;
	memset(ob, 0, sizeof(dtTileCacheObstacle));
	ob->salt = salt;
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_CYLINDER;
	dtVcopy(ob->cylinder.pos, pos);
	ob->cylinder.radius = radius;
	ob->cylinder.height = height;

	auto& req = m_reqs[m_nreqs++];

	req = {};
	req.action = REQUEST_ADD;
	req.ref = getObstacleRef(ob);

	if (result)
		*result = req.ref;

	return DT_SUCCESS;
}

dtStatus dtTileCache::addBoxObstacle(const float* bmin, const float* bmax, dtObstacleRef* result)
{
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;

	dtTileCacheObstacle* ob = 0;
	if (m_nextFreeObstacle)
	{
		ob = m_nextFreeObstacle;
		m_nextFreeObstacle = ob->next;
		ob->next = 0;
	}
	if (!ob)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	unsigned short salt = ob->salt;
	memset(ob, 0, sizeof(dtTileCacheObstacle));
	ob->salt = salt;
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_BOX;
	dtVcopy(ob->box.bmin, bmin);
	dtVcopy(ob->box.bmax, bmax);

	auto& req = m_reqs[m_nreqs++]; // 処理リクエストに追加

	req = {};
	req.action = REQUEST_ADD; // 処理リクエストの種類を設定
	req.ref = getObstacleRef(ob); // 処理リクエストの障害物リストの添え値を設定

	if (result)
		*result = req.ref;

	return DT_SUCCESS;
}

dtStatus dtTileCache::addBoxObstacle(const float* center, const float* halfExtents, const float yRadians, dtObstacleRef* result)
{
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;

	dtTileCacheObstacle* ob = 0;
	if (m_nextFreeObstacle)
	{
		ob = m_nextFreeObstacle;
		m_nextFreeObstacle = ob->next;
		ob->next = 0;
	}
	if (!ob)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	unsigned short salt = ob->salt;
	memset(ob, 0, sizeof(dtTileCacheObstacle));
	ob->salt = salt;
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_ORIENTED_BOX;
	dtVcopy(ob->orientedBox.center, center);
	dtVcopy(ob->orientedBox.halfExtents, halfExtents);

	float coshalf= cosf(0.5f*yRadians);
	float sinhalf = sinf(-0.5f*yRadians);
	ob->orientedBox.rotAux[0] = coshalf*sinhalf;
	ob->orientedBox.rotAux[1] = coshalf*coshalf - 0.5f;

	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_ADD;
	req->ref = getObstacleRef(ob);

	if (result)
		*result = req->ref;

	return DT_SUCCESS;
}

dtStatus dtTileCache::removeObstacle(const dtObstacleRef ref)
{
	if (!ref)
		return DT_SUCCESS;
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;

	auto& req = m_reqs[m_nreqs++]; // 処理リクエストに追加

	req = {};
	req.action = REQUEST_REMOVE; // 処理リクエストの種類を設定
	req.ref = ref; // 処理リクエストの障害物リストの添え値を設定

	return DT_SUCCESS;
}

dtStatus dtTileCache::MoveObstacle(const dtObstacleRef ref, const float* move_pos)
{
	if (!ref)
		return DT_SUCCESS;
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;

	unsigned int idx = decodeObstacleIdObstacle(ref);
	if ((int)idx >= m_params.maxObstacles)
		return DT_FAILURE;

	auto& ob = m_obstacles[idx];
	unsigned int salt = decodeObstacleIdSalt(ref);
	if (ob.salt != salt)
		return DT_FAILURE;

	auto& req = m_reqs[m_nreqs++]; // 処理リクエストに追加

	req.action = REQUEST_MOVE; // 処理リクエストの種類を設定
	req.ref = ref; // 処理リクエストの障害物リストの添え値を設定

	if (ob.type == DT_OBSTACLE_CYLINDER)
	{
		auto& data{ ob.cylinder };

		// 座標を計算
		dtVcopy(data.before_pos, data.pos);
		dtVcopy(data.pos, move_pos);
		AdjPosCylinder(data.pos);
	}
	else if(ob.type == DT_OBSTACLE_BOX)
	{
		auto& data{ ob.box };

		/// boxのサイズを計算
		float box_size[3];
		dtVsub(box_size, data.bmax, data.bmin);
		dtVabs(box_size);

		// 座標を計算
		dtVcopy(data.before_bmin, data.bmin);
		dtVcopy(data.before_bmax, data.bmax);
		CalcBoxPos(move_pos, box_size, &data);
		AdjPosBox(data.bmin, data.bmax);
	}
	else if(ob.type == DT_OBSTACLE_ORIENTED_BOX)
	{
		auto& data{ ob.orientedBox };
		float p[3];

		// 座標を計算
		dtVcopy(data.center, move_pos);
		CalcBoxPos(move_pos, &data);
		AdjPosBox(data.center);
	}

	return DT_SUCCESS;
}

void dtTileCache::AdjPosCylinderObstacle(float* out_pos, const dtObstacleCylinder& cylinder) const
{
	dtVcopy(out_pos, cylinder.pos);
	AdjPosCylinder(out_pos);
}

void dtTileCache::AdjPosBoxObstacle(float* out_pos_min, float* out_pos_max, const dtObstacleBox& box) const
{
	dtVcopy(out_pos_min, box.bmin);
	dtVcopy(out_pos_max, box.bmax);
	AdjPosBox(out_pos_min, out_pos_max);
}

void dtTileCache::AdjPosBoxObstacle(float* out_pos, const dtObstacleOrientedBox& box) const
{
	dtVcopy(out_pos, box.center);
	AdjPosBox(out_pos);
}

void dtTileCache::CalcBoxPos(const float* middle_pos, const float* box_size, dtObstacleBox* box)
{
	float half_size[3]{};

	// サイズの半分を求める
	dtVscale(half_size, box_size, 0.5f);

	// 追加座標が中心に来るようにする
	dtVsub(box->bmin, middle_pos, half_size);
	dtVadd(box->bmax, middle_pos, half_size);

	// 下がってしまうので戻す
	box->bmin[1] += half_size[1];
	box->bmax[1] += half_size[1];
}

void dtTileCache::CalcBoxPos(const float* middle_pos, dtObstacleOrientedBox* box)
{
	dtVcopy(box->center, middle_pos);

	// 追加座標が中心に来るようにする
	box->center[1] = middle_pos[1] + box->halfExtents[1];
}

dtStatus dtTileCache::queryTiles(const float* bmin, const float* bmax,
	dtCompressedTileRef* results, int* resultCount, const int maxResults) const
{
	constexpr int MAX_TILES = 32;
	std::array<dtCompressedTileRef, MAX_TILES> tiles;

	int n{};

	const float tw = m_params.width * m_params.cs;
	const float th = m_params.height * m_params.cs;
	const int tx0 = (int)dtMathFloorf((bmin[0] - m_params.orig[0]) / tw);
	const int tx1 = (int)dtMathFloorf((bmax[0] - m_params.orig[0]) / tw);
	const int ty0 = (int)dtMathFloorf((bmin[2] - m_params.orig[2]) / th);
	const int ty1 = (int)dtMathFloorf((bmax[2] - m_params.orig[2]) / th);

	for (int ty = ty0; ty <= ty1; ++ty)
	{
		for (int tx = tx0; tx <= tx1; ++tx)
		{
			const int ntiles = getTilesAt(tx, ty, tiles.data(), MAX_TILES);

			for (int i = 0; i < ntiles; ++i)
			{
				const dtCompressedTile* tile = &m_tiles[decodeTileIdTile(tiles[i])];
				float tbmin[3], tbmax[3];
				calcTightTileBounds(tile->header, tbmin, tbmax);

				if (dtOverlapBounds(bmin, bmax, tbmin, tbmax))
				{
					if (n < maxResults)
						results[n++] = tiles[i];
				}
			}
		}
	}

	*resultCount = n;

	return DT_SUCCESS;
}

dtStatus dtTileCache::update(const float /*dt*/, class dtNavMesh* navmesh, bool* upToDate)
{
	if (m_nupdate == 0)
	{
		// Process requests.
		// リクエストを処理します。
		for (int i = 0; i < m_nreqs; ++i)
		{
			auto& req = m_reqs[i];

			unsigned int idx = decodeObstacleIdObstacle(req.ref);
			if ((int)idx >= m_params.maxObstacles)
				continue;
			auto& ob = m_obstacles[idx];
			unsigned int salt = decodeObstacleIdSalt(req.ref);
			if (ob.salt != salt)
				continue;

			// 障害物の追加
			switch (req.action)
			{
				case dtTileCache::REQUEST_ADD:
				{
					// Find touched tiles.
					// タッチされたタイルを見つけます。
					float bmin[3], bmax[3];
					getObstacleBounds(&ob, bmin, bmax);

					int ntouched = 0;
					queryTiles(bmin, bmax, ob.touched.data(), &ntouched, DT_MAX_TOUCHED_TILES);
					ob.ntouched = (unsigned char)ntouched;
					// Add tiles to update list.
					//タイルを追加してリストを更新します。
					ob.npending = 0;
					for (int j = 0; j < ob.ntouched; ++j)
					{
						if (m_nupdate < MAX_UPDATE)
						{
							if (!contains(m_update, m_nupdate, ob.touched[j]))
								m_update[m_nupdate++] = ob.touched[j]; //

							ob.pending[ob.npending++] = ob.touched[j];
						}
					}

					break;
				}
				case dtTileCache::REQUEST_REMOVE:
				{
					// Prepare to remove obstacle.
					// 障害物を取り除く準備をします。
					ob.state = DT_OBSTACLE_REMOVING;
					// Add tiles to update list.
					// 更新リストにタイルを追加します。
					ob.npending = 0;

					for (int j = 0; j < ob.ntouched; ++j)
					{
						if (m_nupdate < MAX_UPDATE)
						{
							if (!contains(m_update, m_nupdate, ob.touched[j]))
								m_update[m_nupdate++] = ob.touched[j];

							ob.pending[ob.npending++] = ob.touched[j];
						}
					}

					break;
				}
				case dtTileCache::REQUEST_MOVE:
				{
					/// 大きめのサイズで計算する（でないと、障害物の移動時（タイルを越した時）に取り残されてしまう。）
					float adj_bounds_dis_rate{};
					float adj_bounds_distance{};

					if (ob.type == DT_OBSTACLE_CYLINDER)
					{
						const auto& data{ ob.cylinder };
						const float size{ data.radius };
						const float dis{ dtVdist(data.before_pos, data.pos) }; // 距離を計算
						float adj_rate{ (std::max)(dis / size, 1.f) }; // 比率を求め

						adj_bounds_distance = size * adj_rate * 4.f; // どれだけ微調整するかを計算
					}
					else
					{
						const auto& data{ ob.box };
						std::array<float, 3> box_size;

						// サイズを求める
						dtVsub(box_size.data(), data.bmax, data.bmin);
						dtVabs(box_size.data());

						const auto& max_size{ NormalAlgorithm::Max_Element(box_size) }; // 一番大きいサイズを基準にする
						const float size{ *max_size };
						const float dis{ dtVdist(data.before_bmin, data.bmin) }; // 距離を計算
						const float adj_rate{ (std::max)(dis / size, 1.f) }; // 比率を求め

						adj_bounds_distance = size * adj_bounds_dis_rate * 4.f; // どれだけ微調整するかを計算
					}

					float bmin[3], bmax[3];
					getObstacleBounds(&ob, bmin, bmax);

					// サイズを調整
					dtVsubNum(bmin, bmin, adj_bounds_distance);
					dtVaddNum(bmax, bmax, adj_bounds_distance);

					// ナビメッシュの参照を取得する
					int ntouched = 0;
					queryTiles(bmin, bmax, ob.touched.data(), &ntouched, DT_MAX_TOUCHED_TILES);
					ob.ntouched = (unsigned char)ntouched;

					// Prepare to remove obstacle.
					// 障害物を移動させる準備をします。
					ob.state = DT_OBSTACLE_MOVING;
					// Add tiles to update list.
					// 更新リストにタイルを追加します。
					ob.npending = 0;
					for (int j = 0; j < ob.ntouched; ++j)
					{
						if (m_nupdate < MAX_UPDATE)
						{
							if (!contains(m_update, m_nupdate, ob.touched[j]))
							{
								m_update[m_nupdate++] = ob.touched[j]; // 更新リクエストに追加
								//m_update_cache.emplace(ob.touched[i]); // 更新リクエスト履歴に追加
							}

							ob.pending[ob.npending++] = ob.touched[j];
						}
					}

					break;
				}
			}
		}

		m_nreqs = 0;
	}

	dtStatus status = DT_SUCCESS;
	// Process updates
	// 処理しなければならない障害物がある
	if (m_nupdate != 0)
	{
		// Build mesh
		const dtCompressedTileRef ref = m_update.front();
		status = buildNavMeshTile(ref, navmesh);

		{
			constexpr size_t size{ sizeof(dtCompressedTileRef) };

			// 更新リクエスト数をデクリメント、使用済みの更新リクエストに次以降の更新リクエストを移動
			if (--m_nupdate > 0)
				memmove(m_update.data(), m_update.data() + 1, m_nupdate * size);
		}

		// Update obstacle states.
		// 障害物の状態を更新します。
		for (auto& ob : m_obstacles)
		{
			if (ob.state == DT_OBSTACLE_PROCESSING || ob.state == DT_OBSTACLE_REMOVING || ob.state == DT_OBSTACLE_MOVING)
			{
				// Remove handled tile from pending list.
				// 処理中のタイルを保留リストから削除します。
				for (int j = 0; j < (int)ob.npending; j++)
				{
					if (ob.pending[j] == ref)
					{
						ob.pending[j] = ob.pending[(int)ob.npending - 1];
						ob.npending--;
						break;
					}
				}

				// If all pending tiles processed, change state.
				// 保留中のすべてのタイルが処理された場合は、状態を変更します。
				if (ob.npending == 0)
				{
					// 処理中なら処理完了にする
					if (ob.state == DT_OBSTACLE_PROCESSING || ob.state == DT_OBSTACLE_MOVING)
					{
						ob.state = DT_OBSTACLE_PROCESSED;
					}
					// 削除なら空に変更
					else if (ob.state == DT_OBSTACLE_REMOVING)
					{
						ob.state = DT_OBSTACLE_EMPTY;
						// Update salt, salt should never be zero.
						// ソルトを更新します。ソルトがゼロになることはありません。
						ob.salt = (ob.salt + 1) & ((1 << 16) - 1);
						if (ob.salt == 0)
							ob.salt++;
						// Return obstacle to free list.
						// 障害物をフリーリストに戻します。
						ob.next = m_nextFreeObstacle;
						m_nextFreeObstacle = &ob;
					}
				}
			}
		}
	}

	if (upToDate)
		*upToDate = m_nupdate == 0 && m_nreqs == 0;

	return status;
}

dtStatus dtTileCache::buildNavMeshTilesAt(const int tx, const int ty, dtNavMesh* navmesh)
{
	constexpr int MAX_TILES = 32;
	dtCompressedTileRef tiles[MAX_TILES]{};
	const int ntiles = getTilesAt(tx, ty, tiles, MAX_TILES);

	for (int i = 0; i < ntiles; ++i)
	{
		dtStatus status = buildNavMeshTile(tiles[i], navmesh);

		if (dtStatusFailed(status))
			return status;
	}

	return DT_SUCCESS;
}

dtStatus dtTileCache::buildNavMeshTile(const dtCompressedTileRef ref, dtNavMesh* navmesh)
{
	dtAssert(m_talloc);
	dtAssert(m_tcomp);

	unsigned int idx = decodeTileIdTile(ref);

	if (idx > (unsigned int)m_params.maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;

	const dtCompressedTile* tile = &m_tiles[idx];
	unsigned int salt = decodeTileIdSalt(ref);

	if (tile->salt != salt)
		return DT_FAILURE | DT_INVALID_PARAM;

	m_talloc->reset();

	NavMeshTileBuildContext bc(m_talloc);
	const int walkableClimbVx = (int)(m_params.walkableClimb / m_params.ch);
	dtStatus status;

	// Decompress tile layer data.
	// タイルレイヤーデータを解凍します。
	status = dtDecompressTileCacheLayer(m_talloc, m_tcomp, tile->data, tile->dataSize, &bc.layer);
	if (dtStatusFailed(status))
		return status;

	// Rasterize obstacles.
	// 障害物をラスタライズします。
	for (const auto& ob : m_obstacles)
	{
		if (ob.state == DT_OBSTACLE_EMPTY || ob.state == DT_OBSTACLE_REMOVING)
			continue;

		if (contains(ob.touched, ob.ntouched, ref))
		{
			if (ob.type == DT_OBSTACLE_CYLINDER)
			{
				dtMarkCylinderArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
					ob.cylinder.pos, ob.cylinder.radius, ob.cylinder.height, 0);
			}
			else if (ob.type == DT_OBSTACLE_BOX)
			{
				dtMarkBoxArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
					ob.box.bmin, ob.box.bmax, 0);
			}
			else if (ob.type == DT_OBSTACLE_ORIENTED_BOX)
			{
				dtMarkBoxArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
					ob.orientedBox.center, ob.orientedBox.halfExtents, ob.orientedBox.rotAux, 0);
			}
		}
	}

	// Build navmesh
	// ナビメッシュの生成
	status = dtBuildTileCacheRegions(m_talloc, *bc.layer, walkableClimbVx);
	if (dtStatusFailed(status))
		return status;

	bc.lcset = dtAllocTileCacheContourSet(m_talloc);
	if (!bc.lcset)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	status = dtBuildTileCacheContours(m_talloc, *bc.layer, walkableClimbVx,
		m_params.maxSimplificationError, *bc.lcset);
	if (dtStatusFailed(status))
		return status;

	bc.lmesh = dtAllocTileCachePolyMesh(m_talloc);
	if (!bc.lmesh)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	status = dtBuildTileCachePolyMesh(m_talloc, *bc.lcset, *bc.lmesh);
	if (dtStatusFailed(status))
		return status;

	// Early out if the mesh tile is empty.
	// メッシュタイルが空の場合は早めに。
	if (!bc.lmesh->npolys)
	{
		// Remove existing tile.
		navmesh->removeTile(navmesh->getTileRefAt(tile->header->tx, tile->header->ty, tile->header->tlayer), 0, 0);
		return DT_SUCCESS;
	}

	dtNavMeshCreateParams params{};

	params.verts = bc.lmesh->verts;
	params.vertCount = bc.lmesh->nverts;
	params.polys = bc.lmesh->polys;
	params.polyAreas = bc.lmesh->areas;
	params.polyFlags = bc.lmesh->flags;
	params.polyCount = bc.lmesh->npolys;
	params.nvp = DT_VERTS_PER_POLYGON;
	params.walkableHeight = m_params.walkableHeight;
	params.walkableRadius = m_params.walkableRadius;
	params.walkableClimb = m_params.walkableClimb;
	params.tileX = tile->header->tx;
	params.tileY = tile->header->ty;
	params.tileLayer = tile->header->tlayer;
	params.cs = m_params.cs;
	params.ch = m_params.ch;
	params.buildBvTree = false;
	dtVcopy(params.bmin, tile->header->bmin);
	dtVcopy(params.bmax, tile->header->bmax);

	if (m_tmproc)
	{
		m_tmproc->process(&params, bc.lmesh->areas, bc.lmesh->flags);
	}

	unsigned char* navData = 0;
	int navDataSize = 0;
	if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		return DT_FAILURE;

	// Remove existing tile.
	// 既存のタイルを削除します。
	navmesh->removeTile(navmesh->getTileRefAt(tile->header->tx, tile->header->ty, tile->header->tlayer), 0, 0);

	// Add new tile, or leave the location empty.
	// 新しいタイルを追加するか、場所を空のままにします。
	if (navData)
	{
		// Let the navmesh own the data.
		// navmeshにデータを所有させます。
		status = navmesh->addTile(navData, navDataSize, DT_TILE_FREE_DATA, 0, 0);
		if (dtStatusFailed(status))
		{
			dtFree(navData);
			return status;
		}
	}

	return DT_SUCCESS;
}

void dtTileCache::calcTightTileBounds(const dtTileCacheLayerHeader* header, float* bmin, float* bmax) const
{
	const float cs = m_params.cs;
	bmin[0] = header->bmin[0] + header->minx * cs;
	bmin[1] = header->bmin[1];
	bmin[2] = header->bmin[2] + header->miny * cs;
	bmax[0] = header->bmin[0] + (header->maxx + 1) * cs;
	bmax[1] = header->bmax[1];
	bmax[2] = header->bmin[2] + (header->maxy + 1) * cs;
}

void dtTileCache::getObstacleBounds(const struct dtTileCacheObstacle* ob, float* bmin, float* bmax) const
{
	if (ob->type == DT_OBSTACLE_CYLINDER)
	{
		const dtObstacleCylinder& cl = ob->cylinder;

		bmin[0] = cl.pos[0] - cl.radius;
		bmin[1] = cl.pos[1];
		bmin[2] = cl.pos[2] - cl.radius;
		bmax[0] = cl.pos[0] + cl.radius;
		bmax[1] = cl.pos[1] + cl.height;
		bmax[2] = cl.pos[2] + cl.radius;
	}
	else if (ob->type == DT_OBSTACLE_BOX)
	{
		dtVcopy(bmin, ob->box.bmin);
		dtVcopy(bmax, ob->box.bmax);
	}
	else if (ob->type == DT_OBSTACLE_ORIENTED_BOX)
	{
		const dtObstacleOrientedBox &orientedBox = ob->orientedBox;

		float maxr = 1.41f*dtMax(orientedBox.halfExtents[0], orientedBox.halfExtents[2]);
		bmin[0] = orientedBox.center[0] - maxr;
		bmax[0] = orientedBox.center[0] + maxr;
		bmin[1] = orientedBox.center[1] - orientedBox.halfExtents[1];
		bmax[1] = orientedBox.center[1] + orientedBox.halfExtents[1];
		bmin[2] = orientedBox.center[2] - maxr;
		bmax[2] = orientedBox.center[2] + maxr;
	}
}

void dtTileCache::StartMoveObstacles() noexcept
{
	// 初期化
	m_update_cache.reserve(10);
	is_update_chace = true;
}

dtStatus dtTileCache::EndMoveObstacles(dtNavMesh* navmesh)
{
	dtStatus status = DT_SUCCESS;

	// 障害物の移動動作に入っていない
	if (!is_update_chace)	return status;

	// 別のタイルに移動していないのでナビメッシュ生成を生成する必要はない
	if (m_update_cache.size() == 1u)	return status;

	// 以前のナビメッシュの削除

	// 障害物の一回でも通ったナビメッシュの再生成
	for (auto& cache : m_update_cache)
	{
		status = buildNavMeshTile(cache, navmesh);

		if (dtStatusFailed(status))	break;
	}

	// 終了処理
	m_update_cache.clear();
	is_update_chace = false;

	return status;
}

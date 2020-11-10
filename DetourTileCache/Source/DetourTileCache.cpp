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
		constexpr unsigned int h1 = 0x8da6b343; // Large multiplicative constants; // �傫�ȏ�@�萔�B
		constexpr unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes // �����ŔC�ӂɑI�񂾑f��

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

	// Init tiles // �^�C���̏�����
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
	// Init ID�W�F�l���[�^�[�̒l�B
	m_tileBits = dtIlog2(dtNextPow2((unsigned int)m_params.maxTiles));

	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	// �\���g�}�X�N��32�r�b�guint���g�p���Čv�Z����A�I�[�o�[�t���[���邽�߁A31�\���g�r�b�g�݂̂������܂��B
	m_saltBits = dtMin((unsigned int)31, 32 - m_tileBits);

	if (m_saltBits < 10) return DT_FAILURE | DT_INVALID_PARAM;

	return DT_SUCCESS;
}

int dtTileCache::getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const
{
	int n{};

	// Find tile based on hash.
	// �n�b�V���Ɋ�Â��ă^�C���������܂��B
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

	auto& req = m_reqs[m_nreqs++]; // �������N�G�X�g�ɒǉ�

	req = {};
	req.action = REQUEST_ADD; // �������N�G�X�g�̎�ނ�ݒ�
	req.ref = getObstacleRef(ob); // �������N�G�X�g�̏�Q�����X�g�̓Y���l��ݒ�

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

	auto& req = m_reqs[m_nreqs++]; // �������N�G�X�g�ɒǉ�

	req = {};
	req.action = REQUEST_REMOVE; // �������N�G�X�g�̎�ނ�ݒ�
	req.ref = ref; // �������N�G�X�g�̏�Q�����X�g�̓Y���l��ݒ�

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

	auto& req = m_reqs[m_nreqs++]; // �������N�G�X�g�ɒǉ�

	req.action = REQUEST_MOVE; // �������N�G�X�g�̎�ނ�ݒ�
	req.ref = ref; // �������N�G�X�g�̏�Q�����X�g�̓Y���l��ݒ�

	if (ob.type == DT_OBSTACLE_CYLINDER)
	{
		auto& data{ ob.cylinder };

		// ���W���v�Z
		dtVcopy(data.before_pos, data.pos);
		dtVcopy(data.pos, move_pos);
		AdjPosCylinder(data.pos);
	}
	else if(ob.type == DT_OBSTACLE_BOX)
	{
		auto& data{ ob.box };

		/// box�̃T�C�Y���v�Z
		float box_size[3];
		dtVsub(box_size, data.bmax, data.bmin);
		dtVabs(box_size);

		// ���W���v�Z
		dtVcopy(data.before_bmin, data.bmin);
		dtVcopy(data.before_bmax, data.bmax);
		CalcBoxPos(move_pos, box_size, &data);
		AdjPosBox(data.bmin, data.bmax);
	}
	else if(ob.type == DT_OBSTACLE_ORIENTED_BOX)
	{
		auto& data{ ob.orientedBox };
		float p[3];

		// ���W���v�Z
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

	// �T�C�Y�̔��������߂�
	dtVscale(half_size, box_size, 0.5f);

	// �ǉ����W�����S�ɗ���悤�ɂ���
	dtVsub(box->bmin, middle_pos, half_size);
	dtVadd(box->bmax, middle_pos, half_size);

	// �������Ă��܂��̂Ŗ߂�
	box->bmin[1] += half_size[1];
	box->bmax[1] += half_size[1];
}

void dtTileCache::CalcBoxPos(const float* middle_pos, dtObstacleOrientedBox* box)
{
	dtVcopy(box->center, middle_pos);

	// �ǉ����W�����S�ɗ���悤�ɂ���
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
		// ���N�G�X�g���������܂��B
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

			// ��Q���̒ǉ�
			switch (req.action)
			{
				case dtTileCache::REQUEST_ADD:
				{
					// Find touched tiles.
					// �^�b�`���ꂽ�^�C���������܂��B
					float bmin[3], bmax[3];
					getObstacleBounds(&ob, bmin, bmax);

					int ntouched = 0;
					queryTiles(bmin, bmax, ob.touched.data(), &ntouched, DT_MAX_TOUCHED_TILES);
					ob.ntouched = (unsigned char)ntouched;
					// Add tiles to update list.
					//�^�C����ǉ����ă��X�g���X�V���܂��B
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
					// ��Q������菜�����������܂��B
					ob.state = DT_OBSTACLE_REMOVING;
					// Add tiles to update list.
					// �X�V���X�g�Ƀ^�C����ǉ����܂��B
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
					/// �傫�߂̃T�C�Y�Ōv�Z����i�łȂ��ƁA��Q���̈ړ����i�^�C�����z�������j�Ɏ��c����Ă��܂��B�j
					float adj_bounds_dis_rate{};
					float adj_bounds_distance{};

					if (ob.type == DT_OBSTACLE_CYLINDER)
					{
						const auto& data{ ob.cylinder };
						const float size{ data.radius };
						const float dis{ dtVdist(data.before_pos, data.pos) }; // �������v�Z
						float adj_rate{ (std::max)(dis / size, 1.f) }; // �䗦������

						adj_bounds_distance = size * adj_rate * 4.f; // �ǂꂾ�����������邩���v�Z
					}
					else
					{
						const auto& data{ ob.box };
						std::array<float, 3> box_size;

						// �T�C�Y�����߂�
						dtVsub(box_size.data(), data.bmax, data.bmin);
						dtVabs(box_size.data());

						const auto& max_size{ NormalAlgorithm::Max_Element(box_size) }; // ��ԑ傫���T�C�Y����ɂ���
						const float size{ *max_size };
						const float dis{ dtVdist(data.before_bmin, data.bmin) }; // �������v�Z
						const float adj_rate{ (std::max)(dis / size, 1.f) }; // �䗦������

						adj_bounds_distance = size * adj_bounds_dis_rate * 4.f; // �ǂꂾ�����������邩���v�Z
					}

					float bmin[3], bmax[3];
					getObstacleBounds(&ob, bmin, bmax);

					// �T�C�Y�𒲐�
					dtVsubNum(bmin, bmin, adj_bounds_distance);
					dtVaddNum(bmax, bmax, adj_bounds_distance);

					// �i�r���b�V���̎Q�Ƃ��擾����
					int ntouched = 0;
					queryTiles(bmin, bmax, ob.touched.data(), &ntouched, DT_MAX_TOUCHED_TILES);
					ob.ntouched = (unsigned char)ntouched;

					// Prepare to remove obstacle.
					// ��Q�����ړ������鏀�������܂��B
					ob.state = DT_OBSTACLE_MOVING;
					// Add tiles to update list.
					// �X�V���X�g�Ƀ^�C����ǉ����܂��B
					ob.npending = 0;
					for (int j = 0; j < ob.ntouched; ++j)
					{
						if (m_nupdate < MAX_UPDATE)
						{
							if (!contains(m_update, m_nupdate, ob.touched[j]))
							{
								m_update[m_nupdate++] = ob.touched[j]; // �X�V���N�G�X�g�ɒǉ�
								//m_update_cache.emplace(ob.touched[i]); // �X�V���N�G�X�g�����ɒǉ�
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
	// �������Ȃ���΂Ȃ�Ȃ���Q��������
	if (m_nupdate != 0)
	{
		// Build mesh
		const dtCompressedTileRef ref = m_update.front();
		status = buildNavMeshTile(ref, navmesh);

		{
			constexpr size_t size{ sizeof(dtCompressedTileRef) };

			// �X�V���N�G�X�g�����f�N�������g�A�g�p�ς݂̍X�V���N�G�X�g�Ɏ��ȍ~�̍X�V���N�G�X�g���ړ�
			if (--m_nupdate > 0)
				memmove(m_update.data(), m_update.data() + 1, m_nupdate * size);
		}

		// Update obstacle states.
		// ��Q���̏�Ԃ��X�V���܂��B
		for (auto& ob : m_obstacles)
		{
			if (ob.state == DT_OBSTACLE_PROCESSING || ob.state == DT_OBSTACLE_REMOVING || ob.state == DT_OBSTACLE_MOVING)
			{
				// Remove handled tile from pending list.
				// �������̃^�C����ۗ����X�g����폜���܂��B
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
				// �ۗ����̂��ׂẴ^�C�����������ꂽ�ꍇ�́A��Ԃ�ύX���܂��B
				if (ob.npending == 0)
				{
					// �������Ȃ珈�������ɂ���
					if (ob.state == DT_OBSTACLE_PROCESSING || ob.state == DT_OBSTACLE_MOVING)
					{
						ob.state = DT_OBSTACLE_PROCESSED;
					}
					// �폜�Ȃ��ɕύX
					else if (ob.state == DT_OBSTACLE_REMOVING)
					{
						ob.state = DT_OBSTACLE_EMPTY;
						// Update salt, salt should never be zero.
						// �\���g���X�V���܂��B�\���g���[���ɂȂ邱�Ƃ͂���܂���B
						ob.salt = (ob.salt + 1) & ((1 << 16) - 1);
						if (ob.salt == 0)
							ob.salt++;
						// Return obstacle to free list.
						// ��Q�����t���[���X�g�ɖ߂��܂��B
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
	// �^�C�����C���[�f�[�^���𓀂��܂��B
	status = dtDecompressTileCacheLayer(m_talloc, m_tcomp, tile->data, tile->dataSize, &bc.layer);
	if (dtStatusFailed(status))
		return status;

	// Rasterize obstacles.
	// ��Q�������X�^���C�Y���܂��B
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
	// �i�r���b�V���̐���
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
	// ���b�V���^�C������̏ꍇ�͑��߂ɁB
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
	// �����̃^�C�����폜���܂��B
	navmesh->removeTile(navmesh->getTileRefAt(tile->header->tx, tile->header->ty, tile->header->tlayer), 0, 0);

	// Add new tile, or leave the location empty.
	// �V�����^�C����ǉ����邩�A�ꏊ����̂܂܂ɂ��܂��B
	if (navData)
	{
		// Let the navmesh own the data.
		// navmesh�Ƀf�[�^�����L�����܂��B
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
	// ������
	m_update_cache.reserve(10);
	is_update_chace = true;
}

dtStatus dtTileCache::EndMoveObstacles(dtNavMesh* navmesh)
{
	dtStatus status = DT_SUCCESS;

	// ��Q���̈ړ�����ɓ����Ă��Ȃ�
	if (!is_update_chace)	return status;

	// �ʂ̃^�C���Ɉړ����Ă��Ȃ��̂Ńi�r���b�V�������𐶐�����K�v�͂Ȃ�
	if (m_update_cache.size() == 1u)	return status;

	// �ȑO�̃i�r���b�V���̍폜

	// ��Q���̈��ł��ʂ����i�r���b�V���̍Đ���
	for (auto& cache : m_update_cache)
	{
		status = buildNavMeshTile(cache, navmesh);

		if (dtStatusFailed(status))	break;
	}

	// �I������
	m_update_cache.clear();
	is_update_chace = false;

	return status;
}

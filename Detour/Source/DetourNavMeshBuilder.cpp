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

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>  // std::numeric_limitsのため
#include <cstdint> // uint16_tのため
#include <array>
#include <vector>
#include "DetourNavMesh.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourNavMeshBuilder.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"

namespace
{
	constexpr uint16_t MESH_NULL_IDX = (std::numeric_limits<uint16_t>::max)();

	struct BVItem
	{
		std::array<uint16_t, 3> bmin, bmax;
		int i;
	};

	inline int compareItemX(const void* va, const void* vb)
	{
		const BVItem* a = (const BVItem*)va;
		const BVItem* b = (const BVItem*)vb;

		return (a->bmin[0] > b->bmin[0]) - (a->bmin[0] < b->bmin[0]);
	}

	inline int compareItemY(const void* va, const void* vb)
	{
		const BVItem* a = (const BVItem*)va;
		const BVItem* b = (const BVItem*)vb;

		return (a->bmin[1] > b->bmin[1]) - (a->bmin[1] < b->bmin[1]);
	}

	inline int compareItemZ(const void* va, const void* vb)
	{
		const BVItem* a = (const BVItem*)va;
		const BVItem* b = (const BVItem*)vb;

		return (a->bmin[2] > b->bmin[2]) - (a->bmin[2] < b->bmin[2]);
	}

	void calcExtends(const std::vector<BVItem>& items, const int /*nitems*/, const int imin, const int imax,
		uint16_t* bmin, uint16_t* bmax)
	{
		bmin[0] = items[imin].bmin[0];
		bmin[1] = items[imin].bmin[1];
		bmin[2] = items[imin].bmin[2];

		bmax[0] = items[imin].bmax[0];
		bmax[1] = items[imin].bmax[1];
		bmax[2] = items[imin].bmax[2];

		for (int i = imin + 1; i < imax; ++i)
		{
			const BVItem& it = items[i];
			if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
			if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];
			if (it.bmin[2] < bmin[2]) bmin[2] = it.bmin[2];

			if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
			if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
			if (it.bmax[2] > bmax[2]) bmax[2] = it.bmax[2];
		}
	}

	inline int longestAxis(uint16_t x, uint16_t y, uint16_t z)
	{
		int	axis = 0;
		uint16_t maxVal = x;
		if (y > maxVal)
		{
			axis = 1;
			maxVal = y;
		}
		if (z > maxVal)
		{
			axis = 2;
		}
		return axis;
	}

	void subdivide(std::vector<BVItem>& items, int nitems, int imin, int imax, int& curNode, dtBVNode* nodes)
	{
		int inum = imax - imin;
		int icur = curNode;

		dtBVNode& node = nodes[curNode++];

		if (inum == 1)
		{
			// Leaf
			node.bmin[0] = items[imin].bmin[0];
			node.bmin[1] = items[imin].bmin[1];
			node.bmin[2] = items[imin].bmin[2];

			node.bmax[0] = items[imin].bmax[0];
			node.bmax[1] = items[imin].bmax[1];
			node.bmax[2] = items[imin].bmax[2];

			node.i = items[imin].i;
		}
		else
		{
			// Split
			calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);

			int	axis = longestAxis(node.bmax[0] - node.bmin[0],
				node.bmax[1] - node.bmin[1],
				node.bmax[2] - node.bmin[2]);

			if (axis == 0)
			{
				// Sort along x-axis
				qsort(items.data() + imin, inum, sizeof(BVItem), compareItemX);
			}
			else if (axis == 1)
			{
				// Sort along y-axis
				qsort(items.data() + imin, inum, sizeof(BVItem), compareItemY);
			}
			else
			{
				// Sort along z-axis
				qsort(items.data() + imin, inum, sizeof(BVItem), compareItemZ);
			}

			int isplit = imin + inum / 2;

			// Left
			subdivide(items, nitems, imin, isplit, curNode, nodes);
			// Right
			subdivide(items, nitems, isplit, imax, curNode, nodes);

			int iescape = curNode - icur;
			// Negative index means escape.
			node.i = -iescape;
		}
	}

	int createBVTree(dtNavMeshCreateParams* params, dtBVNode* nodes, int /*nnodes*/)
	{
		// Build tree
		float quantFactor = 1 / params->cs;

		std::vector<BVItem> items(params->polyCount);

		for (int i = 0; i < params->polyCount; i++)
		{
			BVItem& it = items[i];
			it.i = i;
			// Calc polygon bounds. Use detail meshes if available.
			// ポリゴンの境界を計算します。 可能であれば詳細メッシュを使用します。
			if (params->detailMeshes)
			{
				int vb = (int)params->detailMeshes[i * 4 + 0];
				int ndv = (int)params->detailMeshes[i * 4 + 1];
				float bmin[3];
				float bmax[3];

				const float* dv = &params->detailVerts[vb * 3];
				dtVcopy(bmin, dv);
				dtVcopy(bmax, dv);

				for (int j = 1; j < ndv; j++)
				{
					dtVmin(bmin, &dv[j * 3]);
					dtVmax(bmax, &dv[j * 3]);
				}

				// BV-tree uses cs for all dimensions
				it.bmin[0] = (uint16_t)dtClamp((int)((bmin[0] - params->bmin[0]) * quantFactor), 0, 0xffff);
				it.bmin[1] = (uint16_t)dtClamp((int)((bmin[1] - params->bmin[1]) * quantFactor), 0, 0xffff);
				it.bmin[2] = (uint16_t)dtClamp((int)((bmin[2] - params->bmin[2]) * quantFactor), 0, 0xffff);

				it.bmax[0] = (uint16_t)dtClamp((int)((bmax[0] - params->bmin[0]) * quantFactor), 0, 0xffff);
				it.bmax[1] = (uint16_t)dtClamp((int)((bmax[1] - params->bmin[1]) * quantFactor), 0, 0xffff);
				it.bmax[2] = (uint16_t)dtClamp((int)((bmax[2] - params->bmin[2]) * quantFactor), 0, 0xffff);
			}
			else
			{
				const uint16_t* p = &params->polys[i * params->nvp * 2];
				it.bmin[0] = it.bmax[0] = params->verts[p[0] * 3 + 0];
				it.bmin[1] = it.bmax[1] = params->verts[p[0] * 3 + 1];
				it.bmin[2] = it.bmax[2] = params->verts[p[0] * 3 + 2];

				for (int j = 1; j < params->nvp; ++j)
				{
					if (p[j] == MESH_NULL_IDX) break;
					uint16_t x = params->verts[p[j] * 3 + 0];
					uint16_t y = params->verts[p[j] * 3 + 1];
					uint16_t z = params->verts[p[j] * 3 + 2];

					if (x < it.bmin[0]) it.bmin[0] = x;
					if (y < it.bmin[1]) it.bmin[1] = y;
					if (z < it.bmin[2]) it.bmin[2] = z;

					if (x > it.bmax[0]) it.bmax[0] = x;
					if (y > it.bmax[1]) it.bmax[1] = y;
					if (z > it.bmax[2]) it.bmax[2] = z;
				}
				// Remap y
				it.bmin[1] = (uint16_t)dtMathFloorf((float)it.bmin[1] * params->ch / params->cs);
				it.bmax[1] = (uint16_t)dtMathCeilf((float)it.bmax[1] * params->ch / params->cs);
			}
		}

		int curNode = 0;
		subdivide(items, params->polyCount, 0, params->polyCount, curNode, nodes);

		return curNode;
	}

	unsigned char classifyOffMeshPoint(const float* pt, const float* bmin, const float* bmax)
	{
		static const unsigned char XP = 1 << 0;
		static const unsigned char ZP = 1 << 1;
		static const unsigned char XM = 1 << 2;
		static const unsigned char ZM = 1 << 3;

		unsigned char outcode = 0;
		outcode |= (pt[0] >= bmax[0]) ? XP : 0;
		outcode |= (pt[2] >= bmax[2]) ? ZP : 0;
		outcode |= (pt[0] < bmin[0]) ? XM : 0;
		outcode |= (pt[2] < bmin[2]) ? ZM : 0;

		switch (outcode)
		{
			case XP: return 0;
			case XP | ZP: return 1;
			case ZP: return 2;
			case XM | ZP: return 3;
			case XM: return 4;
			case XM | ZM: return 5;
			case ZM: return 6;
			case XP | ZM: return 7;
		};

		return 0xff;
	}
}

// TODO: Better error handling. // エラー処理の改善。

// @par
//
// The output data array is allocated using the detour allocator (dtAlloc()).  The method
// used to free the memory will be determined by how the tile is added to the navigation
// mesh.
// 出力データ配列は、迂回アロケーター（dtAlloc（））を使用して割り当てられます。
// メモリを解放するために使用されるメソッドは、タイルがナビゲーションメッシュに追加される方法によって決まります。
//
// @see dtNavMesh, dtNavMesh::addTile()
bool dtCreateNavMeshData(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize)
{
	//ポリゴンごとの頂点の最大値を超えている
	if (params->nvp > DT_VERTS_PER_POLYGON) return false;

	// ポリゴンメッシュの最大値を超えている（「0xffff」は気持ちが悪いのでこうした。uint16_t は自分がunsigned系使う場合に使っている）
	if (params->vertCount >= (std::numeric_limits<uint16_t>::max)()) return false;

	// ポリゴンメッシュが存在しない or ポリゴンメッシュの頂点が０
	if (!params->vertCount || !params->verts) return false;

	// ポリゴンが存在しない or ポリゴン数が０
	if (!params->polyCount || !params->polys) return false;

	const int nvp = params->nvp;

	// Classify off-mesh connection points.
	// オフメッシュ接続ポイントを分類します。
	// We store only the connections whose start point is inside the tile.
	// 開始点がタイル内にある接続のみを保存します。
	unsigned char* offMeshConClass{};
	int storedOffMeshConCount{};
	int offMeshConLinkCount{};

	if (params->offMeshConCount > 0)
	{
		offMeshConClass = (unsigned char*)dtAlloc(sizeof(unsigned char) * params->offMeshConCount * 2, DT_ALLOC_TEMP);

		// ポインタが死んでいる
		if (!offMeshConClass) return false;

		// Find tight heigh bounds, used for culling out off-mesh start locations.
		// メッシュ外の開始位置を選別するために使用される、タイトな高さの境界を見つけます。
		float hmin = (std::numeric_limits<float>::max)();
		float hmax = -(std::numeric_limits<float>::max)();

		if (params->detailVerts && params->detailVertsCount)
		{
			for (int i = 0; i < params->detailVertsCount; ++i)
			{
				const float h = params->detailVerts[i * 3 + 1];
				hmin = dtMin(hmin, h);
				hmax = dtMax(hmax, h);
			}
		}
		else
		{
			for (int i = 0; i < params->vertCount; ++i)
			{
				const uint16_t* iv = &params->verts[i * 3];
				const float h = params->bmin[1] + iv[1] * params->ch;
				hmin = dtMin(hmin, h);
				hmax = dtMax(hmax, h);
			}
		}

		hmin -= params->walkableClimb;
		hmax += params->walkableClimb;

		float bmin[3]{}, bmax[3]{};

		dtVcopy(bmin, params->bmin);
		dtVcopy(bmax, params->bmax);
		bmin[1] = hmin;
		bmax[1] = hmax;

		for (int i = 0; i < params->offMeshConCount; ++i)
		{
			constexpr int UCharMax{ (std::numeric_limits<uint8_t>::max)() };
			const float* p0 = &params->offMeshConVerts[(i * 2 + 0) * 3];
			const float* p1 = &params->offMeshConVerts[(i * 2 + 1) * 3];

			offMeshConClass[i * 2 + 0] = classifyOffMeshPoint(p0, bmin, bmax);
			offMeshConClass[i * 2 + 1] = classifyOffMeshPoint(p1, bmin, bmax);

			// Zero out off-mesh start positions which are not even potentially touching the mesh.
			// メッシュに触れていない可能性があるメッシュ以外の開始位置をゼロにします。
			if (offMeshConClass[i * 2 + 0] == UCharMax)
			{
				if (p0[1] < bmin[1] || p0[1] > bmax[1])
					offMeshConClass[i * 2 + 0] = 0;
			}

			// Cound how many links should be allocated for off-mesh connections.
			// オフメッシュ接続に割り当てられるリンクの数を数えます。
			if (offMeshConClass[i * 2 + 0] == UCharMax)
				offMeshConLinkCount++;

			if (offMeshConClass[i * 2 + 1] == UCharMax)
				offMeshConLinkCount++;

			if (offMeshConClass[i * 2 + 0] == UCharMax)
				storedOffMeshConCount++;
		}
	}

	// Off-mesh connectionss are stored as polygons, adjust values.
	// オフメッシュ接続はポリゴンとして保存され、値を調整します。
	const int totPolyCount = params->polyCount + storedOffMeshConCount;
	const int totVertCount = params->vertCount + storedOffMeshConCount * 2;

	// Find portal edges which are at tile borders.
	// タイルの境界にあるポータルエッジを見つけます。
	int edgeCount = 0;
	int portalCount = 0;
	for (int i = 0; i < params->polyCount; ++i)
	{
		const uint16_t* p = &params->polys[i * 2 * nvp];

		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == MESH_NULL_IDX) break;

			edgeCount++;

			if (p[nvp + j] & 0x8000)
			{
				uint16_t dir = p[nvp + j] & 0xf;

				if (dir != 0xf) portalCount++;
			}
		}
	}

	const int maxLinkCount = edgeCount + portalCount * 2 + offMeshConLinkCount * 2;

	// Find unique detail vertices.
	// 一意の詳細頂点を見つけます。
	int uniqueDetailVertCount{};
	int detailTriCount{};

	if (params->detailMeshes)
	{
		// Has detail mesh, count unique detail vertex count and use input detail tri count.
		// 詳細メッシュを持ち、一意の詳細頂点カウントをカウントし、入力詳細トライカウントを使用します。

		detailTriCount = params->detailTriCount;

		for (int i = 0; i < params->polyCount; ++i)
		{
			const uint16_t* p = &params->polys[i * nvp * 2];
			int ndv = params->detailMeshes[i * 4 + 1];
			int nv{};

			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == MESH_NULL_IDX) break;

				nv++;
			}

			ndv -= nv;
			uniqueDetailVertCount += ndv;
		}
	}
	else
	{
		// No input detail mesh, build detail mesh from nav polys.
		// 入力詳細メッシュなし、nav polysから詳細メッシュを構築します。

		uniqueDetailVertCount = 0; // No extra detail verts. // 追加の詳細頂点はありません。
		detailTriCount = 0;

		for (int i = 0; i < params->polyCount; ++i)
		{
			const uint16_t* p = &params->polys[i * nvp * 2];
			int nv{};

			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == MESH_NULL_IDX) break;

				nv++;
			}

			detailTriCount += nv - 2;
		}
	}

	// Calculate data size
	// データサイズを計算します
	constexpr int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float) * 3 * totVertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly) * totPolyCount);
	const int linksSize = dtAlign4(sizeof(dtLink) * maxLinkCount);
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail) * params->polyCount);
	const int detailVertsSize = dtAlign4(sizeof(float) * 3 * uniqueDetailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char) * 4 * detailTriCount);
	const int bvTreeSize = params->buildBvTree ? dtAlign4(sizeof(dtBVNode) * params->polyCount * 2) : 0;
	const int offMeshConsSize = dtAlign4(sizeof(dtOffMeshConnection) * storedOffMeshConCount);

	const int dataSize =
		headerSize + vertsSize + polysSize + linksSize +
		detailMeshesSize + detailVertsSize + detailTrisSize +
		bvTreeSize + offMeshConsSize;

	unsigned char* data = (unsigned char*)dtAlloc(sizeof(unsigned char) * dataSize, DT_ALLOC_PERM);

	// メモリー割り当てが失敗
	if (!data)
	{
		dtFree(offMeshConClass);
		return false;
	}

	memset(data, 0, dataSize);

	unsigned char* d = data;

	dtMeshHeader* header = dtGetThenAdvanceBufferPointer<dtMeshHeader>(d, headerSize);
	float* navVerts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	dtPoly* navPolys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);

	// Ignore links; just leave enough space for them. They'll be created on load.
	// リンクを無視します; 十分なスペースを残してください。 ロード時に作成されます。
	d += linksSize;

	dtPolyDetail* navDMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	float* navDVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	unsigned char* navDTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	dtBVNode* navBvtree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvTreeSize);
	dtOffMeshConnection* offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(d, offMeshConsSize);

	// Store header // ヘッダーを保存
	header->magic = DT_NAVMESH_MAGIC;
	header->version = DT_NAVMESH_VERSION;
	header->x = params->tileX;
	header->y = params->tileY;
	header->layer = params->tileLayer;
	header->userId = params->userId;
	header->polyCount = totPolyCount;
	header->vertCount = totVertCount;
	header->maxLinkCount = maxLinkCount;

	dtVcopy(header->bmin, params->bmin);
	dtVcopy(header->bmax, params->bmax);

	header->detailMeshCount = params->polyCount;
	header->detailVertCount = uniqueDetailVertCount;
	header->detailTriCount = detailTriCount;
	header->bvQuantFactor = 1.f / params->cs;
	header->offMeshBase = params->polyCount;
	header->walkableHeight = params->walkableHeight;
	header->walkableRadius = params->walkableRadius;
	header->walkableClimb = params->walkableClimb;
	header->offMeshConCount = storedOffMeshConCount;
	header->bvNodeCount = params->buildBvTree ? params->polyCount * 2 : 0;

	const int offMeshVertsBase = params->vertCount;
	const int offMeshPolyBase = params->polyCount;

	// Store vertices // 頂点を保存します
	// Mesh vertices  // メッシュの頂点
	for (int i = 0; i < params->vertCount; ++i)
	{
		const uint16_t* iv = &params->verts[i * 3];
		float* v = &navVerts[i * 3];

		v[0] = params->bmin[0] + iv[0] * params->cs;
		v[1] = params->bmin[1] + iv[1] * params->ch;
		v[2] = params->bmin[2] + iv[2] * params->cs;
	}

	// Off-mesh link vertices.
	// オフメッシュリンクの頂点。
	for (int i = 0, n = 0; i < params->offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		// このタイルから始まる接続のみを保存します。
		if (offMeshConClass[i * 2 + 0] == (std::numeric_limits<uint8_t>::max)())
		{
			const float* linkv = &params->offMeshConVerts[i * 2 * 3];
			float* v = &navVerts[(offMeshVertsBase + n * 2) * 3];

			dtVcopy(&v[0], &linkv[0]);
			dtVcopy(&v[3], &linkv[3]);
			n++;
		}
	}

	// Store polygons // ポリゴンを保存します
	// Mesh polys	  // メッシュポリゴン
	const uint16_t* src = params->polys;

	for (int i = 0; i < params->polyCount; ++i)
	{
		dtPoly* p = &navPolys[i];

		p->vertCount = 0;
		p->flags = params->polyFlags[i];
		p->setArea(params->polyAreas[i]);
		p->setType(DT_POLYTYPE_GROUND);

		for (int j = 0; j < nvp; ++j)
		{
			if (src[j] == MESH_NULL_IDX) break;

			p->verts[j] = src[j];

			if (src[nvp + j] & 0x8000)
			{
				// Border or portal edge.
				// 境界線またはポータルエッジ。
				uint16_t dir = src[nvp + j] & 0xf;

				switch (dir)
				{
					case 0xf: // Border // 境界線
					{
						p->neis[j] = 0;
						break;
					}
					case 0: // Portal x-
					{
						p->neis[j] = DT_EXT_LINK | 4;
						break;
					}
					case 1: // Portal z+
					{
						p->neis[j] = DT_EXT_LINK | 2;
						break;
					}
					case 2: // Portal x+
					{
						p->neis[j] = DT_EXT_LINK | 0;
						break;
					}
					case 3: // Portal z-
					{
						p->neis[j] = DT_EXT_LINK | 6;
						break;
					}
				}
			}
			else
			{
				// Normal connection // 通常の接続
				p->neis[j] = src[nvp + j] + 1;
			}

			p->vertCount++;
		}
		src += nvp * 2;
	}

	// Off-mesh connection vertices.
	// メッシュ外の接続頂点。
	for (int i = 0, n = 0; i < params->offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		// このタイルから始まる接続のみを保存します。
		if (offMeshConClass[i * 2 + 0] == 0xff)
		{
			dtPoly* p = &navPolys[offMeshPolyBase + n];

			p->vertCount = 2;
			p->verts[0] = (uint16_t)(offMeshVertsBase + n * 2 + 0);
			p->verts[1] = (uint16_t)(offMeshVertsBase + n * 2 + 1);
			p->flags = params->offMeshConFlags[i];
			p->setArea(params->offMeshConAreas[i]);
			p->setType(DT_POLYTYPE_OFFMESH_CONNECTION);
			n++;
		}
	}

	// Store detail meshes and vertices.
	// 詳細メッシュと頂点を保存します。
	// The nav polygon vertices are stored as the first vertices on each mesh.
	// ナビゲーションポリゴンの頂点は、各メッシュの最初の頂点として保存されます。
	// We compress the mesh data by skipping them and using the navmesh coordinates.
	// メッシュデータをスキップし、ナビメッシュ座標を使用して、メッシュデータを圧縮します。
	if (params->detailMeshes)
	{
		uint16_t vbase{};

		for (int i = 0; i < params->polyCount; ++i)
		{
			dtPolyDetail& dtl = navDMeshes[i];
			const int vb = (int)params->detailMeshes[i * 4 + 0];
			const int ndv = (int)params->detailMeshes[i * 4 + 1];
			const int nv = navPolys[i].vertCount;

			dtl.vertBase = (unsigned int)vbase;
			dtl.vertCount = (unsigned char)(ndv - nv);
			dtl.triBase = (unsigned int)params->detailMeshes[i * 4 + 2];
			dtl.triCount = (unsigned char)params->detailMeshes[i * 4 + 3];

			// Copy vertices except the first 'nv' verts which are equal to nav poly verts.
			// nav poly vertと等しい最初の「nv」頂点以外の頂点をコピーします。
			if (ndv - nv)
			{
				memcpy(&navDVerts[vbase * 3], &params->detailVerts[(vb + nv) * 3], sizeof(float) * 3 * (ndv - nv));
				vbase += (uint16_t)(ndv - nv);
			}
		}

		// Store triangles.
		// 三角形を保存します。
		memcpy(navDTris, params->detailTris, sizeof(unsigned char) * 4 * params->detailTriCount);
	}
	else
	{
		// Create dummy detail mesh by triangulating polys.
		// ポリゴンを三角形分割してダミーの詳細メッシュを作成します。
		int tbase{};
		for (int i = 0; i < params->polyCount; ++i)
		{
			dtPolyDetail& dtl = navDMeshes[i];
			const int nv = navPolys[i].vertCount;

			dtl.vertBase = 0;
			dtl.vertCount = 0;
			dtl.triBase = (unsigned int)tbase;
			dtl.triCount = (unsigned char)(nv - 2);

			// Triangulate polygon (local indices).
			// ポリゴンを三角形化します（ローカルインデックス）。
			for (int j = 2; j < nv; ++j)
			{
				unsigned char* t = &navDTris[tbase * 4];

				t[0] = 0;
				t[1] = (unsigned char)(j - 1);
				t[2] = (unsigned char)j;
				// Bit for each edge that belongs to poly boundary.
				// ポリゴン境界に属する各エッジのビット。
				t[3] = (1 << 2);

				if (j == 2) t[3] |= (1 << 0);
				if (j == nv - 1) t[3] |= (1 << 4);

				tbase++;
			}
		}
	}

	// Store and create BVtree.
	// BVtreeを保存および作成します。
	if (params->buildBvTree)
		createBVTree(params, navBvtree, 2 * params->polyCount);

	// Store Off-Mesh connections.
	// オフメッシュ接続を保存します。
	for (int i = 0, n = 0; i < params->offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		// このタイルから始まる接続のみを保存します。
		if (offMeshConClass[i * 2 + 0] == 0xff)
		{
			dtOffMeshConnection* con = &offMeshCons[n];

			con->poly = (uint16_t)(offMeshPolyBase + n);

			// Copy connection end-points.
			// 接続のエンドポイントをコピーします。
			const float* endPts = &params->offMeshConVerts[i * 2 * 3];

			dtVcopy(&con->pos[0], &endPts[0]);
			dtVcopy(&con->pos[3], &endPts[3]);
			con->rad = params->offMeshConRad[i];
			con->flags = params->offMeshConDir[i] ? DT_OFFMESH_CON_BIDIR : 0;
			con->side = offMeshConClass[i * 2 + 1];

			if (params->offMeshConUserID)
				con->userId = params->offMeshConUserID[i];

			n++;
		}
	}

	dtFree(offMeshConClass);

	*outData = data;
	*outDataSize = dataSize;

	return true;
}

bool dtNavMeshHeaderSwapEndian(unsigned char* data, const int /*dataSize*/)
{
	dtMeshHeader* header = (dtMeshHeader*)data;

	int swappedMagic = DT_NAVMESH_MAGIC;
	int swappedVersion = DT_NAVMESH_VERSION;
	dtSwapEndian(&swappedMagic);
	dtSwapEndian(&swappedVersion);

	if ((header->magic != DT_NAVMESH_MAGIC || header->version != DT_NAVMESH_VERSION) &&
		(header->magic != swappedMagic || header->version != swappedVersion))
	{
		return false;
	}

	dtSwapEndian(&header->magic);
	dtSwapEndian(&header->version);
	dtSwapEndian(&header->x);
	dtSwapEndian(&header->y);
	dtSwapEndian(&header->layer);
	dtSwapEndian(&header->userId);
	dtSwapEndian(&header->polyCount);
	dtSwapEndian(&header->vertCount);
	dtSwapEndian(&header->maxLinkCount);
	dtSwapEndian(&header->detailMeshCount);
	dtSwapEndian(&header->detailVertCount);
	dtSwapEndian(&header->detailTriCount);
	dtSwapEndian(&header->bvNodeCount);
	dtSwapEndian(&header->offMeshConCount);
	dtSwapEndian(&header->offMeshBase);
	dtSwapEndian(&header->walkableHeight);
	dtSwapEndian(&header->walkableRadius);
	dtSwapEndian(&header->walkableClimb);
	dtSwapEndian(&header->bmin[0]);
	dtSwapEndian(&header->bmin[1]);
	dtSwapEndian(&header->bmin[2]);
	dtSwapEndian(&header->bmax[0]);
	dtSwapEndian(&header->bmax[1]);
	dtSwapEndian(&header->bmax[2]);
	dtSwapEndian(&header->bvQuantFactor);

	// Freelist index and pointers are updated when tile is added, no need to swap.

	return true;
}

// @par
//
// @warning This function assumes that the header is in the correct endianess already.
// Call #dtNavMeshHeaderSwapEndian() first on the data if the data is expected to be in wrong endianess
// to start with. Call #dtNavMeshHeaderSwapEndian() after the data has been swapped if converting from
// native to foreign endianess.
bool dtNavMeshDataSwapEndian(unsigned char* data, const int /*dataSize*/)
{
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return false;
	if (header->version != DT_NAVMESH_VERSION)
		return false;

	// Patch header pointers.
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float) * 3 * header->vertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly) * header->polyCount);
	const int linksSize = dtAlign4(sizeof(dtLink) * (header->maxLinkCount));
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail) * header->detailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float) * 3 * header->detailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char) * 4 * header->detailTriCount);
	const int bvtreeSize = dtAlign4(sizeof(dtBVNode) * header->bvNodeCount);
	const int offMeshLinksSize = dtAlign4(sizeof(dtOffMeshConnection) * header->offMeshConCount);

	unsigned char* d = data + headerSize;
	float* verts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	dtPoly* polys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
	d += linksSize; // Ignore links; they technically should be endian-swapped but all their data is overwritten on load anyway.
	//dtLink* links = dtGetThenAdvanceBufferPointer<dtLink>(d, linksSize);
	dtPolyDetail* detailMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	float* detailVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	d += detailTrisSize; // Ignore detail tris; single bytes can't be endian-swapped.
	//unsigned char* detailTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	dtBVNode* bvTree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvtreeSize);
	dtOffMeshConnection* offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(d, offMeshLinksSize);

	// Vertices
	for (int i = 0; i < header->vertCount * 3; ++i)
	{
		dtSwapEndian(&verts[i]);
	}

	// Polys
	for (int i = 0; i < header->polyCount; ++i)
	{
		dtPoly* p = &polys[i];
		// poly->firstLink is update when tile is added, no need to swap.
		for (int j = 0; j < DT_VERTS_PER_POLYGON; ++j)
		{
			dtSwapEndian(&p->verts[j]);
			dtSwapEndian(&p->neis[j]);
		}
		dtSwapEndian(&p->flags);
	}

	// Links are rebuild when tile is added, no need to swap.

	// Detail meshes
	for (int i = 0; i < header->detailMeshCount; ++i)
	{
		dtPolyDetail* pd = &detailMeshes[i];
		dtSwapEndian(&pd->vertBase);
		dtSwapEndian(&pd->triBase);
	}

	// Detail verts
	for (int i = 0; i < header->detailVertCount * 3; ++i)
	{
		dtSwapEndian(&detailVerts[i]);
	}

	// BV-tree
	for (int i = 0; i < header->bvNodeCount; ++i)
	{
		dtBVNode* node = &bvTree[i];
		for (int j = 0; j < 3; ++j)
		{
			dtSwapEndian(&node->bmin[j]);
			dtSwapEndian(&node->bmax[j]);
		}
		dtSwapEndian(&node->i);
	}

	// Off-mesh Connections.
	for (int i = 0; i < header->offMeshConCount; ++i)
	{
		dtOffMeshConnection* con = &offMeshCons[i];
		for (int j = 0; j < 6; ++j)
			dtSwapEndian(&con->pos[j]);
		dtSwapEndian(&con->rad);
		dtSwapEndian(&con->poly);
	}

	return true;
}
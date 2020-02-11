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

#include <cfloat>
#include <cstring>
#include "DetourNavMeshQuery.h"
#include "DetourNavMesh.h"
#include "DetourNode.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include <new>

// @class dtQueryFilter
//
// <b>The Default Implementation</b> デフォルトの実装
//
// At construction: All area costs default to 1.0.  All flags are included
// and none are excluded.
// 建設時：すべてのエリアコストのデフォルトは1.0です。すべてのフラグが含まれ、どれも除外されません。
//
// If a polygon has both an include and an exclude flag, it will be excluded.
// ポリゴンに包含フラグと除外フラグの両方がある場合、ポリゴンは除外されます。
//
// The way filtering works, a navigation mesh polygon must have at least one flag
// set to ever be considered by a query. So a polygon with no flags will never
// be considered.
// フィルタリングの仕組みでは、ナビゲーションメッシュポリゴンには、
// クエリで考慮されるために少なくとも1つのフラグが設定されている必要があります。
// そのため、フラグのないポリゴンは考慮されません。
//
// Setting the include flags to 0 will result in all polygons being excluded.
// includeフラグを0に設定すると、すべてのポリゴンが除外されます。
//
// <b>Custom Implementations</b> カスタム実装
//
// DT_VIRTUAL_QUERYFILTER must be defined in order to extend this class.
// このクラスを拡張するには、DT_VIRTUAL_QUERYFILTERを定義する必要があります。
//
// Implement a custom query filter by overriding the virtual passFilter()
// and getCost() functions. If this is done, both functions should be as
// fast as possible. Use cached local copies of data rather than accessing
// your own objects where possible.
// 仮想passFilter（）およびgetCost（）関数をオーバーライドして、カスタムクエリフィルターを実装します。
// これを行うと、両方の機能が可能な限り高速になります。
// 可能な場合は、独自のオブジェクトにアクセスするのではなく、キャッシュされたデータのローカルコピーを使用します。
//
// Custom implementations do not need to adhere to the flags or cost logic
// used by the default implementation.
// カスタム実装は、デフォルト実装で使用されるフラグまたはコストロジックに従う必要はありません。
//
// In order for A* searches to work properly, the cost should be proportional to
// the travel distance. Implementing a cost modifier less than 1.0 is likely
// to lead to problems during pathfinding.
// A *検索が適切に機能するためには、コストは移動距離に比例する必要があります。
// 1.0未満のコスト修飾子を実装すると、パス検索中に問題が発生する可能性があります。
//
// @see dtNavMeshQuery

dtQueryFilter::dtQueryFilter() :
	m_includeFlags(0xffff),
	m_excludeFlags(0)
{
	for (int i = 0; i < DT_MAX_AREAS; ++i)
		m_areaCost[i] = 1.f;
}

#ifdef DT_VIRTUAL_QUERYFILTER
bool dtQueryFilter::passFilter(const dtPolyRef /*ref*/,
	const dtMeshTile* /*tile*/,
	const dtPoly* poly) const
{
	return (poly->flags & m_includeFlags) != 0 && (poly->flags & m_excludeFlags) == 0;
}

float dtQueryFilter::getCost(const float* pa, const float* pb,
	const dtPolyRef /*prevRef*/, const dtMeshTile* /*prevTile*/, const dtPoly* /*prevPoly*/,
	const dtPolyRef /*curRef*/, const dtMeshTile* /*curTile*/, const dtPoly* curPoly,
	const dtPolyRef /*nextRef*/, const dtMeshTile* /*nextTile*/, const dtPoly* /*nextPoly*/) const
{
	return dtVdist(pa, pb) * m_areaCost[curPoly->getArea()];
}
#else
inline bool dtQueryFilter::passFilter(const dtPolyRef /*ref*/,
	const dtMeshTile* /*tile*/,
	const dtPoly* poly) const
{
	return (poly->flags & m_includeFlags) != 0 && (poly->flags & m_excludeFlags) == 0;
}

inline float dtQueryFilter::getCost(const float* pa, const float* pb,
	const dtPolyRef /*prevRef*/, const dtMeshTile* /*prevTile*/, const dtPoly* /*prevPoly*/,
	const dtPolyRef /*curRef*/, const dtMeshTile* /*curTile*/, const dtPoly* curPoly,
	const dtPolyRef /*nextRef*/, const dtMeshTile* /*nextTile*/, const dtPoly* /*nextPoly*/) const
{
	return dtVdist(pa, pb) * m_areaCost[curPoly->getArea()];
}
#endif

constexpr float H_SCALE = 0.999f; // Search heuristic scale. 検索ヒューリスティックスケール。

// Allocates a query object using the Detour allocator.
// Detourアロケーターを使用してクエリオブジェクトを割り当てます。
dtNavMeshQuery* dtAllocNavMeshQuery()
{
	void* mem = dtAlloc(sizeof(dtNavMeshQuery), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtNavMeshQuery;
}

void dtFreeNavMeshQuery(dtNavMeshQuery* navmesh)
{
	if (!navmesh) return;
	navmesh->~dtNavMeshQuery();
	dtFree(navmesh);
}

////////////////////////////////////////////////////////////

// @class dtNavMeshQuery
//
// For methods that support undersized buffers, if the buffer is too small
// to hold the entire result set the return status of the method will include
// the #DT_BUFFER_TOO_SMALL flag.
// サイズの小さいバッファをサポートするメソッドの場合や、バッファが小さすぎて結果セット全体を保持できない場合は、
// メソッドの戻りステータスには#DT_BUFFER_TOO_SMALLフラグが含まれます。
//
// Constant member functions can be used by multiple clients without side
// effects. (E.g. No change to the closed list. No impact on an in-progress
// sliced path query. Etc.)
// 定数メンバー関数は、副作用なしで複数のクライアントで使用できます。
//（例：クローズドリストへの変更はありません。進行中のスライスパスクエリへの影響はありません。など）
//
// Walls and portals: A @e wall is a polygon segment that is
// considered impassable. A @e portal is a passable segment between polygons.
// A portal may be treated as a wall based on the dtQueryFilter used for a query.
// 壁とポータル：壁は、通過できないと見なされるポリゴンセグメントです。
// ポータルは、ポリゴン間の通過可能なセグメントです。
// ポータルは、クエリに使用されるdtQueryFilterに基づいて壁として扱うことができます。
//
// @see dtNavMesh, dtQueryFilter, #dtAllocNavMeshQuery(), #dtAllocNavMeshQuery()

dtNavMeshQuery::dtNavMeshQuery() :
	m_nav(0),
	m_tinyNodePool(0),
	m_nodePool(0),
	m_openList(0)
{
	memset(&m_query, 0, sizeof(dtQueryData));
}

dtNavMeshQuery::~dtNavMeshQuery()
{
	if (m_tinyNodePool)
		m_tinyNodePool->~dtNodePool();
	if (m_nodePool)
		m_nodePool->~dtNodePool();
	if (m_openList)
		m_openList->~dtNodeQueue();
	dtFree(m_tinyNodePool);
	dtFree(m_nodePool);
	dtFree(m_openList);
}

// @par
//
// Must be the first function called after construction, before other
// functions are used.
// 他の関数が使用される前に、構築後に最初に呼び出される関数でなければなりません。
//
// This function can be used multiple times.
// この関数は複数回使用できます。
dtStatus dtNavMeshQuery::init(const dtNavMesh* nav, const int maxNodes)
{
	if (maxNodes > DT_NULL_IDX || maxNodes > (1 << DT_NODE_PARENT_BITS) - 1)
		return DT_FAILURE | DT_INVALID_PARAM;

	m_nav = nav;

	if (!m_nodePool || m_nodePool->getMaxNodes() < maxNodes)
	{
		if (m_nodePool)
		{
			m_nodePool->~dtNodePool();
			dtFree(m_nodePool);
			m_nodePool = 0;
		}
		m_nodePool = new (dtAlloc(sizeof(dtNodePool), DT_ALLOC_PERM)) dtNodePool(maxNodes, dtNextPow2(maxNodes / 4));
		if (!m_nodePool)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
	}
	else
	{
		m_nodePool->clear();
	}

	if (!m_tinyNodePool)
	{
		m_tinyNodePool = new (dtAlloc(sizeof(dtNodePool), DT_ALLOC_PERM)) dtNodePool(64, 32);
		if (!m_tinyNodePool)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
	}
	else
	{
		m_tinyNodePool->clear();
	}

	if (!m_openList || m_openList->getCapacity() < maxNodes)
	{
		if (m_openList)
		{
			m_openList->~dtNodeQueue();
			dtFree(m_openList);
			m_openList = 0;
		}
		m_openList = new (dtAlloc(sizeof(dtNodeQueue), DT_ALLOC_PERM)) dtNodeQueue(maxNodes);
		if (!m_openList)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
	}
	else
	{
		m_openList->clear();
	}

	return DT_SUCCESS;
}

// navmeshのランダムな位置を返します。
// ポリゴンは、エリアごとに重み付けされて選択されます。検索は、ポリゴンの数に関連する線形で実行されます。
dtStatus dtNavMeshQuery::findRandomPoint(const dtQueryFilter* filter, float (*frand)(),
	dtPolyRef* randomRef, float* randomPt) const
{
	dtAssert(m_nav);

	// Randomly pick one tile. Assume that all tiles cover roughly the same area.
	// ランダムに1つのタイルを選択します。すべてのタイルがほぼ同じ領域をカバーすると仮定します。
	const dtMeshTile* tile = 0;
	float tsum = 0.0f;
	for (int i = 0; i < m_nav->getMaxTiles(); i++)
	{
		const dtMeshTile* t = m_nav->getTile(i);
		if (!t || !t->header) continue;

		// Choose random tile using reservoi sampling.
		//リザーバーサンプリングを使用してランダムタイルを選択します。
		const float area = 1.f; // Could be tile area too. タイル領域にもなります。
		tsum += area;
		const float u = frand();
		if (u * tsum <= area)
			tile = t;
	}
	if (!tile)
		return DT_FAILURE;

	// Randomly pick one polygon weighted by polygon area.
	// ポリゴン領域で重み付けされた1つのポリゴンをランダムに選択します。
	const dtPoly* poly = 0;
	dtPolyRef polyRef = 0;
	const dtPolyRef base = m_nav->getPolyRefBase(tile);

	float areaSum = 0.0f;
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];

		// Do not return off-mesh connection polygons.
		// オフメッシュ接続ポリゴンを返しません。
		if (p->getType() != DT_POLYTYPE_GROUND)
			continue;

		// Must pass filter
		// フィルタを渡す必要があります
		const dtPolyRef ref = base | (dtPolyRef)i;
		if (!filter->passFilter(ref, tile, p))
			continue;

		// Calc area of the polygon.
		// ポリゴンの面積を計算します。
		float polyArea = 0.0f;
		for (int j = 2; j < p->vertCount; ++j)
		{
			const float* va = &tile->verts[p->verts[0] * 3];
			const float* vb = &tile->verts[p->verts[j - 1] * 3];
			const float* vc = &tile->verts[p->verts[j] * 3];
			polyArea += dtTriArea2D(va, vb, vc);
		}

		// Choose random polygon weighted by area, using reservoi sampling.
		//リザーバサンプリングを使用して、面積で重み付けされたランダムなポリゴンを選択します。
		areaSum += polyArea;
		const float u = frand();
		if (u * areaSum <= polyArea)
		{
			poly = p;
			polyRef = ref;
		}
	}

	if (!poly) return DT_FAILURE;

	// Randomly pick point on polygon.
	// ポリゴン上のポイントをランダムに選択します。
	const float* v = &tile->verts[poly->verts[0] * 3];
	float verts[3 * DT_VERTS_PER_POLYGON];
	float areas[DT_VERTS_PER_POLYGON];

	dtVcopy(&verts[0 * 3], v); // コピー

	for (int j = 1; j < poly->vertCount; ++j)
	{
		v = &tile->verts[poly->verts[j] * 3];
		dtVcopy(&verts[j * 3], v); // コピー
	}

	const float s = frand();
	const float t = frand();

	float pt[3];
	dtRandomPointInConvexPoly(verts, poly->vertCount, areas, s, t, pt);

	float ht{ 0.0f };
	dtStatus status = getPolyHeight(polyRef, pt, &ht);

	if (dtStatusFailed(status))
		return status;

	pt[1] = ht;

	dtVcopy(randomPt, pt); // コピー
	*randomRef = polyRef;

	return DT_SUCCESS;
}

// 指定された場所の範囲内でnavmeshのランダムな場所を返します。
// ポリゴンは、エリアごとに重み付けされて選択されます。検索は、ポリゴンの数に関連する線形で実行されます。
// 位置は円によって厳密に制約されませんが、訪問したポリゴンを制限します。
dtStatus dtNavMeshQuery::findRandomPointAroundCircle(dtPolyRef startRef, const float* centerPos, const float maxRadius,
	const dtQueryFilter* filter, float (*frand)(),
	dtPolyRef* randomRef, float* randomPt) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	// Validate input
	//入力を検証します
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;

	const dtMeshTile* startTile{ nullptr };
	const dtPoly* startPoly{ nullptr };

	// ポリゴン参照のタイルとポリゴンを取得
	m_nav->getTileAndPolyByRefUnsafe(startRef, &startTile, &startPoly);

	if (!filter->passFilter(startRef, startTile, startPoly))
		return DT_FAILURE | DT_INVALID_PARAM;

	m_nodePool->clear();
	m_openList->clear();

	dtNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos); // コピー
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);

	dtStatus status = DT_SUCCESS;

	const float radiusSqr = dtSqr(maxRadius);  // 二乗
	float areaSum = 0.0f;

	const dtMeshTile* randomTile{ nullptr };
	const dtPoly* randomPoly{ nullptr };
	dtPolyRef randomPolyRef{ 0u };

	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;

		// Get poly and tile.
		// ポリゴンとタイルを取得します。
		// The API input has been cheked already, skip checking internal data.
		// API入力は既にチェックされており、内部データのチェックをスキップします。
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile{ nullptr };
		const dtPoly* bestPoly{ nullptr };

		// ポリゴン参照のタイルとポリゴンを取得
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);

		// Place random locations on on ground.
		//ランダムな場所を地面に置きます。
		if (bestPoly->getType() == DT_POLYTYPE_GROUND)
		{
			// Calc area of the polygon.
			// ポリゴンの面積を計算します。
			float polyArea = 0.0f;
			for (int j = 2; j < bestPoly->vertCount; ++j)
			{
				const float* va = &bestTile->verts[bestPoly->verts[0] * 3];
				const float* vb = &bestTile->verts[bestPoly->verts[j - 1] * 3];
				const float* vc = &bestTile->verts[bestPoly->verts[j] * 3];
				polyArea += dtTriArea2D(va, vb, vc);
			}

			// Choose random polygon weighted by area, using reservoi sampling.
			// リザーバサンプリングを使用して、面積で重み付けされたランダムなポリゴンを選択します。
			areaSum += polyArea;
			const float u = frand();
			if (u * areaSum <= polyArea)
			{
				randomTile = bestTile;
				randomPoly = bestPoly;
				randomPolyRef = bestRef;
			}
		}

		// Get parent poly and tile.
		// 親ポリゴンとタイルを取得します。
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);

		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const dtLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;

			// Skip invalid neighbours and do not follow back to parent.
			// 無効な付近をスキップし、親にフォローしないでください。
			if (!neighbourRef || neighbourRef == parentRef)
				continue;

			// Expand to neighbour
			// 付近に展開
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);

			// Do not advance if the polygon is excluded by the filter.
			// ポリゴンがフィルターによって除外されている場合、先に進まないでください。
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;

			// Find edge and calc distance to the edge.
			// エッジを見つけ、エッジまでの距離を計算します。
			float va[3]{}, vb[3]{};
			if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;

			// If the circle is not touching the next polygon, skip it.
			// 円が次のポリゴンに接触していない場合、スキップします。
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			if (distSqr > radiusSqr)
				continue;

			dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}

			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;

			// Cost
			if (neighbourNode->flags == 0)
				dtVlerp(neighbourNode->pos, va, vb, 0.5f);

			const float total = bestNode->total + dtVdist(bestNode->pos, neighbourNode->pos);

			// The node is already in open list and the new result is worse, skip.
			// ノードは既にオープンリストにあり、新しい結果はさらに悪いので、スキップします。
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;

			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;

			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				neighbourNode->flags = DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}

	if (!randomPoly)
		return DT_FAILURE;

	// Randomly pick point on polygon.
	// ポリゴン上のポイントをランダムに選択します。
	const float* v = &randomTile->verts[randomPoly->verts[0] * 3];
	float verts[3 * DT_VERTS_PER_POLYGON];
	float areas[DT_VERTS_PER_POLYGON];
	dtVcopy(&verts[0 * 3], v);

	for (int j = 1; j < randomPoly->vertCount; ++j)
	{
		v = &randomTile->verts[randomPoly->verts[j] * 3];
		dtVcopy(&verts[j * 3], v);
	}

	const float s = frand();
	const float t = frand();

	float pt[3];
	dtRandomPointInConvexPoly(verts, randomPoly->vertCount, areas, s, t, pt);

	float h = 0.0f;
	dtStatus stat = getPolyHeight(randomPolyRef, pt, &h);
	if (dtStatusFailed(status))
		return stat;
	pt[1] = h;

	dtVcopy(randomPt, pt);
	*randomRef = randomPolyRef;

	return DT_SUCCESS;
}

////////////////////////////////////////////////////////////

// @par
//
// Uses the detail polygons to find the surface height. (Most accurate.)
// 詳細ポリゴンを使用して、表面の高さを見つけます。 （最も正確です。）
//
// @p pos does not have to be within the bounds of the polygon or navigation mesh.
// posは、ポリゴンまたはナビゲーションメッシュの境界内にある必要はありません。
//
// See closestPointOnPolyBoundary() for a limited but faster option.
// 制限されているがより高速なオプションについては、closestPointOnPolyBoundary（）を参照してください。
//
dtStatus dtNavMeshQuery::closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const
{
	dtAssert(m_nav);
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
		return DT_FAILURE | DT_INVALID_PARAM;
	if (!tile)
		return DT_FAILURE | DT_INVALID_PARAM;

	// Off-mesh connections don't have detail polygons.
	// オフメッシュ接続には詳細ポリゴンがありません。
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const float* v0 = &tile->verts[poly->verts[0] * 3];
		const float* v1 = &tile->verts[poly->verts[1] * 3];
		const float d0 = dtVdist(pos, v0); // 引き算
		const float d1 = dtVdist(pos, v1); // 引き算
		const float u = d0 / (d0 + d1);
		dtVlerp(closest, v0, v1, u); // 線形補間

		if (posOverPoly)
			*posOverPoly = false;

		return DT_SUCCESS;
	}

	const unsigned int ip = (unsigned int)(poly - tile->polys);
	const dtPolyDetail* pd = &tile->detailMeshes[ip];

	// Clamp point to be inside the polygon.
	//多角形の内側にあるクランプポイント。
	float verts[DT_VERTS_PER_POLYGON * 3];
	float edged[DT_VERTS_PER_POLYGON];
	float edget[DT_VERTS_PER_POLYGON];
	const int nv = poly->vertCount;
	for (int i = 0; i < nv; ++i)
		dtVcopy(&verts[i * 3], &tile->verts[poly->verts[i] * 3]);

	dtVcopy(closest, pos);
	if (!dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget))
	{
		// Point is outside the polygon, dtClamp to nearest edge.
		// ポイントはポリゴンの外側にあり、最も近いエッジにクランプします。
		float dmin = edged[0];
		int imin = 0;
		for (int i = 1; i < nv; ++i)
		{
			if (edged[i] < dmin)
			{
				dmin = edged[i];
				imin = i;
			}
		}
		const float* va = &verts[imin * 3];
		const float* vb = &verts[((imin + 1) % nv) * 3];
		dtVlerp(closest, va, vb, edget[imin]);

		if (posOverPoly)
			*posOverPoly = false;
	}
	else
	{
		if (posOverPoly)
			*posOverPoly = true;
	}

	// Find height at the location.
	// その場所の高さを見つけます。
	for (int j = 0; j < pd->triCount; ++j)
	{
		const unsigned char* t = &tile->detailTris[(pd->triBase + j) * 4];
		const float* v[3];
		for (int k = 0; k < 3; ++k)
		{
			if (t[k] < poly->vertCount)
				v[k] = &tile->verts[poly->verts[t[k]] * 3];
			else
				v[k] = &tile->detailVerts[(pd->vertBase + (t[k] - poly->vertCount)) * 3];
		}
		float h;
		if (dtClosestHeightPointTriangle(closest, v[0], v[1], v[2], h))
		{
			closest[1] = h;
			break;
		}
	}

	return DT_SUCCESS;
}

// @par
//
// Much faster than closestPointOnPoly().
// nearestPointOnPoly（）よりもはるかに高速。
//
// If the provided position lies within the polygon's xz-bounds (above or below),
// then @p pos and @p closest will be equal.
// 指定された位置がポリゴンのxz境界内（上または下）にある場合、その後、@ p posと@p最も近い値が等しくなります。
//
// The height of @p closest will be the polygon boundary.  The height detail is not used.
// @pの最も近い高さがポリゴンの境界になります。高さの詳細は使用されません。
//
// @p pos does not have to be within the bounds of the polybon or the navigation mesh.
// @p posは、polybonまたはナビゲーションメッシュの境界内にある必要はありません。
//
dtStatus dtNavMeshQuery::closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, float* closest) const
{
	dtAssert(m_nav);

	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
		return DT_FAILURE | DT_INVALID_PARAM;

	// Collect vertices.
	//頂点を収集します。
	float verts[DT_VERTS_PER_POLYGON * 3];
	float edged[DT_VERTS_PER_POLYGON];
	float edget[DT_VERTS_PER_POLYGON];
	int nv = 0;
	for (int i = 0; i < (int)poly->vertCount; ++i)
	{
		dtVcopy(&verts[nv * 3], &tile->verts[poly->verts[i] * 3]); // コピー
		nv++;
	}

	bool inside = dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget);
	if (inside)
	{
		// Point is inside the polygon, return the point.
		//ポイントはポリゴン内にあり、ポイントを返します。
		dtVcopy(closest, pos); // コピー
	}
	else
	{
		// Point is outside the polygon, dtClamp to nearest edge.
		// ポイントはポリゴンの外側にあり、最も近いエッジにクランプします。
		float dmin = edged[0];
		int imin = 0;
		for (int i = 1; i < nv; ++i)
		{
			if (edged[i] < dmin)
			{
				dmin = edged[i];
				imin = i;
			}
		}
		const float* va = &verts[imin * 3];
		const float* vb = &verts[((imin + 1) % nv) * 3];
		dtVlerp(closest, va, vb, edget[imin]);  // 2つのベクトル間で線形補間
	}

	return DT_SUCCESS;
}

// @par
//
// Will return #DT_FAILURE if the provided position is outside the xz-bounds
// of the polygon.
// 指定された位置がポリゴンのxz境界の外側にある場合、＃DT_FAILUREを返します。
//
dtStatus dtNavMeshQuery::getPolyHeight(dtPolyRef ref, const float* pos, float* height) const
{
	dtAssert(m_nav);

	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
		return DT_FAILURE | DT_INVALID_PARAM;

	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const float* v0 = &tile->verts[poly->verts[0] * 3];
		const float* v1 = &tile->verts[poly->verts[1] * 3];
		const float d0 = dtVdist2D(pos, v0);
		const float d1 = dtVdist2D(pos, v1);
		const float u = d0 / (d0 + d1);
		if (height)
			*height = v0[1] + (v1[1] - v0[1]) * u;
		return DT_SUCCESS;
	}
	else
	{
		const unsigned int ip = (unsigned int)(poly - tile->polys);
		const dtPolyDetail* pd = &tile->detailMeshes[ip];
		for (int j = 0; j < pd->triCount; ++j)
		{
			const unsigned char* t = &tile->detailTris[(pd->triBase + j) * 4];
			const float* v[3];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < poly->vertCount)
					v[k] = &tile->verts[poly->verts[t[k]] * 3];
				else
					v[k] = &tile->detailVerts[(pd->vertBase + (t[k] - poly->vertCount)) * 3];
			}
			float h;
			if (dtClosestHeightPointTriangle(pos, v[0], v[1], v[2], h))
			{
				if (height)
					*height = h;
				return DT_SUCCESS;
			}
		}
	}

	return DT_FAILURE | DT_INVALID_PARAM;
}

class dtFindNearestPolyQuery : public dtPolyQuery
{
	const dtNavMeshQuery* m_query;
	const float* m_center;
	float m_nearestDistanceSqr;
	dtPolyRef m_nearestRef;
	float m_nearestPoint[3];

public:
	dtFindNearestPolyQuery(const dtNavMeshQuery* query, const float* center)
		: m_query(query), m_center(center), m_nearestDistanceSqr((std::numeric_limits<float>::max)()), m_nearestRef(0), m_nearestPoint()
	{
	}

	dtPolyRef nearestRef() const { return m_nearestRef; }
	const float* nearestPoint() const { return m_nearestPoint; }

	void process(const dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count)
	{
		dtIgnoreUnused(polys);

		for (int i = 0; i < count; ++i)
		{
			dtPolyRef ref = refs[i];
			float closestPtPoly[3];
			float diff[3];
			bool posOverPoly = false;
			float d;
			m_query->closestPointOnPoly(ref, m_center, closestPtPoly, &posOverPoly);

			// If a point is directly over a polygon and closer than
			// climb height, favor that instead of straight line nearest point.
			// ポイントがポリゴンの真上にあり、上昇の高さよりも近い場合は、
			// 直線に最も近いポイントではなく、ポイントを優先します。
			dtVsub(diff, m_center, closestPtPoly);
			if (posOverPoly)
			{
				d = dtAbs(diff[1]) - tile->header->walkableClimb;
				d = d > 0 ? d * d : 0;
			}
			else
			{
				d = dtVlenSqr(diff);
			}

			if (d < m_nearestDistanceSqr)
			{
				dtVcopy(m_nearestPoint, closestPtPoly);

				m_nearestDistanceSqr = d;
				m_nearestRef = ref;
			}
		}
	}
};

// @par
//
// @note If the search box does not intersect any polygons the search will
// return #DT_SUCCESS, but @p nearestRef will be zero. So if in doubt, check
// @p nearestRef before using @p nearestPt.
// 検索ボックスがポリゴンと交差しない場合、検索は#DT_SUCCESSを返しますが、nearestRefはゼロになります。
// 疑わしい場合は、nearestPtを使用する前に、nearestRefを確認してください。
dtStatus dtNavMeshQuery::findNearestPoly(const float* center, const float* extents,
	const dtQueryFilter* filter,
	dtPolyRef* nearestRef, float* nearestPt) const
{
	dtAssert(m_nav);

	if (!nearestRef)
		return DT_FAILURE | DT_INVALID_PARAM;

	dtFindNearestPolyQuery query(this, center);

	// 検索ボックスに重なるポリゴンを検​​索
	dtStatus status = queryPolygons(center, extents, filter, &query);
	if (dtStatusFailed(status))
		return status;

	*nearestRef = query.nearestRef();

	// Only override nearestPt if we actually found a poly so the nearest point is valid.
	// 最も近いポイントが有効になるように実際にポリゴンを見つけた場合のみ、nearestPtをオーバーライドします。
	if (nearestPt && *nearestRef)
		dtVcopy(nearestPt, query.nearestPoint());

	return DT_SUCCESS;
}

// タイル内のポリゴンを照会します。
void dtNavMeshQuery::queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
	const dtQueryFilter* filter, dtPolyQuery* query) const
{
	dtAssert(m_nav);
	constexpr int batchSize = 32;
	dtPolyRef polyRefs[batchSize];
	dtPoly* polys[batchSize];
	int n = 0;

	if (tile->bvTree)
	{
		const dtBVNode* node = &tile->bvTree[0];
		const dtBVNode* end = &tile->bvTree[tile->header->bvNodeCount];
		const auto& tbmin = tile->header->bmin;
		const auto& tbmax = tile->header->bmax;
		const float qfac = tile->header->bvQuantFactor;

		// Calculate quantized box
		// 量子化ボックスを計算
		uint16_t bmin[3], bmax[3];

		// dtClamp query box to world box.
		// dtクエリボックスをワールドボックスにクランプします。
		float minx = dtClamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
		float miny = dtClamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
		float minz = dtClamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
		float maxx = dtClamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
		float maxy = dtClamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
		float maxz = dtClamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];

		// Quantize
		// クオンタイズ
		bmin[0] = (uint16_t)(qfac * minx) & 0xfffe;
		bmin[1] = (uint16_t)(qfac * miny) & 0xfffe;
		bmin[2] = (uint16_t)(qfac * minz) & 0xfffe;
		bmax[0] = (uint16_t)(qfac * maxx + 1) | 1;
		bmax[1] = (uint16_t)(qfac * maxy + 1) | 1;
		bmax[2] = (uint16_t)(qfac * maxz + 1) | 1;

		// Traverse tree（検索アルゴリズム：ツリートラバーサル）
		const dtPolyRef base = m_nav->getPolyRefBase(tile);
		while (node < end)
		{
			const bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin.data(), node->bmax.data());
			const bool isLeafNode = node->i >= 0;

			if (isLeafNode && overlap)
			{
				dtPolyRef ref = base | (dtPolyRef)node->i;
				if (filter->passFilter(ref, tile, &tile->polys[node->i]))
				{
					polyRefs[n] = ref;
					polys[n] = &tile->polys[node->i];

					if (n == batchSize - 1)
					{
						query->process(tile, polys, polyRefs, batchSize);
						n = 0;
					}
					else
					{
						n++;
					}
				}
			}

			if (overlap || isLeafNode)
				node++;
			else
			{
				const int escapeIndex = -node->i;
				node += escapeIndex;
			}
		}
	}
	else
	{
		float bmin[3], bmax[3];
		const dtPolyRef base = m_nav->getPolyRefBase(tile);
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			dtPoly* p{ &tile->polys[i] };

			// Do not return off-mesh connection polygons.
			// オフメッシュ接続ポリゴンを返しません。
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;

			// Must pass filter
			// フィルターを通過する必要があります
			const dtPolyRef ref{ base | (dtPolyRef)i };
			if (!filter->passFilter(ref, tile, p))
				continue;

			// Calc polygon bounds.
			// ポリゴンの境界を計算します。
			const float* v = &tile->verts[p->verts[0] * 3];
			dtVcopy(bmin, v);  // コピー
			dtVcopy(bmax, v);  // コピー

			for (int j = 1; j < p->vertCount; ++j)
			{
				v = &tile->verts[p->verts[j] * 3];
				dtVmin(bmin, v);
				dtVmax(bmax, v);
			}
			if (dtOverlapBounds(qmin, qmax, bmin, bmax))
			{
				polyRefs[n] = ref;
				polys[n] = p;

				if (n == batchSize - 1)
				{
					query->process(tile, polys, polyRefs, batchSize);
					n = 0;
				}
				else
				{
					n++;
				}
			}
		}
	}

	// Process the last polygons that didn't make a full batch.
	// 完全なバッチを作成しなかった最後のポリゴンを処理します。
	if (n > 0)
		query->process(tile, polys, polyRefs, n);
}

class dtCollectPolysQuery : public dtPolyQuery
{
	dtPolyRef* m_polys;
	const int m_maxPolys;
	int m_numCollected;
	bool m_overflow;

public:
	dtCollectPolysQuery(dtPolyRef* polys, const int maxPolys)
		: m_polys(polys), m_maxPolys(maxPolys), m_numCollected(0), m_overflow(false)
	{
	}

	int numCollected() const { return m_numCollected; }
	bool overflowed() const { return m_overflow; }

	void process(const dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count)
	{
		dtIgnoreUnused(tile);
		dtIgnoreUnused(polys);

		int numLeft = m_maxPolys - m_numCollected;
		int toCopy = count;
		if (toCopy > numLeft)
		{
			m_overflow = true;
			toCopy = numLeft;
		}

		memcpy(m_polys + m_numCollected, refs, (size_t)toCopy * sizeof(dtPolyRef));
		m_numCollected += toCopy;
	}
};

// @par
//
// If no polygons are found, the function will return #DT_SUCCESS with a
// @p polyCount of zero.
// ポリゴンが見つからない場合、関数はゼロの@p polyCountで#DT_SUCCESSを返します。
//
// If @p polys is too small to hold the entire result set, then the array will
// be filled to capacity. The method of choosing which polygons from the
// full set are included in the partial result set is undefined.
// @p polysが小さすぎて結果セット全体を保持できない場合、配列は容量いっぱいになります。
// 完全なセットからどのポリゴンを部分的な結果セットに含めるかを選択する方法は未定義です。
//
dtStatus dtNavMeshQuery::queryPolygons(const float* center, const float* extents,
	const dtQueryFilter* filter,
	dtPolyRef* polys, int* polyCount, const int maxPolys) const
{
	if (!polys || !polyCount || maxPolys < 0)
		return DT_FAILURE | DT_INVALID_PARAM;

	dtCollectPolysQuery collector(polys, maxPolys);

	dtStatus status = queryPolygons(center, extents, filter, &collector);
	if (dtStatusFailed(status))
		return status;

	*polyCount = collector.numCollected();
	return collector.overflowed() ? DT_SUCCESS | DT_BUFFER_TOO_SMALL : DT_SUCCESS;
}

// @par
//
// The query will be invoked with batches of polygons. Polygons passed
// to the query have bounding boxes that overlap with the center and extents
// passed to this function. The dtPolyQuery::process function is invoked multiple
// times until all overlapping polygons have been processed.
// クエリは、ポリゴンのバッチで呼び出されます。
// クエリに渡されるポリゴンには、この関数に渡される中心および範囲と重複する境界ボックスがあります。
// dtPolyQuery::process関数は、重複するすべてのポリゴンが処理されるまで複数回呼び出されます。
//
dtStatus dtNavMeshQuery::queryPolygons(const float* center, const float* extents,
	const dtQueryFilter* filter, dtPolyQuery* query) const
{
	dtAssert(m_nav);

	if (!center || !extents || !filter || !query)
		return DT_FAILURE | DT_INVALID_PARAM;

	float bmin[3], bmax[3];
	dtVsub(bmin, center, extents);
	dtVadd(bmax, center, extents);

	// Find tiles the query touches.
	// クエリが触れるタイルを見つけます。
	int minx, miny, maxx, maxy;
	m_nav->calcTileLoc(bmin, &minx, &miny);
	m_nav->calcTileLoc(bmax, &maxx, &maxy);

	constexpr int MAX_NEIS = 32;
	const dtMeshTile* neis[MAX_NEIS];

	for (int y = miny; y <= maxy; ++y)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const int nneis = m_nav->getTilesAt(x, y, neis, MAX_NEIS);

			for (int j = 0; j < nneis; ++j)
			{
				queryPolygonsInTile(neis[j], bmin, bmax, filter, query);
			}
		}
	}

	return DT_SUCCESS;
}

// @par
//
// If the end polygon cannot be reached through the navigation graph,
// the last polygon in the path will be the nearest the end polygon.
// ナビゲーショングラフから終了ポリゴンに到達できない場合、
//  パス内の最後のポリゴンが終了ポリゴンに最も近くなります。
//
// If the path array is to small to hold the full result, it will be filled as
// far as possible from the start polygon toward the end polygon.
// 完全な結果を保持するためにパス配列が小さすぎる場合、
// 開始ポリゴンから終了ポリゴンに向かって可能な限り埋められます。
//
// The start and end positions are used to calculate traversal costs.
// (The y-values impact the result.)
// 開始位置と終了位置は、走査コストの計算に使用されます。 （y値は結果に影響します。）
//
// 実際に経路探索を行っている部分
dtStatus dtNavMeshQuery::findPath(dtPolyRef startRef, dtPolyRef endRef,
	const float* startPos, const float* endPos,
	const dtQueryFilter* filter,
	dtPolyRef* path, int* pathCount, const int maxPath) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	if (pathCount)
		*pathCount = 0;

	// Validate input
	// 入力を検証する (ポリゴン参照の有効性を確認)
	if (!m_nav->isValidPolyRef(startRef) || !m_nav->isValidPolyRef(endRef) ||
		!startPos || !endPos || !filter || maxPath <= 0 || !path || !pathCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	// スタート地点とゴール地点が同じ場所
	if (startRef == endRef)
	{
		// パスをスタート地点に設定する
		path[0] = startRef;
		*pathCount = 1;
		return DT_SUCCESS;
	}

	// ノードプールとノードリストの初期化
	m_nodePool->clear();
	m_openList->clear();

	// ノードの取得
	dtNode* startNode = m_nodePool->getNode(startRef);

	dtVcopy(startNode->pos, startPos);  // コピー

	// 追加するノードへの初期設定
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = dtVdist(startPos, endPos) * H_SCALE;  // 引算
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;

	// リストにスタート地点を追加
	m_openList->push(startNode);

	// 経路探索前の初期設定

	dtNode* lastBestNode = startNode;
	float lastBestNodeCost = startNode->total;

	bool outOfNodes{ false };

	// 開かれたリストが無くなるまで実行
	while (!m_openList->empty())
	{
		// Remove node from open list and put it in closed list.
		// 開いているリストからノードを削除し、閉じたリストに入れます。
		dtNode* bestNode{ m_openList->pop() };

		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;

		// Reached the goal, stop searching.
		// 目標に到達し、検索を停止します。
		if (bestNode->id == endRef)
		{
			lastBestNode = bestNode;
			break;
		}

		// Get current poly and tile.
		// The API input has been cheked already, skip checking internal data.
		// 現在のポリゴンとタイルを取得します。
		// API入力は既にチェックされており、内部データのチェックをスキップします。
		const dtPolyRef bestRef{ bestNode->id };
		const dtMeshTile* bestTile{ nullptr };
		const dtPoly* bestPoly{ nullptr };

		// 指定されたポリゴン参照のタイルとポリゴンを返します。
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);

		// Get parent poly and tile.
		// 親ポリゴンとタイルを取得します。
		dtPolyRef parentRef{};
		const dtMeshTile* parentTile{ nullptr };
		const dtPoly* parentPoly{ nullptr };

		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;

		// 指定されたポリゴン参照のタイルとポリゴンを取得
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);

		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			dtPolyRef neighbourRef{ bestTile->links[i].ref };

			// Skip invalid ids and do not expand back to where we came from.
			// 無効なIDをスキップし、元の場所に展開しないでください。
			if (!neighbourRef || neighbourRef == parentRef)
				continue;

			// Get neighbour poly and tile.
			// The API input has been cheked already, skip checking internal data.
			// 隣のポリゴンとタイルを取得します。
			// API入力は既にチェックされており、内部データのチェックをスキップします。
			const dtMeshTile* neighbourTile{ nullptr };
			const dtPoly* neighbourPoly{ nullptr };

			// 指定されたポリゴン参照のタイルとポリゴンを取得
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);

			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;

			// deal explicitly with crossing tile boundaries
			// 交差するタイル境界を明示的に処理する
			unsigned char crossSide = 0;
			if (bestTile->links[i].side != 0xff)
				crossSide = bestTile->links[i].side >> 1;

			// ノードを取得する
			dtNode* neighbourNode{ m_nodePool->getNode(neighbourRef, crossSide) };

			if (!neighbourNode)
			{
				outOfNodes = true;
				continue;
			}

			// If the node is visited the first time, calculate node position.
			// ノードに初めてアクセスした場合、ノードの位置を計算します。
			if (neighbourNode->flags == 0)
			{
				// 2つのポリゴン間のエッジの中点を取得
				getEdgeMidPoint(bestRef, bestPoly, bestTile,
					neighbourRef, neighbourPoly, neighbourTile,
					neighbourNode->pos);
			}

			// Calculate cost and heuristic.
			// コストとヒューリスティックを計算します。
			float cost{ 0.f };
			float heuristic{ 0.f };

			// Special case for last node.
			// 最後のノードの特別な場合
			if (neighbourRef == endRef)
			{
				// Cost
				const float curCost = filter->getCost(bestNode->pos, neighbourNode->pos,
					parentRef, parentTile, parentPoly,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly);

				const float endCost = filter->getCost(neighbourNode->pos, endPos,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly,
					0, 0, 0);

				cost = bestNode->cost + curCost + endCost;
				heuristic = 0;
			}
			else
			{
				// Cost
				const float curCost = filter->getCost(bestNode->pos, neighbourNode->pos,
					parentRef, parentTile, parentPoly,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly);
				cost = bestNode->cost + curCost;
				heuristic = dtVdist(neighbourNode->pos, endPos) * H_SCALE;
			}

			const float total = cost + heuristic;

			// The node is already in open list and the new result is worse, skip.
			// ノードは既にオープンリストにあり、新しい結果はさらに悪いので、スキップします。
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;

			// The node is already visited and process, and the new result is worse, skip.
			// ノードはすでにアクセスされて処理されており、新しい結果はさらに悪いのでスキップします。
			if ((neighbourNode->flags & DT_NODE_CLOSED) && total >= neighbourNode->total)
				continue;

			// Add or update the node.
			// ノードを追加または更新します。
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->cost = cost;
			neighbourNode->total = total;

			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				// Already in open, update node location.
				// 既に開いており、ノードの場所を更新
				m_openList->modify(neighbourNode);
			}
			else
			{
				// Put the node in open list.
				// ノードをオープンリストに配置
				neighbourNode->flags |= DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}

			// Update nearest node to target so far.
			// これまでのターゲットに最も近いノードを更新
			if (heuristic < lastBestNodeCost)
			{
				lastBestNodeCost = heuristic;
				lastBestNode = neighbourNode;
			}
		}
	}

	// 指定された終了ノードへのパスを取得
	dtStatus status{ getPathToNode(lastBestNode, path, pathCount, maxPath) };

	// ゴールまでパスを発見できず、最も最善のパスを返す
	if (lastBestNode->id != endRef)
		status |= DT_PARTIAL_RESULT;

	if (outOfNodes)
		status |= DT_OUT_OF_NODES;

	return status;
}

// 指定された終了ノードへのパスを取得します。
dtStatus dtNavMeshQuery::getPathToNode(dtNode* endNode, dtPolyRef* path, int* pathCount, int maxPath) const
{
	// Find the length of the entire path.
	// パス全体の長さを見つけます。
	dtNode* curNode = endNode;
	int length = 0;
	do
	{
		length++;
		curNode = m_nodePool->getNodeAtIdx(curNode->pidx);
	} while (curNode);

	// If the path cannot be fully stored then advance to the last node we will be able to store.
	// パスを完全に保存できない場合は、保存できる最後のノードに進みます。
	curNode = endNode;
	int writeCount;
	for (writeCount = length; writeCount > maxPath; writeCount--)
	{
		dtAssert(curNode);

		curNode = m_nodePool->getNodeAtIdx(curNode->pidx);
	}

	// Write path
	// パスを書き込む
	for (int i = writeCount - 1; i >= 0; i--)
	{
		dtAssert(curNode);

		path[i] = curNode->id;
		curNode = m_nodePool->getNodeAtIdx(curNode->pidx);
	}

	dtAssert(!curNode);

	*pathCount = dtMin(length, maxPath);

	if (length > maxPath)
		return DT_SUCCESS | DT_BUFFER_TOO_SMALL;

	return DT_SUCCESS;
}

// @par
//
// @warning Calling any non-slice methods before calling finalizeSlicedFindPath()
// or finalizeSlicedFindPathPartial() may result in corrupted data!
// @warning finalizeSlicedFindPath（）またはfinalizeSlicedFindPathPartial（）を
// 呼び出す前に非スライスメソッドを呼び出すと、データが破損する可能性があります。
//
// The @p filter pointer is stored and used for the duration of the sliced
// path query.
// @pフィルターポインターは保存され、スライスパスクエリの実行中に使用されます。
//
dtStatus dtNavMeshQuery::initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
	const float* startPos, const float* endPos,
	const dtQueryFilter* filter, const unsigned int options)
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	// Init path state.
	// パス状態を初期化する
	memset(&m_query, 0, sizeof(dtQueryData));
	m_query.status = DT_FAILURE;
	m_query.startRef = startRef;
	m_query.endRef = endRef;
	dtVcopy(m_query.startPos.data(), startPos); // コピー
	dtVcopy(m_query.endPos.data(), endPos);     // コピー
	m_query.filter = filter;
	m_query.options = options;
	m_query.raycastLimitSqr = (std::numeric_limits<float>::max)();

	if (!startRef || !endRef)
		return DT_FAILURE | DT_INVALID_PARAM;

	// Validate input
	// 入力を検証する
	if (!m_nav->isValidPolyRef(startRef) || !m_nav->isValidPolyRef(endRef))
		return DT_FAILURE | DT_INVALID_PARAM;

	// trade quality with performance?
	// 品質とパフォーマンスのトレード？
	if (options & DT_FINDPATH_ANY_ANGLE)
	{
		// limiting to several times the character radius yields nice results. It is not sensitive
		// so it is enough to compute it from the first tile.
		// 文字の半径を数倍に制限すると、素晴らしい結果が得られます。
		// 敏感ではないので、最初のタイルから計算すれば十分です。
		const dtMeshTile* tile = m_nav->getTileByRef(startRef);
		float agentRadius = tile->header->walkableRadius;
		m_query.raycastLimitSqr = dtSqr(agentRadius * DT_RAY_CAST_LIMIT_PROPORTIONS);
	}

	if (startRef == endRef)
	{
		m_query.status = DT_SUCCESS;
		return DT_SUCCESS;
	}

	m_nodePool->clear();
	m_openList->clear();

	dtNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, startPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = dtVdist(startPos, endPos) * H_SCALE;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);

	m_query.status = DT_IN_PROGRESS;
	m_query.lastBestNode = startNode;
	m_query.lastBestNodeCost = startNode->total;

	return m_query.status;
}

dtStatus dtNavMeshQuery::updateSlicedFindPath(const int maxIter, int* doneIters)
{
	if (!dtStatusInProgress(m_query.status))
		return m_query.status;

	// Make sure the request is still valid.
	// リクエストがまだ有効であることを確認します。
	if (!m_nav->isValidPolyRef(m_query.startRef) || !m_nav->isValidPolyRef(m_query.endRef))
	{
		m_query.status = DT_FAILURE;
		return DT_FAILURE;
	}

	dtRaycastHit rayHit;
	rayHit.maxPath = 0;

	int iter{};
	while (iter < maxIter && !m_openList->empty())
	{
		iter++;

		// Remove node from open list and put it in closed list.
		// オープンリストからノードを削除し、クローズリストに配置します。
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;

		// Reached the goal, stop searching.
		// 目標に到達し、検索を停止します。
		if (bestNode->id == m_query.endRef)
		{
			m_query.lastBestNode = bestNode;
			const dtStatus details = m_query.status & DT_STATUS_DETAIL_MASK;
			m_query.status = DT_SUCCESS | details;

			if (doneIters) *doneIters = iter;

			return m_query.status;
		}

		// Get current poly and tile.
		// 現在のポリゴンとタイルを取得します。
		// The API input has been cheked already, skip checking internal data.
		// API入力は既にチェックされており、内部データのチェックをスキップします。
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile{};
		const dtPoly* bestPoly{};

		if (dtStatusFailed(m_nav->getTileAndPolyByRef(bestRef, &bestTile, &bestPoly)))
		{
			// The polygon has disappeared during the sliced query, fail.
			// スライスされたクエリ中にポリゴンが消えた、失敗。
			m_query.status = DT_FAILURE;

			if (doneIters) *doneIters = iter;

			return m_query.status;
		}

		// Get parent and grand parent poly and tile.
		// 親および親のポリゴンとタイルを取得します。
		dtPolyRef parentRef{}, grandpaRef{};
		const dtMeshTile* parentTile{};
		const dtPoly* parentPoly{};
		dtNode* parentNode{};

		if (bestNode->pidx)
		{
			parentNode = m_nodePool->getNodeAtIdx(bestNode->pidx);
			parentRef = parentNode->id;

			if (parentNode->pidx)
				grandpaRef = m_nodePool->getNodeAtIdx(parentNode->pidx)->id;
		}

		if (parentRef)
		{
			bool invalidParent = dtStatusFailed(m_nav->getTileAndPolyByRef(parentRef, &parentTile, &parentPoly));

			if (invalidParent || (grandpaRef && !m_nav->isValidPolyRef(grandpaRef)))
			{
				// The polygon has disappeared during the sliced query, fail.
				// スライスされたクエリ中にポリゴンが消えた、失敗。
				m_query.status = DT_FAILURE;

				if (doneIters) *doneIters = iter;

				return m_query.status;
			}
		}

		// decide whether to test raycast to previous nodes
		// 前のノードへのレイキャストをテストするかどうかを決定します
		bool tryLOS = false;

		if (m_query.options & DT_FINDPATH_ANY_ANGLE)
		{
			if ((parentRef != 0) && (dtVdistSqr(parentNode->pos, bestNode->pos) < m_query.raycastLimitSqr))
				tryLOS = true;
		}

		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			dtPolyRef neighbourRef = bestTile->links[i].ref;

			// Skip invalid ids and do not expand back to where we came from.
			// 無効なIDをスキップし、元の場所に展開しません。
			if (!neighbourRef || neighbourRef == parentRef)
				continue;

			// Get neighbour poly and tile.
			// 隣接するポリゴンとタイルを取得します。
			// The API input has been cheked already, skip checking internal data.
			// API入力は既にチェックされており、内部データのチェックをスキップします。
			const dtMeshTile* neighbourTile{};
			const dtPoly* neighbourPoly{};

			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);

			if (!m_query.filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;

			// get the neighbor node
			// 隣接ノードを取得します
			dtNode* neighbourNode = m_nodePool->getNode(neighbourRef, 0);
			if (!neighbourNode)
			{
				m_query.status |= DT_OUT_OF_NODES;
				continue;
			}

			// do not expand to nodes that were already visited from the same parent
			// 同じ親から既にアクセスされたノードには展開しません
			if (neighbourNode->pidx != 0 && neighbourNode->pidx == bestNode->pidx)
				continue;

			// If the node is visited the first time, calculate node position.
			// ノードに初めてアクセスした場合、ノードの位置を計算します。
			if (neighbourNode->flags == 0)
			{
				getEdgeMidPoint(bestRef, bestPoly, bestTile,
					neighbourRef, neighbourPoly, neighbourTile,
					neighbourNode->pos);
			}

			// Calculate cost and heuristic.
			// コストとヒューリスティックを計算します。
			float cost{};
			float heuristic{};

			// raycast parent
			// レイキャストの親
			bool foundShortCut = false;
			rayHit.pathCost = rayHit.t = 0;

			if (tryLOS)
			{
				raycast(parentRef, parentNode->pos, neighbourNode->pos, m_query.filter, DT_RAYCAST_USE_COSTS, &rayHit, grandpaRef);
				foundShortCut = rayHit.t >= 1.f;
			}

			// update move cost
			// 移動コストを更新します
			if (foundShortCut)
			{
				// shortcut found using raycast. Using shorter cost instead
				// raycastを使用して見つかったショートカット。代わりに短いコストを使用する
				cost = parentNode->cost + rayHit.pathCost;
			}
			else
			{
				// No shortcut found.
				// ショートカットが見つかりません。
				const float curCost = m_query.filter->getCost(bestNode->pos, neighbourNode->pos,
					parentRef, parentTile, parentPoly,
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly);
				cost = bestNode->cost + curCost;
			}

			// Special case for last node.
			// 最後のノードの特別な場合。
			if (neighbourRef == m_query.endRef)
			{
				const float endCost = m_query.filter->getCost(neighbourNode->pos, m_query.endPos.data(),
					bestRef, bestTile, bestPoly,
					neighbourRef, neighbourTile, neighbourPoly,
					0, 0, 0);

				cost = cost + endCost;
				heuristic = 0;
			}
			else
			{
				heuristic = dtVdist(neighbourNode->pos, m_query.endPos.data()) * H_SCALE;
			}

			const float total = cost + heuristic;

			// The node is already in open list and the new result is worse, skip.
			// ノードは既にオープンリストにあり、新しい結果はさらに悪いので、スキップします。
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;

			// The node is already visited and process, and the new result is worse, skip.
			// ノードはすでにアクセスされて処理されており、新しい結果はさらに悪いのでスキップします。
			if ((neighbourNode->flags & DT_NODE_CLOSED) && total >= neighbourNode->total)
				continue;

			// Add or update the node.
			// ノードを追加または更新します。
			neighbourNode->pidx = foundShortCut ? bestNode->pidx : m_nodePool->getNodeIdx(bestNode);
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~(DT_NODE_CLOSED | DT_NODE_PARENT_DETACHED));
			neighbourNode->cost = cost;
			neighbourNode->total = total;

			if (foundShortCut)
				neighbourNode->flags = (neighbourNode->flags | DT_NODE_PARENT_DETACHED);

			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				// Already in open, update node location.
				// 既に開いており、ノードの場所を更新します。
				m_openList->modify(neighbourNode);
			}
			else
			{
				// Put the node in open list.
				// ノードをオープンリストに配置します。
				neighbourNode->flags |= DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}

			// Update nearest node to target so far.
			// これまでのターゲットに最も近いノードを更新します。
			if (heuristic < m_query.lastBestNodeCost)
			{
				m_query.lastBestNodeCost = heuristic;
				m_query.lastBestNode = neighbourNode;
			}
		}
	}

	// Exhausted all nodes, but could not find path.
	// これまでのターゲットに最も近いノードを更新します。
	if (m_openList->empty())
	{
		const dtStatus details = m_query.status & DT_STATUS_DETAIL_MASK;
		m_query.status = DT_SUCCESS | details;
	}

	if (doneIters)
		*doneIters = iter;

	return m_query.status;
}

dtStatus dtNavMeshQuery::finalizeSlicedFindPath(dtPolyRef* path, int* pathCount, const int maxPath)
{
	*pathCount = 0;

	if (dtStatusFailed(m_query.status))
	{
		// Reset query. // クエリをリセットします。
		memset(&m_query, 0, sizeof(dtQueryData));
		return DT_FAILURE;
	}

	int n = 0;

	if (m_query.startRef == m_query.endRef)
	{
		// Special case: the search starts and ends at same poly.
		// 特別な場合：検索は同じポリゴンで開始および終了します。
		path[n++] = m_query.startRef;
	}
	else
	{
		// Reverse the path.
		// パスを逆にします。
		dtAssert(m_query.lastBestNode);

		if (m_query.lastBestNode->id != m_query.endRef)
			m_query.status |= DT_PARTIAL_RESULT;

		dtNode* prev{};
		dtNode* node = m_query.lastBestNode;
		int prevRay{};

		do
		{
			dtNode* next = m_nodePool->getNodeAtIdx(node->pidx);
			node->pidx = m_nodePool->getNodeIdx(prev);
			prev = node;

			// keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
			// 親が隣接していないかどうかを追跡します（つまり、レイキャストショートカットによる）
			int nextRay = node->flags & DT_NODE_PARENT_DETACHED;

			// and store it in the reversed path's node
			// そして、逆のパスのノードに保存します
			node->flags = (node->flags & ~DT_NODE_PARENT_DETACHED) | prevRay;
			prevRay = nextRay;
			node = next;
		} while (node);

		// Store path // パスを保存します
		node = prev;
		do
		{
			dtNode* next = m_nodePool->getNodeAtIdx(node->pidx);
			dtStatus status{};

			if (node->flags & DT_NODE_PARENT_DETACHED)
			{
				float t{}, normal[3]{};
				int m{};

				status = raycast(node->id, node->pos, next->pos, m_query.filter, &t, normal, path + n, &m, maxPath - n);
				n += m;

				// raycast ends on poly boundary and the path might include the next poly boundary.
				//レイキャストはポリ境界で終了し、パスには次のポリ境界が含まれる場合があります。
				if (path[n - 1] == next->id)
					n--; // remove to avoid duplicates // 重複を避けるために削除します
			}
			else
			{
				path[n++] = node->id;
				if (n >= maxPath)
					status = DT_BUFFER_TOO_SMALL;
			}

			if (status & DT_STATUS_DETAIL_MASK)
			{
				m_query.status |= status & DT_STATUS_DETAIL_MASK;
				break;
			}
			node = next;
		} while (node);
	}

	const dtStatus details = m_query.status & DT_STATUS_DETAIL_MASK;

	// Reset query. // クエリをリセットします。
	memset(&m_query, 0, sizeof(dtQueryData));

	*pathCount = n;

	return DT_SUCCESS | details;
}

dtStatus dtNavMeshQuery::finalizeSlicedFindPathPartial(const dtPolyRef* existing, const int existingSize,
	dtPolyRef* path, int* pathCount, const int maxPath)
{
	*pathCount = 0;

	if (existingSize == 0)
	{
		return DT_FAILURE;
	}

	if (dtStatusFailed(m_query.status))
	{
		// Reset query.
		memset(&m_query, 0, sizeof(dtQueryData));
		return DT_FAILURE;
	}

	int n = 0;

	if (m_query.startRef == m_query.endRef)
	{
		// Special case: the search starts and ends at same poly.
		path[n++] = m_query.startRef;
	}
	else
	{
		// Find furthest existing node that was visited.
		dtNode* prev = 0;
		dtNode* node = 0;
		for (int i = existingSize - 1; i >= 0; --i)
		{
			m_nodePool->findNodes(existing[i], &node, 1);
			if (node)
				break;
		}

		if (!node)
		{
			m_query.status |= DT_PARTIAL_RESULT;
			dtAssert(m_query.lastBestNode);
			node = m_query.lastBestNode;
		}

		// Reverse the path.
		int prevRay = 0;
		do
		{
			dtNode* next = m_nodePool->getNodeAtIdx(node->pidx);
			node->pidx = m_nodePool->getNodeIdx(prev);
			prev = node;
			int nextRay = node->flags & DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
			node->flags = (node->flags & ~DT_NODE_PARENT_DETACHED) | prevRay; // and store it in the reversed path's node
			prevRay = nextRay;
			node = next;
		} while (node);

		// Store path
		node = prev;
		do
		{
			dtNode* next = m_nodePool->getNodeAtIdx(node->pidx);
			dtStatus status = 0;
			if (node->flags & DT_NODE_PARENT_DETACHED)
			{
				float t, normal[3];
				int m;
				status = raycast(node->id, node->pos, next->pos, m_query.filter, &t, normal, path + n, &m, maxPath - n);
				n += m;
				// raycast ends on poly boundary and the path might include the next poly boundary.
				if (path[n - 1] == next->id)
					n--; // remove to avoid duplicates
			}
			else
			{
				path[n++] = node->id;
				if (n >= maxPath)
					status = DT_BUFFER_TOO_SMALL;
			}

			if (status & DT_STATUS_DETAIL_MASK)
			{
				m_query.status |= status & DT_STATUS_DETAIL_MASK;
				break;
			}
			node = next;
		} while (node);
	}

	const dtStatus details = m_query.status & DT_STATUS_DETAIL_MASK;

	// Reset query.
	memset(&m_query, 0, sizeof(dtQueryData));

	*pathCount = n;

	return DT_SUCCESS | details;
}

dtStatus dtNavMeshQuery::appendVertex(const float* pos, const unsigned char flags, const dtPolyRef ref,
	float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
	int* straightPathCount, const int maxStraightPath) const
{
	if ((*straightPathCount) > 0 && dtVequal(&straightPath[((*straightPathCount) - 1) * 3], pos))
	{
		// The vertices are equal, update flags and poly.
		if (straightPathFlags)
			straightPathFlags[(*straightPathCount) - 1] = flags;
		if (straightPathRefs)
			straightPathRefs[(*straightPathCount) - 1] = ref;
	}
	else
	{
		// Append new vertex.
		dtVcopy(&straightPath[(*straightPathCount) * 3], pos);
		if (straightPathFlags)
			straightPathFlags[(*straightPathCount)] = flags;
		if (straightPathRefs)
			straightPathRefs[(*straightPathCount)] = ref;
		(*straightPathCount)++;

		// If there is no space to append more vertices, return.
		if ((*straightPathCount) >= maxStraightPath)
		{
			return DT_SUCCESS | DT_BUFFER_TOO_SMALL;
		}

		// If reached end of path, return.
		if (flags == DT_STRAIGHTPATH_END)
		{
			return DT_SUCCESS;
		}
	}
	return DT_IN_PROGRESS;
}

dtStatus dtNavMeshQuery::appendPortals(const int startIdx, const int endIdx, const float* endPos, const dtPolyRef* path,
	float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
	int* straightPathCount, const int maxStraightPath, const int options) const
{
	const float* startPos = &straightPath[(*straightPathCount - 1) * 3];
	// Append or update last vertex
	dtStatus stat = 0;
	for (int i = startIdx; i < endIdx; i++)
	{
		// Calculate portal
		const dtPolyRef from = path[i];
		const dtMeshTile* fromTile = 0;
		const dtPoly* fromPoly = 0;
		if (dtStatusFailed(m_nav->getTileAndPolyByRef(from, &fromTile, &fromPoly)))
			return DT_FAILURE | DT_INVALID_PARAM;

		const dtPolyRef to = path[i + 1];
		const dtMeshTile* toTile = 0;
		const dtPoly* toPoly = 0;
		if (dtStatusFailed(m_nav->getTileAndPolyByRef(to, &toTile, &toPoly)))
			return DT_FAILURE | DT_INVALID_PARAM;

		float left[3], right[3];
		if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right)))
			break;

		if (options & DT_STRAIGHTPATH_AREA_CROSSINGS)
		{
			// Skip intersection if only area crossings are requested.
			if (fromPoly->getArea() == toPoly->getArea())
				continue;
		}

		// Append intersection
		float s, t;
		if (dtIntersectSegSeg2D(startPos, endPos, left, right, s, t))
		{
			float pt[3];
			dtVlerp(pt, left, right, t);

			stat = appendVertex(pt, 0, path[i + 1],
				straightPath, straightPathFlags, straightPathRefs,
				straightPathCount, maxStraightPath);
			if (stat != DT_IN_PROGRESS)
				return stat;
		}
	}
	return DT_IN_PROGRESS;
}

// @par
//
// This method peforms what is often called 'string pulling'.
// このメソッドは、「ストリングプル」と呼ばれることが多い方法を実行します。
//
// The start position is clamped to the first polygon in the path, and the
// end position is clamped to the last. So the start and end positions should
// normally be within or very near the first and last polygons respectively.
// 開始位置はパスの最初のポリゴンに固定され、終了位置は最後に固定されます。
// そのため、通常、開始位置と終了位置は、それぞれ最初と最後のポリゴン内または非常に近くにある必要があります。
//
// The returned polygon references represent the reference id of the polygon
// that is entered at the associated path position. The reference id associated
// with the end point will always be zero.  This allows, for example, matching
// off-mesh link points to their representative polygons.
// 返されるポリゴン参照は、関連付けられたパス位置に入力されたポリゴンの参照IDを表します。
// エンドポイントに関連付けられた参照IDは常にゼロになります。
// これにより、たとえば、メッシュ外のリンクポイントを代表的なポリゴンに一致させることができます。
//
// If the provided result buffers are too small for the entire result set,
// they will be filled as far as possible from the start toward the end
// position.
// 指定された結果バッファーが結果セット全体に対して小さすぎる場合、
// 開始位置から終了位置に向かって可能な限りいっぱいになります。
//
dtStatus dtNavMeshQuery::findStraightPath(const float* startPos, const float* endPos,
	const dtPolyRef* path, const int pathSize,
	float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
	int* straightPathCount, const int maxStraightPath, const int options) const
{
	dtAssert(m_nav);

	*straightPathCount = 0;

	if (!maxStraightPath)
		return DT_FAILURE | DT_INVALID_PARAM;

	if (!path[0])
		return DT_FAILURE | DT_INVALID_PARAM;

	dtStatus stat = 0;

	// TODO: Should this be callers responsibility?
	float closestStartPos[3];
	if (dtStatusFailed(closestPointOnPolyBoundary(path[0], startPos, closestStartPos)))
		return DT_FAILURE | DT_INVALID_PARAM;

	float closestEndPos[3];
	if (dtStatusFailed(closestPointOnPolyBoundary(path[pathSize - 1], endPos, closestEndPos)))
		return DT_FAILURE | DT_INVALID_PARAM;

	// Add start point.
	stat = appendVertex(closestStartPos, DT_STRAIGHTPATH_START, path[0],
		straightPath, straightPathFlags, straightPathRefs,
		straightPathCount, maxStraightPath);
	if (stat != DT_IN_PROGRESS)
		return stat;

	if (pathSize > 1)
	{
		float portalApex[3], portalLeft[3], portalRight[3];
		dtVcopy(portalApex, closestStartPos);
		dtVcopy(portalLeft, portalApex);
		dtVcopy(portalRight, portalApex);
		int apexIndex = 0;
		int leftIndex = 0;
		int rightIndex = 0;

		unsigned char leftPolyType = 0;
		unsigned char rightPolyType = 0;

		dtPolyRef leftPolyRef = path[0];
		dtPolyRef rightPolyRef = path[0];

		for (int i = 0; i < pathSize; ++i)
		{
			float left[3], right[3];
			unsigned char toType;

			if (i + 1 < pathSize)
			{
				unsigned char fromType; // fromType is ignored.

				// Next portal.
				if (dtStatusFailed(getPortalPoints(path[i], path[i + 1], left, right, fromType, toType)))
				{
					// Failed to get portal points, in practice this means that path[i+1] is invalid polygon.
					// Clamp the end point to path[i], and return the path so far.

					if (dtStatusFailed(closestPointOnPolyBoundary(path[i], endPos, closestEndPos)))
					{
						// This should only happen when the first polygon is invalid.
						return DT_FAILURE | DT_INVALID_PARAM;
					}

					// Apeend portals along the current straight path segment.
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
					{
						// Ignore status return value as we're just about to return anyway.
						appendPortals(apexIndex, i, closestEndPos, path,
							straightPath, straightPathFlags, straightPathRefs,
							straightPathCount, maxStraightPath, options);
					}

					// Ignore status return value as we're just about to return anyway.
					appendVertex(closestEndPos, 0, path[i],
						straightPath, straightPathFlags, straightPathRefs,
						straightPathCount, maxStraightPath);

					return DT_SUCCESS | DT_PARTIAL_RESULT | ((*straightPathCount >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
				}

				// If starting really close the portal, advance.
				if (i == 0)
				{
					float t;
					if (dtDistancePtSegSqr2D(portalApex, left, right, t) < dtSqr(0.001f))
						continue;
				}
			}
			else
			{
				// End of the path.
				dtVcopy(left, closestEndPos);
				dtVcopy(right, closestEndPos);

				toType = DT_POLYTYPE_GROUND;
			}

			// Right vertex.
			if (dtTriArea2D(portalApex, portalRight, right) <= 0.0f)
			{
				if (dtVequal(portalApex, portalRight) || dtTriArea2D(portalApex, portalLeft, right) > 0.0f)
				{
					dtVcopy(portalRight, right);
					rightPolyRef = (i + 1 < pathSize) ? path[i + 1] : 0;
					rightPolyType = toType;
					rightIndex = i;
				}
				else
				{
					// Append portals along the current straight path segment.
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
					{
						stat = appendPortals(apexIndex, leftIndex, portalLeft, path,
							straightPath, straightPathFlags, straightPathRefs,
							straightPathCount, maxStraightPath, options);
						if (stat != DT_IN_PROGRESS)
							return stat;
					}

					dtVcopy(portalApex, portalLeft);
					apexIndex = leftIndex;

					unsigned char flags = 0;
					if (!leftPolyRef)
						flags = DT_STRAIGHTPATH_END;
					else if (leftPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
						flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
					dtPolyRef ref = leftPolyRef;

					// Append or update vertex
					stat = appendVertex(portalApex, flags, ref,
						straightPath, straightPathFlags, straightPathRefs,
						straightPathCount, maxStraightPath);
					if (stat != DT_IN_PROGRESS)
						return stat;

					dtVcopy(portalLeft, portalApex);
					dtVcopy(portalRight, portalApex);
					leftIndex = apexIndex;
					rightIndex = apexIndex;

					// Restart
					i = apexIndex;

					continue;
				}
			}

			// Left vertex.
			if (dtTriArea2D(portalApex, portalLeft, left) >= 0.0f)
			{
				if (dtVequal(portalApex, portalLeft) || dtTriArea2D(portalApex, portalRight, left) < 0.0f)
				{
					dtVcopy(portalLeft, left);
					leftPolyRef = (i + 1 < pathSize) ? path[i + 1] : 0;
					leftPolyType = toType;
					leftIndex = i;
				}
				else
				{
					// Append portals along the current straight path segment.
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
					{
						stat = appendPortals(apexIndex, rightIndex, portalRight, path,
							straightPath, straightPathFlags, straightPathRefs,
							straightPathCount, maxStraightPath, options);
						if (stat != DT_IN_PROGRESS)
							return stat;
					}

					dtVcopy(portalApex, portalRight);
					apexIndex = rightIndex;

					unsigned char flags = 0;
					if (!rightPolyRef)
						flags = DT_STRAIGHTPATH_END;
					else if (rightPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
						flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
					dtPolyRef ref = rightPolyRef;

					// Append or update vertex
					stat = appendVertex(portalApex, flags, ref,
						straightPath, straightPathFlags, straightPathRefs,
						straightPathCount, maxStraightPath);
					if (stat != DT_IN_PROGRESS)
						return stat;

					dtVcopy(portalLeft, portalApex);
					dtVcopy(portalRight, portalApex);
					leftIndex = apexIndex;
					rightIndex = apexIndex;

					// Restart
					i = apexIndex;

					continue;
				}
			}
		}

		// Append portals along the current straight path segment.
		if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
		{
			stat = appendPortals(apexIndex, pathSize - 1, closestEndPos, path,
				straightPath, straightPathFlags, straightPathRefs,
				straightPathCount, maxStraightPath, options);
			if (stat != DT_IN_PROGRESS)
				return stat;
		}
	}

	// Ignore status return value as we're just about to return anyway.
	appendVertex(closestEndPos, DT_STRAIGHTPATH_END, 0,
		straightPath, straightPathFlags, straightPathRefs,
		straightPathCount, maxStraightPath);

	return DT_SUCCESS | ((*straightPathCount >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
}

// @par
//
// This method is optimized for small delta movement and a small number of
// polygons. If used for too great a distance, the result set will form an
// incomplete path.
// この方法は、小さなデルタ移動と少数のポリゴン用に最適化されています。
// 長すぎる距離を使用すると、結果セットは不完全なパスを形成します。
//
// @p resultPos will equal the @p endPos if the end is reached.
// Otherwise the closest reachable position will be returned.
// resultPosは、終了に達すると@p endPosと等しくなります。
// それ以外の場合、最も近い到達可能な位置が返されます。
//
// @p resultPos is not projected onto the surface of the navigation
// mesh. Use #getPolyHeight if this is needed.
// resultPosは、ナビゲーションメッシュの表面に投影されません。 これが必要な場合は#getPolyHeightを使用します。
//
// This method treats the end position in the same manner as
// the #raycast method. (As a 2D point.) See that method's documentation
// for details.
// このメソッドは、終了位置を#raycastメソッドと同じ方法で処理します。（2Dポイントとして）
// 詳細については、そのメソッドのドキュメントを参照してください。
//
// If the @p visited array is too small to hold the entire result set, it will
// be filled as far as possible from the start position toward the end
// position.
// 訪問された@p配列が小さすぎて結果セット全体を保持できない場合、可能な限り開始位置から終了位置に向かって埋められます。
//
dtStatus dtNavMeshQuery::moveAlongSurface(dtPolyRef startRef, const float* startPos, const float* endPos,
	const dtQueryFilter* filter,
	float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) const
{
	dtAssert(m_nav);
	dtAssert(m_tinyNodePool);

	*visitedCount = 0;

	// Validate input
	if (!startRef)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (!m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;

	dtStatus status = DT_SUCCESS;

	constexpr int MAX_STACK = 48;
	dtNode* stack[MAX_STACK];
	int nstack = 0;

	m_tinyNodePool->clear();

	dtNode* startNode = m_tinyNodePool->getNode(startRef);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_CLOSED;
	stack[nstack++] = startNode;

	float bestPos[3];
	float bestDist = (std::numeric_limits<float>::max)();
	dtNode* bestNode = 0;
	dtVcopy(bestPos, startPos);

	// Search constraints
	float searchPos[3], searchRadSqr;
	dtVlerp(searchPos, startPos, endPos, 0.5f);
	searchRadSqr = dtSqr(dtVdist(startPos, endPos) / 2.0f + 0.001f);

	float verts[DT_VERTS_PER_POLYGON * 3];

	while (nstack)
	{
		// Pop front.
		dtNode* curNode = stack[0];
		for (int i = 0; i < nstack - 1; ++i)
			stack[i] = stack[i + 1];
		nstack--;

		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef curRef = curNode->id;
		const dtMeshTile* curTile = 0;
		const dtPoly* curPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(curRef, &curTile, &curPoly);

		// Collect vertices.
		const int nverts = curPoly->vertCount;
		for (int i = 0; i < nverts; ++i)
			dtVcopy(&verts[i * 3], &curTile->verts[curPoly->verts[i] * 3]);

		// If target is inside the poly, stop search.
		if (dtPointInPolygon(endPos, verts, nverts))
		{
			bestNode = curNode;
			dtVcopy(bestPos, endPos);
			break;
		}

		// Find wall edges and find nearest point inside the walls.
		for (int i = 0, j = (int)curPoly->vertCount - 1; i < (int)curPoly->vertCount; j = i++)
		{
			// Find links to neighbours.
			constexpr int MAX_NEIS = 8;
			int nneis = 0;
			dtPolyRef neis[MAX_NEIS];

			if (curPoly->neis[j] & DT_EXT_LINK)
			{
				// Tile border.
				for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
				{
					const dtLink* link = &curTile->links[k];
					if (link->edge == j)
					{
						if (link->ref != 0)
						{
							const dtMeshTile* neiTile = 0;
							const dtPoly* neiPoly = 0;
							m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
							if (filter->passFilter(link->ref, neiTile, neiPoly))
							{
								if (nneis < MAX_NEIS)
									neis[nneis++] = link->ref;
							}
						}
					}
				}
			}
			else if (curPoly->neis[j])
			{
				const unsigned int idx = (unsigned int)(curPoly->neis[j] - 1);
				const dtPolyRef ref = m_nav->getPolyRefBase(curTile) | idx;
				if (filter->passFilter(ref, curTile, &curTile->polys[idx]))
				{
					// Internal edge, encode id.
					neis[nneis++] = ref;
				}
			}

			if (!nneis)
			{
				// Wall edge, calc distance.
				const float* vj = &verts[j * 3];
				const float* vi = &verts[i * 3];
				float tseg;
				const float distSqr = dtDistancePtSegSqr2D(endPos, vj, vi, tseg);
				if (distSqr < bestDist)
				{
					// Update nearest distance.
					dtVlerp(bestPos, vj, vi, tseg);
					bestDist = distSqr;
					bestNode = curNode;
				}
			}
			else
			{
				for (int k = 0; k < nneis; ++k)
				{
					// Skip if no node can be allocated.
					dtNode* neighbourNode = m_tinyNodePool->getNode(neis[k]);
					if (!neighbourNode)
						continue;
					// Skip if already visited.
					if (neighbourNode->flags & DT_NODE_CLOSED)
						continue;

					// Skip the link if it is too far from search constraint.
					// TODO: Maybe should use getPortalPoints(), but this one is way faster.
					const float* vj = &verts[j * 3];
					const float* vi = &verts[i * 3];
					float tseg;
					float distSqr = dtDistancePtSegSqr2D(searchPos, vj, vi, tseg);
					if (distSqr > searchRadSqr)
						continue;

					// Mark as the node as visited and push to queue.
					if (nstack < MAX_STACK)
					{
						neighbourNode->pidx = m_tinyNodePool->getNodeIdx(curNode);
						neighbourNode->flags |= DT_NODE_CLOSED;
						stack[nstack++] = neighbourNode;
					}
				}
			}
		}
	}

	int n = 0;
	if (bestNode)
	{
		// Reverse the path.
		dtNode* prev = 0;
		dtNode* node = bestNode;
		do
		{
			dtNode* next = m_tinyNodePool->getNodeAtIdx(node->pidx);
			node->pidx = m_tinyNodePool->getNodeIdx(prev);
			prev = node;
			node = next;
		} while (node);

		// Store result
		node = prev;
		do
		{
			visited[n++] = node->id;
			if (n >= maxVisitedSize)
			{
				status |= DT_BUFFER_TOO_SMALL;
				break;
			}
			node = m_tinyNodePool->getNodeAtIdx(node->pidx);
		} while (node);
	}

	dtVcopy(resultPos, bestPos);

	*visitedCount = n;

	return status;
}

dtStatus dtNavMeshQuery::getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right,
	unsigned char& fromType, unsigned char& toType) const
{
	dtAssert(m_nav);

	const dtMeshTile* fromTile = 0;
	const dtPoly* fromPoly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(from, &fromTile, &fromPoly)))
		return DT_FAILURE | DT_INVALID_PARAM;
	fromType = fromPoly->getType();

	const dtMeshTile* toTile = 0;
	const dtPoly* toPoly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(to, &toTile, &toPoly)))
		return DT_FAILURE | DT_INVALID_PARAM;
	toType = toPoly->getType();

	return getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right);
}

// Returns portal points between two polygons.
dtStatus dtNavMeshQuery::getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
	dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
	float* left, float* right) const
{
	// Find the link that points to the 'to' polygon.
	const dtLink* link = 0;
	for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
	{
		if (fromTile->links[i].ref == to)
		{
			link = &fromTile->links[i];
			break;
		}
	}
	if (!link)
		return DT_FAILURE | DT_INVALID_PARAM;

	// Handle off-mesh connections.
	if (fromPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		// Find link that points to first vertex.
		for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
		{
			if (fromTile->links[i].ref == to)
			{
				const int v = fromTile->links[i].edge;
				dtVcopy(left, &fromTile->verts[fromPoly->verts[v] * 3]);
				dtVcopy(right, &fromTile->verts[fromPoly->verts[v] * 3]);
				return DT_SUCCESS;
			}
		}
		return DT_FAILURE | DT_INVALID_PARAM;
	}

	if (toPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		for (unsigned int i = toPoly->firstLink; i != DT_NULL_LINK; i = toTile->links[i].next)
		{
			if (toTile->links[i].ref == from)
			{
				const int v = toTile->links[i].edge;
				dtVcopy(left, &toTile->verts[toPoly->verts[v] * 3]);
				dtVcopy(right, &toTile->verts[toPoly->verts[v] * 3]);
				return DT_SUCCESS;
			}
		}
		return DT_FAILURE | DT_INVALID_PARAM;
	}

	// Find portal vertices.
	const int v0 = fromPoly->verts[link->edge];
	const int v1 = fromPoly->verts[(link->edge + 1) % (int)fromPoly->vertCount];
	dtVcopy(left, &fromTile->verts[v0 * 3]);
	dtVcopy(right, &fromTile->verts[v1 * 3]);

	// If the link is at tile boundary, dtClamp the vertices to
	// the link width.
	if (link->side != 0xff)
	{
		// Unpack portal limits.
		if (link->bmin != 0 || link->bmax != 255)
		{
			const float s = 1.f / 255.0f;
			const float tmin = link->bmin * s;
			const float tmax = link->bmax * s;
			dtVlerp(left, &fromTile->verts[v0 * 3], &fromTile->verts[v1 * 3], tmin);
			dtVlerp(right, &fromTile->verts[v0 * 3], &fromTile->verts[v1 * 3], tmax);
		}
	}

	return DT_SUCCESS;
}

// Returns edge mid point between two polygons.
dtStatus dtNavMeshQuery::getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const
{
	float left[3], right[3];
	unsigned char fromType, toType;
	if (dtStatusFailed(getPortalPoints(from, to, left, right, fromType, toType)))
		return DT_FAILURE | DT_INVALID_PARAM;
	mid[0] = (left[0] + right[0]) * 0.5f;
	mid[1] = (left[1] + right[1]) * 0.5f;
	mid[2] = (left[2] + right[2]) * 0.5f;
	return DT_SUCCESS;
}

// 2つのポリゴン間のエッジの中点を返します。
dtStatus dtNavMeshQuery::getEdgeMidPoint(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
	dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
	float* mid) const
{
	float left[3], right[3];
	if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right)))
		return DT_FAILURE | DT_INVALID_PARAM;
	mid[0] = (left[0] + right[0]) * 0.5f;
	mid[1] = (left[1] + right[1]) * 0.5f;
	mid[2] = (left[2] + right[2]) * 0.5f;
	return DT_SUCCESS;
}

// @par
//
// This method is meant to be used for quick, short distance checks.
// このメソッドは、迅速な短距離チェックに使用することを目的としています。
//
// If the path array is too small to hold the result, it will be filled as
// far as possible from the start postion toward the end position.
// パス配列が小さすぎて結果を保持できない場合、可能な限り開始位置から終了位置に向かって埋められます。
//
// <b>Using the Hit Parameter ヒットパラメーターの使用 (t)</b>
//
// If the hit parameter is a very high value ((std::numeric_limits<float>::max)()), then the ray has hit
// the end position. In this case the path represents a valid corridor to the
// end position and the value of @p hitNormal is undefined.
// ヒットパラメーターが非常に高い値（(std::numeric_limits<float>::max)()）の場合、レイは終了位置にヒットしています。
// この場合、パスは終了位置への有効なコリドーを表し、@ p hitNormalの値は未定義です。
//
// If the hit parameter is zero, then the start position is on the wall that
// was hit and the value of @p hitNormal is undefined.
// ヒットパラメータがゼロの場合、開始位置はヒットした壁上にあり、@ p hitNormalの値は未定義です。
//
// If 0 < t < 1.0 then the following applies:
// 0 <t <1.0の場合、以下が適用されます。
//
// @code
// distanceToHitBorder = distanceToEndPosition * t
// hitPoint = startPos + (endPos - startPos) * t
// @endcode
//
// <b>Use Case Restriction ユースケースの制限 </b>
//
// The raycast ignores the y-value of the end position. (2D check.) This
// places significant limits on how it can be used. For example:
// レイキャストは終了位置のy値を無視します。 （2Dチェック）
// これは、その使用方法に大きな制限を課します。 例えば：
//
// Consider a scene where there is a main floor with a second floor balcony
// that hangs over the main floor. So the first floor mesh extends below the
// balcony mesh. The start position is somewhere on the first floor. The end
// position is on the balcony.
// メインフロアがあり、メインフロアに2階のバルコニーがかかっているシーンを考えます。
// そのため、1階メッシュはバルコニーメッシュの下に伸びています。
// 開始位置は1階のどこかにあります。 終了位置はバルコニーにあります。
//
// The raycast will search toward the end position along the first floor mesh.
// If it reaches the end position's xz-coordinates it will indicate (std::numeric_limits<float>::max)()
// (no wall hit), meaning it reached the end position. This is one example of why
// this method is meant for short distance checks.
// レイキャストは、1階メッシュに沿って終了位置に向かって検索します。
// 終了位置のxz座標に到達すると、(std::numeric_limits<float>::max)()（壁に衝突しない）、つまり終了位置に到達したことを示します。
// これは、この方法が短距離チェック用である理由の一例です。
//
dtStatus dtNavMeshQuery::raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
	const dtQueryFilter* filter,
	float* t, float* hitNormal, dtPolyRef* path, int* pathCount, const int maxPath) const
{
	dtRaycastHit hit;
	hit.path = path;
	hit.maxPath = maxPath;

	dtStatus status = raycast(startRef, startPos, endPos, filter, 0, &hit);

	*t = hit.t;
	if (hitNormal)
		dtVcopy(hitNormal, hit.hitNormal.data());
	if (pathCount)
		*pathCount = hit.pathCount;

	return status;
}

// @par
//
// This method is meant to be used for quick, short distance checks.
// このメソッドは、迅速な短距離チェックに使用することを目的としています。
//
// If the path array is too small to hold the result, it will be filled as
// far as possible from the start postion toward the end position.
// パス配列が小さすぎて結果を保持できない場合、可能な限り開始位置から終了位置に向かって埋められます。
//
// <b>Using the Hit Parameter t of RaycastHit ヒットパラメーターの使用 </b>
//
// If the hit parameter is a very high value ((std::numeric_limits<float>::max)()), then the ray has hit
// the end position. In this case the path represents a valid corridor to the
// end position and the value of @p hitNormal is undefined.
// ヒットパラメーターが非常に高い値（(std::numeric_limits<float>::max)()）の場合、レイは終了位置にヒットしています。
// この場合、パスは終了位置への有効なコリドーを表し、@ p hitNormalの値は未定義です。
//
// If the hit parameter is zero, then the start position is on the wall that
// was hit and the value of @p hitNormal is undefined.
// ヒットパラメータがゼロの場合、開始位置はヒットした壁上にあり、@ p hitNormalの値は未定義です。
//
// If 0 < t < 1.0 then the following applies:
// 0 <t <1.0の場合、以下が適用されます。
//
// @code
// distanceToHitBorder = distanceToEndPosition * t
// hitPoint = startPos + (endPos - startPos) * t
// @endcode
//
// <b>Use Case Restriction</b>
//
// The raycast ignores the y-value of the end position. (2D check.) This
// places significant limits on how it can be used. For example:
// レイキャストは終了位置のy値を無視します。 （2Dチェック）
// これは、その使用方法に大きな制限を課します。 例えば：
//
// Consider a scene where there is a main floor with a second floor balcony
// that hangs over the main floor. So the first floor mesh extends below the
// balcony mesh. The start position is somewhere on the first floor. The end
// position is on the balcony.
// メインフロアがあり、メインフロアに2階のバルコニーがかかっているシーンを考えます。
// そのため、1階メッシュはバルコニーメッシュの下に伸びています。
// 開始位置は1階のどこかにあります。 終了位置はバルコニーにあります。
//
// The raycast will search toward the end position along the first floor mesh.
// If it reaches the end position's xz-coordinates it will indicate (std::numeric_limits<float>::max)()
// (no wall hit), meaning it reached the end position. This is one example of why
// this method is meant for short distance checks.
// レイキャストは、1階メッシュに沿って終了位置に向かって検索します。
// 終了位置のxz座標に到達すると、(std::numeric_limits<float>::max)()（壁に衝突しない）、つまり終了位置に到達したことを示します。
// これは、この方法が短距離チェック用である理由の一例です。
//
dtStatus dtNavMeshQuery::raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
	const dtQueryFilter* filter, const unsigned int options,
	dtRaycastHit* hit, dtPolyRef prevRef) const
{
	dtAssert(m_nav);

	hit->t = 0;
	hit->pathCount = 0;
	hit->pathCost = 0;

	// Validate input // 入力を検証します
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;
	if (prevRef && !m_nav->isValidPolyRef(prevRef))
		return DT_FAILURE | DT_INVALID_PARAM;

	float dir[3], curPos[3], lastPos[3];
	float verts[DT_VERTS_PER_POLYGON * 3 + 3];
	int n = 0;

	dtVcopy(curPos, startPos);
	dtVsub(dir, endPos, startPos);
	dtVset(&hit->hitNormal, 0, 0, 0);

	dtStatus status = DT_SUCCESS;

	const dtMeshTile* prevTile, * tile, * nextTile;
	const dtPoly* prevPoly, * poly, * nextPoly;
	dtPolyRef curRef;

	// The API input has been checked already, skip checking internal data.
	// API入力は既にチェックされています。内部データのチェックをスキップします。
	curRef = startRef;
	tile = 0;
	poly = 0;
	m_nav->getTileAndPolyByRefUnsafe(curRef, &tile, &poly);
	nextTile = prevTile = tile;
	nextPoly = prevPoly = poly;
	if (prevRef)
		m_nav->getTileAndPolyByRefUnsafe(prevRef, &prevTile, &prevPoly);

	while (curRef)
	{
		// Cast ray against current polygon.
		// 現在のポリゴンに対してレイをキャストします。
		// Collect vertices.
		// 頂点を収集します。
		int nv = 0;
		for (int i = 0; i < (int)poly->vertCount; ++i)
		{
			dtVcopy(&verts[nv * 3], &tile->verts[poly->verts[i] * 3]);
			nv++;
		}

		float tmin, tmax;
		int segMin, segMax;
		if (!dtIntersectSegmentPoly2D(startPos, endPos, verts, nv, tmin, tmax, segMin, segMax))
		{
			// Could not hit the polygon, keep the old t and report hit.
			// ポリゴンをヒットできませんでした。古い「t」を保持し、レポートをヒットしました。
			hit->pathCount = n;
			return status;
		}

		hit->hitEdgeIndex = segMax;

		// Keep track of furthest t so far.
		// これまでの最も遠い「t」を追跡します。
		if (tmax > hit->t)
			hit->t = tmax;

		// Store visited polygons.
		// 訪問したポリゴンを保存します。
		if (n < hit->maxPath)
			hit->path[n++] = curRef;
		else
			status |= DT_BUFFER_TOO_SMALL;

		// Ray end is completely inside the polygon.
		// レイエンドは完全にポリゴン内にあります。
		if (segMax == -1)
		{
			hit->t = (std::numeric_limits<float>::max)();
			hit->pathCount = n;

			// コストを追加します
			if (options & DT_RAYCAST_USE_COSTS)
				hit->pathCost += filter->getCost(curPos, endPos, prevRef, prevTile, prevPoly, curRef, tile, poly, curRef, tile, poly);
			return status;
		}

		// Follow neighbours. // 隣人をフォローします。
		dtPolyRef nextRef = 0;

		for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
		{
			const dtLink* link = &tile->links[i];

			// Find link which contains this edge.
			// このエッジを含むリンクを見つけます。
			if ((int)link->edge != segMax)
				continue;

			// Get pointer to the next polygon.
			// 次のポリゴンへのポインターを取得します。
			nextTile = 0;
			nextPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(link->ref, &nextTile, &nextPoly);

			// Skip off-mesh connections.
			// オフメッシュ接続をスキップします。
			if (nextPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;

			// Skip links based on filter.
			// フィルタに基づいてリンクをスキップします。
			if (!filter->passFilter(link->ref, nextTile, nextPoly))
				continue;

			// If the link is internal, just return the ref.
			// リンクが内部リンクの場合、参照を返すだけです。
			if (link->side == 0xff)
			{
				nextRef = link->ref;
				break;
			}

			// If the link is at tile boundary, Check if the link spans the whole edge, and accept.
			// リンクがタイル境界にある場合、リンクがエッジ全体にまたがっているかどうかを確認し、応じます。
			if (link->bmin == 0 && link->bmax == 255)
			{
				nextRef = link->ref;
				break;
			}

			// Check for partial edge links.
			// 部分的なエッジリンクを確認します。
			const int v0 = poly->verts[link->edge];
			const int v1 = poly->verts[(link->edge + 1) % poly->vertCount];
			const float* left = &tile->verts[v0 * 3];
			const float* right = &tile->verts[v1 * 3];

			// Check that the intersection lies inside the link portal.
			// 交差点がリンクポータル内にあることを確認します。
			if (link->side == 0 || link->side == 4)
			{
				// Calculate link size.
				// リンクサイズを計算します。
				const float s = 1.f / 255.0f;
				float lmin = left[2] + (right[2] - left[2]) * (link->bmin * s);
				float lmax = left[2] + (right[2] - left[2]) * (link->bmax * s);
				if (lmin > lmax) dtSwap(lmin, lmax);

				// Find Z intersection.
				// Z交差点を見つけます。
				float z = startPos[2] + (endPos[2] - startPos[2]) * tmax;
				if (z >= lmin && z <= lmax)
				{
					nextRef = link->ref;
					break;
				}
			}
			else if (link->side == 2 || link->side == 6)
			{
				// Calculate link size.
				// リンクサイズを計算します。
				const float s = 1.f / 255.0f;
				float lmin = left[0] + (right[0] - left[0]) * (link->bmin * s);
				float lmax = left[0] + (right[0] - left[0]) * (link->bmax * s);
				if (lmin > lmax) dtSwap(lmin, lmax);

				// Find X intersection.
				// X交差点を見つけます。
				float x = startPos[0] + (endPos[0] - startPos[0]) * tmax;
				if (x >= lmin && x <= lmax)
				{
					nextRef = link->ref;
					break;
				}
			}
		}

		//コストを追加します
		if (options & DT_RAYCAST_USE_COSTS)
		{
			// compute the intersection point at the furthest end of the polygon
			// and correct the height (since the raycast moves in 2d)
			// ポリゴンの最も遠い端で交点を計算し、高さを修正します（レイキャストが2dで移動するため）
			dtVcopy(lastPos, curPos);
			dtVmad(curPos, startPos, dir, hit->t);

			float* e1 = &verts[segMax * 3];
			float* e2 = &verts[((segMax + 1) % nv) * 3];
			float eDir[3]{}, diff[3]{};

			dtVsub(eDir, e2, e1);
			dtVsub(diff, curPos, e1);

			float s = dtSqr(eDir[0]) > dtSqr(eDir[2]) ? diff[0] / eDir[0] : diff[2] / eDir[2];

			curPos[1] = e1[1] + eDir[1] * s;

			hit->pathCost += filter->getCost(lastPos, curPos, prevRef, prevTile, prevPoly, curRef, tile, poly, nextRef, nextTile, nextPoly);
		}

		if (!nextRef)
		{
			// No neighbour, we hit a wall.
			// 隣人はいません、壁にぶつかります。
			// Calculate hit normal.
			// 通常のヒットを計算します。
			const int a = segMax;
			const int b = segMax + 1 < nv ? segMax + 1 : 0;
			const float* va = &verts[a * 3];
			const float* vb = &verts[b * 3];
			const float dx = vb[0] - va[0];
			const float dz = vb[2] - va[2];
			dtVset(&hit->hitNormal, dz, 0.f, -dx);
			dtVnormalize(&hit->hitNormal);

			hit->pathCount = n;
			return status;
		}

		// No hit, advance to neighbour polygon.
		// ヒットせず、隣のポリゴンに進みます。
		prevRef = curRef;
		curRef = nextRef;
		prevTile = tile;
		tile = nextTile;
		prevPoly = poly;
		poly = nextPoly;
	}

	hit->pathCount = n;

	return status;
}

// @par
//
// At least one result array must be provided.
// 少なくとも1つの結果配列を提供する必要があります。
//
// The order of the result set is from least to highest cost to reach the polygon.
// 結果セットの順序は、ポリゴンに到達するための最小コストから最大コストです。
//
// A common use case for this method is to perform Dijkstra searches.
// Candidate polygons are found by searching the graph beginning at the start polygon.
// このメソッドの一般的な使用例は、ダイクストラ検索を実行することです。
// 候補ポリゴンは、開始ポリゴンから始まるグラフを検索することで見つけられます。
//
// If a polygon is not found via the graph search, even if it intersects the
// search circle, it will not be included in the result set. For example:
// グラフ検索で多角形が見つからない場合、たとえそれが交差しても
// 検索サークル、それは結果セットに含まれません。例えば：
//
// polyA is the start polygon.
// polyB shares an edge with polyA. (Is adjacent.)
// polyC shares an edge with polyB, but not with polyA
// Even if the search circle overlaps polyC, it will not be included in the
// result set unless polyB is also in the set.
// polyAは開始ポリゴンです。
// polyBはpolyAとエッジを共有します。 （隣接しています。）
// polyCはpolyBとエッジを共有しますが、polyAとは共有しません。
// 検索サークルがpolyCと重複しても、polyBもセットに含まれていない限り、結果セットには含まれません。
//
// The value of the center point is used as the start position for cost
// calculations. It is not projected onto the surface of the mesh, so its
// y-value will effect the costs.
// 中心点の値は、コスト計算の開始位置として使用されます。メッシュの表面には投影されないため、y値はコストに影響します。
//
// Intersection tests occur in 2D. All polygons and the search circle are
// projected onto the xz-plane. So the y-value of the center point does not
// effect intersection tests.
// 交差点テストは2Dで行われます。すべてのポリゴンと検索円は、xz平面に投影されます。
// したがって、中心点のy値は交差テストに影響しません。
//
// If the result arrays are to small to hold the entire result set, they will be
// filled to capacity.
// 結果セット全体を保持するために結果配列が小さすぎる場合、それらは容量がフルになります。
//
dtStatus dtNavMeshQuery::findPolysAroundCircle(dtPolyRef startRef, const float* centerPos, const float radius,
	const dtQueryFilter* filter,
	dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
	int* resultCount, const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	*resultCount = 0;

	// Validate input
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;

	m_nodePool->clear();
	m_openList->clear();

	dtNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);

	dtStatus status = DT_SUCCESS;

	int n = 0;

	const float radiusSqr = dtSqr(radius);

	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;

		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);

		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);

		if (n < maxResult)
		{
			if (resultRef)
				resultRef[n] = bestRef;
			if (resultParent)
				resultParent[n] = parentRef;
			if (resultCost)
				resultCost[n] = bestNode->total;
			++n;
		}
		else
		{
			status |= DT_BUFFER_TOO_SMALL;
		}

		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const dtLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;

			// Expand to neighbour
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);

			// Do not advance if the polygon is excluded by the filter.
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;

			// Find edge and calc distance to the edge.
			float va[3], vb[3];
			if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;

			// If the circle is not touching the next polygon, skip it.
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			if (distSqr > radiusSqr)
				continue;

			dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}

			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;

			// Cost
			if (neighbourNode->flags == 0)
				dtVlerp(neighbourNode->pos, va, vb, 0.5f);

			float cost = filter->getCost(
				bestNode->pos, neighbourNode->pos,
				parentRef, parentTile, parentPoly,
				bestRef, bestTile, bestPoly,
				neighbourRef, neighbourTile, neighbourPoly);

			const float total = bestNode->total + cost;

			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;

			neighbourNode->id = neighbourRef;
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;

			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				neighbourNode->flags = DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}

	*resultCount = n;

	return status;
}

// @par
//
// The order of the result set is from least to highest cost.
// 結果セットの順序は、最小から最大のコストです。
//
// At least one result array must be provided.
// 少なくとも1つの結果配列を提供する必要があります。
//
// A common use case for this method is to perform Dijkstra searches.
// Candidate polygons are found by searching the graph beginning at the start
// polygon.
// このメソッドの一般的な使用例は、ダイクストラ検索を実行することです。
// 候補ポリゴンは、開始ポリゴンから始まるグラフを検索することで見つけられます。
//
// The same intersection test restrictions that apply to findPolysAroundCircle()
// method apply to this method.
// findPolysAroundCircle（）メソッドに適用される同じ交差テストの制限がこのメソッドに適用されます。
//
// The 3D centroid of the search polygon is used as the start position for cost
// calculations.
// 検索ポリゴンの3D重心は、コスト計算の開始位置として使用されます。
//
// Intersection tests occur in 2D. All polygons are projected onto the
// xz-plane. So the y-values of the vertices do not effect intersection tests.
// 交差点テストは2Dで行われます。 すべてのポリゴンはxz平面に投影されます。
// したがって、頂点のy値は交差テストに影響しません。
//
// If the result arrays are is too small to hold the entire result set, they will
// be filled to capacity.
// 結果の配列が小さすぎて結果セット全体を保持できない場合、容量がフルになります。
//
dtStatus dtNavMeshQuery::findPolysAroundShape(dtPolyRef startRef, const float* verts, const int nverts,
	const dtQueryFilter* filter,
	dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
	int* resultCount, const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	*resultCount = 0;

	// Validate input
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;

	m_nodePool->clear();
	m_openList->clear();

	float centerPos[3] = { 0,0,0 };
	for (int i = 0; i < nverts; ++i)
		dtVadd(centerPos, centerPos, &verts[i * 3]);
	dtVscale(centerPos, centerPos, 1.f / nverts);

	dtNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);

	dtStatus status = DT_SUCCESS;

	int n = 0;

	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;

		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);

		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);

		if (n < maxResult)
		{
			if (resultRef)
				resultRef[n] = bestRef;
			if (resultParent)
				resultParent[n] = parentRef;
			if (resultCost)
				resultCost[n] = bestNode->total;

			++n;
		}
		else
		{
			status |= DT_BUFFER_TOO_SMALL;
		}

		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const dtLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;

			// Expand to neighbour
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);

			// Do not advance if the polygon is excluded by the filter.
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;

			// Find edge and calc distance to the edge.
			float va[3], vb[3];
			if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;

			// If the poly is not touching the edge to the next polygon, skip the connection it.
			float tmin, tmax;
			int segMin, segMax;
			if (!dtIntersectSegmentPoly2D(va, vb, verts, nverts, tmin, tmax, segMin, segMax))
				continue;
			if (tmin > 1.f || tmax < 0.0f)
				continue;

			dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}

			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;

			// Cost
			if (neighbourNode->flags == 0)
				dtVlerp(neighbourNode->pos, va, vb, 0.5f);

			float cost = filter->getCost(
				bestNode->pos, neighbourNode->pos,
				parentRef, parentTile, parentPoly,
				bestRef, bestTile, bestPoly,
				neighbourRef, neighbourTile, neighbourPoly);

			const float total = bestNode->total + cost;

			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;

			neighbourNode->id = neighbourRef;
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;

			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				neighbourNode->flags = DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}

	*resultCount = n;

	return status;
}

dtStatus dtNavMeshQuery::getPathFromDijkstraSearch(dtPolyRef endRef, dtPolyRef* path, int* pathCount, int maxPath) const
{
	if (!m_nav->isValidPolyRef(endRef) || !path || !pathCount || maxPath < 0)
		return DT_FAILURE | DT_INVALID_PARAM;

	*pathCount = 0;

	dtNode* endNode;
	if (m_nodePool->findNodes(endRef, &endNode, 1) != 1 ||
		(endNode->flags & DT_NODE_CLOSED) == 0)
		return DT_FAILURE | DT_INVALID_PARAM;

	return getPathToNode(endNode, path, pathCount, maxPath);
}

// @par
//
// This method is optimized for a small search radius and small number of result
// polygons.
// この方法は、検索半径が小さく、結果のポリゴン数が少ない場合に最適化されます。
//
// Candidate polygons are found by searching the navigation graph beginning at
// the start polygon.
// 候補ポリゴンは、開始ポリゴンから始まるナビゲーショングラフを検索することで検出されます。
//
// The same intersection test restrictions that apply to the findPolysAroundCircle
// mehtod applies to this method.
// findPolysAroundCircleメソッドに適用される同じ交差テストの制限がこのメソッドに適用されます。
//
// The value of the center point is used as the start point for cost calculations.
// It is not projected onto the surface of the mesh, so its y-value will effect
// the costs.
// 中心点の値は、コスト計算の開始点として使用されます。メッシュの表面には投影されないため、y値はコストに影響します。
//
// Intersection tests occur in 2D. All polygons and the search circle are
// projected onto the xz-plane. So the y-value of the center point does not
// effect intersection tests.
// 交差点テストは2Dで行われます。 すべてのポリゴンと検索円は、xz平面に投影されます。
// したがって、中心点のy値は交差テストに影響しません。
//
// If the result arrays are is too small to hold the entire result set, they will
// be filled to capacity.
// 結果の配列が小さすぎて結果セット全体を保持できない場合、容量がいっぱいになります。
//
dtStatus dtNavMeshQuery::findLocalNeighbourhood(dtPolyRef startRef, const float* centerPos, const float radius,
	const dtQueryFilter* filter,
	dtPolyRef* resultRef, dtPolyRef* resultParent,
	int* resultCount, const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_tinyNodePool);

	*resultCount = 0;

	// Validate input
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;

	constexpr int MAX_STACK = 48;
	dtNode* stack[MAX_STACK];
	int nstack = 0;

	m_tinyNodePool->clear();

	dtNode* startNode = m_tinyNodePool->getNode(startRef);
	startNode->pidx = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_CLOSED;
	stack[nstack++] = startNode;

	const float radiusSqr = dtSqr(radius);

	float pa[DT_VERTS_PER_POLYGON * 3];
	float pb[DT_VERTS_PER_POLYGON * 3];

	dtStatus status = DT_SUCCESS;

	int n = 0;
	if (n < maxResult)
	{
		resultRef[n] = startNode->id;
		if (resultParent)
			resultParent[n] = 0;
		++n;
	}
	else
	{
		status |= DT_BUFFER_TOO_SMALL;
	}

	while (nstack)
	{
		// Pop front.
		dtNode* curNode = stack[0];
		for (int i = 0; i < nstack - 1; ++i)
			stack[i] = stack[i + 1];
		nstack--;

		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef curRef = curNode->id;
		const dtMeshTile* curTile = 0;
		const dtPoly* curPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(curRef, &curTile, &curPoly);

		for (unsigned int i = curPoly->firstLink; i != DT_NULL_LINK; i = curTile->links[i].next)
		{
			const dtLink* link = &curTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours.
			if (!neighbourRef)
				continue;

			// Skip if cannot alloca more nodes.
			dtNode* neighbourNode = m_tinyNodePool->getNode(neighbourRef);
			if (!neighbourNode)
				continue;
			// Skip visited.
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;

			// Expand to neighbour
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);

			// Skip off-mesh connections.
			if (neighbourPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;

			// Do not advance if the polygon is excluded by the filter.
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;

			// Find edge and calc distance to the edge.
			float va[3], vb[3];
			if (!getPortalPoints(curRef, curPoly, curTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;

			// If the circle is not touching the next polygon, skip it.
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			if (distSqr > radiusSqr)
				continue;

			// Mark node visited, this is done before the overlap test so that
			// we will not visit the poly again if the test fails.
			neighbourNode->flags |= DT_NODE_CLOSED;
			neighbourNode->pidx = m_tinyNodePool->getNodeIdx(curNode);

			// Check that the polygon does not collide with existing polygons.

			// Collect vertices of the neighbour poly.
			const int npa = neighbourPoly->vertCount;
			for (int k = 0; k < npa; ++k)
				dtVcopy(&pa[k * 3], &neighbourTile->verts[neighbourPoly->verts[k] * 3]);

			bool overlap = false;
			for (int j = 0; j < n; ++j)
			{
				dtPolyRef pastRef = resultRef[j];

				// Connected polys do not overlap.
				bool connected = false;
				for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
				{
					if (curTile->links[k].ref == pastRef)
					{
						connected = true;
						break;
					}
				}
				if (connected)
					continue;

				// Potentially overlapping.
				const dtMeshTile* pastTile = 0;
				const dtPoly* pastPoly = 0;
				m_nav->getTileAndPolyByRefUnsafe(pastRef, &pastTile, &pastPoly);

				// Get vertices and test overlap
				const int npb = pastPoly->vertCount;
				for (int k = 0; k < npb; ++k)
					dtVcopy(&pb[k * 3], &pastTile->verts[pastPoly->verts[k] * 3]);

				if (dtOverlapPolyPoly2D(pa, npa, pb, npb))
				{
					overlap = true;
					break;
				}
			}
			if (overlap)
				continue;

			// This poly is fine, store and advance to the poly.
			if (n < maxResult)
			{
				resultRef[n] = neighbourRef;
				if (resultParent)
					resultParent[n] = curRef;
				++n;
			}
			else
			{
				status |= DT_BUFFER_TOO_SMALL;
			}

			if (nstack < MAX_STACK)
			{
				stack[nstack++] = neighbourNode;
			}
		}
	}

	*resultCount = n;

	return status;
}

struct dtSegInterval
{
	dtPolyRef ref;
	short tmin, tmax;
};

static void insertInterval(dtSegInterval* ints, int& nints, const int maxInts,
	const short tmin, const short tmax, const dtPolyRef ref)
{
	if (nints + 1 > maxInts) return;
	// Find insertion point.
	int idx = 0;
	while (idx < nints)
	{
		if (tmax <= ints[idx].tmin)
			break;
		idx++;
	}
	// Move current results.
	if (nints - idx)
		memmove(ints + idx + 1, ints + idx, sizeof(dtSegInterval) * (nints - idx));
	// Store
	ints[idx].ref = ref;
	ints[idx].tmin = tmin;
	ints[idx].tmax = tmax;
	nints++;
}

// @par
//
// If the @p segmentRefs parameter is provided, then all polygon segments will be returned.
// Otherwise only the wall segments are returned.
// segmentRefsパラメーターが指定されている場合、すべてのポリゴンセグメントが返されます。
// それ以外の場合、壁セグメントのみが返されます。
//
// A segment that is normally a portal will be included in the result set as a
// wall if the @p filter results in the neighbor polygon becoomming impassable.
// 通常はポータルであるセグメントは、フィルターが近隣ポリゴンを通過できない場合、結果セットに壁として含まれます。
//
// The @p segmentVerts and @p segmentRefs buffers should normally be sized for the
// maximum segments per polygon of the source navigation mesh.
// 通常、segmentVertsおよびsegmentRefsバッファーは、
// ソースナビゲーションメッシュのポリゴンごとの最大セグメントに合わせてサイズを調整する必要があります。
//
dtStatus dtNavMeshQuery::getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter,
	float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
	const int maxSegments) const
{
	dtAssert(m_nav);

	*segmentCount = 0;

	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
		return DT_FAILURE | DT_INVALID_PARAM;

	int n = 0;
	constexpr int MAX_INTERVAL = 16;
	dtSegInterval ints[MAX_INTERVAL];
	int nints;

	const bool storePortals = segmentRefs != 0;

	dtStatus status = DT_SUCCESS;

	for (int i = 0, j = (int)poly->vertCount - 1; i < (int)poly->vertCount; j = i++)
	{
		// Skip non-solid edges.
		nints = 0;
		if (poly->neis[j] & DT_EXT_LINK)
		{
			// Tile border.
			for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
			{
				const dtLink* link = &tile->links[k];
				if (link->edge == j)
				{
					if (link->ref != 0)
					{
						const dtMeshTile* neiTile = 0;
						const dtPoly* neiPoly = 0;
						m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
						if (filter->passFilter(link->ref, neiTile, neiPoly))
						{
							insertInterval(ints, nints, MAX_INTERVAL, link->bmin, link->bmax, link->ref);
						}
					}
				}
			}
		}
		else
		{
			// Internal edge
			dtPolyRef neiRef = 0;
			if (poly->neis[j])
			{
				const unsigned int idx = (unsigned int)(poly->neis[j] - 1);
				neiRef = m_nav->getPolyRefBase(tile) | idx;
				if (!filter->passFilter(neiRef, tile, &tile->polys[idx]))
					neiRef = 0;
			}

			// If the edge leads to another polygon and portals are not stored, skip.
			if (neiRef != 0 && !storePortals)
				continue;

			if (n < maxSegments)
			{
				const float* vj = &tile->verts[poly->verts[j] * 3];
				const float* vi = &tile->verts[poly->verts[i] * 3];
				float* seg = &segmentVerts[n * 6];
				dtVcopy(seg + 0, vj);
				dtVcopy(seg + 3, vi);
				if (segmentRefs)
					segmentRefs[n] = neiRef;
				n++;
			}
			else
			{
				status |= DT_BUFFER_TOO_SMALL;
			}

			continue;
		}

		// Add sentinels
		insertInterval(ints, nints, MAX_INTERVAL, -1, 0, 0);
		insertInterval(ints, nints, MAX_INTERVAL, 255, 256, 0);

		// Store segments.
		const float* vj = &tile->verts[poly->verts[j] * 3];
		const float* vi = &tile->verts[poly->verts[i] * 3];
		for (int k = 1; k < nints; ++k)
		{
			// Portal segment.
			if (storePortals && ints[k].ref)
			{
				const float tmin = ints[k].tmin / 255.0f;
				const float tmax = ints[k].tmax / 255.0f;
				if (n < maxSegments)
				{
					float* seg = &segmentVerts[n * 6];
					dtVlerp(seg + 0, vj, vi, tmin);
					dtVlerp(seg + 3, vj, vi, tmax);
					if (segmentRefs)
						segmentRefs[n] = ints[k].ref;
					n++;
				}
				else
				{
					status |= DT_BUFFER_TOO_SMALL;
				}
			}

			// Wall segment.
			const int imin = ints[k - 1].tmax;
			const int imax = ints[k].tmin;
			if (imin != imax)
			{
				const float tmin = imin / 255.0f;
				const float tmax = imax / 255.0f;
				if (n < maxSegments)
				{
					float* seg = &segmentVerts[n * 6];
					dtVlerp(seg + 0, vj, vi, tmin);
					dtVlerp(seg + 3, vj, vi, tmax);
					if (segmentRefs)
						segmentRefs[n] = 0;
					n++;
				}
				else
				{
					status |= DT_BUFFER_TOO_SMALL;
				}
			}
		}
	}

	*segmentCount = n;

	return status;
}

// @par
//
// @p hitPos is not adjusted using the height detail data.
// hitPosは、高さ詳細データを使用して調整されていません。
//
// @p hitDist will equal the search radius if there is no wall within the
// radius. In this case the values of @p hitPos and @p hitNormal are
// undefined.
// 半径内に壁がない場合、hitDistは検索半径と等しくなります。この場合、hitPosとhitNormalの値は未定義です。
//
// The normal will become unpredicable if @p hitDist is a very small number.
// hitDistが非常に小さい場合、法線は予測不能になります。
//
dtStatus dtNavMeshQuery::findDistanceToWall(dtPolyRef startRef, const float* centerPos, const float maxRadius,
	const dtQueryFilter* filter,
	float* hitDist, float* hitPos, float* hitNormal) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	// Validate input
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;

	m_nodePool->clear();
	m_openList->clear();

	dtNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);

	float radiusSqr = dtSqr(maxRadius);

	dtStatus status = DT_SUCCESS;

	const float* bestvj = 0, * bestvi = 0;

	while (!m_openList->empty())
	{
		dtNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;

		// Get poly and tile.
		// The API input has been cheked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const dtMeshTile* bestTile = 0;
		const dtPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);

		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const dtMeshTile* parentTile = 0;
		const dtPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);

		// Hit test walls.
		for (int i = 0, j = (int)bestPoly->vertCount - 1; i < (int)bestPoly->vertCount; j = i++)
		{
			// Skip non-solid edges.
			if (bestPoly->neis[j] & DT_EXT_LINK)
			{
				// Tile border.
				bool solid = true;
				for (unsigned int k = bestPoly->firstLink; k != DT_NULL_LINK; k = bestTile->links[k].next)
				{
					const dtLink* link = &bestTile->links[k];
					if (link->edge == j)
					{
						if (link->ref != 0)
						{
							const dtMeshTile* neiTile = 0;
							const dtPoly* neiPoly = 0;
							m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
							if (filter->passFilter(link->ref, neiTile, neiPoly))
								solid = false;
						}
						break;
					}
				}
				if (!solid) continue;
			}
			else if (bestPoly->neis[j])
			{
				// Internal edge
				const unsigned int idx = (unsigned int)(bestPoly->neis[j] - 1);
				const dtPolyRef ref = m_nav->getPolyRefBase(bestTile) | idx;
				if (filter->passFilter(ref, bestTile, &bestTile->polys[idx]))
					continue;
			}

			// Calc distance to the edge.
			const float* vj = &bestTile->verts[bestPoly->verts[j] * 3];
			const float* vi = &bestTile->verts[bestPoly->verts[i] * 3];
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, vj, vi, tseg);

			// Edge is too far, skip.
			if (distSqr > radiusSqr)
				continue;

			// Hit wall, update radius.
			radiusSqr = distSqr;
			// Calculate hit pos.
			hitPos[0] = vj[0] + (vi[0] - vj[0]) * tseg;
			hitPos[1] = vj[1] + (vi[1] - vj[1]) * tseg;
			hitPos[2] = vj[2] + (vi[2] - vj[2]) * tseg;
			bestvj = vj;
			bestvi = vi;
		}

		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const dtLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;

			// Expand to neighbour.
			const dtMeshTile* neighbourTile = 0;
			const dtPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);

			// Skip off-mesh connections.
			if (neighbourPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;

			// Calc distance to the edge.
			const float* va = &bestTile->verts[bestPoly->verts[link->edge] * 3];
			const float* vb = &bestTile->verts[bestPoly->verts[(link->edge + 1) % bestPoly->vertCount] * 3];
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);

			// If the circle is not touching the next polygon, skip it.
			if (distSqr > radiusSqr)
				continue;

			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;

			dtNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}

			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;

			// Cost
			if (neighbourNode->flags == 0)
			{
				getEdgeMidPoint(bestRef, bestPoly, bestTile,
					neighbourRef, neighbourPoly, neighbourTile, neighbourNode->pos);
			}

			const float total = bestNode->total + dtVdist(bestNode->pos, neighbourNode->pos);

			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;

			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;

			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				neighbourNode->flags |= DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}

	// Calc hit normal.
	if (bestvj && bestvi)
	{
		float tangent[3];
		dtVsub(tangent, bestvi, bestvj);

		hitNormal[0] = tangent[2];
		hitNormal[1] = 0;
		hitNormal[2] = -tangent[0];

		dtVnormalize(hitNormal);
	}

	*hitDist = dtMathSqrtf(radiusSqr);

	return status;
}

bool dtNavMeshQuery::isValidPolyRef(dtPolyRef ref, const dtQueryFilter* filter) const
{
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	dtStatus status = m_nav->getTileAndPolyByRef(ref, &tile, &poly);
	// If cannot get polygon, assume it does not exists and boundary is invalid.
	if (dtStatusFailed(status))
		return false;
	// If cannot pass filter, assume flags has changed and boundary is invalid.
	if (!filter->passFilter(ref, tile, poly))
		return false;
	return true;
}

// @par
//
// The closed list is the list of polygons that were fully evaluated during
// the last navigation graph search. (A* or Dijkstra)
// 閉じたリストは、最後のナビゲーショングラフ検索中に完全に評価されたポリゴンのリストです。（A*またはダイクストラ）
//
bool dtNavMeshQuery::isInClosedList(dtPolyRef ref) const
{
	if (!m_nodePool) return false;

	dtNode* nodes[DT_MAX_STATES_PER_NODE];
	int n = m_nodePool->findNodes(ref, nodes, DT_MAX_STATES_PER_NODE);

	for (int i = 0; i < n; i++)
	{
		if (nodes[i]->flags & DT_NODE_CLOSED)
			return true;
	}

	return false;
}
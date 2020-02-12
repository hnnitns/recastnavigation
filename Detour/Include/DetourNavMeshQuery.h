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

#ifndef DETOURNAVMESHQUERY_H
#define DETOURNAVMESHQUERY_H

#include <array>
#include "DetourConfig.h"
#include "DetourNavMesh.h"
#include "DetourStatus.h"

// Defines polygon filtering and traversal costs for navigation mesh query operations.
// ナビゲーションメッシュクエリ操作のポリゴンフィルタリングとトラバーサルコストを定義します。
// @ingroup detour
class dtQueryFilter
{
	// Cost per area type. (Used by default implementation.)
	// エリアタイプごとのコスト。 （デフォルトの実装で使用されます。）
	std::array<float, DT_MAX_AREAS> m_areaCost;

	// Flags for polygons that can be visited. (Used by default implementation.)
	// 訪問できるポリゴンのフラグ。 （デフォルトの実装で使用されます。）
	unsigned short m_includeFlags;

	// Flags for polygons that should not be visted. (Used by default implementation.)
	// 参照されるべきではないポリゴンのフラグ。 （デフォルトの実装で使用されます。）
	unsigned short m_excludeFlags;

public:
	dtQueryFilter();

#ifdef DT_VIRTUAL_QUERYFILTER
	virtual ~dtQueryFilter() { }
#endif

#ifdef DT_VIRTUAL_QUERYFILTER
	virtual bool passFilter(const dtPolyRef ref,
		const dtMeshTile* tile,
		const dtPoly* poly) const;
#else
	// Returns true if the polygon can be visited.  (I.e. Is traversable.)
	// ポリゴンにアクセスできる場合はtrueを返します。 （つまり、トラバース可能です。）
	// @param[in] ref : The reference id of the polygon test.
	// ポリゴンテストの参照ID。
	// @param[in] tile : The tile containing the polygon.
	// ポリゴンを含むタイル。
	// @param[in] poly : The polygon to test.
	// テストするポリゴン。
	bool passFilter(
		const dtPolyRef ref,
		const dtMeshTile* tile,
		const dtPoly* poly) const;
#endif

#ifdef DT_VIRTUAL_QUERYFILTER
	virtual float getCost(const float* pa, const float* pb,
		const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
		const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
		const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const;
#else
	// Returns cost to move from the beginning to the end of a line segment that is fully contained within a polygon.
	// ポリゴン内に完全に含まれるラインセグメントの最初から最後まで移動するコストを返します。
	// @param[in] pa : The start position on the edge of the previous and current polygon. [(x, y, z)]
	// 前のポリゴンと現在のポリゴンの端の開始位置。 [（x、y、z）]
	// @param[in] pb : The end position on the edge of the current and next polygon. [(x, y, z)]
	// 現在および次のポリゴンの端の終了位置。 [（x、y、z）]
	// @param[in] prevRef : The reference id of the previous polygon. [opt]
	// 前のポリゴンの参照ID。 [最適化]
	// @param[in] prevTile : The tile containing the previous polygon. [opt]
	// 前のポリゴンを含むタイル。 [最適化]
	// @param[in] prevPoly : The previous polygon. [opt]
	// 前のポリゴン。 [最適化]
	// @param[in] curRef : The reference id of the current polygon.
	// 現在のポリゴンの参照ID。
	// @param[in] curTile : The tile containing the current polygon.
	// 現在のポリゴンを含むタイル。
	// @param[in] curPoly : The current polygon.
	// 現在のポリゴン。
	// @param[in] nextRef : The refernece id of the next polygon. [opt]
	// 次のポリゴンの参照ID。 [最適化]
	// @param[in] nextTile : The tile containing the next polygon. [opt]
	// 次のポリゴンを含むタイル。 [最適化]
	// @param[in] nextPoly : The next polygon. [opt]
	// 次のポリゴン。 [最適化]
	float getCost(const std::array<float, 3>& pa, const float* pb,
		const dtPolyRef prevRef, const dtMeshTile* prevTile, const dtPoly* prevPoly,
		const dtPolyRef curRef, const dtMeshTile* curTile, const dtPoly* curPoly,
		const dtPolyRef nextRef, const dtMeshTile* nextTile, const dtPoly* nextPoly) const;
#endif

	// @name Getters and setters for the default implementation data.
	// デフォルトの実装データの取得メソッドと設定メソッド。
	//@{

	// Returns the traversal cost of the area.
	// エリアの横断コストを返します。
	//  @param[in]		i		The id of the area.
	// @returns The traversal cost of the area.
	inline float getAreaCost(const int i) const { return m_areaCost[i]; }

	// Sets the traversal cost of the area.
	// エリアのトラバーサルコストを設定します。
	//  @param[in]		i		The id of the area.
	//  @param[in]		cost	The new cost of traversing the area.
	inline void setAreaCost(const int i, const float cost) { m_areaCost[i] = cost; }

	// Returns the include flags for the filter.
	// フィルタのインクルードフラグを返します。
	// Any polygons that include one or more of these flags will be included in the operation.
	// これらのフラグの1つ以上を含むポリゴンは、操作に含まれます。
	inline unsigned short getIncludeFlags() const { return m_includeFlags; }

	// Sets the include flags for the filter.
	// フィルタのインクルードフラグを設定します。
	// @param[in]		flags	The new flags.
	inline void setIncludeFlags(const unsigned short flags) { m_includeFlags = flags; }

	// Returns the exclude flags for the filter.
	// フィルタの除外フラグを返します。
	// Any polygons that include one ore more of these flags will be excluded from the operation.
	// これらのフラグを1つ以上含むポリゴンはすべて、操作から除外されます。
	inline unsigned short getExcludeFlags() const { return m_excludeFlags; }

	// Sets the exclude flags for the filter.
	// フィルタの除外フラグを設定します。
	// @param[in]		flags		The new flags.
	inline void setExcludeFlags(const unsigned short flags) { m_excludeFlags = flags; }

	//@}
};

// Provides information about raycast hit
// filled by dtNavMeshQuery::raycast
// @ingroup detour
struct dtRaycastHit
{
	// The hit parameter. (FLT_MAX if no wall hit.)
	float t;

	// hitNormal	The normal of the nearest wall hit. [(x, y, z)]
	std::array<float, 3> hitNormal;

	// The index of the edge on the final polygon where the wall was hit.
	int hitEdgeIndex;

	// Pointer to an array of reference ids of the visited polygons. [opt]
	dtPolyRef* path;

	// The number of visited polygons. [opt]
	int pathCount;

	// The maximum number of polygons the @p path array can hold.
	int maxPath;

	//  The cost of the path until hit.
	float pathCost;
};

// Provides custom polygon query behavior.
// Used by dtNavMeshQuery::queryPolygons.
// @ingroup detour
class dtPolyQuery
{
public:
	virtual ~dtPolyQuery() { }

	// Called for each batch of unique polygons touched by the search area in dtNavMeshQuery::queryPolygons.
	// This can be called multiple times for a single query.
	virtual void process(const dtMeshTile* tile, dtPoly** polys, dtPolyRef* refs, int count) = 0;
};

// Provides the ability to perform pathfinding related queries against
// a navigation mesh.
// @ingroup detour
class dtNavMeshQuery
{
public:
	dtNavMeshQuery();
	~dtNavMeshQuery();

	// クエリオブジェクトを初期化します
	// @param[in] nav：すべてのクエリに使用するdtNavMeshオブジェクトへのポインター
	// @param[in] maxNodes：検索ノードの最大数。[制限：0 < 値 <= 65535]
	// @returnsクエリのステータスフラグ
	// Initializes the query object.
	// @param[in]		nav			Pointer to the dtNavMesh object to use for all queries.
	// @param[in]		maxNodes	Maximum number of search nodes. [Limits: 0 < value <= 65535]
	// @returns The status flags for the query.
	dtStatus init(const dtNavMesh* nav, const int maxNodes);

	// 開始ポリゴンから終了ポリゴンまでのパスを検索します。
	// param [in] startRef	：開始ポリゴンの参照ID
	// param [in] endRef		：終了ポリゴンの参照ID
	// param [in] startPos	：開始ポリゴン内の位置。 [（x、y、z）]
	// param [in] endPos		：終了ポリゴン内の位置。 [（x、y、z）]
	// param [in] filter		：クエリに適用するポリゴンフィルター
	// param [out] path		：パスを表すポリゴン参照の順序付きリスト （始めから終わりまで。）
	// param [out] pathCount	：@pパス配列で返されるポリゴンの数
	// param [in] maxPath	：@pパス配列が保持できるポリゴンの最大数。 [制限：> = 1]
	// @name Standard Pathfinding Functions
	// /@{
	// Finds a path from the start polygon to the end polygon.
	//  @param[in]		startRef	The refrence id of the start polygon.
	//  @param[in]		endRef		The reference id of the end polygon.
	//  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
	//  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
	//  @param[in]		filter		The polygon filter to apply to the query.
	//  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
	//  							[(polyRef) * @p pathCount]
	//  @param[out]	pathCount	The number of polygons returned in the @p path array.
	//  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 1]
	dtStatus findPath(dtPolyRef startRef, dtPolyRef endRef,
		const float* startPos, const float* endPos,
		const dtQueryFilter* filter,
		dtPolyRef* path, int* pathCount, const int maxPath) const;

	// ポリゴンコリドー内の開始位置から終了位置までの直線パスを検索します。
	// @param[in] startPos：パスの開始位置。[（x、y、z）]
	// @param[in] endPos：パスの終了位置。[（x、y、z）]
	// @param[in] path：パスコリドーを表すポリゴン参照の配列。
	// @param[in] pathSize：@p path：配列内のポリゴンの数。
	// @param[out] straightPath：直線パスを記述するポイント。[（x、y、z） * @p straightPathCount]。
	// @param[out] straightPathFlags：各ポイントを説明するフラグ。 （#dtStraightPathFlagsを参照）
	// @param[out] straightPathRefs：各ポイントで入力されているポリゴンの参照ID。[最適化]
	// @param[out] straightPathCount：直線パス内のポイントの数。
	// @param[in] maxStraightPath：直線パス配列が保持できるポイントの最大数。[制限： > 0]
	// @param[in] options：クエリオプション。 （参照：#dtStraightPathOptions）
	// @returnsクエリのステータスフラグ。
	// Finds the straight path from the start to the end position within the polygon corridor.
	//  @param[in]		startPos			Path start position. [(x, y, z)]
	//  @param[in]		endPos				Path end position. [(x, y, z)]
	//  @param[in]		path				An array of polygon references that represent the path corridor.
	//  @param[in]		pathSize			The number of polygons in the @p path array.
	//  @param[out]	straightPath		Points describing the straight path. [(x, y, z) * @p straightPathCount].
	//  @param[out]	straightPathFlags	Flags describing each point. (See: #dtStraightPathFlags) [opt]
	//  @param[out]	straightPathRefs	The reference id of the polygon that is being entered at each point. [opt]
	//  @param[out]	straightPathCount	The number of points in the straight path.
	//  @param[in]		maxStraightPath		The maximum number of points the straight path arrays can hold.  [Limit: > 0]
	//  @param[in]		options				Query options. (see: #dtStraightPathOptions)
	// @returns The status flags for the query.
	dtStatus findStraightPath(const float* startPos, const float* endPos,
		const dtPolyRef* path, const int pathSize,
		float* straightPath, uint8_t* straightPathFlags, dtPolyRef* straightPathRefs,
		int* straightPathCount, const int maxStraightPath, const int options = 0) const;

	// @nameスライスパス検索関数
	// 一般的な使用例：
	// - ＃initSlicedFindPath（）を呼び出して、スライスパスクエリを初期化します。
	// - ＃完了するまでupdateSlicedFindPath（）を呼び出します。
	// - ＃パスを取得するためにfinalizeSlicedFindPath（）を呼び出します。
	// スライスパスクエリを初期化します。
	// @param[in] startRef開始ポリゴンの参照ID。
	// @param[in] endRef終了ポリゴンの参照ID。
	// @param[in] startPos開始ポリゴン内の位置。[（x、y、z）]
	// @param[in] endPos終了ポリゴン内の位置。[（x、y、z）]
	// @param[in] filterクエリに適用するポリゴンフィルター。
	// @param[in] optionsクエリオプション（#dtFindPathOptionsを参照）
	// @returnsクエリのステータスフラグ。
	//@}
	// @name Sliced Pathfinding Functions
	// Common use case:
	//	-# Call initSlicedFindPath() to initialize the sliced path query.
	//	-# Call updateSlicedFindPath() until it returns complete.
	//	-# Call finalizeSlicedFindPath() to get the path.
	//@{
	// Intializes a sliced path query.
	//  @param[in]		startRef	The refrence id of the start polygon.
	//  @param[in]		endRef		The reference id of the end polygon.
	//  @param[in]		startPos	A position within the start polygon. [(x, y, z)]
	//  @param[in]		endPos		A position within the end polygon. [(x, y, z)]
	//  @param[in]		filter		The polygon filter to apply to the query.
	//  @param[in]		options		query options (see: #dtFindPathOptions)
	// @returns The status flags for the query.
	dtStatus initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
		const float* startPos, const float* endPos,
		const dtQueryFilter* filter, const unsigned int options = 0);

	// 進行中のスライスパスクエリを更新します。
	// @param[in] maxIter実行する反復の最大数。
	// @param[out] doneIters完了した実際の反復回数。[最適化]
	// @returnsクエリのステータスフラグ。
	// Updates an in-progress sliced path query.
	//  @param[in]		maxIter		The maximum number of iterations to perform.
	//  @param[out]	doneIters	The actual number of iterations completed. [opt]
	// @returns The status flags for the query.
	dtStatus updateSlicedFindPath(const int maxIter, int* doneIters);

	// スライスパスクエリの結果を確定して返します。
	// @param[out] pathパスを表すポリゴン参照の順序付きリスト。 （始めから終わりまで。）
	// [（polyRef） * @p pathCount]
	// @param[out] pathCount @pパス配列で返されるポリゴンの数。
	// @param[in] maxPathパス配列が保持できるポリゴンの最大数。[制限： > = 1]
	// @returnsクエリのステータスフラグ。
	// Finalizes and returns the results of a sliced path query.
	//  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
	//  							[(polyRef) * @p pathCount]
	//  @param[out]	pathCount	The number of polygons returned in the @p path array.
	//  @param[in]		maxPath		The max number of polygons the path array can hold. [Limit: >= 1]
	// @returns The status flags for the query.
	dtStatus finalizeSlicedFindPath(dtPolyRef* path, int* pathCount, const int maxPath);

	// 不完全なスライスパスクエリの結果を確定して返し、検索中にアクセスした既存のパス上の最も遠いポリゴンへのパスを返します。
	// @param[in] existing既存のパスのポリゴン参照の配列。
	// @param[in] existingSize @p既存の配列内のポリゴンの数。
	// @param[out] pathパスを表すポリゴン参照の順序付きリスト。 （始めから終わりまで。）
	// [（polyRef） * @p pathCount]
	// @param[out] pathCount @pパス配列で返されるポリゴンの数。
	// @param[in] maxPath @pパス配列が保持できるポリゴンの最大数。[制限： > = 1]
	// @returnsクエリのステータスフラグ。
	// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
	// polygon on the existing path that was visited during the search.
	//  @param[in]		existing		An array of polygon references for the existing path.
	//  @param[in]		existingSize	The number of polygon in the @p existing array.
	//  @param[out]	path			An ordered list of polygon references representing the path. (Start to end.)
	//  								[(polyRef) * @p pathCount]
	//  @param[out]	pathCount		The number of polygons returned in the @p path array.
	//  @param[in]		maxPath			The max number of polygons the @p path array can hold. [Limit: >= 1]
	// @returns The status flags for the query.
	dtStatus finalizeSlicedFindPathPartial(const dtPolyRef* existing, const int existingSize,
		dtPolyRef* path, int* pathCount, const int maxPath);

	// @name Dijkstra検索関数
	// 指定された円に接触するナビゲーショングラフに沿ってポリゴンを検索します。
	// @param[in] startRef検索を開始するポリゴンの参照ID。
	// @param[in] centerPos検索サークルの中心。[（x、y、z）]
	// @param[in] radius検索円の半径。
	// @param[in] filterクエリに適用するポリゴンフィルター。
	// @param[out] resultRef円が接触するポリゴンの参照ID。[最適化]
	// @param[out] resultParent各結果の親ポリゴンの参照ID。
	// 結果のポリゴンに親がない場合はゼロ。[最適化]
	// @param[out] resultCost @p centerPosからポリゴンまでの検索コスト。[最適化]
	// @param[out] resultCount見つかったポリゴンの数。[最適化]
	// @param[in] maxResult結果の配列が保持できるポリゴンの最大数。
	// @returnsクエリのステータスフラグ。
	//@}
	// @name Dijkstra Search Functions
	// @{
	// Finds the polygons along the navigation graph that touch the specified circle.
	//  @param[in]		startRef		The reference id of the polygon where the search starts.
	//  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	//  @param[in]		radius			The radius of the search circle.
	//  @param[in]		filter			The polygon filter to apply to the query.
	//  @param[out]	resultRef		The reference ids of the polygons touched by the circle. [opt]
	//  @param[out]	resultParent	The reference ids of the parent polygons for each result.
	//  								Zero if a result polygon has no parent. [opt]
	//  @param[out]	resultCost		The search cost from @p centerPos to the polygon. [opt]
	//  @param[out]	resultCount		The number of polygons found. [opt]
	//  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	// @returns The status flags for the query.
	dtStatus findPolysAroundCircle(dtPolyRef startRef, const float* centerPos, const float radius,
		const dtQueryFilter* filter,
		dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
		int* resultCount, const int maxResult) const;

	// 指定された凸多角形に接触するナビゲーショングラフに沿って多角形を検索します。
	// @param [in] startRef検索を開始するポリゴンの参照ID。
	// @param [in] verts凸多角形を記述する頂点。 （CCW）
	// [（x、y、z）* @p nverts]
	// @param [in] nvertsポリゴン内の頂点の数。
	// @param [in] filterクエリに適用するポリゴンフィルター。
	// @param [out] resultRef検索ポリゴンが接触しているポリゴンの参照ID。 [最適化]
	// @param [out] resultParent各結果の親ポリゴンの参照ID。 ゼロの場合結果のポリゴンには親がありません。 [最適化]
	// @param [out] resultCost重心点からポリゴンまでの検索コスト。 [最適化]
	// @param [out] resultCount見つかったポリゴンの数。
	// @param [in] maxResult結果の配列が保持できるポリゴンの最大数。
	// @returnsクエリのステータスフラグ。
	// Finds the polygons along the naviation graph that touch the specified convex polygon.
	//  @param[in]		startRef		The reference id of the polygon where the search starts.
	//  @param[in]		verts			The vertices describing the convex polygon. (CCW)
	//  								[(x, y, z) * @p nverts]
	//  @param[in]		nverts			The number of vertices in the polygon.
	//  @param[in]		filter			The polygon filter to apply to the query.
	//  @param[out]	resultRef		The reference ids of the polygons touched by the search polygon. [opt]
	//  @param[out]	resultParent	The reference ids of the parent polygons for each result. Zero if a
	//  								result polygon has no parent. [opt]
	//  @param[out]	resultCost		The search cost from the centroid point to the polygon. [opt]
	//  @param[out]	resultCount		The number of polygons found.
	//  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	// @returns The status flags for the query.
	dtStatus findPolysAroundShape(dtPolyRef startRef, const float* verts, const int nverts,
		const dtQueryFilter* filter,
		dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
		int* resultCount, const int maxResult) const;

	//前の検索で探索されたノードからパスを取得します。
	// @param [in] endRef終了ポリゴンの参照ID。
	// @param [out] pathパスを表すポリゴン参照の順序付きリスト。 （始めから終わりまで）[（polyRef）* @p pathCount]
	// @param [out] pathCount @pパス配列で返されるポリゴンの数。
	// @param [in] maxPath @pパス配列が保持できるポリゴンの最大数。 [制限：> = 0]
	// @returnsステータスフラグ。 DT_FAILUREを返します| DT_INVALID_PARAMパラメータが間違っている場合、または
	// @p endRefは、以前の検索では探索されませんでした。 DT_SUCCESSを返します| DT_BUFFER_TOO_SMALL
	// @pパスにパス全体を含めることができない場合。この場合、部分的なパスで容量がいっぱいになります。
	//それ以外の場合はDT_SUCCESSを返します。
	// @remarksこの関数の結果は、クエリオブジェクトの状態に依存します。そのため、
	// 2つのダイクストラ検索のいずれか、findPolysAroundCircleまたはfindPolysAroundShapeの直後に使用されます。
	// Gets a path from the explored nodes in the previous search.
	//  @param[in]		endRef		The reference id of the end polygon.
	//  @param[out]	path		An ordered list of polygon references representing the path. (Start to end.)
	//  							[(polyRef) * @p pathCount]
	//  @param[out]	pathCount	The number of polygons returned in the @p path array.
	//  @param[in]		maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 0]
	//  @returns		The status flags. Returns DT_FAILURE | DT_INVALID_PARAM if any parameter is wrong, or if
	//  				@p endRef was not explored in the previous search. Returns DT_SUCCESS | DT_BUFFER_TOO_SMALL
	//  				if @p path cannot contain the entire path. In this case it is filled to capacity with a partial path.
	//  				Otherwise returns DT_SUCCESS.
	//  @remarks		The result of this function depends on the state of the query object. For that reason it should only
	//  				be used immediately after one of the two Dijkstra searches, findPolysAroundCircle or findPolysAroundShape.
	dtStatus getPathFromDijkstraSearch(dtPolyRef endRef, dtPolyRef* path, int* pathCount, int maxPath) const;

	// @nameローカルクエリ関数
	//指定された中心点に最も近いポリゴンを検索します。
	// @param [in] center検索ボックスの中心。 [（x、y、z）]
	// @param [in] extents各軸に沿った検索距離。 [（x、y、z）]
	// @param [in] filterクエリに適用するポリゴンフィルター。
	// @param [out] nearestRef最も近いポリゴンの参照ID。
	// @param [out] nearestPtポリゴン上の最も近いポイント。 [opt] [（x、y、z）]
	// @returnsクエリのステータスフラグ。
	// @}
	// @name Local Query Functions
	//@{
	// Finds the polygon nearest to the specified center point.
	//  @param[in]		center		The center of the search box. [(x, y, z)]
	//  @param[in]		extents		The search distance along each axis. [(x, y, z)]
	//  @param[in]		filter		The polygon filter to apply to the query.
	//  @param[out]	nearestRef	The reference id of the nearest polygon.
	//  @param[out]	nearestPt	The nearest point on the polygon. [opt] [(x, y, z)]
	// @returns The status flags for the query.
	dtStatus findNearestPoly(const float* center, const float* extents,
		const dtQueryFilter* filter,
		dtPolyRef* nearestRef, float* nearestPt) const;

	// 検索ボックスに重なるポリゴンを検​​索します。
	// @param [in] center検索ボックスの中心。[（x、y、z）]
	// @param [in] halfExtents各軸に沿った検索距離。[（x、y、z）]
	// @param [in] filterクエリに適用するポリゴンフィルター。
	// @param [out] polysクエリボックスに重なるポリゴンの参照ID。
	// @param [out] polyCount検索結果のポリゴンの数。
	// @param [in] maxPolys検索結果が保持できるポリゴンの最大数。
	// @returnsクエリのステータスフラグ。
	// Finds polygons that overlap the search box.
	//  @param[in]		center		The center of the search box. [(x, y, z)]
	//  @param[in]		extents		The search distance along each axis. [(x, y, z)]
	//  @param[in]		filter		The polygon filter to apply to the query.
	//  @param[out]	polys		The reference ids of the polygons that overlap the query box.
	//  @param[out]	polyCount	The number of polygons in the search result.
	//  @param[in]		maxPolys	The maximum number of polygons the search result can hold.
	// @returns The status flags for the query.
	dtStatus queryPolygons(const float* center, const float* extents,
		const dtQueryFilter* filter,
		dtPolyRef* polys, int* polyCount, const int maxPolys) const;

	// 検索ボックスに重なるポリゴンを検​​索します。
	// @param [in] center検索ボックスの中心。[（x、y、z）]
	// @param [in] halfExtents各軸に沿った検索距離。[（x、y、z）]
	// @param [in] filterクエリに適用するポリゴンフィルター。
	// @param [in] queryクエリ。見つかったポリゴンはバッチ処理され、このクエリに渡されます。
	// Finds polygons that overlap the search box.
	//  @param[in]		center		The center of the search box. [(x, y, z)]
	//  @param[in]		extents		The search distance along each axis. [(x, y, z)]
	//  @param[in]		filter		The polygon filter to apply to the query.
	//  @param[in]		query		The query. Polygons found will be batched together and passed to this query.
	dtStatus queryPolygons(const float* center, const float* extents,
		const dtQueryFilter* filter, dtPolyQuery* query) const;

	// 中心位置の周りのローカル近傍で重複しないナビゲーションポリゴンを検​​索します。
	// @param [in] startRef検索を開始するポリゴンの参照ID。
	// @param [in] centerPosクエリサークルの中心。[（x、y、z）]
	// @param [in] radiusクエリ円の半径。
	// @param [in] filterクエリに適用するポリゴンフィルター。
	// @param [out] resultRef円が接触するポリゴンの参照ID。
	// @param [out] resultParent各結果の親ポリゴンの参照ID。
	// 結果のポリゴンに親がない場合はゼロ。[最適化]
	// @param [out] resultCount見つかったポリゴンの数。
	// @param [in] maxResult結果の配列が保持できるポリゴンの最大数。
	// @returnsクエリのステータスフラグ。
	// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position.
	//  @param[in]		startRef		The reference id of the polygon where the search starts.
	//  @param[in]		centerPos		The center of the query circle. [(x, y, z)]
	//  @param[in]		radius			The radius of the query circle.
	//  @param[in]		filter			The polygon filter to apply to the query.
	//  @param[out]	resultRef		The reference ids of the polygons touched by the circle.
	//  @param[out]	resultParent	The reference ids of the parent polygons for each result.
	//  								Zero if a result polygon has no parent. [opt]
	//  @param[out]	resultCount		The number of polygons found.
	//  @param[in]		maxResult		The maximum number of polygons the result arrays can hold.
	// @returns The status flags for the query.
	dtStatus findLocalNeighbourhood(dtPolyRef startRef, const float* centerPos, const float radius,
		const dtQueryFilter* filter,
		dtPolyRef* resultRef, dtPolyRef* resultParent,
		int* resultCount, const int maxResult) const;

	// ナビゲーションメッシュに制約された開始位置から終了位置に移動します。
	// @param [in] startRef開始ポリゴンの参照ID。
	// @param [in] startPos開始ポリゴン内のムーバーの位置。[（x、y、x）]
	// @param [in] endPosムーバーの目的の終了位置。[（x、y、z）]
	// @param [in] filterクエリに適用するポリゴンフィルター。
	// @param [out] resultPosムーバーの結果の位置。[（x、y、z）]
	// @param [out] visited移動中に訪問されたポリゴンの参照ID。
	// @param [out] VisitedCount移動中に訪問されたポリゴンの数。
	// @param [in] maxVisitedSize @pが訪問した配列が保持できるポリゴンの最大数。
	// @returnsクエリのステータスフラグ。
	// Moves from the start to the end position constrained to the navigation mesh.
	//  @param[in]		startRef		The reference id of the start polygon.
	//  @param[in]		startPos		A position of the mover within the start polygon. [(x, y, x)]
	//  @param[in]		endPos			The desired end position of the mover. [(x, y, z)]
	//  @param[in]		filter			The polygon filter to apply to the query.
	//  @param[out]	resultPos		The result position of the mover. [(x, y, z)]
	//  @param[out]	visited			The reference ids of the polygons visited during the move.
	//  @param[out]	visitedCount	The number of polygons visited during the move.
	//  @param[in]		maxVisitedSize	The maximum number of polygons the @p visited array can hold.
	// @returns The status flags for the query.
	dtStatus moveAlongSurface(dtPolyRef startRef, const float* startPos, const float* endPos,
		const dtQueryFilter* filter,
		float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) const;

	// ナビゲーションメッシュの表面に沿って「歩行可能性」光線をキャストします
	// 終了位置に向かう開始位置。
	// @note raycast（...、RaycastHit *）のラッパー。下位互換性のために保持されます。
	// @param [in] startRef開始ポリゴンの参照ID。
	// @param [in] startPos開始ポリゴン内の位置を表す
	// レイの開始。[（x、y、z）]
	// @param [in] endPos光線を向ける位置。[（x、y、z）]
	// @param [out] tヒットパラメータ。（壁に当たらない場合はFLT_MAX。）
	// @param [out] hitNormal最も近い壁のヒットの法線。[（x、y、z）]
	// @param [in] filterクエリに適用するポリゴンフィルター
	// @param [out] path訪問したポリゴンの参照ID。[最適化]
	// @param [out] pathCount訪問したポリゴンの数。[最適化]
	// @param [in] maxPath @pパス配列が保持できるポリゴンの最大数
	// @returnsクエリのステータスフラグ
	// Casts a 'walkability' ray along the surface of the navigation mesh from
	// the start position toward the end position.
	// @note A wrapper around raycast(..., RaycastHit*). Retained for backward compatibility.
	//  @param[in]		startRef	The reference id of the start polygon.
	//  @param[in]		startPos	A position within the start polygon representing
	//  							the start of the ray. [(x, y, z)]
	//  @param[in]		endPos		The position to cast the ray toward. [(x, y, z)]
	//  @param[out]	t			The hit parameter. (FLT_MAX if no wall hit.)
	//  @param[out]	hitNormal	The normal of the nearest wall hit. [(x, y, z)]
	//  @param[in]		filter		The polygon filter to apply to the query.
	//  @param[out]	path		The reference ids of the visited polygons. [opt]
	//  @param[out]	pathCount	The number of visited polygons. [opt]
	//  @param[in]		maxPath		The maximum number of polygons the @p path array can hold.
	// @returns The status flags for the query.
	dtStatus raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
		const dtQueryFilter* filter,
		float* t, float* hitNormal, dtPolyRef* path, int* pathCount, const int maxPath) const;

	// ナビゲーションメッシュの表面に沿って「歩行可能性」光線をキャストします
	// 終了位置に向かう開始位置。
	// @param [in] startRef開始ポリゴンの参照ID。
	// @param [in] startPos開始ポリゴン内の位置を表す
	// レイの開始。[（x、y、z）]
	// @param [in] endPos光線を向ける位置。[（x、y、z）]
	// @param [in] filterクエリに適用するポリゴンフィルター。
	// @param [in]フラグは、レイキャストの動作を制御します。dtRaycastOptionsを参照してください
	// @param [out] hit結果で満たされるレイキャストヒット構造体へのポインタ。
	// @param [in] prevRef start refの親。コスト計算中に使用[opt]
	// @returnsクエリのステータスフラグ。
	// Casts a 'walkability' ray along the surface of the navigation mesh from
	// the start position toward the end position.
	//  @param[in]		startRef	The reference id of the start polygon.
	//  @param[in]		startPos	A position within the start polygon representing
	//  							the start of the ray. [(x, y, z)]
	//  @param[in]		endPos		The position to cast the ray toward. [(x, y, z)]
	//  @param[in]		filter		The polygon filter to apply to the query.
	//  @param[in]		flags		govern how the raycast behaves. See dtRaycastOptions
	//  @param[out]	hit			Pointer to a raycast hit structure which will be filled by the results.
	//  @param[in]		prevRef		parent of start ref. Used during for cost calculation [opt]
	// @returns The status flags for the query.
	dtStatus raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
		const dtQueryFilter* filter, const unsigned int options,
		dtRaycastHit* hit, dtPolyRef prevRef = 0) const;

	// ナビゲーションメッシュの表面に沿って「歩行可能性」光線をキャストします
	// 終了位置に向かう開始位置。
	// @param [in] startRef開始ポリゴンの参照ID。
	// @param [in] startPos開始ポリゴン内の位置を表す
	// レイの開始。[（x、y、z）]
	// @param [in] endPos光線を向ける位置。[（x、y、z）]
	// @param [in] filterクエリに適用するポリゴンフィルター。
	// @param [in]フラグは、レイキャストの動作を制御します。dtRaycastOptionsを参照してください
	// @param [out] hit結果で満たされるレイキャストヒット構造体へのポインタ。
	// @param [in] prevRef start refの親。コスト計算中に使用[opt]
	// @returnsクエリのステータスフラグ。
	// Finds the distance from the specified position to the nearest polygon wall.
	//  @param[in]		startRef		The reference id of the polygon containing @p centerPos.
	//  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	//  @param[in]		maxRadius		The radius of the search circle.
	//  @param[in]		filter			The polygon filter to apply to the query.
	//  @param[out]	hitDist			The distance to the nearest wall from @p centerPos.
	//  @param[out]	hitPos			The nearest position on the wall that was hit. [(x, y, z)]
	//  @param[out]	hitNormal		The normalized ray formed from the wall point to the
	//  								source point. [(x, y, z)]
	// @returns The status flags for the query.
	dtStatus findDistanceToWall(dtPolyRef startRef, const float* centerPos, const float maxRadius,
		const dtQueryFilter* filter,
		float* hitDist, float* hitPos, float* hitNormal) const;

	// オプションでポータルを含む、指定されたポリゴンのセグメントを返します。
	// @param [in] refポリゴンの参照ID。
	// @param [in] filterクエリに適用するポリゴンフィルター。
	// @param [out] segmentVertsセグメント。[（ax、ay、az、bx、by、bz）* segmentCount]
	// @param [out] segmentRefs各セグメントの隣接ポリゴンの参照ID。
	// または、セグメントが壁の場合はゼロ。[opt] [（parentRef）* @p segmentCount]
	// @param [out] segmentCount返されるセグメントの数。
	// @param [in] maxSegments結果の配列が保持できるセグメントの最大数。
	// @returnsクエリのステータスフラグ。
	// Returns the segments for the specified polygon, optionally including portals.
	//  @param[in]		ref				The reference id of the polygon.
	//  @param[in]		filter			The polygon filter to apply to the query.
	//  @param[out]	segmentVerts	The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
	//  @param[out]	segmentRefs		The reference ids of each segment's neighbor polygon.
	//  								Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount]
	//  @param[out]	segmentCount	The number of segments returned.
	//  @param[in]		maxSegments		The maximum number of segments the result arrays can hold.
	// @returns The status flags for the query.
	dtStatus getPolyWallSegments(dtPolyRef ref, const dtQueryFilter* filter,
		float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
		const int maxSegments) const;

	// navmeshのランダムな位置を返します。
	// ポリゴンは、エリアごとに重み付けされて選択されます。検索は、ポリゴンの数に関連する線形で実行されます。
	// @param [in] filterクエリに適用するポリゴンフィルター。
	// @param [in] frand乱数を返す関数[0..1）。
	// @param [out] randomRefランダムな場所の参照ID。
	// @param [out] randomPtランダムな場所。
	// @returnsクエリのステータスフラグ。
	// Returns random location on navmesh.
	// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	//  @param[in]		filter			The polygon filter to apply to the query.
	//  @param[in]		frand			Function returning a random number [0..1).
	//  @param[out]	randomRef		The reference id of the random location.
	//  @param[out]	randomPt		The random location.
	// @returns The status flags for the query.
	dtStatus findRandomPoint(const dtQueryFilter* filter, float (*frand)(),
		dtPolyRef* randomRef, float* randomPt) const;

	// 指定された場所の範囲内でnavmeshのランダムな場所を返します。
	// ポリゴンは、エリアごとに重み付けされて選択されます。検索は、ポリゴンの数に関連する線形で実行されます。
	// 位置は円によって厳密に制約されませんが、訪問したポリゴンを制限します。
	// @param [in] startRef検索を開始するポリゴンの参照ID。
	// @param [in] centerPos検索サークルの中心。[（x、y、z）]
	// @param [in] filterクエリに適用するポリゴンフィルター。
	// @param [in] frand乱数を返す関数[0..1）。
	// @param [out] randomRefランダムな場所の参照ID。
	// @param [out] randomPtランダムな場所。[（x、y、z）]
	// @returnsクエリのステータスフラグ。
	// Returns random location on navmesh within the reach of specified location.
	// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	// The location is not exactly constrained by the circle, but it limits the visited polygons.
	//  @param[in]		startRef		The reference id of the polygon where the search starts.
	//  @param[in]		centerPos		The center of the search circle. [(x, y, z)]
	//  @param[in]		filter			The polygon filter to apply to the query.
	//  @param[in]		frand			Function returning a random number [0..1).
	//  @param[out]	randomRef		The reference id of the random location.
	//  @param[out]	randomPt		The random location. [(x, y, z)]
	// @returns The status flags for the query.
	dtStatus findRandomPointAroundCircle(dtPolyRef startRef, const float* centerPos, const float maxRadius,
		const dtQueryFilter* filter, float (*frand)(),
		dtPolyRef* randomRef, float* randomPt) const;

	// 指定されたポリゴンで最も近いポイントを検索します。
	// @param [in] refポリゴンの参照ID。
	// @param [in] posチェックする位置。[（x、y、z）]
	// @param [out] nearestポリゴン上の最も近いポイント。[（x、y、z）]
	// @param [out] posOverPoly位置がポリゴン上にあることの真。
	// @returnsクエリのステータスフラグ。
	// Finds the closest point on the specified polygon.
	//  @param[in]		ref			The reference id of the polygon.
	//  @param[in]		pos			The position to check. [(x, y, z)]
	//  @param[out]	closest		The closest point on the polygon. [(x, y, z)]
	//  @param[out]	posOverPoly	True of the position is over the polygon.
	// @returns The status flags for the query.
	dtStatus closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;

	// ソースポイントが外側にある場合、ソースポイントに最も近い境界上のポイントを返します
	// ポリゴンのxz境界。
	// @param [in] refポリゴンへの参照ID。
	// @param [in] posチェックする位置。[（x、y、z）]
	// @param [out] nearest最も近いポイント。[（x、y、z）]
	// @returnsクエリのステータスフラグ。
	// Returns a point on the boundary closest to the source point if the source point is outside the
	// polygon's xz-bounds.
	//  @param[in]		ref			The reference id to the polygon.
	//  @param[in]		pos			The position to check. [(x, y, z)]
	//  @param[out]	closest		The closest point. [(x, y, z)]
	// @returns The status flags for the query.
	dtStatus closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, std::array<float, 3>* closest) const;

	// 高さの詳細を使用して、指定された位置のポリゴンの高さを取得します。（最も正確です。）
	//  @param [in] refポリゴンの参照ID。
	//  @param [in] posポリゴンのxz境界内の位置。[（x、y、z）]
	//  @param [out] heightポリゴンの表面の高さ。
	//  @returnsクエリのステータスフラグ。
	// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
	//  @param[in]		ref			The reference id of the polygon.
	//  @param[in]		pos			A position within the xz-bounds of the polygon. [(x, y, z)]
	//  @param[out]	height		The height at the surface of the polygon.
	// @returns The status flags for the query.
	dtStatus getPolyHeight(dtPolyRef ref, const float* pos, float* height) const;

	// @}
	// @name Miscellaneous Functions その他の関数
	// @{
	// ポリゴン参照が有効で、フィルター制限に合格した場合にtrueを返します。
	// @param [in] refチェックするポリゴン参照。
	// @param [in] filter適用するフィルター。
	// Returns true if the polygon reference is valid and passes the filter restrictions.
	//  @param[in]		ref			The polygon reference to check.
	//  @param[in]		filter		The filter to apply.
	bool isValidPolyRef(dtPolyRef ref, const dtQueryFilter* filter) const;

	// ポリゴン参照が閉じたリストにある場合、trueを返します。
	// @param [in] refチェックするポリゴンの参照ID。
	// @returnsポリゴンが閉じたリストにある場合はTrue。
	// Returns true if the polygon reference is in the closed list.
	//  @param[in]		ref		The reference id of the polygon to check.
	// @returns True if the polygon is in closed list.
	bool isInClosedList(dtPolyRef ref) const;

	// ノードプールを取得します。
	// @returnsノードプール。
	// Gets the node pool.
	// @returns The node pool.
	class dtNodePool* getNodePool() const { return m_nodePool; }

	// クエリオブジェクトが使用しているナビゲーションメッシュを取得します。
	// @returnクエリオブジェクトが使用しているナビゲーションメッシュ。
	// Gets the navigation mesh the query object is using.
	// @return The navigation mesh the query object is using.
	const dtNavMesh* getAttachedNavMesh() const { return m_nav; }

	// @}

private:
	// Explicitly disabled copy constructor and copy assignment operator
	dtNavMeshQuery(const dtNavMeshQuery&);
	dtNavMeshQuery& operator=(const dtNavMeshQuery&);

	// Queries polygons within a tile.
	// タイル内のポリゴンを照会します。
	void queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
		const dtQueryFilter* filter, dtPolyQuery* query) const;

	// Returns portal points between two polygons.
	dtStatus getPortalPoints(
		dtPolyRef from, dtPolyRef to, std::array<float, 3>* left, std::array<float, 3>* right,
		uint8_t& fromType, uint8_t& toType) const;
	dtStatus getPortalPoints(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
		dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
		std::array<float, 3>* left, std::array<float, 3>* right) const;

	// Returns edge mid point between two polygons.
	// 2つのポリゴン間のエッジの中点を返します。
	dtStatus getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const;
	// 2つのポリゴン間のエッジの中点を返します。
	dtStatus getEdgeMidPoint(dtPolyRef from, const dtPoly* fromPoly, const dtMeshTile* fromTile,
		dtPolyRef to, const dtPoly* toPoly, const dtMeshTile* toTile,
		std::array<float, 3>* mid) const;

	// Appends vertex to a straight path
	dtStatus appendVertex(const std::array<float, 3>& pos, const uint8_t flags, const dtPolyRef ref,
		float* straightPath, uint8_t* straightPathFlags, dtPolyRef* straightPathRefs,
		int* straightPathCount, const int maxStraightPath) const;

	// Appends intermediate portal points to a straight path.
	dtStatus appendPortals(
		const int startIdx, const int endIdx, const std::array<float, 3>* endPos, const dtPolyRef* path,
		float* straightPath, uint8_t* straightPathFlags, dtPolyRef* straightPathRefs,
		int* straightPathCount, const int maxStraightPath, const int options) const;

	// Gets the path leading to the specified end node.
	// 指定された終了ノードへのパスを取得します。
	dtStatus getPathToNode(struct dtNode* endNode, dtPolyRef* path, int* pathCount, int maxPath) const;

	const dtNavMesh* m_nav;				//< Pointer to navmesh data.

	struct dtQueryData
	{
		dtStatus status;
		struct dtNode* lastBestNode;
		float lastBestNodeCost;
		dtPolyRef startRef, endRef;
		std::array<float, 3> startPos, endPos;
		const dtQueryFilter* filter;
		unsigned int options;
		float raycastLimitSqr;
	};
	dtQueryData m_query;				//< Sliced query state. スライスされたクエリ状態。

	class dtNodePool* m_tinyNodePool;	//< Pointer to small node pool. スモールノードプールへのポインター
	class dtNodePool* m_nodePool;		//< Pointer to node pool.       ノードプールへのポインタ
	class dtNodeQueue* m_openList;		//< Pointer to open list queue. リストキューを開くためのポインター
};

// Allocates a query object using the Detour allocator.
// Detourアロケーターを使用してクエリオブジェクトを割り当てます。
// @return An allocated query object, or null on failure.
// @return割り当てられたクエリオブジェクト、または失敗した場合はnull。
// @ingroup detour
dtNavMeshQuery* dtAllocNavMeshQuery();

// Frees the specified query object using the Detour allocator.
// Detourアロケーターを使用して、指定されたクエリオブジェクトを解放します。
//  @param[in]		query		A query object allocated using #dtAllocNavMeshQuery
// @param [in] query #dtAllocNavMeshQueryを使用して割り当てられたクエリオブジェクト
// @ingroup detour
void dtFreeNavMeshQuery(dtNavMeshQuery* query);

#endif // DETOURNAVMESHQUERY_H

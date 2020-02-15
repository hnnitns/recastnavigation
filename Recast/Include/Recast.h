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

#ifndef RECAST_H
#define RECAST_H

#include <utility>
#include <algorithm>
#include <array>
#include <memory>
#include "DebugNewDef.h"
#include "RecastConfig.h"

// The value of PI used by Recast.
// Recastが使用するPIの値。
constexpr float RC_PI = 3.14159265f;

// Recast log categories.
// ログカテゴリをリキャストします。
// @see rcContext
enum rcLogCategory
{
	RC_LOG_PROGRESS = 1,	//< A progress log entry. // 進捗ログエントリ。
	RC_LOG_WARNING,			//< A warning log entry.  // 警告ログエントリ。
	RC_LOG_ERROR,			//< An error log entry.   // エラーログエントリ。
};

// Recast performance timer categories.
// パフォーマンスタイマーカテゴリをリキャストします。
// @see rcContext
enum rcTimerLabel
{
	// The user defined total time of the build.
	// ユーザー定義のビルドの合計時間。
	RC_TIMER_TOTAL,
	// A user defined build time.
	// ユーザー定義のビルド時間。
	RC_TIMER_TEMP,
	// The time to rasterize the triangles. (See: #rcRasterizeTriangle)
	// 三角形をラスタライズする時間。
	RC_TIMER_RASTERIZE_TRIANGLES,
	// The time to build the compact heightfield. (See: #rcBuildCompactHeightfield)
	// コンパクトな地形を構築する時間。
	RC_TIMER_BUILD_COMPACTHEIGHTFIELD,
	// The total time to build the contours. (See: #rcBuildContours)
	// 輪郭を構築するための合計時間。
	RC_TIMER_BUILD_CONTOURS,
	// The time to trace the boundaries of the contours. (See: #rcBuildContours)
	// 等高線の境界をトレースする時間。
	RC_TIMER_BUILD_CONTOURS_TRACE,
	// The time to simplify the contours. (See: #rcBuildContours)
	// 輪郭を単純化する時間。
	RC_TIMER_BUILD_CONTOURS_SIMPLIFY,
	// The time to filter ledge spans. (See: #rcFilterLedgeSpans)
	// レッジスパンをフィルタリングする時間。
	RC_TIMER_FILTER_BORDER,
	// The time to filter low height spans. (See: #rcFilterWalkableLowHeightSpans)
	// 低い高さのスパンをフィルタリングする時間。
	RC_TIMER_FILTER_WALKABLE,
	// The time to apply the median filter. (See: #rcMedianFilterWalkableArea)
	// 中央値フィルターを適用する時間。
	RC_TIMER_MEDIAN_AREA,
	// The time to filter low obstacles. (See: #rcFilterLowHangingWalkableObstacles)
	// 低障害物をフィルタリングする時間。
	RC_TIMER_FILTER_LOW_OBSTACLES,
	// The time to build the polygon mesh. (See: #rcBuildPolyMesh)
	// ポリゴンメッシュを構築する時間。
	RC_TIMER_BUILD_POLYMESH,
	// The time to merge polygon meshes. (See: #rcMergePolyMeshes)
	// ポリゴンメッシュをマージする時間。
	RC_TIMER_MERGE_POLYMESH,
	// The time to erode the walkable area. (See: #rcErodeWalkableArea)
	// 歩行可能エリアを侵食する時間。
	RC_TIMER_ERODE_AREA,
	// The time to mark a box area. (See: #rcMarkBoxArea)
	// ボックス領域をマークする時間。
	RC_TIMER_MARK_BOX_AREA,
	// The time to mark a cylinder area. (See: #rcMarkCylinderArea)
	// 円柱領域をマークする時間。
	RC_TIMER_MARK_CYLINDER_AREA,
	// The time to mark a convex polygon area. (See: #rcMarkConvexPolyArea)
	// 凸多角形領域をマークする時間。
	RC_TIMER_MARK_CONVEXPOLY_AREA,
	// The total time to build the distance field. (See: #rcBuildDistanceField)
	// 距離フィールドを構築するための合計時間。
	RC_TIMER_BUILD_DISTANCEFIELD,
	// The time to build the distances of the distance field. (See: #rcBuildDistanceField)
	// 距離フィールドの距離を構築する時間。
	RC_TIMER_BUILD_DISTANCEFIELD_DIST,
	// The time to blur the distance field. (See: #rcBuildDistanceField)
	// 距離フィールドをぼかす時間。
	RC_TIMER_BUILD_DISTANCEFIELD_BLUR,
	// The total time to build the regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
	// 領域を構築するための合計時間。
	RC_TIMER_BUILD_REGIONS,
	// The total time to apply the watershed algorithm. (See: #rcBuildRegions)
	// 流域アルゴリズムを適用する合計時間。
	RC_TIMER_BUILD_REGIONS_WATERSHED,
	// The time to expand regions while applying the watershed algorithm. (See: #rcBuildRegions)
	// 流域アルゴリズムを適用しながら領域を拡大する時間。
	RC_TIMER_BUILD_REGIONS_EXPAND,
	// The time to flood regions while applying the watershed algorithm. (See: #rcBuildRegions)
	// 流域アルゴリズムを適用しながら領域をフラッディングする時間。
	RC_TIMER_BUILD_REGIONS_FLOOD,
	// The time to filter out small regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
	// 小さな領域を除外する時間。
	RC_TIMER_BUILD_REGIONS_FILTER,
	// The time to build heightfield layers. (See: #rcBuildHeightfieldLayers)
	// 地形レイヤーを構築する時間。
	RC_TIMER_BUILD_LAYERS,
	// The time to build the polygon mesh detail. (See: #rcBuildPolyMeshDetail)
	// ポリゴンメッシュの詳細を作成する時間。
	RC_TIMER_BUILD_POLYMESHDETAIL,
	// The time to merge polygon mesh details. (See: #rcMergePolyMeshDetails)
	// ポリゴンメッシュの詳細をマージする時間。
	RC_TIMER_MERGE_POLYMESHDETAIL,
	// The maximum number of timers.  (Used for iterating timers.)
	// タイマーの最大数。 （タイマーの反復に使用されます。）
	RC_MAX_TIMERS
};

// Provides an interface for optional logging and performance tracking of the Recast
// build process.
// Recastビルドプロセスのオプションのログとパフォーマンス追跡用のインターフェイスを提供します。
// @ingroup recast
class rcContext
{
public:

	// Contructor.
	//  @param[in]		state	TRUE if the logging and performance timers should be enabled.  [Default: true]
	// ログおよびパフォーマンスタイマーを有効にする必要がある場合はTRUE。[デフォルト：true]
	inline rcContext(bool state = true) : m_logEnabled(state), m_timerEnabled(state) {}
	virtual ~rcContext() {}

	// Enables or disables logging.
	// ログを有効または無効にします。
	//  @param[in]		state	TRUE if logging should be enabled. // ログを有効にする必要がある場合はTRUE。
	inline void enableLog(bool state) { m_logEnabled = state; }

	// Clears all log entries.
	// すべてのログエントリをクリアします。
	inline void resetLog() { if (m_logEnabled) doResetLog(); }

	// Logs a message.
	// メッセージを記録します。
	//  @param[in]		category	The category of the message. // メッセージのカテゴリ。
	//  @param[in]		format		The message.
	void log(const rcLogCategory category, const char* format, ...);

	// Enables or disables the performance timers.
	// パフォーマンスタイマーを有効または無効にします。
	//  @param[in]		state	TRUE if timers should be enabled. // タイマーを有効にする必要がある場合はTRUE。
	inline void enableTimer(bool state) { m_timerEnabled = state; }

	// Clears all peformance timers. (Resets all to unused.)
	// すべてのパフォーマンスタイマーをクリアします。（すべてを未使用にリセットします。）
	inline void resetTimers() { if (m_timerEnabled) doResetTimers(); }

	// Starts the specified performance timer.
	// 指定されたパフォーマンスタイマーを開始します。
	//  @param	label	The category of the timer. // タイマーのカテゴリ。
	inline void startTimer(const rcTimerLabel label) { if (m_timerEnabled) doStartTimer(label); }

	// Stops the specified performance timer.
	// 指定されたパフォーマンスタイマーを停止します。
	//  @param	label	The category of the timer. // タイマーのカテゴリ。
	inline void stopTimer(const rcTimerLabel label) { if (m_timerEnabled) doStopTimer(label); }

	// Returns the total accumulated time of the specified performance timer.
	// 指定したパフォーマンスタイマーの合計累積時間を返します。
	// @param	label	The category of the timer. // タイマーのカテゴリ。
	// @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
	// タイマーの累積時間。タイマーが無効になっているか、タイマーが開始されていない場合は-1。
	inline int getAccumulatedTime(const rcTimerLabel label) const { return m_timerEnabled ? doGetAccumulatedTime(label) : -1; }

protected:

	// Clears all log entries.
	// すべてのログエントリをクリアします。
	virtual void doResetLog() {}

	// Logs a message.
	// メッセージを記録します。
	//  @param[in]		category	The category of the message. // メッセージのカテゴリ。
	//  @param[in]		msg			The formatted message. // フォーマットされたメッセージ。
	//  @param[in]		len			The length of the formatted message. // フォーマットされたメッセージの長さ。
	virtual void doLog(const rcLogCategory /*category*/, const char* /*msg*/, const int /*len*/) {}

	// Clears all timers. (Resets all to unused.)
	// すべてのタイマーをクリアします。（すべてを未使用にリセットします。）
	virtual void doResetTimers() {}

	// Starts the specified performance timer.
	// 指定されたパフォーマンスタイマーを開始します。
	//  @param[in]		label	The category of timer. // タイマーのカテゴリ。
	virtual void doStartTimer(const rcTimerLabel /*label*/) {}

	// Stops the specified performance timer.
	// 指定されたパフォーマンスタイマーを停止します。
	//  @param[in]		label	The category of the timer. // タイマーのカテゴリ。
	virtual void doStopTimer(const rcTimerLabel /*label*/) {}

	// Returns the total accumulated time of the specified performance timer.
	// 指定したパフォーマンスタイマーの合計累積時間を返します。
	//  @param[in]		label	The category of the timer. // タイマーのカテゴリ。
	//  @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
	//  タイマーの累積時間。タイマーが無効になっているか、タイマーが開始されていない場合は-1。
	virtual int doGetAccumulatedTime(const rcTimerLabel /*label*/) const { return -1; }

	// True if logging is enabled.
	// ログが有効な場合はTrue。
	bool m_logEnabled;

	// True if the performance timers are enabled.
	// パフォーマンスタイマーが有効になっている場合はtrue。
	bool m_timerEnabled;
};

// A helper to first start a timer and then stop it when this helper goes out of scope.
// 最初にタイマーを開始し、このヘルパーが範囲外になったときにタイマーを停止するヘルパー。
// @see rcContext
class rcScopedTimer
{
public:
	// Constructs an instance and starts the timer.
	// インスタンスを構築し、タイマーを開始します。
	//  @param[in]		ctx		The context to use. 使用するコンテキスト。
	//  @param[in]		label	The category of the timer. タイマーのカテゴリー。
	inline rcScopedTimer(rcContext* ctx, const rcTimerLabel label) : m_ctx(ctx), m_label(label) { m_ctx->startTimer(m_label); }
	inline ~rcScopedTimer() { m_ctx->stopTimer(m_label); }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcScopedTimer(const rcScopedTimer&) = delete;
	rcScopedTimer& operator=(const rcScopedTimer&) = delete;
	rcScopedTimer(rcScopedTimer&&) noexcept = delete;
	rcScopedTimer& operator=(rcScopedTimer&&) noexcept = delete;

	rcContext* const m_ctx;
	const rcTimerLabel m_label;
};

// Specifies a configuration to use when performing Recast builds.
// リキャストビルドを実行するときに使用する構成を指定します。
// @ingroup recast
struct rcConfig
{
	// The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
	// x軸に沿ったフィールドの幅。[制限：> = 0] [単位：vx]
	int width;

	// The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
	// z軸に沿ったフィールドの高さ。[制限：> = 0] [単位：vx]
	int height;

	// The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx]
	// xz平面上のタイルの幅/高さのサイズ。[制限：> = 0] [単位：vx]
	int tileSize;

	// The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
	// 地形の周りのナビゲートできない境界のサイズ。[制限：> = 0] [単位：vx]
	int borderSize;

	// The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu]
	// フィールドに使用するxz平面のセルサイズ。[制限：> 0] [単位：wu]
	float cs;

	// The y-axis cell size to use for fields. [Limit: > 0] [Units: wu]
	// フィールドに使用するy軸のセルサイズ。[制限：> 0] [単位：wu]
	float ch;

	// The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	// フィールドのAABBの最小境界。[（x、y、z）] [単位：wu]
	std::array<float, 3> bmin;

	// The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	// フィールドのAABBの最大境界。[（x、y、z）] [単位：wu]
	std::array<float, 3> bmax;

	// The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees]
	// 歩行可能と見なされる最大勾配。[制限：0 <=値<90] [単位：度]
	float walkableSlopeAngle;

	// Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3] [Units: vx]
	// 床面積が歩行可能と見なされるようにする最小床から「天井」までの高さ。 [制限：> = 3] [単位：vx]
	int walkableHeight;

	// Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx]
	// まだ通過可能であると見なされる最大棚の高さ。[制限：> = 0] [単位：vx]
	int walkableClimb;

	// The distance to erode/shrink the walkable area of the heightfield away from obstructions.  [Limit: >=0] [Units: vx]
	// 地形の歩行可能領域を障害物から侵食/収縮する距離。 [制限：> = 0] [単位：vx]
	int walkableRadius;

	// The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx]
	// メッシュの境界に沿った輪郭エッジの最大許容長。[制限：> = 0] [単位：vx]
	int maxEdgeLen;

	// The maximum distance a simplfied contour's border edges should deviate the original raw contour. [Limit: >=0] [Units: vx]
	// 単純化された輪郭の境界エッジの最大距離は、元の生の輪郭から逸脱するはずです。 [制限：> = 0] [単位：vx]
	float maxSimplificationError;

	// The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx]
	// 孤立した島の領域を形成できるセルの最小数。[制限：> = 0] [単位：vx]
	int minRegionArea;

	// Any regions with a span count smaller than this value will, if possible,
	// be merged with larger regions. [Limit: >=0] [Units: vx]
	// 可能な場合、スパンカウントがこの値よりも小さい領域は、より大きな領域とマージされます。 [制限：> = 0] [単位：vx]
	int mergeRegionArea;

	// The maximum number of vertices allowed for polygons generated during the
	// contour to polygon conversion process. [Limit: >= 3]
	// 輪郭からポリゴンへの変換プロセス中に生成されるポリゴンに許可される頂点の最大数。 [制限：> = 3]
	int maxVertsPerPoly;

	// Sets the sampling distance to use when generating the detail mesh.
	// (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu]
	//詳細メッシュを生成するときに使用するサンプリング距離を設定します。（高さの詳細のみ）[制限：0または> = 0.9] [単位：wu]
	float detailSampleDist;

	// The maximum distance the detail mesh surface should deviate from heightfield
	// data. (For height detail only.) [Limit: >=0] [Units: wu]
	// 詳細メッシュの表面が地形データから逸脱する最大距離。（高さの詳細のみ）[制限：> = 0] [単位：wu]
	float detailSampleMaxError;
};

// Defines the maximum value for rcSpan::smin and rcSpan::smax.
// rcSpan :: sminおよびrcSpan :: smaxの最大値を定義します。
constexpr int RC_SPAN_MAX_HEIGHT = (1 << RC_SPAN_HEIGHT_BITS) - 1;

// The number of spans allocated per span spool.
// スパンスプールごとに割り当てられたスパンの数。
// @see rcSpanPool
constexpr int RC_SPANS_PER_POOL = 2048;

// Represents a span in a heightfield.
// スパンスプールごとに割り当てられたスパンの数。
// @see rcHeightfield
struct rcSpan
{
	// The lower limit of the span. [Limit: < #smax]
	// スパンの下限。[制限：<#smax]
	uint32_t smin : RC_SPAN_HEIGHT_BITS;

	// The upper limit of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT]
	// スパンの上限。[制限：<= #RC_SPAN_MAX_HEIGHT]
	uint32_t smax : RC_SPAN_HEIGHT_BITS;

	// The area id assigned to the span.
	// スパンに割り当てられたエリアID。
	uint32_t area : 6;

	// The next span higher up in column.
	// 列の上の次のスパン。
	rcSpan* next;
};

// A memory pool used for quick allocation of spans within a heightfield.
// 地形内のスパンの迅速な割り当てに使用されるメモリプール。
// @see rcHeightfield
struct rcSpanPool
{
	rcSpanPool* next;					// The next span pool. // 次のスパンプール。
	std::array<rcSpan, RC_SPANS_PER_POOL> items;	// Array of spans in the pool. // プール内のスパンの配列。
};

// A dynamic heightfield representing obstructed space.
// 遮られた空間を表す動的な地形。
// @ingroup recast
struct rcHeightfield
{
	rcHeightfield();
	~rcHeightfield();

	// The width of the heightfield. (Along the x-axis in cell units.)
	// 地形の幅。（セル単位のx軸に沿って。）
	int width;

	// The height of the heightfield. (Along the z-axis in cell units.)
	// 地形の高さ。（セル単位のz軸に沿って。）
	int height;

	// The minimum bounds in world space. [(x, y, z)]
	// ワールド空間の最小境界。[（x、y、z）]
	std::array<float, 3> bmin;

	// The maximum bounds in world space. [(x, y, z)]
	// ワールド空間の最大境界。[（x、y、z）]
	std::array<float, 3> bmax;

	// The size of each cell. (On the xz-plane.)
	// 各セルのサイズ。（xz平面上。）
	float cs;

	// The height of each cell. (The minimum increment along the y-axis.)
	// 各セルの高さ。（y軸に沿った最小増分。）
	float ch;

	// Heightfield of spans (width*height).
	// スパンの地形（width * height）。
	rcSpan** spans;

	// Linked list of span pools.
	// スパンプールのリンクリスト。
	rcSpanPool* pools;

	// The next free span.
	// 次の空きスパン。
	rcSpan* freelist;

private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	// 明示的に無効化されたコピーコンストラクターとコピー割り当て演算子。
	rcHeightfield(const rcHeightfield&) = delete;
	rcHeightfield& operator=(const rcHeightfield&) = delete;
};

// Provides information on the content of a cell column in a compact heightfield.
// コンパクトな地形のセル列のコンテンツに関する情報を提供します。
struct rcCompactCell
{
	uint32_t index : 24;	// Index to the first span in the column. // 列の最初のスパンへのインデックス。
	uint32_t count : 8;		// Number of spans in the column. // 列内のスパンの数。
};

// Represents a span of unobstructed space within a compact heightfield.
// コンパクトな地形内の遮るもののない空間の範囲を表します。
struct rcCompactSpan
{
	// The lower extent of the span. (Measured from the heightfield's base.)
	// スパンの下限。（ハイトフィールドのベースから測定。）
	uint16_t y;

	// The id of the region the span belongs to. (Or zero if not in a region.)
	// スパンが属する領域のID。（または、地域にない場合はゼロ。）
	uint16_t reg;

	// Packed neighbor connection data.
	// パックされたネイバー接続データ。
	uint32_t con : 24;

	// The height of the span.  (Measured from #y.)
	// スパンの高さ。（#yから測定）
	uint32_t h : 8;
};

// 遮られていない空間を表すコンパクトで静的な地形。
// A compact, static heightfield representing unobstructed space.
// @ingroup recast
struct rcCompactHeightfield
{
	// The width of the heightfield. (Along the x-axis in cell units.)
	// 地形の幅。（セル単位のx軸に沿って）
	int width;

	// The height of the heightfield. (Along the z-axis in cell units.)
	// 地形の高さ。（セル単位のz軸に沿って）
	int height;

	// The number of spans in the heightfield.
	// 地形のスパン(人間の手を基準とした長さの単位)の数
	int spanCount;

	// The walkable height used during the build of the field.  (See: rcConfig::walkableHeight)
	// フィールドの構築中に使用される歩行可能な高さ
	int walkableHeight;

	// The walkable climb used during the build of the field. (See: rcConfig::walkableClimb)
	// フィールドの構築中に使用される歩行可能な上昇
	int walkableClimb;

	// The AABB border size used during the build of the field. (See: rcConfig::borderSize)
	// フィールドのビルド中に使用されるAABB境界サイズ
	int borderSize;

	// The maximum distance value of any span within the field.
	// フィールド内の任意のスパンの最大距離値
	uint16_t maxDistance;

	// The maximum region id of any span within the field.
	// フィールド内の任意のスパンの最大領域ID
	uint16_t maxRegions;

	// The minimum bounds in world space. [(x, y, z)]
	// ワールド空間の最小境界。[（x、y、z）]
	std::array<float, 3> bmin;

	// The maximum bounds in world space. [(x, y, z)]
	// ワールド空間の最大境界。[（x、y、z）]
	std::array<float, 3> bmax;

	// The size of each cell. (On the xz-plane.)
	// 各セルのサイズ。（xz平面上。）
	float cs;

	// The height ofeach cell. (The minimum increment along the y-axis.)
	// 各セルの高さ。（y軸に沿った最小増分。）
	float ch;

	// Array of cells. [Size: #width*#height]
	// セルの配列
	rcCompactCell* cells;

	// Array of spans. [Size: #spanCount]
	// スパンの配列
	rcCompactSpan* spans;

	// Array containing border distance data. [Size: #spanCount]
	// 境界距離データを含む配列
	uint16_t* dist;

	// Array containing area id data. [Size: #spanCount]
	// エリアIDデータを含む配列
	uint8_t* areas;
};

// Represents a heightfield layer within a layer set.
// レイヤーセット内の地形レイヤーを表します。
// @see rcHeightfieldLayerSet
struct rcHeightfieldLayer
{
	// The minimum bounds in world space. [(x, y, z)]
	// ワールド空間の最小境界。[（x、y、z）]
	std::array<float, 3> bmin;

	// The maximum bounds in world space. [(x, y, z)]
	// ワールド空間の最大境界。[（x、y、z）]
	std::array<float, 3> bmax;

	// The size of each cell. (On the xz-plane.)
	// 各セルのサイズ。（xz平面上）
	float cs;

	// The height of each cell. (The minimum increment along the y-axis.)
	// 各セルの高さ。（y軸に沿った最小増分）
	float ch;

	// The width of the heightfield. (Along the x-axis in cell units.)
	// 地形の幅。（セル単位のx軸に沿う）
	int width;

	// The height of the heightfield. (Along the z-axis in cell units.)
	// 地形の高さ。（セル単位のz軸に沿う）
	int height;

	// The minimum x-bounds of usable data.
	// 使用可能なデータの最小x境界。
	int minx;

	// The maximum x-bounds of usable data.
	// 使用可能なデータの最大x境界。
	int maxx;

	// The minimum y-bounds of usable data. (Along the z-axis.)
	// 使用可能なデータの最小y境界。（z軸に沿う）
	int miny;

	// The maximum y-bounds of usable data. (Along the z-axis.)
	// 使用可能なデータの最大y境界。（z軸に沿う）
	int maxy;

	// The minimum height bounds of usable data. (Along the y-axis.)
	// 使用可能なデータの最小の高さの境界。（y軸に沿う）
	int hmin;

	// The maximum height bounds of usable data. (Along the y-axis.)
	// 使用可能なデータの最大高さの境界。（y軸に沿う）
	int hmax;

	// The heightfield. [Size: width * height]
	// 地形。[サイズ：幅 * 高さ]
	uint8_t* heights;

	// Area ids. [Size: Same as #heights]
	// エリアID。[サイズ：#heightsと同じ]
	uint8_t* areas;

	// Packed neighbor connection information. [Size: Same as #heights]
	// パックされたネイバー接続情報。[サイズ：#heightsと同じ]
	uint8_t* cons;
};

// Represents a set of heightfield layers.
// 地形レイヤーのセットを表します。
// @ingroup recast
// @see rcAllocHeightfieldLayerSet, rcFreeHeightfieldLayerSet
struct rcHeightfieldLayerSet
{
	rcHeightfieldLayer* layers;			// The layers in the set. [Size: #nlayers] // セット内のレイヤー。[サイズ：#nlayers]
	int nlayers;						// The number of layers in the set. // セット内のレイヤーの数。
};

// Represents a simple, non-overlapping contour in field space.
// フィールド空間内の単純な重複しない輪郭を表します。
struct rcContour
{
	// Simplified contour vertex and connection data. [Size: 4 * #nverts]
	// 簡略化された輪郭の頂点と接続データ。[サイズ：4 * #nverts]
	int* verts;

	// The number of vertices in the simplified contour.
	// 単純化された輪郭の頂点の数。
	int nverts;

	// Raw contour vertex and connection data. [Size: 4 * #nrverts]
	// 生の輪郭の頂点と接続データ。[サイズ：4 * #nrverts]
	int* rverts;

	// The number of vertices in the raw contour.
	// 生の輪郭の頂点の数。
	int nrverts;

	// The region id of the contour.
	// 輪郭の領域ID。
	uint16_t reg;

	// The area id of the contour.
	// 輪郭のエリアID。
	uint8_t area;
};

// Represents a group of related contours.
// 関連する輪郭のグループを表します。
// @ingroup recast
struct rcContourSet
{
	// An array of the contours in the set. [Size: #nconts]
	// セット内の輪郭の配列。[サイズ：#nconts]
	rcContour* conts;

	// The number of contours in the set.
	// セット内の輪郭の数。
	int nconts;

	// The minimum bounds in world space. [(x, y, z)]
	// ワールド空間の最小境界。[（x、y、z）]
	std::array<float, 3> bmin;

	// The maximum bounds in world space. [(x, y, z)]
	// ワールド空間の最大境界。[（x、y、z）]
	std::array<float, 3> bmax;

	// The size of each cell. (On the xz-plane.)
	// 各セルのサイズ。（xz平面上）
	float cs;

	// The height of each cell. (The minimum increment along the y-axis.)
	// 各セルの高さ。（y軸に沿った最小増分）
	float ch;

	// The width of the set. (Along the x-axis in cell units.)
	// セットの幅。（セル単位のx軸に沿う）
	int width;

	// The height of the set. (Along the z-axis in cell units.)
	// セットの高さ。（セル単位のz軸に沿う）
	int height;

	// The AABB border size used to generate the source data from which the contours were derived.
	// 輪郭の派生元のソースデータを生成するために使用されるAABB境界サイズ。
	int borderSize;

	// The max edge error that this contour set was simplified with.
	// この輪郭セットが単純化された最大エッジエラー。
	float maxError;
};

// Represents a polygon mesh suitable for use in building a navigation mesh.
// ナビゲーションメッシュの構築に使用するのに適したポリゴンメッシュを表します。
// @ingroup recast
struct rcPolyMesh
{
	// The mesh vertices. [Form: (x, y, z) * #nverts]
	// メッシュの頂点。[形式：（x、y、z）* #nverts]
	uint16_t* verts;

	// Polygon and neighbor data. [Length: #maxpolys * 2 * #nvp]
	// ポリゴンと近傍データ。[長さ：#maxpolys * 2 * #nvp]
	uint16_t* polys;

	// The region id assigned to each polygon. [Length: #maxpolys]
	// 各ポリゴンに割り当てられたリージョンID。[長さ：#maxpolys]
	uint16_t* regs;

	// The user defined flags for each polygon. [Length: #maxpolys]
	// 各ポリゴンのユーザー定義フラグ。[長さ：#maxpolys]
	uint16_t* flags;

	// The area id assigned to each polygon. [Length: #maxpolys]
	// 各ポリゴンに割り当てられたエリアID。[長さ：#maxpolys]
	uint8_t* areas;

	// The number of vertices.
	// 頂点の数。
	int nverts;

	// The number of polygons.
	// ポリゴンの数。
	int npolys;

	// The number of allocated polygons.
	// 割り当てられたポリゴンの数。
	int maxpolys;

	// The maximum number of vertices per polygon.
	// ポリゴンごとの頂点の最大数。
	int nvp;

	// The minimum bounds in world space. [(x, y, z)]
	// ワールド空間の最小境界。[（x、y、z）]
	std::array<float, 3> bmin;

	// The maximum bounds in world space. [(x, y, z)]
	// ワールド空間の最大境界。[（x、y、z）]
	std::array<float, 3> bmax;

	// The size of each cell. (On the xz-plane.)
	// 各セルのサイズ。（xz平面上。）
	float cs;

	// The height of each cell. (The minimum increment along the y-axis.)
	// 各セルの高さ。（y軸に沿った最小増分。）
	float ch;

	// The AABB border size used to generate the source data from which the mesh was derived.
	// メッシュの派生元のソースデータを生成するために使用されるAABB境界サイズ。
	int borderSize;

	// The max error of the polygon edges in the mesh.
	// メッシュ内のポリゴンエッジの最大誤差。
	float maxEdgeError;
};

// Contains triangle meshes that represent detailed height data associated
// with the polygons in its associated polygon mesh object.
// 関連付けられたポリゴンメッシュオブジェクト内のポリゴンに関連付けられた詳細な高さデータを表す三角形メッシュが含まれます。
// @ingroup recast
struct rcPolyMeshDetail
{
	// The sub-mesh data. [Size: 4*#nmeshes]
	// サブメッシュデータ。[サイズ：4 *＃nmeshes]
	uint32_t* meshes;

	// The mesh vertices. [Size: 3*#nverts]
	// メッシュの頂点。[サイズ：3 *＃nverts]
	float* verts;

	// The mesh triangles. [Size: 4*#ntris]
	// メッシュの三角形。[サイズ：4 *＃ntris]
	uint8_t* tris;

	// The number of sub-meshes defined by #meshes.
	// #meshesで定義されたサブメッシュの数。
	int nmeshes;

	// The number of vertices in #verts.
	// #vertsの頂点の数。
	int nverts;

	// The number of triangles in #tris.
	// #trisの三角形の数。
	int ntris;
};

constexpr uint8_t RC_AREA_FLAGS_MASK = 0x3F;

// @ingroup recast
class rcAreaModification
{
public:
	// Mask is set to all available bits, which means value is fully applied
	// マスクは使用可能なすべてのビットに設定されます。つまり、値は完全に適用されます
	//  @param[in] value	The area id to apply. [Limit: <= #RC_AREA_FLAGS_MASK]
	//  @param [in] value適用するエリアID。 [制限：<= #RC_AREA_FLAGS_MASK]
	rcAreaModification(uint8_t value);
	//  @param[in] value	The area id to apply. [Limit: <= #RC_AREA_FLAGS_MASK]
	//  @param [in] value適用するエリアID。 [制限：<= #RC_AREA_FLAGS_MASK]
	//  @param[in] mask	Bitwise mask used when applying value. [Limit: <= #RC_AREA_FLAGS_MASK]
	//  @param [in] mask値を適用するときに使用されるビット単位のマスク。 [制限：<= #RC_AREA_FLAGS_MASK]
	rcAreaModification(uint8_t value, uint8_t mask);
	rcAreaModification(const rcAreaModification& other);
	void operator = (const rcAreaModification& other);
	bool operator == (const rcAreaModification& other) const;
	bool operator != (const rcAreaModification& other) const;
	void apply(uint8_t& area) const;
	uint8_t getMaskedValue() const;

	uint8_t m_value;	// Value to apply to target area id // ターゲットエリアIDに適用する値
	uint8_t m_mask;	// Bitwise mask used when applying value to target area id //ターゲットエリアIDに値を適用するときに使用されるビット単位のマスク
};

// @name Allocation Functions
// Functions used to allocate and de-allocate Recast objects.
// Recastオブジェクトの割り当てと割り当て解除に使用される関数。
// @see rcAllocSetCustom
// @{
// Allocates a heightfield object using the Recast allocator.
// Recastアロケーターを使用して、heightfieldオブジェクトを割り当てます。
//  @return A heightfield that is ready for initialization, or null on failure.
//	初期化の準備ができている高さフィールド、または失敗した場合はnull。
//  @ingroup recast
//  @see rcCreateHeightfield, rcFreeHeightField
rcHeightfield* rcAllocHeightfield();

// Frees the specified heightfield object using the Recast allocator.
// Recastアロケーターを使用して、指定されたheightfieldオブジェクトを解放します。
//  @param[in]		hf	A heightfield allocated using #rcAllocHeightfield
//  #rcAllocHeightfieldを使用して割り当てられた高さフィールド
//  @ingroup recast
//  @see rcAllocHeightfield
void rcFreeHeightField(rcHeightfield* hf);

// Allocates a compact heightfield object using the Recast allocator.
// Recastアロケーターを使用して、コンパクトなheightfieldオブジェクトを割り当てます。
// @return A compact heightfield that is ready for initialization, or null on failure.
// 初期化の準備ができているコンパクトな高さフィールド、または失敗した場合はnull。
// @ingroup recast
// @see rcBuildCompactHeightfield, rcFreeCompactHeightfield
rcCompactHeightfield* rcAllocCompactHeightfield();

// Frees the specified compact heightfield object using the Recast allocator.
// Recastアロケーターを使用して、指定されたコンパクトなheightfieldオブジェクトを解放します。
//  @param[in]		chf		A compact heightfield allocated using #rcAllocCompactHeightfield
//  #rcAllocCompactHeightfieldを使用して割り当てられたコンパクトな高さフィールド
//  @ingroup recast
//  @see rcAllocCompactHeightfield
void rcFreeCompactHeightfield(rcCompactHeightfield* chf);

// Allocates a heightfield layer set using the Recast allocator.
// Recastアロケーターを使用して、地形レイヤーセットを割り当てます。
// @return A heightfield layer set that is ready for initialization, or null on failure.
// 初期化の準備ができている地形レイヤーセット、または失敗した場合はnull。
// @ingroup recast
// @see rcBuildHeightfieldLayers, rcFreeHeightfieldLayerSet
rcHeightfieldLayerSet* rcAllocHeightfieldLayerSet();

// Frees the specified heightfield layer set using the Recast allocator.
// Recastアロケーターを使用して、指定されたheightfieldレイヤーセットを解放します。
//  @param[in]		lset	A heightfield layer set allocated using #rcAllocHeightfieldLayerSet
//  #rcAllocHeightfieldLayerSetを使用して割り当てられた高さフィールドレイヤーセット
//  @ingroup recast
//  @see rcAllocHeightfieldLayerSet
void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* lset);

// Allocates a contour set object using the Recast allocator.
// Recastアロケーターを使用して輪郭セットオブジェクトを割り当てます。
//  @return A contour set that is ready for initialization, or null on failure.
//  初期化の準備ができている輪郭セット、または失敗した場合はnull。
//  @ingroup recast
//  @see rcBuildContours, rcFreeContourSet
rcContourSet* rcAllocContourSet();

// Frees the specified contour set using the Recast allocator.
// Recastアロケーターを使用して、指定された輪郭セットを解放します。
//  @param[in]		cset	A contour set allocated using #rcAllocContourSet
//  @param[in]		#rcAllocContourSetを使用して割り当てられた輪郭セット
//  @ingroup recast
//  @see rcAllocContourSet
void rcFreeContourSet(rcContourSet* cset);

// Allocates a polygon mesh object using the Recast allocator.
// Recastアロケーターを使用してポリゴンメッシュオブジェクトを割り当てます。
//  @return A polygon mesh that is ready for initialization, or null on failure.
//  初期化の準備ができているポリゴンメッシュ、または失敗した場合はnull。
//  @ingroup recast
//  @see rcBuildPolyMesh, rcFreePolyMesh
rcPolyMesh* rcAllocPolyMesh();

// Frees the specified polygon mesh using the Recast allocator.
// Recastアロケーターを使用して、指定されたポリゴンメッシュを解放します。
//  @param[in]		pmesh	A polygon mesh allocated using #rcAllocPolyMesh
//  #rcAllocPolyMeshを使用して割り当てられたポリゴンメッシュ
//  @ingroup recast
//  @see rcAllocPolyMesh
void rcFreePolyMesh(rcPolyMesh* pmesh);

// Allocates a detail mesh object using the Recast allocator.
// Recastアロケーターを使用して詳細メッシュオブジェクトを割り当てます。
//  @return A detail mesh that is ready for initialization, or null on failure.
//  初期化の準備ができている詳細メッシュ、または失敗した場合はnull。
//  @ingroup recast
//  @see rcBuildPolyMeshDetail, rcFreePolyMeshDetail
rcPolyMeshDetail* rcAllocPolyMeshDetail();

// Frees the specified detail mesh using the Recast allocator.
// Recastアロケーターを使用して、指定された詳細メッシュを解放します。
//  @param[in]		dmesh	A detail mesh allocated using #rcAllocPolyMeshDetail
//  #rcAllocPolyMeshDetailを使用して割り当てられた詳細メッシュ
//  @ingroup recast
//  @see rcAllocPolyMeshDetail
void rcFreePolyMeshDetail(rcPolyMeshDetail* dmesh);

// @}

// Heighfield border flag.
// 地形境界フラグ。
// If a heightfield region ID has this bit set, then the region is a border region and its spans are considered unwalkable.
// 地形の領域IDにこのビットが設定されている場合、その領域は境界領域であり、そのスパンは歩行不能と見なされます。
// (Used during the region and contour build process.)
//（領域および輪郭の構築プロセス中に使用されます。）
// @see rcCompactSpan::reg
constexpr uint16_t RC_BORDER_REG = 0x8000;

// Polygon touches multiple regions.
// ポリゴンは複数の領域に接触します。
// If a polygon has this region ID it was merged with or created from polygons of different regions during the polymesh
// build step that removes redundant border vertices.
// ポリゴンにこの領域IDがある場合、冗長な境界頂点を削除するpolymeshビルドステップ中に、異なる領域のポリゴンとマージまたは作成されました。
// (Used during the polymesh and detail polymesh build processes)
//（polymeshおよび詳細polymeshビルドプロセス中に使用）
// @see rcPolyMesh::regs
constexpr uint16_t RC_MULTIPLE_REGS = 0;

// Border vertex flag.
// 境界頂点フラグ。
// If a region ID has this bit set, then the associated element lies on a tile border.
// 領域IDにこのビットが設定されている場合、関連する要素はタイルの境界線上にあります。
// If a contour vertex's region ID has this bit set,
// 等高線の頂点の領域IDにこのビットが設定されている場合、
// the vertex will later be removed in order to match the segments and vertices at tile boundaries.
// タイルの境界でセグメントと頂点を一致させるために、後で頂点が削除されます。
// (Used during the build process.)
//（ビルドプロセス中に使用されます。）
// @see rcCompactSpan::reg, #rcContour::verts, #rcContour::rverts
constexpr int RC_BORDER_VERTEX = 0x10000;

// Area border flag.
// エリア境界フラグ。
// If a region ID has this bit set, then the associated element lies on the border of an area.
// 領域IDにこのビットが設定されている場合、関連する要素は領域の境界にあります。
// (Used during the region and contour build process.)
//（領域および輪郭の構築プロセス中に使用されます。）
// @see rcCompactSpan::reg, #rcContour::verts, #rcContour::rverts
constexpr int RC_AREA_BORDER = 0x20000;

// Contour build flags. // 輪郭構築フラグ。
// @see rcBuildContours
enum rcBuildContoursFlags
{
	// Tessellate solid (impassable) edges during contour simplification.
	// 輪郭の単純化中にソリッド（通過できない）エッジをテッセレーションします。
	RC_CONTOUR_TESS_WALL_EDGES = 0x01,
	// Tessellate edges between areas during contour simplification.
	// 輪郭の単純化中に領域間のエッジをテッセレーションします。
	RC_CONTOUR_TESS_AREA_EDGES = 0x02,
};

// Applied to the region id field of contour vertices in order to extract the region id.
// 領域IDを抽出するために、輪郭頂点の領域IDフィールドに適用されます。
// The region id field of a vertex may have several flags applied to it.
// 頂点の領域IDフィールドには、いくつかのフラグが適用される場合があります。
// So the fields value can't be used directly.
// そのため、フィールドの値を直接使用することはできません。
// @see rcContour::verts, rcContour::rverts
constexpr int RC_CONTOUR_REG_MASK = 0xffff;

// An value which indicates an invalid index within a mesh.
// メッシュ内の無効なインデックスを示す値。
// @note This does not necessarily indicate an error.
// これは必ずしもエラーを示しているわけではありません。
// @see rcPolyMesh::polys
constexpr uint16_t RC_MESH_NULL_IDX = 0xffff;

// Represents the null area. // nullエリアを表します。
// When a data element is given this value it is considered to no longer be
// assigned to a usable area.  (E.g. It is unwalkable.)
//データ要素にこの値が与えられると、使用可能な領域に割り当てられなくなったと見なされます。(例：歩行不能）
constexpr uint8_t RC_NULL_AREA = 0;

// The default area id used to indicate a walkable polygon.
// This is also the maximum allowed area id, and the only non-null area id
// recognized by some steps in the build process.
// デフォルトの領域は、歩行可能なポリゴンを示すために使用されます。
// これは、許可される最大エリアIDであり、ビルドプロセスのいくつかのステップで認識される唯一の非ヌルエリアIDです。
constexpr uint8_t RC_WALKABLE_AREA = 63;

// The value returned by #rcGetCon if the specified direction is not connected
// to another span. (Has no neighbor.)
// 指定された方向が別のスパンに接続されていない場合に#rcGetConによって返される値。(付近に存在しません）
constexpr int RC_NOT_CONNECTED = 0x3f;

// @name General helper functions
// @{
// Used to ignore a function parameter. VS complains about unused parameters and this silences the warning.
// 関数パラメーターを無視するために使用されます。 VSは未使用のパラメーターについて不平を言っており、これは警告を黙らせます。
//  @param [in] _ Unused parameter
template<class T> void rcIgnoreUnused(const T&) { }

// Swaps the values of the two parameters.
// 2つのパラメーターの値を交換します。
//  @param[in,out]	a	Value A
//  @param[in,out]	b	Value B
template<class T> inline void rcSwap(T& a, T& b) { std::swap(a, b); }

// Returns the minimum of two values.
// 最小の2つの値を返します。
//  @param[in]		a	Value A
//  @param[in]		b	Value B
//  @return The minimum of the two values.
template<class T> inline constexpr T rcMin(T a, T b) { return (std::min)(a, b); }

// Returns the maximum of two values.
// 最大2つの値を返します。
//  @param[in]		a	Value A
//  @param[in]		b	Value B
//  @return The maximum of the two values.
template<class T> inline constexpr T rcMax(T a, T b) { return (std::max)(a, b); }

// Returns the absolute value.
// 絶対値を返します。
//  @param[in]		a	The value.
//  @return The absolute value of the specified value.
template<class T> inline constexpr T rcAbs(T a) { return a < 0 ? -a : a; }

// Returns the square of the value.
// 値の二乗を返します。
//  @param[in]		a	The value.
//  @return The square of the value.
template<class T> inline constexpr T rcSqr(T a) { return a * a; }

// Clamps the value to the specified range.
// 指定した範囲に値をクランプします。
//  @param[in]		v	The value to clamp.
//  @param[in]		mn	The minimum permitted return value.
//  @param[in]		mx	The maximum permitted return value.
//  @return The value, clamped to the specified range.
template<class T> inline constexpr T rcClamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

// Returns the square root of the value.
// 値の平方根を返します。
//  @param[in]		x	The value.
//  @return The square root of the vlaue.
inline float rcSqrt(float x)
{
	return sqrtf(x);
}

// @}
// @name Vector helper functions.
// @{

// Derives the cross product of two vectors. (@p v1 x @p v2)
// 2つのベクトルの外積を導出します。
//  @param[out]	dest	The cross product. [(x, y, z)]
//  @param[in]		v1		A Vector [(x, y, z)]
//  @param[in]		v2		A vector [(x, y, z)]
inline constexpr void rcVcross(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
	dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
	dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// Derives the cross product of two vectors. (@p v1 x @p v2)
// 2つのベクトルの外積を導出します。
//  @param[out]	dest	The cross product. [(x, y, z)]
//  @param[in]		v1		A Vector [(x, y, z)]
//  @param[in]		v2		A vector [(x, y, z)]
inline constexpr void rcVcross(
	std::array<float, 3>* dest, const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	dest->at(0) = v1[1] * v2[2] - v1[2] * v2[1];
	dest->at(1) = v1[2] * v2[0] - v1[0] * v2[2];
	dest->at(2) = v1[0] * v2[1] - v1[1] * v2[0];
}

// Derives the dot product of two vectors. (@p v1 . @p v2)
// 2つのベクトルのドット積を導出します。
//  @param[in]		v1	A Vector [(x, y, z)]
//  @param[in]		v2	A vector [(x, y, z)]
// @return The dot product.
inline constexpr float rcVdot(const float* v1, const float* v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// Derives the dot product of two vectors. (@p v1 . @p v2)
// 2つのベクトルのドット積を導出します。
//  @param[in]		v1	A Vector [(x, y, z)]
//  @param[in]		v2	A vector [(x, y, z)]
// @return The dot product.
inline constexpr float rcVdot(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
// スケーリングされたベクトル加算を実行します。
//  @param[out]	dest	The result vector. [(x, y, z)]
//  結果ベクトル。[（x、y、z）]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  ベースベクトル。[（x、y、z）]
//  @param[in]		v2		The vector to scale and add to @p v1. [(x, y, z)]
//  スケーリングして@p v1に追加するベクトル。[（x、y、z）]
//  @param[in]		s		The amount to scale @p v2 by before adding to @p v1.
//  v1に追加する前に@p v2をスケーリングする量。
inline constexpr void rcVmad(float* dest, const float* v1, const float* v2, const float s)
{
	dest[0] = v1[0] + v2[0] * s;
	dest[1] = v1[1] + v2[1] * s;
	dest[2] = v1[2] + v2[2] * s;
}

// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
// スケーリングされたベクトル加算を実行します。
//  @param[out]	dest	The result vector. [(x, y, z)]
//  結果ベクトル。[（x、y、z）]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  ベースベクトル。[（x、y、z）]
//  @param[in]		v2		The vector to scale and add to @p v1. [(x, y, z)]
//  スケーリングして@p v1に追加するベクトル。[（x、y、z）]
//  @param[in]		s		The amount to scale @p v2 by before adding to @p v1.
//  v1に追加する前に@p v2をスケーリングする量。
inline constexpr void rcVmad(
	std::array<float, 3>* dest, const std::array<float, 3>& v1, const std::array<float, 3>& v2, const float s)
{
	dest->at(0) = v1[0] + v2[0] * s;
	dest->at(1) = v1[1] + v2[1] * s;
	dest->at(2) = v1[2] + v2[2] * s;
}

// Performs a vector addition. (@p v1 + @p v2)
// ベクトルの加算を実行します。（@p v1 + @p v2）
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  @param[in]		v2		The vector to add to @p v1. [(x, y, z)]
inline constexpr void rcVadd(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0] + v2[0];
	dest[1] = v1[1] + v2[1];
	dest[2] = v1[2] + v2[2];
}

// Performs a vector subtraction. (@p v1 - @p v2)
// ベクトル減算を実行します。（@p v1-@p v2）
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  @param[in]		v2		The vector to subtract from @p v1. [(x, y, z)]
inline constexpr void rcVsub(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0] - v2[0];
	dest[1] = v1[1] - v2[1];
	dest[2] = v1[2] - v2[2];
}

namespace RcOperator
{
	using ArrayF = std::array<float, 3>;

	// Performs a vector addition. (@p v1 + @p v2)
	// ベクトルの加算を実行します。 （v1 + v2）
	//  @param[out]	dest	The result vector. [(x, y, z)]
	//  @param[in]		v1		The base vector. [(x, y, z)]
	//  @param[in]		v2		The vector to add to @p v1. [(x, y, z)]
	inline auto operator+(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		std::array<float, 3> dest;

		dest[0] = v1[0] + v2[0];
		dest[1] = v1[1] + v2[1];
		dest[2] = v1[2] + v2[2];

		return dest;
	}

	inline void operator+=(std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		v1[0] += v2[0];
		v1[1] += v2[1];
		v1[2] += v2[2];
	}

	// Performs a vector subtraction. (@p v1 - @p v2)
	// ベクトル減算を実行します。（v1 - v2）
	//  @param[out]	dest	The result vector. [(x, y, z)]
	//  @param[in]		v1		The base vector. [(x, y, z)]
	//  @param[in]		v2		The vector to subtract from @p v1. [(x, y, z)]
	inline auto operator-(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		std::array<float, 3> dest;

		dest[0] = v1[0] - v2[0];
		dest[1] = v1[1] - v2[1];
		dest[2] = v1[2] - v2[2];

		return dest;
	}

	inline void operator-=(std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		v1[0] -= v2[0];
		v1[1] -= v2[1];
		v1[2] -= v2[2];
	}

	inline auto operator*(const std::array<float, 3>& v1, const float num)
	{
		std::array<float, 3> dest;

		dest[0] = v1[0] * num;
		dest[1] = v1[1] * num;
		dest[2] = v1[2] * num;

		return dest;
	}

	inline void operator*=(std::array<float, 3>& v1, const float num)
	{
		v1[0] *= num;
		v1[1] *= num;
		v1[2] *= num;
	}
}

// Selects the minimum value of each element from the specified vectors.
// 指定されたベクトルから各要素の最小値を選択します。
//  @param[in,out]	mn	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]		v	A vector. [(x, y, z)]
inline constexpr void rcVmin(float* mn, const float* v)
{
	mn[0] = rcMin(mn[0], v[0]);
	mn[1] = rcMin(mn[1], v[1]);
	mn[2] = rcMin(mn[2], v[2]);
}

// Selects the minimum value of each element from the specified vectors.
// 指定されたベクトルから各要素の最小値を選択します。
//  @param[in,out]	mn	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]		v	A vector. [(x, y, z)]
inline constexpr void rcVmin(std::array<float, 3>* mn, const std::array<float, 3>& v)
{
	mn->at(0) = rcMin(mn->at(0), v[0]);
	mn->at(1) = rcMin(mn->at(1), v[1]);
	mn->at(2) = rcMin(mn->at(2), v[2]);
}

// Selects the maximum value of each element from the specified vectors.
// 指定されたベクトルから各要素の最大値を選択します。
//  @param[in,out]	mx	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]		v	A vector. [(x, y, z)]
inline constexpr void rcVmax(float* mx, const float* v)
{
	mx[0] = rcMax(mx[0], v[0]);
	mx[1] = rcMax(mx[1], v[1]);
	mx[2] = rcMax(mx[2], v[2]);
}

// Selects the maximum value of each element from the specified vectors.
// 指定されたベクトルから各要素の最大値を選択します。
//  @param[in,out]	mx	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]		v	A vector. [(x, y, z)]
inline constexpr void rcVmax(std::array<float, 3>* mn, const std::array<float, 3>& v)
{
	mn->at(0) = rcMax(mn->at(0), v[0]);
	mn->at(1) = rcMax(mn->at(1), v[1]);
	mn->at(2) = rcMax(mn->at(2), v[2]);
}

// Performs a vector copy.
// ベクターコピーを実行します。
//  @param[out]	dest	The result. [(x, y, z)]
//  @param[in]		v		The vector to copy. [(x, y, z)]
inline constexpr void rcVcopy(float* dest, const float* v)
{
	dest[0] = v[0];
	dest[1] = v[1];
	dest[2] = v[2];
}

// Returns the distance between two points.
// 2点間の距離を返します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The distance between the two points.
inline float rcVdist(const float* v1, const float* v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return rcSqrt(dx * dx + dy * dy + dz * dz);
}

// Returns the distance between two points.
// 2点間の距離を返します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The distance between the two points.
inline float rcVdist(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return rcSqrt(dx * dx + dy * dy + dz * dz);
}

// Returns the square of the distance between two points.
// 2点間の距離の2乗を返します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The square of the distance between the two points.
inline constexpr float rcVdistSqr(const float* v1, const float* v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return dx * dx + dy * dy + dz * dz;
}

// Returns the square of the distance between two points.
// 2点間の距離の2乗を返します。
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The square of the distance between the two points.
inline constexpr float rcVdistSqr(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return dx * dx + dy * dy + dz * dz;
}

// Normalizes the vector.
// ベクトルを正規化します。
//  @param[in,out]	v	The vector to normalize. [(x, y, z)]
inline void rcVnormalize(float* v)
{
	float d = 1.f / rcSqrt(rcSqr(v[0]) + rcSqr(v[1]) + rcSqr(v[2]));
	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

// Normalizes the vector.
// ベクトルを正規化します。
//  @param[in,out]	v	The vector to normalize. [(x, y, z)]
inline void rcVnormalize(std::array<float, 3>* v)
{
	float d = 1.f / rcSqrt(rcSqr(v->at(0)) + rcSqr(v->at(1)) + rcSqr(v->at(2)));
	v->at(0) *= d;
	v->at(1) *= d;
	v->at(2) *= d;
}

// @}
// @name Heightfield Functions
// @see rcHeightfield
// @{
// Calculates the bounding box of an array of vertices.
// 頂点の配列の境界ボックスを計算します。
//  @ingroup recast
//  @param[in]		verts	An array of vertices. [(x, y, z) * @p nv]
//  verts頂点の配列。[（x、y、z）* @p nv]
//  @param[in]		nv		The number of vertices in the @p verts array.
//  verts配列内の頂点の数。
//  @param[out]	bmin	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABBの最小境界。[（x、y、z）] [単位：wu]
//  @param[out]	bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABBの最大境界。[（x、y、z）] [単位：wu]
void rcCalcBounds(const float* verts, int nv, float* bmin, float* bmax);

// Calculates the grid size based on the bounding box and grid cell size.
// 境界ボックスとグリッドセルサイズに基づいてグリッドサイズを計算します。
//  @ingroup recast
//  @param[in] bmin : The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABBの最小境界。 [（x、y、z）] [単位：wu]
//  @param[in] bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABBの最大境界。 [（x、y、z）] [単位：wu]
//  @param[in] cs : The xz-plane cell size. [Limit: > 0] [Units: wu]
//  xz平面のセルサイズ。 [制限：> 0] [単位：wu]
//  @param[out] w : The width along the x-axis. [Limit: >= 0] [Units: vx]
//  x軸に沿った幅。 [制限：> = 0] [単位：vx]
//  @param[out] h : The height along the z-axis. [Limit: >= 0] [Units: vx]
//  z軸に沿った高さ。 [制限：> = 0] [単位：vx]
void rcCalcGridSize(const std::array<float, 3>& bmin, const std::array<float, 3>& bmax, float cs, int* w, int* h);

// Calculates the grid size based on the bounding box and grid cell size.
// 境界ボックスとグリッドセルサイズに基づいてグリッドサイズを計算します。
//  @ingroup recast
//  @param[in] bmin : The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABBの最小境界。 [（x、y、z）] [単位：wu]
//  @param[in] bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABBの最大境界。 [（x、y、z）] [単位：wu]
//  @param[in] cs : The xz-plane cell size. [Limit: > 0] [Units: wu]
//  xz平面のセルサイズ。 [制限：> 0] [単位：wu]
//  @param[out] w : The width along the x-axis. [Limit: >= 0] [Units: vx]
//  x軸に沿った幅。 [制限：> = 0] [単位：vx]
//  @param[out] h : The height along the z-axis. [Limit: >= 0] [Units: vx]
//  z軸に沿った高さ。 [制限：> = 0] [単位：vx]
void rcCalcGridSize(const float* bmin, const float* bmax, float cs, int* w, int* h);

// Initializes a new heightfield. // 新しい地形を初期化します。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in,out] hf : The allocated heightfield to initialize.
//	初期化するために割り当てられた高さフィールド。
//  @param[in] width : The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
//	x軸に沿ったフィールドの幅。 [制限：> = 0] [単位：vx]
//  @param[in] height : The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
//	z軸に沿ったフィールドの高さ。 [制限：> = 0] [単位：vx]
//  @param[in] bmin : The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
//	フィールドのAABBの最小境界。 [（x、y、z）] [単位：wu]
//  @param[in] bmax : The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
//	フィールドのAABBの最大境界。 [（x、y、z）] [単位：wu]
//  @param[in] cs : The xz-plane cell size to use for the field. [Limit: > 0] [Units: wu]
//	フィールドに使用するxz平面のセルサイズ。 [制限：> 0] [単位：wu]
//  @param[in] ch : The y-axis cell size to use for field. [Limit: > 0] [Units: wu]
//	フィールドに使用するy軸のセルサイズ。 [制限：> 0] [単位：wu]
//  @returns True if the operation completed successfully.
//	操作が正常に完了した場合はtrue。
bool rcCreateHeightfield(rcContext* ctx, rcHeightfield& hf, int width, int height,
	const float* bmin, const float* bmax,
	float cs, float ch);

// Modifies the area id of all triangles with a slope below the specified value.
//	指定された値より低い勾配ですべての三角形のエリアIDを変更します。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in walkableSlopeAngle : The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees]
//	歩行可能と見なされる最大勾配。 [制限：0 <=値<90] [単位：度]
//  @param[in] verts	 : The vertices. [(x, y, z) * @p nv]
//	頂点。 [（x、y、z）* @p nv]
//  @param[in] nv : The number of vertices.
//	頂点の数。
//  @param[in] tris : The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
//	三角形の頂点のインデックス。 [（vertA、vertB、vertC）* @p nt]
//  @param[in] nt : The number of triangles.
//	三角形の数。
//  @param[out] areas : The triangle area ids. [Length: >= @p nt]
//	三角形のエリアID。 [長さ：> = @p nt]
//  @param[in] areaMod : The area modification to apply.
//	適用するエリアの変更。
// 歩行可能な三角形をマーク
void rcMarkWalkableTriangles(rcContext* ctx, const float walkableSlopeAngle, const float* verts, int nv,
	const int* tris, int nt, uint8_t* areas, rcAreaModification areaMod);

// Modifies the area id of all triangles with a slope greater than or equal to the specified value.
//  @ingroup recast
//  @param[in,out]	ctx					The build context to use during the operation.
//  @param[in]		walkableSlopeAngle	The maximum slope that is considered walkable.
//  									[Limits: 0 <= value < 90] [Units: Degrees]
//  @param[in]		verts				The vertices. [(x, y, z) * @p nv]
//  @param[in]		nv					The number of vertices.
//  @param[in]		tris				The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
//  @param[in]		nt					The number of triangles.
//  @param[out]	areas				The triangle area ids. [Length: >= @p nt]
void rcClearUnwalkableTriangles(rcContext* ctx, const float walkableSlopeAngle, const float* verts, int nv,
	const int* tris, int nt, uint8_t* areas);

// Adds a span to the specified heightfield.
//	指定された高さフィールドにスパンを追加します。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in,out] hf : An initialized heightfield.
//	初期化された高さフィールド。
//  @param[in] x	 : The width index where the span is to be added. [Limits: 0 <= value < rcHeightfield::width]
//	スパンが追加される幅インデックス。 [制限：0 <=値<rcHeightfield :: width]
//  @param[in] y	 : The height index where the span is to be added. [Limits: 0 <= value < rcHeightfield::height]
//	スパンが追加される高さインデックス。 [制限：0 <=値<rcHeightfield :: height]
//  @param[in] smin : The minimum height of the span. [Limit: < @p smax] [Units: vx]
//	スパンの最小の高さ。 [制限：<@p smax] [単位：vx]
//  @param[in] smax : The maximum height of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT] [Units: vx]
//	スパンの最大の高さ。 [制限：<= #RC_SPAN_MAX_HEIGHT] [単位：vx]
//  @param[in] area : The area id of the span. [Limit: <= #RC_WALKABLE_AREA)
//	スパンのエリアID。 [制限：<= #RC_WALKABLE_AREA）
//  @param[in] flagMergeThr : The merge theshold. [Limit: >= 0] [Units: vx]
//	マージのしきい値。 [制限：> = 0] [単位：vx]
//  @returns True if the operation completed successfully.
//	操作が正常に完了した場合はtrue。
bool rcAddSpan(rcContext* ctx, rcHeightfield& hf, const int x, const int y,
	const uint16_t smin, const uint16_t smax,
	const uint8_t area, const int flagMergeThr);

// Rasterizes a triangle into the specified heightfield.
//  @ingroup recast
//  @param[in,out]	ctx				The build context to use during the operation.
//  @param[in]		v0				Triangle vertex 0 [(x, y, z)]
//  @param[in]		v1				Triangle vertex 1 [(x, y, z)]
//  @param[in]		v2				Triangle vertex 2 [(x, y, z)]
//  @param[in]		area			The area id of the triangle. [Limit: <= #RC_WALKABLE_AREA]
//  @param[in,out]	solid			An initialized heightfield.
//  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag.
//  								[Limit: >= 0] [Units: vx]
//  @returns True if the operation completed successfully.
bool rcRasterizeTriangle(rcContext* ctx, const float* v0, const float* v1, const float* v2,
	const uint8_t area, rcHeightfield& solid,
	const int flagMergeThr = 1);

// Rasterizes an indexed triangle mesh into the specified heightfield.
//	インデックス付き三角形メッシュを指定された地形にラスタライズします。
//  @ingroup recast
//  @param[in,out] ctx	 : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in] verts	 : The vertices. [(x, y, z) * @p nv]
//	頂点。 [（x、y、z）* @p nv]
//  @param[in] nv	 : The number of vertices.
//	頂点の数。
//  @param[in] tris	 : The triangle indices. [(vertA, vertB, vertC) * @p nt]
//	三角形のインデックス。 [（vertA、vertB、vertC）* @p nt]
//  @param[in] areas	 : The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
//	三角形のエリアID。 [制限：<= #RC_WALKABLE_AREA] [サイズ：@p nt]
//  @param[in] nt	 : The number of triangles.
//	三角形の数。
//  @param[in,out] solid	 : An initialized heightfield.
//	初期化された地形。
//  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag. [Limit: >= 0] [Units: vx]
//	歩行不可能フラグよりも歩行可能フラグが優先される距離。 [制限：> = 0] [単位：vx]
//  @returns True if the operation completed successfully.
//	操作が正常に完了した場合はtrue。
// 三角形のラスタライズ
bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const int nv,
	const int* tris, const uint8_t* areas, const int nt,
	rcHeightfield& solid, const int flagMergeThr = 1);

// Rasterizes an indexed triangle mesh into the specified heightfield.
//  @ingroup recast
//  @param[in,out]	ctx			The build context to use during the operation.
//  @param[in]		verts		The vertices. [(x, y, z) * @p nv]
//  @param[in]		nv			The number of vertices.
//  @param[in]		tris		The triangle indices. [(vertA, vertB, vertC) * @p nt]
//  @param[in]		areas		The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
//  @param[in]		nt			The number of triangles.
//  @param[in,out]	solid		An initialized heightfield.
//  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag.
//  							[Limit: >= 0] [Units: vx]
//  @returns True if the operation completed successfully.
bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const int nv,
	const uint16_t* tris, const uint8_t* areas, const int nt,
	rcHeightfield& solid, const int flagMergeThr = 1);

// Rasterizes triangles into the specified heightfield.
//  @ingroup recast
//  @param[in,out]	ctx				The build context to use during the operation.
//  @param[in]		verts			The triangle vertices. [(ax, ay, az, bx, by, bz, cx, by, cx) * @p nt]
//  @param[in]		areas			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
//  @param[in]		nt				The number of triangles.
//  @param[in,out]	solid			An initialized heightfield.
//  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag.
//  								[Limit: >= 0] [Units: vx]
//  @returns True if the operation completed successfully.
bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const uint8_t* areas, const int nt,
	rcHeightfield& solid, const int flagMergeThr = 1);

// Marks non-walkable spans as walkable if their maximum is within @p walkableClimp of a walkable neighbor.
// 最大値が歩行可能な隣接の歩行可能なClimp内にある場合、歩行不能としてスパンをマークします。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in] walkableClimb : Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx]
//	まだ通過可能であると見なされる最大の出張りの高さ。 [制限：> = 0] [単位：vx]
//  @param[in,out] solid : A fully built heightfield.  (All spans have been added.)
//	完全に構築された地形。（すべてのスパンが追加された）
void rcFilterLowHangingWalkableObstacles(rcContext* ctx, const int walkableClimb, rcHeightfield& solid);

// Marks spans that are ledges as not-walkable.
// 出張りであるスパンを歩行不能としてマークします。
//
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in] walkableHeight : Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3] [Units: vx]
//	床面積が歩行可能と見なされるようにする最小床から「天井」までの高さ。[制限：> = 3] [単位：vx]
//  @param[in] walkableClimb : Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx]
//	まだ通過可能であると見なされる最大の出張りの高さ。[制限：> = 0] [単位：vx]
//  @param[in,out] solid : A fully built heightfield. (All spans have been added.)
//	完全に構築された地形。（すべてのスパンが追加されました。）
void rcFilterLedgeSpans(rcContext* ctx, const int walkableHeight,
	const int walkableClimb, rcHeightfield& solid);

// Marks walkable spans as not walkable if the clearence above the span is less than the specified height.
// スパンの上のクリアランスが指定された高さよりも小さい場合、歩行可能スパンを歩行不可としてマークします。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation. // 操作中に使用するビルドコンテキスト。
//  @param[in] walkableHeight : Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3] [Units: vx]
// 床面積が歩行可能と見なされるようにする最小床から「天井」までの高さ。 [制限：> = 3] [単位：vx]
//  @param[in,out] solid : A fully built heightfield.(All spans have been added.) // 完全に構築された地形。(すべてのスパンが追加されました。）
void rcFilterWalkableLowHeightSpans(rcContext* ctx, int walkableHeight, rcHeightfield& solid);

// Returns the number of spans contained in the specified heightfield.
//　指定された高さフィールドに含まれるスパンの数を返します。
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//　操作中に使用するビルドコンテキスト。
//  @param[in]		hf		An initialized heightfield.
//　初期化された高さフィールド。
//  @returns The number of spans in the heightfield.
//　高さフィールドのスパンの数。
int rcGetHeightFieldSpanCount(rcContext* ctx, rcHeightfield& hf);

// @}
// @name Compact Heightfield Functions
// @see rcCompactHeightfield
// @{
// Builds a compact heightfield representing open space, from a heightfield representing solid space.
// ソリッドスペースを表す地形から、オープンスペースを表すコンパクトな地形を構築します。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in] walkableHeight : Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3] [Units: vx]
//	床面積が歩行可能と見なされるようにする最小床から「天井」までの高さ。 [制限：> = 3] [単位：vx]
//  @param[in] walkableClimb : Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx]
//	まだ通過可能であると見なされる最大の出張りの高さ。 制限：> = 0] [単位：vx]
//  @param[in] hf : The heightfield to be compacted. // 圧縮される高さフィールド。
//  @param[out] chf : The resulting compact heightfield. (Must be pre-allocated.)
//	結果のコンパクトな高さフィールド。(事前に割り当てる必要があります。）
//  @returns True if the operation completed successfully. // 操作が正常に完了した場合はtrue。
//  コンパクトな地形の構築
bool rcBuildCompactHeightfield(rcContext* ctx, const int walkableHeight, const int walkableClimb,
	rcHeightfield& hf, rcCompactHeightfield& chf);

// Erodes the walkable area within the heightfield by the specified radius.
// 指定された半径だけ地形内の歩行可能領域を侵食します。
// @ingroup recast
// @param[in,out] ctx : The build context to use during the operation.
// 操作中に使用するビルドコンテキスト。
// @param[in] radius : The radius of erosion. [Limits: 0 < value < 255] [Units: vx]
// 侵食の半径。 [制限：0 <値<255] [単位：vx]
// @param[in,out] chf : The populated compact heightfield to erode.
// 侵食するために読み込まれたコンパクトな高さフィールド。
// @returns True if the operation completed successfully. // 操作が正常に完了した場合はtrue。
bool rcErodeWalkableArea(rcContext* ctx, int radius, rcCompactHeightfield& chf);

// Applies a median filter to walkable area types (based on area id), removing noise.
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//  @param[in,out]	chf		A populated compact heightfield.
//  @returns True if the operation completed successfully.
bool rcMedianFilterWalkableArea(rcContext* ctx, rcCompactHeightfield& chf);

// Applies an area id to all spans within the specified bounding box. (AABB)
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//  @param[in]		bmin	The minimum of the bounding box. [(x, y, z)]
//  @param[in]		bmax	The maximum of the bounding box. [(x, y, z)]
//  @param[in]		areaMod	The area modification to apply.
//  @param[in,out]	chf		A populated compact heightfield.
void rcMarkBoxArea(rcContext* ctx, const float* bmin, const float* bmax, rcAreaModification areaMod,
	rcCompactHeightfield& chf);

// Applies the area id to the all spans within the specified convex polygon.
// 指定された凸多角形内のすべてのスパンにエリアIDを適用します。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in] verts : The vertices of the polygon [Fomr: (x, y, z) * @p nverts]
//	ポリゴンの頂点[Fomr：（x、y、z）* @p nverts]
//  @param[in] nverts : The number of vertices in the polygon.
//	ポリゴン内の頂点の数。
//  @param[in] hmin : The height of the base of the polygon.
//	ポリゴンのベースの高さ。
//  @param[in] hmax : The height of the top of the polygon.
//	多角形の上部の高さ。
//  @param[in] areaMod : The area modification to apply.
//	適用するエリアの変更。
//  @param[in,out] chf : A populated compact heightfield.
//	読み込まれたコンパクトな地形。
void rcMarkConvexPolyArea(rcContext* ctx, const float* verts, const int nverts,
	const float hmin, const float hmax, rcAreaModification areaMod,
	rcCompactHeightfield& chf);

// Helper function to offset voncex polygons for rcMarkConvexPolyArea.
//  @ingroup recast
//  @param[in]		verts		The vertices of the polygon [Form: (x, y, z) * @p nverts]
//  @param[in]		nverts		The number of vertices in the polygon.
//  @param[out]	outVerts	The offset vertices (should hold up to 2 * @p nverts) [Form: (x, y, z) * return value]
//  @param[in]		maxOutVerts	The max number of vertices that can be stored to @p outVerts.
//  @returns Number of vertices in the offset polygon or 0 if too few vertices in @p outVerts.
int rcOffsetPoly(const float* verts, const int nverts, const float offset,
	float* outVerts, const int maxOutVerts);

// Applies the area id to all spans within the specified cylinder.
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//  @param[in]		pos		The center of the base of the cylinder. [Form: (x, y, z)]
//  @param[in]		r		The radius of the cylinder.
//  @param[in]		h		The height of the cylinder.
//  @param[in]		areaMod	The area modification to apply.
//  @param[in,out]	chf	A populated compact heightfield.
void rcMarkCylinderArea(rcContext* ctx, const float* pos,
	const float r, const float h, rcAreaModification areaMod,
	rcCompactHeightfield& chf);

// Builds the distance field for the specified compact heightfield.
// 指定したコンパクトな地形の距離フィールドを構築します。
// @ingroup recast
// @param[in,out] ctx : The build context to use during the operation. 操作中に使用するビルドコンテキスト。
// @param[in,out] chf : A populated compact heightfield. 読み込まれたコンパクトな地形。
// @returns True if the operation completed successfully. 操作が正常に完了した場合はTrue。
bool rcBuildDistanceField(rcContext* ctx, rcCompactHeightfield& chf);

// 流域分割を使用して、地形の領域データを構築します。
//  @ingroup recast
//  @param[in,out]	ctx				操作中に使用するビルドコンテキスト。
//  @param[in,out]	chf				読み込まれたコンパクトな地形。
//  @param[in]		borderSize		地形の周りのナビゲーション不可の境界線のサイズ。[Limit: >=0] [Units: vx]
//  @param[in]		minRegionArea	孤立した島の領域を形成できるセルの最小数。[Limit: >=0] [Units: vx].
//  @param[in]		mergeRegionArea		スパンカウントがこの値よりも小さい領域は、可能であればより大きな領域とマージされます。 [Limit: >=0] [Units: vx]
//  @returns 操作が正常に完了した場合はTrue。
// Builds region data for the heightfield using watershed partitioning.
//  @ingroup recast
//  @param[in,out]	ctx				The build context to use during the operation.
//  @param[in,out]	chf				A populated compact heightfield.
//  @param[in]		borderSize		The size of the non-navigable border around the heightfield.
//  								[Limit: >=0] [Units: vx]
//  @param[in]		minRegionArea	The minimum number of cells allowed to form isolated island areas.
//  								[Limit: >=0] [Units: vx].
//  @param[in]		mergeRegionArea		Any regions with a span count smaller than this value will, if possible,
//  								be merged with larger regions. [Limit: >=0] [Units: vx]
//  @returns True if the operation completed successfully.
bool rcBuildRegions(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea, const int mergeRegionArea);

// Builds region data for the heightfield by partitioning the heightfield in non-overlapping layers.
// 重なり合わないレイヤーで地形を分割することにより、地形の領域データを構築します。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in,out] chf : A populated compact heightfield.
//	読み込まれたコンパクトな地形。
//  @param[in] borderSize : The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
//	地形の周りのナビゲーション不可能な境界線のサイズ。 [制限：> = 0] [単位：vx]
//  @param[in] minRegionArea : The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx].
//	孤立した島の領域を形成できるセルの最小数。 [制限：> = 0] [単位：vx]。
//  @returns True if the operation completed successfully.
//	操作が正常に完了した場合はtrue。
bool rcBuildLayerRegions(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea);

// Builds region data for the heightfield using simple monotone partitioning.
// 単純なモノトーン分割を使用して、地形の領域データを構築します。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in,out] chf : A populated compact heightfield.
//	読み込まれたコンパクトな高さフィールド。
//  @param[in] borderSize : The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
//	heightfieldの周りのナビゲーション不可能な境界線のサイズ。 [制限：> = 0] [単位：vx]
//  @param[in] minRegionArea : The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx].
//	孤立した島の領域を形成できるセルの最小数。 [制限：> = 0] [単位：vx]。
//  @param[in] mergeRegionArea : Any regions with a span count smaller than this value will, if possible, be merged with larger regions. [Limit: >=0] [Units: vx]
//	可能な場合、スパンカウントがこの値よりも小さい領域は、より大きな領域とマージされます。 [制限：> = 0] [単位：vx]
//  @returns True if the operation completed successfully.
//	操作が正常に完了した場合はtrue。
bool rcBuildRegionsMonotone(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea, const int mergeRegionArea);

// Sets the neighbor connection data for the specified direction.
//  @param[in]		s		The span to update.
//  @param[in]		dir		The direction to set. [Limits: 0 <= value < 4]
//  @param[in]		i		The index of the neighbor span.
inline void rcSetCon(rcCompactSpan& s, int dir, int i)
{
	const uint32_t shift = (uint32_t)dir * 6;
	uint32_t con = s.con;
	s.con = (con & ~(0x3f << shift)) | (((uint32_t)i & 0x3f) << shift);
}

// Gets neighbor connection data for the specified direction.
// 指定された方向の近隣接続データを取得します。
//  @param[in]		s		The span to check. チェックするスパン
//  @param[in]		dir		The direction to check. チェックする方向 [Limits: 0 <= value < 4]
//  @return The neighbor connection data for the specified direction,
//  	or #RC_NOT_CONNECTED if there is no connection.
// 指定された方向の隣接接続データ、接続がない場合は#RC_NOT_CONNECTED。
inline int rcGetCon(const rcCompactSpan& s, int dir)
{
	const uint32_t shift = (uint32_t)dir * 6;
	return (s.con >> shift) & 0x3f;
}

// Gets the standard width (x-axis) offset for the specified direction.
// 指定された方向の標準幅（x軸）オフセットを取得します。
//  @param[in]		dir		The direction. [Limits: 0 <= value < 4]
//  @return The width offset to apply to the current cell position to move
//  	in the direction.
//方向に移動するために現在のセル位置に適用する幅オフセット。
inline int rcGetDirOffsetX(int dir)
{
	constexpr int offset[4] = { -1, 0, 1, 0, };
	return offset[dir & 0x03];
}

// Gets the standard height (z-axis) offset for the specified direction.
// 指定された方向の標準の高さ（z軸）オフセットを取得します。
//  @param[in]		dir		The direction. [Limits: 0 <= value < 4]
//  @return The height offset to apply to the current cell position to move
//  	in the direction.
// 方向に移動するために現在のセル位置に適用する高さオフセット。
inline int rcGetDirOffsetY(int dir)
{
	constexpr int offset[4] = { 0, 1, 0, -1 };
	return offset[dir & 0x03];
}

// Gets the direction for the specified offset. One of x and y should be 0.
//  @param[in]		x		The x offset. [Limits: -1 <= value <= 1]
//  @param[in]		y		The y offset. [Limits: -1 <= value <= 1]
//  @return The direction that represents the offset.
inline int rcGetDirForOffset(int x, int y)
{
	constexpr int dirs[5] = { 3, 0, -1, 2, 1 };
	return dirs[((y + 1) << 1) + x];
}

// @}
// @name Layer, Contour, Polymesh, and Detail Mesh Functions
// @see rcHeightfieldLayer, rcContourSet, rcPolyMesh, rcPolyMeshDetail
// @{
// Builds a layer set from the specified compact heightfield.
// 指定されたコンパクトな地形からレイヤーセットを構築します。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in] chf : A fully built compact heightfield.
//	完全に構築されたコンパクトな地形。
//  @param[in] borderSize : The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
//  heightfieldの周りのナビゲーション不可能な境界線のサイズ。 [制限：> = 0] [単位：vx]
//  @param[in] walkableHeight : Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3] [Units: vx]
// 床面積が歩行可能と見なされるようにする最小床から「天井」までの高さ。 [制限：> = 3] [単位：vx]
//  @param[out] lset : The resulting layer set. (Must be pre-allocated.)
// 結果のレイヤーセット。 （事前に割り当てる必要があります。）
//  @returns True if the operation completed successfully.
// 操作が正常に完了した場合はtrue。
bool rcBuildHeightfieldLayers(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int walkableHeight,
	rcHeightfieldLayerSet& lset);

// Builds a contour set from the region outlines in the provided compact heightfield.
//	指定されたコンパクトな高さフィールドの領域アウトラインから輪郭セットを構築します。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in] chf : A fully built compact heightfield.
//	完全に構築されたコンパクトなハイトフィールド。
//  @param[in] maxError : The maximum distance a simplfied contour's border edges should deviate the original raw contour. [Limit: >=0] [Units: wu]
//	単純化された輪郭の境界エッジが元の生の輪郭から逸脱する最大距離。 [制限：> = 0] [単位：wu]
//  @param[in] maxEdgeLen : The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx]
//	メッシュの境界に沿った輪郭エッジの最大許容長。 [制限：> = 0] [単位：vx]
//  @param[out] cset : The resulting contour set. (Must be pre-allocated.)
//	結果の輪郭セット。 （事前に割り当てる必要があります。）
//  @param[in] buildFlags : The build flags. (See: #rcBuildContoursFlags)
//	ビルドフラグ。 （#rcBuildContoursFlagsを参照）
//  @returns True if the operation completed successfully.
//	操作が正常に完了した場合はtrue。
bool rcBuildContours(rcContext* ctx, rcCompactHeightfield& chf,
	const float maxError, const int maxEdgeLen,
	rcContourSet& cset, const int buildFlags = RC_CONTOUR_TESS_WALL_EDGES);

// Builds a polygon mesh from the provided contours.
//	指定された輪郭からポリゴンメッシュを構築します。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in] cset : A fully built contour set.
//	完全に構築された輪郭セット。
//  @param[in] nvp : The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process. [Limit: >= 3]
//	輪郭からポリゴンへの変換プロセス中に生成されるポリゴンに許可される頂点の最大数。 [制限：> = 3]
//  @param[out] mesh : The resulting polygon mesh. (Must be re-allocated.)
//	結果のポリゴンメッシュ。 （再割り当てする必要があります。）
//  @returns True if the operation completed successfully.
//	操作が正常に完了した場合はtrue。
bool rcBuildPolyMesh(rcContext* ctx, rcContourSet& cset, const int nvp, rcPolyMesh& mesh);

// Merges multiple polygon meshes into a single mesh.
//	複数のポリゴンメッシュを単一のメッシュにマージします。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in] meshes : An array of polygon meshes to merge. [Size: @p nmeshes]
//	マージするポリゴンメッシュの配列。 [サイズ：@p nmeshes]
//  @param[in] nmeshes : The number of polygon meshes in the meshes array.
//	メッシュ配列内のポリゴンメッシュの数。
//  @param[in] mesh : The resulting polygon mesh. (Must be pre-allocated.)
//	結果のポリゴンメッシュ。 （事前に割り当てる必要があります。）
//  @returns True if the operation completed successfully.
//	操作が正常に完了した場合はtrue。
bool rcMergePolyMeshes(rcContext* ctx, rcPolyMesh** meshes, const int nmeshes, rcPolyMesh& mesh);

// Builds a detail mesh from the provided polygon mesh.
//	指定されたポリゴンメッシュから詳細メッシュを構築します。
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in] mesh : A fully built polygon mesh.
//	完全に構築されたポリゴンメッシュ。
//  @param[in] chf : The compact heightfield used to build the polygon mesh.
//	ポリゴンメッシュの構築に使用されるコンパクトな高さフィールド。
//  @param[in] sampleDist : Sets the distance to use when samping the heightfield. [Limit: >=0] [Units: wu]
//	地形をサンプリングするときに使用する距離を設定します。 [制限：> = 0] [単位：wu]
//  @param[in] sampleMaxError : The maximum distance the detail mesh surface should deviate from heightfield data. [Limit: >=0] [Units: wu]
//	詳細メッシュ表面が地形データから逸脱する最大距離。 [制限：> = 0] [単位：wu]
//  @param[out]	dmesh : The resulting detail mesh.  (Must be pre-allocated.)
//	結果の詳細メッシュ。 （事前に割り当てる必要があります。）
//  @returns True if the operation completed successfully.
//	操作が正常に完了した場合はtrue。
bool rcBuildPolyMeshDetail(rcContext* ctx, const rcPolyMesh& mesh, const rcCompactHeightfield& chf,
	const float sampleDist, const float sampleMaxError,
	rcPolyMeshDetail& dmesh);

// Copies the poly mesh data from src to dst.
//	ポリメッシュデータをsrcからdstにコピーします。
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in]		src		The source mesh to copy from.
//	コピー元のソースメッシュ。
//  @param[out]	dst		The resulting detail mesh. (Must be pre-allocated, must be empty mesh.)
//	結果の詳細メッシュ。 （事前に割り当てられている必要があり、空のメッシュである必要があります。）
//  @returns True if the operation completed successfully.
//	操作が正常に完了した場合はtrue。
bool rcCopyPolyMesh(rcContext* ctx, const rcPolyMesh& src, rcPolyMesh& dst);

// Merges multiple detail meshes into a single detail mesh.
//	複数の詳細メッシュを単一の詳細メッシュにマージします。
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//	操作中に使用するビルドコンテキスト。
//  @param[in]		meshes	An array of detail meshes to merge. [Size: @p nmeshes]
//	マージする詳細メッシュの配列。 [サイズ：@p nmeshes]
//  @param[in]		nmeshes	The number of detail meshes in the meshes array.
//	メッシュ配列内の詳細メッシュの数。
//  @param[out]	mesh	The resulting detail mesh. (Must be pre-allocated.)
//	結果の詳細メッシュ。 （事前に割り当てる必要があります。）
//  @returns True if the operation completed successfully.
//	操作が正常に完了した場合はtrue。
bool rcMergePolyMeshDetails(rcContext* ctx, rcPolyMeshDetail** meshes, const int nmeshes, rcPolyMeshDetail& mesh);

// @}

#endif // RECAST_H

//////////////////////////////////////////////////

// Due to the large amount of detail documentation for this file,
// the content normally located at the end of the header file has been separated
// out to a file in /Docs/Extern.
//このファイルの詳細なドキュメントが大量にあるため、通常ヘッダーファイルの最後にあるコンテンツは/ Docs / Externのファイルに分離されています。
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

#include <float.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

// Must be 255 or smaller (not 256) because layer IDs are stored as
// a byte where 255 is a special value.
static const int RC_MAX_LAYERS = 63;
static const int RC_MAX_NEIS = 16;

struct rcLayerRegion
{
	unsigned char layers[RC_MAX_LAYERS];
	unsigned char neis[RC_MAX_NEIS];
	unsigned short ymin, ymax;
	unsigned char layerId;		// Layer ID
	unsigned char nlayers;		// Layer count
	unsigned char nneis;		// Neighbour count
	unsigned char base;		// Flag indicating if the region is the base of merged regions.
};

static bool contains(const unsigned char* a, const unsigned char an, const unsigned char v)
{
	const int n = (int)an;
	for (int i = 0; i < n; ++i)
	{
		if (a[i] == v)
			return true;
	}
	return false;
}

static bool addUnique(unsigned char* a, unsigned char& an, int anMax, unsigned char v)
{
	if (contains(a, an, v))
		return true;

	if ((int)an >= anMax)
		return false;

	a[an] = v;
	an++;
	return true;
}

inline bool overlapRange(const unsigned short amin, const unsigned short amax,
	const unsigned short bmin, const unsigned short bmax)
{
	return (amin > bmax || amax < bmin) ? false : true;
}

struct rcLayerSweepSpan
{
	unsigned short ns;	// number samples
	unsigned char id;	// region id
	unsigned char nei;	// neighbour id
};

/// @par
///
/// See the #rcConfig documentation for more information on the configuration parameters.
/// 構成パラメーターの詳細については、＃rcConfigのドキュメントを参照してください。
///
/// @see rcAllocHeightfieldLayerSet, rcCompactHeightfield, rcHeightfieldLayerSet, rcConfig
bool rcBuildHeightfieldLayers(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int walkableHeight,
	rcHeightfieldLayerSet& lset)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_BUILD_LAYERS);

	const int w = chf.width;
	const int h = chf.height;

	rcScopedDelete<unsigned char> srcReg((unsigned char*)rcAlloc(sizeof(unsigned char) * chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'srcReg' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg, 0xff, sizeof(unsigned char) * chf.spanCount);

	const int nsweeps = chf.width;
	rcScopedDelete<rcLayerSweepSpan> sweeps((rcLayerSweepSpan*)rcAlloc(sizeof(rcLayerSweepSpan) * nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}

	// Partition walkable area into monotone regions.
	// 歩行可能領域をモノトーン領域に分割します。
	int prevCount[256];
	unsigned char regId = 0;

	for (int y = borderSize; y < h - borderSize; ++y)
	{
		memset(prevCount, 0, sizeof(int) * regId);
		unsigned char sweepId = 0;

		for (int x = borderSize; x < w - borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];

				if (chf.areas[i] == RC_NULL_AREA) continue;

				unsigned char sid = 0xff;

				// -x
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, 0);
					if (chf.areas[ai] != RC_NULL_AREA && srcReg[ai] != 0xff)
						sid = srcReg[ai];
				}

				if (sid == 0xff)
				{
					sid = sweepId++;
					sweeps[sid].nei = 0xff;
					sweeps[sid].ns = 0;
				}

				// -y
				if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, 3);
					const unsigned char nr = srcReg[ai];
					if (nr != 0xff)
					{
						// Set neighbour when first valid neighbour is encoutered.
						// 最初の有効な近隣が検出されたときに隣を設定します。
						if (sweeps[sid].ns == 0)
							sweeps[sid].nei = nr;

						if (sweeps[sid].nei == nr)
						{
							// Update existing neighbour
							// 既存の隣を更新します
							sweeps[sid].ns++;
							prevCount[nr]++;
						}
						else
						{
							// This is hit if there is nore than one neighbour.
							// Invalidate the neighbour.
							// これは、複数の隣がいる場合にヒットします。隣を無効にします。
							sweeps[sid].nei = 0xff;
						}
					}
				}

				srcReg[i] = sid;
			}
		}

		// Create unique ID.
		// 一意のIDを作成します。
		for (int i = 0; i < sweepId; ++i)
		{
			// If the neighbour is set and there is only one continuous connection to it,
			// the sweep will be merged with the previous one, else new region is created.
			// 隣が設定されており、隣への連続接続が1つしかない場合、
			// スイープは前のスイープとマージされ、そうでない場合は新しい領域が作成されます。
			if (sweeps[i].nei != 0xff && prevCount[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				if (regId == 255)
				{
					ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Region ID overflow.");
					return false;
				}
				sweeps[i].id = regId++;
			}
		}

		// Remap local sweep ids to region ids.
		// ローカルスイープIDをリージョンIDに再マッピングします。
		for (int x = borderSize; x < w - borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				if (srcReg[i] != 0xff)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}

	// Allocate and init layer regions.
	// レイヤー領域を割り当てて初期化します。
	const int nregs = (int)regId;
	rcScopedDelete<rcLayerRegion> regs((rcLayerRegion*)rcAlloc(sizeof(rcLayerRegion) * nregs, RC_ALLOC_TEMP));
	if (!regs)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'regs' (%d).", nregs);
		return false;
	}
	memset(regs, 0, sizeof(rcLayerRegion) * nregs);
	for (int i = 0; i < nregs; ++i)
	{
		regs[i].layerId = 0xff;
		regs[i].ymin = 0xffff;
		regs[i].ymax = 0;
	}

	// Find region neighbours and overlapping regions.
	// 隣接する地域と重複する地域を検索します。
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];

			unsigned char lregs[RC_MAX_LAYERS];
			int nlregs = 0;

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned char ri = srcReg[i];
				if (ri == 0xff) continue;

				regs[ri].ymin = rcMin(regs[ri].ymin, s.y);
				regs[ri].ymax = rcMax(regs[ri].ymax, s.y);

				// Collect all region layers.
				// すべてのリージョンレイヤーを収集します。
				if (nlregs < RC_MAX_LAYERS)
					lregs[nlregs++] = ri;

				// Update neighbours
				// 隣を更新します
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
						const unsigned char rai = srcReg[ai];
						if (rai != 0xff && rai != ri)
						{
							// Don't check return value -- if we cannot add the neighbor
							// it will just cause a few more regions to be created, which
							// is fine.
							// 戻り値をチェックしない-隣人を追加できない場合、
							// さらにいくつかの領域が作成されますが、これは問題ありません。
							addUnique(regs[ri].neis, regs[ri].nneis, RC_MAX_NEIS, rai);
						}
					}
				}
			}

			// Update overlapping regions.
			// 重複する領域を更新します。
			for (int i = 0; i < nlregs - 1; ++i)
			{
				for (int j = i + 1; j < nlregs; ++j)
				{
					if (lregs[i] != lregs[j])
					{
						rcLayerRegion& ri = regs[lregs[i]];
						rcLayerRegion& rj = regs[lregs[j]];

						if (!addUnique(ri.layers, ri.nlayers, RC_MAX_LAYERS, lregs[j]) ||
							!addUnique(rj.layers, rj.nlayers, RC_MAX_LAYERS, lregs[i]))
						{
							ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: layer overflow (too many overlapping walkable platforms). Try increasing RC_MAX_LAYERS.");
							return false;
						}
					}
				}
			}
		}
	}

	// Create 2D layers from regions.
	// 領域から2Dレイヤーを作成します。
	unsigned char layerId = 0;

	static const int MAX_STACK = 64;
	unsigned char stack[MAX_STACK];
	int nstack = 0;

	for (int i = 0; i < nregs; ++i)
	{
		rcLayerRegion& root = regs[i];

		// Skip already visited.
		// 既にアクセスしたものをスキップします。
		if (root.layerId != 0xff)
			continue;

		// Start search.
		root.layerId = layerId;
		root.base = 1;

		nstack = 0;
		stack[nstack++] = (unsigned char)i;

		while (nstack)
		{
			// Pop front
			rcLayerRegion& reg = regs[stack[0]];
			nstack--;
			for (int j = 0; j < nstack; ++j)
				stack[j] = stack[j + 1];

			const int nneis = (int)reg.nneis;
			for (int j = 0; j < nneis; ++j)
			{
				const unsigned char nei = reg.neis[j];
				rcLayerRegion& regn = regs[nei];

				// Skip already visited.
				// 既にアクセスしたものをスキップします。
				if (regn.layerId != 0xff) continue;

				// Skip if the neighbour is overlapping root region.
				// 隣がルート領域と重複している場合はスキップします。
				if (contains(root.layers, root.nlayers, nei)) continue;

				// Skip if the height range would become too large.
				// 高さの範囲が大きくなりすぎる場合はスキップします。
				const int ymin = rcMin(root.ymin, regn.ymin);
				const int ymax = rcMax(root.ymax, regn.ymax);

				if ((ymax - ymin) >= 255) continue;

				if (nstack < MAX_STACK)
				{
					// Deepen
					stack[nstack++] = (unsigned char)nei;

					// Mark layer id
					// レイヤーIDをマークします
					regn.layerId = layerId;

					// Merge current layers to root.
					// 現在のレイヤーをルートにマージします。
					for (int k = 0; k < regn.nlayers; ++k)
					{
						if (!addUnique(root.layers, root.nlayers, RC_MAX_LAYERS, regn.layers[k]))
						{
							ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: layer overflow (too many overlapping walkable platforms). Try increasing RC_MAX_LAYERS.");
							return false;
						}
					}
					root.ymin = rcMin(root.ymin, regn.ymin);
					root.ymax = rcMax(root.ymax, regn.ymax);
				}
			}
		}

		layerId++;
	}

	// Merge non-overlapping regions that are close in height.
	// 高さが近い重複しない領域をマージします。
	const unsigned short mergeHeight = (unsigned short)walkableHeight * 4;

	for (int i = 0; i < nregs; ++i)
	{
		rcLayerRegion& ri = regs[i];

		if (!ri.base) continue;

		unsigned char newId = ri.layerId;

		for (;;)
		{
			unsigned char oldId = 0xff;

			for (int j = 0; j < nregs; ++j)
			{
				if (i == j) continue;

				rcLayerRegion& rj = regs[j];

				if (!rj.base) continue;

				// Skip if the regions are not close to each other.
				// リージョンが互いに近接していない場合はスキップします。
				if (!overlapRange(ri.ymin, ri.ymax + mergeHeight, rj.ymin, rj.ymax + mergeHeight))
					continue;

				// Skip if the height range would become too large.
				// 高さの範囲が大きくなりすぎる場合はスキップします。
				const int ymin = rcMin(ri.ymin, rj.ymin);
				const int ymax = rcMax(ri.ymax, rj.ymax);

				if ((ymax - ymin) >= 255) continue;

				// Make sure that there is no overlap when merging 'ri' and 'rj'.
				// 'ri'と 'rj'をマージするときに重複がないことを確認します。
				bool overlap = false;

				// Iterate over all regions which have the same layerId as 'rj'
				// 'rj'と同じレイヤーを持つすべての領域を反復処理します
				for (int k = 0; k < nregs; ++k)
				{
					if (regs[k].layerId != rj.layerId) continue;

					// Check if region 'k' is overlapping region 'ri'
					// Index to 'regs' is the same as region id.
					// 領域 'k'が領域 'ri'に重複しているかどうかを確認します。
					// 'regs 'へのインデックスは、領域IDと同じです。
					if (contains(ri.layers, ri.nlayers, (unsigned char)k))
					{
						overlap = true;
						break;
					}
				}

				// Cannot merge of regions overlap.
				// リージョンのオーバーラップをマージできません。
				if (overlap) continue;

				// Can merge i and j.
				// iとjをマージできます。
				oldId = rj.layerId;
				break;
			}

			// Could not find anything to merge with, stop.
			//マージするものが見つかりませんでした。停止します。
			if (oldId == 0xff) break;

			// Merge
			for (int j = 0; j < nregs; ++j)
			{
				rcLayerRegion& rj = regs[j];
				if (rj.layerId == oldId)
				{
					rj.base = 0;
					// Remap layerIds.
					rj.layerId = newId;

					// Add overlaid layers from 'rj' to 'ri'.
					// オーバーレイレイヤーを「rj」から「ri」に追加します。
					for (int k = 0; k < rj.nlayers; ++k)
					{
						if (!addUnique(ri.layers, ri.nlayers, RC_MAX_LAYERS, rj.layers[k]))
						{
							ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: layer overflow (too many overlapping walkable platforms). Try increasing RC_MAX_LAYERS.");
							return false;
						}
					}

					// Update height bounds.
					// 高さの境界を更新します。
					ri.ymin = rcMin(ri.ymin, rj.ymin);
					ri.ymax = rcMax(ri.ymax, rj.ymax);
				}
			}
		}
	}

	// Compact layer Ids
	// コンパクトレイヤーID
	unsigned char remap[256];
	memset(remap, 0, 256);

	// Find number of unique layers.
	// 一意のレイヤーの数を見つけます。
	layerId = 0;

	for (int i = 0; i < nregs; ++i)
		remap[regs[i].layerId] = 1;

	for (int i = 0; i < 256; ++i)
	{
		if (remap[i])
			remap[i] = layerId++;
		else
			remap[i] = 0xff;
	}

	// Remap ids.
	// IDを再マップします。
	for (int i = 0; i < nregs; ++i)
		regs[i].layerId = remap[regs[i].layerId];

	// No layers, return empty.
	//レイヤーなし、空を返します。
	if (layerId == 0)
		return true;

	// Create layers.
	rcAssert(lset.layers == 0);

	const int lw = w - borderSize * 2;
	const int lh = h - borderSize * 2;

	// Build contracted bbox for layers.
	// レイヤーのコントラクトbboxを構築します。
	float bmin[3], bmax[3];
	rcVcopy(bmin, chf.bmin);
	rcVcopy(bmax, chf.bmax);
	bmin[0] += borderSize * chf.cs;
	bmin[2] += borderSize * chf.cs;
	bmax[0] -= borderSize * chf.cs;
	bmax[2] -= borderSize * chf.cs;

	lset.nlayers = (int)layerId;

	lset.layers = (rcHeightfieldLayer*)rcAlloc(sizeof(rcHeightfieldLayer) * lset.nlayers, RC_ALLOC_PERM);
	if (!lset.layers)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'layers' (%d).", lset.nlayers);
		return false;
	}
	memset(lset.layers, 0, sizeof(rcHeightfieldLayer) * lset.nlayers);

	// Store layers.
	//レイヤーを保存します。
	for (int i = 0; i < lset.nlayers; ++i)
	{
		unsigned char curId = (unsigned char)i;

		rcHeightfieldLayer* layer = &lset.layers[i];

		const int gridSize = sizeof(unsigned char) * lw * lh;

		layer->heights = (unsigned char*)rcAlloc(gridSize, RC_ALLOC_PERM);
		if (!layer->heights)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'heights' (%d).", gridSize);
			return false;
		}
		memset(layer->heights, 0xff, gridSize);

		layer->areas = (unsigned char*)rcAlloc(gridSize, RC_ALLOC_PERM);
		if (!layer->areas)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'areas' (%d).", gridSize);
			return false;
		}
		memset(layer->areas, 0, gridSize);

		layer->cons = (unsigned char*)rcAlloc(gridSize, RC_ALLOC_PERM);
		if (!layer->cons)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'cons' (%d).", gridSize);
			return false;
		}
		memset(layer->cons, 0, gridSize);

		// Find layer height bounds.
		// レイヤーの高さの境界を見つけます。
		int hmin = 0, hmax = 0;
		for (int j = 0; j < nregs; ++j)
		{
			if (regs[j].base && regs[j].layerId == curId)
			{
				hmin = (int)regs[j].ymin;
				hmax = (int)regs[j].ymax;
			}
		}

		layer->width = lw;
		layer->height = lh;
		layer->cs = chf.cs;
		layer->ch = chf.ch;

		// Adjust the bbox to fit the heightfield.
		// bboxを調整して、地形に合わせます。
		rcVcopy(layer->bmin, bmin);
		rcVcopy(layer->bmax, bmax);
		layer->bmin[1] = bmin[1] + hmin * chf.ch;
		layer->bmax[1] = bmin[1] + hmax * chf.ch;
		layer->hmin = hmin;
		layer->hmax = hmax;

		// Update usable data region.
		// 使用可能なデータ領域を更新します。
		layer->minx = layer->width;
		layer->maxx = 0;
		layer->miny = layer->height;
		layer->maxy = 0;

		// Copy height and area from compact heightfield.
		// コンパクトな高さフィールドから高さと面積をコピーします。
		for (int y = 0; y < lh; ++y)
		{
			for (int x = 0; x < lw; ++x)
			{
				const int cx = borderSize + x;
				const int cy = borderSize + y;
				const rcCompactCell& c = chf.cells[cx + cy * w];
				for (int j = (int)c.index, nj = (int)(c.index + c.count); j < nj; ++j)
				{
					const rcCompactSpan& s = chf.spans[j];

					// Skip unassigned regions.
					// 未割り当ての領域をスキップします。
					if (srcReg[j] == 0xff) continue;

					// Skip of does nto belong to current layer.
					// 現在のレイヤーに属さない場合はスキップします。
					unsigned char lid = regs[srcReg[j]].layerId;
					if (lid != curId) continue;

					// Update data bounds.
					// データ境界を更新します。
					layer->minx = rcMin(layer->minx, x);
					layer->maxx = rcMax(layer->maxx, x);
					layer->miny = rcMin(layer->miny, y);
					layer->maxy = rcMax(layer->maxy, y);

					// Store height and area type.
					// 高さと面積の種類を保存します。
					const int idx = x + y * lw;
					layer->heights[idx] = (unsigned char)(s.y - hmin);
					layer->areas[idx] = chf.areas[j];

					// Check connection.
					// 接続を確認します。
					unsigned char portal = 0;
					unsigned char con = 0;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int ax = cx + rcGetDirOffsetX(dir);
							const int ay = cy + rcGetDirOffsetY(dir);
							const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
							unsigned char alid = srcReg[ai] != 0xff ? regs[srcReg[ai]].layerId : 0xff;

							// Portal mask
							// ポータルマスク
							if (chf.areas[ai] != RC_NULL_AREA && lid != alid)
							{
								portal |= (unsigned char)(1 << dir);
								// Update height so that it matches on both sides of the portal.
								// 高さを更新して、ポータルの両側で一致するようにします。
								const rcCompactSpan& as = chf.spans[ai];

								if (as.y > hmin)
									layer->heights[idx] = rcMax(layer->heights[idx], (unsigned char)(as.y - hmin));
							}

							// Valid connection mask
							// 有効な接続マスク
							if (chf.areas[ai] != RC_NULL_AREA && lid == alid)
							{
								const int nx = ax - borderSize;
								const int ny = ay - borderSize;
								if (nx >= 0 && ny >= 0 && nx < lw && ny < lh)
									con |= (unsigned char)(1 << dir);
							}
						}
					}

					layer->cons[idx] = (portal << 4) | con;
				}
			}
		}

		if (layer->minx > layer->maxx)
			layer->minx = layer->maxx = 0;
		if (layer->miny > layer->maxy)
			layer->miny = layer->maxy = 0;
	}

	return true;
}
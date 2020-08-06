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
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

namespace
{
	struct LevelStackEntry
	{
		LevelStackEntry(int x_, int y_, int index_) : x(x_), y(y_), index(index_) {}
		int x;
		int y;
		int index;
	};

	void calculateDistanceField(rcCompactHeightfield& chf, unsigned short* src, unsigned short& maxDist)
	{
		const int w{ chf.width };
		const int h{ chf.height };

		// Init distance and points.
		// 距離とポイントを初期化します。
		for (int i = 0; i < chf.spanCount; ++i)
			src[i] = 0xffff;

		// Mark boundary cells.
		// 境界セルをマークします。
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c{ chf.cells[x + y * w] };

				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					const rcCompactSpan& s{ chf.spans[i] };
					const unsigned char area{ chf.areas[i] };

					int nc{ 0 };

					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int ax{ x + rcGetDirOffsetX(dir) };
							const int ay{ y + rcGetDirOffsetY(dir) };
							const int ai{ (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir) };

							if (area == chf.areas[ai]) nc++;
						}
					}
					if (nc != 4) src[i] = 0;
				}
			}
		}

		// Pass 1
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c{ chf.cells[x + y * w] };

				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					const rcCompactSpan& s = chf.spans[i];

					if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
					{
						// (-1,0)
						const int ax{ x + rcGetDirOffsetX(0) };
						const int ay{ y + rcGetDirOffsetY(0) };
						const int ai{ (int)chf.cells[ax + ay * w].index + rcGetCon(s, 0) };
						const rcCompactSpan& as{ chf.spans[ai] };

						if (src[ai] + 2 < src[i])
							src[i] = src[ai] + 2;

						// (-1,-1)
						if (rcGetCon(as, 3) != RC_NOT_CONNECTED)
						{
							const int aax{ ax + rcGetDirOffsetX(3) };
							const int aay{ ay + rcGetDirOffsetY(3) };
							const int aai{ (int)chf.cells[aax + aay * w].index + rcGetCon(as, 3) };

							if (src[aai] + 3 < src[i])
								src[i] = src[aai] + 3;
						}
					}

					if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
					{
						// (0,-1)
						const int ax{ x + rcGetDirOffsetX(3) };
						const int ay{ y + rcGetDirOffsetY(3) };
						const int ai{ (int)chf.cells[ax + ay * w].index + rcGetCon(s, 3) };
						const rcCompactSpan& as{ chf.spans[ai] };

						if (src[ai] + 2 < src[i])
							src[i] = src[ai] + 2;

						// (1,-1)
						if (rcGetCon(as, 2) != RC_NOT_CONNECTED)
						{
							const int aax{ ax + rcGetDirOffsetX(2) };
							const int aay{ ay + rcGetDirOffsetY(2) };
							const int aai{ (int)chf.cells[aax + aay * w].index + rcGetCon(as, 2) };

							if (src[aai] + 3 < src[i])
								src[i] = src[aai] + 3;
						}
					}
				}
			}
		}

		// Pass 2
		for (int y = h - 1; y >= 0; --y)
		{
			for (int x = w - 1; x >= 0; --x)
			{
				const rcCompactCell& c{ chf.cells[x + y * w] };

				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					const rcCompactSpan& s{ chf.spans[i] };

					if (rcGetCon(s, 2) != RC_NOT_CONNECTED)
					{
						// (1,0)
						const int ax{ x + rcGetDirOffsetX(2) };
						const int ay{ y + rcGetDirOffsetY(2) };
						const int ai{ (int)chf.cells[ax + ay * w].index + rcGetCon(s, 2) };
						const rcCompactSpan& as{ chf.spans[ai] };

						if (src[ai] + 2 < src[i])
							src[i] = src[ai] + 2;

						// (1,1)
						if (rcGetCon(as, 1) != RC_NOT_CONNECTED)
						{
							const int aax{ ax + rcGetDirOffsetX(1) };
							const int aay{ ay + rcGetDirOffsetY(1) };
							const int aai{ (int)chf.cells[aax + aay * w].index + rcGetCon(as, 1) };

							if (src[aai] + 3 < src[i])
								src[i] = src[aai] + 3;
						}
					}
					if (rcGetCon(s, 1) != RC_NOT_CONNECTED)
					{
						// (0,1)
						const int ax{ x + rcGetDirOffsetX(1) };
						const int ay{ y + rcGetDirOffsetY(1) };
						const int ai{ (int)chf.cells[ax + ay * w].index + rcGetCon(s, 1) };
						const rcCompactSpan& as{ chf.spans[ai] };

						if (src[ai] + 2 < src[i])
							src[i] = src[ai] + 2;

						// (-1,1)
						if (rcGetCon(as, 0) != RC_NOT_CONNECTED)
						{
							const int aax{ ax + rcGetDirOffsetX(0) };
							const int aay{ ay + rcGetDirOffsetY(0) };
							const int aai{ (int)chf.cells[aax + aay * w].index + rcGetCon(as, 0) };

							if (src[aai] + 3 < src[i])
								src[i] = src[aai] + 3;
						}
					}
				}
			}
		}

		maxDist = 0;

		for (int i = 0; i < chf.spanCount; ++i)
			maxDist = rcMax(src[i], maxDist);
	}

	unsigned short* boxBlur(rcCompactHeightfield& chf, int thr,
		unsigned short* src, unsigned short* dst)
	{
		const int w = chf.width;
		const int h = chf.height;

		thr *= 2;

		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.cells[x + y * w];
				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					const rcCompactSpan& s = chf.spans[i];
					const unsigned short cd = src[i];
					if (cd <= thr)
					{
						dst[i] = cd;
						continue;
					}

					int d = (int)cd;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int ax = x + rcGetDirOffsetX(dir);
							const int ay = y + rcGetDirOffsetY(dir);
							const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
							d += (int)src[ai];

							const rcCompactSpan& as = chf.spans[ai];
							const int dir2 = (dir + 1) & 0x3;
							if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
							{
								const int ax2 = ax + rcGetDirOffsetX(dir2);
								const int ay2 = ay + rcGetDirOffsetY(dir2);
								const int ai2 = (int)chf.cells[ax2 + ay2 * w].index + rcGetCon(as, dir2);
								d += (int)src[ai2];
							}
							else
							{
								d += cd;
							}
						}
						else
						{
							d += cd * 2;
						}
					}
					dst[i] = (unsigned short)((d + 5) / 9);
				}
			}
		}
		return dst;
	}

	bool floodRegion(int x, int y, int i,
		unsigned short level, unsigned short r,
		rcCompactHeightfield& chf,
		unsigned short* srcReg, unsigned short* srcDist,
		rcTempVector<LevelStackEntry>& stack)
	{
		const int w = chf.width;

		const unsigned char area = chf.areas[i];

		// Flood fill mark region.
		stack.clear();
		stack.push_back(LevelStackEntry(x, y, i));
		srcReg[i] = r;
		srcDist[i] = 0;

		unsigned short lev = level >= 2 ? level - 2 : 0;
		int count = 0;

		while (stack.size() > 0)
		{
			LevelStackEntry& back = stack.back();
			int cx = back.x;
			int cy = back.y;
			int ci = back.index;
			stack.pop_back();

			const rcCompactSpan& cs = chf.spans[ci];

			// Check if any of the neighbours already have a valid region set.
			unsigned short ar = 0;
			for (int dir = 0; dir < 4; ++dir)
			{
				// 8 connected
				if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
				{
					const int ax = cx + rcGetDirOffsetX(dir);
					const int ay = cy + rcGetDirOffsetY(dir);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(cs, dir);
					if (chf.areas[ai] != area)
						continue;
					unsigned short nr = srcReg[ai];
					if (nr & RC_BORDER_REG) // Do not take borders into account.
						continue;
					if (nr != 0 && nr != r)
					{
						ar = nr;
						break;
					}

					const rcCompactSpan& as = chf.spans[ai];

					const int dir2 = (dir + 1) & 0x3;
					if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
					{
						const int ax2 = ax + rcGetDirOffsetX(dir2);
						const int ay2 = ay + rcGetDirOffsetY(dir2);
						const int ai2 = (int)chf.cells[ax2 + ay2 * w].index + rcGetCon(as, dir2);
						if (chf.areas[ai2] != area)
							continue;
						unsigned short nr2 = srcReg[ai2];
						if (nr2 != 0 && nr2 != r)
						{
							ar = nr2;
							break;
						}
					}
				}
			}
			if (ar != 0)
			{
				srcReg[ci] = 0;
				continue;
			}

			count++;

			// Expand neighbours.
			for (int dir = 0; dir < 4; ++dir)
			{
				if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
				{
					const int ax = cx + rcGetDirOffsetX(dir);
					const int ay = cy + rcGetDirOffsetY(dir);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(cs, dir);
					if (chf.areas[ai] != area)
						continue;
					if (chf.dist[ai] >= lev && srcReg[ai] == 0)
					{
						srcReg[ai] = r;
						srcDist[ai] = 0;
						stack.push_back(LevelStackEntry(ax, ay, ai));
					}
				}
			}
		}

		return count > 0;
	}

	// Struct to keep track of entries in the region table that have been changed.
	struct DirtyEntry
	{
		DirtyEntry(int index_, unsigned short region_, unsigned short distance2_)
			: index(index_), region(region_), distance2(distance2_) {}
		int index;
		unsigned short region;
		unsigned short distance2;
	};

	void expandRegions(int maxIter, unsigned short level,
		rcCompactHeightfield& chf,
		unsigned short* srcReg, unsigned short* srcDist,
		rcTempVector<LevelStackEntry>& stack,
		bool fillStack)
	{
		const int w = chf.width;
		const int h = chf.height;

		if (fillStack)
		{
			// Find cells revealed by the raised level.
			// 上げられたレベルによって明らかにされたセルを見つけます。
			stack.clear();
			for (int y = 0; y < h; ++y)
			{
				for (int x = 0; x < w; ++x)
				{
					const rcCompactCell& c = chf.cells[x + y * w];
					for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
					{
						if (chf.dist[i] >= level && srcReg[i] == 0 && chf.areas[i] != RC_NULL_AREA)
						{
							stack.push_back(LevelStackEntry(x, y, i));
						}
					}
				}
			}
		}
		// use cells in the input stack
		// 入力スタックでセルを使用します
		else
		{
			// mark all cells which already have a region
			// すでに領域があるすべてのセルをマークします
			for (int j = 0; j < stack.size(); j++)
			{
				int i = stack[j].index;
				if (srcReg[i] != 0)
					stack[j].index = -1;
			}
		}

		rcTempVector<DirtyEntry> dirtyEntries;
		int iter = 0;
		while (stack.size() > 0)
		{
			int failed = 0;
			dirtyEntries.clear();

			for (int j = 0; j < stack.size(); j++)
			{
				int x = stack[j].x;
				int y = stack[j].y;
				int i = stack[j].index;
				if (i < 0)
				{
					failed++;
					continue;
				}

				unsigned short r = srcReg[i];
				unsigned short d2 = 0xffff;
				const unsigned char area = chf.areas[i];
				const rcCompactSpan& s = chf.spans[i];
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) == RC_NOT_CONNECTED) continue;
					const int ax = x + rcGetDirOffsetX(dir);
					const int ay = y + rcGetDirOffsetY(dir);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
					if (chf.areas[ai] != area) continue;
					if (srcReg[ai] > 0 && (srcReg[ai] & RC_BORDER_REG) == 0)
					{
						if ((int)srcDist[ai] + 2 < (int)d2)
						{
							r = srcReg[ai];
							d2 = srcDist[ai] + 2;
						}
					}
				}
				if (r)
				{
					stack[j].index = -1; // mark as used
					dirtyEntries.push_back(DirtyEntry(i, r, d2));
				}
				else
				{
					failed++;
				}
			}

			// Copy entries that differ between src and dst to keep them in sync.
			for (int i = 0; i < dirtyEntries.size(); i++) {
				int idx = dirtyEntries[i].index;
				srcReg[idx] = dirtyEntries[i].region;
				srcDist[idx] = dirtyEntries[i].distance2;
			}

			if (failed == stack.size())
				break;

			if (level > 0)
			{
				++iter;
				if (iter >= maxIter)
					break;
			}
		}
	}

	void sortCellsByLevel(unsigned short startLevel,
		rcCompactHeightfield& chf,
		const unsigned short* srcReg,
		unsigned int nbStacks, rcTempVector<LevelStackEntry>* stacks,
		unsigned short loglevelsPerStack) // the levels per stack (2 in our case) as a bit shift
	{
		const int w = chf.width;
		const int h = chf.height;
		startLevel = startLevel >> loglevelsPerStack;

		for (unsigned int j = 0; j < nbStacks; ++j)
			stacks[j].clear();

		// put all cells in the level range into the appropriate stacks
		// レベル範囲内のすべてのセルを適切なスタックに配置します
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.cells[x + y * w];

				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					if (chf.areas[i] == RC_NULL_AREA || srcReg[i] != 0)
						continue;

					int level = chf.dist[i] >> loglevelsPerStack;
					int sId = startLevel - level;
					if (sId >= (int)nbStacks)
						continue;
					if (sId < 0)
						sId = 0;

					stacks[sId].push_back(LevelStackEntry(x, y, i));
				}
			}
		}
	}

	void appendStacks(const rcTempVector<LevelStackEntry>& srcStack,
		rcTempVector<LevelStackEntry>& dstStack,
		const unsigned short* srcReg)
	{
		for (int j = 0; j < srcStack.size(); j++)
		{
			int i = srcStack[j].index;
			if ((i < 0) || (srcReg[i] != 0))
				continue;
			dstStack.push_back(srcStack[j]);
		}
	}

	struct rcRegion
	{
		inline rcRegion(unsigned short i) :
			spanCount(0),
			id(i),
			areaType(0),
			remap(false),
			visited(false),
			overlap(false),
			connectsToBorder(false),
			ymin(0xffff),
			ymax(0)
		{}

		int spanCount;					// Number of spans belonging to this region //この領域に属するスパンの数
		unsigned short id;				// ID of the region //領域のID
		unsigned char areaType;			// Are type.　// タイプ
		bool remap;
		bool visited;
		bool overlap;
		bool connectsToBorder;
		unsigned short ymin, ymax;
		rcIntArray connections;
		rcIntArray floors;
	};

	void removeAdjacentNeighbours(rcRegion& reg)
	{
		// Remove adjacent duplicates.
		// 隣接する重複を削除します。
		for (int i = 0; i < reg.connections.size() && reg.connections.size() > 1; )
		{
			int ni = (i + 1) % reg.connections.size();
			if (reg.connections[i] == reg.connections[ni])
			{
				// Remove duplicate
				// 重複を削除します
				for (int j = i; j < reg.connections.size() - 1; ++j)
					reg.connections[j] = reg.connections[j + 1];
				reg.connections.pop();
			}
			else
				++i;
		}
	}

	void replaceNeighbour(rcRegion& reg, unsigned short oldId, unsigned short newId)
	{
		bool neiChanged = false;
		for (int i = 0; i < reg.connections.size(); ++i)
		{
			if (reg.connections[i] == oldId)
			{
				reg.connections[i] = newId;
				neiChanged = true;
			}
		}
		for (int i = 0; i < reg.floors.size(); ++i)
		{
			if (reg.floors[i] == oldId)
				reg.floors[i] = newId;
		}
		if (neiChanged)
			removeAdjacentNeighbours(reg);
	}

	bool canMergeWithRegion(const rcRegion& rega, const rcRegion& regb)
	{
		if (rega.areaType != regb.areaType)
			return false;
		int n = 0;
		for (int i = 0; i < rega.connections.size(); ++i)
		{
			if (rega.connections[i] == regb.id)
				n++;
		}
		if (n > 1)
			return false;
		for (int i = 0; i < rega.floors.size(); ++i)
		{
			if (rega.floors[i] == regb.id)
				return false;
		}
		return true;
	}

	void addUniqueFloorRegion(rcRegion& reg, int n)
	{
		for (int i = 0; i < reg.floors.size(); ++i)
			if (reg.floors[i] == n)
				return;
		reg.floors.push(n);
	}

	bool mergeRegions(rcRegion& rega, rcRegion& regb)
	{
		unsigned short aid = rega.id;
		unsigned short bid = regb.id;

		// Duplicate current neighbourhood.
		// 現在の周辺を複製します。
		rcIntArray acon;
		acon.resize(rega.connections.size());
		for (int i = 0; i < rega.connections.size(); ++i)
			acon[i] = rega.connections[i];
		rcIntArray& bcon = regb.connections;

		// Find insertion point on A.
		// Aの挿入ポイントを見つけます
		int insa = -1;
		for (int i = 0; i < acon.size(); ++i)
		{
			if (acon[i] == bid)
			{
				insa = i;
				break;
			}
		}
		if (insa == -1)
			return false;

		// Find insertion point on B.
		// Bの挿入ポイントを見つけます
		int insb = -1;
		for (int i = 0; i < bcon.size(); ++i)
		{
			if (bcon[i] == aid)
			{
				insb = i;
				break;
			}
		}
		if (insb == -1)
			return false;

		// Merge neighbours.
		// 周辺とマージする
		rega.connections.resize(0);
		for (int i = 0, ni = acon.size(); i < ni - 1; ++i)
			rega.connections.push(acon[(insa + 1 + i) % ni]);

		for (int i = 0, ni = bcon.size(); i < ni - 1; ++i)
			rega.connections.push(bcon[(insb + 1 + i) % ni]);

		removeAdjacentNeighbours(rega);

		for (int j = 0; j < regb.floors.size(); ++j)
			addUniqueFloorRegion(rega, regb.floors[j]);
		rega.spanCount += regb.spanCount;
		regb.spanCount = 0;
		regb.connections.resize(0);

		return true;
	}

	bool isRegionConnectedToBorder(const rcRegion& reg)
	{
		// Region is connected to border if
		// one of the neighbours is null id.
		// リージョンは次の場合にボーダーに接続されます
		// 近隣の1つがnull IDです。
		for (int i = 0; i < reg.connections.size(); ++i)
		{
			if (reg.connections[i] == 0)
				return true;
		}
		return false;
	}

	bool isSolidEdge(rcCompactHeightfield& chf, unsigned short* srcReg,
		int x, int y, int i, int dir)
	{
		const rcCompactSpan& s = chf.spans[i];
		unsigned short r = 0;
		if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
		{
			const int ax = x + rcGetDirOffsetX(dir);
			const int ay = y + rcGetDirOffsetY(dir);
			const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(s, dir);
			r = srcReg[ai];
		}
		if (r == srcReg[i])
			return false;
		return true;
	}

	void walkContour(int x, int y, int i, int dir,
		rcCompactHeightfield& chf,
		unsigned short* srcReg,
		rcIntArray& cont)
	{
		int startDir = dir;
		int starti = i;

		const rcCompactSpan& ss = chf.spans[i];
		unsigned short curReg = 0;
		if (rcGetCon(ss, dir) != RC_NOT_CONNECTED)
		{
			const int ax = x + rcGetDirOffsetX(dir);
			const int ay = y + rcGetDirOffsetY(dir);
			const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(ss, dir);
			curReg = srcReg[ai];
		}
		cont.push(curReg);

		int iter = 0;
		while (++iter < 40000)
		{
			const rcCompactSpan& s = chf.spans[i];

			if (isSolidEdge(chf, srcReg, x, y, i, dir))
			{
				// Choose the edge corner
				// エッジコーナーを選択します
				unsigned short r = 0;
				if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(dir);
					const int ay = y + rcGetDirOffsetY(dir);
					const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(s, dir);
					r = srcReg[ai];
				}
				if (r != curReg)
				{
					curReg = r;
					cont.push(curReg);
				}

				dir = (dir + 1) & 0x3;  // Rotate CW // 時計回り回る
			}
			else
			{
				int ni = -1;
				const int nx = x + rcGetDirOffsetX(dir);
				const int ny = y + rcGetDirOffsetY(dir);
				if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
				{
					const rcCompactCell& nc = chf.cells[nx + ny * chf.width];
					ni = (int)nc.index + rcGetCon(s, dir);
				}
				if (ni == -1)
				{
					// Should not happen.
					// 起こるべきではありません。
					return;
				}
				x = nx;
				y = ny;
				i = ni;
				dir = (dir + 3) & 0x3;	// Rotate CCW // 反時計回り回る
			}

			if (starti == i && startDir == dir)
			{
				break;
			}
		}

		// Remove adjacent duplicates.
		// 隣接する重複を削除します。
		if (cont.size() > 1)
		{
			for (int j = 0; j < cont.size(); )
			{
				int nj = (j + 1) % cont.size();
				if (cont[j] == cont[nj])
				{
					for (int k = j; k < cont.size() - 1; ++k)
						cont[k] = cont[k + 1];
					cont.pop();
				}
				else
					++j;
			}
		}
	}

	bool mergeAndFilterRegions(rcContext* ctx, int minRegionArea, int mergeRegionSize,
		unsigned short& maxRegionId,
		rcCompactHeightfield& chf,
		unsigned short* srcReg, rcIntArray& overlaps)
	{
		const int w = chf.width;
		const int h = chf.height;

		const int nreg = maxRegionId + 1;
		rcTempVector<rcRegion> regions;

		if (!regions.reserve(nreg)) {
			ctx->log(RC_LOG_ERROR, "mergeAndFilterRegions: Out of memory 'regions' (%d).", nreg);
			return false;
		}

		// Construct regions
		// 領域を構築します
		for (int i = 0; i < nreg; ++i)
			regions.push_back(rcRegion((unsigned short)i));

		// Find edge of a region and find connections around the contour.
		// 領域のエッジを見つけ、輪郭の周りの接続を見つけます。
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.cells[x + y * w];
				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					unsigned short r = srcReg[i];
					if (r == 0 || r >= nreg)
						continue;

					rcRegion& reg = regions[r];
					reg.spanCount++;

					// Update floors.
					// フロアを更新します。
					for (int j = (int)c.index; j < ni; ++j)
					{
						if (i == j) continue;
						unsigned short floorId = srcReg[j];
						if (floorId == 0 || floorId >= nreg)
							continue;
						if (floorId == r)
							reg.overlap = true;
						addUniqueFloorRegion(reg, floorId);
					}

					// Have found contour
					// 輪郭を見つけました
					if (reg.connections.size() > 0)
						continue;

					reg.areaType = chf.areas[i];

					// Check if this cell is next to a border.
					// このセルが境界線の隣にあるかどうかを確認します。
					int ndir = -1;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (isSolidEdge(chf, srcReg, x, y, i, dir))
						{
							ndir = dir;
							break;
						}
					}

					if (ndir != -1)
					{
						// The cell is at border.
						// Walk around the contour to find all the neighbours.
						// セルは境界にあります。
						// 輪郭を歩き回り、すべての隣人を見つけます。
						walkContour(x, y, i, ndir, chf, srcReg, reg.connections);
					}
				}
			}
		}

		// Remove too small regions.
		// 小さすぎる領域を削除します。
		rcIntArray stack(32);
		rcIntArray trace(32);
		for (int i = 0; i < nreg; ++i)
		{
			rcRegion& reg = regions[i];
			if (reg.id == 0 || (reg.id & RC_BORDER_REG))
				continue;
			if (reg.spanCount == 0)
				continue;
			if (reg.visited)
				continue;

			// Count the total size of all the connected regions.
			// Also keep track of the regions connects to a tile border.
			// 接続されているすべての領域の合計サイズをカウントします。
			// また、タイル境界に接続する領域を追跡します。
			bool connectsToBorder = false;
			int spanCount = 0;
			stack.resize(0);
			trace.resize(0);

			reg.visited = true;
			stack.push(i);

			while (stack.size())
			{
				// Pop
				int ri = stack.pop();

				rcRegion& creg = regions[ri];

				spanCount += creg.spanCount;
				trace.push(ri);

				for (int j = 0; j < creg.connections.size(); ++j)
				{
					if (creg.connections[j] & RC_BORDER_REG)
					{
						connectsToBorder = true;
						continue;
					}
					rcRegion& neireg = regions[creg.connections[j]];
					if (neireg.visited)
						continue;
					if (neireg.id == 0 || (neireg.id & RC_BORDER_REG))
						continue;
					// Visit
					stack.push(neireg.id);
					neireg.visited = true;
				}
			}

			// If the accumulated regions size is too small, remove it.
			// Do not remove areas which connect to tile borders
			// as their size cannot be estimated correctly and removing them
			// can potentially remove necessary areas.
			// 蓄積された領域のサイズが小さすぎる場合は、削除します。
			// サイズを正しく推定できないため、タイルの境界に接続する領域を削除しないでください。
			// 削除すると、必要な領域が削除される可能性があります。
			if (spanCount < minRegionArea && !connectsToBorder)
			{
				// Kill all visited regions.
				// 訪問したすべてのリージョンを強制終了します。
				for (int j = 0; j < trace.size(); ++j)
				{
					regions[trace[j]].spanCount = 0;
					regions[trace[j]].id = 0;
				}
			}
		}

		// Merge too small regions to neighbour regions.
		// 小さすぎる領域を隣接する領域にマージします。
		int mergeCount = 0;
		do
		{
			mergeCount = 0;
			for (int i = 0; i < nreg; ++i)
			{
				rcRegion& reg = regions[i];
				if (reg.id == 0 || (reg.id & RC_BORDER_REG))
					continue;
				if (reg.overlap)
					continue;
				if (reg.spanCount == 0)
					continue;

				// Check to see if the region should be merged.
				// 領域をマージする必要があるかどうかを確認します。
				if (reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg))
					continue;

				// Small region with more than 1 connection.
				// Or region which is not connected to a border at all.
				// Find smallest neighbour region that connects to this one.
				// 複数の接続がある小さな領域。
				// または境界にまったく接続されていない地域。
				// これに接続する最小の隣接領域を見つけます。
				int smallest = 0xfffffff;
				unsigned short mergeId = reg.id;
				for (int j = 0; j < reg.connections.size(); ++j)
				{
					if (reg.connections[j] & RC_BORDER_REG) continue;
					rcRegion& mreg = regions[reg.connections[j]];
					if (mreg.id == 0 || (mreg.id & RC_BORDER_REG) || mreg.overlap) continue;
					if (mreg.spanCount < smallest &&
						canMergeWithRegion(reg, mreg) &&
						canMergeWithRegion(mreg, reg))
					{
						smallest = mreg.spanCount;
						mergeId = mreg.id;
					}
				}

				// Found new id.
				// 新しいIDが見つかりました。
				if (mergeId != reg.id)
				{
					unsigned short oldId = reg.id;
					rcRegion& target = regions[mergeId];

					// Merge neighbours.
					// 周辺とマージします。
					if (mergeRegions(target, reg))
					{
						// Fixup regions pointing to current region.
						// 現在の領域を指す修正領域。
						for (int j = 0; j < nreg; ++j)
						{
							if (regions[j].id == 0 || (regions[j].id & RC_BORDER_REG)) continue;

							// If another region was already merged into current region
							// change the nid of the previous region too.
							// 別の領域がすでに現在の領域にマージされている場合、前の領域のIDも変更します。
							if (regions[j].id == oldId)
								regions[j].id = mergeId;

							// Replace the current region with the new one if the
							// current regions is neighbour.
							// 現在の領域が隣接している場合、現在の領域を新しい領域に置き換えます。
							replaceNeighbour(regions[j], oldId, mergeId);
						}
						mergeCount++;
					}
				}
			}
		} while (mergeCount > 0);

		// Compress region Ids.
		// リージョンIDを圧縮します。
		for (int i = 0; i < nreg; ++i)
		{
			regions[i].remap = false;
			if (regions[i].id == 0) continue;       // Skip nil regions. //ゼロ領域をスキップします。
			if (regions[i].id & RC_BORDER_REG) continue;    // Skip external regions. //外部領域をスキップします。
			regions[i].remap = true;
		}

		unsigned short regIdGen = 0;
		for (int i = 0; i < nreg; ++i)
		{
			if (!regions[i].remap)
				continue;
			unsigned short oldId = regions[i].id;
			unsigned short newId = ++regIdGen;
			for (int j = i; j < nreg; ++j)
			{
				if (regions[j].id == oldId)
				{
					regions[j].id = newId;
					regions[j].remap = false;
				}
			}
		}
		maxRegionId = regIdGen;

		// Remap regions.
		// 領域を再マップします。
		for (int i = 0; i < chf.spanCount; ++i)
		{
			if ((srcReg[i] & RC_BORDER_REG) == 0)
				srcReg[i] = regions[srcReg[i]].id;
		}

		// Return regions that we found to be overlapping.
		// 重複していることがわかった領域を返します。
		for (int i = 0; i < nreg; ++i)
			if (regions[i].overlap)
				overlaps.push(regions[i].id);

		return true;
	}

	void addUniqueConnection(rcRegion& reg, int n)
	{
		for (int i = 0; i < reg.connections.size(); ++i)
			if (reg.connections[i] == n)
				return;
		reg.connections.push(n);
	}

	bool mergeAndFilterLayerRegions(rcContext* ctx, int minRegionArea,
		unsigned short& maxRegionId,
		rcCompactHeightfield& chf,
		unsigned short* srcReg)
	{
		const int w = chf.width;
		const int h = chf.height;

		const int nreg = maxRegionId + 1;
		rcTempVector<rcRegion> regions;

		// Construct regions
		if (!regions.reserve(nreg)) {
			ctx->log(RC_LOG_ERROR, "mergeAndFilterLayerRegions: Out of memory 'regions' (%d).", nreg);
			return false;
		}
		for (int i = 0; i < nreg; ++i)
			regions.push_back(rcRegion((unsigned short)i));

		// Find region neighbours and overlapping regions.
		rcIntArray lregs(32);
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell& c = chf.cells[x + y * w];

				lregs.resize(0);

				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					const rcCompactSpan& s = chf.spans[i];
					const unsigned short ri = srcReg[i];
					if (ri == 0 || ri >= nreg) continue;
					rcRegion& reg = regions[ri];

					reg.spanCount++;

					reg.ymin = rcMin(reg.ymin, s.y);
					reg.ymax = rcMax(reg.ymax, s.y);

					// Collect all region layers.
					lregs.push(ri);

					// Update neighbours
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int ax = x + rcGetDirOffsetX(dir);
							const int ay = y + rcGetDirOffsetY(dir);
							const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
							const unsigned short rai = srcReg[ai];
							if (rai > 0 && rai < nreg && rai != ri)
								addUniqueConnection(reg, rai);
							if (rai & RC_BORDER_REG)
								reg.connectsToBorder = true;
						}
					}
				}

				// Update overlapping regions.
				for (int i = 0; i < lregs.size() - 1; ++i)
				{
					for (int j = i + 1; j < lregs.size(); ++j)
					{
						if (lregs[i] != lregs[j])
						{
							rcRegion& ri = regions[lregs[i]];
							rcRegion& rj = regions[lregs[j]];
							addUniqueFloorRegion(ri, lregs[j]);
							addUniqueFloorRegion(rj, lregs[i]);
						}
					}
				}
			}
		}

		// Create 2D layers from regions.
		unsigned short layerId = 1;

		for (int i = 0; i < nreg; ++i)
			regions[i].id = 0;

		// Merge montone regions to create non-overlapping areas.
		rcIntArray stack(32);
		for (int i = 1; i < nreg; ++i)
		{
			rcRegion& root = regions[i];
			// Skip already visited.
			if (root.id != 0)
				continue;

			// Start search.
			root.id = layerId;

			stack.resize(0);
			stack.push(i);

			while (stack.size() > 0)
			{
				// Pop front
				rcRegion& reg = regions[stack[0]];
				for (int j = 0; j < stack.size() - 1; ++j)
					stack[j] = stack[j + 1];
				stack.resize(stack.size() - 1);

				const int ncons = (int)reg.connections.size();
				for (int j = 0; j < ncons; ++j)
				{
					const int nei = reg.connections[j];
					rcRegion& regn = regions[nei];
					// Skip already visited.
					if (regn.id != 0)
						continue;
					// Skip if the neighbour is overlapping root region.
					bool overlap = false;
					for (int k = 0; k < root.floors.size(); k++)
					{
						if (root.floors[k] == nei)
						{
							overlap = true;
							break;
						}
					}
					if (overlap)
						continue;

					// Deepen
					stack.push(nei);

					// Mark layer id
					regn.id = layerId;
					// Merge current layers to root.
					for (int k = 0; k < regn.floors.size(); ++k)
						addUniqueFloorRegion(root, regn.floors[k]);
					root.ymin = rcMin(root.ymin, regn.ymin);
					root.ymax = rcMax(root.ymax, regn.ymax);
					root.spanCount += regn.spanCount;
					regn.spanCount = 0;
					root.connectsToBorder = root.connectsToBorder || regn.connectsToBorder;
				}
			}

			layerId++;
		}

		// Remove small regions
		for (int i = 0; i < nreg; ++i)
		{
			if (regions[i].spanCount > 0 && regions[i].spanCount < minRegionArea && !regions[i].connectsToBorder)
			{
				unsigned short reg = regions[i].id;
				for (int j = 0; j < nreg; ++j)
					if (regions[j].id == reg)
						regions[j].id = 0;
			}
		}

		// Compress region Ids.
		for (int i = 0; i < nreg; ++i)
		{
			regions[i].remap = false;
			if (regions[i].id == 0) continue;				// Skip nil regions.
			if (regions[i].id & RC_BORDER_REG) continue;    // Skip external regions.
			regions[i].remap = true;
		}

		unsigned short regIdGen = 0;
		for (int i = 0; i < nreg; ++i)
		{
			if (!regions[i].remap)
				continue;
			unsigned short oldId = regions[i].id;
			unsigned short newId = ++regIdGen;
			for (int j = i; j < nreg; ++j)
			{
				if (regions[j].id == oldId)
				{
					regions[j].id = newId;
					regions[j].remap = false;
				}
			}
		}
		maxRegionId = regIdGen;

		// Remap regions.
		for (int i = 0; i < chf.spanCount; ++i)
		{
			if ((srcReg[i] & RC_BORDER_REG) == 0)
				srcReg[i] = regions[srcReg[i]].id;
		}

		return true;
	}

	void paintRectRegion(int minx, int maxx, int miny, int maxy, unsigned short regId,
		rcCompactHeightfield& chf, unsigned short* srcReg)
	{
		const int w = chf.width;
		for (int y = miny; y < maxy; ++y)
		{
			for (int x = minx; x < maxx; ++x)
			{
				const rcCompactCell& c = chf.cells[x + y * w];
				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					if (chf.areas[i] != RC_NULL_AREA)
						srcReg[i] = regId;
				}
			}
		}
	}

	constexpr unsigned short RC_NULL_NEI = 0xffff;

	struct rcSweepSpan
	{
		unsigned short rid;	// row id
		unsigned short id;	// region id
		unsigned short ns;	// number samples
		unsigned short nei;	// neighbour id
	};
}

// @par
//
// This is usually the second to the last step in creating a fully built
// compact heightfield.  This step is required before regions are built
// using #rcBuildRegions or #rcBuildRegionsMonotone.
// これは通常、完全に構築されたコンパクトな地形を作成する最後のステップの2番目です。
// この手順は、＃rcBuildRegionsまたは#rcBuildRegionsMonotoneを使用して領域を構築する前に必要です。
//
// After this step, the distance data is available via the rcCompactHeightfield::maxDistance
// and rcCompactHeightfield::dist fields.
// この手順の後、距離データはrcCompactHeightfield::maxDistanceおよびrcCompactHeightfield::distフィールドを介して利用できます。
//
// @see rcCompactHeightfield, rcBuildRegions, rcBuildRegionsMonotone
bool rcBuildDistanceField(rcContext* ctx, rcCompactHeightfield& chf)
{
	rcAssert(ctx);

	rcScopedTimer timer{ ctx, RC_TIMER_BUILD_DISTANCEFIELD };

	if (chf.dist)
	{
		rcFree(chf.dist);
		chf.dist = nullptr;
	}

	unsigned short* src
	{ (unsigned short*)rcAlloc(sizeof(unsigned short) * chf.spanCount, RC_ALLOC_TEMP) };

	if (!src)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}

	unsigned short* dst
	{ (unsigned short*)rcAlloc(sizeof(unsigned short) * chf.spanCount, RC_ALLOC_TEMP) };

	if (!dst)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildDistanceField: Out of memory 'dst' (%d).", chf.spanCount);
		rcFree(src);
		return false;
	}

	unsigned short maxDist{ 0 };

	{
		rcScopedTimer timerDist(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST);

		calculateDistanceField(chf, src, maxDist);
		chf.maxDistance = maxDist;
	}

	{
		rcScopedTimer timerBlur(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

		// Blur
		if (boxBlur(chf, 1, src, dst) != src)
			rcSwap(src, dst);

		// Store distance.
		chf.dist = src;
	}

	rcFree(dst);

	return true;
}

// @par
//
// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
// 非ヌル領域は、単一の輪郭を形成する、接続された重複しない歩行可能なスパンで構成されます。
// Contours will form simple polygons.
// 輪郭は単純なポリゴンを形成します。
//
// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
// re-assigned to the zero (null) region.
// 複数の領域が@p minRegionAreaより小さい領域を形成する場合、すべてのスパンはゼロ（ヌル）領域に再割り当てされます。
//
// Partitioning can result in smaller than necessary regions. @p mergeRegionArea helps reduce unecessarily small regions.
// パーティション化により、必要な領域よりも小さくなる場合があります。mergeRegionAreaは、不必要に小さい領域を減らすのに役立ちます。
//
// See the #rcConfig documentation for more information on the configuration parameters.
// 構成パラメータの詳細については、＃rcConfigのドキュメントをご覧ください。
//
// The region data will be available via the rcCompactHeightfield::maxRegions and rcCompactSpan::reg fields.
// 領域データは、rcCompactHeightfield::maxRegionsおよびrcCompactSpan::regフィールドを介して利用できます。
//
// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
// 領域を構築する前に、＃rcBuildDistanceFieldを使用して距離フィールドを作成する必要があります。
//
// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
bool rcBuildRegionsMonotone(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);

	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;

	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short) * chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg, 0, sizeof(unsigned short) * chf.spanCount);

	const int nsweeps = rcMax(chf.width, chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan) * nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegionsMonotone: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}

	// Mark border regions.
	// 境界領域をマークします。
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		// ボーダーがオーバーフローしないことを確認します。
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);

		// Paint regions
		// 領域をペイントします
		paintRectRegion(0, bw, 0, h, id | RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w - bw, w, 0, h, id | RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id | RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h - bh, h, id | RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;

	rcIntArray prev(256);

	// Sweep one line at a time.
	// 一度に1行ずつスイープします。
	for (int y = borderSize; y < h - borderSize; ++y)
	{
		// Collect spans from this row.
		// この行からスパンを収集します。
		prev.resize(id + 1);
		memset(&prev[0], 0, sizeof(int) * id);
		unsigned short rid = 1;

		for (int x = borderSize; x < w - borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;

				// -x
				unsigned short previd = 0;
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, 0);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}

				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}

				// -y
				if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, 3);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short nr = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == nr)
						{
							sweeps[previd].nei = nr;
							sweeps[previd].ns++;
							prev[nr]++;
						}
						else
						{
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}

				srcReg[i] = previd;
			}
		}

		// Create unique ID.
		// 一意のIDを作成します。
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}

		// Remap IDs
		// IDを再マップします
		for (int x = borderSize; x < w - borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}

	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out small regions.
		// 領域をマージし、小さな領域を除外します。
		rcIntArray overlaps;
		chf.maxRegions = id;
		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// Monotone partitioning does not generate overlapping regions.
		// モノトーンパーティショニングは、オーバーラップ領域を生成しません。
	}

	// Store the result out.
	// 結果を保存します。
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];

	return true;
}

// @par
//
// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
// Contours will form simple polygons.
// 非ヌル領域は、単一の輪郭を形成する、接続された重複しない歩行可能なスパンで構成されます。
// 輪郭は単純なポリゴンを形成します。
//
// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
// re-assigned to the zero (null) region.
// 複数の領域が@p minRegionAreaより小さい領域を形成する場合、すべてのスパンはゼロ（ヌル）領域に再割り当てされます。
//
// Watershed partitioning can result in smaller than necessary regions, especially in diagonal corridors.
// @p mergeRegionArea helps reduce unecessarily small regions.
// 流域分割により、特に斜めの廊下では、必要な領域よりも小さくなります。
// mergeRegionAreaは、不必要に小さい領域を減らすのに役立ちます。
//
// See the #rcConfig documentation for more information on the configuration parameters.
// 構成パラメータの詳細については、＃rcConfigのドキュメントをご覧ください。
//
// The region data will be available via the rcCompactHeightfield::maxRegions
// and rcCompactSpan::reg fields.
// 領域データは、rcCompactHeightfield::maxRegionsおよびrcCompactSpan::regフィールドを介して利用できます。
//
// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
// 領域を構築する前に、＃rcBuildDistanceFieldを使用して距離フィールドを作成する必要があります。
//
// @see rcCompactHeightfield, rcCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, rcConfig
bool rcBuildRegions(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);

	const int w = chf.width;
	const int h = chf.height;

	rcScopedDelete<unsigned short> buf((unsigned short*)rcAlloc(sizeof(unsigned short) * chf.spanCount * 2, RC_ALLOC_TEMP));
	if (!buf)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildRegions: Out of memory 'tmp' (%d).", chf.spanCount * 4);
		return false;
	}

	ctx->startTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

	const int LOG_NB_STACKS = 3;
	const int NB_STACKS = 1 << LOG_NB_STACKS;
	rcTempVector<LevelStackEntry> lvlStacks[NB_STACKS];

	for (int i = 0; i < NB_STACKS; ++i)
		lvlStacks[i].reserve(256);

	rcTempVector<LevelStackEntry> stack;
	stack.reserve(256);

	unsigned short* srcReg = buf;
	unsigned short* srcDist = buf + chf.spanCount;

	memset(srcReg, 0, sizeof(unsigned short) * chf.spanCount);
	memset(srcDist, 0, sizeof(unsigned short) * chf.spanCount);

	unsigned short regionId = 1;
	unsigned short level = (chf.maxDistance + 1) & ~1;

	// TODO: Figure better formula, expandIters defines how much the watershed "overflows" and simplifies the regions. Tying it to agent radius was usually good indication how greedy it could be.
	// TODO：式の改善、expendIturesは、分水界が「オーバーフロー」する量を定義し、地域を単純化します。それをエージェントの半径に結び付けることは、通常、それがどれほど貪欲であるかを示す良い指標でした。
	// constexpr int expandIters = 4 + walkableRadius * 2;
	constexpr int expandIters = 8;

	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		// ボーダーがオーバーフローしないことを確認してください。
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);

		// Paint regions
		// 領域をペイントします
		paintRectRegion(0, bw, 0, h, regionId | RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(w - bw, w, 0, h, regionId | RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, 0, bh, regionId | RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, h - bh, h, regionId | RC_BORDER_REG, chf, srcReg); regionId++;
	}

	chf.borderSize = borderSize;

	int sId = -1;

	while (level > 0)
	{
		level = level >= 2 ? level - 2 : 0;
		sId = (sId + 1) & (NB_STACKS - 1);

		//		ctx->startTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		if (sId == 0)
			sortCellsByLevel(level, chf, srcReg, NB_STACKS, lvlStacks, 1);
		else
			appendStacks(lvlStacks[sId - 1], lvlStacks[sId], srcReg); // copy left overs from last level. 最後のレベルから残りをコピーする

//		ctx->stopTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		{
			rcScopedTimer timerExpand(ctx, RC_TIMER_BUILD_REGIONS_EXPAND);

			// Expand current regions until no empty connected cells found.
			// 空の接続されたセルが見つからなくなるまで現在の領域を展開します。
			expandRegions(expandIters, level, chf, srcReg, srcDist, lvlStacks[sId], false);
		}

		{
			rcScopedTimer timerFloor(ctx, RC_TIMER_BUILD_REGIONS_FLOOD);

			// Mark new regions with IDs.
			// 新しい領域にIDを付けます。
			for (int j = 0; j < lvlStacks[sId].size(); j++)
			{
				LevelStackEntry current = lvlStacks[sId][j];
				int x = current.x;
				int y = current.y;
				int i = current.index;
				if (i >= 0 && srcReg[i] == 0)
				{
					if (floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, stack))
					{
						if (regionId == 0xFFFF)
						{
							ctx->log(RC_LOG_ERROR, "rcBuildRegions: Region ID overflow");
							return false;
						}

						regionId++;
					}
				}
			}
		}
	}

	// Expand current regions until no empty connected cells found.
	// 空の接続されたセルが見つからなくなるまで現在の領域を展開します。
	expandRegions(expandIters * 8, 0, chf, srcReg, srcDist, stack, true);

	ctx->stopTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out smalle regions.
		// 領域をマージし、小さな領域を除外します。
		rcIntArray overlaps;
		chf.maxRegions = regionId;

		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// If overlapping regions were found during merging, split those regions.
		// マージ中に重複する領域が見つかった場合、それらの領域を分割します。
		if (overlaps.size() > 0)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildRegions: %d overlapping regions.", overlaps.size());
		}
	}

	// Write the result out.
	// 結果を書き出します。
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];

	return true;
}

bool rcBuildLayerRegions(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);

	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;

	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short) * chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg, 0, sizeof(unsigned short) * chf.spanCount);

	const int nsweeps = rcMax(chf.width, chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan) * nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildLayerRegions: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}

	// Mark border regions.
	// 境界領域をマークします。
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		// ボーダーがオーバーフローしないことを確認します。
		const int bw = rcMin(w, borderSize);
		const int bh = rcMin(h, borderSize);

		// Paint regions
		// 領域をペイントします
		paintRectRegion(0, bw, 0, h, id | RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w - bw, w, 0, h, id | RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id | RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h - bh, h, id | RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;

	rcIntArray prev(256);

	// Sweep one line at a time.
	// 一度に1行ずつスイープします。
	for (int y = borderSize; y < h - borderSize; ++y)
	{
		// Collect spans from this row.
		//この行からスパンを収集します。
		prev.resize(id + 1);
		memset(&prev[0], 0, sizeof(int) * id);
		unsigned short rid = 1;

		for (int x = borderSize; x < w - borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;

				// -x
				unsigned short previd = 0;
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, 0);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}

				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}

				// -y
				if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, 3);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short nr = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == nr)
						{
							sweeps[previd].nei = nr;
							sweeps[previd].ns++;
							prev[nr]++;
						}
						else
						{
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}

				srcReg[i] = previd;
			}
		}

		// Create unique ID.
		// 一意のIDを作成します。
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}

		// Remap IDs
		// IDを再マップします
		for (int x = borderSize; x < w - borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}

	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge monotone regions to layers and remove small regions.
		// モノトーン領域をレイヤーにマージし、小さな領域を削除します。
		chf.maxRegions = id;
		if (!mergeAndFilterLayerRegions(ctx, minRegionArea, chf.maxRegions, chf, srcReg))
			return false;
	}

	// Store the result out.
	// 結果を保存します。
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];

	return true;
}
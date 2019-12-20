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

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAssert.h"

// @par
//
// Allows the formation of walkable regions that will flow over low lying
// objects such as curbs, and up structures such as stairways.
// 縁石などの低層オブジェクトや階段などの構造物の上を流れる歩行可能領域の形成を許可します。
//
// Two neighboring spans are walkable if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) < waklableClimb</tt>
// 次の場合、2つの隣接スパンは歩行可能です：<tt> rcAbs（currentSpan.smax-neighborSpan.smax）<waklableClimb </ tt>
//
// @warning Will override the effect of #rcFilterLedgeSpans.  So if both filters are used, call
// #rcFilterLedgeSpans after calling this filter.
// #rcFilterLedgeSpansの効果をオーバーライドします。
// したがって、両方のフィルターを使用する場合は、このフィルターを呼び出した後に#rcFilterLedgeSpansを呼び出してください。
//
// @see rcHeightfield, rcConfig
void rcFilterLowHangingWalkableObstacles(rcContext* ctx, const int walkableClimb, rcHeightfield& solid)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_FILTER_LOW_OBSTACLES);

	const int w = solid.width;
	const int h = solid.height;

	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			rcSpan* ps = 0;
			bool previousWalkable = false;
			unsigned char previousArea = RC_NULL_AREA;

			for (rcSpan* s = solid.spans[x + y * w]; s; ps = s, s = s->next)
			{
				const bool walkable = s->area != RC_NULL_AREA;

				// If current span is not walkable, but there is walkable
				// span just below it, mark the span above it walkable too.
				// 現在のスパンが歩行可能ではないが、そのすぐ下に歩行可能なスパンがある場合、
				// その上のスパンも歩行可能にマークします。
				if (!walkable && previousWalkable)
				{
					if (rcAbs((int)s->smax - (int)ps->smax) <= walkableClimb)
						s->area = previousArea;
				}

				// Copy walkable flag so that it cannot propagate
				// past multiple non-walkable objects.
				// walkable flagをコピーして、複数の歩くことができないオブジェクトを超えて伝播できないようにします。
				previousWalkable = walkable;
				previousArea = s->area;
			}
		}
	}
}

// @par
//
// A ledge is a span with one or more neighbors whose maximum is further away than @p walkableClimb
// from the current span's maximum.
// 出張りとは、最大値が現在のスパンの最大値からwalkableClimbよりも離れている1つ以上の隣接するスパンです。
// This method removes the impact of the overestimation of conservative voxelization
// so the resulting mesh will not have regions hanging in the air over ledges.
// このメソッドは、保守的なボクセル化の過大評価の影響を除去するため、
// 結果のメッシュには、出張りの上に空中にぶら下がった領域がありません。
//
// A span is a ledge if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) > walkableClimb</tt>
// 次の場合、スパンは出張りです：<tt> rcAbs（currentSpan.smax-neighborSpan.smax）> walkableClimb </ tt>
//
// @see rcHeightfield, rcConfig
void rcFilterLedgeSpans(rcContext* ctx, const int walkableHeight, const int walkableClimb,
	rcHeightfield& solid)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_FILTER_BORDER);

	const int w = solid.width;
	const int h = solid.height;
	const int MAX_HEIGHT = 0xffff;

	// Mark border spans.
	// ボーダースパンをマークします。
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			for (rcSpan* s = solid.spans[x + y * w]; s; s = s->next)
			{
				// Skip non walkable spans.
				// 歩行不可能なスパンをスキップします。
				if (s->area == RC_NULL_AREA)
					continue;

				const int bot = (int)(s->smax);
				const int top = s->next ? (int)(s->next->smin) : MAX_HEIGHT;

				// Find neighbours minimum height.
				// 近隣の最小の高さを見つけます。
				int minh = MAX_HEIGHT;

				// Min and max height of accessible neighbours.
				// アクセス可能な近隣の最小および最大の高さ。
				int asmin = s->smax;
				int asmax = s->smax;

				for (int dir = 0; dir < 4; ++dir)
				{
					int dx = x + rcGetDirOffsetX(dir);
					int dy = y + rcGetDirOffsetY(dir);

					// Skip neighbours which are out of bounds.
					// 範囲外の近隣をスキップします。
					if (dx < 0 || dy < 0 || dx >= w || dy >= h)
					{
						minh = rcMin(minh, -walkableClimb - bot);
						continue;
					}

					// From minus infinity to the first span.
					// 負の無限大から最初のスパンまで。
					rcSpan* ns = solid.spans[dx + dy * w];
					int nbot = -walkableClimb;
					int ntop = ns ? (int)ns->smin : MAX_HEIGHT;

					// Skip neighbour if the gap between the spans is too small.
					// スパン間のギャップが小さすぎる場合、隣接をスキップします。
					if (rcMin(top, ntop) - rcMax(bot, nbot) > walkableHeight)
						minh = rcMin(minh, nbot - bot);

					// Rest of the spans.
					// スパンの残り。
					for (ns = solid.spans[dx + dy * w]; ns; ns = ns->next)
					{
						nbot = (int)ns->smax;
						ntop = ns->next ? (int)ns->next->smin : MAX_HEIGHT;

						// Skip neighbour if the gap between the spans is too small.
						// スパン間のギャップが小さすぎる場合、隣接をスキップします。
						if (rcMin(top, ntop) - rcMax(bot, nbot) > walkableHeight)
						{
							minh = rcMin(minh, nbot - bot);

							// Find min/max accessible neighbour height.
							// アクセス可能な最小/最大隣接高さを見つけます。
							if (rcAbs(nbot - bot) <= walkableClimb)
							{
								if (nbot < asmin) asmin = nbot;
								if (nbot > asmax) asmax = nbot;
							}
						}
					}
				}

				// The current span is close to a ledge if the drop to any
				// neighbour span is less than the walkableClimb.
				// 隣接するスパンへのドロップが「walkableClimb」より小さい場合、現在のスパンは出張りに近い。
				if (minh < -walkableClimb)
				{
					s->area = RC_NULL_AREA;
				}

				// If the difference between all neighbours is too large,
				// we are at steep slope, mark the span as ledge.
				// すべての隣同士の差が大きすぎる場合、急勾配になり、スパンをレッジとしてマークします。
				else if ((asmax - asmin) > walkableClimb)
				{
					s->area = RC_NULL_AREA;
				}
			}
		}
	}
}

// @par
//
// For this filter, the clearance above the span is the distance from the span's
// maximum to the next higher span's minimum. (Same grid column.)
// このフィルターでは、スパンの上のクリアランスは、
// スパンの最大値から次の高いスパンの最小値までの距離です。 （同じグリッド列。）
//
// @see rcHeightfield, rcConfig
void rcFilterWalkableLowHeightSpans(rcContext* ctx, int walkableHeight, rcHeightfield& solid)
{
	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_FILTER_WALKABLE);

	const int w = solid.width;
	const int h = solid.height;
	const int MAX_HEIGHT = 0xffff;

	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	// エージェントがそこに立つのに十分なスペースがないスパンから歩行可能フラグを削除します。
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			for (rcSpan* s = solid.spans[x + y * w]; s; s = s->next)
			{
				const int bot = (int)(s->smax);
				const int top = s->next ? (int)(s->next->smin) : MAX_HEIGHT;
				if ((top - bot) <= walkableHeight)
					s->area = RC_NULL_AREA;
			}
		}
	}
}
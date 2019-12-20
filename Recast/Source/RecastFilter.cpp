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
// ���΂Ȃǂ̒�w�I�u�W�F�N�g��K�i�Ȃǂ̍\�����̏�𗬂����s�\�̈�̌`���������܂��B
//
// Two neighboring spans are walkable if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) < waklableClimb</tt>
// ���̏ꍇ�A2�̗אڃX�p���͕��s�\�ł��F<tt> rcAbs�icurrentSpan.smax-neighborSpan.smax�j<waklableClimb </ tt>
//
// @warning Will override the effect of #rcFilterLedgeSpans.  So if both filters are used, call
// #rcFilterLedgeSpans after calling this filter.
// #rcFilterLedgeSpans�̌��ʂ��I�[�o�[���C�h���܂��B
// ���������āA�����̃t�B���^�[���g�p����ꍇ�́A���̃t�B���^�[���Ăяo�������#rcFilterLedgeSpans���Ăяo���Ă��������B
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
				// ���݂̃X�p�������s�\�ł͂Ȃ����A���̂������ɕ��s�\�ȃX�p��������ꍇ�A
				// ���̏�̃X�p�������s�\�Ƀ}�[�N���܂��B
				if (!walkable && previousWalkable)
				{
					if (rcAbs((int)s->smax - (int)ps->smax) <= walkableClimb)
						s->area = previousArea;
				}

				// Copy walkable flag so that it cannot propagate
				// past multiple non-walkable objects.
				// walkable flag���R�s�[���āA�����̕������Ƃ��ł��Ȃ��I�u�W�F�N�g�𒴂��ē`�d�ł��Ȃ��悤�ɂ��܂��B
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
// �o����Ƃ́A�ő�l�����݂̃X�p���̍ő�l����walkableClimb��������Ă���1�ȏ�̗אڂ���X�p���ł��B
// This method removes the impact of the overestimation of conservative voxelization
// so the resulting mesh will not have regions hanging in the air over ledges.
// ���̃��\�b�h�́A�ێ�I�ȃ{�N�Z�����̉ߑ�]���̉e�����������邽�߁A
// ���ʂ̃��b�V���ɂ́A�o����̏�ɋ󒆂ɂԂ牺�������̈悪����܂���B
//
// A span is a ledge if: <tt>rcAbs(currentSpan.smax - neighborSpan.smax) > walkableClimb</tt>
// ���̏ꍇ�A�X�p���͏o����ł��F<tt> rcAbs�icurrentSpan.smax-neighborSpan.smax�j> walkableClimb </ tt>
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
	// �{�[�_�[�X�p�����}�[�N���܂��B
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			for (rcSpan* s = solid.spans[x + y * w]; s; s = s->next)
			{
				// Skip non walkable spans.
				// ���s�s�\�ȃX�p�����X�L�b�v���܂��B
				if (s->area == RC_NULL_AREA)
					continue;

				const int bot = (int)(s->smax);
				const int top = s->next ? (int)(s->next->smin) : MAX_HEIGHT;

				// Find neighbours minimum height.
				// �ߗׂ̍ŏ��̍����������܂��B
				int minh = MAX_HEIGHT;

				// Min and max height of accessible neighbours.
				// �A�N�Z�X�\�ȋߗׂ̍ŏ�����эő�̍����B
				int asmin = s->smax;
				int asmax = s->smax;

				for (int dir = 0; dir < 4; ++dir)
				{
					int dx = x + rcGetDirOffsetX(dir);
					int dy = y + rcGetDirOffsetY(dir);

					// Skip neighbours which are out of bounds.
					// �͈͊O�̋ߗׂ��X�L�b�v���܂��B
					if (dx < 0 || dy < 0 || dx >= w || dy >= h)
					{
						minh = rcMin(minh, -walkableClimb - bot);
						continue;
					}

					// From minus infinity to the first span.
					// ���̖����傩��ŏ��̃X�p���܂ŁB
					rcSpan* ns = solid.spans[dx + dy * w];
					int nbot = -walkableClimb;
					int ntop = ns ? (int)ns->smin : MAX_HEIGHT;

					// Skip neighbour if the gap between the spans is too small.
					// �X�p���Ԃ̃M���b�v������������ꍇ�A�אڂ��X�L�b�v���܂��B
					if (rcMin(top, ntop) - rcMax(bot, nbot) > walkableHeight)
						minh = rcMin(minh, nbot - bot);

					// Rest of the spans.
					// �X�p���̎c��B
					for (ns = solid.spans[dx + dy * w]; ns; ns = ns->next)
					{
						nbot = (int)ns->smax;
						ntop = ns->next ? (int)ns->next->smin : MAX_HEIGHT;

						// Skip neighbour if the gap between the spans is too small.
						// �X�p���Ԃ̃M���b�v������������ꍇ�A�אڂ��X�L�b�v���܂��B
						if (rcMin(top, ntop) - rcMax(bot, nbot) > walkableHeight)
						{
							minh = rcMin(minh, nbot - bot);

							// Find min/max accessible neighbour height.
							// �A�N�Z�X�\�ȍŏ�/�ő�אڍ����������܂��B
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
				// �אڂ���X�p���ւ̃h���b�v���uwalkableClimb�v��菬�����ꍇ�A���݂̃X�p���͏o����ɋ߂��B
				if (minh < -walkableClimb)
				{
					s->area = RC_NULL_AREA;
				}

				// If the difference between all neighbours is too large,
				// we are at steep slope, mark the span as ledge.
				// ���ׂĂ̗ד��m�̍����傫������ꍇ�A�}���z�ɂȂ�A�X�p�������b�W�Ƃ��ă}�[�N���܂��B
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
// ���̃t�B���^�[�ł́A�X�p���̏�̃N���A�����X�́A
// �X�p���̍ő�l���玟�̍����X�p���̍ŏ��l�܂ł̋����ł��B �i�����O���b�h��B�j
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
	// �G�[�W�F���g�������ɗ��̂ɏ\���ȃX�y�[�X���Ȃ��X�p��������s�\�t���O���폜���܂��B
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
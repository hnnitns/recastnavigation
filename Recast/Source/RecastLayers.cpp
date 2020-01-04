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
	// Must be 255 or smaller (not 256) because layer IDs are stored as a byte where 255 is a special value.
	// ���C���[ID��255�����ʂȒl�ł���o�C�g�Ƃ��ĕۑ�����邽�߁A255�ȉ��i256�ł͂Ȃ��j�łȂ���΂Ȃ�܂���B
	constexpr int RC_MAX_LAYERS = 63;
	constexpr int RC_MAX_NEIS = 16;

	struct rcLayerRegion
	{
		unsigned char layers[RC_MAX_LAYERS];
		unsigned char neis[RC_MAX_NEIS];
		unsigned short ymin, ymax;
		unsigned char layerId;	// Layer ID // ���C���[ID
		unsigned char nlayers;	// Layer count // ���C���[��
		unsigned char nneis;	// Neighbour count // �אڃJ�E���g
		// Flag indicating if the region is the base of merged regions.
		// ���[�W�������}�[�W���ꂽ���[�W�����̃x�[�X�ł��邩�ǂ����������t���O�B
		unsigned char base;
	};

	inline bool contains(const unsigned char* a, const unsigned char an, const unsigned char v)
	{
		const int n = (int)an;

		for (int i = 0; i < n; ++i)
		{
			if (a[i] == v) return true;
		}

		return false;
	}

	inline bool addUnique(unsigned char* a, unsigned char& an, int anMax, unsigned char v)
	{
		if (contains(a, an, v)) return true;

		if ((int)an >= anMax) return false;

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
		unsigned short ns;	// number samples // �T���v����
		unsigned char id;	// region id      // �̈�ID
		unsigned char nei;	// neighbour id   // ��ID
	};
}

/// @par
///
/// See the #rcConfig documentation for more information on the configuration parameters.
/// �\���p�����[�^�[�̏ڍׂɂ��ẮA��rcConfig�̃h�L�������g���Q�Ƃ��Ă��������B
///
/// @see rcAllocHeightfieldLayerSet, rcCompactHeightfield, rcHeightfieldLayerSet, rcConfig
bool rcBuildHeightfieldLayers(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int walkableHeight,
	rcHeightfieldLayerSet& lset)
{
	constexpr int UCharMax{ (std::numeric_limits<uint8_t>::max)() };

	rcAssert(ctx);

	rcScopedTimer timer(ctx, RC_TIMER_BUILD_LAYERS);

	const int w = chf.width;
	const int h = chf.height;

	rcScopedDelete<unsigned char> srcReg((unsigned char*)rcAlloc(sizeof(unsigned char) * chf.spanCount, RC_ALLOC_TEMP));

	if (!srcReg)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'srcReg' (%d).", chf.spanCount); // �������[�s���usrcReg�v
		return false;
	}

	memset(srcReg, UCharMax, sizeof(unsigned char) * chf.spanCount);

	const int nsweeps = chf.width;
	rcScopedDelete<rcLayerSweepSpan> sweeps((rcLayerSweepSpan*)rcAlloc(sizeof(rcLayerSweepSpan) * nsweeps, RC_ALLOC_TEMP));

	if (!sweeps)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'sweeps' (%d).", nsweeps); // �������[�s���usweeps�v
		return false;
	}

	// Partition walkable area into monotone regions.
	// ���s�\�̈�����m�g�[���̈�ɕ������܂��B
	int prevCount[256]{};
	unsigned char regId{};

	for (int y = borderSize; y < h - borderSize; ++y)
	{
		memset(prevCount, 0, sizeof(int) * regId);
		unsigned char sweepId{};

		for (int x = borderSize; x < w - borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];

				if (chf.areas[i] == RC_NULL_AREA) continue;

				unsigned char sid = UCharMax;

				// -x
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, 0);

					if (chf.areas[ai] != RC_NULL_AREA && srcReg[ai] != UCharMax)
						sid = srcReg[ai];
				}

				if (sid == UCharMax)
				{
					sid = sweepId++;
					sweeps[sid].nei = UCharMax;
					sweeps[sid].ns = 0;
				}

				// -y
				if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, 3);
					const unsigned char nr = srcReg[ai];

					if (nr != UCharMax)
					{
						// Set neighbour when first valid neighbour is encoutered.
						// �ŏ��̗L���ȋߗׂ����o���ꂽ�Ƃ��ɗׂ�ݒ肵�܂��B
						if (sweeps[sid].ns == 0)
							sweeps[sid].nei = nr;

						if (sweeps[sid].nei == nr)
						{
							// Update existing neighbour
							// �����ׂ̗��X�V���܂�
							sweeps[sid].ns++;
							prevCount[nr]++;
						}
						else
						{
							// This is hit if there is nore than one neighbour.
							// ����́A�����ׂ̗�����ꍇ�Ƀq�b�g���܂��B
							// Invalidate the neighbour.
							// �ׂ𖳌��ɂ��܂��B
							sweeps[sid].nei = UCharMax;
						}
					}
				}

				srcReg[i] = sid;
			}
		}

		// Create unique ID.
		// ��ӂ�ID���쐬���܂��B
		for (int i = 0; i < sweepId; ++i)
		{
			// If the neighbour is set and there is only one continuous connection to it,
			// the sweep will be merged with the previous one, else new region is created.
			// �ׂ��ݒ肳��Ă���A�ׂւ̘A���ڑ���1�����Ȃ��ꍇ�A
			// �X�C�[�v�͑O�̃X�C�[�v�ƃ}�[�W����A�����łȂ��ꍇ�͐V�����̈悪�쐬����܂��B
			if (sweeps[i].nei != UCharMax && prevCount[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				if (regId == UCharMax)
				{
					ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Region ID overflow."); // �̈�ID�̃I�[�o�[�t���[�B
					return false;
				}

				sweeps[i].id = regId++;
			}
		}

		// Remap local sweep ids to region ids.
		// ���[�J���X�C�[�vID�����[�W����ID�ɍă}�b�s���O���܂��B
		for (int x = borderSize; x < w - borderSize; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				if (srcReg[i] != UCharMax)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}

	// Allocate and init layer regions.
	// ���C���[�̈�����蓖�Ăď��������܂��B
	const int nregs = (int)regId;
	rcScopedDelete<rcLayerRegion> regs((rcLayerRegion*)rcAlloc(sizeof(rcLayerRegion) * nregs, RC_ALLOC_TEMP));

	if (!regs)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'regs' (%d).", nregs); // �������[�s���uregs�v

		return false;
	}

	memset(regs, 0, sizeof(rcLayerRegion) * nregs);

	for (int i = 0; i < nregs; ++i)
	{
		regs[i].layerId = UCharMax;
		regs[i].ymin = (std::numeric_limits<uint16_t>::max)();
		regs[i].ymax = 0;
	}

	// Find region neighbours and overlapping regions.
	// �אڂ���n��Əd������n����������܂��B
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];

			unsigned char lregs[RC_MAX_LAYERS];
			int nlregs{};

			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = chf.spans[i];
				const unsigned char ri = srcReg[i];

				if (ri == UCharMax) continue;

				regs[ri].ymin = rcMin(regs[ri].ymin, s.y);
				regs[ri].ymax = rcMax(regs[ri].ymax, s.y);

				// Collect all region layers.
				// ���ׂẴ��[�W�������C���[�����W���܂��B
				if (nlregs < RC_MAX_LAYERS)
					lregs[nlregs++] = ri;

				// Update neighbours
				// �ׂ��X�V���܂�
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
						const unsigned char rai = srcReg[ai];

						if (rai != UCharMax && rai != ri)
						{
							// Don't check return value -- if we cannot add the neighbor
							// it will just cause a few more regions to be created, which is fine.
							// �߂�l���`�F�b�N���Ȃ�-�אl��ǉ��ł��Ȃ��ꍇ�A
							// ����ɂ������̗̈悪�쐬����܂����A����͖�肠��܂���B
							addUnique(regs[ri].neis, regs[ri].nneis, RC_MAX_NEIS, rai);
						}
					}
				}
			}

			// Update overlapping regions.
			// �d������̈���X�V���܂��B
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
							// ���C���[�I�[�o�[�t���[�i�d��������s�\�ȃv���b�g�t�H�[������������)�BRC_MAX_LAYERS�𑝂₵�Ă݂Ă��������B
							ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: "
								"layer overflow (too many overlapping walkable platforms). Try increasing RC_MAX_LAYERS.");
							return false;
						}
					}
				}
			}
		}
	}

	// Create 2D layers from regions.
	// �̈悩��2D���C���[���쐬���܂��B
	unsigned char layerId{};

	static const int MAX_STACK = 64;
	unsigned char stack[MAX_STACK];
	int nstack{};

	for (int i = 0; i < nregs; ++i)
	{
		rcLayerRegion& root = regs[i];

		// Skip already visited.
		// ���ɃA�N�Z�X�������̂��X�L�b�v���܂��B
		if (root.layerId != UCharMax) continue;

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
				// ���ɃA�N�Z�X�������̂��X�L�b�v���܂��B
				if (regn.layerId != UCharMax) continue;

				// Skip if the neighbour is overlapping root region.
				// �ׂ����[�g�̈�Əd�����Ă���ꍇ�̓X�L�b�v���܂��B
				if (contains(root.layers, root.nlayers, nei)) continue;

				// Skip if the height range would become too large.
				// �����͈̔͂��傫���Ȃ肷����ꍇ�̓X�L�b�v���܂��B
				const int ymin = rcMin(root.ymin, regn.ymin);
				const int ymax = rcMax(root.ymax, regn.ymax);

				if ((ymax - ymin) >= 255) continue;

				if (nstack < MAX_STACK)
				{
					// Deepen // �[�߂�
					stack[nstack++] = (unsigned char)nei;

					// Mark layer id
					// ���C���[ID���}�[�N���܂�
					regn.layerId = layerId;

					// Merge current layers to root.
					// ���݂̃��C���[�����[�g�Ƀ}�[�W���܂��B
					for (int k = 0; k < regn.nlayers; ++k)
					{
						if (!addUnique(root.layers, root.nlayers, RC_MAX_LAYERS, regn.layers[k]))
						{
							// ���C���[�I�[�o�[�t���[�i�d��������s�\�ȃv���b�g�t�H�[������������)�BRC_MAX_LAYERS�𑝂₵�Ă݂Ă��������B
							ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers:"
								"layer overflow (too many overlapping walkable platforms). Try increasing RC_MAX_LAYERS.");
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
	// �������߂��d�����Ȃ��̈���}�[�W���܂��B
	const unsigned short mergeHeight = (unsigned short)walkableHeight * 4;

	for (int i = 0; i < nregs; ++i)
	{
		rcLayerRegion& ri = regs[i];

		if (!ri.base) continue;

		unsigned char newId = ri.layerId;

		while (true)
		{
			unsigned char oldId = UCharMax;

			for (int j = 0; j < nregs; ++j)
			{
				if (i == j) continue;

				rcLayerRegion& rj = regs[j];

				if (!rj.base) continue;

				// Skip if the regions are not close to each other.
				// ���[�W�������݂��ɋߐڂ��Ă��Ȃ��ꍇ�̓X�L�b�v���܂��B
				if (!overlapRange(ri.ymin, ri.ymax + mergeHeight, rj.ymin, rj.ymax + mergeHeight))
					continue;

				// Skip if the height range would become too large.
				// �����͈̔͂��傫���Ȃ肷����ꍇ�̓X�L�b�v���܂��B
				const int ymin = rcMin(ri.ymin, rj.ymin);
				const int ymax = rcMax(ri.ymax, rj.ymax);

				if ((ymax - ymin) >= 255) continue;

				// Make sure that there is no overlap when merging 'ri' and 'rj'.
				// 'ri'�� 'rj'���}�[�W����Ƃ��ɏd�����Ȃ����Ƃ��m�F���܂��B
				bool overlap = false;

				// Iterate over all regions which have the same layerId as 'rj'
				// 'rj'�Ɠ������C���[�������ׂĂ̗̈�𔽕��������܂�
				for (int k = 0; k < nregs; ++k)
				{
					if (regs[k].layerId != rj.layerId) continue;

					// Check if region 'k' is overlapping region 'ri'
					// Index to 'regs' is the same as region id.
					// �̈� 'k'���̈� 'ri'�ɏd�����Ă��邩�ǂ������m�F���܂��B
					// 'regs '�ւ̃C���f�b�N�X�́A�̈�ID�Ɠ����ł��B
					if (contains(ri.layers, ri.nlayers, (unsigned char)k))
					{
						overlap = true;
						break;
					}
				}

				// Cannot merge of regions overlap.
				// ���[�W�����̃I�[�o�[���b�v���}�[�W�ł��܂���B
				if (overlap) continue;

				// Can merge i and j.
				// i��j���}�[�W�ł��܂��B
				oldId = rj.layerId;
				break;
			}

			// Could not find anything to merge with, stop.
			//�}�[�W������̂�������܂���ł����B��~���܂��B
			if (oldId == UCharMax) break;

			// Merge
			for (int j = 0; j < nregs; ++j)
			{
				rcLayerRegion& rj = regs[j];
				if (rj.layerId == oldId)
				{
					rj.base = 0;
					// Remap layerIds. // ���C��ID���ă}�b�v���܂��B
					rj.layerId = newId;

					// Add overlaid layers from 'rj' to 'ri'.
					// �I�[�o�[���C���C���[���urj�v����uri�v�ɒǉ����܂��B
					for (int k = 0; k < rj.nlayers; ++k)
					{
						if (!addUnique(ri.layers, ri.nlayers, RC_MAX_LAYERS, rj.layers[k]))
						{
							// ���C���[�I�[�o�[�t���[�i�d��������s�\�ȃv���b�g�t�H�[������������)�BRC_MAX_LAYERS�𑝂₵�Ă݂Ă��������B
							ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers:"
								"layer overflow (too many overlapping walkable platforms). Try increasing RC_MAX_LAYERS.");
							return false;
						}
					}

					// Update height bounds.
					// �����̋��E���X�V���܂��B
					ri.ymin = rcMin(ri.ymin, rj.ymin);
					ri.ymax = rcMax(ri.ymax, rj.ymax);
				}
			}
		}
	}

	// Compact layer Ids
	// �R���p�N�g���C���[ID
	unsigned char remap[256]{};
	memset(remap, 0, 256);

	// Find number of unique layers.
	// ��ӂ̃��C���[�̐��������܂��B
	layerId = 0;

	for (int i = 0; i < nregs; ++i)
		remap[regs[i].layerId] = 1;

	for (int i = 0; i < 256; ++i)
	{
		if (remap[i])
			remap[i] = layerId++;
		else
			remap[i] = UCharMax;
	}

	// Remap ids.
	// ID���ă}�b�v���܂��B
	for (int i = 0; i < nregs; ++i)
		regs[i].layerId = remap[regs[i].layerId];

	// No layers, return empty.
	//���C���[�Ȃ��A���Ԃ��܂��B
	if (layerId == 0)
		return true;

	// Create layers.
	rcAssert(lset.layers == 0);

	const int lw = w - borderSize * 2;
	const int lh = h - borderSize * 2;

	// Build contracted bbox for layers.
	// ���C���[�̃R���g���N�gbbox���\�z���܂��B
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
		ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'layers' (%d).", lset.nlayers); // �������[�s���ulayers�v
		return false;
	}

	memset(lset.layers, 0, sizeof(rcHeightfieldLayer) * lset.nlayers);

	// Store layers.
	//���C���[��ۑ����܂��B
	for (int i = 0; i < lset.nlayers; ++i)
	{
		unsigned char curId = (unsigned char)i;

		rcHeightfieldLayer* layer = &lset.layers[i];

		const int gridSize = sizeof(unsigned char) * lw * lh;

		layer->heights = (unsigned char*)rcAlloc(gridSize, RC_ALLOC_PERM);

		if (!layer->heights)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'heights' (%d).", gridSize); // �������[�s���uheights�v
			return false;
		}

		memset(layer->heights, UCharMax, gridSize);

		layer->areas = (unsigned char*)rcAlloc(gridSize, RC_ALLOC_PERM);

		if (!layer->areas)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'areas' (%d).", gridSize); // �������[�s���uareas�v
			return false;
		}

		memset(layer->areas, 0, gridSize);

		layer->cons = (unsigned char*)rcAlloc(gridSize, RC_ALLOC_PERM);

		if (!layer->cons)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildHeightfieldLayers: Out of memory 'cons' (%d).", gridSize); // �������[�s���ucons�v
			return false;
		}

		memset(layer->cons, 0, gridSize);

		// Find layer height bounds.
		// ���C���[�̍����̋��E�������܂��B
		int hmin{}, hmax{};
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
		// bbox�𒲐����āA�n�`�ɍ��킹�܂��B
		rcVcopy(layer->bmin, bmin);
		rcVcopy(layer->bmax, bmax);
		layer->bmin[1] = bmin[1] + hmin * chf.ch;
		layer->bmax[1] = bmin[1] + hmax * chf.ch;
		layer->hmin = hmin;
		layer->hmax = hmax;

		// Update usable data region.
		// �g�p�\�ȃf�[�^�̈���X�V���܂��B
		layer->minx = layer->width;
		layer->maxx = 0;
		layer->miny = layer->height;
		layer->maxy = 0;

		// Copy height and area from compact heightfield.
		// �R���p�N�g�ȍ����t�B�[���h���獂���Ɩʐς��R�s�[���܂��B
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
					// �����蓖�Ă̗̈���X�L�b�v���܂��B
					if (srcReg[j] == UCharMax) continue;

					// Skip of does nto belong to current layer.
					// ���݂̃��C���[�ɑ����Ȃ��ꍇ�̓X�L�b�v���܂��B
					unsigned char lid = regs[srcReg[j]].layerId;

					if (lid != curId) continue;

					// Update data bounds.
					// �f�[�^���E���X�V���܂��B
					layer->minx = rcMin(layer->minx, x);
					layer->maxx = rcMax(layer->maxx, x);
					layer->miny = rcMin(layer->miny, y);
					layer->maxy = rcMax(layer->maxy, y);

					// Store height and area type.
					// �����Ɩʐς̎�ނ�ۑ����܂��B
					const int idx = x + y * lw;
					layer->heights[idx] = (unsigned char)(s.y - hmin);
					layer->areas[idx] = chf.areas[j];

					// Check connection.
					// �ڑ����m�F���܂��B
					unsigned char portal{};
					unsigned char con{};

					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int ax = cx + rcGetDirOffsetX(dir);
							const int ay = cy + rcGetDirOffsetY(dir);
							const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
							unsigned char alid = srcReg[ai] != UCharMax ? regs[srcReg[ai]].layerId : UCharMax;

							// Portal mask
							// �|�[�^���}�X�N
							if (chf.areas[ai] != RC_NULL_AREA && lid != alid)
							{
								portal |= (unsigned char)(1 << dir);
								// Update height so that it matches on both sides of the portal.
								// �������X�V���āA�|�[�^���̗����ň�v����悤�ɂ��܂��B
								const rcCompactSpan& as = chf.spans[ai];

								if (as.y > hmin)
									layer->heights[idx] = rcMax(layer->heights[idx], (unsigned char)(as.y - hmin));
							}

							// Valid connection mask
							// �L���Ȑڑ��}�X�N
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
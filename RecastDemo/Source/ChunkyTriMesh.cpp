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

#include "ChunkyTriMesh.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <array>
#include <vector>
#include <ppl.h>
#include <DirectXMath.h>
#include "OtherFiles\\Math.hpp"

#include "AlgorithmHelper.h"

namespace
{
	namespace exec = std::execution;
	namespace PPL = Concurrency;

	struct BoundsItem
	{
		std::array<float, 2> bmin, bmax;
		int i;
	};

	inline int compareItemX(const void* va, const void* vb)
	{
		const BoundsItem* a = (const BoundsItem*)va;
		const BoundsItem* b = (const BoundsItem*)vb;

		return (a->bmin[0] > b->bmin[0]) - (a->bmin[0] < b->bmin[0]);
	}

	inline int compareItemY(const void* va, const void* vb)
	{
		const BoundsItem* a = (const BoundsItem*)va;
		const BoundsItem* b = (const BoundsItem*)vb;

		return (a->bmin[1] > b->bmin[1]) - (a->bmin[1] < b->bmin[1]);
	}

	void calcExtends(const std::vector<BoundsItem>& items, [[maybe_unused]]const int nitems,
		const int imin, const int imax,
		std::array<float, 2>& bmin, std::array<float, 2>& bmax)
	{
		bmin[0] = items[imin].bmin[0];
		bmin[1] = items[imin].bmin[1];

		bmax[0] = items[imin].bmax[0];
		bmax[1] = items[imin].bmax[1];

		PPL::parallel_for(imin + 1, imax, 1, [&](const int i)
			{
				const BoundsItem& it = items[i];

				if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
				if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];

				if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
				if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
			});
	}

	inline constexpr int longestAxis(float x, float y)
	{
		return y > x ? 1 : 0;
	}

	void subdivide(std::vector<BoundsItem>& items, int nitems, int imin, int imax, int trisPerChunk,
		int& curNode, std::vector<rcChunkyTriMeshNode>& nodes, const int maxNodes,
		int& curTri, std::vector<int>& outTris, const int* inTris)
	{
		int inum = imax - imin;
		int icur = curNode;

		if (curNode > maxNodes)
			return;

		rcChunkyTriMeshNode& node = nodes[curNode++];

		if (inum <= trisPerChunk)
		{
			// Leaf
			calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);

			// Copy triangles.
			node.i = curTri;
			node.n = inum;

			for (int i = imin; i < imax; ++i)
			{
				const size_t in_index{ items[i].i * 3u };
				const size_t out_index{ curTri * 3u };

				const int* src = &inTris[in_index];

				curTri++;

				for (size_t j = 0; j < 3u; j++)
				{
					outTris[out_index + j] = src[j];
				}
			}
		}
		else
		{
			// Split
			calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);

			int	axis = longestAxis(node.bmax[0] - node.bmin[0],
				node.bmax[1] - node.bmin[1]);

			if (axis == 0)
			{
				// Sort along x-axis
				// X軸に沿って並べ替え
				qsort(items.data() + imin, static_cast<size_t>(inum), sizeof(BoundsItem), compareItemX);
			}
			else if (axis == 1)
			{
				// Sort along y-axis
				// y軸に沿って並べ替え
				qsort(items.data() + imin, static_cast<size_t>(inum), sizeof(BoundsItem), compareItemY);
			}

			int isplit = imin + inum / 2;

			// Left
			subdivide(items, nitems, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
			// Right
			subdivide(items, nitems, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);

			int iescape = curNode - icur;
			// Negative index means escape.
			// 負のインデックスはエスケープを意味します。
			node.i = -iescape;
		}
	}

	bool checkOverlapSegment(const std::array<float, 2>& p, const std::array<float, 2>& q,
		const std::array<float, 2>& bmin, const std::array<float, 2>& bmax)
	{
		constexpr float EPSILON = 1e-6f;

		float tmin = 0;
		float tmax = 1;
		float d[2];
		d[0] = q[0] - p[0];
		d[1] = q[1] - p[1];

		for (int i = 0; i < 2; i++)
		{
			if (fabsf(d[i]) < EPSILON)
			{
				// Ray is parallel to slab. No hit if origin not within slab
				if (p[i] < bmin[i] || p[i] > bmax[i])
					return false;
			}
			else
			{
				// Compute intersection t value of ray with near and far plane of slab
				float ood = 1.f / d[i];
				float t1 = (bmin[i] - p[i]) * ood;
				float t2 = (bmax[i] - p[i]) * ood;
				if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
				if (t1 > tmin) tmin = t1;
				if (t2 < tmax) tmax = t2;
				if (tmin > tmax) return false;
			}
		}
		return true;
	}

	inline constexpr bool checkOverlapRect(const float amin[2], const float amax[2],
		const std::array<float, 2>& bmin, const std::array<float, 2>& bmax)
	{
		bool overlap = true;
		overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
		overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
		return overlap;
	}
}

bool rcCreateChunkyTriMesh(
	const std::vector<float>& verts, const std::vector<int>& tris, int ntris, int trisPerChunk,
	rcChunkyTriMesh* cm)
{
	int nchunks = (ntris + trisPerChunk - 1) / trisPerChunk;

	try
	{
		cm->nodes.resize(nchunks * 4);
		cm->tris.resize(ntris * 3);
	}
	catch (const std::exception&)
	{
		return false;
	}

	cm->ntris = ntris;

	// Build tree
	// ツリーを構築します
	std::vector<BoundsItem> items;

	try
	{
		items.resize(ntris);
	}
	catch (const std::exception&)
	{
		return false;
	}

	PPL::parallel_for(0, ntris, 1, [&](const int i)
		{
			const int* t = &tris[i * 3];
			BoundsItem& it = items[i];
			it.i = i;
			// Calc triangle XZ bounds.
			// 三角形のXZ境界を計算します。
			it.bmin[0] = it.bmax[0] = verts[t[0] * 3 + 0];
			it.bmin[1] = it.bmax[1] = verts[t[0] * 3 + 2];
			for (int j = 1; j < 3; ++j)
			{
				const float* v = &verts[t[j] * 3];
				if (v[0] < it.bmin[0]) it.bmin[0] = v[0];
				if (v[2] < it.bmin[1]) it.bmin[1] = v[2];

				if (v[0] > it.bmax[0]) it.bmax[0] = v[0];
				if (v[2] > it.bmax[1]) it.bmax[1] = v[2];
			}
		});

	int curTri{}, curNode{};

	subdivide(items, ntris, 0, ntris, trisPerChunk, curNode, cm->nodes, nchunks * 4, curTri, cm->tris, tris.data());

	cm->nnodes = curNode;

	// Calc max tris per node.
	// ノードごとの三角形の最大数を計算します。
	cm->maxTrisPerChunk = 0;
	for (int i = 0; i < cm->nnodes; ++i)
	{
		rcChunkyTriMeshNode& node = cm->nodes[i];
		const bool isLeaf = node.i >= 0;

		if (!isLeaf) continue;

		if (node.n > cm->maxTrisPerChunk)
			cm->maxTrisPerChunk = node.n;
	}

	For_Each_N(cm->nodes, cm->nnodes, [](rcChunkyTriMeshNode& node)
		{
			node.origin_bmax = node.bmax;
			node.origin_bmin = node.bmin;
		}, exec::par);

	return true;
}

// チャンクの重複する四角形を取得
int rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm,
	float bmin[2], float bmax[2],
	int* ids, const int maxIds)
{
	// Traverse tree // トラバースツリー（データ構造）
	int i{};
	int n{};

	while (i < cm->nnodes)
	{
		const rcChunkyTriMeshNode* node = &cm->nodes[i];
		const bool overlap = checkOverlapRect(bmin, bmax, node->bmin, node->bmax);
		const bool isLeafNode = (node->i >= 0);

		if (isLeafNode && overlap)
		{
			if (n < maxIds)
			{
				ids[n] = i;
				n++;
			}
		}

		if (overlap || isLeafNode)
			i++;
		else
		{
			const int escapeIndex = -node->i;
			i += escapeIndex;
		}
	}

	return n;
}

int rcGetChunksOverlappingSegment(const rcChunkyTriMesh* cm,
	const std::array<float, 2>& p, const std::array<float, 2>& q, int* ids, const int maxIds)
{
	// Traverse tree
	int i = 0;
	int n = 0;
	while (i < cm->nnodes)
	{
		const rcChunkyTriMeshNode* node = &cm->nodes[i];
		const bool overlap = checkOverlapSegment(p, q, node->bmin, node->bmax);
		const bool isLeafNode = node->i >= 0;

		if (isLeafNode && overlap)
		{
			if (n < maxIds)
			{
				ids[n] = i;
				n++;
			}
		}

		if (overlap || isLeafNode)
			i++;
		else
		{
			const int escapeIndex = -node->i;
			i += escapeIndex;
		}
	}

	return n;
}

void rcChunkyTriMesh::MoveNodes(
	const std::array<float, 3>& pos, const std::array<float, 3>& rotate, const std::array<float, 3>& scale)
{
	For_Each_N(nodes, nnodes, [&](rcChunkyTriMeshNode& node)
		{
			node.bmax[0] = node.origin_bmax[0] + pos[0];
			node.bmax[1] = node.origin_bmax[1] + pos[1];
		}, exec::par);

	using namespace DirectX;
	using Math::ToRadian;

	const auto S = XMMatrixScaling(scale[0], scale[1], scale[2]);
	const auto R = XMMatrixRotationRollPitchYaw(ToRadian(rotate[0]), ToRadian(rotate[1]), ToRadian(rotate[2]));
	const auto T = XMMatrixTranslation(pos[0], pos[1], pos[2]);

	// ワールド変換行列
	auto W = S * R * T;

	For_Each_N(nodes, nnodes, [&](rcChunkyTriMeshNode& node)
		{
			const auto pos_bmax = XMMatrixTranslation(node.origin_bmax.front(), node.origin_bmax.back(), 0);
			const auto pos_bmin = XMMatrixTranslation(node.origin_bmin.front(), node.origin_bmin.back(), 0);

			{
				const auto result{ pos_bmax * W };
				auto& vs_pos{ result.r[3].m128_f32 };

				node.bmax[0] = vs_pos[0];
				node.bmax[1] = vs_pos[1];
			}

			{
				const auto result{ pos_bmin * W };
				auto& vs_pos{ result.r[3].m128_f32 };

				node.bmin[0] = vs_pos[0];
				node.bmin[1] = vs_pos[1];
			}
		}, exec::par);
}

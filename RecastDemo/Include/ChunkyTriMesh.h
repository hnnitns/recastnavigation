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

#ifndef CHUNKYTRIMESH_H
#define CHUNKYTRIMESH_H

#include <array>
#include <vector>

struct rcChunkyTriMeshNode
{
	std::array<float, 2> bmin, bmax;
	int i;
	int n;
};

struct rcChunkyTriMesh
{
	inline rcChunkyTriMesh() : nnodes(0), ntris(0), maxTrisPerChunk(0) {};
	inline ~rcChunkyTriMesh() noexcept = default;

	std::vector<rcChunkyTriMeshNode> nodes;
	int nnodes;
	std::vector<int> tris;
	int ntris;
	int maxTrisPerChunk;

public:
	rcChunkyTriMesh(rcChunkyTriMesh&& _rt) noexcept
	{
		using std::move;

		nodes = move(_rt.nodes);
		nnodes = (_rt.nnodes);
		tris = move(_rt.tris);
		ntris = (_rt.ntris);
		maxTrisPerChunk = (_rt.maxTrisPerChunk);
	}
	rcChunkyTriMesh& operator=(rcChunkyTriMesh&& _rt) noexcept
	{
		using std::move;

		if (this != &_rt)
		{
			nodes = move(_rt.nodes);
			nnodes = (_rt.nnodes);
			tris = move(_rt.tris);
			ntris = (_rt.ntris);
			maxTrisPerChunk = (_rt.maxTrisPerChunk);
		}
	}
	rcChunkyTriMesh(const rcChunkyTriMesh& _rt) noexcept
	{
		nodes = (_rt.nodes);
		nnodes = (_rt.nnodes);
		tris = (_rt.tris);
		ntris = (_rt.ntris);
		maxTrisPerChunk = (_rt.maxTrisPerChunk);
	}
	rcChunkyTriMesh& operator=(const rcChunkyTriMesh& _rt) noexcept
	{
		if (this != &_rt)
		{
			nodes = (_rt.nodes);
			nnodes = (_rt.nnodes);
			tris = (_rt.tris);
			ntris = (_rt.ntris);
			maxTrisPerChunk = (_rt.maxTrisPerChunk);
		}
	}
};

// Creates partitioned triangle mesh (AABB tree), where each node contains at max trisPerChunk triangles.
// 分割された三角形メッシュ（AABBツリー）を作成します。各ノードには最大でtrisPerChunkの三角形が含まれます。
// 分厚い三角形メッシュを作成する
bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris, int trisPerChunk, rcChunkyTriMesh* cm);

// Returns the chunk indices which overlap the input rectable.
// 入力rectableと重複するチャンクインデックスを返します。
// チャンクの重複する四角形を取得
int rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm, float bmin[2], float bmax[2], int* ids, const int maxIds);

// Returns the chunk indices which overlap the input segment.
// 入力セグメントと重複するチャンクインデックスを返します。
// チャンク重複セグメントの取得
int rcGetChunksOverlappingSegment(const rcChunkyTriMesh* cm,
	const std::array<float, 2>& p, const std::array<float, 2>& q, int* ids, const int maxIds);

#endif // CHUNKYTRIMESH_H

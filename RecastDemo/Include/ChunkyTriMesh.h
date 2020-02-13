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
	rcChunkyTriMesh() : nnodes(0), ntris(0), maxTrisPerChunk(0) {};
	~rcChunkyTriMesh() noexcept = default;

	std::vector<rcChunkyTriMeshNode> nodes;
	int nnodes;
	std::vector<int> tris;
	int ntris;
	int maxTrisPerChunk;

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcChunkyTriMesh(const rcChunkyTriMesh&) = delete;
	rcChunkyTriMesh(rcChunkyTriMesh&&) = delete;
	rcChunkyTriMesh& operator=(const rcChunkyTriMesh&) = delete;
	rcChunkyTriMesh& operator=(rcChunkyTriMesh&&) = delete;
};

// Creates partitioned triangle mesh (AABB tree), where each node contains at max trisPerChunk triangles.
// �������ꂽ�O�p�`���b�V���iAABB�c���[�j���쐬���܂��B�e�m�[�h�ɂ͍ő��trisPerChunk�̎O�p�`���܂܂�܂��B
// �������O�p�`���b�V�����쐬����
bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris, int trisPerChunk, rcChunkyTriMesh* cm);

// Returns the chunk indices which overlap the input rectable.
// ����rectable�Əd������`�����N�C���f�b�N�X��Ԃ��܂��B
// �`�����N�̏d������l�p�`���擾
int rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm, float bmin[2], float bmax[2], int* ids, const int maxIds);

// Returns the chunk indices which overlap the input segment.
// ���̓Z�O�����g�Əd������`�����N�C���f�b�N�X��Ԃ��܂��B
// �`�����N�d���Z�O�����g�̎擾
int rcGetChunksOverlappingSegment(const rcChunkyTriMesh* cm, float p[2], float q[2], int* ids, const int maxIds);

#endif // CHUNKYTRIMESH_H

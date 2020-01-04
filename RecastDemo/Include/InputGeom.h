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

#ifndef INPUTGEOM_H
#define INPUTGEOM_H

#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"
#include "Recast.h"

constexpr int MAX_CONVEXVOL_PTS = 12;

struct ConvexVolume
{
	ConvexVolume()
		: areaMod(RC_AREA_FLAGS_MASK), verts{}, hmin{}, hmax{}, nverts{}
	{}

	std::array<float, MAX_CONVEXVOL_PTS * 3> verts;
	float hmin, hmax;
	int nverts;
	rcAreaModification areaMod;
};

struct BuildSettings
{
	// Cell size in world units // ���[���h�P�ʂ̃Z���T�C�Y
	float cellSize;

	// Cell height in world units // ���[���h�P�ʂ̃Z���̍���
	float cellHeight;

	// Agent height in world units // ���[���h�P�ʂł̃G�[�W�F���g�̍���
	float agentHeight;

	// Agent radius in world units // ���[���h�P�ʂ̃G�[�W�F���g���a
	float agentRadius;

	// Agent max climb in world units // ���[���h�P�ʂł̃G�[�W�F���g�ő�㏸
	float agentMaxClimb;

	// Agent max slope in degrees // �G�[�W�F���g�̍ő���z�i�x�P�ʁj
	float agentMaxSlope;

	// Region minimum size in voxels. // �{�N�Z���P�ʂ̗̈�̍ŏ��T�C�Y�B
	// regionMinSize = sqrt(regionMinArea)
	float regionMinSize;

	// Region merge size in voxels. // �{�N�Z���P�ʂ̗̈�}�[�W�T�C�Y�B
	// regionMergeSize = sqrt(regionMergeArea)
	float regionMergeSize;

	// Edge max length in world units // ���[���h�P�ʂ̃G�b�W�̍ő咷
	float edgeMaxLen;

	// Edge max error in voxels // �{�N�Z���̃G�b�W�ő�덷
	float edgeMaxError;
	float vertsPerPoly;

	// Detail sample distance in voxels // �{�N�Z���P�ʂ̏ڍׂȃT���v������
	float detailSampleDist;

	// Detail sample max error in voxel heights. // �{�N�Z���̍����̍ő�T���v���G���[�̏ڍׁB
	float detailSampleMaxError;

	// Partition type, see SamplePartitionType // �����^�C�v�ASamplePartitionType���Q��
	int partitionType;

	// Bounds of the area to mesh // ���b�V���̗̈�̋��E
	std::array<float, 3> navMeshBMin;
	std::array<float, 3> navMeshBMax;

	// Size of the tiles in voxels // �{�N�Z���̃^�C���̃T�C�Y
	float tileSize;
};

class InputGeom
{
	std::unique_ptr<rcChunkyTriMesh> m_chunkyMesh;
	std::unique_ptr<rcMeshLoaderObj> m_mesh;
	std::array<float, 3> m_meshBMin, m_meshBMax; // ���b�V���f�[�^�̈ʒu�I�ȍő�l�A�ŏ��l
	BuildSettings m_buildSettings;
	bool m_hasBuildSettings;

	// @name Off-Mesh connections. // �I�t���b�V���ڑ��B
	//@{
	static constexpr int MAX_OFFMESH_CONNECTIONS{ 256 };

	std::array<float, MAX_OFFMESH_CONNECTIONS * 3 * 2> m_offMeshConVerts;
	std::array<float, MAX_OFFMESH_CONNECTIONS> m_offMeshConRads;
	std::array<uint8_t, MAX_OFFMESH_CONNECTIONS> m_offMeshConDirs;
	std::array<uint8_t, MAX_OFFMESH_CONNECTIONS> m_offMeshConAreas;
	std::array<uint16_t, MAX_OFFMESH_CONNECTIONS> m_offMeshConFlags;
	std::array<uint32_t, MAX_OFFMESH_CONNECTIONS> m_offMeshConId;
	int m_offMeshConCount;
	//@}

	// @name Convex Volumes. // �ʃ{�����[���B
	//@{
	static constexpr int MAX_VOLUMES = 256;
	std::array<ConvexVolume, MAX_VOLUMES> m_volumes;
	int m_volumeCount;
	//@}

	bool loadMesh(class rcContext* ctx, const std::string& filepath);
	bool loadGeomSet(class rcContext* ctx, const std::string& filepath);
public:
	InputGeom();
	~InputGeom() noexcept = default;

	bool load(class rcContext* ctx, const std::string& filepath);
	bool saveGeomSet(const BuildSettings* settings);

	// Method to return static mesh data.
	// �ÓI���b�V���f�[�^��Ԃ����\�b�h�B
	const auto& getMesh() const { return m_mesh; }
	const auto* getMeshBoundsMin() const { return &m_meshBMin; } // ���b�V�����E�̍ŏ��l���擾
	const auto* getMeshBoundsMax() const { return &m_meshBMax; } // ���b�V�����E�̍ő�l���擾
	// �i�r���b�V�����E�̍ŏ��l���擾
	const auto* getNavMeshBoundsMin() const { return m_hasBuildSettings ? &m_buildSettings.navMeshBMin : &m_meshBMin; }
	// �i�r���b�V�����E�̍ő�l���擾
	const auto* getNavMeshBoundsMax() const { return m_hasBuildSettings ? &m_buildSettings.navMeshBMax : &m_meshBMax; }
	const auto& getChunkyMesh() const { return m_chunkyMesh; }
	const BuildSettings* getBuildSettings() const { return m_hasBuildSettings ? &m_buildSettings : nullptr; }

	// ���b�V���f�[�^�ƃ}�E�X�̃��C�Ƃ̔���
	// src : ���C�̎n�_�Adst�F���C�̏I�_�A���C�̒�����1�Ƃ������̃��b�V���f�[�^�Ƃ̋�����̌�_
	bool raycastMesh(float* src, float* dst, float& tmin);

	// @name Off-Mesh connections. // �I�t���b�V���ڑ��B
	//@{
	int getOffMeshConnectionCount() const { return m_offMeshConCount; }
	const auto* getOffMeshConnectionVerts() const { return &m_offMeshConVerts; }
	const auto* getOffMeshConnectionRads() const { return &m_offMeshConRads; }
	const auto* getOffMeshConnectionDirs() const { return &m_offMeshConDirs; }
	const auto* getOffMeshConnectionAreas() const { return &m_offMeshConAreas; }
	const auto* getOffMeshConnectionFlags() const { return &m_offMeshConFlags; }
	const auto* getOffMeshConnectionId() const { return &m_offMeshConId; }
	void addOffMeshConnection(const float* spos, const float* epos, const float rad,
		unsigned char bidir, unsigned char area, unsigned short flags);
	void deleteOffMeshConnection(int i);
	void drawOffMeshConnections(struct duDebugDraw* dd, bool hilight = false);
	//@}

	// @name Box Volumes. // �{�b�N�X�{�����[���B
	//@{
	int getConvexVolumeCount() const { return m_volumeCount; }
	const auto* getConvexVolumes() const { return &m_volumes; }
	void addConvexVolume(const float* verts, const int nverts,
		const float minh, const float maxh, rcAreaModification areaMod);
	void deleteConvexVolume(int i);
	void drawConvexVolumes(struct duDebugDraw* dd, bool hilight = false);
	//@}

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	InputGeom(const InputGeom&) = delete;
	InputGeom(InputGeom&&) = delete;
	InputGeom& operator=(const InputGeom&) = delete;
	InputGeom& operator=(InputGeom&&) = delete;
};

#endif // INPUTGEOM_H

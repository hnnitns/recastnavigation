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
#include <optional>
#include <deque>

constexpr int MAX_CONVEXVOL_PTS = 12;

struct ConvexVolume
{
	ConvexVolume()
		: area(0), verts{}, hmin{}, hmax{}, nverts{}
	{}

	std::array<float, MAX_CONVEXVOL_PTS * 3> verts;
	float hmin, hmax;
	int nverts;
	int area;
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

	struct MeshData
	{
		std::array<float, 3> pos, scale, rotate;
	};

	// ���b�V���̍��W�A�傫���A�p�x
	std::deque<MeshData> meshes;
};

class InputGeom
{
public:
	struct LoadGeomMesh
	{
		LoadGeomMesh()
			: pos(), rotate()
		{
			scale.fill(1.f);
		}
		~LoadGeomMesh() = default;

		std::optional<rcChunkyTriMesh> m_chunkyMesh;
		std::optional<rcMeshLoaderObj> m_mesh;
		std::array<float, 3> m_meshBMin{}, m_meshBMax{}; // ���b�V���f�[�^�̈ʒu�I�ȍő�l�A�ŏ��l
		bool is_selected{ true };
		std::array<float, 3> pos, scale, rotate;
		bool is_changed{};

		void Update();

		LoadGeomMesh(const LoadGeomMesh& _rt) noexcept
		{
			m_chunkyMesh = (_rt.m_chunkyMesh);
			pos = _rt.pos;
			scale = _rt.scale;
			rotate = _rt.rotate;
			m_mesh = (_rt.m_mesh);
			m_meshBMin = (_rt.m_meshBMin);
			m_meshBMax = (_rt.m_meshBMax);
			is_selected = (_rt.is_selected);
		}
		LoadGeomMesh& operator=(const LoadGeomMesh& _rt) noexcept
		{
			if (this != &_rt)
			{
				m_chunkyMesh = (_rt.m_chunkyMesh);
				m_mesh = (_rt.m_mesh);
				pos = _rt.pos;
				scale = _rt.scale;
				rotate = _rt.rotate;
				m_meshBMin = (_rt.m_meshBMin);
				m_meshBMax = (_rt.m_meshBMax);
				is_selected = (_rt.is_selected);
			}

			return (*this);
		}

		LoadGeomMesh(LoadGeomMesh&& _rt) noexcept
		{
			using std::move;

			m_chunkyMesh = move(_rt.m_chunkyMesh);
			m_mesh = move(_rt.m_mesh);
			m_meshBMin = move(_rt.m_meshBMin);
			m_meshBMax = move(_rt.m_meshBMax);
			is_selected = (_rt.is_selected);
			pos = move(_rt.pos);
			scale = move(_rt.scale);
			rotate = move(_rt.rotate);
		}
		LoadGeomMesh& operator=(LoadGeomMesh&& _rt) noexcept
		{
			using std::move;

			if (this != &_rt)
			{
				m_chunkyMesh = move(_rt.m_chunkyMesh);
				m_mesh = move(_rt.m_mesh);
				m_meshBMin = move(_rt.m_meshBMin);
				m_meshBMax = move(_rt.m_meshBMax);
				is_selected = (_rt.is_selected);
				pos = move(_rt.pos);
				scale = move(_rt.scale);
				rotate = move(_rt.rotate);
			}

			return (*this);
		}
	};
	struct RaycastMeshHitInfo
	{
		float dis{}; // �Փ˒n�_�܂ł̋���
		std::array<float, 3> pos{}; // �Փ˒n�_
		std::array<float, 3> vec{}; // �Փ˂ւ̃x�N�g��
	};

private:
	std::deque<LoadGeomMesh> load_geom_meshes; // �ǂݍ��񂾃��b�V��
	std::array<float, 3> all_meshBMin, all_meshBMax; // �S���b�V���f�[�^�̈ʒu�I�ȍő�l�A�ŏ��l
	std::optional<BuildSettings> m_buildSettings;

	// @name Off-Mesh connections. // �I�t���b�V���ڑ��B
	//@{
	static constexpr int MAX_OFFMESH_CONNECTIONS{ 256 };

	std::array<float, MAX_OFFMESH_CONNECTIONS * 3 * 2> m_offMeshConVerts{};
	std::array<float, MAX_OFFMESH_CONNECTIONS> m_offMeshConRads{};
	std::array<uint8_t, MAX_OFFMESH_CONNECTIONS> m_offMeshConDirs{};
	std::array<uint8_t, MAX_OFFMESH_CONNECTIONS> m_offMeshConAreas{};
	std::array<uint16_t, MAX_OFFMESH_CONNECTIONS> m_offMeshConFlags{};
	std::array<uint32_t, MAX_OFFMESH_CONNECTIONS> m_offMeshConId{};
	int m_offMeshConCount;
	//@}

	// @name Convex Volumes. // �ʃ{�����[���B
	//@{
	static constexpr int MAX_VOLUMES = 256;
	std::array<ConvexVolume, MAX_VOLUMES> m_volumes{};
	int m_volumeCount;
	//@}

public:

	bool LoadMesh(class rcContext* ctx, const std::string& filepath);
	bool LoadGeomSet(class rcContext* ctx, const std::string& filepath);
	void CalcAllMeshBounds();

	InputGeom();
	~InputGeom() noexcept = default;

	bool Load(class rcContext* ctx, const std::string& filepath);
	bool SaveGeomSet(const BuildSettings* settings);

	// ���b�V���f�[�^�ƃ}�E�X�̃��C�Ƃ̔���
	// src : ���C�̎n�_�Adst�F���C�̏I�_
	bool RaycastMesh(const std::array<float, 3>& ray_start, const std::array<float, 3>& ray_end,
		RaycastMeshHitInfo* hit_info = nullptr);

	void ClearLoadGeomMesh() noexcept { load_geom_meshes.clear(); }
	std::deque<LoadGeomMesh>::iterator EraseSelectLoadGeomMesh() noexcept;

	// Method to return static mesh data.
	// �ÓI���b�V���f�[�^��Ԃ����\�b�h�B
	const auto& getLoadGeomMesh() const noexcept { return load_geom_meshes; }
	auto& getEditableLoadGeomMesh() noexcept { return load_geom_meshes; }
	size_t getLoadGeomMeshSize() const noexcept { return load_geom_meshes.size(); }
	bool isLoadGeomMeshEmpty() const noexcept { return load_geom_meshes.empty(); }
	const auto& getMeshAt(const size_t num) const { return load_geom_meshes.at(num).m_mesh; }
	const auto& getMeshBoundsMin() const { return all_meshBMin; } // ���b�V�����E�̍ŏ��l���擾
	const auto& getMeshBoundsMax() const { return all_meshBMax; } // ���b�V�����E�̍ő�l���擾
	// �i�r���b�V�����E�̍ŏ��l���擾
	const auto& getNavMeshBoundsMin() const
	{ return m_buildSettings ? m_buildSettings->navMeshBMin : all_meshBMin; }
	// �i�r���b�V�����E�̍ő�l���擾
	const auto& getNavMeshBoundsMax() const
	{ return m_buildSettings ? m_buildSettings->navMeshBMax : all_meshBMax; }
	const auto& getChunkyMeshAt(const size_t num) const { return load_geom_meshes.at(num).m_chunkyMesh; }
	const BuildSettings* getBuildSettings() const { return m_buildSettings ? &(*m_buildSettings) : nullptr; }

	// @name Off-Mesh connections. // �I�t���b�V���ڑ��B
	//@{
	int getOffMeshConnectionCount() const noexcept { return m_offMeshConCount; }
	const auto& getOffMeshConnectionVerts() const noexcept { return m_offMeshConVerts; }
	const auto& getOffMeshConnectionRads() const noexcept { return m_offMeshConRads; }
	const auto& getOffMeshConnectionDirs() const noexcept { return m_offMeshConDirs; }
	const auto& getOffMeshConnectionAreas() const noexcept { return m_offMeshConAreas; }
	const auto& getOffMeshConnectionFlags() const noexcept { return m_offMeshConFlags; }
	const auto& getOffMeshConnectionId() const noexcept { return m_offMeshConId; }
	void addOffMeshConnection(const float* spos, const float* epos, const float rad,
		unsigned char bidir, unsigned char area, unsigned short flags);
	void deleteOffMeshConnection(int i);
	void drawOffMeshConnections(struct duDebugDraw* dd, bool hilight = false);
	//@}

	// @name Box Volumes. // �{�b�N�X�{�����[���B
	//@{
	int getConvexVolumeCount() const noexcept { return m_volumeCount; }
	const auto& getConvexVolumes() const noexcept { return m_volumes; }
	void addConvexVolume(const float* verts, const int nverts,
		const float minh, const float maxh, unsigned char area);
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

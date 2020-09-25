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
	// Cell size in world units // ワールド単位のセルサイズ
	float cellSize;

	// Cell height in world units // ワールド単位のセルの高さ
	float cellHeight;

	// Agent height in world units // ワールド単位でのエージェントの高さ
	float agentHeight;

	// Agent radius in world units // ワールド単位のエージェント半径
	float agentRadius;

	// Agent max climb in world units // ワールド単位でのエージェント最大上昇
	float agentMaxClimb;

	// Agent max slope in degrees // エージェントの最大勾配（度単位）
	float agentMaxSlope;

	// Region minimum size in voxels. // ボクセル単位の領域の最小サイズ。
	// regionMinSize = sqrt(regionMinArea)
	float regionMinSize;

	// Region merge size in voxels. // ボクセル単位の領域マージサイズ。
	// regionMergeSize = sqrt(regionMergeArea)
	float regionMergeSize;

	// Edge max length in world units // ワールド単位のエッジの最大長
	float edgeMaxLen;

	// Edge max error in voxels // ボクセルのエッジ最大誤差
	float edgeMaxError;
	float vertsPerPoly;

	// Detail sample distance in voxels // ボクセル単位の詳細なサンプル距離
	float detailSampleDist;

	// Detail sample max error in voxel heights. // ボクセルの高さの最大サンプルエラーの詳細。
	float detailSampleMaxError;

	// Partition type, see SamplePartitionType // 分割タイプ、SamplePartitionTypeを参照
	int partitionType;

	// Bounds of the area to mesh // メッシュの領域の境界
	std::array<float, 3> navMeshBMin;
	std::array<float, 3> navMeshBMax;

	// Size of the tiles in voxels // ボクセルのタイルのサイズ
	float tileSize;

	struct MeshData
	{
		std::array<float, 3> pos, scale, rotate;
	};

	// メッシュの座標、大きさ、角度
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
		std::array<float, 3> m_meshBMin{}, m_meshBMax{}; // メッシュデータの位置的な最大値、最小値
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
		float dis{}; // 衝突地点までの距離
		std::array<float, 3> pos{}; // 衝突地点
		std::array<float, 3> vec{}; // 衝突へのベクトル
	};

private:
	std::deque<LoadGeomMesh> load_geom_meshes; // 読み込んだメッシュ
	std::array<float, 3> all_meshBMin, all_meshBMax; // 全メッシュデータの位置的な最大値、最小値
	std::optional<BuildSettings> m_buildSettings;

	// @name Off-Mesh connections. // オフメッシュ接続。
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

	// @name Convex Volumes. // 凸ボリューム。
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

	// メッシュデータとマウスのレイとの判定
	// src : レイの始点、dst：レイの終点
	bool RaycastMesh(const std::array<float, 3>& ray_start, const std::array<float, 3>& ray_end,
		RaycastMeshHitInfo* hit_info = nullptr);

	void ClearLoadGeomMesh() noexcept { load_geom_meshes.clear(); }
	std::deque<LoadGeomMesh>::iterator EraseSelectLoadGeomMesh() noexcept;

	// Method to return static mesh data.
	// 静的メッシュデータを返すメソッド。
	const auto& getLoadGeomMesh() const noexcept { return load_geom_meshes; }
	auto& getEditableLoadGeomMesh() noexcept { return load_geom_meshes; }
	size_t getLoadGeomMeshSize() const noexcept { return load_geom_meshes.size(); }
	bool isLoadGeomMeshEmpty() const noexcept { return load_geom_meshes.empty(); }
	const auto& getMeshAt(const size_t num) const { return load_geom_meshes.at(num).m_mesh; }
	const auto& getMeshBoundsMin() const { return all_meshBMin; } // メッシュ境界の最小値を取得
	const auto& getMeshBoundsMax() const { return all_meshBMax; } // メッシュ境界の最大値を取得
	// ナビメッシュ境界の最小値を取得
	const auto& getNavMeshBoundsMin() const
	{ return m_buildSettings ? m_buildSettings->navMeshBMin : all_meshBMin; }
	// ナビメッシュ境界の最大値を取得
	const auto& getNavMeshBoundsMax() const
	{ return m_buildSettings ? m_buildSettings->navMeshBMax : all_meshBMax; }
	const auto& getChunkyMeshAt(const size_t num) const { return load_geom_meshes.at(num).m_chunkyMesh; }
	const BuildSettings* getBuildSettings() const { return m_buildSettings ? &(*m_buildSettings) : nullptr; }

	// @name Off-Mesh connections. // オフメッシュ接続。
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

	// @name Box Volumes. // ボックスボリューム。
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

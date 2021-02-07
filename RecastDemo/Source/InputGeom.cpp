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
#include <cmath>
#include <cstdio>
#include <cctype>
#include <cstring>
#include <vector>
#include <algorithm>
#include <execution>
#include <atomic>
#include <limits>

#include "Recast.h"
#include "InputGeom.h"
#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"
#include "DebugDraw.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "Sample.h"
#include "OtherFiles\\AlgorithmHelper.hpp"

using namespace RcMath;
namespace exec = std::execution;

namespace
{
	bool intersectSegmentTriangle(const std::array<float, 3>& sp, const std::array<float, 3>& sq,
		const float* a, const float* b, const float* c, float& t)
	{
		float v{}, w{};
		std::array<float, 3> ab{}, ac{}, qp{}, ap{}, norm{}, e{};

		rcVsub(ab.data(), b, a);
		rcVsub(ac.data(), c, a);
		qp = sp - sq;

		// Compute triangle normal. Can be precalculated or cached if
		// intersecting multiple segments against the same triangle
		// 三角形の法線を計算します。 同じ三角形に対して複数のセグメントを交差させる場合、事前計算またはキャッシュできます
		rcVcross(&norm, ab, ac);

		// Compute denominator d. If d <= 0, segment is parallel to or points
		// away from triangle, so exit early
		// 分母dを計算します。 d <= 0の場合、セグメントは三角形に平行か、三角形から遠ざかるので、早く終了します
		float d = rcVdot(qp, norm);
		if (d <= 0.0f) return false;

		// Compute intersection dis value of pq with plane of triangle. A ray
		// intersects iff 0 <= dis. Segment intersects iff 0 <= dis <= 1. Delay
		// dividing by d until intersection has been found to pierce triangle
		// 三角形の平面とのpqの交差t値を計算します。
		// 0 <= tの場合、光線は交差します。 0 <= dis <= 1の場合、セグメントは交差します。
		// 交差点が三角形を突き抜けていることがわかるまで、dによる除算を遅らせます。
		rcVsub(ap.data(), sp.data(), a);
		t = rcVdot(ap, norm);
		if (t < 0.0f) return false;
		if (t > d) return false; // For segment; exclude this code line for a ray test セグメント; 光線テストのためにこのコード行を除外します

		// Compute barycentric coordinate components and test if within bounds
		// 重心座標成分を計算し、範囲内かどうかをテストします。
		rcVcross(&e, qp, ap);
		v = rcVdot(ac, e);
		if (v < 0.0f || v > d) return false;
		w = -rcVdot(ab, e);
		if (w < 0.0f || v + w > d) return false;

		// Segment/ray intersects triangle. Perform delayed division
		// セグメント/レイは三角形と交差します。 遅延除算を実行します。
		t /= d;

		return true;
	}

	char* parseRow(char* buf, char* bufEnd, char* row, int len)
	{
		bool start = true;
		bool done = false;
		int n = 0;
		while (!done && buf < bufEnd)
		{
			char c = *buf;
			buf++;
			// multirow
			switch (c)
			{
			case '\n':
				if (start) break;
				done = true;
				break;
			case '\r':
				break;
			case '\t':
			case ' ':
				if (start) break;
				// else falls through
			default:
				start = false;
				row[n++] = c;
				if (n >= len - 1)
					done = true;
				break;
			}
		}
		row[n] = '\0';
		return buf;
	}

	// メッシュデータの全てを囲む四角とレイとの判定
	bool isectSegAABB(const std::array<float, 3>& sp, const std::array<float, 3>& sq,
		const std::array<float, 3>& amin, const std::array<float, 3>& amax,
		float& tmin, float& tmax)
	{
		constexpr float EPS = 1e-6f;

		std::array<float, 3> d{ sq - sp };

		// 線上で最初のヒットを、-FLT_MAXに設定
		tmin = 0.0;

		// 光線が移動できる最大距離に設定（セグメント用）
		tmax = 1.f;

		// 3つのスラブすべて
		for (int i = 0; i < 3; i++)
		{
			if (fabsf(d[i]) < EPS)
			{
				// 光線はスラブに平行か、原点がスラブ内にない場合はヒットなし
				if (sp[i] < amin[i] || sp[i] > amax[i])
					return false;
			}
			else
			{
				//スラブのニアおよびファープレーンとレイの交差t値を計算します
				const float ood = 1.f / d[i];
				float t1 = (amin[i] - sp[i]) * ood;
				float t2 = (amax[i] - sp[i]) * ood;

				// t1を近くの平面と交差させ、t2を遠くの平面と交差させる
				if (t1 > t2) std::swap(t1, t2);
				if (t1 > tmin) tmin = t1;
				if (t2 < tmax) tmax = t2;

				// スラブの交差点が無くなるとすぐに衝突なしで終了
				if (tmin > tmax) return false;
			}
		}

		return true;
	}
}

InputGeom::InputGeom()
	: m_offMeshConCount{}, m_volumeCount{}, all_meshBMin{}, all_meshBMax{}
{}

bool InputGeom::LoadMesh(rcContext* ctx, const std::string& filepath)
{
	m_offMeshConCount = 0;
	m_volumeCount = 0;

	For_Each(load_geom_meshes, [](LoadGeomMesh& mesh) { mesh.is_selected = false; }, exec::par);

	auto& load_meshes{ load_geom_meshes.emplace_back() };

	try
	{
		load_meshes.m_mesh.emplace();
	}
	catch (const std::exception&)
	{
		ctx->log(RC_LOG_ERROR, "loadMesh: Out of memory 'm_mesh'."); // メモリー不足「m_mesh」
		return false;
	}

	if (!load_meshes.m_mesh->load(filepath))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Could not load '%s'", filepath.c_str()); // 読み込めない
		return false;
	}

	rcCalcBounds(load_meshes.m_mesh->getVerts().data(), load_meshes.m_mesh->getVertCount(),
		load_meshes.m_meshBMin.data(), load_meshes.m_meshBMax.data());

	CalcAllMeshBounds();

	try
	{
		load_meshes.m_chunkyMesh.emplace();
	}
	catch (const std::exception&)
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'm_chunkyMesh'."); // メモリー不足「m_chunkyMesh」
		return false;
	}

	if (!rcCreateChunkyTriMesh(load_meshes.m_mesh->getVerts(), load_meshes.m_mesh->getTris(), load_meshes.m_mesh->getTriCount(), 256, &(*load_meshes.m_chunkyMesh)))
	{
		ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Failed to build chunky mesh."); // チャンキーメッシュの構築に失敗しました
		return false;
	}

	load_meshes.is_selected = true;

	return true;
}

bool InputGeom::LoadGeomSet(rcContext* ctx, const std::string& filepath)
{
	std::vector<char> buf;
	FILE* fp = fopen(filepath.c_str(), "rb");
	if (!fp)
	{
		return false;
	}
	if (fseek(fp, 0, SEEK_END) != 0)
	{
		fclose(fp);
		return false;
	}

	long bufSize = ftell(fp);
	if (bufSize < 0)
	{
		fclose(fp);
		return false;
	}
	if (fseek(fp, 0, SEEK_SET) != 0)
	{
		fclose(fp);
		return false;
	}

	try
	{
		buf.resize(bufSize);
	}
	catch (const std::exception&)
	{
		fclose(fp);
		return false;
	}

	size_t readLen = fread(buf.data(), bufSize, 1, fp);
	fclose(fp);

	if (readLen != 1)
	{
		return false;
	}

	m_offMeshConCount = 0;
	m_volumeCount = 0;
	load_geom_meshes.clear();

	char* src = buf.data();
	char* srcEnd = buf.data() + bufSize;
	std::array<char, 512> row{};

	while (src < srcEnd)
	{
		// Parse one row
		// 1行を解析します
		row[0] = '\0';
		src = parseRow(src, srcEnd, row.data(), sizeof(row) / sizeof(char));

		if (row[0] == 'f')
		{
			// File name.
			const char* name = row.data() + 1;

			// Skip white spaces
			while (*name && isspace(*name))
				name++;

			if (*name)
			{
				if (!LoadMesh(ctx, name)) return false;
			}
		}
		else if (row[0] == 'c')
		{
			// Off-mesh connection
			if (m_offMeshConCount < MAX_OFFMESH_CONNECTIONS)
			{
				float* v = &m_offMeshConVerts[m_offMeshConCount * 3 * 2];
				int bidir{}, area{}, flags{}, is_auto{};
				float rad{};

				sscanf_s(row.data() + 1, "%f %f %f  %f %f %f %f %d %d %d %d",
					&v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &rad, &bidir, &area, &flags, &is_auto);

				m_offMeshConRads[m_offMeshConCount] = rad;
				m_offMeshConDirs[m_offMeshConCount] = (unsigned char) bidir;
				m_offMeshConAreas[m_offMeshConCount] = (unsigned char) area;
				m_offMeshConFlags[m_offMeshConCount] = (unsigned short) flags;
				off_mesh_con_auto[m_offMeshConCount] = (bool) is_auto;
				m_offMeshConCount++;
			}
		}
		else if (row[0] == 'v')
		{
			// Convex volumes
			if (m_volumeCount < MAX_VOLUMES)
			{
				ConvexVolume* vol = &m_volumes[m_volumeCount++];
				sscanf_s(row.data() + 1, "%d %d %f %f", &vol->nverts, &vol->area, &vol->hmin, &vol->hmax);

				for (int i = 0; i < vol->nverts; ++i)
				{
					row[0] = '\0';
					src = parseRow(src, srcEnd, row.data(), sizeof(row) / sizeof(char));
					sscanf_s(row.data(), "%f %f %f", &vol->verts[i * 3 + 0], &vol->verts[i * 3 + 1], &vol->verts[i * 3 + 2]);
				}
			}
		}
		else if (row[0] == 's')
		{
			// Settings
			m_buildSettings.emplace();

			sscanf_s(row.data() + 1, "%f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f",
				&m_buildSettings->cellSize,
				&m_buildSettings->cellHeight,
				&m_buildSettings->agentHeight,
				&m_buildSettings->agentRadius,
				&m_buildSettings->agentMaxClimb,
				&m_buildSettings->agentMaxSlope,
				&m_buildSettings->regionMinSize,
				&m_buildSettings->regionMergeSize,
				&m_buildSettings->edgeMaxLen,
				&m_buildSettings->edgeMaxError,
				&m_buildSettings->vertsPerPoly,
				&m_buildSettings->detailSampleDist,
				&m_buildSettings->detailSampleMaxError,
				&m_buildSettings->partitionType,
				&m_buildSettings->navMeshBMin[0],
				&m_buildSettings->navMeshBMin[1],
				&m_buildSettings->navMeshBMin[2],
				&m_buildSettings->navMeshBMax[0],
				&m_buildSettings->navMeshBMax[1],
				&m_buildSettings->navMeshBMax[2],
				&m_buildSettings->tileSize);
		}
		else if (row[0] == 'm')
		{
			// setting (mesh)

			if (m_buildSettings)
			{
				int count{};

				sscanf_s(row.data() + 1, "%d", &count);

				auto& meshs{ m_buildSettings->meshes };

				for (int i = 0; i < count; i++)
				{
					auto& setting_mesh{ meshs.emplace_back() };

					sscanf_s(row.data() + 3, "%f %f %f %f %f %f %f %f %f",
						&setting_mesh.pos[0], &setting_mesh.pos[1], &setting_mesh.pos[2],
						&setting_mesh.scale[0], &setting_mesh.scale[1], &setting_mesh.scale[2],
						&setting_mesh.rotate[0], &setting_mesh.rotate[1], &setting_mesh.rotate[2]);
				}
			}
		}
	}

	// 座標、大きさ、角度を適用
	if (!load_geom_meshes.empty())
	{
		for (size_t i = 0; i < load_geom_meshes.size(); i++)
		{
			auto& dest{ load_geom_meshes[i] };
			const auto& src{ m_buildSettings->meshes[i] };

			dest.pos = src.pos;
			dest.scale = src.scale;
			dest.rotate = src.rotate;
		}

		for (auto& mesh : load_geom_meshes)
		{
			mesh.Update();
		}

		CalcAllMeshBounds();
	}

	return true;
}

void InputGeom::CalcAllMeshBounds()
{
	// 初期化
	all_meshBMax.fill((std::numeric_limits<float>::lowest)());
	all_meshBMin.fill((std::numeric_limits<float>::max)());

	// 計算
	for (const auto& mesh : load_geom_meshes)
	{
		for (size_t i = 0; i < 3u; i++)
		{
			if (all_meshBMax[i] < mesh.m_meshBMax[i])
				all_meshBMax[i] = mesh.m_meshBMax[i];

			if (all_meshBMin[i] > mesh.m_meshBMin[i])
				all_meshBMin[i] = mesh.m_meshBMin[i];
		}
	}

	m_buildSettings->navMeshBMax = all_meshBMax;
	m_buildSettings->navMeshBMin = all_meshBMin;
}

bool InputGeom::Load(rcContext* ctx, const std::string& filepath)
{
	size_t extensionPos = filepath.find_last_of('.');
	if (extensionPos == std::string::npos)
		return false;

	std::string extension = filepath.substr(extensionPos);
	std::transform(extension.begin(), extension.end(), extension.begin(), tolower);

	if (extension == ".gset")
		return LoadGeomSet(ctx, filepath);

	if (extension == ".obj")
		return LoadMesh(ctx, filepath);

	return false;
}

bool InputGeom::SaveGeomSet(const BuildSettings* settings)
{
	if (load_geom_meshes.empty()) return false;

	auto& load_mesh{ load_geom_meshes.front() };

	// Change extension
	// 拡張子を変更します
	std::string filepath = load_mesh.m_mesh->getFileName();
	size_t extPos = filepath.find_last_of('.');

	if (extPos != std::string::npos)
		filepath = filepath.substr(0, extPos);

	filepath += ".gset";

	FILE* fp{};

	if (fopen_s(&fp, filepath.c_str(), "w") != 0)
		return false;

	if (!fp) return false;

	// Store mesh filename.
	// メッシュファイル名を保存します。
	fprintf_s(fp, "f %s\n", load_mesh.m_mesh->getFileName().c_str());

	// Store settings if any
	// 設定があれば保存します
	if (settings)
	{
		fprintf_s(fp,
			"s %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f\n",
			settings->cellSize,
			settings->cellHeight,
			settings->agentHeight,
			settings->agentRadius,
			settings->agentMaxClimb,
			settings->agentMaxSlope,
			settings->regionMinSize,
			settings->regionMergeSize,
			settings->edgeMaxLen,
			settings->edgeMaxError,
			settings->vertsPerPoly,
			settings->detailSampleDist,
			settings->detailSampleMaxError,
			settings->partitionType,
			settings->navMeshBMin[0],
			settings->navMeshBMin[1],
			settings->navMeshBMin[2],
			settings->navMeshBMax[0],
			settings->navMeshBMax[1],
			settings->navMeshBMax[2],
			settings->tileSize);

		const int size{ static_cast<int>(settings->meshes.size()) };

		fprintf_s(fp, "m %d ", size);

		for (auto& mesh : settings->meshes)
		{
			fprintf_s(fp, "%f %f %f %f %f %f %f %f %f\n",
				mesh.pos[0], mesh.pos[1], mesh.pos[2],
				mesh.scale[0], mesh.scale[1], mesh.scale[2],
				mesh.rotate[0], mesh.rotate[1], mesh.rotate[2]);
		}
	}

	// Store off-mesh links.
	// オフメッシュリンクを保存します。
	for (int i = 0; i < m_offMeshConCount; ++i)
	{
		const float* v = &m_offMeshConVerts[i * 3 * 2];
		const float rad = m_offMeshConRads[i];
		const int bidir = m_offMeshConDirs[i];
		const int area = m_offMeshConAreas[i];
		const int flags = m_offMeshConFlags[i];
		const int is_auto = off_mesh_con_auto[i];

		fprintf_s(fp, "c %f %f %f  %f %f %f  %f %d %d %d %d\n",
			v[0], v[1], v[2], v[3], v[4], v[5], rad, bidir, area, flags, is_auto);
	}

	// Convex volumes
	// 凸ボリューム
	for (int i = 0; i < m_volumeCount; ++i)
	{
		ConvexVolume* vol = &m_volumes[i];

		fprintf_s(fp, "v %d %d %f %f\n", vol->nverts, vol->area, vol->hmin, vol->hmax);

		for (int j = 0; j < vol->nverts; ++j)
			fprintf_s(fp, "%f %f %f\n", vol->verts[j * 3 + 0], vol->verts[j * 3 + 1], vol->verts[j * 3 + 2]);
	}

	fclose(fp);

	return true;
}

// メッシュデータとマウスのレイとの判定
// ray_start : レイの始点、dst：レイの終点
bool InputGeom::RaycastMesh(const std::array<float, 3>& ray_start, const std::array<float, 3>& ray_end,
	RaycastMeshHitInfo* hit_info)
{
	if (load_geom_meshes.empty())	return false;

	std::atomic<float> hit_dis{ 1.f }; // レイの長さを1とした時のメッシュデータとの距離上の交点
	std::atomic_bool is_hit{ false };
	std::atomic<LoadGeomMesh*> is_hit_mesh{};

	For_Each(load_geom_meshes, [&](LoadGeomMesh& load_mesh)
		{
			// メッシュデータの全てを囲む四角（以降スラブとする）の２交点
			std::array<float, 2> p_min{}, q_max{};

			load_mesh.is_selected = false;

			// スラブとレイとの判定と交点を求める
			{
				std::array<float, 3> ray_vec{ ray_end - ray_start };

				// 始点を０で終点を１とした時、スラブとの交点のレイ上の位置（最小値：最大値）
				float btmin{}, btmax{};

				// Prune is_hit ray.
				// スラブとレイとの判定
				if (!isectSegAABB(ray_start, ray_end, load_mesh.m_meshBMin, load_mesh.m_meshBMax, btmin, btmax))
					return;

				// スラブとレイの２交点（最大値、最小値）を求める
				p_min[0] = ray_start[0] + (ray_vec[0]) * btmin;
				p_min[1] = ray_start[2] + (ray_vec[2]) * btmin;
				q_max[0] = ray_start[0] + (ray_vec[0]) * btmax;
				q_max[1] = ray_start[2] + (ray_vec[2]) * btmax;
			}

			std::array<int, 512> cid{};
			// 入力セグメントとオーバーラップするチャンクインデックスを返す
			const int ncid
			{ rcGetChunksOverlappingSegment(&(*load_mesh.m_chunkyMesh), p_min, q_max, cid.data(), 512) };

			if (!ncid) return;

			const auto& verts{ load_mesh.m_mesh->getVerts() };

			// メッシュデータとレイの判定
#if false
			std::vector<float> distance_array(ncid, -1.f);

			Transform_N(cid, ncid, distance_array, [&](const int id)
				{
					const rcChunkyTriMeshNode& node{ load_mesh.m_chunkyMesh->nodes[id] };
					const int* tris{ &load_mesh.m_chunkyMesh->tris[node.i * 3] };
					const int ntris{ node.n };

					float shortest{ (std::numeric_limits<float>::max)() };
					bool hit{};

					for (int j = 0; j < ntris * 3; j += 3)
					{
						float dis{ 1 };

						// レイとポリゴンとの判定
						if (intersectSegmentTriangle(ray_start, ray_end,
							&verts[tris[j] * 3],
							&verts[tris[j + 1] * 3],
							&verts[tris[j + 2] * 3], dis))
						{
							// 終点の距離より短いなら
							if (dis < shortest)
							{
								shortest = dis;  // 交点上の距離を代入
							}

							hit = true;
						}
					}

					return (hit ? shortest : -1);
				}, exec::par);

			auto&& itr{ Min_Element(distance_array, exec::par) };

			hit_dis = *itr;
			is_hit = (hit_dis != -1.f);
#elif false
			for (int id = 0; id < ncid; id++)
			{
				const rcChunkyTriMeshNode& node{ load_mesh.m_chunkyMesh->nodes[id] };
				const int* tris{ &load_mesh.m_chunkyMesh->tris[node.i * 3] };
				const int ntris{ node.n };

				for (int j = 0; j < ntris * 3; j += 3)
				{
					float dis{ 1 };

					// レイとポリゴンとの判定
					if (intersectSegmentTriangle(ray_start, ray_end,
						&verts[tris[j] * 3],
						&verts[tris[j + 1] * 3],
						&verts[tris[j + 2] * 3], dis))
					{
						// 終点の距離より短いなら
						if (dis < hit_dis)
						{
							hit_dis = dis;  // 交点上の距離を代入
						}

						is_hit = true;  // 当たっている
					}
				}
			}
#else
			For_Each_N(cid, ncid, [&](const int id)
				{
					const rcChunkyTriMeshNode& node{ load_mesh.m_chunkyMesh->nodes[id] };
					const int* tris{ &load_mesh.m_chunkyMesh->tris[node.i * 3] };
					const int ntris{ node.n };

					for (int j = 0; j < ntris * 3; j += 3)
					{
						float dis{ 1 };

						// レイとポリゴンとの判定
						if (intersectSegmentTriangle(ray_start, ray_end,
							&verts[tris[j] * 3],
							&verts[tris[j + 1] * 3],
							&verts[tris[j + 2] * 3], dis))
						{
							// 終点の距離より短いなら
							if (dis < hit_dis)
							{
								hit_dis = dis;  // 交点上の距離を代入
								is_hit_mesh = &load_mesh;
							}

							is_hit = true;  // 当たっている
						}
					}
				}, exec::par);
#endif
		}, exec::par);

	if (!is_hit || !is_hit_mesh)	return false;

	is_hit_mesh.load()->is_selected = true;

	if (hit_info)
	{
		// レイのベクトルを求める
		hit_info->vec = ray_end - ray_start;

		// 実際の交点を計算
		hit_info->pos = ray_start + (hit_info->vec * hit_dis);

		// レイの始点から衝突地点までの距離を計算
		hit_info->dis = rcVdist(ray_start, hit_info->pos);
	}

	return true;
}

// メッシュデータとマウスのレイとの判定
// ray_start : レイの始点、dst：レイの終点
bool InputGeom::RaycastMesh(const std::array<float, 3>& ray_start, const std::array<float, 3>& ray_end,
	RaycastMeshHitInfo* hit_info) const
{
	if (load_geom_meshes.empty())	return false;

	std::atomic<float> hit_dis{ 1.f }; // レイの長さを1とした時のメッシュデータとの距離上の交点
	std::atomic_bool is_hit{ false };

	For_Each(load_geom_meshes, [&](const LoadGeomMesh& load_mesh)
		{
			// メッシュデータの全てを囲む四角（以降スラブとする）の２交点
			std::array<float, 2> p_min{}, q_max{};

			// スラブとレイとの判定と交点を求める
			{
				std::array<float, 3> ray_vec{ ray_end - ray_start };

				// 始点を０で終点を１とした時、スラブとの交点のレイ上の位置（最小値：最大値）
				float btmin{}, btmax{};

				// Prune is_hit ray.
				// スラブとレイとの判定
				if (!isectSegAABB(ray_start, ray_end, load_mesh.m_meshBMin, load_mesh.m_meshBMax, btmin, btmax))
					return;

				// スラブとレイの２交点（最大値、最小値）を求める
				p_min[0] = ray_start[0] + (ray_vec[0]) * btmin;
				p_min[1] = ray_start[2] + (ray_vec[2]) * btmin;
				q_max[0] = ray_start[0] + (ray_vec[0]) * btmax;
				q_max[1] = ray_start[2] + (ray_vec[2]) * btmax;
			}

			std::array<int, 512> cid{};
			// 入力セグメントとオーバーラップするチャンクインデックスを返す
			const int ncid
			{ rcGetChunksOverlappingSegment(&(*load_mesh.m_chunkyMesh), p_min, q_max, cid.data(), 512) };

			if (!ncid) return;

			const auto& verts{ load_mesh.m_mesh->getVerts() };

			// メッシュデータとレイの判定
#if false
			std::vector<float> distance_array(ncid, -1.f);

			Transform_N(cid, ncid, distance_array, [&](const int id)
				{
					const rcChunkyTriMeshNode& node{ load_mesh.m_chunkyMesh->nodes[id] };
					const int* tris{ &load_mesh.m_chunkyMesh->tris[node.i * 3] };
					const int ntris{ node.n };

					float shortest{ (std::numeric_limits<float>::max)() };
					bool hit{};

					for (int j = 0; j < ntris * 3; j += 3)
					{
						float dis{ 1 };

						// レイとポリゴンとの判定
						if (intersectSegmentTriangle(ray_start, ray_end,
							&verts[tris[j] * 3],
							&verts[tris[j + 1] * 3],
							&verts[tris[j + 2] * 3], dis))
						{
							// 終点の距離より短いなら
							if (dis < shortest)
							{
								shortest = dis;  // 交点上の距離を代入
							}

							hit = true;
						}
					}

					return (hit ? shortest : -1);
				}, exec::par);

			auto&& itr{ Min_Element(distance_array, exec::par) };

			hit_dis = *itr;
			is_hit = (hit_dis != -1.f);
#elif false
			for (int id = 0; id < ncid; id++)
			{
				const rcChunkyTriMeshNode& node{ load_mesh.m_chunkyMesh->nodes[id] };
				const int* tris{ &load_mesh.m_chunkyMesh->tris[node.i * 3] };
				const int ntris{ node.n };

				for (int j = 0; j < ntris * 3; j += 3)
				{
					float dis{ 1 };

					// レイとポリゴンとの判定
					if (intersectSegmentTriangle(ray_start, ray_end,
						&verts[tris[j] * 3],
						&verts[tris[j + 1] * 3],
						&verts[tris[j + 2] * 3], dis))
					{
						// 終点の距離より短いなら
						if (dis < hit_dis)
						{
							hit_dis = dis;  // 交点上の距離を代入
						}

						is_hit = true;  // 当たっている
					}
				}
			}
#else
			For_Each_N(cid, ncid, [&](const int id)
				{
					const rcChunkyTriMeshNode& node{ load_mesh.m_chunkyMesh->nodes[id] };
					const int* tris{ &load_mesh.m_chunkyMesh->tris[node.i * 3] };
					const int ntris{ node.n };

					for (int j = 0; j < ntris * 3; j += 3)
					{
						float dis{ 1 };

						// レイとポリゴンとの判定
						if (intersectSegmentTriangle(ray_start, ray_end,
							&verts[tris[j] * 3],
							&verts[tris[j + 1] * 3],
							&verts[tris[j + 2] * 3], dis))
						{
							// 終点の距離より短いなら
							if (dis < hit_dis)
							{
								hit_dis = dis;  // 交点上の距離を代入
							}

							is_hit = true;  // 当たっている
						}
					}
				}, exec::par);
#endif
		}, exec::par);

	if (!is_hit)	return false;

	if (hit_info)
	{
		// レイのベクトルを求める
		hit_info->vec = ray_end - ray_start;

		// 実際の交点を計算
		hit_info->pos = ray_start + (hit_info->vec * hit_dis);

		// レイの始点から衝突地点までの距離を計算
		hit_info->dis = rcVdist(ray_start, hit_info->pos);
	}

	return true;
}

std::deque<InputGeom::LoadGeomMesh>::iterator InputGeom::EraseSelectLoadGeomMesh() noexcept
{
	const size_t size{ load_geom_meshes.size() };

	auto itr{ Erase_Remove_If(
		load_geom_meshes, [](const LoadGeomMesh& mesh) { return mesh.is_selected; }, exec::par) };

	// 削除した場合は再計算
	if (size != load_geom_meshes.size())
		CalcAllMeshBounds();

	return (itr);
}

int InputGeom::addOffMeshConnection(const float* spos, const float* epos, const float rad,
	unsigned char bidir, unsigned char area, unsigned short flags, const bool is_auto_build)
{
	if (m_offMeshConCount >= MAX_OFFMESH_CONNECTIONS) return -1;
	float* v = &m_offMeshConVerts[m_offMeshConCount * 3 * 2];
	m_offMeshConRads[m_offMeshConCount] = rad;
	m_offMeshConDirs[m_offMeshConCount] = bidir;
	m_offMeshConAreas[m_offMeshConCount] = area;
	m_offMeshConFlags[m_offMeshConCount] = flags;
	m_offMeshConId[m_offMeshConCount] = 1000 + m_offMeshConCount;
	off_mesh_con_auto[m_offMeshConCount] = is_auto_build;
	rcVcopy(&v[0], spos);
	rcVcopy(&v[3], epos);
	m_offMeshConCount++;

	return m_offMeshConCount;
}

void InputGeom::deleteOffMeshConnection(int i)
{
	m_offMeshConCount--;

	float* src = &m_offMeshConVerts[m_offMeshConCount * 3 * 2];
	float* dst = &m_offMeshConVerts[i * 3 * 2];

	rcVcopy(&dst[0], &src[0]);
	rcVcopy(&dst[3], &src[3]);

	m_offMeshConRads[i] = m_offMeshConRads[m_offMeshConCount];
	m_offMeshConDirs[i] = m_offMeshConDirs[m_offMeshConCount];
	m_offMeshConAreas[i] = m_offMeshConAreas[m_offMeshConCount];
	m_offMeshConFlags[i] = m_offMeshConFlags[m_offMeshConCount];
	off_mesh_con_auto[i] = false;
}

void InputGeom::ClearOffMeshConnection()
{
	for (size_t i = 0; i < m_offMeshConCount; i++)
	{
		deleteOffMeshConnection(i);
	}
}

void InputGeom::ClearAutoBuildOffMeshConnection()
{
	for (size_t i = 0; i < m_offMeshConCount; i++)
	{
		if (off_mesh_con_auto[i])	continue;

		deleteOffMeshConnection(i);
	}
}

void InputGeom::drawOffMeshConnections(duDebugDraw* dd, bool hilight)
{
	constexpr unsigned int conColor = duRGBA(192, 0, 128, 192);
	constexpr unsigned int baseColor = duRGBA(0, 0, 0, 64);

	dd->depthMask(false);

	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < m_offMeshConCount; ++i)
	{
		float* v = &m_offMeshConVerts[i * 3 * 2];

		dd->vertex(v[0], v[1], v[2], baseColor);
		dd->vertex(v[0], v[1] + 0.2f, v[2], baseColor);

		dd->vertex(v[3], v[4], v[5], baseColor);
		dd->vertex(v[3], v[4] + 0.2f, v[5], baseColor);

		duAppendCircle(dd, v[0], v[1] + 0.1f, v[2], m_offMeshConRads[i], baseColor);
		duAppendCircle(dd, v[3], v[4] + 0.1f, v[5], m_offMeshConRads[i], baseColor);

		if (hilight)
		{
			duAppendArc(dd, v[0], v[1], v[2], v[3], v[4], v[5], 0.25f,
				(m_offMeshConDirs[i] & 1) ? 0.6f : 0.0f, 0.6f, conColor);
		}
	}
	dd->end();

	dd->depthMask(true);
}

void InputGeom::addConvexVolume(const float* verts, const int nverts,
	const float minh, const float maxh, unsigned char area)
{
	if (m_volumeCount >= MAX_VOLUMES) return;

	ConvexVolume* vol = &m_volumes[m_volumeCount++];

	memset(vol, 0, sizeof(ConvexVolume));
	memcpy(&vol->verts, verts, sizeof(float) * 3 * nverts);

	vol->hmin = minh;
	vol->hmax = maxh;
	vol->nverts = nverts;
	vol->area = area;
}

void InputGeom::deleteConvexVolume(int i)
{
	m_volumeCount--;
	m_volumes[i] = m_volumes[m_volumeCount];
}

void InputGeom::drawConvexVolumes(struct duDebugDraw* dd, bool /*hilight*/)
{
	dd->depthMask(false);

	dd->begin(DU_DRAW_TRIS);
	for (int i = 0; i < m_volumeCount; ++i)
	{
		const ConvexVolume* vol = &m_volumes[i];
		unsigned int col = duTransCol(dd->areaToCol(vol->area), 32);
		for (int j = 0, k = vol->nverts - 1; j < vol->nverts; k = j++)
		{
			const float* va = &vol->verts[k * 3];
			const float* vb = &vol->verts[j * 3];

			dd->vertex(vol->verts[0], vol->hmax, vol->verts[2], col);
			dd->vertex(vb[0], vol->hmax, vb[2], col);
			dd->vertex(va[0], vol->hmax, va[2], col);

			dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
			dd->vertex(va[0], vol->hmax, va[2], col);
			dd->vertex(vb[0], vol->hmax, vb[2], col);

			dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
			dd->vertex(vb[0], vol->hmax, vb[2], col);
			dd->vertex(vb[0], vol->hmin, vb[2], duDarkenCol(col));
		}
	}
	dd->end();

	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < m_volumeCount; ++i)
	{
		const ConvexVolume* vol = &m_volumes[i];
		unsigned int col = duTransCol(dd->areaToCol(vol->area), 220);
		for (int j = 0, k = vol->nverts - 1; j < vol->nverts; k = j++)
		{
			const float* va = &vol->verts[k * 3];
			const float* vb = &vol->verts[j * 3];
			dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
			dd->vertex(vb[0], vol->hmin, vb[2], duDarkenCol(col));
			dd->vertex(va[0], vol->hmax, va[2], col);
			dd->vertex(vb[0], vol->hmax, vb[2], col);
			dd->vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
			dd->vertex(va[0], vol->hmax, va[2], col);
		}
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 3.0f);
	for (int i = 0; i < m_volumeCount; ++i)
	{
		const ConvexVolume* vol = &m_volumes[i];
		unsigned int col = duDarkenCol(duTransCol(dd->areaToCol(vol->area), 220));
		for (int j = 0; j < vol->nverts; ++j)
		{
			dd->vertex(vol->verts[j * 3 + 0], vol->verts[j * 3 + 1] + 0.1f, vol->verts[j * 3 + 2], col);
			dd->vertex(vol->verts[j * 3 + 0], vol->hmin, vol->verts[j * 3 + 2], col);
			dd->vertex(vol->verts[j * 3 + 0], vol->hmax, vol->verts[j * 3 + 2], col);
		}
	}
	dd->end();

	dd->depthMask(true);
}

void InputGeom::LoadGeomMesh::Update()
{
	auto verts{ m_mesh->getVerts() };

	// 更新
	m_mesh->MoveVerts(pos, rotate, scale);
	//m_chunkyMesh->MoveNodes(pos, rotate, scale);

	m_chunkyMesh.emplace();

	if (!rcCreateChunkyTriMesh(m_mesh->getVerts(), m_mesh->getTris(), m_mesh->getTriCount(), 256, &(*m_chunkyMesh)))
	{
		return;
	}

	// 再計算
	rcCalcBounds(m_mesh->getVerts().data(), m_mesh->getVertCount(), m_meshBMin.data(), m_meshBMax.data());
}
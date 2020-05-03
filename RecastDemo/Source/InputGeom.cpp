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
#include "Recast.h"
#include "InputGeom.h"
#include "ChunkyTriMesh.h"
#include "MeshLoaderObj.h"
#include "DebugDraw.h"
#include "RecastDebugDraw.h"
#include "DetourNavMesh.h"
#include "Sample.h"

namespace
{
	bool intersectSegmentTriangle(const float* sp, const float* sq,
		const float* a, const float* b, const float* c,
		float& t)
	{
		float v, w;
		float ab[3], ac[3], qp[3], ap[3], norm[3], e[3];
		rcVsub(ab, b, a);
		rcVsub(ac, c, a);
		rcVsub(qp, sp, sq);

		// Compute triangle normal. Can be precalculated or cached if
		// intersecting multiple segments against the same triangle
		// 三角形の法線を計算します。 同じ三角形に対して複数のセグメントを交差させる場合、事前計算またはキャッシュできます
		rcVcross(norm, ab, ac);

		// Compute denominator d. If d <= 0, segment is parallel to or points
		// away from triangle, so exit early
		// 分母dを計算します。 d <= 0の場合、セグメントは三角形に平行か、三角形から遠ざかるので、早く終了します
		float d = rcVdot(qp, norm);
		if (d <= 0.0f) return false;

		// Compute intersection t value of pq with plane of triangle. A ray
		// intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
		// dividing by d until intersection has been found to pierce triangle
		// 三角形の平面とのpqの交差t値を計算します。
		// 0 <= tの場合、光線は交差します。 0 <= t <= 1の場合、セグメントは交差します。
		// 交差点が三角形を突き抜けていることがわかるまで、dによる除算を遅らせます。
		rcVsub(ap, sp, a);
		t = rcVdot(ap, norm);
		if (t < 0.0f) return false;
		if (t > d) return false; // For segment; exclude this code line for a ray test セグメント; 光線テストのためにこのコード行を除外します

		// Compute barycentric coordinate components and test if within bounds
		// 重心座標成分を計算し、範囲内かどうかをテストします。
		rcVcross(e, qp, ap);
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
	bool isectSegAABB(const float* sp, const float* sq,
		const float* amin, const float* amax,
		float& tmin, float& tmax)
	{
		constexpr float EPS = 1e-6f;

		float d[3];
		d[0] = sq[0] - sp[0];
		d[1] = sq[1] - sp[1];
		d[2] = sq[2] - sp[2];

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
	: m_offMeshConCount{}, m_volumeCount{},	m_all_meshBMin{}, m_all_meshBMax{}
{}

bool InputGeom::loadMesh(rcContext* ctx, const std::string& filepath)
{
	if (!load_geom_meshes.empty())
	{
		load_geom_meshes.clear();
	}

	m_offMeshConCount = 0;
	m_volumeCount = 0;

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

	rcCalcBounds(load_meshes.m_mesh->getVerts(), load_meshes.m_mesh->getVertCount(),
		load_meshes.m_meshBMin.data(), load_meshes.m_meshBMax.data());

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

	return true;
}

bool InputGeom::loadGeomSet(rcContext* ctx, const std::string& filepath)
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
				if (!loadMesh(ctx, name))
				{
					return false;
				}
			}
		}
		else if (row[0] == 'c')
		{
			// Off-mesh connection
			if (m_offMeshConCount < MAX_OFFMESH_CONNECTIONS)
			{
				float* v = &m_offMeshConVerts[m_offMeshConCount * 3 * 2];
				int bidir{}, area{}, flags{};
				float rad{};

				sscanf_s(row.data() + 1, "%f %f %f  %f %f %f %f %d %d %d",
					&v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &rad, &bidir, &area, &flags);

				m_offMeshConRads[m_offMeshConCount] = rad;
				m_offMeshConDirs[m_offMeshConCount] = (unsigned char)bidir;
				m_offMeshConAreas[m_offMeshConCount] = (unsigned char)area;
				m_offMeshConFlags[m_offMeshConCount] = (unsigned short)flags;
				m_offMeshConCount++;
			}
		}
		else if (row[0] == 'v')
		{
			// Convex volumes
			if (m_volumeCount < MAX_VOLUMES)
			{
				ConvexVolume* vol = &m_volumes[m_volumeCount++];
				sscanf_s(row.data() + 1, "%d %c %c %f %f", &vol->nverts, &vol->areaMod.m_value, sizeof(vol->areaMod.m_value), &vol->areaMod.m_mask, sizeof(vol->areaMod.m_mask), &vol->hmin, &vol->hmax);

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
			m_buildSettings = {};

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
	}

	return true;
}

bool InputGeom::load(rcContext* ctx, const std::string& filepath)
{
	size_t extensionPos = filepath.find_last_of('.');
	if (extensionPos == std::string::npos)
		return false;

	std::string extension = filepath.substr(extensionPos);
	std::transform(extension.begin(), extension.end(), extension.begin(), tolower);

	if (extension == ".gset")
		return loadGeomSet(ctx, filepath);

	if (extension == ".obj")
		return loadMesh(ctx, filepath);

	return false;
}

bool InputGeom::saveGeomSet(const BuildSettings* settings)
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

		fprintf_s(fp, "c %f %f %f  %f %f %f  %f %d %d %d\n",
			v[0], v[1], v[2], v[3], v[4], v[5], rad, bidir, area, flags);
	}

	// Convex volumes
	// 凸ボリューム
	for (int i = 0; i < m_volumeCount; ++i)
	{
		ConvexVolume* vol = &m_volumes[i];

		fprintf_s(fp, "v %d %c %c %f %f\n", vol->nverts, vol->areaMod.m_value, vol->areaMod.m_mask, vol->hmin, vol->hmax);

		for (int j = 0; j < vol->nverts; ++j)
			fprintf_s(fp, "%f %f %f\n", vol->verts[j * 3 + 0], vol->verts[j * 3 + 1], vol->verts[j * 3 + 2]);
	}

	fclose(fp);

	return true;
}

// メッシュデータとマウスのレイとの判定
// src : レイの始点、dst：レイの終点、レイの長さを1とした時のメッシュデータとの距離上の交点
bool InputGeom::raycastMesh(float* src, float* dst, float& tmin)
{
	if (load_geom_meshes.empty())	return false;

	auto& load_mesh{ load_geom_meshes.front() };

	// メッシュデータの全てを囲む四角（以降スラブとする）の２交点
	float p_min[2]{}, q_max[2]{};

	// スラブとレイとの判定と交点を求める
	{
		float ray_vec[3]{};

		// 始点を０で終点を１とした時、スラブとの交点のレイ上の位置（最小値：最大値）
		float btmin{ 0.f }, btmax{ 0.f };

		rcVsub(ray_vec, dst, src);  // マウスのレイのベクトルを求める

		// Prune hit ray.
		// スラブとレイとの判定
		if (!isectSegAABB(src, dst, load_mesh.m_meshBMin.data(), load_mesh.m_meshBMax.data(), btmin, btmax))
			return false;

		// スラブとレイの２交点（最大値、最小値）を求める
		p_min[0] = src[0] + (ray_vec[0]) * btmin;
		p_min[1] = src[2] + (ray_vec[2]) * btmin;
		q_max[0] = src[0] + (ray_vec[0]) * btmax;
		q_max[1] = src[2] + (ray_vec[2]) * btmax;
	}

	int cid[512]{};
	// 入力セグメントとオーバーラップするチャンクインデックスを返す
	const int ncid = rcGetChunksOverlappingSegment(&(*load_mesh.m_chunkyMesh), p_min, q_max, cid, 512);

	if (!ncid) return false;

	tmin = 1.f;

	bool hit = false;
	const float* verts = load_mesh.m_mesh->getVerts();

	// メッシュデータとレイの判定
	for (int i = 0; i < ncid; ++i)
	{
		const rcChunkyTriMeshNode& node = load_mesh.m_chunkyMesh->nodes[cid[i]];
		const int* tris = &load_mesh.m_chunkyMesh->tris[node.i * 3];
		const int ntris = node.n;

		for (int j = 0; j < ntris * 3; j += 3)
		{
			float t = 1;

			// レイとポリゴンとの判定
			if (intersectSegmentTriangle(src, dst,
				&verts[tris[j] * 3],
				&verts[tris[j + 1] * 3],
				&verts[tris[j + 2] * 3], t))
			{
				// 終点の距離より短いなら
				if (t < tmin)
					tmin = t;  // 交点上の距離を代入

				hit = true;  // 当たっている
			}
		}
	}

	return hit;
}

void InputGeom::addOffMeshConnection(const float* spos, const float* epos, const float rad,
	unsigned char bidir, unsigned char area, unsigned short flags)
{
	if (m_offMeshConCount >= MAX_OFFMESH_CONNECTIONS) return;
	float* v = &m_offMeshConVerts[m_offMeshConCount * 3 * 2];
	m_offMeshConRads[m_offMeshConCount] = rad;
	m_offMeshConDirs[m_offMeshConCount] = bidir;
	m_offMeshConAreas[m_offMeshConCount] = area;
	m_offMeshConFlags[m_offMeshConCount] = flags;
	m_offMeshConId[m_offMeshConCount] = 1000 + m_offMeshConCount;
	rcVcopy(&v[0], spos);
	rcVcopy(&v[3], epos);
	m_offMeshConCount++;
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
	const float minh, const float maxh, rcAreaModification areaMod)
{
	if (m_volumeCount >= MAX_VOLUMES) return;

	ConvexVolume* vol = &m_volumes[m_volumeCount++];

	memset(vol, 0, sizeof(ConvexVolume));
	memcpy(&vol->verts, verts, sizeof(float) * 3 * nverts);

	vol->hmin = minh;
	vol->hmax = maxh;
	vol->nverts = nverts;
	vol->areaMod = areaMod;
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
		unsigned int col = duTransCol(dd->areaToCol(vol->areaMod.getMaskedValue()), 32);
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
		unsigned int col = duTransCol(dd->areaToCol(vol->areaMod.getMaskedValue()), 220);
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
		unsigned int col = duDarkenCol(duTransCol(dd->areaToCol(vol->areaMod.getMaskedValue()), 220));
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
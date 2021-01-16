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
#include <string>
#include <cfloat>
#include "AlgorithmHelper.h"
#include "SDL.h"
#include "SDL_opengl.h"
#ifdef __APPLE__
#	include <OpenGL/glu>
#else
#	include <GL/glu.h>
#endif
#include "imgui.h"
#include "OffMeshConnectionTool.h"
#include "InputGeom.h"
#include "Sample.h"
#include "Recast.h"
#include "RecastDebugDraw.h"
#include "DetourDebugDraw.h"
#include "DetourNavMesh.h"

#ifdef WIN32
#	define snprintf _snprintf
#endif

namespace
{
	static inline float distancePtLine2d(const float* pt, const float* p, const float* q)
	{
		float pqx = q[0] - p[0];
		float pqz = q[2] - p[2];
		float dx = pt[0] - p[0];
		float dz = pt[2] - p[2];
		float d = pqx * pqx + pqz * pqz;
		float t = pqx * dx + pqz * dz;

		if (d != 0) t /= d;

		dx = p[0] + t * pqx - pt[0];
		dz = p[2] + t * pqz - pt[2];

		return dx * dx + dz * dz;
	}

	using namespace RcMath;
	namespace exec = std::execution;
}

OffMeshConnectionTool::OffMeshConnectionTool() :
	sample(0),
	hit_pos_set(0),
	m_bidir(true),
	m_oldFlags(0),
	links_arrow(true),
	horizontal_distance(1.5f),
	vertical_distance(5.f),
	divistion_distance(0.5f)
{}

OffMeshConnectionTool::~OffMeshConnectionTool()
{
	if (sample)
	{
		sample->setNavMeshDrawFlags(m_oldFlags);
	}
}

void OffMeshConnectionTool::init(Sample* smp)
{
	edges.clear();

	if (this->sample != smp)
	{
		this->sample = smp;
		m_oldFlags = this->sample->getNavMeshDrawFlags();
		this->sample->setNavMeshDrawFlags(m_oldFlags & ~DU_DRAWNAVMESH_OFFMESHCONS);
	}
}

void OffMeshConnectionTool::reset()
{
	hit_pos_set = false;
	edges.clear();
}

void OffMeshConnectionTool::handleMenu()
{
	// 一方通行
	if (imguiCheck("One Way", !m_bidir))
		m_bidir = false;
	// 双方向通行
	if (imguiCheck("Bidirectional", m_bidir))
		m_bidir = true;

	imguiSeparator();

	imguiValue("Auto OffMeshLink");

	imguiSlider("horizontal_dis", &horizontal_distance, 0.1f, 25.f, 0.1f);
	imguiSlider("vertical_dis", &vertical_distance, 0.1f, 25.f, 0.1f);
	imguiSlider("divistion_dis", &divistion_distance, 0.1f, 10.f, 0.1f);

	// 自動生成
	if (imguiButton("Link Build"))
		AutoLinksBuild();
	// 削除
	if (imguiButton("Link Clear"))
		edges.clear();

	// 生成完了
	if (!edges.empty())
	{
		// 始点から終点への矢印
		if (imguiCheck("Links Arrow", links_arrow))
			links_arrow ^= true;
	}
}

void OffMeshConnectionTool::handleClickDown(const float* /*s*/, const float* p, bool shift)
{
	if (!sample) return;

	auto& geom = sample->getInputGeom();

	if (!geom) return;

	if (shift)
	{
		// Delete
		// Find nearest link end-point
		// 最も近いリンクのエンドポイントを見つける
		float nearestDist = (std::numeric_limits<float>::max)();
		int nearestIndex = -1;

		const auto& verts = geom->getOffMeshConnectionVerts();

		for (int i = 0; i < geom->getOffMeshConnectionCount() * 2; ++i)
		{
			float d = rcVdistSqr(p, &verts[i * 3]);

			if (d < nearestDist)
			{
				nearestDist = d;
				nearestIndex = i / 2; // Each link has two vertices. // 各リンクには2つの頂点があります。
			}
		}

		// If end point close enough, delete it.
		// エンドポイントが十分に近い場合は、削除します。
		if (nearestIndex != -1 &&
			sqrtf(nearestDist) < sample->getAgentRadius())
		{
			geom->deleteOffMeshConnection(nearestIndex);
		}
	}
	else
	{
		// Create
		if (!hit_pos_set)
		{
			rcVcopy(hit_pos, p);
			hit_pos_set = true;
		}
		else
		{
			constexpr unsigned char area = SAMPLE_POLYAREA_JUMP;
			constexpr unsigned short flags = SAMPLE_POLYFLAGS_JUMP;
			geom->addOffMeshConnection(hit_pos, p, sample->getAgentRadius(), m_bidir ? 1 : 0, area, flags);
			hit_pos_set = false;
		}
	}
}

void OffMeshConnectionTool::handleToggle()
{
}

void OffMeshConnectionTool::handleStep()
{
}

void OffMeshConnectionTool::handleUpdate(const float /*dt*/)
{
}

void OffMeshConnectionTool::handleRender()
{
	duDebugDraw& dd = sample->getDebugDraw();
	const float s = sample->getAgentRadius();

	if (hit_pos_set)
		duDebugDrawCross(&dd, hit_pos[0], hit_pos[1] + 0.1f, hit_pos[2], s, duRGBA(0, 0, 0, 128), 2.0f);

	auto& geom = sample->getInputGeom();
	if (geom)
		geom->drawOffMeshConnections(&dd, true);

	if (!edges.empty())
	{
		dd.begin(DU_DRAW_LINES, 2.0f);
		for (const auto& edge : edges)
		{
			constexpr float LineAdjY{ 2.f };

			auto start{ edge.start }, end{ edge.end };

			// 始点・終点
			{
				constexpr UINT32 EdgeColor{ duRGBA(255, 0, 0, 200) };

				// 高くする
				start[1] += LineAdjY;
				end[1] += LineAdjY;

				// 始点
				dd.vertex(edge.start.data(), EdgeColor);
				dd.vertex(start.data(), EdgeColor);

				// 終点
				dd.vertex(edge.end.data(), EdgeColor);
				dd.vertex(end.data(), EdgeColor);
			}

			// 分割点
			for (auto& point : edge.points)
			{
				constexpr float PointAdjY{ LineAdjY * 0.75f };
				constexpr UINT32 PointColor{ duRGBA(255, 125, 125, 200) };

				start = end = point;

				end[1] += PointAdjY;

				dd.vertex(start.data(), PointColor);
				dd.vertex(end.data(), PointColor);
			}

			// 矢印
			if (links_arrow)
			{
				constexpr float ArrowAdjY{ LineAdjY * 0.375f };
				constexpr UINT32 ArrowColor{ duRGBA(255, 255, 0, 150) };

				start = edge.start;
				end = edge.end;

				// 見やする為に少し短くする
				auto vec{ end - start };
				const float len{ rcVdist(start, end) };
				const float short_len{ len / 15.f };

				rcVnormalize(&vec);

				// 前後方向に少し位置をずらす
				start = start + (vec * short_len);
				end = end + (vec * -short_len);

				// 高さを再調整
				start[1] += ArrowAdjY;
				end[1] += ArrowAdjY;

				duAppendArrow(&dd,
					start[0], start[1], start[2],
					end[0], end[1], end[2],
					0.0f, 0.4f, ArrowColor);
			}
		}
		dd.end();

		//dd.begin(DU_DRAW_LINES, 2.0f);
		//for (const auto& edge : edges)
		//{
		//	constexpr UINT32 EdgeColor{ duRGBA(255, 0, 0, 200) };

		//	auto start{ edge.start }, end{ edge.end };

		//	start[1] += 1.f;
		//	end[1] += 1.f;

		//}
		//dd.end();
	}
}

void OffMeshConnectionTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;

	// Draw start and end point labels
	if (hit_pos_set && gluProject((GLdouble)hit_pos[0], (GLdouble)hit_pos[1], (GLdouble)hit_pos[2],
		model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int)x, (int)(y - 25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0, 0, 0, 220));
	}

	// Tool help
	const int h = view[3];
	if (!hit_pos_set)
	{
		imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Create new connection.  SHIFT+LMB: Delete existing connection, click close to start or end point.", imguiRGBA(255, 255, 255, 192));
	}
	else
	{
		imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Set connection end point and finish.", imguiRGBA(255, 255, 255, 192));
	}
}

void OffMeshConnectionTool::AutoLinksBuild()
{
	// ナビメッシュのエッジの作成
	CalcNavMeshEdges();

	// エッジを分割
	CalcEdgeDivision();

	// エッジの有効性を確認


	// 終点を計算


	// 始点・終点を決定

}

void OffMeshConnectionTool::CalcNavMeshEdges()
{
	// 初期化し忘れ
	if (!sample)	return;

	// 初期化
	edges.clear();

	const dtNavMesh* mesh{ sample->getNavMesh() };

	// ナビメッシュを生成していない
	if (!mesh)	return;

	for (int idx = 0; idx < mesh->getMaxTiles(); ++idx)
	{
		const dtMeshTile* tile = mesh->getTile(idx);

		// 無効なタイル
		if (!tile->header) continue;

		constexpr float thr{ 0.01f * 0.01f };

		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			const dtPoly* p = &tile->polys[i];

			// メッシュ外のOffMeshLink接続
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) continue;

			const dtPolyDetail* pd = &tile->detailMeshes[i];

			for (int j = 0, nj = (int) p->vertCount; j < nj; ++j)
			{
				// 参照先が無い
				if (p->neis[j] != 0) continue;

				const float* v0 = &tile->verts[p->verts[j] * 3];
				const float* v1 = &tile->verts[p->verts[(j + 1) % nj] * 3];

				// Draw detail mesh edges which align with the actual poly edge.
				// This is really slow.
				// 実際のポリエッジと整列する詳細メッシュエッジを描画します。
				// これは本当に遅いです。
				for (int k = 0; k < pd->triCount; ++k)
				{
					const unsigned char* t = &tile->detailTris[(pd->triBase + k) * 4];
					const float* tv[3];
					for (int m = 0; m < 3; ++m)
					{
						if (t[m] < p->vertCount)
							tv[m] = &tile->verts[p->verts[t[m]] * 3];
						else
							tv[m] = &tile->detailVerts[(pd->vertBase + (t[m] - p->vertCount)) * 3];
					}
					for (int m = 0, n = 2; m < 3; n = m++)
					{
						if ((dtGetDetailTriEdgeFlags(t[3], n) & DT_DETAIL_EDGE_BOUNDARY) == 0)
							continue;

						if (distancePtLine2d(tv[n], v0, v1) < thr &&
							distancePtLine2d(tv[m], v0, v1) < thr)
						{
							auto& edge{ edges.emplace_back() };

							rcVcopy(edge.start.data(), tv[n]);
							rcVcopy(edge.end.data(), tv[m]);
						}
					}
				}
			}
		}
	}
}

void OffMeshConnectionTool::CalcEdgeDivision()
{
	For_Each(edges, [this](NavMeshEdge& edge)
		{
			auto vec{ edge.end - edge.start };
			const float len{ rcVdist(edge.start, edge.end) };

			rcVnormalize(&vec);

			edge.points.clear();
			edge.points.reserve(static_cast<size_t>(len / divistion_distance) + 1u);

			// 分割距離分徐々に追加
			for (float dist = divistion_distance; dist < len; dist += divistion_distance)
			{
				edge.points.emplace_back(edge.start + (vec * dist));
			}
		}, exec::par);
}

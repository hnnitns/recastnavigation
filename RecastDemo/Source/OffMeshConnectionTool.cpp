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
#include "Sample_TempObstacles.h"

#include "OtherFiles\\AlgorithmHelper.hpp"
#include "OtherFiles\\XMFLOAT_Helper.hpp"
#include "OtherFiles\\DirectXMathAlias.hpp"
#include "OtherFiles\\XMFLOAT_Math.hpp"

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

	DIRECTX_MATH_ALIAS;
}

OffMeshConnectionTool::OffMeshConnectionTool() :
	sample(nullptr),
	obstacle_sample(nullptr),
	hit_pos_set(0),
	m_bidir(true),
	m_oldFlags(0),
	draw_links_arrow(true),
	draw_tentative_link(true),
	draw_horizontal_point(true),
	draw_edge_point(true),
	draw_division_point(true),
	draw_end_point(true),
	draw_navmesh_nearest_point(true),
	draw_error_dis(false),
	horizontal_dis(5.f),
	vertical_dis(7.5f),
	divistion_dis(0.5f),
	link_end_error_dis(0.2f),
	orthognal_error_dis(0.5f),
	link_equal_error_dis(0.25f),
	hit_pos()
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

		// 構築したサンプルが障害物系のものかどうかを確認
		obstacle_sample = dynamic_cast<Sample_TempObstacles*>(sample);
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

	imguiSeparatorLine();

	imguiValue("Auto OffMeshLink");

	imguiSlider("horizontal_dis", &horizontal_dis, 0.1f, 25.f, 0.1f);
	imguiSlider("vertical_dis", &vertical_dis, 0.1f, 25.f, 0.1f);
	imguiSlider("divistion_dis", &divistion_dis, 0.1f, 10.f, 0.1f);
	imguiSlider("link_end_error", &link_end_error_dis, 0.1f, 5.f, 0.1f);
	imguiSlider("max_orth_error", &orthognal_error_dis, 0.1f, 5.f, 0.1f);
	imguiSlider("link_equal_error_dis", &link_equal_error_dis, 0.01f, 1.f, 0.01f);

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
		if (imguiCheck("Links Arrow", draw_links_arrow))
			draw_links_arrow ^= true;

		// エッジの始点終点
		if (imguiCheck("Edge Point", draw_edge_point))
			draw_edge_point ^= true;

		// エッジの分割点
		if (imguiCheck("Division Point", draw_division_point))
			draw_division_point ^= true;

		// 仮リンク
		if (imguiCheck("Tentative Link", draw_tentative_link))
			draw_tentative_link ^= true;

		// 水平上のポイント
		if (imguiCheck("Horizontal Point", draw_horizontal_point))
			draw_horizontal_point ^= true;

		// 終点
		if (imguiCheck("End Point", draw_end_point))
			draw_end_point ^= true;

		// ナビメッシュに最も近いポイント
		if (imguiCheck("Navmesh Nearest Point", draw_navmesh_nearest_point))
			draw_navmesh_nearest_point ^= true;

		// 誤差範囲
		if (imguiCheck("Error Dis", draw_error_dis))
			draw_error_dis ^= true;

		// 生成時間
		{
			std::string text{ "build_time: " };

			text += std::to_string(auto_build_time_ms);

			text += "ms";

			imguiValue(text.data());
		}
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
{}

void OffMeshConnectionTool::handleStep()
{}

void OffMeshConnectionTool::handleUpdate(const float /*dt*/)
{}

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
		// ライン
		dd.begin(DU_DRAW_LINES, 2.0f);
		for (const auto& edge : edges)
		{
			constexpr float LineAdjY{ 2.f };

			auto start{ edge.start }, end{ edge.end };

			// 始点・終点
			if (draw_edge_point)
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
			if (draw_division_point)
			{
				for (const auto& point : edge.points)
				{
					constexpr float PointAdjY{ LineAdjY * 0.75f };
					constexpr UINT32 PointColor{ duRGBA(0, 255, 0, 200) };

					start = end = point;

					end[1] += PointAdjY;

					dd.vertex(start.data(), PointColor);
					dd.vertex(end.data(), PointColor);
				}
			}

			// 矢印描画
			if (draw_links_arrow)
			{
				// 始点から終点へのベクトル
				{
					constexpr float ArrowAdjY{ LineAdjY * 0.375f }, AdjDistance{ 1.f };
					constexpr UINT32 ArrowColor{ duRGBA(255, 255, 0, 200) };

					// 内側へずらす
					{
						const auto&& orth_vec{ edge.orthogonal_vec * -AdjDistance };

						start = edge.start + orth_vec;
						end = edge.end + orth_vec;
					}

					// 見やする為に少し短くする
					auto vec{ end - start };
					const float len{ rcVdist(start, end) };
					const float short_len{ len / 7.5f };

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

				// 水平・垂直ベクトル
				{
					constexpr UINT32 ArrowColor{ duRGBA(255, 0, 255, 200) };

					start = edge.start;
					end = edge.end;

					auto vec{ end - start };
					const float len{ rcVdist(start, end) };

					rcVnormalize(&vec);

					const auto&& middle_pos{ start + (vec * (len * 0.5f)) };

					// 水平ベクトル
					{
						start = middle_pos;
						end = middle_pos + (edge.orthogonal_vec * horizontal_dis);

						duAppendArrow(&dd,
							start[0], start[1], start[2],
							end[0], end[1], end[2],
							0.0f, 0.4f, ArrowColor);
					}

					// 垂直ベクトル
					{
						constexpr Point Down{ 0.f, -1.f, 0.f };

						start = end;
						end = start + (Down * vertical_dis);

						duAppendArrow(&dd,
							start[0], start[1], start[2],
							end[0], end[1], end[2],
							0.0f, 0.4f, ArrowColor);
					}
				}
			}

			// 許容範囲
			if (draw_error_dis)
			{
				constexpr auto Color{ duRGBA(255, 255, 255, 150) };

				for (auto& link : edge.links)
				{
					// 地形の当たり座標とナビメッシュの当たり座標間の許容範囲
					duAppendCircle(&dd, link.nearest_pos[0], link.nearest_pos[1], link.nearest_pos[2],
						link_end_error_dis, Color);

					// 垂直ベクトルで構築不可になる許容範囲
					duAppendCircle(&dd, link.start[0], link.start[1], link.start[2],
						orthognal_error_dis, Color);

					// 他の仮リンクとの重なりを認識する許容範囲
					duAppendCircle(&dd, link.start[0], link.start[1] + 2.f, link.start[2],
						link_equal_error_dis, Color);

					duAppendCircle(&dd, link.end[0], link.end[1] + 2.f, link.start[2],
						link_equal_error_dis, Color);
				}
			}

			// 仮リンク
			if (draw_tentative_link)
			{
				constexpr UINT32 ArrowColor{ duRGBA(255, 255, 255, 200) };

				for (const auto& link : edge.links)
				{
					start = link.start;
					end = link.end;

					duAppendArc(&dd,
						start[0], start[1], start[2],
						end[0], end[1], end[2],
						0.25f, link.is_bidir ? 0.6f : 0.f, 0.6f, ArrowColor);
				}
			}
		}
		dd.end();

		// ポイント
		dd.begin(DU_DRAW_POINTS, 5.f);
		for (const auto& edge : edges)
		{
			auto start{ edge.start };

			// Link
			for (const auto& link : edge.links)
			{
				// 終点
				if (draw_end_point)
				{
					constexpr UINT32 PointColor{ duRGBA(0, 0, 0, 200) };

					start = link.end;

					dd.vertex(start.data(), PointColor);
				}

				// ナビメッシュの最接近ポイント
				if (draw_navmesh_nearest_point)
				{
					constexpr UINT32 PointColor{ duRGBA(50, 50, 255, 200) };

					start = link.nearest_pos;

					dd.vertex(start.data(), PointColor);
				}

				// 水平上のポイント
				if (draw_horizontal_point)
				{
					constexpr UINT32 PointColor{ duRGBA(165,42,42, 200) };

					start = link.horizontal_pos;

					dd.vertex(start.data(), PointColor);
				}
			}
		}
		dd.end();
	}
}

void OffMeshConnectionTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;

	// Draw start and end point labels
	if (hit_pos_set && gluProject((GLdouble) hit_pos[0], (GLdouble) hit_pos[1], (GLdouble) hit_pos[2],
		model, proj, view, &x, &y, &z))
	{
		imguiDrawText((int) x, (int) (y - 25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0, 0, 0, 220));
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
	auto* ctx{ sample->GetContext() };

	// タイマー計測開始
	ctx->resetTimers();
	auto_build_time_ms = 0.f;
	ctx->startTimer(RC_TIMER_TEMP);

	// ナビメッシュのエッジの作成
	CalcNavMeshEdges();

	// エッジを分割
	CalcEdgeDivision();

	// 終点を計算し仮リンクを作成
	CalcTentativeLink();

	// 決定した仮リンクに調整・修正
	CheckTentativeLink();

	// OffMesh Linkを作成する
	BuildLink();

	// タイマー計測終了
	ctx->stopTimer(RC_TIMER_TEMP);
	auto_build_time_ms = ctx->getAccumulatedTime(RC_TIMER_TEMP) / 1000.f;
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

							// エッジの始点と終点を決定
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
			edge.points.reserve(static_cast<size_t>(len / divistion_dis) + 1u);

			// 分割距離分徐々に追加
			for (float dist = divistion_dis; dist < len; dist += divistion_dis)
			{
				edge.points.emplace_back(edge.start + (vec * dist));
			}
		}, exec::par);
}

void OffMeshConnectionTool::CalcTentativeLink()
{
	For_Each(edges, [this](NavMeshEdge& edge)
		{
			std::mutex mt;

			VF3 vec3{ ToXMFLOAT(edge.end - edge.start) };
			const VF2&& vec{ vec3.x, vec3.z }; // Y軸は無視
			const VF2 orth_vec2d{ VectorNormalize(VectorOrthogonal(vec)) }; // XZ平面上の垂直ベクトル

			edge.orthogonal_vec = { orth_vec2d.x, 0.f/*Y軸は無視*/, orth_vec2d.y };

			edge.links.clear();
			edge.links.reserve(edge.points.size());

			// 各分割点から終点を計算する
			For_Each(edge.points,
				[&edge, orth_vec2d, &mt, this](const Point& point)
				{
					const auto& geom{ sample->getInputGeom() };
					const float agent_radius{ sample->getAgentRadius() };
					const auto* navmesh_query{ sample->getNavMeshQuery() };

					InputGeom::RaycastMeshHitInfo hit_info{};

					// 水平上のポイント（中継ポイント）
					auto horizontal_point
					{ point + (edge.orthogonal_vec * horizontal_dis) };

					// 水平上のポイントまでに地形が存在するか？
#if false
					if (geom->RaycastMesh(point, horizontal_point, &hit_info))	return;
#else
					if (geom->RaycastMesh(point, horizontal_point, &hit_info))
					{
						// 当たった座標にポイントを決定する
						horizontal_point = hit_info.pos + (edge.orthogonal_vec * agent_radius);

						const float diameter{ agent_radius * 2.f };
						const float distance{ diameter + orthognal_error_dis }; // 最低でもエージェントの直径は必要

						// 分割点に近すぎる
						if (rcVdistSqr(horizontal_point, point) <= distance * distance)	return;

						const auto&& horizon_vec{ horizontal_point - point };
						const VF2&& horizon_vec2d{ horizon_vec.front(), horizon_vec.back() };

						// 分割点よりも後方に存在する（念の為）
						if (VectorDot(orth_vec2d, horizon_vec2d) <= 0.f)	return;
					}
#endif

					auto inv_horizona_vec{ horizontal_point - point };

					rcVnormalize(&inv_horizona_vec);

					constexpr Point HalfExtents{ 0.5f, 0.5f, 0.5f };

					// 分割点直下付近から水平上のポイントまで検索
					for (float dis = agent_radius * agent_radius; dis < horizontal_dis - agent_radius * agent_radius;
						dis += HalfExtents.front())
					{
						constexpr Point Down{ 0.f, -1.f, 0.f };

						// 直下ベクトルの始点と終点
						const Point&& start{ point + (inv_horizona_vec * dis) },
							&& end{ start + (Down * vertical_dis) };

						// 垂直上に地形が存在しない
						if (!geom->RaycastMesh(start, end, &hit_info))	continue;

						const dtQueryFilter filter;
						dtPolyRef ref{};
						Point nearest_pos{};
						constexpr Point Zero{};

						// ナビメッシュとの判定
						const auto status{ navmesh_query->findNearestPoly(hit_info.pos.data(),
							HalfExtents.data(), &filter, &ref, nearest_pos.data()) };

						// ナビメッシュ状に垂直上のポイントが存在しない
						if (dtStatusFailed(status) || ref == 0 || nearest_pos == Zero)	continue;

						const float error_dis_sqr{ link_end_error_dis * link_end_error_dis };

						// 地形のヒットポイントとナビメッシュのヒットポイントの距離が遠い
						if (rcVdistSqr(nearest_pos, hit_info.pos) > error_dis_sqr) continue;

						std::lock_guard<std::mutex> lg{ mt }; // 競合阻止

						// OffMesh Linkの始点と終点を確定
						edge.links.emplace_back(point, hit_info.pos, nearest_pos, horizontal_point);

						break; // 構築終了
					}
				}, exec::par);
		}, exec::par);
}

void OffMeshConnectionTool::CheckTentativeLink()
{
	// 仮リンクの被りをチェックする
#if true
	const size_t length{ edges.size() };

	std::vector<size_t> indeces(length);

	NormalAlgorithm::Iota(indeces);

	For_Each(indeces, [this, length](const size_t i)
		{
			std::mutex mt;
			auto& base{ edges[i] };

			// 終了
			if (i + 1u > length)	return;

			std::for_each(exec::par, std::next(edges.begin(), +i + 1u), edges.end(),
				[&base, this, &mt](NavMeshEdge& other)
				{
					For_Each(base.links, [this, &other, &mt](NavMeshEdge::Link& base_link)
						{
							// 既に削除が決定している
							if (base_link.is_delete)	return;

							const float error_sqr{ link_equal_error_dis * 2.f };
							// 一致する他の仮リンクを検索
							auto&& itr{ Find_If(other.links,
								[&base_link, error_sqr](NavMeshEdge::Link& oth_link)
								{
									// 既に削除が決定している
									if (oth_link.is_delete)	return false;

									// 一定範囲内に始点と終点が交差している
									return (rcVdistSqr(base_link.start, oth_link.end) <= error_sqr &&
										rcVdistSqr(base_link.end, oth_link.start) <= error_sqr);
								}, exec::par) };

							// 発見したので削除予定に追加
							if (itr != other.links.end())
							{
								std::lock_guard<std::mutex> lg{ mt };

								base_link.is_delete = true; // 削除
								itr->is_bidir = true; // 削除しない側を双方向通行に設定
							}
						}, exec::par);
				});
		}, exec::par);
#else
	for (size_t i = 0, length = edges.size(); i < length; i++)
	{
		std::mutex mt;
		auto& base{ edges[i] };

		// 終了
		if (i + 1u > length)	break;

		std::for_each(exec::par, std::next(edges.begin(), +i + 1u), edges.end(),
			[&base, this, &mt](NavMeshEdge& other)
			{
				For_Each(base.links, [this, &other, &mt](NavMeshEdge::Link& base_link)
					{
						// 既に削除が決定している
						if (base_link.is_delete)	return;

						const float error_sqr{ link_equal_error_dis * 2.f };
						// 一致する他の仮リンクを検索
						auto&& itr{ Find_If(other.links,
							[&base_link, error_sqr](NavMeshEdge::Link& oth_link)
							{
								// 既に削除が決定している
								if (oth_link.is_delete)	return false;

								// 一定範囲内に始点と終点が交差している
								return (rcVdistSqr(base_link.start, oth_link.end) <= error_sqr &&
									rcVdistSqr(base_link.end, oth_link.start) <= error_sqr);
							}, exec::par) };

						// 発見したので削除予定に追加
						if (itr != other.links.end())
						{
							std::lock_guard<std::mutex> lg{ mt };

							base_link.is_delete = true; // 削除
							itr->is_bidir = true; // 削除しない側を双方向通行に設定
						}
					}, exec::par);
			});
	}
#endif

	// 余分な仮リンクを削除する
	For_Each(edges, [](NavMeshEdge& edge)
		{
			Erase_Remove_If(edge.links, [](NavMeshEdge::Link& link) { return link.is_delete; }, exec::par);
		}, exec::par);
}

void OffMeshConnectionTool::BuildLink()
{}
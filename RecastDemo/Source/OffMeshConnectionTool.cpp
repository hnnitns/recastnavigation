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
	using namespace RcMath;
	namespace exec = std::execution;

	DIRECTX_MATH_ALIAS;

	inline float distancePtLine2d(const float* pt, const float* p, const float* q)
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

	bool IsectSegAABB(const float* sp, const float* sq,
		const std::array<float, 3>& amin, const std::array<float, 3>& amax,
		float& tmin, float& tmax)
	{
		constexpr float EPS = 1e-6f;

		std::array<float, 3> d{};
		rcVsub(d.data(), sq, sp);

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

	inline constexpr bool IsPointInsideAABB(const std::array<float, 3>& point,
		const std::array<float, 3>& box_min, const std::array<float, 3>& box_max)
	{
		return (point[0] <= box_max[0] && point[0] >= box_min[0]) &&
			(point[1] <= box_max[1] && point[1] >= box_min[1]) &&
			(point[2] <= box_max[2] && point[2] >= box_min[2]);
	}
}

OffMeshConnectionTool::OffMeshConnectionTool() :
	sample(nullptr), obstacle_sample(nullptr), hit_pos_set(0), m_bidir(true), m_oldFlags(0),
	draw_links_arrow(true), draw_tentative_link(true), draw_horizontal_point(true),
	draw_edge_point(true), draw_division_point(true), draw_end_point(true),
	draw_navmesh_nearest_point(true), draw_error_dis(false), is_buildable_height_limit(true),
	horizontal_dis(5.f), vertical_dis(7.5f), divistion_dis(0.5f), link_end_error_dis(0.2f),
	orthognal_error_dis(0.5f), link_equal_error_dis(0.25f), climbable_height(0.5f),
	min_buildable_height(0.5f), hit_pos(), box_descent(0.f), box_height(5.f), is_not_build_area()
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

	imguiSlider("climbable_height", &climbable_height, 0.1f, 7.5f, 0.1f);
	imguiSlider("horizontal_dis", &horizontal_dis, 0.1f, 25.f, 0.1f);
	imguiSlider("vertical_dis", &vertical_dis, 0.1f, 25.f, 0.1f);
	imguiSlider("divistion_dis", &divistion_dis, 0.1f, 10.f, 0.1f);
	imguiSlider("link_end_error", &link_end_error_dis, 0.1f, 5.f, 0.1f);
	imguiSlider("max_orth_error", &orthognal_error_dis, 0.1f, 5.f, 0.1f);
	imguiSlider("link_equal_error_dis", &link_equal_error_dis, 0.01f, 1.f, 0.01f);

	// 「横跳びリンク」に制限を設けるか？
	{
		if (imguiCheck("Buildable Height Limit", is_buildable_height_limit))
			is_buildable_height_limit ^= true;

		if (is_buildable_height_limit)
		{
			imguiSlider("min_buildable_height", &min_buildable_height, 0.1f, 10.f, 0.1f);
		}
	}

	// 自動生成
	if (imguiButton("Link Build"))
		AutoLinksBuild();
	// 削除
	if (imguiButton("Link Clear"))
		edges.clear();

	// 自動構築不可エリア設定
	{
		if (imguiCheck("Not Build Area", is_not_build_area))
			is_not_build_area ^= true;

		if (is_not_build_area)
		{
			imguiSlider("box_descent", &box_descent, 0.1f, 10.f, 0.1f);
			imguiSlider("box_height", &link_equal_error_dis, 0.1f, 10.f, 0.1f);

			if (imguiButton("All Clear"))
			{
				not_build_areas.clear();
			}
		}
	}

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

		{
			std::string text{ "build_time: " };

			// 生成時間
			{
				text += std::to_string(auto_build_time_ms);
				text += "ms";
				imguiValue(text.data());
			}

			// 自動生成されたエッジの総数
			{
				text = "auto Edge size: ";
				text += std::to_string(edges.size());
				imguiValue(text.data());
			}

			// 自動生成されたリンクの総数
			{
				const size_t accumulate{ NormalAlgorithm::Accumulate(edges, 0u,
					[](const size_t i, const NavMeshEdge& edge) { return i + edge.links.size(); }) };

				text = "auto Link size: ";
				text += std::to_string(accumulate);
				imguiValue(text.data());
			}
		}
	}

}

void OffMeshConnectionTool::handleClickDown(const float* s, const float* p, bool shift)
{
	if (!sample) return;

	auto& geom = sample->getInputGeom();

	if (!geom) return;

	if (shift)
	{
		// 構築不可エリア
		if (is_not_build_area)
		{
			// Delete
			int nearestIndex{ -1 };
			float nearest_dist{ (std::numeric_limits<float>::max)() };

			for (size_t i = 0; i < not_build_areas.size(); i++)
			{
				const auto& area{ not_build_areas[i] };
				float tmax{}, tmin{};

				if (IsectSegAABB(s, p, area.aabb_min, area.aabb_max, tmin, tmax) && nearest_dist > tmin)
				{
					nearestIndex = i;
					nearest_dist = tmin;
				}
			}

			// エンドポイントが十分に近い場合は削除
			if (nearestIndex != -1)
			{
				not_build_areas.erase(not_build_areas.begin() + nearestIndex);
			}
		}
		// OffMeshConnection
		else
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
	}
	else
	{
		// 構築不可エリア
		if (is_not_build_area)
		{
			// 追加する
			if (not_build_areas.empty())	not_build_areas.emplace_back();

			if (auto& area{ not_build_areas.back() };
				area.nhit_points == NotBuildArea::MaxHitPoint)
			{
				// 最後のポイントをクリックすると、形状が確定する
				if (rcVdistSqr(p, area.hit_points[area.nhit_points - 1].data()) < rcSqr(0.2f))
				{
					not_build_areas.emplace_back();
					area.is_built = true;
				}
				else
				{
					// 既存ポイントを更新
					rcVcopy(area.hit_points.back().data(), p);

					// 最大・最小値を取得
					{
						const auto& hit_points{ area.hit_points };

						for (size_t i = 0; i < area.aabb_max.size(); i++)
						{
							area.aabb_max[i] = rcMax(hit_points.front()[i], hit_points.back()[i]);
							area.aabb_min[i] = rcMin(hit_points.front()[i], hit_points.back()[i]);
						}

						area.aabb_max[1] += box_height;
						area.aabb_min[1] -= box_descent;
					}

					// 各頂点を計算
					{
						auto& vertex{ area.aabb_vertex };
						const Point& min{ area.aabb_min }, max{ area.aabb_max };
						const VF3&& min_vs{ min.front(), min[1], min.back() },
							max_vs{ max.front(), max[1], max.back() };

						// 下部
						vertex[0] = area.aabb_min;                    // 左下
						vertex[1] = { min_vs.x, min_vs.y, max_vs.z }; // 左上
						vertex[2] = { max_vs.x, min_vs.y, max_vs.z }; // 右上
						vertex[3] = { max_vs.x, min_vs.y, min_vs.z }; // 右下

						// 上部
						vertex[4] = { min_vs.x, max_vs.y, min_vs.z }; // 左下
						vertex[5] = { min_vs.x, max_vs.y, max_vs.z }; // 左上
						vertex[6] = area.aabb_max;                    // 右上
						vertex[7] = { max_vs.x, max_vs.y, min_vs.z }; // 右下
					}
				}
			}
			else
			{
				// 新しいポイントを追加
				rcVcopy(area.hit_points[area.nhit_points].data(), p);

				if (area.nhit_points < NotBuildArea::MaxHitPoint)
					area.nhit_points++;
			}
		}
		// OffMeshConnection
		else
		{
			// Create
			if (!hit_pos_set)
			{
				rcVcopy(hit_pos.data(), p);
				hit_pos_set = true;
			}
			else
			{
				constexpr unsigned char area = SAMPLE_POLYAREA_JUMP;
				constexpr unsigned short flags = SAMPLE_POLYFLAGS_JUMP;
				geom->addOffMeshConnection(hit_pos.data(), p, sample->getAgentRadius(), m_bidir ? 1 : 0, area, flags);
				hit_pos_set = false;
			}
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

	dd.depthMask(false);

	if (hit_pos_set)
		duDebugDrawCross(&dd, hit_pos[0], hit_pos[1] + 0.1f, hit_pos[2], s, duRGBA(0, 0, 0, 128), 2.0f);

	auto& geom = sample->getInputGeom();
	if (geom)
		geom->drawOffMeshConnections(&dd, true);

	// オフメッシュリンクの自動生成関係
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

					start = end = point.base_point;

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
					start = edge.start;
					end = edge.end;

					auto vec{ end - start };
					const float len{ rcVdist(start, end) };

					rcVnormalize(&vec);

					const auto&& middle_pos{ start + (vec * (len * 0.5f)) },
						&& horizontal_pos{ middle_pos + (edge.orthogonal_vec * horizontal_dis) };

					// 垂直ベクトル
					{
						constexpr UINT32 ArrowColor{ duRGBA(255, 0, 255, 200) };

						start = middle_pos;
						end = horizontal_pos;

						duAppendArrow(&dd,
							start[0], start[1], start[2],
							end[0], end[1], end[2],
							0.0f, 0.4f, ArrowColor);
					}

					// 直下ベクトル
					{
						constexpr UINT32 ArrowColor{ duRGBA(255, 0, 100, 200) };
						constexpr Point Down{ 0.f, -1.f, 0.f };

						start = horizontal_pos;
						end = start + (Down * vertical_dis);

						duAppendArrow(&dd,
							start[0], start[1], start[2],
							end[0], end[1], end[2],
							0.0f, 0.4f, ArrowColor);
					}

					// 直上ベクトル
					{
						constexpr UINT32 ArrowColor{ duRGBA(100, 0, 255, 200) };
						constexpr Point Up{ 0.f, 1.f, 0.f };

						start = horizontal_pos;
						end = start + (Up * climbable_height);

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
				constexpr auto Color{ duRGBA(0, 0, 0, 150) };

				for (auto& link : edge.links)
				{
					// 地形の当たり座標とナビメッシュの当たり座標間の許容範囲
					duAppendCircle(&dd, link.nearest_pos[0], link.nearest_pos[1], link.nearest_pos[2],
						link_end_error_dis, Color);

					// 垂直ベクトルで構築不可になる許容範囲
					{
						start = link.start;
						end = start + (edge.orthogonal_vec * orthognal_error_dis);
						dd.vertex(start.data(), Color);
						dd.vertex(end.data(), Color);
					}

					// 他の仮リンクとの重なりを認識する許容範囲
					duAppendCircle(&dd, link.start[0], link.start[1] + 0.5f, link.start[2],
						link_equal_error_dis, Color);

					duAppendCircle(&dd, link.end[0], link.end[1] + 0.5f, link.start[2],
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
						0.35f, link.is_bidir ? 0.6f : 0.f, 0.6f, ArrowColor);
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

	// 構築不可エリア
	for (auto& area : not_build_areas)
	{
		if (area.nhit_points == 0)	continue;

		dd.begin(DU_DRAW_POINTS, 4.0f);
		for (int i = 0; i < area.hit_points.size(); ++i)
		{
			const Point& point{ area.hit_points[i] };
			unsigned int col = duRGBA(255, 255, 255, 255);

			if (i == NotBuildArea::MaxHitPoint - 1 && !area.is_built)
				col = duRGBA(240, 32, 16, 255);

			dd.vertex(point.front(), point[1] + 0.1f, point.back(), col);
		}
		dd.end();

		if (area.nhit_points == NotBuildArea::MaxHitPoint)
		{
			dd.begin(DU_DRAW_LINES, 2.0f);

			static constexpr UINT32 Color{ duRGBA(255, 255, 255, 64) };
			const auto& vertex{ area.aabb_vertex };

			auto VertexFunc
			{ [&dd](const Point& point) { dd.vertex(point.front(), point[1], point.back(), Color); } };

			// X
			VertexFunc(vertex[0]); VertexFunc(vertex[3]);
			VertexFunc(vertex[1]); VertexFunc(vertex[2]);
			VertexFunc(vertex[4]); VertexFunc(vertex[7]);
			VertexFunc(vertex[5]); VertexFunc(vertex[6]);

			// Y
			VertexFunc(vertex[0]); VertexFunc(vertex[4]);
			VertexFunc(vertex[1]); VertexFunc(vertex[5]);
			VertexFunc(vertex[2]); VertexFunc(vertex[6]);
			VertexFunc(vertex[3]); VertexFunc(vertex[7]);

			// Z
			VertexFunc(vertex[0]); VertexFunc(vertex[1]);
			VertexFunc(vertex[3]); VertexFunc(vertex[2]);
			VertexFunc(vertex[4]); VertexFunc(vertex[5]);
			VertexFunc(vertex[7]); VertexFunc(vertex[6]);

			dd.end();
		}
	}

#if false
	dd.begin(DU_DRAW_TRIS);

	for (int i = 0; i < m_volumeCount; ++i)
	{
		const ConvexVolume* vol = &m_volumes[i];
		unsigned int col = duTransCol(dd.areaToCol(vol->area), 32);
		for (int j = 0, k = vol->nverts - 1; j < vol->nverts; k = j++)
		{
			const float* va = &vol->verts[k * 3];
			const float* vb = &vol->verts[j * 3];

			dd.vertex(vol->verts[0], vol->hmax, vol->verts[2], col);
			dd.vertex(vb[0], vol->hmax, vb[2], col);
			dd.vertex(va[0], vol->hmax, va[2], col);

			dd.vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
			dd.vertex(va[0], vol->hmax, va[2], col);
			dd.vertex(vb[0], vol->hmax, vb[2], col);

			dd.vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
			dd.vertex(vb[0], vol->hmax, vb[2], col);
			dd.vertex(vb[0], vol->hmin, vb[2], duDarkenCol(col));
		}
	}

	dd.end();

	dd.begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < m_volumeCount; ++i)
	{
		const ConvexVolume* vol = &m_volumes[i];
		unsigned int col = duTransCol(dd.areaToCol(vol->area), 220);
		for (int j = 0, k = vol->nverts - 1; j < vol->nverts; k = j++)
		{
			const float* va = &vol->verts[k * 3];
			const float* vb = &vol->verts[j * 3];
			dd.vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
			dd.vertex(vb[0], vol->hmin, vb[2], duDarkenCol(col));
			dd.vertex(va[0], vol->hmax, va[2], col);
			dd.vertex(vb[0], vol->hmax, vb[2], col);
			dd.vertex(va[0], vol->hmin, va[2], duDarkenCol(col));
			dd.vertex(va[0], vol->hmax, va[2], col);
		}
	}
	dd.end();

	dd.begin(DU_DRAW_POINTS, 3.0f);
	for (int i = 0; i < m_volumeCount; ++i)
	{
		const ConvexVolume* vol = &m_volumes[i];
		unsigned int col = duDarkenCol(duTransCol(dd.areaToCol(vol->area), 220));
		for (int j = 0; j < vol->nverts; ++j)
		{
			dd.vertex(vol->verts[j * 3 + 0], vol->verts[j * 3 + 1] + 0.1f, vol->verts[j * 3 + 2], col);
			dd.vertex(vol->verts[j * 3 + 0], vol->hmin, vol->verts[j * 3 + 2], col);
			dd.vertex(vol->verts[j * 3 + 0], vol->hmax, vol->verts[j * 3 + 2], col);
		}
	}
	dd.end();
#endif

	dd.depthMask(true);
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
			const float agent_height{ sample->getAgentHeight() };

			rcVnormalize(&vec);

			edge.points.clear();
			edge.points.reserve(static_cast<size_t>(len / divistion_dis) + 1u);

			// 分割距離分徐々に追加
			for (float dist = divistion_dis; dist < len; dist += divistion_dis)
			{
				const Point base{ edge.start + (vec * dist) };

				// 自動構築不可エリアに分割点が存在する
				if (Any_Of(not_build_areas, [&base](const NotBuildArea& area)
					{ return IsPointInsideAABB(base, area.aabb_min, area.aabb_max); }, exec::par))	continue;

				auto& point{ edge.points.emplace_back() };

				point.base_point = base;
				point.height_point = base + Point{ 0.f, climbable_height, 0.f };
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
				[&edge, orth_vec2d, &mt, this](const NavMeshEdge::DivisionPoint& point)
				{
					const auto& geom{ sample->getInputGeom() };
					const float agent_radius{ sample->getAgentRadius() };
					const auto* navmesh_query{ sample->getNavMeshQuery() };

					InputGeom::RaycastMeshHitInfo hit_info{};

					// 水平上のポイント（中継ポイント）
					auto horizontal_point{ point.height_point + (edge.orthogonal_vec * horizontal_dis) };

					// 水平上のポイントまでに地形が存在するか？
					if (geom->RaycastMesh(point.height_point, horizontal_point, &hit_info) ||
						geom->RaycastMesh(horizontal_point, point.height_point, &hit_info))
					{
						// 当たった座標にポイントを決定する
						horizontal_point = hit_info.pos + (edge.orthogonal_vec * agent_radius);

						const float diameter{ agent_radius * 2.f };
						const float distance{ diameter + orthognal_error_dis }; // 最低でもエージェントの直径は必要

						// 分割点に近すぎる
						if (rcVdistSqr(horizontal_point, point.height_point) <= distance * distance)	return;

						const auto&& horizon_vec{ horizontal_point - point.height_point };
						const VF2&& horizon_vec2d{ horizon_vec.front(), horizon_vec.back() };

						// 分割点よりも後方に存在する（念の為）
						if (VectorDot(orth_vec2d, horizon_vec2d) <= 0.f)	return;
					}

					auto inv_horizona_vec{ horizontal_point - point.height_point };

					rcVnormalize(&inv_horizona_vec);

					constexpr Point HalfExtents{ 0.5f, 0.5f, 0.5f };

					// 分割点直下付近から水平上のポイントまで検索
					for (float dis = agent_radius * agent_radius; dis < horizontal_dis - agent_radius * agent_radius;
						dis += HalfExtents.front())
					{
						constexpr Point Down{ 0.f, -1.f, 0.f };

						// 直下ベクトルの始点と終点
						const Point&& start{ point.height_point + (inv_horizona_vec * dis) },
							&& end{ start + (Down * (vertical_dis + climbable_height)) };

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
						edge.links.emplace_back(
							point.base_point, hit_info.pos, nearest_pos, horizontal_point);

						break; // 構築終了
					}
				}, exec::par);
		}, exec::par);
}

void OffMeshConnectionTool::CheckTentativeLink()
{
	// 仮リンクの被りをチェックする
#if true // 全てマルチスレッド化
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
#else // 一部だけ逐次ループ
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

	For_Each(edges, [this](NavMeshEdge& edge)
		{
			For_Each(edge.links, [this](NavMeshEdge::Link& link)
				{
					// 既に削除済み
					if (link.is_delete)	return;

					// 「横跳びリンク」を削除
					if (is_buildable_height_limit)
					{
						const float div_buildable_height{ min_buildable_height / 2.f };

						if (Math::IsBetweenNumber(
							link.end[1] - div_buildable_height, link.end[1] + div_buildable_height,
							link.start[1]))
						{
							link.is_delete = true;
						}
					}

					//// 仮リンク間の障害物をチェックする
					//{
					//	const auto& geom{ sample->getInputGeom() };
					//	const float agent_height{ sample->getAgentHeight() };

					//	InputGeom::RaycastMeshHitInfo hit_info{};

					//	Point start{}, end{};

					//	// 斜めに例を飛ばすなどがある
					//	if (geom->RaycastMesh(start, end, &hit_info))
					//	{
					//		link.is_delete = true;
					//	}
					//}
				}, exec::par);
		}, exec::par);

	// 余分な仮リンクを削除する
	For_Each(edges, [](NavMeshEdge& edge)
		{
			Erase_Remove_If(edge.links, [](NavMeshEdge::Link& link) { return link.is_delete; }, exec::par);
		}, exec::par);
}

void OffMeshConnectionTool::BuildLink()
{}
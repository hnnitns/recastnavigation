#include <basetsd.h>

#include "OffMeshLinkManager.h"
#include "Sample.h"
#include "DetourDebugDraw.h"
#include "Sample_TempObstacles.h"

#include "OtherFiles\\AlgorithmHelper.hpp"
#include "OtherFiles\\XMFLOAT_Helper.hpp"
#include "OtherFiles\\DirectXMathAlias.hpp"
#include "OtherFiles\\XMFLOAT_Math.hpp"
#include "OtherFiles\\Array_Hlper.hpp"

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

OffMeshLinkManager::OffMeshLinkManager()
	: draw_links_arrow(true), draw_tentative_link(true), draw_horizontal_point(true),
	draw_edge_point(true), draw_division_point(true), draw_end_point(true),
	draw_navmesh_nearest_point(true), is_buildable_height_limit(true), draw_all(true)
{}

OffMeshLinkManager::~OffMeshLinkManager()
{
	if (!sample)	return;

	sample->setNavMeshDrawFlags(old_flags);
}

bool OffMeshLinkManager::Init(Sample * smp)
{
	edges.clear();

	if (this->sample != smp)
	{
		this->sample = smp;
		old_flags = this->sample->getNavMeshDrawFlags();
		this->sample->setNavMeshDrawFlags(old_flags & ~DU_DRAWNAVMESH_OFFMESHCONS);

		// 構築したサンプルが障害物系のものかどうかを確認
		obstacle_sample = dynamic_cast<Sample_TempObstacles*>(sample);
	}

	return true;
}

void OffMeshLinkManager::Draw()
{
	duDebugDraw& dd = sample->getDebugDraw();
	const float s = sample->getAgentRadius();

	auto& geom = sample->getInputGeom();
	if (!geom)	return;

	geom->drawOffMeshConnections(&dd, true);

	// オフメッシュリンクの自動生成関係
	if (!edges.empty() && draw_all)
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
	{
		// 設定中
		for (auto& area : not_build_areas)
		{
			if (area.is_built)	continue;

			if (area.nhit_points > 0)
			{
				dd.begin(DU_DRAW_POINTS, 4.0f);
				for (size_t i = 0; i < area.hit_points.size(); ++i)
				{
					const Point& point{ area.hit_points[i] };
					unsigned int col = duRGBA(255, 255, 255, 255);

					if (i == NotBuildArea::MaxHitPoint - 1u && !area.is_built)
						col = duRGBA(240, 32, 16, 255);

					dd.vertex(point.front(), point[1] + 0.1f, point.back(), col);
				}
				dd.end();

				if (area.nhit_points == NotBuildArea::MaxHitPoint - 1)
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
		}

		// 設定完了
		for (auto& area : not_build_areas)
		{
			if (!area.is_built)	continue;

			constexpr UINT32 BaseColor{ duRGBA(255, 50, 50, 150) };
			const auto& vertex{ area.aabb_vertex };

			auto VertexFunc{ [&dd](const Point& point, const UINT32 color)
				{ dd.vertex(point.data(), color); } };

			dd.begin(DU_DRAW_TRIS);
			{
				constexpr UINT32 col{ duTransCol(BaseColor, 32) };

				// 下部
				VertexFunc(vertex[2], col); VertexFunc(vertex[1], col); VertexFunc(vertex[0], col);
				VertexFunc(vertex[0], col); VertexFunc(vertex[3], col); VertexFunc(vertex[2], col);

				// 上部
				VertexFunc(vertex[4], col); VertexFunc(vertex[5], col); VertexFunc(vertex[6], col);
				VertexFunc(vertex[6], col); VertexFunc(vertex[7], col); VertexFunc(vertex[4], col);

				// 左部
				VertexFunc(vertex[1], col); VertexFunc(vertex[5], col); VertexFunc(vertex[4], col);
				VertexFunc(vertex[4], col); VertexFunc(vertex[0], col); VertexFunc(vertex[1], col);

				// 右部
				VertexFunc(vertex[2], col); VertexFunc(vertex[3], col); VertexFunc(vertex[7], col);
				VertexFunc(vertex[7], col); VertexFunc(vertex[6], col); VertexFunc(vertex[2], col);

				// 前部
				VertexFunc(vertex[0], col); VertexFunc(vertex[4], col); VertexFunc(vertex[7], col);
				VertexFunc(vertex[7], col); VertexFunc(vertex[3], col); VertexFunc(vertex[0], col);

				// 後部
				VertexFunc(vertex[1], col); VertexFunc(vertex[2], col); VertexFunc(vertex[6], col);
				VertexFunc(vertex[6], col); VertexFunc(vertex[5], col); VertexFunc(vertex[1], col);
			}
			dd.end();

			dd.begin(DU_DRAW_LINES, 2.0f);
			{
				// 簡易ワイヤーフレーム
				{
					constexpr UINT32 col{ duTransCol(BaseColor, 200) };

					// X
					VertexFunc(vertex[0], col); VertexFunc(vertex[3], col);
					VertexFunc(vertex[1], col); VertexFunc(vertex[2], col);
					VertexFunc(vertex[4], col); VertexFunc(vertex[7], col);
					VertexFunc(vertex[5], col); VertexFunc(vertex[6], col);

					// Y
					VertexFunc(vertex[0], col); VertexFunc(vertex[4], col);
					VertexFunc(vertex[1], col); VertexFunc(vertex[5], col);
					VertexFunc(vertex[2], col); VertexFunc(vertex[6], col);
					VertexFunc(vertex[3], col); VertexFunc(vertex[7], col);

					// Z
					VertexFunc(vertex[0], col); VertexFunc(vertex[1], col);
					VertexFunc(vertex[3], col); VertexFunc(vertex[2], col);
					VertexFunc(vertex[4], col); VertexFunc(vertex[5], col);
					VertexFunc(vertex[7], col); VertexFunc(vertex[6], col);
				}

				// 構築不可エリア
				{
					constexpr UINT32 col{ duRGBA(50, 50, 50, 150) };

					const Point& max{ area.nonbuild_area_max }, min{ area.nonbuild_area_min };
					const float pos_y{ (min[1] + max[1]) / 2.f };
					const Point&& l_top{ min.front(), pos_y, max.back() },
						l_bottom{ min.front(), pos_y, min.back() },
						r_top{ max.front(), pos_y, max.back() },
						r_bottom{ max.front(), pos_y, min.back() };

					VertexFunc(l_bottom, col); VertexFunc(l_top, col);    // 左
					VertexFunc(l_top, col); VertexFunc(r_top, col);       // 上
					VertexFunc(r_top, col); VertexFunc(r_bottom, col);    // 右
					VertexFunc(r_bottom, col); VertexFunc(l_bottom, col); // 下
				}
			}
			dd.end();

			dd.begin(DU_DRAW_POINTS, 3.0f);
			{
				constexpr UINT32 col{ duTransCol(BaseColor, 220) };

				for (auto& vtx : vertex)
				{
					VertexFunc(vtx, col);
				}
			}
			dd.end();
		}
	}
}

int OffMeshLinkManager::AddLink(
	const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos, const bool is_bidirectional)
{
	constexpr unsigned char area = SAMPLE_POLYAREA_JUMP;
	constexpr unsigned short flags = SAMPLE_POLYFLAGS_JUMP;

	auto& geom = sample->getInputGeom();

	if (!geom)	return -1;

	return geom->addOffMeshConnection(start_pos.data(), end_pos.data(), sample->getAgentRadius(),
		is_bidirectional ? 1 : 0, area, flags);
}

void OffMeshLinkManager::DeleteLink(const int index)
{
	auto& geom = sample->getInputGeom();

	if (!geom)	return;

	geom->deleteOffMeshConnection(index);
}

bool OffMeshLinkManager::HitLinkPoint(const std::array<float, 3>& point, const int index)
{
	auto& geom = sample->getInputGeom();

	// 不正なインデックス
	if (!geom || index < 0 || index >= geom->getOffMeshConnectionCount())	return false;

	const auto& verts{ geom->getOffMeshConnectionVerts() };

	const float dist{ rcVdistSqr(point.data(), &verts[index * 3]) };

	// エンドポイントが十分に近い場合
	return (::sqrtf(dist) < sample->getAgentRadius());
}

bool OffMeshLinkManager::HitLinksPoint(const std::array<float, 3>& point, int* index)
{
	auto& geom = sample->getInputGeom();

	if (!(geom && index))	return false;

	// 最も近いリンクのエンドポイントを見つける
	float nearestDist{ (std::numeric_limits<float>::max)() };
	int nearestIndex{ -1 };

	const auto& verts{ geom->getOffMeshConnectionVerts() };

	for (int i = 0; i < geom->getOffMeshConnectionCount() * 2; ++i)
	{
		const float d{ rcVdistSqr(point.data(), &verts[i * 3]) };

		if (d < nearestDist)
		{
			nearestDist = d;
			nearestIndex = i / 2; // 各リンクには2つの頂点があります。
		}
	}

	if (nearestIndex == -1)	return false;

	*index = -1;

	// エンドポイントが十分に近い場合
	if (::sqrtf(nearestDist) < sample->getAgentRadius())
	{
		*index = nearestIndex;
		return true;
	}

	return false;
}

int OffMeshLinkManager::GetLinkSize() const noexcept
{
	auto& geom = sample->getInputGeom();

	if (!geom)	return 0;

	return geom->getOffMeshConnectionCount();
}

void OffMeshLinkManager::AutoTentativeLinksBuild()
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

	edge_link_accumulate = NormalAlgorithm::Accumulate(edges, 0u,
		[](const size_t i, const NavMeshEdge& edge) { return i + edge.links.size(); });

	// タイマー計測終了
	ctx->stopTimer(RC_TIMER_TEMP);
	auto_build_time_ms = ctx->getAccumulatedTime(RC_TIMER_TEMP) / 1000.f;
}

void OffMeshLinkManager::ReBuildNavMeshLink()
{
	using Link = NavMeshEdge::Link;

	if (!obstacle_sample || edge_link_accumulate == 0)	return;

	BuildLink();

	// タイル情報
	struct TileInfo final
	{
		Point pos{};
		dtTileRef tile_ref{};
	};

	std::vector<TileInfo> tile_infos(edge_link_accumulate * 2);

	// 各リンクのタイル情報を取得
	{
		auto&& itr{ tile_infos.begin() };

		for (const auto& edge : edges)
		{
			auto GetTileInfoFunc{ [this](const Point& pos)
			{
				TileInfo info;

				// タイル情報を取得
				info.tile_ref = obstacle_sample->GetTileRef(pos.data());
				// 座標を保存
				info.pos = pos;

				return info;
			} };
			const size_t count{ edge.links.size() };

			// 始点
			std::transform(exec::par, edge.links.begin(), edge.links.end(), itr,
				[&GetTileInfoFunc](const Link& link) { return GetTileInfoFunc(link.start); });

			itr += count;

			// 終点
			std::transform(exec::par, edge.links.begin(), edge.links.end(), itr,
				[&GetTileInfoFunc](const Link& link) { return GetTileInfoFunc(link.end); });

			itr += count;
		}
	}

	// 重複削除の為に並び替える
	Sort(tile_infos, [](const TileInfo& left, const TileInfo& right)
		{ return (left.tile_ref < right.tile_ref); }, exec::par);

	// 重複を検出
	auto&& itr{ Unique(tile_infos, [](const TileInfo& left, const TileInfo& right)
		{ return (left.tile_ref == right.tile_ref); }, exec::par) };

	// 実際に削除
	tile_infos.erase(itr, tile_infos.end());

	// ナビメッシュが存在しない場所
	if (tile_infos.empty())	return;

	for (const auto& info : tile_infos)
	{
		// その地点のタイルを再構築する
		obstacle_sample->buildTile(info.pos.data());
	}
}

void OffMeshLinkManager::ClearBuiltAutoLink()
{
	auto& geom{ sample->getInputGeom() };

	if (!geom)	return;

	// 以前に自動構築されたリンクを全削除する
	geom->ClearAutoBuildOffMeshConnection();
}

void OffMeshLinkManager::CalcNavMeshEdges()
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

void OffMeshLinkManager::CalcEdgeDivision()
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
					{ return area.is_built && IsPointInsideAABB(base, area.aabb_min, area.aabb_max); },
					exec::par))	continue;

				auto& point{ edge.points.emplace_back() };
				const auto& geom{ sample->getInputGeom() };

				point.base_point = base;
				point.height_point = base + Point{ 0.f, climbable_height, 0.f };

				InputGeom::RaycastMeshHitInfo hit_info_top{}, hit_info_bottom{};

				// 高さの調整をする際にレイで確認する
				const bool hit_mesh_top
				{ geom->RaycastMesh(point.height_point, point.base_point, &hit_info_top) },
					hit_mesh_bottom{ geom->RaycastMesh(point.base_point, point.height_point, &hit_info_bottom) };

				constexpr float AdjYDis{ 0.25f };

				// ヒットポイントに位置を調整する
				if (hit_mesh_bottom)
				{
					point.height_point = hit_info_bottom.pos;
					point.height_point[1] -= AdjYDis;
				}
				else if (hit_mesh_top)
				{
					point.height_point = hit_info_top.pos;
					point.height_point[1] -= AdjYDis;
				}
			}
		}, exec::par);
}

void OffMeshLinkManager::CalcTentativeLink()
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

					const bool hit_mesh
					{ geom->RaycastMesh(point.height_point, horizontal_point, &hit_info) ||
						geom->RaycastMesh(horizontal_point, point.height_point, &hit_info) };

					// 水平上のポイントまでに地形が存在するか？
					if (hit_mesh)
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
					for (float dis = agent_radius * agent_radius;
						dis < horizontal_dis - agent_radius * agent_radius; dis += HalfExtents.front())
					{
						// 地形との判定
						{
							constexpr Point Down{ 0.f, -1.f, 0.f };

							// 直下ベクトルの始点と終点
							const Point&& start{ point.height_point + (inv_horizona_vec * dis) },
								&& end{ start + (Down * (vertical_dis + climbable_height)) };

							// 垂直上に地形が存在しない
							if (!geom->RaycastMesh(start, end, &hit_info))	continue;
						}

						// 自動構築不可エリアとの判定
						{
							auto NotBuildAreaFunc{ [&hit_info](const NotBuildArea& area)
							{
									// 構築されていて、ヒットポイントがエリア外に存在
									return (area.is_built && IsPointInsideAABB(
										hit_info.pos, area.nonbuild_area_min, area.nonbuild_area_max));
								} };

							// 自動構築不可エリアにヒットポイントが存在する
							if (Any_Of(not_build_areas, NotBuildAreaFunc, exec::par))	continue;
						}

						Point navmesh_pos{};

						// ナビメッシュとの判定
						{
							const dtQueryFilter filter;
							dtPolyRef ref{};
							constexpr Point Zero{};

							// ナビメッシュとの判定
							const auto status{ navmesh_query->findNearestPoly(hit_info.pos.data(),
								HalfExtents.data(), &filter, &ref, navmesh_pos.data()) };

							// ナビメッシュ状に垂直上のポイントが存在しない
							if (dtStatusFailed(status) || ref == 0 || navmesh_pos == Zero)	continue;
						}

						// ヒットポイントを修正
						{
							const float error_dis_sqr{ link_end_error_dis * link_end_error_dis };

							// 地形のヒットポイントとナビメッシュのヒットポイントの距離が遠い
							if (rcVdistSqr(navmesh_pos, hit_info.pos) > error_dis_sqr) continue;
						}

						// 仮リンクを構築する
						{
							std::lock_guard<std::mutex> lg{ mt }; // 競合阻止

							const Point&& start{ point.base_point + (edge.orthogonal_vec * -max_start_link_error) },
								& end{ hit_info.pos };

							// OffMesh Linkの始点と終点を確定
							auto& link{ edge.links.emplace_back() };

							// 必ず高い方を開始地点にする
							//if (navmesh_pos[1] <= start[1])
							{
								link.start = start;
								link.end = navmesh_pos;
							}
							//else
							//{
							//	link.start = navmesh_pos;
							//	link.end = start;
							//}
							link.nearest_pos = end;
							link.horizontal_pos = horizontal_point;
							link.base_edge = &edge;
						}

						break; // 構築終了
					}
				}, exec::par);
		}, exec::par);
}

void OffMeshLinkManager::CheckTentativeLink()
{
	// 仮リンクの被りをチェックする
	const size_t length{ edges.size() };

	std::vector<size_t> indeces(length);

	NormalAlgorithm::Iota(indeces);

	For_Each(indeces, [this, length](const size_t i)
		{
			std::mutex mt;
			auto& base{ edges[i] };

			// 終了
			if (i + 1u > length)	return;

			std::for_each(exec::par, std::next(edges.begin(), i + 1u), edges.end(),
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
								std::lock_guard<std::mutex> lg{ mt }; // 競合阻止

								base_link.is_delete = true; // 削除
								itr->is_bidir = true; // 削除しない側を双方向通行に設定
							}
						}, exec::par);
				});
		}, exec::par);

	auto CheckEdgeFunc{ [this](NavMeshEdge& edge)
	{
		For_Each(edge.links,  [this, &edge](NavMeshEdge::Link& link)
		{
			if (link.is_delete)	return; // 既に削除済み

			// 双方向通行の時だけ少し終点をナビメッシュ側にずらす
			{
				auto ShortDistFunc{ [](const Point& line_point1, const Point& line_point2,
					const Point& point)
				{
					auto MagnitudeFunc{ [](const Point& vec)
					{
					  return sqrtf(powf(vec.front(), 2.f) + powf(vec[1], 2.f) + powf(vec.back(), 2.f));
					} };

					const Point&& ab{ line_point2 - line_point1 };
					const Point&& ac{ point - line_point1 };
					Point cross{};
					rcVcross(&cross, ab, ac);

					float area = MagnitudeFunc(cross);
					float cd = area / MagnitudeFunc(ab);
					return cd;
				} };
				auto FindShortDistFunc{ [&ShortDistFunc](const Point& lt, const Point& rt, const Point& base)
					{
						return (std::min) (ShortDistFunc(lt, rt, base), ShortDistFunc(rt, lt, base));
					} };
				auto FindShortestEdgeFunc{ [&link, &FindShortDistFunc]
				(const NavMeshEdge& lt, const NavMeshEdge& rt)
				{
						const Point& end{ link.end };
						const Point&& lt_middle{ MiddlePoint(lt.start, lt.end) },
							rt_middle{ MiddlePoint(rt.start, rt.end) };

#if false
						float left_dist{ FindShortDistFunc(lt.start, lt.end, end) },
							right_dist{ FindShortDistFunc(rt.start, rt.end, end) };

						if (const float dist{ rcVdist(lt_middle, end) };
							left_dist > dist)	left_dist = dist;

						if (const float dist{ rcVdist(rt_middle, end) };
							right_dist > dist)	right_dist = dist;
#else
						const float left_dist{ (std::min) (
							{ rcVdistSqr(lt_middle, end), rcVdistSqr(lt.start, end), rcVdistSqr(lt.end, end) }) };
						const float right_dist{ (std::min) (
							{ rcVdistSqr(rt_middle, end), rcVdistSqr(rt.start, end), rcVdistSqr(rt.end, end) }) };
#endif
						return (left_dist < right_dist);
				} };

				auto&& itr{ Min_Element(edges, FindShortestEdgeFunc, exec::par) };

				if (itr != edges.end())
				{
					Point& end{ link.end };

					// リンクの終点と終点に最も近いエッジ間の最短距離
					const float dist1{ FindShortDistFunc(itr->start, itr->end, end) };
					const float dist2{ rcVdistSqr(MiddlePoint(itr->start, itr->end), end) };
					const VF2&& vec1{ ToXZ_PlaneXMFLOAT2(edge.orthogonal_vec) },
						&& vec2{ ToXZ_PlaneXMFLOAT2(itr->orthogonal_vec) };
					const float between_angle{ AngleBetweenVectors(vec1, vec2) }; // ベクトル間の角度

					link.end_edge_dist = dist1;
					link.end_edge_angle = between_angle;

					// 終点付近にナビメッシュエッジが存在する
					if (dist1 < max_link_end_edge_dis /*&& dist2 < rcVdistSqr(itr->start, itr->end) * 2.f*/)
					{
						link.end_edge = &*itr;
						// 近くのナビメッシュエッジの垂直ベクトルの逆方向に少しだけずらす
						link.end += (itr->orthogonal_vec * -max_end_link_error);
					}
					// 終点付近にナビメッシュエッジが存在しない
					else
					{
						// 始点の付近のナビメッシュエッジの垂直ベクトルに少しだけずらす
						link.end += (link.base_edge->orthogonal_vec * max_end_link_error);
					}
				}
			}

			// TempObstacleを貫通しているリンクを削除
			if (obstacle_sample)
			{
				const auto&& ref1{ obstacle_sample->HitTestObstacle(link.start.data(), link.end.data()) },
					&& ref2{ obstacle_sample->HitTestObstacle(link.end.data(), link.start.data()) };

				// 下から上方向だと反応しないので、念の為双方向で確認している
				if (ref1 + ref2 != 0) link.is_delete = true;
			}

			if (link.is_delete)	return; // 既に削除済み

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

			if (link.is_delete)	return; // 既に削除済み

			// リンクの始点・終点のナビメッシュエッジの角度に制限を設ける
			if (!Math::AdjEqual(limit_link_angle, 0.f)) // 0°は全て削除してしまうので無視する
			{
				constexpr float Pai{ Math::PAI<float> };

				// 終点付近にナビメッシュエッジが存在する
					// 始点・終点のナビエッジが指定値以上向かい合っていない
				if ((link.end_edge) && (::fabsf(link.end_edge_angle) < Pai - limit_link_angle))
				{
					link.is_delete = true;
				}
			}

			if (link.is_delete)	return; // 既に削除済み

			// 双方向通行にならない問題の解決
			if (!link.is_bidir && is_non_navedge_bidirectional) // 既に双方向通行なら無視
			{
				const Point& start{ link.start }, end{ link.end };
				const float dist_y{ ::fabsf(start[1] - end[1]) };

				// 最大の登れる高さ以下で「横跳び」よりも高い
				if (dist_y <= climbable_height && dist_y > min_buildable_height)
				{
					link.is_bidir = true;
				}
			}
		}, exec::par);
	} };

	// 全ループ
	For_Each(edges, CheckEdgeFunc, exec::par);

	// 余分な仮リンクを削除する
	For_Each(edges, [](NavMeshEdge& edge)
		{
			Erase_Remove_If(edge.links, [](NavMeshEdge::Link& link) { return link.is_delete; }, exec::par);
		}, exec::par);
}

void OffMeshLinkManager::BuildLink()
{
	auto& geom{ sample->getInputGeom() };

	if (!geom)	return;

	// 以前に自動構築されたリンクを全削除する
	geom->ClearAutoBuildOffMeshConnection();

	for (const auto& edge : edges)
	{
		for (const auto& link : edge.links)
		{
			constexpr unsigned char area = SAMPLE_POLYAREA_JUMP;
			constexpr unsigned short flags = SAMPLE_POLYFLAGS_JUMP;

			// 実際に追加する
			geom->addOffMeshConnection(link.start.data(), link.end.data(), sample->getAgentRadius(),
				link.is_bidir, area, flags, true);
		}
	}
}
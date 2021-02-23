#pragma once

#include <array>
#include <vector>

class OffMeshLinkManager final
{
private:
	using Point = std::array<float, 3>;

private:
	struct NavMeshEdge final
	{
		struct Link final
		{
			Link() = default;
			~Link() = default;

			Point start{}/*始点*/, end{}/*終点*/;
			Point nearest_pos{}/*垂直上のポイントに最も近い点*/, horizontal_pos{}/*水平上のポイント*/;
			bool is_delete{}, is_bidir{};
			float end_edge_dist{}, end_edge_angle{};
			NavMeshEdge* end_edge{}, * base_edge{};
		};
		struct DivisionPoint final
		{
			Point base_point{}, height_point{};
		};

		Point start{}, end{};
		Point orthogonal_vec{};

		std::vector<DivisionPoint> points;
		std::vector<Link> links;
	};
	struct NotBuildArea final
	{
		static constexpr size_t MaxHitPoint{ 2u };

		std::array<Point, 2> hit_points{};
		int nhit_points{};
		bool is_built{};
		std::array<Point, 8> aabb_vertex{};
		Point aabb_max{}, aabb_min{}, nonbuild_area_max{}, nonbuild_area_min{};
	};

public:
	OffMeshLinkManager();
	~OffMeshLinkManager();
	OffMeshLinkManager(const OffMeshLinkManager&) = delete;
	OffMeshLinkManager(OffMeshLinkManager&&) = delete;
	auto& operator=(const OffMeshLinkManager&) = delete;
	auto& operator=(OffMeshLinkManager&&) = delete;

public:
	bool Init(class Sample* sample);
	void Draw();

	// オフメッシュリンクの追加
	int AddLink(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
		const bool is_bidirectional);
	// オフメッシュリンクの削除
	void DeleteLink(const int index);
	// リンクと点の判定
	bool HitLinkPoint(const std::array<float, 3>& point, const int index);
	// 複数リンクと点の判定
	bool HitLinksPoint(const std::array<float, 3>& point, int* index);
	// 総リンク数取得
	int GetLinkSize() const noexcept;
	// 仮のオフメッシュリンクの自動生成
	void AutoTentativeLinksBuild();
	// 仮のオフメッシュリンクを実際に構築する
	void ReBuildNavMeshLink();
	// 自動生成したオフメッシュリンクを削除する
	void ClearBuiltAutoLink();

private:
	void CalcNavMeshEdges();
	void CalcEdgeDivision();
	void CalcTentativeLink();
	void CheckTentativeLink();
	void BuildLink();

private:
	class Sample* sample{};
	class Sample_TempObstacles* obstacle_sample{};
	unsigned char old_flags{};
	float auto_build_time_ms{};

	bool draw_links_arrow, draw_tentative_link, draw_horizontal_point, draw_edge_point,
		draw_division_point, draw_end_point, draw_navmesh_nearest_point, draw_error_dis{}, draw_all;

	// 仮リンク間の「横跳び」を許容するか？
	bool is_buildable_height_limit{};
	// 終了地点付近にナビメッシュエッジが存在しなくても、双方向通行にするか？
	bool is_non_navedge_bidirectional{ true };

	float horizontal_dis{ 5.f };            // 構築可能な水平距離
	float vertical_dis{ 7.5f };             // 構築可能な垂直距離
	float divistion_dis{ 1.2f };            // 分割点間の長さ
	float climbable_height{ 0.5f };         // 分割点から構築する時の高さ（登れる高さ）
	float	min_buildable_height{ 0.5f };     //「横跳び」を許容する時の高さ
	float	link_end_error_dis{ 0.2f };       // 地形の当たり座標とナビメッシュの当たり座標間の許容範囲
	float	orthognal_error_dis{ 0.5f };      // 垂直ベクトルで構築不可になる許容範囲
	float	link_equal_error_dis{ 0.25f };    // 他の仮リンクとの重なりを認識する許容範囲
	float max_start_link_error{ 0.2f };     // 始点をエッジからどれだけ離すか
	float max_end_link_error{ 0.2f };       // 終点をエッジからどれだけ離すか
	float limit_link_angle{ 3.1415 / 4.f }; // リンクの始点・終点のナビメッシュエッジの制限角度
	float max_link_end_edge_dis{ 0.5f };    // リンクの終点が最も近いエッジだと判断出来る最大距離
	size_t edge_link_accumulate{};          // 総リンク数
	std::vector<NavMeshEdge> edges;

	std::vector<NotBuildArea> not_build_areas;
	float box_descent{ 0.25f };     // 箱の下部の高さ
	float box_height{ 5.f };        // 箱の上部の高さ
	float box_nonbuild_dis{ 1.2f }; // 箱から構築不可の長さ
	bool is_non_build_area{};
};


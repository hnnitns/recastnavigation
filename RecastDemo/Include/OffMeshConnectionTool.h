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

#ifndef OFFMESHCONNECTIONTOOL_H
#define OFFMESHCONNECTIONTOOL_H

#include <array>
#include <vector>
#include <functional>
#include <optional>
#include "Sample.h"

// Tool to create off-mesh connection for InputGeom

class OffMeshConnectionTool : public SampleTool
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

private:
	class Sample* sample{};
	class Sample_TempObstacles* obstacle_sample{};

	std::array<float, 3> hit_pos{};
	bool hit_pos_set{};
	bool m_bidir;
	unsigned char m_oldFlags{};
	float auto_build_time_ms{};

	bool draw_links_arrow, draw_tentative_link, draw_horizontal_point, draw_edge_point,
		draw_division_point, draw_end_point, draw_navmesh_nearest_point, draw_error_dis{}, draw_all;

	// 仮リンク間の「横跳び」を許容するか？
	bool is_buildable_height_limit{};
	// 終了地点付近にナビメッシュエッジが存在しなくても、双方向通行にするか？
	bool is_non_navedge_bidirectional{ true };

	float horizontal_dis{ 5.f };         // 構築可能な水平距離
	float vertical_dis{ 7.5f };          // 構築可能な垂直距離
	float divistion_dis{ 1.2f };         // 分割点間の長さ
	float climbable_height{ 0.5f };      // 分割点から構築する時の高さ（登れる高さ）
	float	min_buildable_height{ 0.5f };  //「横跳び」を許容する時の高さ
	float	link_end_error_dis{ 0.2f };    // 地形の当たり座標とナビメッシュの当たり座標間の許容範囲
	float	orthognal_error_dis{ 0.5f };   // 垂直ベクトルで構築不可になる許容範囲
	float	link_equal_error_dis{ 0.25f }; // 他の仮リンクとの重なりを認識する許容範囲
	float max_start_link_error{ 0.2f };  // 始点をエッジからどれだけ離すか
	float max_end_link_error{ 0.2f };    // 終点をエッジからどれだけ離すか
	float limit_link_angle{ M_PI_4 };    // リンクの始点・終点のナビメッシュエッジの制限角度
	float max_link_end_edge_dis{ 0.5f }; // リンクの終点が最も近いエッジだと判断出来る最大距離
	std::vector<NavMeshEdge> edges;

	std::vector<NotBuildArea> not_build_areas;
	float box_descent{ 0.25f };      // 箱の下部の高さ
	float box_height{ 5.f };       // 箱の上部の高さ
	float box_nonbuild_dis{ 1.2f }; // 箱から構築不可の長さ
	bool is_non_build_area{};

public:
	OffMeshConnectionTool();
	~OffMeshConnectionTool();

	virtual int type() { return TOOL_OFFMESH_CONNECTION; }
	virtual void init(Sample* sample);
	virtual void reset();
	virtual void handleMenu();
	virtual void handleClickDown(const float* s, const float* p, bool shift);
	void handleClickUp(const float* /*s*/, const float* /*p*/) override {}
	void handleClick(const float* /*s*/, const float* /*p*/) override {}
	virtual void handleToggle();
	virtual void handleStep();
	virtual void handleUpdate(const float dt);
	virtual void handleRender();
	virtual void handleRenderOverlay(double* proj, double* model, int* view);

private:
	void AutoTentativeLinksBuild();
	void CalcNavMeshEdges();
	void CalcEdgeDivision();
	void CalcTentativeLink();
	void CheckTentativeLink();
	void BuildLink();
	void ReBuildNavMeshLink();

	void ClearBuiltAutoLink();
};

#endif // OFFMESHCONNECTIONTOOL_H

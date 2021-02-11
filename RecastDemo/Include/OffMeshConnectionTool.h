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
			Link(const Point& start, const Point& end, const Point& nearest_pos,
				const Point& horizontal_pos)
				: start(start), end(end), nearest_pos(nearest_pos), horizontal_pos(horizontal_pos)
			{}
			~Link() = default;

			Point start{}/*始点*/, end{}/*終点*/;
			Point nearest_pos{}/*垂直上のポイントに最も近い点*/, horizontal_pos{}/*水平上のポイント*/;
			bool is_delete{}, is_bidir{};
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
	class Sample* sample;
	class Sample_TempObstacles* obstacle_sample;

	std::array<float, 3> hit_pos;
	bool hit_pos_set;
	bool m_bidir;
	unsigned char m_oldFlags;
	float auto_build_time_ms;

	bool draw_links_arrow, draw_tentative_link, draw_horizontal_point, draw_edge_point,
		draw_division_point, draw_end_point, draw_navmesh_nearest_point, draw_error_dis, draw_all;

	bool is_buildable_height_limit; // 仮リンク間の「横跳び」を許容するか？
	float horizontal_dis;           // 構築可能な水平距離
	float vertical_dis;             // 構築可能な垂直距離
	float divistion_dis;            // 分割点間の長さ
	float climbable_height;         // 分割点から構築する時の高さ（登れる高さ）
	float	min_buildable_height;     //「横跳び」を許容する時の高さ
	float	link_end_error_dis;       // 地形の当たり座標とナビメッシュの当たり座標間の許容範囲
	float	orthognal_error_dis;      // 垂直ベクトルで構築不可になる許容範囲
	float	link_equal_error_dis;     // 他の仮リンクとの重なりを認識する許容範囲
	float	max_builable_height;      // 仮リンクの構築を許容できる最大の高さ
	float max_start_link_error;     // 始点をエッジからどれだけ離すか
	float max_end_link_error;       // 終点をエッジからどれだけ離すか
	std::vector<NavMeshEdge> edges;

	std::vector<NotBuildArea> not_build_areas;
	float box_descent; // 箱の下部の高さ
	float box_height;  // 箱の上部の高さ
	float box_nonbuild_dis; // 箱から構築不可の長さ
	bool is_not_build_area;

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
};

#endif // OFFMESHCONNECTIONTOOL_H

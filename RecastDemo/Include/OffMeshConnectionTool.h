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

			Point start{}/*�n�_*/, end{}/*�I�_*/;
			Point nearest_pos{}/*������̃|�C���g�ɍł��߂��_*/, horizontal_pos{}/*������̃|�C���g*/;
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

	bool is_buildable_height_limit; // �������N�Ԃ́u�����сv�����e���邩�H
	float horizontal_dis;           // �\�z�\�Ȑ�������
	float vertical_dis;             // �\�z�\�Ȑ�������
	float divistion_dis;            // �����_�Ԃ̒���
	float climbable_height;         // �����_����\�z���鎞�̍����i�o��鍂���j
	float	min_buildable_height;     //�u�����сv�����e���鎞�̍���
	float	link_end_error_dis;       // �n�`�̓�������W�ƃi�r���b�V���̓�������W�Ԃ̋��e�͈�
	float	orthognal_error_dis;      // �����x�N�g���ō\�z�s�ɂȂ鋖�e�͈�
	float	link_equal_error_dis;     // ���̉������N�Ƃ̏d�Ȃ��F�����鋖�e�͈�
	float	max_builable_height;      // �������N�̍\�z�����e�ł���ő�̍���
	float max_start_link_error;     // �n�_���G�b�W����ǂꂾ��������
	float max_end_link_error;       // �I�_���G�b�W����ǂꂾ��������
	std::vector<NavMeshEdge> edges;

	std::vector<NotBuildArea> not_build_areas;
	float box_descent; // ���̉����̍���
	float box_height;  // ���̏㕔�̍���
	float box_nonbuild_dis; // ������\�z�s�̒���
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

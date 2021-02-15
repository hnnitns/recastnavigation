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
#include "OtherFiles\\Array_Hlper.hpp"

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

		// ����ōŏ��̃q�b�g���A-FLT_MAX�ɐݒ�
		tmin = 0.0;

		// �������ړ��ł���ő勗���ɐݒ�i�Z�O�����g�p�j
		tmax = 1.f;

		// 3�̃X���u���ׂ�
		for (int i = 0; i < 3; i++)
		{
			if (fabsf(d[i]) < EPS)
			{
				// �����̓X���u�ɕ��s���A���_���X���u���ɂȂ��ꍇ�̓q�b�g�Ȃ�
				if (sp[i] < amin[i] || sp[i] > amax[i])
					return false;
			}
			else
			{
				//�X���u�̃j�A����уt�@�[�v���[���ƃ��C�̌���t�l���v�Z���܂�
				const float ood = 1.f / d[i];
				float t1 = (amin[i] - sp[i]) * ood;
				float t2 = (amax[i] - sp[i]) * ood;

				// t1���߂��̕��ʂƌ��������At2�������̕��ʂƌ���������
				if (t1 > t2) std::swap(t1, t2);
				if (t1 > tmin) tmin = t1;
				if (t2 < tmax) tmax = t2;

				// �X���u�̌����_�������Ȃ�Ƃ����ɏՓ˂Ȃ��ŏI��
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
	hit_pos(), draw_all(true)
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

		// �\�z�����T���v������Q���n�̂��̂��ǂ������m�F
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
	// ����ʍs
	if (imguiCheck("One Way", !m_bidir))
		m_bidir = false;
	// �o�����ʍs
	if (imguiCheck("Bidirectional", m_bidir))
		m_bidir = true;

	imguiSeparatorLine();

	imguiValue("Auto OffMeshLink");

	imguiSlider("climbable_height", &climbable_height, 0.1f, 10.f, 0.1f);
	imguiSlider("horizontal_dis", &horizontal_dis, 0.1f, 25.f, 0.1f);
	imguiSlider("vertical_dis", &vertical_dis, 0.1f, 25.f, 0.1f);
	imguiSlider("divistion_dis", &divistion_dis, sample->getAgentRadius(), 5.f, 0.1f);
	imguiSlider("link_end_error", &link_end_error_dis, 0.1f, 5.f, 0.1f);
	imguiSlider("max_orth_error", &orthognal_error_dis, 0.1f, 5.f, 0.1f);
	imguiSlider("link_equal_error_dis", &link_equal_error_dis, 0.01f, 1.f, 0.01f);
	imguiSlider("max_start_link_error", &max_start_link_error, 0.1f, 1.f, 0.1f);
	imguiSlider("max_end_link_error", &max_end_link_error, 0.1f, 1.f, 0.1f);
	limit_link_angle = Math::ToDegree(limit_link_angle);
	imguiSlider("limit_link_angle", &limit_link_angle, 0.f, 90.f, 0.1f); // 90������͐�΂ɃA���G�i�C
	limit_link_angle = Math::ToRadian(limit_link_angle);
	imguiSlider("max_link_end_edge_dis", &max_link_end_edge_dis, 0.1f, 1.f, 0.1f);

	imguiValue("        ");

	// �u�����у����N�v�ɐ�����݂��邩�H
	{
		if (imguiCheck("Buildable Height Limit", is_buildable_height_limit))
			is_buildable_height_limit ^= true;

		if (is_buildable_height_limit)
		{
			imguiSlider("min_buildable_height", &min_buildable_height, 0.1f, 10.f, 0.1f);
		}
	}

	imguiValue("        ");

	// �������N�̎�������
	if (imguiButton("Tentative Link Build"))
		AutoTentativeLinksBuild();

	if (!edges.empty() && imguiButton("Link Build"))
		BuildLink();

	if (imguiButton("ClearBuiltAutoLink"))
		ClearBuiltAutoLink();

	// �폜
	if (imguiButton("Link Clear"))
		edges.clear();

	imguiValue("        ");

	// �����\�z�s�G���A�ݒ�
	{
		if (imguiCheck("Not Build Area", is_non_build_area))
			is_non_build_area ^= true;

		if (is_non_build_area)
		{
			imguiSlider("box_descent", &box_descent, 0.1f, 10.f, 0.1f);
			imguiSlider("box_height", &box_height, 0.1f, 10.f, 0.1f);
			imguiSlider("box_nonbuild_dis", &box_nonbuild_dis, 0.1f, 5.f, 0.1f);

			if (imguiButton("All Clear"))
			{
				not_build_areas.clear();
			}
		}
	}

	imguiValue("        ");

	if (imguiCheck("All Draw", draw_all))
		draw_all ^= true;

	// ��������
	if (!edges.empty())
	{
		if (draw_all)
		{
			// �n�_����I�_�ւ̖��
			if (imguiCheck("Links Arrow", draw_links_arrow))
				draw_links_arrow ^= true;

			// �G�b�W�̎n�_�I�_
			if (imguiCheck("Edge Point", draw_edge_point))
				draw_edge_point ^= true;

			// �G�b�W�̕����_
			if (imguiCheck("Division Point", draw_division_point))
				draw_division_point ^= true;

			// �������N
			if (imguiCheck("Tentative Link", draw_tentative_link))
				draw_tentative_link ^= true;

			// ������̃|�C���g
			if (imguiCheck("Horizontal Point", draw_horizontal_point))
				draw_horizontal_point ^= true;

			// �I�_
			if (imguiCheck("End Point", draw_end_point))
				draw_end_point ^= true;

			// �i�r���b�V���ɍł��߂��|�C���g
			if (imguiCheck("Navmesh Nearest Point", draw_navmesh_nearest_point))
				draw_navmesh_nearest_point ^= true;

			// �덷�͈�
			if (imguiCheck("Error Dis", draw_error_dis))
				draw_error_dis ^= true;
		}

		imguiValue("        ");

		{
			std::string text{ "build_time: " };

			// ��������
			{
				text += std::to_string(auto_build_time_ms);
				text += "ms";
				imguiValue(text.data());
			}

			// �����������ꂽ�G�b�W�̑���
			{
				text = "auto Edge size: ";
				text += std::to_string(edges.size());
				imguiValue(text.data());
			}

			// �����������ꂽ�����N�̑���
			{
				static size_t temp{ (std::numeric_limits<size_t>::max)() };
				size_t accumulate{};

				if (temp != accumulate)
				{
					accumulate = NormalAlgorithm::Accumulate(edges, 0u,
						[](const size_t i, const NavMeshEdge& edge) { return i + edge.links.size(); });
				}

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
		// �\�z�s�G���A
		if (is_non_build_area)
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

			// �G���h�|�C���g���\���ɋ߂��ꍇ�͍폜
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
			// �ł��߂������N�̃G���h�|�C���g��������
			float nearestDist = (std::numeric_limits<float>::max)();
			int nearestIndex = -1;

			const auto& verts = geom->getOffMeshConnectionVerts();

			for (int i = 0; i < geom->getOffMeshConnectionCount() * 2; ++i)
			{
				float d = rcVdistSqr(p, &verts[i * 3]);

				if (d < nearestDist)
				{
					nearestDist = d;
					nearestIndex = i / 2; // Each link has two vertices. // �e�����N�ɂ�2�̒��_������܂��B
				}
			}

			// If end point close enough, delete it.
			// �G���h�|�C���g���\���ɋ߂��ꍇ�́A�폜���܂��B
			if (nearestIndex != -1 &&
				sqrtf(nearestDist) < sample->getAgentRadius())
			{
				geom->deleteOffMeshConnection(nearestIndex);
			}
		}
	}
	else
	{
		// �\�z�s�G���A
		if (is_non_build_area)
		{
			// �ǉ�����
			if (not_build_areas.empty())	not_build_areas.emplace_back();

			auto&& itr{ Find_If(not_build_areas,
				[](NotBuildArea& area) { return !area.is_built; }, exec::par) };

			if (itr != not_build_areas.end())
			{
				if (auto& area{ *itr };
					area.nhit_points == NotBuildArea::MaxHitPoint - 1)
				{
					// �Ō�̃|�C���g���N���b�N����ƁA�`�󂪊m�肷��
					if (rcVdistSqr(p, area.hit_points[area.nhit_points].data()) < rcSqr(0.2f))
					{
						area.is_built = true; // �\�z�ς�
						not_build_areas.emplace_back(); // ���̃G���A��ǉ�
					}
					else
					{
						// �����|�C���g���X�V
						rcVcopy(area.hit_points.back().data(), p);

						// �ő�E�ŏ��l���擾
						{
							const auto& hit_points{ area.hit_points };

							for (size_t i = 0; i < area.aabb_max.size(); i++)
							{
								area.aabb_max[i] = rcMax(hit_points.front()[i], hit_points.back()[i]);
								area.aabb_min[i] = rcMin(hit_points.front()[i], hit_points.back()[i]);
							}

							area.aabb_max[1] += box_height;
							area.aabb_min[1] -= box_descent;

							const Point dis{ box_nonbuild_dis, 0.f, box_nonbuild_dis };

							area.nonbuild_area_max = area.aabb_max + dis;
							area.nonbuild_area_min = area.aabb_min - dis;
						}

						// �e���_���v�Z
						{
							auto& vertex{ area.aabb_vertex };
							const Point& min{ area.aabb_min }, max{ area.aabb_max };
							const VF3&& min_vs{ min.front(), min[1], min.back() },
								max_vs{ max.front(), max[1], max.back() };

							// ����
							vertex[0] = area.aabb_min;                    // ����
							vertex[1] = { min_vs.x, min_vs.y, max_vs.z }; // ����
							vertex[2] = { max_vs.x, min_vs.y, max_vs.z }; // �E��
							vertex[3] = { max_vs.x, min_vs.y, min_vs.z }; // �E��

							// �㕔
							vertex[4] = { min_vs.x, max_vs.y, min_vs.z }; // ����
							vertex[5] = { min_vs.x, max_vs.y, max_vs.z }; // ����
							vertex[6] = area.aabb_max;                    // �E��
							vertex[7] = { max_vs.x, max_vs.y, min_vs.z }; // �E��
						}
					}
				}
				else
				{
					// �V�����|�C���g��ǉ�
					rcVcopy(area.hit_points[area.nhit_points].data(), p);

					if (area.nhit_points < NotBuildArea::MaxHitPoint - 1)
						area.nhit_points++;
				}
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

	// �I�t���b�V�������N�̎��������֌W
	if (!edges.empty() && draw_all)
	{
		// ���C��
		dd.begin(DU_DRAW_LINES, 2.0f);
		for (const auto& edge : edges)
		{
			constexpr float LineAdjY{ 2.f };

			auto start{ edge.start }, end{ edge.end };

			// �n�_�E�I�_
			if (draw_edge_point)
			{
				constexpr UINT32 EdgeColor{ duRGBA(255, 0, 0, 200) };

				// ��������
				start[1] += LineAdjY;
				end[1] += LineAdjY;

				// �n�_
				dd.vertex(edge.start.data(), EdgeColor);
				dd.vertex(start.data(), EdgeColor);

				// �I�_
				dd.vertex(edge.end.data(), EdgeColor);
				dd.vertex(end.data(), EdgeColor);
			}

			// �����_
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

			// ���`��
			if (draw_links_arrow)
			{
				// �n�_����I�_�ւ̃x�N�g��
				{
					constexpr float ArrowAdjY{ LineAdjY * 0.375f }, AdjDistance{ 1.f };
					constexpr UINT32 ArrowColor{ duRGBA(255, 255, 0, 200) };

					// �����ւ��炷
					{
						const auto&& orth_vec{ edge.orthogonal_vec * -AdjDistance };

						start = edge.start + orth_vec;
						end = edge.end + orth_vec;
					}

					// ���₷��ׂɏ����Z������
					auto vec{ end - start };
					const float len{ rcVdist(start, end) };
					const float short_len{ len / 7.5f };

					rcVnormalize(&vec);

					// �O������ɏ����ʒu�����炷
					start = start + (vec * short_len);
					end = end + (vec * -short_len);

					// �������Ē���
					start[1] += ArrowAdjY;
					end[1] += ArrowAdjY;

					duAppendArrow(&dd,
						start[0], start[1], start[2],
						end[0], end[1], end[2],
						0.0f, 0.4f, ArrowColor);
				}

				// �����E�����x�N�g��
				{
					start = edge.start;
					end = edge.end;

					auto vec{ end - start };
					const float len{ rcVdist(start, end) };

					rcVnormalize(&vec);

					const auto&& middle_pos{ start + (vec * (len * 0.5f)) },
						&& horizontal_pos{ middle_pos + (edge.orthogonal_vec * horizontal_dis) };

					// �����x�N�g��
					{
						constexpr UINT32 ArrowColor{ duRGBA(255, 0, 255, 200) };

						start = middle_pos;
						end = horizontal_pos;

						duAppendArrow(&dd,
							start[0], start[1], start[2],
							end[0], end[1], end[2],
							0.0f, 0.4f, ArrowColor);
					}

					// �����x�N�g��
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

					// ����x�N�g��
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

			// ���e�͈�
			if (draw_error_dis)
			{
				constexpr auto Color{ duRGBA(0, 0, 0, 150) };

				for (auto& link : edge.links)
				{
					// �n�`�̓�������W�ƃi�r���b�V���̓�������W�Ԃ̋��e�͈�
					duAppendCircle(&dd, link.nearest_pos[0], link.nearest_pos[1], link.nearest_pos[2],
						link_end_error_dis, Color);

					// �����x�N�g���ō\�z�s�ɂȂ鋖�e�͈�
					{
						start = link.start;
						end = start + (edge.orthogonal_vec * orthognal_error_dis);
						dd.vertex(start.data(), Color);
						dd.vertex(end.data(), Color);
					}

					// ���̉������N�Ƃ̏d�Ȃ��F�����鋖�e�͈�
					duAppendCircle(&dd, link.start[0], link.start[1] + 0.5f, link.start[2],
						link_equal_error_dis, Color);

					duAppendCircle(&dd, link.end[0], link.end[1] + 0.5f, link.start[2],
						link_equal_error_dis, Color);
				}
			}

			// �������N
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

			for (const auto& link : edge.links)
			{
				if (!link.end_edge)	continue;

				if (link.end_edge_dist <= max_link_end_edge_dis)	continue;

				constexpr UINT32 Color{ duRGBAf(0.5f, 0.5f, 0.5f, 0.8f) };

				const auto* end_edge{ link.end_edge };
				const Point&& middle_pos{ MiddlePoint(end_edge->start, end_edge->end) };

				duAppendArrow(&dd, link.end.front(), link.end[1], link.end.back(),
					middle_pos.front(), middle_pos[1], middle_pos.back(), 0.4f, 0.4f, Color);
			}
		}
		dd.end();

		// �|�C���g
		dd.begin(DU_DRAW_POINTS, 5.f);
		for (const auto& edge : edges)
		{
			auto start{ edge.start };

			// Link
			for (const auto& link : edge.links)
			{
				// �I�_
				if (draw_end_point)
				{
					constexpr UINT32 PointColor{ duRGBA(0, 0, 0, 200) };

					start = link.end;

					dd.vertex(start.data(), PointColor);
				}

				// �i�r���b�V���̍Őڋ߃|�C���g
				if (draw_navmesh_nearest_point)
				{
					constexpr UINT32 PointColor{ duRGBA(50, 50, 255, 200) };

					start = link.nearest_pos;

					dd.vertex(start.data(), PointColor);
				}

				// ������̃|�C���g
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

	// �\�z�s�G���A
	{
		// �ݒ蒆
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

		// �ݒ芮��
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

				// ����
				VertexFunc(vertex[2], col); VertexFunc(vertex[1], col); VertexFunc(vertex[0], col);
				VertexFunc(vertex[0], col); VertexFunc(vertex[3], col); VertexFunc(vertex[2], col);

				// �㕔
				VertexFunc(vertex[4], col); VertexFunc(vertex[5], col); VertexFunc(vertex[6], col);
				VertexFunc(vertex[6], col); VertexFunc(vertex[7], col); VertexFunc(vertex[4], col);

				// ����
				VertexFunc(vertex[1], col); VertexFunc(vertex[5], col); VertexFunc(vertex[4], col);
				VertexFunc(vertex[4], col); VertexFunc(vertex[0], col); VertexFunc(vertex[1], col);

				// �E��
				VertexFunc(vertex[2], col); VertexFunc(vertex[3], col); VertexFunc(vertex[7], col);
				VertexFunc(vertex[7], col); VertexFunc(vertex[6], col); VertexFunc(vertex[2], col);

				// �O��
				VertexFunc(vertex[0], col); VertexFunc(vertex[4], col); VertexFunc(vertex[7], col);
				VertexFunc(vertex[7], col); VertexFunc(vertex[3], col); VertexFunc(vertex[0], col);

				// �㕔
				VertexFunc(vertex[1], col); VertexFunc(vertex[2], col); VertexFunc(vertex[6], col);
				VertexFunc(vertex[6], col); VertexFunc(vertex[5], col); VertexFunc(vertex[1], col);
			}
			dd.end();

			dd.begin(DU_DRAW_LINES, 2.0f);
			{
				// �ȈՃ��C���[�t���[��
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

				// �\�z�s�G���A
				{
					constexpr UINT32 col{ duRGBA(50, 50, 50, 150) };

					const Point& max{ area.nonbuild_area_max }, min{ area.nonbuild_area_min };
					const float pos_y{ (min[1] + max[1]) / 2.f };
					const Point&& l_top{ min.front(), pos_y, max.back() },
						l_bottom{ min.front(), pos_y, min.back() },
						r_top{ max.front(), pos_y, max.back() },
						r_bottom{ max.front(), pos_y, min.back() };

					VertexFunc(l_bottom, col); VertexFunc(l_top, col);    // ��
					VertexFunc(l_top, col); VertexFunc(r_top, col);       // ��
					VertexFunc(r_top, col); VertexFunc(r_bottom, col);    // �E
					VertexFunc(r_bottom, col); VertexFunc(l_bottom, col); // ��
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

	dd.depthMask(true);
}

void OffMeshConnectionTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	GLdouble x, y, z;
	const int h = view[3];

	if (!is_non_build_area)
	{
		// Draw start and end point labels
		if (hit_pos_set && gluProject((GLdouble) hit_pos[0], (GLdouble) hit_pos[1], (GLdouble) hit_pos[2],
			model, proj, view, &x, &y, &z))
		{
			imguiDrawText((int) x, (int) (y - 25), IMGUI_ALIGN_CENTER, "Start", imguiRGBA(0, 0, 0, 220));
		}

		// Tool help
		if (!hit_pos_set)
		{
			imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Create new connection.  SHIFT+LMB: Delete existing connection, click close to start or end point.", imguiRGBA(255, 255, 255, 192));
		}
		else
		{
			imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Set connection end point and finish.", imguiRGBA(255, 255, 255, 192));
		}
	}
	else
	{
		if (not_build_areas.empty() || not_build_areas.back().nhit_points == 0)
		{
			imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Create new shape.  SHIFT+LMB: Delete existing shape (click inside a shape).", imguiRGBA(255, 255, 255, 192));
		}
		else
		{
			imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "Click LMB to add end points. Click on the red point to finish the shape.", imguiRGBA(255, 255, 255, 192));
		}
	}

	for (auto& edge : edges)
	{
		for (auto& link : edge.links)
		{
			if (!link.end_edge)	continue;

			if (link.end_edge_dist <= max_link_end_edge_dis)	continue;

			if (gluProject((GLdouble) link.end[0], (GLdouble) link.end[1], (GLdouble) link.end[2],
				model, proj, view, &x, &y, &z))
			{
				std::string str;

				str += "dist: ";
				str += std::to_string(link.end_edge_dist);
				str += ", angle: ";
				str += std::to_string(link.end_edge_angle);

				imguiDrawText(x, y, IMGUI_ALIGN_LEFT, str.data(), imguiRGBA(255, 255, 255, 255));
			}
		}
	}
}

void OffMeshConnectionTool::AutoTentativeLinksBuild()
{
	auto* ctx{ sample->GetContext() };

	// �^�C�}�[�v���J�n
	ctx->resetTimers();
	auto_build_time_ms = 0.f;
	ctx->startTimer(RC_TIMER_TEMP);

	// �i�r���b�V���̃G�b�W�̍쐬
	CalcNavMeshEdges();

	// �G�b�W�𕪊�
	CalcEdgeDivision();

	// �I�_���v�Z���������N���쐬
	CalcTentativeLink();

	// ���肵���������N�ɒ����E�C��
	CheckTentativeLink();

	// �^�C�}�[�v���I��
	ctx->stopTimer(RC_TIMER_TEMP);
	auto_build_time_ms = ctx->getAccumulatedTime(RC_TIMER_TEMP) / 1000.f;
}

void OffMeshConnectionTool::CalcNavMeshEdges()
{
	// ���������Y��
	if (!sample)	return;

	// ������
	edges.clear();

	const dtNavMesh* mesh{ sample->getNavMesh() };

	// �i�r���b�V���𐶐����Ă��Ȃ�
	if (!mesh)	return;

	for (int idx = 0; idx < mesh->getMaxTiles(); ++idx)
	{
		const dtMeshTile* tile = mesh->getTile(idx);

		// �����ȃ^�C��
		if (!tile->header) continue;

		constexpr float thr{ 0.01f * 0.01f };

		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			const dtPoly* p = &tile->polys[i];

			// ���b�V���O��OffMeshLink�ڑ�
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) continue;

			const dtPolyDetail* pd = &tile->detailMeshes[i];

			for (int j = 0, nj = (int) p->vertCount; j < nj; ++j)
			{
				// �Q�Ɛ悪����
				if (p->neis[j] != 0) continue;

				const float* v0 = &tile->verts[p->verts[j] * 3];
				const float* v1 = &tile->verts[p->verts[(j + 1) % nj] * 3];

				// Draw detail mesh edges which align with the actual poly edge.
				// This is really slow.
				// ���ۂ̃|���G�b�W�Ɛ��񂷂�ڍ׃��b�V���G�b�W��`�悵�܂��B
				// ����͖{���ɒx���ł��B
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

							// �G�b�W�̎n�_�ƏI�_������
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

			// �������������X�ɒǉ�
			for (float dist = divistion_dis; dist < len; dist += divistion_dis)
			{
				const Point base{ edge.start + (vec * dist) };

				// �����\�z�s�G���A�ɕ����_�����݂���
				if (Any_Of(not_build_areas, [&base](const NotBuildArea& area)
					{ return area.is_built && IsPointInsideAABB(base, area.aabb_min, area.aabb_max); },
					exec::par))	continue;

				auto& point{ edge.points.emplace_back() };
				const auto& geom{ sample->getInputGeom() };

				point.base_point = base;
				point.height_point = base + Point{ 0.f, climbable_height, 0.f };

				InputGeom::RaycastMeshHitInfo hit_info_top{}, hit_info_bottom{};

				// �����̒���������ۂɃ��C�Ŋm�F����
				const bool hit_mesh_top
				{ geom->RaycastMesh(point.height_point, point.base_point, &hit_info_top) },
					hit_mesh_bottom{ geom->RaycastMesh(point.base_point, point.height_point, &hit_info_bottom) };

				constexpr float AdjYDis{ 0.25f };

				// �q�b�g�|�C���g�Ɉʒu�𒲐�����
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

void OffMeshConnectionTool::CalcTentativeLink()
{
	For_Each(edges, [this](NavMeshEdge& edge)
		{
			std::mutex mt;

			VF3 vec3{ ToXMFLOAT(edge.end - edge.start) };
			const VF2&& vec{ vec3.x, vec3.z }; // Y���͖���
			const VF2 orth_vec2d{ VectorNormalize(VectorOrthogonal(vec)) }; // XZ���ʏ�̐����x�N�g��

			edge.orthogonal_vec = { orth_vec2d.x, 0.f/*Y���͖���*/, orth_vec2d.y };

			edge.links.clear();
			edge.links.reserve(edge.points.size());

			// �e�����_����I�_���v�Z����
			For_Each(edge.points,
				[&edge, orth_vec2d, &mt, this](const NavMeshEdge::DivisionPoint& point)
				{
					const auto& geom{ sample->getInputGeom() };
					const float agent_radius{ sample->getAgentRadius() };
					const auto* navmesh_query{ sample->getNavMeshQuery() };

					InputGeom::RaycastMeshHitInfo hit_info{};

					// ������̃|�C���g�i���p�|�C���g�j
					auto horizontal_point{ point.height_point + (edge.orthogonal_vec * horizontal_dis) };

					const bool hit_mesh
					{ geom->RaycastMesh(point.height_point, horizontal_point, &hit_info) ||
						geom->RaycastMesh(horizontal_point, point.height_point, &hit_info) };

					// ������̃|�C���g�܂łɒn�`�����݂��邩�H
					if (hit_mesh)
					{
						// �����������W�Ƀ|�C���g�����肷��
						horizontal_point = hit_info.pos + (edge.orthogonal_vec * agent_radius);

						const float diameter{ agent_radius * 2.f };
						const float distance{ diameter + orthognal_error_dis }; // �Œ�ł��G�[�W�F���g�̒��a�͕K�v

						// �����_�ɋ߂�����
						if (rcVdistSqr(horizontal_point, point.height_point) <= distance * distance)	return;

						const auto&& horizon_vec{ horizontal_point - point.height_point };
						const VF2&& horizon_vec2d{ horizon_vec.front(), horizon_vec.back() };

						// �����_��������ɑ��݂���i�O�ׁ̈j
						if (VectorDot(orth_vec2d, horizon_vec2d) <= 0.f)	return;
					}

					auto inv_horizona_vec{ horizontal_point - point.height_point };

					rcVnormalize(&inv_horizona_vec);

					constexpr Point HalfExtents{ 0.5f, 0.5f, 0.5f };

					// �����_�����t�߂��琅����̃|�C���g�܂Ō���
					for (float dis = agent_radius * agent_radius;
						dis < horizontal_dis - agent_radius * agent_radius; dis += HalfExtents.front())
					{
						// �n�`�Ƃ̔���
						{
							constexpr Point Down{ 0.f, -1.f, 0.f };

							// �����x�N�g���̎n�_�ƏI�_
							const Point&& start{ point.height_point + (inv_horizona_vec * dis) },
								&& end{ start + (Down * (vertical_dis + climbable_height)) };

							// ������ɒn�`�����݂��Ȃ�
							if (!geom->RaycastMesh(start, end, &hit_info))	continue;
						}

						// �����\�z�s�G���A�Ƃ̔���
						{
							auto NotBuildAreaFunc{ [&hit_info](const NotBuildArea& area)
							{
									// �\�z����Ă��āA�q�b�g�|�C���g���G���A�O�ɑ���
									return (area.is_built && IsPointInsideAABB(
										hit_info.pos, area.nonbuild_area_min, area.nonbuild_area_max));
								} };

							// �����\�z�s�G���A�Ƀq�b�g�|�C���g�����݂���
							if (Any_Of(not_build_areas, NotBuildAreaFunc, exec::par))	continue;
						}

						Point navmesh_pos{};

						// �i�r���b�V���Ƃ̔���
						{
							const dtQueryFilter filter;
							dtPolyRef ref{};
							constexpr Point Zero{};

							// �i�r���b�V���Ƃ̔���
							const auto status{ navmesh_query->findNearestPoly(hit_info.pos.data(),
								HalfExtents.data(), &filter, &ref, navmesh_pos.data()) };

							// �i�r���b�V����ɐ�����̃|�C���g�����݂��Ȃ�
							if (dtStatusFailed(status) || ref == 0 || navmesh_pos == Zero)	continue;
						}

						// �q�b�g�|�C���g���C��
						{
							const float error_dis_sqr{ link_end_error_dis * link_end_error_dis };

							// �n�`�̃q�b�g�|�C���g�ƃi�r���b�V���̃q�b�g�|�C���g�̋���������
							if (rcVdistSqr(navmesh_pos, hit_info.pos) > error_dis_sqr) continue;
						}

						// �������N���\�z����
						{
							std::lock_guard<std::mutex> lg{ mt }; // �����j�~

							const Point&& start{ point.base_point + (edge.orthogonal_vec * -max_start_link_error) },
								& end{ hit_info.pos };

							// OffMesh Link�̎n�_�ƏI�_���m��
							auto& link{ edge.links.emplace_back() };

							link.start = start;
							link.end = navmesh_pos;
							link.nearest_pos = end;
							link.horizontal_pos = horizontal_point;
						}

						break; // �\�z�I��
					}
				}, exec::par);
		}, exec::par);
}

void OffMeshConnectionTool::CheckTentativeLink()
{
	// �������N�̔����`�F�b�N����
#if true // �S�ă}���`�X���b�h��
	const size_t length{ edges.size() };

	std::vector<size_t> indeces(length);

	NormalAlgorithm::Iota(indeces);

	For_Each(indeces, [this, length](const size_t i)
		{
			std::mutex mt;
			auto& base{ edges[i] };

			// �I��
			if (i + 1u > length)	return;

			std::for_each(exec::par, std::next(edges.begin(), i + 1u), edges.end(),
				[&base, this, &mt](NavMeshEdge& other)
				{
					For_Each(base.links, [this, &other, &mt](NavMeshEdge::Link& base_link)
						{
							// ���ɍ폜�����肵�Ă���
							if (base_link.is_delete)	return;

							const float error_sqr{ link_equal_error_dis * 2.f };
							// ��v���鑼�̉������N������
							auto&& itr{ Find_If(other.links,
								[&base_link, error_sqr](NavMeshEdge::Link& oth_link)
								{
									// ���ɍ폜�����肵�Ă���
									if (oth_link.is_delete)	return false;

									// ���͈͓��Ɏn�_�ƏI�_���������Ă���
									return (rcVdistSqr(base_link.start, oth_link.end) <= error_sqr &&
										rcVdistSqr(base_link.end, oth_link.start) <= error_sqr);
								}, exec::par) };

							// ���������̂ō폜�\��ɒǉ�
							if (itr != other.links.end())
							{
								std::lock_guard<std::mutex> lg{ mt }; // �����j�~

								base_link.is_delete = true; // �폜
								itr->is_bidir = true; // �폜���Ȃ�����o�����ʍs�ɐݒ�
							}
						}, exec::par);
				});
		}, exec::par);
#else // �ꕔ�����������[�v
	for (size_t i = 0, length = edges.size(); i < length; i++)
	{
		std::mutex mt;
		auto& base{ edges[i] };

		// �I��
		if (i + 1u > length)	break;

		std::for_each(exec::par, std::next(edges.begin(), i + 1u), edges.end(),
			[&base, this, &mt](NavMeshEdge& other)
			{
				For_Each(base.links, [this, &other, &mt](NavMeshEdge::Link& base_link)
					{
						// ���ɍ폜�����肵�Ă���
						if (base_link.is_delete)	return;

						const float error_sqr{ link_equal_error_dis * 2.f };
						// ��v���鑼�̉������N������
						auto&& itr{ Find_If(other.links,
							[&base_link, error_sqr](NavMeshEdge::Link& oth_link)
							{
								// ���ɍ폜�����肵�Ă���
								if (oth_link.is_delete)	return false;

								// ���͈͓��Ɏn�_�ƏI�_���������Ă���
								return (rcVdistSqr(base_link.start, oth_link.end) <= error_sqr &&
									rcVdistSqr(base_link.end, oth_link.start) <= error_sqr);
							}, exec::par) };

						// ���������̂ō폜�\��ɒǉ�
						if (itr != other.links.end())
						{
							std::lock_guard<std::mutex> lg{ mt };

							base_link.is_delete = true; // �폜
							itr->is_bidir = true; // �폜���Ȃ�����o�����ʍs�ɐݒ�
							}
					}, exec::par);
			});
	}
#endif

	auto CheckEdgeFunc{ [this](NavMeshEdge& edge)
	{
		For_Each(edge.links,  [this, &edge](NavMeshEdge::Link& link)
		{
			if (link.is_delete)	return; // ���ɍ폜�ς�

			// �o�����ʍs�̎����������I�_���i�r���b�V�����ɂ��炷
			{
				using DivPoint = NavMeshEdge::DivisionPoint;

				auto FindShortestPoint{ [&link](const DivPoint& l_point, const DivPoint& r_point)
				{
					return (rcVdistSqr(link.end, l_point.base_point) <
						rcVdistSqr(link.end, r_point.base_point));
				} };

				auto FindShortestEdge{ [&link]	(const NavMeshEdge& left, const NavMeshEdge& right)
				{
					// ��̂�OK
					const float left_dist{ rcVdistSqr(link.end, MiddlePoint(left.start, left.end)) },
						right_dist{ rcVdistSqr(link.end, MiddlePoint(right.start, right.end)) };

					return (left_dist < right_dist);
				} };

				auto&& itr{ Min_Element(edges, FindShortestEdge, exec::par) };

				if (itr != edges.end())
				{
					//link.end_edge = ToArray(MiddlePoint(ToXMFLOAT(itr->start), ToXMFLOAT(itr->end)));
					link.end_edge = &*itr;

					auto&& point_itr{ Min_Element(itr->points, FindShortestPoint, exec::par) };

					if (point_itr == itr->points.end())
					{
						link.end_edge = nullptr;
					}
					else
					{
						// �����N�̏I�_�ƏI�_�ɍł��߂��G�b�W�Ԃ̍ŒZ����
						const float dist{ rcVdist(point_itr->base_point, link.end) };

						const VF3&& vec1{ ToXMFLOAT(edge.orthogonal_vec) }, && vec2{ ToXMFLOAT(itr->orthogonal_vec) };
						const float between_angle{ AngleBetweenVectors(vec1, vec2) }; // �x�N�g���Ԃ̊p�x

						link.end_edge_dist = dist;
						link.end_edge_angle = between_angle;

						if (dist < max_link_end_edge_dis)
							link.end += (itr->orthogonal_vec * -max_end_link_error);
						//else
						//	link.end_edge = nullptr;
					}
				}
			}

			// TempObstacle���ђʂ��Ă��郊���N���폜
			if (obstacle_sample)
			{
				const auto&& ref1{ obstacle_sample->HitTestObstacle(link.start.data(), link.end.data()) },
					&& ref2{ obstacle_sample->HitTestObstacle(link.end.data(), link.start.data()) };

				// �������������Ɣ������Ȃ��̂ŁA�O�̈בo�����Ŋm�F���Ă���
				if (ref1 + ref2 != 0) link.is_delete = true;
			}

			if (link.is_delete)	return; // ���ɍ폜�ς�

			// �u�����у����N�v���폜
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

			if (link.is_delete)	return; // ���ɍ폜�ς�

			// �����N�̎n�_�E�I�_�̃i�r���b�V���G�b�W�̊p�x�ɐ�����݂���
			if (!Math::AdjEqual(limit_link_angle, 0.f)) // 0���͑S�č폜���Ă��܂��̂Ŗ�������
			{
				if (link.end_edge_dist < max_link_end_edge_dis)
				{
					constexpr float Pai{ Math::PAI<float> };

					// �n�_�E�I�_�̃i�r�G�b�W���w��l�ȏ�����������Ă��Ȃ�
					if (::fabsf(link.end_edge_angle) < Pai - limit_link_angle ||
						::fabsf(link.end_edge_angle) > Pai + limit_link_angle)
					{
						link.is_delete = true;
					}
				}
			}
		}, exec::par);
	} };

	// �S���[�v
	For_Each(edges, CheckEdgeFunc, exec::par);

	// �]���ȉ������N���폜����
	For_Each(edges, [](NavMeshEdge& edge)
		{
			Erase_Remove_If(edge.links, [](NavMeshEdge::Link& link) { return link.is_delete; }, exec::par);
		}, exec::par);
}

void OffMeshConnectionTool::BuildLink()
{
	auto& geom{ sample->getInputGeom() };

	if (!geom)	return;

	// �ȑO�Ɏ����\�z���ꂽ�����N��S�폜����
	geom->ClearAutoBuildOffMeshConnection();

	for (const auto& edge : edges)
	{
		for (const auto& link : edge.links)
		{
			constexpr unsigned char area = SAMPLE_POLYAREA_JUMP;
			constexpr unsigned short flags = SAMPLE_POLYFLAGS_JUMP;

			// ���ۂɒǉ�����
			geom->addOffMeshConnection(
				link.start.data(), link.end.data(), sample->getAgentRadius(), link.is_bidir, area, flags);
		}
	}
}

void OffMeshConnectionTool::ClearBuiltAutoLink()
{
	auto& geom{ sample->getInputGeom() };

	if (!geom)	return;

	// �ȑO�Ɏ����\�z���ꂽ�����N��S�폜����
	geom->ClearAutoBuildOffMeshConnection();
}
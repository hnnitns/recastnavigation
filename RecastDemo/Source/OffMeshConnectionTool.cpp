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
	horizontal_distance(2.5f),
	vertical_distance(7.5f),
	divistion_distance(0.5f),
	link_end_error_dis(0.2f),
	max_orthognal_error_dis(0.5f),
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

	imguiSlider("horizontal_dis", &horizontal_distance, 0.1f, 25.f, 0.1f);
	imguiSlider("vertical_dis", &vertical_distance, 0.1f, 25.f, 0.1f);
	imguiSlider("divistion_dis", &divistion_distance, 0.1f, 10.f, 0.1f);
	imguiSlider("link_end_error", &link_end_error_dis, 0.1f, 5.f, 0.1f);
	imguiSlider("max_orth_error", &max_orthognal_error_dis, 0.1f, 5.f, 0.1f);

	// ��������
	if (imguiButton("Link Build"))
		AutoLinksBuild();
	// �폜
	if (imguiButton("Link Clear"))
		edges.clear();

	// ��������
	if (!edges.empty())
	{
		// �n�_����I�_�ւ̖��
		if (imguiCheck("Links Arrow", draw_links_arrow))
			draw_links_arrow ^= true;

		// �������N
		if (imguiCheck("Tentative Link", draw_tentative_link))
			draw_tentative_link ^= true;

		// �������N
		if (imguiCheck("horizontal_point", draw_horizontal_point))
			draw_horizontal_point ^= true;

		std::string text{ "build_time: " };

		text += std::to_string(auto_build_time_ms);

		text += "ms";

		imguiValue(text.data());
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
		// ���C��
		dd.begin(DU_DRAW_LINES, 2.0f);
		for (const auto& edge : edges)
		{
			constexpr float LineAdjY{ 2.f };

			auto start{ edge.start }, end{ edge.end };

			// �n�_�E�I�_
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
			for (const auto& point : edge.points)
			{
				constexpr float PointAdjY{ LineAdjY * 0.75f };
				constexpr UINT32 PointColor{ duRGBA(0, 255, 0, 200) };

				start = end = point;

				end[1] += PointAdjY;

				dd.vertex(start.data(), PointColor);
				dd.vertex(end.data(), PointColor);
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
					constexpr UINT32 ArrowColor{ duRGBA(255, 0, 255, 200) };

					start = edge.start;
					end = edge.end;

					auto vec{ end - start };
					const float len{ rcVdist(start, end) };

					rcVnormalize(&vec);

					const auto&& middle_pos{ start + (vec * (len * 0.5f)) };

					// �����x�N�g��
					{
						start = middle_pos;
						end = middle_pos + (edge.orthogonal_vec * edge.horizontal_dis);

						duAppendArrow(&dd,
							start[0], start[1], start[2],
							end[0], end[1], end[2],
							0.0f, 0.4f, ArrowColor);
					}

					// �����x�N�g��
					{
						constexpr Point Down{ 0.f, -1.f, 0.f };

						start = end;
						end = start + (Down * edge.vertical_dis);

						duAppendArrow(&dd,
							start[0], start[1], start[2],
							end[0], end[1], end[2],
							0.0f, 0.4f, ArrowColor);
					}
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
						0.25f, 0.6f, 0.6f, ArrowColor);
				}
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
				{
					constexpr UINT32 PointColor{ duRGBA(0, 0, 0, 200) };

					start = link.end;

					dd.vertex(start.data(), PointColor);
				}

				// �i�r���b�V���̍Őڋ߃|�C���g
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

	// �^�C�}�[�v���J�n
	ctx->resetTimers();
	auto_build_time_ms = 0.f;
	ctx->startTimer(RC_TIMER_TEMP);

	// �i�r���b�V���̃G�b�W�̍쐬
	CalcNavMeshEdges();

	// �G�b�W�𕪊�
	CalcEdgeDivision();

	// �I�_���v�Z���������N���쐬
	CalcEndPointAndTentativeLink();

	// ���肵���������N�ɒ����E�C��
	CheckTentativeLink();

	// OffMesh Link���쐬����
	BuildLink();

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

							edge.horizontal_dis = horizontal_distance;
							edge.vertical_dis = vertical_distance;
							edge.divistion_dis = divistion_distance;
							edge.link_end_error_dis = link_end_error_dis;
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

			// �������������X�ɒǉ�
			for (float dist = divistion_distance; dist < len; dist += divistion_distance)
			{
				edge.points.emplace_back(edge.start + (vec * dist));
			}
		}, exec::par);
}

void OffMeshConnectionTool::CalcEndPointAndTentativeLink()
{
	For_Each(edges, [this](NavMeshEdge& edge)
		{
			std::mutex mt;

			VF3 vec3{ ToXMFLOAT(edge.end - edge.start) };
			const VF2&& vec{ vec3.x, vec3.z }; // Y���͖���
			const VF2 orth_vec2d{ VectorNormalize(VectorOrthogonal(vec)) }; // XZ���ʏ�̒��s�x�N�g��

			edge.orthogonal_vec = { orth_vec2d.x, 0.f/*Y���͖���*/, orth_vec2d.y };

			edge.links.clear();
			edge.links.reserve(edge.points.size());

			// �e�����_����I�_���v�Z����
			For_Each(edge.points,
				[&edge, orth_vec2d, &mt, this](const Point& point)
				{
					const auto& geom{ sample->getInputGeom() };
					const float agent_radius{ sample->getAgentRadius() };
					const auto* navmesh_query{ sample->getNavMeshQuery() };

					InputGeom::RaycastMeshHitInfo hit_info{};

					// ������̃|�C���g�i���p�|�C���g�j
					auto horizontal_point
					{ point + (edge.orthogonal_vec * edge.horizontal_dis) };

					// ������̃|�C���g�܂łɒn�`�����݂��邩�H
#if false
					if (geom->RaycastMesh(point, horizontal_point, &hit_info))	return;
#else
					if (geom->RaycastMesh(point, horizontal_point, &hit_info))
					{
						// �����������W�Ƀ|�C���g�����肷��
						horizontal_point = hit_info.pos + (edge.orthogonal_vec * agent_radius);

						const float diameter{ agent_radius * 2.f };
						const float distance{ diameter + max_orthognal_error_dis }; // �Œ�ł��G�[�W�F���g�̒��a�͕K�v

						// �����_�ɋ߂�����
						if (rcVdistSqr(horizontal_point, point) <= distance * distance)	return;

						const auto&& horizon_vec{ horizontal_point - point };
						const VF2&& horizon_vec2d{ horizon_vec.front(), horizon_vec.back() };

						// �����_��������ɑ��݂���i�O�ׁ̈j
						if (VectorDot(orth_vec2d, horizon_vec2d) <= 0.f)	return;
					}
#endif

					auto inv_horizona_vec{ horizontal_point - point };

					rcVnormalize(&inv_horizona_vec);

					constexpr Point HalfExtents{ 0.5f, 0.5f, 0.5f };

					// �����������番���_�����t�߂܂Ō���
					for (float dis = 0; dis < edge.horizontal_dis - agent_radius * agent_radius;
						dis += HalfExtents.front())
					{
						constexpr Point Down{ 0.f, -1.f, 0.f };

						// ������̃|�C���g���^���x�N�g���̎n�_�ƏI�_
						const auto&& start{ horizontal_point + (inv_horizona_vec * dis) },
							&&end{ start + (Down * edge.vertical_dis) };

						// ������ɒn�`�����݂��Ȃ�
						if (!geom->RaycastMesh(start, end, &hit_info))	continue;

						const dtQueryFilter filter;
						dtPolyRef ref{};
						Point nearest_pos{};
						constexpr Point Zero{};

						// �i�r���b�V���Ƃ̔���
						const auto status{ navmesh_query->findNearestPoly(hit_info.pos.data(),
							HalfExtents.data(), &filter, &ref, nearest_pos.data()) };

						// �i�r���b�V����ɐ�����̃|�C���g�����݂��Ȃ�
						if (dtStatusFailed(status) || ref == 0 || nearest_pos == Zero)	continue;

						const float error_dis_sqr{ edge.link_end_error_dis * edge.link_end_error_dis };

						// �n�`�̃q�b�g�|�C���g�ƃi�r���b�V���̃q�b�g�|�C���g�̋���������
						if (rcVdistSqr(nearest_pos, hit_info.pos) > error_dis_sqr) continue;

						std::lock_guard<std::mutex> lg{ mt }; // �����j�~

						auto& link{ edge.links.emplace_back() };

						// OffMesh Link�̎n�_�ƏI�_���m��
						link.start = point;
						link.end = hit_info.pos;
						link.nearest_pos = nearest_pos;
						link.horizontal_pos = horizontal_point;

						break; // �\�z�I��
					}
				}, exec::par);
		}, exec::par);
}

void OffMeshConnectionTool::CheckTentativeLink()
{
}

void OffMeshConnectionTool::BuildLink()
{}

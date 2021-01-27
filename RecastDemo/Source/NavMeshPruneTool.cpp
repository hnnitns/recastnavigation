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
#include <cstring>
#include <cfloat>
#include <vector>

#include "SDL.h"
#include "SDL_opengl.h"
#include "imgui.h"
#include "NavMeshPruneTool.h"
#include "InputGeom.h"
#include "Sample.h"
#include "DetourNavMesh.h"
#include "DetourCommon.h"
#include "DetourAssert.h"
#include "DetourDebugDraw.h"
#include "OtherFiles\\AlgorithmHelper.hpp"

class NavmeshFlags
{
	struct TileFlags
	{
		std::vector<unsigned char> flags;
		dtPolyRef base;
	};

	const dtNavMesh* m_nav;
	std::vector<TileFlags> m_tiles;

public:
	NavmeshFlags() : m_nav(nullptr) {}

	~NavmeshFlags()
	{
		m_tiles.clear();
	}

	bool init(const dtNavMesh* nav)
	{
		const int ntiles{ nav->getMaxTiles() };

		if (ntiles == 0) return true;

		m_tiles.clear();

		try
		{
			m_tiles.resize(ntiles, {});
		}
		catch (const std::exception&)
		{
			return false;
		}

		// Alloc flags for each tile.
		// 各タイルにフラグを割り当てます。
		for (int i = 0; i < ntiles; ++i)
		{
			const dtMeshTile* tile = nav->getTile(i);
			if (!tile->header) continue;

			TileFlags& tf{ m_tiles[i] };

			const int nflags{ tile->header->polyCount };
			tf.base = nav->getPolyRefBase(tile);

			if (nflags != 0)
			{
				tf.flags.clear();

				try
				{
					tf.flags.resize(nflags, '\0');
				}
				catch (const std::exception&)
				{
					return false;
				}
			}
		}

		m_nav = nav;

		return false;
	}

	inline void clearAllFlags()
	{
		for (auto& tile : m_tiles)
		{
			if (!tile.flags.empty())
				NormalAlgorithm::Fill(tile.flags, '\0');
		}
	}

	inline unsigned char getFlags(dtPolyRef ref) const
	{
		dtAssert(m_nav);
		dtAssert(!m_tiles.empty());

		// Assume the ref is valid, no bounds checks.
		// 参照が有効であり、境界チェックがないと仮定します。
		unsigned int salt, it, ip;
		m_nav->decodePolyId(ref, salt, it, ip);
		return m_tiles[it].flags[ip];
	}

	inline void setFlags(dtPolyRef ref, SamplePolyFlags flags)
	{
		dtAssert(m_nav);
		dtAssert(!m_tiles.empty());

		// Assume the ref is valid, no bounds checks.
		// 参照が有効であり、境界チェックがないと仮定します。
		unsigned int salt, it, ip;
		m_nav->decodePolyId(ref, salt, it, ip);
		m_tiles[it].flags[ip] = static_cast<UINT8>(flags);
	}
};

namespace
{
	void floodNavmesh(dtNavMesh* nav, std::unique_ptr<NavmeshFlags>& flags, dtPolyRef start, const SamplePolyFlags flag)
	{
		// If already visited, skip.
		// 既にアクセスしている場合はスキップします。
		if (flags->getFlags(start)) return;

		flags->setFlags(start, flag);

		std::vector<dtPolyRef> openList;
		openList.emplace_back(start);

		while (!openList.empty())
		{
			const dtPolyRef ref = openList.back();
			openList.pop_back();

			// Get current poly and tile.
			// The API input has been cheked already, skip checking internal data.
			// 現在のポリゴンとタイルを取得します。
			// API入力は既にチェックされており、内部データのチェックをスキップします。
			const dtMeshTile* tile = 0;
			const dtPoly* poly = 0;
			nav->getTileAndPolyByRefUnsafe(ref, &tile, &poly);

			// Visit linked polygons.
			// リンクされたポリゴンにアクセスします。
			for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
			{
				const dtPolyRef neiRef = tile->links[i].ref;

				// Skip invalid and already visited.
				// 無効で既にアクセスしたものをスキップします。
				if (!neiRef || flags->getFlags(neiRef))
					continue;

				// Mark as visited
				// 訪問済みとしてマーク
				flags->setFlags(neiRef, flag);

				// ナビメッシュのフラグを有効に
				//nav->setPolyFlags(ref, static_cast<UINT8>(flag));

				// Visit neighbours
				// 隣人を訪問
				openList.emplace_back(neiRef);
			}
		}
	}

	void disableUnvisitedPolys(dtNavMesh* nav, const std::unique_ptr<NavmeshFlags>& flags)
	{
		for (int i = 0; i < nav->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = ((const dtNavMesh*)nav)->getTile(i);

			if (!tile->header) continue;

			const dtPolyRef base = nav->getPolyRefBase(tile);

			for (int j = 0; j < tile->header->polyCount; ++j)
			{
				const dtPolyRef ref = base | (unsigned int)j;
				if (!flags->getFlags(ref))
				{
					unsigned short f{};
					nav->getPolyFlags(ref, &f);
					nav->setPolyFlags(ref, f | SAMPLE_POLYFLAGS_DISABLED);
				}
			}
		}
	}
}

NavMeshPruneTool::NavMeshPruneTool() :
	m_sample(0),
	m_hitPosSet(false)
{}

NavMeshPruneTool::~NavMeshPruneTool()
{
	m_flags = nullptr;
}

void NavMeshPruneTool::init(Sample* sample)
{
	m_sample = sample;
}

void NavMeshPruneTool::reset()
{
	m_hitPosSet = false;
	m_flags = nullptr;
	m_flags = 0;
}

void NavMeshPruneTool::handleMenu()
{
	dtNavMesh* nav = m_sample->getNavMesh();
	if (!(nav && m_flags)) return;

	if (imguiButton("Clear Selection"))
	{
		m_flags->clearAllFlags();
	}

	if (imguiButton("Prune Unselected"))
	{
		disableUnvisitedPolys(nav, m_flags);
		m_flags = nullptr;
	}
}

void NavMeshPruneTool::handleClickDown(const float* s, const float* p, bool shift)
{
	rcIgnoreUnused(s);
	rcIgnoreUnused(shift);

	if (!m_sample) return;
	auto& geom = m_sample->getInputGeom();
	if (!geom) return;
	dtNavMesh* nav = m_sample->getNavMesh();
	if (!nav) return;
	dtNavMeshQuery* query = m_sample->getNavMeshQuery();
	if (!query) return;

	dtVcopy(m_hitPos, p);
	m_hitPosSet = true;

	if (!m_flags)
	{
		m_flags = std::make_unique<NavmeshFlags>();
		m_flags->init(nav);
	}

	constexpr float halfExtents[3]{ 2, 4, 2 };
	dtQueryFilter filter;
	dtPolyRef ref = 0;
	query->findNearestPoly(p, halfExtents, &filter, &ref, 0);

	floodNavmesh(nav, m_flags, ref, SAMPLE_POLYFLAGS_WALK);
}

void NavMeshPruneTool::handleToggle()
{
}

void NavMeshPruneTool::handleStep()
{
}

void NavMeshPruneTool::handleUpdate(const float /*dt*/)
{
}

void NavMeshPruneTool::handleRender()
{
	duDebugDraw& dd = m_sample->getDebugDraw();

	if (m_hitPosSet)
	{
		const float s = m_sample->getAgentRadius();
		constexpr unsigned int col = duRGBA(255, 0, 0, 255);
		dd.begin(DU_DRAW_LINES);
		dd.vertex(m_hitPos[0] - s, m_hitPos[1], m_hitPos[2], col);
		dd.vertex(m_hitPos[0] + s, m_hitPos[1], m_hitPos[2], col);
		dd.vertex(m_hitPos[0], m_hitPos[1] - s, m_hitPos[2], col);
		dd.vertex(m_hitPos[0], m_hitPos[1] + s, m_hitPos[2], col);
		dd.vertex(m_hitPos[0], m_hitPos[1], m_hitPos[2] - s, col);
		dd.vertex(m_hitPos[0], m_hitPos[1], m_hitPos[2] + s, col);
		dd.end();
	}

	const dtNavMesh* nav = m_sample->getNavMesh();
	if (m_flags && nav)
	{
		for (int i = 0; i < nav->getMaxTiles(); ++i)
		{
			const dtMeshTile* tile = nav->getTile(i);
			if (!tile->header) continue;
			const dtPolyRef base = nav->getPolyRefBase(tile);
			for (int j = 0; j < tile->header->polyCount; ++j)
			{
				const dtPolyRef ref = base | (unsigned int)j;
				if (m_flags->getFlags(ref))
				{
					duDebugDrawNavMeshPoly(&dd, *nav, ref, duRGBA(255, 255, 255, 128));
				}
			}
		}
	}
}

void NavMeshPruneTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	rcIgnoreUnused(model);
	rcIgnoreUnused(proj);

	// Tool help
	const int h = view[3];

	imguiDrawText(280, h - 40, IMGUI_ALIGN_LEFT, "LMB: Click fill area.", imguiRGBA(255, 255, 255, 192));
}
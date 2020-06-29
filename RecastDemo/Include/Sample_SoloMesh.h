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

#ifndef RECASTSAMPLESOLOMESH_H
#define RECASTSAMPLESOLOMESH_H

#include "Sample.h"
#include "DetourNavMesh.h"
#include "Recast.h"

class Sample_SoloMesh : public Sample
{
protected:
	bool m_keepInterResults;
	float m_totalBuildTimeMs;

	std::vector<uint8_t> m_triareas;
	rcHeightfield* m_solid;
	rcCompactHeightfield* m_chf;
	rcContourSet* m_cset;
	rcPolyMesh* m_pmesh;
	rcConfig m_cfg;
	rcPolyMeshDetail* m_dmesh;

	enum class DrawMode
	{
		NAVMESH,
		NAVMESH_TRANS,
		NAVMESH_BVTREE,
		NAVMESH_NODES,
		NAVMESH_INVIS,
		MESH,
		VOXELS,
		VOXELS_WALKABLE,
		COMPACT,
		COMPACT_DISTANCE,
		COMPACT_REGIONS,
		REGION_CONNECTIONS,
		RAW_CONTOURS,
		BOTH_CONTOURS,
		CONTOURS,
		POLYMESH,
		POLYMESH_DETAIL,
		MAX,
	};

	DrawMode m_drawMode;

	void CleanUp();

public:
	Sample_SoloMesh();
	virtual ~Sample_SoloMesh();

	virtual void handleSettings() override;
	virtual void handleTools() override;
	virtual void handleDebugMode() override;

	virtual void handleRender() override;
	virtual void handleRenderOverlay(double* proj, double* model, int* view) override;
	virtual void handleMeshChanged() override;
	virtual bool handleBuild() override;

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Sample_SoloMesh(const Sample_SoloMesh&) = delete;
	Sample_SoloMesh& operator=(const Sample_SoloMesh&) = delete;
};

#endif // RECASTSAMPLESOLOMESHSIMPLE_H

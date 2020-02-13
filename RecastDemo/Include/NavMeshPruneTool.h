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

#ifndef NAVMESHPRUNETOOL_H
#define NAVMESHPRUNETOOL_H

#include "Sample.h"

// Prune navmesh to accessible locations from a point.

class NavMeshPruneTool : public SampleTool
{
	Sample* m_sample;

	class NavmeshFlags* m_flags;

	std::array<float, 3> m_hitPos;
	bool m_hitPosSet;

public:
	NavMeshPruneTool();
	virtual ~NavMeshPruneTool();

	int type() override { return TOOL_NAVMESH_PRUNE; }
	void init(Sample* sample) override;
	void reset() override;
	void handleMenu() override;
	void handleClick(const float* s, const float* p, bool shift) override;
	void handleToggle() override;
	void handleStep() override;
	void handleUpdate(const float dt) override;
	void handleRender() override;
	void handleRenderOverlay(double* proj, double* model, int* view) override;

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	NavMeshPruneTool(const NavMeshPruneTool&) = delete;
	NavMeshPruneTool& operator=(const NavMeshPruneTool&) = delete;
};

#endif // NAVMESHPRUNETOOL_H

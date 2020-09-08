#include <DetourTileCache.h>
#include <DetourCommon.h>

#include "TempObstacleCreateTool.h"
#include "imgui.h"

TempObstacleCreateTool::TempObstacleCreateTool()
	: m_sample(), add_pos(), m_ref((std::numeric_limits<dtObstacleRef>::max)())
{
	dtVset(box_size, 3.f, 5.f, 3.f);

	add_data.cylinder = { {}, {}, 1.f, 2.0f };
	add_data.box = {};
	add_data.type = DT_OBSTACLE_CYLINDER;
}

TempObstacleCreateTool::~TempObstacleCreateTool()
{
}

void TempObstacleCreateTool::init(Sample* sample)
{
	m_sample = (Sample_TempObstacles*)sample;
}

void TempObstacleCreateTool::handleMenu()
{
	imguiLabel("Create Temp Obstacles");

	if (imguiButton("Remove All"))
		m_sample->clearAllTempObstacles();

	imguiSeparator();

	if (imguiButton("Change Obstacles Type"))
	{
		ObstacleType& type{ add_data.type };

		if (type == DT_OBSTACLE_CYLINDER)
			type = DT_OBSTACLE_BOX;
		else
			type = DT_OBSTACLE_CYLINDER;
	}

	if (add_data.type == DT_OBSTACLE_CYLINDER)
	{
		imguiValue("Cylinder");

		auto& data{ add_data.cylinder };

		imguiSlider("height", &data.height, 1.5f, 10.f, 0.1f);
		imguiSlider("radius", &data.radius, 0.5f, 8.f, 0.1f);
	}
	else
	{
		imguiValue("Box");

		imguiSlider("Size : x", &box_size[0], 2.5f, 12.f, 0.1f);
		imguiSlider("Size : y", &box_size[1], 3.0f, 15.f, 0.1f);
		imguiSlider("Size : z", &box_size[2], 2.5f, 12.f, 0.1f);
	}

	imguiSeparator();

	imguiValue("Click LMB to create an obstacle.");
	imguiValue("Shift+LMB to remove an obstacle.");
}

void TempObstacleCreateTool::handleClickDown(const float* s, const float* p, bool shift)
{
	if (!m_sample)	return;

	if (mouse_middle_push)
	{
		m_ref = m_sample->HitTestObstacle(s, p);
		m_sample->StartMoveObstacles();
		return;
	}

	rcVcopy(add_pos, p);

	if (shift)
		m_sample->removeTempObstacle(s, p);
	else
	{
		// cylinder
		if (add_data.type == DT_OBSTACLE_CYLINDER)
		{
			rcVcopy(add_data.cylinder.pos, add_pos);
		}
		else // box
		{
			m_sample->CalcBoxPos(add_pos, box_size, &add_data.box);
		}

		m_sample->addTempObstacle(add_data);
	}
}

void TempObstacleCreateTool::handleClickUp(const float*, const float*)
{
	if (mouse_middle_push)
		m_sample->EndMoveObstacles();
}

void TempObstacleCreateTool::handleClick(const float*, const float* p)
{
	if (mouse_middle_push)
		m_sample->MoveTempObstacle(m_ref, p);
}
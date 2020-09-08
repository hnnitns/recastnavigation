#include "TempObstacleTool.h"

void TempObstacleTool::init(Sample* sample)
{
	m_sample = (Sample_TempObstacles*)sample;
	create_tool = std::make_unique<TempObstacleCreateTool>();
	create_tool->init(sample);
}

void TempObstacleTool::reset()
{
	create_tool->reset();
}

void TempObstacleTool::handleMenu()
{
	create_tool->handleMenu();
}

void TempObstacleTool::handleClickDown(const float* s, const float* p, bool shift)
{
	create_tool->handleClickDown(s, p, shift);
}

void TempObstacleTool::handleClickUp(const float* s, const float* p)
{
	create_tool->handleClickUp(s, p);
}

void TempObstacleTool::handleClick(const float* s, const float* p)
{
	create_tool->handleClick(s, p);
}

void TempObstacleTool::handleToggle()
{
	create_tool->handleToggle();
}

void TempObstacleTool::handleStep()
{
	create_tool->handleStep();
}

void TempObstacleTool::handleUpdate(const float dt)
{
	create_tool->handleUpdate(dt);
}

void TempObstacleTool::handleRender()
{
	create_tool->handleRender();
}

void TempObstacleTool::handleRenderOverlay(double* proj, double* model, int* view)
{
	create_tool->handleRenderOverlay(proj, model, view);
}

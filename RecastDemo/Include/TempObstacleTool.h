#pragma once

#include "Sample.h"
#include "TempObstacleCreateTool.h"

class TempObstacleTool final :
    public SampleTool
{
private:
	class Sample_TempObstacles* m_sample;
	std::unique_ptr<class TempObstacleCreateTool> create_tool;

public:
	int type() override { return TOOL_TEMP_OBSTACLE; }

	void init(Sample* sample) override;

	void reset() override;

	void handleMenu() override;

	void handleClickDown(const float* s, const float* p, bool shift) override;
	void handleClickUp(const float* s, const float* p)  override;
	void handleClick(const float* s, const float* p)  override;

	void handleToggle() override;

	void handleStep() override;

	void handleUpdate(const float dt) override;

	void handleRender() override;

	void handleRenderOverlay(double* proj, double* model, int* view) override;
};


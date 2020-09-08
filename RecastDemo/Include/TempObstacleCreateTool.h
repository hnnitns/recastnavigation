#pragma once

#include "Sample.h"
#include "Sample_TempObstacles.h"

class TempObstacleCreateTool
	: public SampleTool
{
	class Sample_TempObstacles* m_sample;
	AddObstacleData add_data;
	float add_pos[3], box_size[3];
	dtObstacleRef m_ref;

public:

	TempObstacleCreateTool();
	~TempObstacleCreateTool();

	int type() override { return TOOL_TEMP_OBSTACLE; }

	void init(Sample* sample) override;

	void reset() override {}

	void handleMenu() override;

	void handleClickDown(const float* s, const float* p, bool shift) override;

	void handleClickUp(const float* /*s*/, const float* /*p*/) override;

	void handleClick(const float* /*s*/, const float* p) override;

	void handleToggle() override {}
	void handleStep() override {}
	void handleUpdate(const float /*dt*/) override {}
	void handleRender() override {}
	void handleRenderOverlay(double* /*proj*/, double* /*model*/, int* /*view*/) override { }
};

#include "NavMeshFindManager.h"
#include "NavMeshTesterTool.h"

NavMeshFindManager::NavMeshFindManager()
	: m_sample()
{}

bool NavMeshFindManager::Init(Sample* sample)
{
	if (m_sample != sample)
	{
		m_sample = sample;
	}

	if (!sample) return false;

	if (!m_state) m_state = std::make_unique<NavMeshTesterTool>();

	m_state->init(sample);

	return true;
}

void NavMeshFindManager::Update(const float dt)
{
	if (m_state)	m_state->handleUpdate(dt);
}

void NavMeshFindManager::ImGuiUpdate()
{
	if (m_state)	m_state->handleMenu();
}

void NavMeshFindManager::Draw()
{
	if (m_state)	m_state->handleRender();
}

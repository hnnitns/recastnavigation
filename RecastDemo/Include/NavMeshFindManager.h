#pragma once

#include <memory>

class NavMeshFindManager final
{
private:
	static constexpr int MAX_POLYS{ 256 };
	static constexpr int MAX_SMOOTH{ 2048 };
	static constexpr int MAX_RAND_POINTS{ 64 };
	static constexpr int MAX_STEER_POINTS{ 10 };

public:
	NavMeshFindManager();
	~NavMeshFindManager() = default;
	NavMeshFindManager(const NavMeshFindManager&) = delete;
	auto& operator=(const NavMeshFindManager&) = delete;
	NavMeshFindManager(NavMeshFindManager&&) = delete;
	auto& operator=(NavMeshFindManager&&) = delete;

public:
	bool Init(class Sample* sample);

	// �X�V
	void Update(const float dt);

	// ImGui�p�̍X�V
	void ImGuiUpdate();

	// �`��
	void Draw();

private:
	class Sample* m_sample;
	std::unique_ptr<class NavMeshTesterTool> m_state;
};


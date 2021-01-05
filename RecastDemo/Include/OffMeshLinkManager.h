#pragma once

#include <array>

class OffMeshLinkManager final
{
public:
	OffMeshLinkManager();
	~OffMeshLinkManager();
	OffMeshLinkManager(const OffMeshLinkManager&) = delete;
	OffMeshLinkManager(OffMeshLinkManager&&) = delete;
	auto& operator=(const OffMeshLinkManager&) = delete;
	auto& operator=(OffMeshLinkManager&&) = delete;

public:
	bool Init(class Sample* sample);
	void Draw();

	// �I�t���b�V�������N�̒ǉ�
	int AddLink(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
		const bool is_bidirectional);
	// �I�t���b�V�������N�̍폜
	void DeleteLink(const int index);
	// �����N�Ɠ_�̔���
	bool HitLinkPoint(const std::array<float, 3>& point, const int index);
	// ���������N�Ɠ_�̔���
	bool HitLinksPoint(const std::array<float, 3>& point, int* index);
	// �������N���擾
	int GetLinkSize() const noexcept;

private:
	class Sample* m_sample;
	unsigned char m_oldFlags;
};


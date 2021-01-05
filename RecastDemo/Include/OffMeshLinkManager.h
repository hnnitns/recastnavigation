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

	// オフメッシュリンクの追加
	int AddLink(const std::array<float, 3>& start_pos, const std::array<float, 3>& end_pos,
		const bool is_bidirectional);
	// オフメッシュリンクの削除
	void DeleteLink(const int index);
	// リンクと点の判定
	bool HitLinkPoint(const std::array<float, 3>& point, const int index);
	// 複数リンクと点の判定
	bool HitLinksPoint(const std::array<float, 3>& point, int* index);
	// 総リンク数取得
	int GetLinkSize() const noexcept;

private:
	class Sample* m_sample;
	unsigned char m_oldFlags;
};


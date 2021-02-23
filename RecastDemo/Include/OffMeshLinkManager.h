#pragma once

#include <array>
#include <vector>

class OffMeshLinkManager final
{
private:
	using Point = std::array<float, 3>;

private:
	struct NavMeshEdge final
	{
		struct Link final
		{
			Link() = default;
			~Link() = default;

			Point start{}/*�n�_*/, end{}/*�I�_*/;
			Point nearest_pos{}/*������̃|�C���g�ɍł��߂��_*/, horizontal_pos{}/*������̃|�C���g*/;
			bool is_delete{}, is_bidir{};
			float end_edge_dist{}, end_edge_angle{};
			NavMeshEdge* end_edge{}, * base_edge{};
		};
		struct DivisionPoint final
		{
			Point base_point{}, height_point{};
		};

		Point start{}, end{};
		Point orthogonal_vec{};

		std::vector<DivisionPoint> points;
		std::vector<Link> links;
	};
	struct NotBuildArea final
	{
		static constexpr size_t MaxHitPoint{ 2u };

		std::array<Point, 2> hit_points{};
		int nhit_points{};
		bool is_built{};
		std::array<Point, 8> aabb_vertex{};
		Point aabb_max{}, aabb_min{}, nonbuild_area_max{}, nonbuild_area_min{};
	};

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
	// ���̃I�t���b�V�������N�̎�������
	void AutoTentativeLinksBuild();
	// ���̃I�t���b�V�������N�����ۂɍ\�z����
	void ReBuildNavMeshLink();
	// �������������I�t���b�V�������N���폜����
	void ClearBuiltAutoLink();

private:
	void CalcNavMeshEdges();
	void CalcEdgeDivision();
	void CalcTentativeLink();
	void CheckTentativeLink();
	void BuildLink();

private:
	class Sample* sample{};
	class Sample_TempObstacles* obstacle_sample{};
	unsigned char old_flags{};
	float auto_build_time_ms{};

	bool draw_links_arrow, draw_tentative_link, draw_horizontal_point, draw_edge_point,
		draw_division_point, draw_end_point, draw_navmesh_nearest_point, draw_error_dis{}, draw_all;

	// �������N�Ԃ́u�����сv�����e���邩�H
	bool is_buildable_height_limit{};
	// �I���n�_�t�߂Ƀi�r���b�V���G�b�W�����݂��Ȃ��Ă��A�o�����ʍs�ɂ��邩�H
	bool is_non_navedge_bidirectional{ true };

	float horizontal_dis{ 5.f };            // �\�z�\�Ȑ�������
	float vertical_dis{ 7.5f };             // �\�z�\�Ȑ�������
	float divistion_dis{ 1.2f };            // �����_�Ԃ̒���
	float climbable_height{ 0.5f };         // �����_����\�z���鎞�̍����i�o��鍂���j
	float	min_buildable_height{ 0.5f };     //�u�����сv�����e���鎞�̍���
	float	link_end_error_dis{ 0.2f };       // �n�`�̓�������W�ƃi�r���b�V���̓�������W�Ԃ̋��e�͈�
	float	orthognal_error_dis{ 0.5f };      // �����x�N�g���ō\�z�s�ɂȂ鋖�e�͈�
	float	link_equal_error_dis{ 0.25f };    // ���̉������N�Ƃ̏d�Ȃ��F�����鋖�e�͈�
	float max_start_link_error{ 0.2f };     // �n�_���G�b�W����ǂꂾ��������
	float max_end_link_error{ 0.2f };       // �I�_���G�b�W����ǂꂾ��������
	float limit_link_angle{ 3.1415 / 4.f }; // �����N�̎n�_�E�I�_�̃i�r���b�V���G�b�W�̐����p�x
	float max_link_end_edge_dis{ 0.5f };    // �����N�̏I�_���ł��߂��G�b�W���Ɣ��f�o����ő勗��
	size_t edge_link_accumulate{};          // �������N��
	std::vector<NavMeshEdge> edges;

	std::vector<NotBuildArea> not_build_areas;
	float box_descent{ 0.25f };     // ���̉����̍���
	float box_height{ 5.f };        // ���̏㕔�̍���
	float box_nonbuild_dis{ 1.2f }; // ������\�z�s�̒���
	bool is_non_build_area{};
};


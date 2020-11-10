#pragma once

#include <array>

struct RaycastMeshHitInfo
{
	float dis{}; // �Փ˒n�_�܂ł̋���
	std::array<float, 3> pos{}; // �Փ˒n�_
	std::array<float, 3> vec{}; // �Փ˂ւ̃x�N�g��
};

struct dtObstacleCylinder
{
	float pos[3], before_pos[3];
	float radius;
	float height;
};

struct dtObstacleBox
{
	float bmin[3], bmax[3];
	float before_bmin[3], before_bmax[3];
};

struct dtObstacleOrientedBox
{
	float center[3];
	float halfExtents[3];
	float rotAux[2]; //{ cos(0.5f*angle)*sin(-0.5f*angle); cos(0.5f*angle)*cos(0.5f*angle) - 0.5 }
	float y_radian;
};

enum ObstacleType
{
	DT_OBSTACLE_CYLINDER,
	DT_OBSTACLE_BOX, // AABB
	DT_OBSTACLE_ORIENTED_BOX, // OBB
};

struct AddObstacleData
{
	dtObstacleCylinder cylinder;
	dtObstacleBox box;
	dtObstacleOrientedBox oriented_box;
	ObstacleType type;
};

struct AddAgentStruct
{
	std::array<float, 3> pos{}; // ���ݍ��W
	float radius{}; // ���a
	float height{}; // ����
	float max_accele{ 8.f }; // �ő�����x
	float max_speed{ 3.5f }; // �ő呬�x
	float collision_range{ 12.f }; // �X�e�A�����O����ƌ��Ȃ����O�ɏՓ˗v�f���ǂꂾ���߂��Ȃ���΂Ȃ�Ȃ���
	float path_optimization_range{ 30.f }; // �p�X�̉����̍œK���͈�
};
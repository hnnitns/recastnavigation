//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef DETOURSTATUS_H
#define DETOURSTATUS_H

#include <cstdint>

typedef uint32_t dtStatus;

// High level status.
// �����x���̃X�e�[�^�X

constexpr uint32_t DT_FAILURE = 1u << 31;			// Operation failed.  ����Ɏ��s���܂���
constexpr uint32_t DT_SUCCESS = 1u << 30;			// Operation succeed. ����͐������܂���
constexpr uint32_t DT_IN_PROGRESS = 1u << 29;		// Operation still in progress. ����͂܂��i�s���ł�

// Detail information for status.
// �X�e�[�^�X�̏ڍ׏��

constexpr uint32_t DT_STATUS_DETAIL_MASK = 0x0ffffff;
constexpr uint32_t DT_WRONG_MAGIC = 1 << 0;		// Input data is not recognized.  ���̓f�[�^���F������܂���
constexpr uint32_t DT_WRONG_VERSION = 1 << 1;	// Input data is in wrong version. ���̓f�[�^�̃o�[�W�������Ԉ���Ă��܂�
constexpr uint32_t DT_OUT_OF_MEMORY = 1 << 2;	// Operation ran out of memory. ����Ń��������s�����܂���
constexpr uint32_t DT_INVALID_PARAM = 1 << 3;	// An input parameter was invalid. ���̓p�����[�^�[�������ł���
constexpr uint32_t DT_BUFFER_TOO_SMALL = 1 << 4;	// Result buffer for the query was too small to store all results. �N�G���̌��ʃo�b�t�@�����������āA���ׂĂ̌��ʂ��i�[�ł��܂���ł���
constexpr uint32_t DT_OUT_OF_NODES = 1 << 5;		// Query ran out of nodes during search. �������ɃN�G�����m�[�h���g���ʂ����܂���
constexpr uint32_t DT_PARTIAL_RESULT = 1 << 6;	// Query did not reach the end location, returning best guess. �N�G���͏I���ʒu�ɓ��B�����A�ŗǂ̐�����Ԃ��܂���

// Returns true of status is success.
// �X�e�[�^�X�������̏ꍇ��true��Ԃ��܂�
inline bool dtStatusSucceed(dtStatus status)
{
	return (status & DT_SUCCESS) != 0;
}

// Returns true of status is failure.
// �X�e�[�^�X�����s�̏ꍇ��true��Ԃ��܂��B
inline bool dtStatusFailed(dtStatus status)
{
	return (status & DT_FAILURE) != 0;
}

// Returns true of status is in progress.
// �X�e�[�^�X���i�s���̏ꍇ��true��Ԃ��܂�
inline bool dtStatusInProgress(dtStatus status)
{
	return (status & DT_IN_PROGRESS) != 0;
}

// Returns true if specific detail is set.
// ����̏ڍׂ��ݒ肳��Ă���ꍇ�Atrue��Ԃ��܂�
inline bool dtStatusDetail(dtStatus status, uint32_t detail)
{
	return (status & detail) != 0;
}

#endif // DETOURSTATUS_H

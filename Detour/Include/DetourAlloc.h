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

#ifndef DETOURALLOCATOR_H
#define DETOURALLOCATOR_H

#include <cstddef>

// Provides hint values to the memory allocator on how long the memory is expected to be used.
// �������̎g�p���\�z�������ԂɊւ���q���g�l���������A���P�[�^�ɒ񋟂��܂��B
enum dtAllocHint
{
	// Memory persist after a function call.
	// �֐��Ăяo������������͎������܂��B
	DT_ALLOC_PERM,
	// Memory used temporarily within a function.
	// �֐����ňꎞ�I�Ɏg�p����郁�����B
	DT_ALLOC_TEMP
};

// A memory allocation function.
// ���������蓖�Ċ֐��B
//  @param[in] size : The size, in bytes of memory, to allocate.
//	���蓖�Ă郁�����̃T�C�Y�i�o�C�g�P�ʁj�B
//  @param[in] rcAllocHint : A hint to the allocator on how long the memory is expected to be in use.
//	�������[�̎g�p���\�z�������ԂɊւ���A���P�[�^�[�ւ̃q���g�B
//  @return A pointer to the beginning of the allocated memory block, or null if the allocation failed.
//	���蓖�Ă�ꂽ�������u���b�N�̐擪�ւ̃|�C���^�A�܂��͊��蓖�Ă����s�����ꍇ��null�B
//  @see dtAllocSetCustom
typedef void* (dtAllocFunc)(size_t size, dtAllocHint hint);

// A memory deallocation function.
// ���������蓖�ĉ����֐��B
//  @param[in] ptr : A pointer to a memory block previously allocated using #dtAllocFunc.
//	�ȑO��#dtAllocFunc���g�p���Ċ��蓖�Ă�ꂽ�������u���b�N�ւ̃|�C���^�B
// @see dtAllocSetCustom
typedef void (dtFreeFunc)(void* ptr);

// Sets the base custom allocation functions to be used by Detour.
// Detour���g�p����x�[�X�̃J�X�^�����蓖�Ċ֐���ݒ肵�܂��B
//  @param[in] allocFunc : The memory allocation function to be used by #dtAlloc
//	#dtAlloc���g�p���郁�������蓖�Ċ֐�
//  @param[in] freeFunc : The memory de-allocation function to be used by #dtFree
//	#dtFree�ɂ���Ďg�p����郁�������蓖�ĉ����֐�
void dtAllocSetCustom(dtAllocFunc* allocFunc, dtFreeFunc* freeFunc);

// Allocates a memory block.
// �������u���b�N�����蓖�Ă܂��B
//  @param[in] size : The size, in bytes of memory, to allocate.
//	���蓖�Ă�T�C�Y�i�������̃o�C�g�P�ʁj�B
//  @param[in] hint : A hint to the allocator on how long the memory is expected to be in use.
//	�������[���g�p�����Ɨ\�z�������ԂɊւ���A���P�[�^�[�ւ̃q���g�B
//  @return A pointer to the beginning of the allocated memory block, or null if the allocation failed.
//	���蓖�Ă�ꂽ�������u���b�N�̐擪�ւ̃|�C���^�A�܂��͊��蓖�Ă����s�����ꍇ��null�B
// @see dtFree
void* dtAlloc(size_t size, dtAllocHint hint);

// Deallocates a memory block.
// �������u���b�N�̊��蓖�Ă��������܂��B
//  @param[in] ptr : A pointer to a memory block previously allocated using #dtAlloc.
//	�ȑO��#dtAlloc���g�p���Ċ��蓖�Ă�ꂽ�������u���b�N�ւ̃|�C���^�B
// @see dtAlloc
void dtFree(void* ptr);

#endif

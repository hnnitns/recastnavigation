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

#ifndef RECASTALLOC_H
#define RECASTALLOC_H

#include <stddef.h>

// Provides hint values to the memory allocator on how long the memory is expected to be used.
// ���������g�p�����Ɨ\�z�������ԂɊւ���q���g���������A���P�[�^�ɒ񋟂��܂��B
enum rcAllocHint
{
	RC_ALLOC_PERM,		// Memory will persist after a function call. // �������͊֐��Ăяo������������܂��B
	RC_ALLOC_TEMP		// Memory used temporarily within a function. // �֐����ňꎞ�I�Ɏg�p����郁�����B
};

// A memory allocation function.
//	���������蓖�Ċ֐��B
//  @param[in]		size			The size, in bytes of memory, to allocate.
//	���蓖�Ă�T�C�Y�i�������̃o�C�g�P�ʁj�B
//  @param[in]		rcAllocHint	A hint to the allocator on how long the memory is expected to be in use.
//	�������[���g�p�����Ɨ\�z�������ԂɊւ���A���P�[�^�[�ւ̃q���g�B
//  @return A pointer to the beginning of the allocated memory block, or null if the allocation failed.
//	���蓖�Ă�ꂽ�������u���b�N�̐擪�ւ̃|�C���^�A�܂��͊��蓖�Ă����s�����ꍇ��null�B
//  @see rcAllocSetCustom
typedef void* (rcAllocFunc)(size_t size, rcAllocHint hint);

// A memory deallocation function.
//	���������蓖�ĉ����֐��B
//  @param[in]		ptr		A pointer to a memory block previously allocated using #rcAllocFunc.
//	#rcAllocFunc���g�p���ĈȑO�Ɋ��蓖�Ă�ꂽ�������u���b�N�ւ̃|�C���^�[�B
// @see rcAllocSetCustom
typedef void (rcFreeFunc)(void* ptr);

// Sets the base custom allocation functions to be used by Recast.
//	Recast���g�p����x�[�X�̃J�X�^�����蓖�Ċ֐���ݒ肵�܂��B
//  @param[in]		allocFunc	The memory allocation function to be used by #rcAlloc
//	#rcAlloc�ɂ���Ďg�p����郁�������蓖�Ċ֐��B
//  @param[in]		freeFunc	The memory de-allocation function to be used by #rcFree
//	#rcFree�ɂ���Ďg�p����郁�������蓖�ĉ����֐�
void rcAllocSetCustom(rcAllocFunc* allocFunc, rcFreeFunc* freeFunc);

// Allocates a memory block.
// �������u���b�N�����蓖�Ă܂��B
//  @param[in]		size	The size, in bytes of memory, to allocate.
//	���蓖�Ă�T�C�Y�i�������̃o�C�g�P�ʁj�B
//  @param[in]		hint	A hint to the allocator on how long the memory is expected to be in use.
//	�������[���g�p�����Ɨ\�z�������ԂɊւ���A���P�[�^�[�ւ̃q���g�B
//  @return A pointer to the beginning of the allocated memory block, or null if the allocation failed.
//	���蓖�Ă�ꂽ�������u���b�N�̐擪�ւ̃|�C���^�A�܂��͊��蓖�Ă����s�����ꍇ��null�B
// @see rcFree
void* rcAlloc(size_t size, rcAllocHint hint);

// Deallocates a memory block.
// �������u���b�N�̊��蓖�Ă��������܂��B
//  @param[in]		ptr		A pointer to a memory block previously allocated using #rcAlloc.
//	#rcAlloc���g�p���ĈȑO�Ɋ��蓖�Ă�ꂽ�������u���b�N�ւ̃|�C���^�[�B
// @see rcAlloc
void rcFree(void* ptr);

// A simple dynamic array of integers.
// �����̒P���ȓ��I�z��B
class rcIntArray
{
	int* m_data;
	int m_size, m_cap;

	void doResize(int n);

	// Explicitly disabled copy constructor and copy assignment operator.
	// �R�s�[�R���X�g���N�^�[�ƃR�s�[���蓖�ĉ��Z�q�𖾎��I�ɖ����ɂ��܂��B
	rcIntArray(const rcIntArray&);
	rcIntArray& operator=(const rcIntArray&);

public:
	// Constructs an instance with an initial array size of zero.
	// �����z��T�C�Y���[���̃C���X�^���X���\�z���܂��B
	rcIntArray() : m_data(0), m_size(0), m_cap(0) {}

	// Constructs an instance initialized to the specified size.
	// �w�肳�ꂽ�T�C�Y�ɏ��������ꂽ�C���X�^���X���\�z���܂��B
	//  @param[in]		n	The initial size of the integer array.
	// �����z��̏����T�C�Y�B
	rcIntArray(int n) : m_data(0), m_size(0), m_cap(0) { resize(n); }
	~rcIntArray() { rcFree(m_data); }

	// Specifies the new size of the integer array.
	// �����z��̐V�����T�C�Y���w�肵�܂��B
	//  @param[in]		n	The new size of the integer array.
	// �����z��̐V�����T�C�Y�B
	void resize(int n)
	{
		if (n > m_cap)
			doResize(n);

		m_size = n;
	}

	// Push the specified integer onto the end of the array and increases the size by one.
	// �w�肳�ꂽ������z��̍Ō�Ƀv�b�V�����A�T�C�Y��1���₵�܂��B
	//  @param[in]		item	The new value. //�V�����l�B
	void push(int item) { resize(m_size + 1); m_data[m_size - 1] = item; }

	// Returns the value at the end of the array and reduces the size by one.
	// �z��̍Ō�ɒl��Ԃ��A�T�C�Y��1���炵�܂��B
	//  @return The value at the end of the array. // �z��̍Ō�̒l�B
	int pop()
	{
		if (m_size > 0)
			m_size--;

		return m_data[m_size];
	}

	// The value at the specified array index.
	//	�w�肳�ꂽ�z��C���f�b�N�X�̒l�B
	// @warning Does not provide overflow protection.
	//	�I�[�o�[�t���[�ی��񋟂��܂���B
	//  @param[in]		i	The index of the value.
	//	�l�̃C���f�b�N�X�B
	const int& operator[](int i) const { return m_data[i]; }

	// The value at the specified array index.
	//	�w�肳�ꂽ�z��C���f�b�N�X�̒l�B
	// @warning Does not provide overflow protection.
	//	�I�[�o�[�t���[�ی��񋟂��܂���B
	//  @param[in]		i	The index of the value.
	//	�l�̃C���f�b�N�X�B
	int& operator[](int i) { return m_data[i]; }

	// The current size of the integer array.
	// �����z��̌��݂̃T�C�Y�B
	int size() const noexcept { return m_size; }
};

// A simple helper class used to delete an array when it goes out of scope.
// �z�񂪔͈͊O�ɂȂ����Ƃ��ɔz����폜���邽�߂Ɏg�p�����P���ȃw���p�[�N���X�B
// @note This class is rarely if ever used by the end user.
// @note ���̃N���X�́A�G���h���[�U�[���g�p���邱�Ƃ͂قƂ�ǂ���܂���B
template<class T> class rcScopedDelete
{
	T* ptr;
public:

	// Constructs an instance with a null pointer.
	// null�|�C���^�[���g�p���ăC���X�^���X���\�z���܂��B
	inline rcScopedDelete() : ptr(nullptr) {}

	// Constructs an instance with the specified pointer.
	//  @param[in]		p	An pointer to an allocated array.
	// �w�肳�ꂽ�|�C���^�ŃC���X�^���X���\�z���܂��B
	// @param [in]		p	���蓖�Ă�ꂽ�z��ւ̃|�C���^�[�B
	inline rcScopedDelete(T* p) : ptr(p) {}
	inline ~rcScopedDelete() { rcFree(ptr); }

	// The root array pointer.
	//  @return The root array pointer.
	// ���[�g�z��|�C���^�B
	// @return ���[�g�z��|�C���^�B
	inline operator T* () { return ptr; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcScopedDelete(const rcScopedDelete&) = delete;
	rcScopedDelete& operator=(const rcScopedDelete&) = delete;
};

#endif

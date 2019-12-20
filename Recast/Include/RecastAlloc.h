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
// メモリが使用されると予想される期間に関するヒントをメモリアロケータに提供します。
enum rcAllocHint
{
	RC_ALLOC_PERM,		// Memory will persist after a function call. // メモリは関数呼び出し後も持続します。
	RC_ALLOC_TEMP		// Memory used temporarily within a function. // 関数内で一時的に使用されるメモリ。
};

// A memory allocation function.
//	メモリ割り当て関数。
//  @param[in]		size			The size, in bytes of memory, to allocate.
//	割り当てるサイズ（メモリのバイト単位）。
//  @param[in]		rcAllocHint	A hint to the allocator on how long the memory is expected to be in use.
//	メモリーが使用されると予想される期間に関するアロケーターへのヒント。
//  @return A pointer to the beginning of the allocated memory block, or null if the allocation failed.
//	割り当てられたメモリブロックの先頭へのポインタ、または割り当てが失敗した場合はnull。
//  @see rcAllocSetCustom
typedef void* (rcAllocFunc)(size_t size, rcAllocHint hint);

// A memory deallocation function.
//	メモリ割り当て解除関数。
//  @param[in]		ptr		A pointer to a memory block previously allocated using #rcAllocFunc.
//	#rcAllocFuncを使用して以前に割り当てられたメモリブロックへのポインター。
// @see rcAllocSetCustom
typedef void (rcFreeFunc)(void* ptr);

// Sets the base custom allocation functions to be used by Recast.
//	Recastが使用するベースのカスタム割り当て関数を設定します。
//  @param[in]		allocFunc	The memory allocation function to be used by #rcAlloc
//	#rcAllocによって使用されるメモリ割り当て関数。
//  @param[in]		freeFunc	The memory de-allocation function to be used by #rcFree
//	#rcFreeによって使用されるメモリ割り当て解除関数
void rcAllocSetCustom(rcAllocFunc* allocFunc, rcFreeFunc* freeFunc);

// Allocates a memory block.
// メモリブロックを割り当てます。
//  @param[in]		size	The size, in bytes of memory, to allocate.
//	割り当てるサイズ（メモリのバイト単位）。
//  @param[in]		hint	A hint to the allocator on how long the memory is expected to be in use.
//	メモリーが使用されると予想される期間に関するアロケーターへのヒント。
//  @return A pointer to the beginning of the allocated memory block, or null if the allocation failed.
//	割り当てられたメモリブロックの先頭へのポインタ、または割り当てが失敗した場合はnull。
// @see rcFree
void* rcAlloc(size_t size, rcAllocHint hint);

// Deallocates a memory block.
// メモリブロックの割り当てを解除します。
//  @param[in]		ptr		A pointer to a memory block previously allocated using #rcAlloc.
//	#rcAllocを使用して以前に割り当てられたメモリブロックへのポインター。
// @see rcAlloc
void rcFree(void* ptr);

// A simple dynamic array of integers.
// 整数の単純な動的配列。
class rcIntArray
{
	int* m_data;
	int m_size, m_cap;

	void doResize(int n);

	// Explicitly disabled copy constructor and copy assignment operator.
	// コピーコンストラクターとコピー割り当て演算子を明示的に無効にします。
	rcIntArray(const rcIntArray&);
	rcIntArray& operator=(const rcIntArray&);

public:
	// Constructs an instance with an initial array size of zero.
	// 初期配列サイズがゼロのインスタンスを構築します。
	rcIntArray() : m_data(0), m_size(0), m_cap(0) {}

	// Constructs an instance initialized to the specified size.
	// 指定されたサイズに初期化されたインスタンスを構築します。
	//  @param[in]		n	The initial size of the integer array.
	// 整数配列の初期サイズ。
	rcIntArray(int n) : m_data(0), m_size(0), m_cap(0) { resize(n); }
	~rcIntArray() { rcFree(m_data); }

	// Specifies the new size of the integer array.
	// 整数配列の新しいサイズを指定します。
	//  @param[in]		n	The new size of the integer array.
	// 整数配列の新しいサイズ。
	void resize(int n)
	{
		if (n > m_cap)
			doResize(n);

		m_size = n;
	}

	// Push the specified integer onto the end of the array and increases the size by one.
	// 指定された整数を配列の最後にプッシュし、サイズを1つ増やします。
	//  @param[in]		item	The new value. //新しい値。
	void push(int item) { resize(m_size + 1); m_data[m_size - 1] = item; }

	// Returns the value at the end of the array and reduces the size by one.
	// 配列の最後に値を返し、サイズを1つ減らします。
	//  @return The value at the end of the array. // 配列の最後の値。
	int pop()
	{
		if (m_size > 0)
			m_size--;

		return m_data[m_size];
	}

	// The value at the specified array index.
	//	指定された配列インデックスの値。
	// @warning Does not provide overflow protection.
	//	オーバーフロー保護を提供しません。
	//  @param[in]		i	The index of the value.
	//	値のインデックス。
	const int& operator[](int i) const { return m_data[i]; }

	// The value at the specified array index.
	//	指定された配列インデックスの値。
	// @warning Does not provide overflow protection.
	//	オーバーフロー保護を提供しません。
	//  @param[in]		i	The index of the value.
	//	値のインデックス。
	int& operator[](int i) { return m_data[i]; }

	// The current size of the integer array.
	// 整数配列の現在のサイズ。
	int size() const noexcept { return m_size; }
};

// A simple helper class used to delete an array when it goes out of scope.
// 配列が範囲外になったときに配列を削除するために使用される単純なヘルパークラス。
// @note This class is rarely if ever used by the end user.
// @note このクラスは、エンドユーザーが使用することはほとんどありません。
template<class T> class rcScopedDelete
{
	T* ptr;
public:

	// Constructs an instance with a null pointer.
	// nullポインターを使用してインスタンスを構築します。
	inline rcScopedDelete() : ptr(nullptr) {}

	// Constructs an instance with the specified pointer.
	//  @param[in]		p	An pointer to an allocated array.
	// 指定されたポインタでインスタンスを構築します。
	// @param [in]		p	割り当てられた配列へのポインター。
	inline rcScopedDelete(T* p) : ptr(p) {}
	inline ~rcScopedDelete() { rcFree(ptr); }

	// The root array pointer.
	//  @return The root array pointer.
	// ルート配列ポインタ。
	// @return ルート配列ポインタ。
	inline operator T* () { return ptr; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcScopedDelete(const rcScopedDelete&) = delete;
	rcScopedDelete& operator=(const rcScopedDelete&) = delete;
};

#endif

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
// メモリの使用が予想される期間に関するヒント値をメモリアロケータに提供します。
enum dtAllocHint
{
	// Memory persist after a function call.
	// 関数呼び出し後もメモリは持続します。
	DT_ALLOC_PERM,
	// Memory used temporarily within a function.
	// 関数内で一時的に使用されるメモリ。
	DT_ALLOC_TEMP
};

// A memory allocation function.
// メモリ割り当て関数。
//  @param[in] size : The size, in bytes of memory, to allocate.
//	割り当てるメモリのサイズ（バイト単位）。
//  @param[in] rcAllocHint : A hint to the allocator on how long the memory is expected to be in use.
//	メモリーの使用が予想される期間に関するアロケーターへのヒント。
//  @return A pointer to the beginning of the allocated memory block, or null if the allocation failed.
//	割り当てられたメモリブロックの先頭へのポインタ、または割り当てが失敗した場合はnull。
//  @see dtAllocSetCustom
typedef void* (dtAllocFunc)(size_t size, dtAllocHint hint);

// A memory deallocation function.
// メモリ割り当て解除関数。
//  @param[in] ptr : A pointer to a memory block previously allocated using #dtAllocFunc.
//	以前に#dtAllocFuncを使用して割り当てられたメモリブロックへのポインタ。
// @see dtAllocSetCustom
typedef void (dtFreeFunc)(void* ptr);

// Sets the base custom allocation functions to be used by Detour.
// Detourが使用するベースのカスタム割り当て関数を設定します。
//  @param[in] allocFunc : The memory allocation function to be used by #dtAlloc
//	#dtAllocが使用するメモリ割り当て関数
//  @param[in] freeFunc : The memory de-allocation function to be used by #dtFree
//	#dtFreeによって使用されるメモリ割り当て解除関数
void dtAllocSetCustom(dtAllocFunc* allocFunc, dtFreeFunc* freeFunc);

// Allocates a memory block.
// メモリブロックを割り当てます。
//  @param[in] size : The size, in bytes of memory, to allocate.
//	割り当てるサイズ（メモリのバイト単位）。
//  @param[in] hint : A hint to the allocator on how long the memory is expected to be in use.
//	メモリーが使用されると予想される期間に関するアロケーターへのヒント。
//  @return A pointer to the beginning of the allocated memory block, or null if the allocation failed.
//	割り当てられたメモリブロックの先頭へのポインタ、または割り当てが失敗した場合はnull。
// @see dtFree
void* dtAlloc(size_t size, dtAllocHint hint);

// Deallocates a memory block.
// メモリブロックの割り当てを解除します。
//  @param[in] ptr : A pointer to a memory block previously allocated using #dtAlloc.
//	以前に#dtAllocを使用して割り当てられたメモリブロックへのポインタ。
// @see dtAlloc
void dtFree(void* ptr);

#endif

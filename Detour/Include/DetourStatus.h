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
// 高レベルのステータス

constexpr uint32_t DT_FAILURE = 1u << 31;			// Operation failed.  操作に失敗しました
constexpr uint32_t DT_SUCCESS = 1u << 30;			// Operation succeed. 操作は成功しました
constexpr uint32_t DT_IN_PROGRESS = 1u << 29;		// Operation still in progress. 操作はまだ進行中です

// Detail information for status.
// ステータスの詳細情報

constexpr uint32_t DT_STATUS_DETAIL_MASK = 0x0ffffff;
constexpr uint32_t DT_WRONG_MAGIC = 1 << 0;		// Input data is not recognized.  入力データが認識されません
constexpr uint32_t DT_WRONG_VERSION = 1 << 1;	// Input data is in wrong version. 入力データのバージョンが間違っています
constexpr uint32_t DT_OUT_OF_MEMORY = 1 << 2;	// Operation ran out of memory. 操作でメモリが不足しました
constexpr uint32_t DT_INVALID_PARAM = 1 << 3;	// An input parameter was invalid. 入力パラメーターが無効でした
constexpr uint32_t DT_BUFFER_TOO_SMALL = 1 << 4;	// Result buffer for the query was too small to store all results. クエリの結果バッファが小さすぎて、すべての結果を格納できませんでした
constexpr uint32_t DT_OUT_OF_NODES = 1 << 5;		// Query ran out of nodes during search. 検索中にクエリがノードを使い果たしました
constexpr uint32_t DT_PARTIAL_RESULT = 1 << 6;	// Query did not reach the end location, returning best guess. クエリは終了位置に到達せず、最良の推測を返しました

// Returns true of status is success.
// ステータスが成功の場合はtrueを返します
inline bool dtStatusSucceed(dtStatus status)
{
	return (status & DT_SUCCESS) != 0;
}

// Returns true of status is failure.
// ステータスが失敗の場合はtrueを返します。
inline bool dtStatusFailed(dtStatus status)
{
	return (status & DT_FAILURE) != 0;
}

// Returns true of status is in progress.
// ステータスが進行中の場合はtrueを返します
inline bool dtStatusInProgress(dtStatus status)
{
	return (status & DT_IN_PROGRESS) != 0;
}

// Returns true if specific detail is set.
// 特定の詳細が設定されている場合、trueを返します
inline bool dtStatusDetail(dtStatus status, uint32_t detail)
{
	return (status & detail) != 0;
}

#endif // DETOURSTATUS_H

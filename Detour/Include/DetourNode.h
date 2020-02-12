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

#ifndef DETOURNODE_H
#define DETOURNODE_H

#include "DetourConfig.h"
#include "DetourNavMesh.h"

enum dtNodeFlags
{
	DT_NODE_OPEN = 0x01,
	DT_NODE_CLOSED = 0x02,
	DT_NODE_PARENT_DETACHED = 0x04, // parent of the node is not adjacent. Found using raycast. ノードの親が隣接していません。 raycastを使用して検出されました。
};

constexpr dtNodeIndex DT_NULL_IDX = (dtNodeIndex)~0;

constexpr int DT_NODE_PARENT_BITS = 24;
constexpr int DT_NODE_STATE_BITS = 2;
struct dtNode
{
	// Position of the node.
	// ノードの位置。
	std::array<float, 3> pos;

	// Cost from previous node to current node.
	// 前のノードから現在のノードまでのコスト。
	float cost;

	// Cost up to the node.
	// ノードまでのコスト。
	float total;

	// Index to parent node.
	// 親ノードへのインデックス。
	uint32_t pidx : DT_NODE_PARENT_BITS;

	// extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
	// 追加の状態情報。 polyRefは、異なる追加情報を持つ複数のノードを持つことができます。 DT_MAX_STATES_PER_NODEを参照
	uint32_t state : DT_NODE_STATE_BITS;

	// Node flags. A combination of dtNodeFlags.
	// ノードフラグ。 dtNodeFlagsの組み合わせ。
	uint32_t flags : 3;

	// Polygon ref the node corresponds to.
	// ノードが対応するポリゴン参照。
	dtPolyRef id;
};

constexpr int DT_MAX_STATES_PER_NODE = 1 << DT_NODE_STATE_BITS;	// number of extra states per node. See dtNode::state ノードごとの追加状態の数。 dtNode :: stateを参照してください

class dtNodePool
{
public:
	dtNodePool(int maxNodes, int hashSize);
	~dtNodePool();
	void clear();

	// Get a dtNode by ref and extra state information. If there is none then - allocate
	// There can be more than one node for the same polyRef but with different extra state information
	// refおよび追加の状態情報によってdtNodeを取得します。 何もない場合-割り当てます。
	// 同じpolyRefに対して複数のノードが存在する可能性がありますが、追加の状態情報が異なります。
	dtNode* getNode(dtPolyRef id, unsigned char state = 0);
	dtNode* findNode(dtPolyRef id, unsigned char state);
	uint32_t findNodes(dtPolyRef id, dtNode** nodes, const int maxNodes);

	inline uint32_t getNodeIdx(const dtNode* node) const
	{
		if (!node) return 0;
		return (uint32_t)(node - m_nodes) + 1;
	}

	inline dtNode* getNodeAtIdx(uint32_t idx)
	{
		if (!idx) return 0;
		return &m_nodes[idx - 1];
	}

	inline const dtNode* getNodeAtIdx(uint32_t idx) const
	{
		if (!idx) return 0;
		return &m_nodes[idx - 1];
	}

	inline int getMemUsed() const
	{
		return sizeof(*this) +
			sizeof(dtNode) * m_maxNodes +
			sizeof(dtNodeIndex) * m_maxNodes +
			sizeof(dtNodeIndex) * m_hashSize;
	}

	inline int getMaxNodes() const { return m_maxNodes; }

	inline int getHashSize() const { return m_hashSize; }
	inline dtNodeIndex getFirst(int bucket) const { return m_first[bucket]; }
	inline dtNodeIndex getNext(int i) const { return m_next[i]; }
	inline int getNodeCount() const { return m_nodeCount; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtNodePool(const dtNodePool&) = delete;
	dtNodePool& operator=(const dtNodePool&) = delete;

	dtNode* m_nodes;
	dtNodeIndex* m_first;
	dtNodeIndex* m_next;
	const int m_maxNodes;
	const int m_hashSize;
	int m_nodeCount;
};

class dtNodeQueue
{
public:
	dtNodeQueue(int n);
	~dtNodeQueue();

	inline void clear() { m_size = 0; }

	inline dtNode* top() { return m_heap[0]; }

	inline dtNode* pop()
	{
		dtNode* result = m_heap[0];
		m_size--;
		trickleDown(0, m_heap[m_size]);
		return result;
	}

	inline void push(dtNode* node)
	{
		m_size++;
		bubbleUp(m_size - 1, node);
	}

	inline void modify(dtNode* node)
	{
		for (int i = 0; i < m_size; ++i)
		{
			if (m_heap[i] == node)
			{
				bubbleUp(i, node);
				return;
			}
		}
	}

	inline bool empty() const { return m_size == 0; }

	inline int getMemUsed() const
	{
		return sizeof(*this) +
			sizeof(dtNode*) * (m_capacity + 1);
	}

	inline int getCapacity() const { return m_capacity; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtNodeQueue(const dtNodeQueue&) = delete;
	dtNodeQueue& operator=(const dtNodeQueue&) = delete;

	void bubbleUp(int i, dtNode* node);
	void trickleDown(int i, dtNode* node);

	dtNode** m_heap;
	const int m_capacity;
	int m_size;
};

#endif // DETOURNODE_H

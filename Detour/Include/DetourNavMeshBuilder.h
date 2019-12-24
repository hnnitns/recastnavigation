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

#ifndef DETOURNAVMESHBUILDER_H
#define DETOURNAVMESHBUILDER_H

#include "DetourAlloc.h"

// Represents the source data used to build an navigation mesh tile.
// ナビゲーションメッシュタイルの構築に使用されるソースデータを表します。
// @ingroup detour
struct dtNavMeshCreateParams
{
	// @name Polygon Mesh Attributes // ポリゴンメッシュの属性
	// Used to create the base navigation graph.
	// ベースナビゲーショングラフの作成に使用されます。
	// See #rcPolyMesh for details related to these attributes.
	// @{
	// The polygon mesh vertices. [(x, y, z) * #vertCount] [Unit: vx]
	// ポリゴンメッシュの頂点。 [（x、y、z）* #vertCount] [単位：vx]
	const unsigned short* verts;

	// The number vertices in the polygon mesh. [Limit: >= 3]
	// ポリゴンメッシュの頂点の数。 [制限：> = 3]
	int vertCount;

	// The polygon data. [Size: #polyCount * 2 * #nvp]
	// ポリゴンデータ。 [サイズ：#polyCount * 2 * #nvp]
	const unsigned short* polys;

	// The user defined flags assigned to each polygon. [Size: #polyCount]
	// 各ポリゴンに割り当てられたユーザー定義のフラグ。 [サイズ：#polyCount]
	const unsigned short* polyFlags;

	// The user defined area ids assigned to each polygon. [Size: #polyCount]
	// 各ポリゴンに割り当てられたユーザー定義のエリアID。 [サイズ：#polyCount]
	const unsigned char* polyAreas;

	// Number of polygons in the mesh. [Limit: >= 1]
	// メッシュ内のポリゴンの数。 [制限：> = 1]
	int polyCount;

	// Number maximum number of vertices per polygon. [Limit: >= 3]
	// ポリゴンごとの頂点の最大数を数えます。 [制限：> = 3]
	int nvp;

	// @}
	// @name Height Detail Attributes (Optional) // 高さ詳細属性（オプション）
	// See #rcPolyMeshDetail for details related to these attributes.
	// @{
	// The height detail sub-mesh data. [Size: 4 * #polyCount]
	// 高さの詳細サブメッシュデータ。 [サイズ：4 * #polyCount]
	const unsigned int* detailMeshes;

	// The detail mesh vertices. [Size: 3 * #detailVertsCount] [Unit: wu]
	// 詳細メッシュの頂点。 [サイズ：3 * #detailVertsCount] [単位：wu]
	const float* detailVerts;

	// The number of vertices in the detail mesh.
	// 詳細メッシュの頂点の数。
	int detailVertsCount;

	// The detail mesh triangles. [Size: 4 * #detailTriCount]
	// 詳細メッシュの三角形。 [サイズ：4 * #detailTriCount]
	const unsigned char* detailTris;

	// The number of triangles in the detail mesh.
	// 詳細メッシュの三角形の数。
	int detailTriCount;

	// @}
	// @name Off-Mesh Connections Attributes (Optional) // オフメッシュ接続属性（オプション）
	// Used to define a custom point-to-point edge within the navigation graph, an
	// off-mesh connection is a user defined traversable connection made up to two vertices,
	// at least one of which resides within a navigation mesh polygon.
	// ナビゲーショングラフ内のカスタムポイントツーポイントエッジを定義するために使用されるオフメッシュ接続は、
	// 2つまでの頂点で構成されるユーザー定義のトラバース可能な接続であり、
	// 少なくとも1つはナビゲーションメッシュポリゴン内に存在します。
	// {
	// Off-mesh connection vertices. [(ax, ay, az, bx, by, bz) * #offMeshConCount] [Unit: wu]
	// メッシュ外の接続頂点。 [（ax、ay、az、bx、by、bz）* #offMeshConCount] [単位：wu]
	const float* offMeshConVerts;

	// Off-mesh connection radii. [Size: #offMeshConCount] [Unit: wu]
	// オフメッシュ接続半径。 [サイズ：#offMeshConCount] [単位：wu]
	const float* offMeshConRad;

	// User defined flags assigned to the off-mesh connections. [Size: #offMeshConCount]
	// オフメッシュ接続に割り当てられたユーザー定義フラグ。 [サイズ：#offMeshConCount]
	const unsigned short* offMeshConFlags;

	// User defined area ids assigned to the off-mesh connections. [Size: #offMeshConCount]
	// オフメッシュ接続に割り当てられたユーザー定義のエリアID。 [サイズ：#offMeshConCount]
	const unsigned char* offMeshConAreas;

	// The permitted travel direction of the off-mesh connections. [Size: #offMeshConCount]
	// オフメッシュ接続の許可された移動方向。 [サイズ：#offMeshConCount]
	//
	// 0 = Travel only from endpoint A to endpoint B.<br/>
	// 0 =エンドポイントAからエンドポイントBへの移動のみ。
	// #DT_OFFMESH_CON_BIDIR = Bidirectional travel.
	const unsigned char* offMeshConDir;

	// The user defined ids of the off-mesh connection. [Size: #offMeshConCount]
	// オフメッシュ接続のユーザー定義ID。 [サイズ：#offMeshConCount]
	const unsigned int* offMeshConUserID;

	// The number of off-mesh connections. [Limit: >= 0]
	// オフメッシュ接続の数。 [制限：> = 0]
	int offMeshConCount;

	// @}
	// @name Tile Attributes // タイル属性
	// @note The tile grid/layer data can be left at zero if the destination is a single tile mesh.
	// 宛先が単一のタイルメッシュの場合、タイルグリッド/レイヤーデータはゼロのままにすることができます。
	// @{
	// The user defined id of the tile.
	// ユーザーが定義したタイルのID。
	unsigned int userId;

	// The tile's x-grid location within the multi-tile destination mesh. (Along the x-axis.)
	// マルチタイル先メッシュ内のタイルのyグリッドの位置。（x軸に沿う）
	int tileX;

	// The tile's y-grid location within the multi-tile desitation mesh. (Along the z-axis.)
	// マルチタイル先のメッシュ内のタイルのyグリッドの位置。（z軸に沿う）
	int tileY;

	// The tile's layer within the layered destination mesh. [Limit: >= 0] (Along the y-axis.)
	// レイヤー化された宛先メッシュ内のタイルのレイヤー。 [制限：> = 0]（y軸に沿って。）
	int tileLayer;

	// The minimum bounds of the tile. [(x, y, z)] [Unit: wu]
	// タイルの最小境界。[（x、y、z）] [単位：wu]
	float bmin[3];

	// The maximum bounds of the tile. [(x, y, z)] [Unit: wu]
	// タイルの最大境界。 [（x、y、z）] [単位：wu]
	float bmax[3];

	// @}
	// @name General Configuration Attributes // 一般的な構成属性
	// @{
	// The agent height. [Unit: wu]
	// エージェントの高さ。 [単位：wu]
	float walkableHeight;

	// The agent radius. [Unit: wu]
	// エージェントの半径。 [単位：wu]
	float walkableRadius;

	// The agent maximum traversable ledge. (Up/Down) [Unit: wu]
	// エージェントの最大移動可能棚。 （上/下）[単位：wu]
	float walkableClimb;

	// The xz-plane cell size of the polygon mesh. [Limit: > 0] [Unit: wu]
	// ポリゴンメッシュのxz平面のセルサイズ。 [制限：> 0] [単位：wu]
	float cs;

	// The y-axis cell height of the polygon mesh. [Limit: > 0] [Unit: wu]
	// ポリゴンメッシュのy軸のセルの高さ。 [制限：> 0] [単位：wu]
	float ch;

	// True if a bounding volume tree should be built for the tile.
	// @note The BVTree is not normally needed for layered navigation meshes.
	// 境界ボリュームツリーをタイルに構築する必要がある場合はTrue。
	// @note BVTreeは通常、階層化されたナビゲーションメッシュには必要ありません。
	bool buildBvTree;

	// @}
};

// Builds navigation mesh tile data from the provided tile creation data.
// 提供されたタイル作成データからナビゲーションメッシュタイルデータを構築します。
// @ingroup detour
// @param[in] params : Tile creation data. // タイル作成データ。
// @param[out] outData : The resulting tile data. // 結果のタイルデータ。
// @param[out] outDataSize : The size of the tile data array. // タイルデータ配列のサイズ。
// @return True if the tile data was successfully created.
//  タイルデータが正常に作成された場合はTrue。
bool dtCreateNavMeshData(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize);

// Swaps the endianess of the tile data's header (#dtMeshHeader).
// タイルデータヘッダーのエンディアンを交換します（#dtMeshHeader）。
//  @param[in,out]	data		The tile data array.
//  @param[in]		dataSize	The size of the data array.
bool dtNavMeshHeaderSwapEndian(unsigned char* data, const int dataSize);

// Swaps endianess of the tile data.
// タイルデータのエンディアンを入れ替えます。
//  @param[in,out]	data		The tile data array.
//  @param[in]		dataSize	The size of the data array.
bool dtNavMeshDataSwapEndian(unsigned char* data, const int dataSize);

#endif // DETOURNAVMESHBUILDER_H

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.
// このセクションには、ソースファイルを持たないメンバーの詳細なドキュメントが含まれています。
// ヘッダーのメインセクションの混乱を軽減します。

/**

@struct dtNavMeshCreateParams
@par

This structure is used to marshal data between the Recast mesh generation pipeline and Detour navigation components.
この構造体は、Recastメッシュ生成パイプラインとDetourナビゲーションコンポーネント間でデータをマーシャリングするために使用されます。

See the rcPolyMesh and rcPolyMeshDetail documentation for detailed information related to mesh structure.
メッシュ構造に関する詳細については、rcPolyMeshおよびrcPolyMeshDetailのドキュメントを参照してください。

Units are usually in voxels (vx) or world units (wu). The units for voxels, grid size, and cell size
are all based on the values of #cs and #ch.
単位は通常、ボクセル（vx）またはワールド単位（wu）です。
ボクセルの単位、グリッドサイズ、およびセルサイズはすべて、＃csおよび#chの値に基づいています。

The standard navigation mesh build process is to create tile data using dtCreateNavMeshData, then add the tile
to a navigation mesh using either the dtNavMesh single tile <tt>init()</tt> function or the dtNavMesh::addTile()
function.
標準のナビゲーションメッシュビルドプロセスは、dtCreateNavMeshDataを使用してタイルデータを作成し、
dtNavMeshシングルタイルinit（）関数またはdtNavMesh :: addTile（）関数を使用してタイルをナビゲーションメッシュに追加します。

@see dtCreateNavMeshData

*/

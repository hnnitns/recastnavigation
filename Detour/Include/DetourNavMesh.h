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

#ifndef DETOURNAVMESH_H
#define DETOURNAVMESH_H

#include "DetourConfig.h"
#include "DetourAlloc.h"
#include "DetourStatus.h"

#ifdef DT_POLYREF64
// TODO: figure out a multiplatform version of uint64_t
// - maybe: https://code.google.com/p/msinttypes/
// - or: http://www.azillionmonkeys.com/qed/pstdint.h
#include <stdint>
#endif

// A handle to a polygon within a navigation mesh tile.
// @ingroup detour
#ifdef DT_POLYREF64
constexpr unsigned int DT_SALT_BITS = 16;
constexpr unsigned int DT_TILE_BITS = 28;
constexpr unsigned int DT_POLY_BITS = 20;
typedef uint64_t dtPolyRef;
#else
//ナビゲーションメッシュタイル内のポリゴンのハンドル。
typedef unsigned int dtPolyRef;
#endif

// A handle to a tile within a navigation mesh.
// @ingroup detour
#ifdef DT_POLYREF64
typedef uint64_t dtTileRef;
#else
// ナビゲーションメッシュ内のタイルへのハンドル。
typedef unsigned int dtTileRef;
#endif

// The maximum number of vertices per navigation polygon.
// ナビゲーションポリゴンごとの頂点の最大数。
// @ingroup detour
constexpr int DT_VERTS_PER_POLYGON = 6;

// @{
// @name Tile Serialization Constants
// These constants are used to detect whether a navigation tile's data and state format is compatible with the current build.
// これらの定数は、ナビゲーションタイルのデータと状態の形式が現在のビルドと互換性があるかどうかを検出するために使用されます。
//

// A magic number used to detect compatibility of navigation tile data.
// ナビゲーションタイルデータの互換性を検出するために使用されるマジックナンバー。
constexpr int DT_NAVMESH_MAGIC = 'D' << 24 | 'N' << 16 | 'A' << 8 | 'V';

// A version number used to detect compatibility of navigation tile data.
// ナビゲーションタイルデータの互換性を検出するために使用されるバージョン番号。
constexpr int DT_NAVMESH_VERSION = 7;

// A magic number used to detect the compatibility of navigation tile states.
// ナビゲーションタイルの状態の互換性を検出するために使用されるマジックナンバー。
constexpr int DT_NAVMESH_STATE_MAGIC = 'D' << 24 | 'N' << 16 | 'M' << 8 | 'S';

// A version number used to detect compatibility of navigation tile states.
// ナビゲーションタイルの状態の互換性を検出するために使用されるバージョン番号。
constexpr int DT_NAVMESH_STATE_VERSION = 1;

// @}

// A flag that indicates that an entity links to an external entity.
// エンティティが外部エンティティにリンクしていることを示すフラグ。
// (E.g. A polygon edge is a portal that links to another polygon.)
// （たとえば、ポリゴンエッジは、別のポリゴンにリンクするポータルです。）
constexpr unsigned short DT_EXT_LINK = 0x8000;

// A value that indicates the entity does not link to anything.
// エンティティが何にもリンクしていないことを示す値。
constexpr unsigned int DT_NULL_LINK = 0xffffffff;

// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
// オフメッシュ接続を両方向に横断できることを示すフラグ。（双方向です。）
constexpr unsigned int DT_OFFMESH_CON_BIDIR = 1;

// The maximum number of user defined area ids.
// ユーザー定義のエリアIDの最大数。
// @ingroup detour
constexpr int DT_MAX_AREAS = 64;

// Tile flags used for various functions and fields.
// さまざまな関数とフィールドに使用されるタイルフラグ。
// For an example, see dtNavMesh::addTile().
// 例については、dtNavMesh :: addTile（）を参照してください。
enum dtTileFlags
{
	// The navigation mesh owns the tile memory and is responsible for freeing it.
	// ナビゲーションメッシュはタイルメモリを所有し、タイルメモリを解放します。
	DT_TILE_FREE_DATA = 0x01,
};

// Vertex flags returned by dtNavMeshQuery::findStraightPath.
// dtNavMeshQuery :: findStraightPathによって返される頂点フラグ。
enum dtStraightPathFlags
{
	// The vertex is the start position in the path.
	// 頂点はパスの開始位置です。
	DT_STRAIGHTPATH_START = 0x01,
	// The vertex is the end position in the path.
	// 頂点はパス内の終了位置です。
	DT_STRAIGHTPATH_END = 0x02,
	// The vertex is the start of an off-mesh connection.
	// 頂点はオフメッシュ接続の開始点です。
	DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04,
};

// Options for dtNavMeshQuery::findStraightPath.
// dtNavMeshQuery :: findStraightPathのオプション。
enum dtStraightPathOptions
{
	// Add a vertex at every polygon edge crossing where area changes.
	// 面積が変わる場所で交差するすべてのポリゴンエッジに頂点を追加します。
	DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01,
	// Add a vertex at every polygon edge crossing.
	// ポリゴンのエッジが交差するたびに頂点を追加します。
	DT_STRAIGHTPATH_ALL_CROSSINGS = 0x02,
};

// Options for dtNavMeshQuery::initSlicedFindPath and updateSlicedFindPath
enum dtFindPathOptions
{
	DT_FINDPATH_ANY_ANGLE = 0x02,		//< use raycasts during pathfind to "shortcut" (raycast still consider costs)
};

// Options for dtNavMeshQuery::raycast
enum dtRaycastOptions
{
	DT_RAYCAST_USE_COSTS = 0x01,		//< Raycast should calculate movement cost along the ray and fill RaycastHit::cost
};

// Limit raycasting during any angle pahfinding
// The limit is given as a multiple of the character radius
constexpr float DT_RAY_CAST_LIMIT_PROPORTIONS = 50.0f;

// Flags representing the type of a navigation mesh polygon.
// ナビゲーションメッシュポリゴンのタイプを表すフラグ。
enum dtPolyTypes
{
	// The polygon is a standard convex polygon that is part of the surface of the mesh.
	// ポリゴンは、メッシュの表面の一部である標準の凸ポリゴンです。
	DT_POLYTYPE_GROUND = 0,
	// The polygon is an off-mesh connection consisting of two vertices.
	// ポリゴンは、2つの頂点で構成されるメッシュ外の接続です。
	DT_POLYTYPE_OFFMESH_CONNECTION = 1,
};

// Defines a polygon within a dtMeshTile object.
// @ingroup detour
struct dtPoly
{
	// Index to first link in linked list. (Or #DT_NULL_LINK if there is no link.)
	unsigned int firstLink;

	// The indices of the polygon's vertices.
	// The actual vertices are located in dtMeshTile::verts.
	unsigned short verts[DT_VERTS_PER_POLYGON];

	// Packed data representing neighbor polygons references and flags for each edge.
	unsigned short neis[DT_VERTS_PER_POLYGON];

	// The user defined polygon flags.
	unsigned short flags;

	// The number of vertices in the polygon.
	unsigned char vertCount;

	// The bit packed area id and polygon type.
	// @note Use the structure's set and get methods to acess this value.
	unsigned char areaAndtype;

	// Sets the user defined area id. [Limit: < #DT_MAX_AREAS]
	inline void setArea(unsigned char a) { areaAndtype = (areaAndtype & 0xc0) | (a & 0x3f); }

	// Sets the polygon type. (See: #dtPolyTypes.)
	inline void setType(unsigned char t) { areaAndtype = (areaAndtype & 0x3f) | (t << 6); }

	// Gets the user defined area id.
	inline unsigned char getArea() const { return areaAndtype & 0x3f; }

	// Gets the polygon type. (See: #dtPolyTypes)
	inline unsigned char getType() const { return areaAndtype >> 6; }
};

// Defines the location of detail sub-mesh data within a dtMeshTile.
struct dtPolyDetail
{
	unsigned int vertBase;			//< The offset of the vertices in the dtMeshTile::detailVerts array.
	unsigned int triBase;			//< The offset of the triangles in the dtMeshTile::detailTris array.
	unsigned char vertCount;		//< The number of vertices in the sub-mesh.
	unsigned char triCount;			//< The number of triangles in the sub-mesh.
};

// Defines a link between polygons.
// @note This structure is rarely if ever used by the end user.
// @see dtMeshTile
struct dtLink
{
	dtPolyRef ref;					//< Neighbour reference. (The neighbor that is linked to.)
	unsigned int next;				//< Index of the next link.
	unsigned char edge;				//< Index of the polygon edge that owns this link.
	unsigned char side;				//< If a boundary link, defines on which side the link is.
	unsigned char bmin;				//< If a boundary link, defines the minimum sub-edge area.
	unsigned char bmax;				//< If a boundary link, defines the maximum sub-edge area.
};

// Bounding volume node.
// @note This structure is rarely if ever used by the end user.
// @see dtMeshTile
struct dtBVNode
{
	unsigned short bmin[3];			//< Minimum bounds of the node's AABB. [(x, y, z)]
	unsigned short bmax[3];			//< Maximum bounds of the node's AABB. [(x, y, z)]
	int i;							//< The node's index. (Negative for escape sequence.)
};

// Defines an navigation mesh off-mesh connection within a dtMeshTile object.
// An off-mesh connection is a user defined traversable connection made up to two vertices.
struct dtOffMeshConnection
{
	// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
	float pos[6];

	// The radius of the endpoints. [Limit: >= 0]
	float rad;

	// The polygon reference of the connection within the tile.
	unsigned short poly;

	// Link flags.
	// @note These are not the connection's user defined flags. Those are assigned via the
	// connection's dtPoly definition. These are link flags used for internal purposes.
	unsigned char flags;

	// End point side.
	unsigned char side;

	// The id of the offmesh connection. (User assigned when the navigation mesh is built.)
	unsigned int userId;
};

// Provides high level information related to a dtMeshTile object.
// dtMeshTileオブジェクトに関連する高レベルの情報を提供します。
// @ingroup detour
struct dtMeshHeader
{
	//< Tile magic number. (Used to identify the data format.)
	// マジック番号を並べて表示します。 （データ形式を識別するために使用されます。）
	int magic;
	//< Tile data format version number.
	// タイルデータ形式のバージョン番号。
	int version;
	//< The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	// dtNavMeshタイルグリッド内のタイルのx位置。 （x、y、レイヤー）
	int x;
	//< The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	// dtNavMeshタイルグリッド内のタイルのy位置。 （x、y、レイヤー）
	int y;
	//< The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
	// dtNavMeshタイルグリッド内のタイルのレイヤー。 （x、y、レイヤー）
	int layer;
	//< The user defined id of the tile.
	// ユーザー定義のタイルのID。
	unsigned int userId;
	//< The number of polygons in the tile.
	// タイル内のポリゴンの数。
	int polyCount;
	//< The number of vertices in the tile.
	// タイル内の頂点の数。
	int vertCount;
	//< The number of allocated links.
	// 割り当てられたリンクの数。
	int maxLinkCount;
	//< The number of sub-meshes in the detail mesh.
	// ディテールメッシュのサブメッシュの数。
	int detailMeshCount;


	// The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
	// ディテールメッシュ内の一意の頂点の数。 （ポリゴンの頂点に加えて）
	int detailVertCount;

	//< The number of triangles in the detail mesh.
	// ディテールメッシュの三角形の数。
	int detailTriCount;
	//< The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	// バウンディングボリュームノードの数。 （バウンディングボリュームが無効になっている場合はゼロ）。
	int bvNodeCount;
	//< The number of off-mesh connections.
	// オフメッシュ接続の数。
	int offMeshConCount;
	//< The index of the first polygon which is an off-mesh connection.
	// オフメッシュ接続である最初のポリゴンのインデックス。
	int offMeshBase;
	//< The height of the agents using the tile.
	// タイルを使用するエージェントの高さ。
	float walkableHeight;
	//< The radius of the agents using the tile.
	// タイルを使用するエージェントの半径。
	float walkableRadius;
	//< The maximum climb height of the agents using the tile.
	// タイルを使用するエージェントの最大上昇高さ。
	float walkableClimb;
	//< The minimum bounds of the tile's AABB. [(x, y, z)]
	// タイルのAABBの最小境界。 [（x、y、z）]
	float bmin[3];
	//< The maximum bounds of the tile's AABB. [(x, y, z)]
	// タイルのAABBの最大境界。 [（x、y、z）]
	float bmax[3];

	// The bounding volume quantization factor.
	// バウンディングボリュームの量子化係数。
	float bvQuantFactor;
};

// Defines a navigation mesh tile.
// ナビゲーションメッシュタイルを定義します。
// @ingroup detour
struct dtMeshTile
{
	//< Counter describing modifications to the tile.
	// タイルへの変更を説明するカウンター。
	unsigned int salt;
	//< Index to the next free link.
	// 次の空きリンクへのインデックス。
	unsigned int linksFreeList;
	//< The tile header.
	// タイルヘッダー。
	dtMeshHeader* header;
	//< The tile polygons. [Size: dtMeshHeader::polyCount]
	// タイルポリゴン。 [サイズ：dtMeshHeader :: polyCount]
	dtPoly* polys;
	//< The tile vertices. [Size: dtMeshHeader::vertCount]
	// タイルの頂点。 [サイズ：dtMeshHeader :: vertCount]
	float* verts;
	//< The tile links. [Size: dtMeshHeader::maxLinkCount]
	// タイルリンク。 [サイズ：dtMeshHeader :: maxLinkCount]
	dtLink* links;
	//< The tile's detail sub-meshes. [Size: dtMeshHeader::detailMeshCount]
	// タイルの詳細サブメッシュ。 [サイズ：dtMeshHeader :: detailMeshCount]
	dtPolyDetail* detailMeshes;


	// The detail mesh's unique vertices. [(x, y, z) * dtMeshHeader::detailVertCount]
	// 詳細メッシュの一意の頂点。 [（X、y、z）* dtMeshHeader :: detailVertCount]
	float* detailVerts;

	// The detail mesh's triangles. [(vertA, vertB, vertC) * dtMeshHeader::detailTriCount]
	// 詳細メッシュの三角形。 [（VertA、vertB、vertC）* dtMeshHeader :: detailTriCount]
	unsigned char* detailTris;

	// The tile bounding volume nodes. [Size: dtMeshHeader::bvNodeCount] (Will be null if bounding volumes are disabled.)
	// タイル境界ボリュームノード。 [サイズ：dtMeshHeader :: bvNodeCount]（バウンディングボリュームが無効になっている場合はnullになります。）
	dtBVNode* bvTree;

	//< The tile off-mesh connections. [Size: dtMeshHeader::offMeshConCount]
	// タイルのオフメッシュ接続。 [サイズ：dtMeshHeader :: offMeshConCount]
	dtOffMeshConnection* offMeshCons;

	//< The tile data. (Not directly accessed under normal situations.)
	// タイルデータ。 （通常は直接アクセスされません。）
	unsigned char* data;
	//< Size of the tile data.
	// タイルデータのサイズ。
	int dataSize;
	//< Tile flags. (See: #dtTileFlags)
	// タイルフラグ。 （参照：#dtTileFlags）
	int flags;
	//< The next free tile, or the next tile in the spatial grid.
	// 次のフリータイル、または空間グリッドの次のタイル。
	dtMeshTile* next;
private:
	dtMeshTile(const dtMeshTile&);
	dtMeshTile& operator=(const dtMeshTile&);
};

// Configuration parameters used to define multi-tile navigation meshes.
// The values are used to allocate space during the initialization of a navigation mesh.
// @see dtNavMesh::init()
// @ingroup detour
struct dtNavMeshParams
{
	float orig[3];					//< The world space origin of the navigation mesh's tile space. [(x, y, z)]
	float tileWidth;				//< The width of each tile. (Along the x-axis.)
	float tileHeight;				//< The height of each tile. (Along the z-axis.)
	int maxTiles;					//< The maximum number of tiles the navigation mesh can contain.
	int maxPolys;					//< The maximum number of polygons each tile can contain.
};

// A navigation mesh based on tiles of convex polygons.
// @ingroup detour
class dtNavMesh
{
public:
	dtNavMesh();
	~dtNavMesh();

	// @{
	// @name Initialization and Tile Management
	// 初期化とタイル管理。

	// Initializes the navigation mesh for tiled use.
	// タイル化された使用のためにナビゲーションメッシュを初期化します。
	// @param[in] params : Initialization parameters.
	// 初期化パラメーター。
	// @return The status flags for the operation.
	// 操作のステータスフラグ。
	dtStatus init(const dtNavMeshParams* params);

	// Initializes the navigation mesh for single tile use.
	//  @param[in]	data		Data of the new tile. (See: #dtCreateNavMeshData)
	//  @param[in]	dataSize	The data size of the new tile.
	//  @param[in]	flags		The tile flags. (See: #dtTileFlags)
	// @return The status flags for the operation.
	//  @see dtCreateNavMeshData
	dtStatus init(unsigned char* data, const int dataSize, const int flags);

	// The navigation mesh initialization params.
	const dtNavMeshParams* getParams() const;

	// Adds a tile to the navigation mesh.
	//	ナビゲーションメッシュにタイルを追加します。
	//  @param[in] data : Data for the new tile mesh. (See: #dtCreateNavMeshData)
	//	新しいタイルメッシュのデータ。 （#dtCreateNavMeshDataを参照）
	//  @param[in] dataSize : Data size of the new tile mesh.
	//	新しいタイルメッシュのデータサイズ。
	//  @param[in] flags : Tile flags. (See: #dtTileFlags)
	//	タイルフラグ。 （参照：#dtTileFlags）
	//  @param[in] lastRef : The desired reference for the tile. (When reloading a tile.) [opt] [Default: 0]
	//	タイルに必要な参照。 （タイルをリロードする場合。）[デフォルト：0]
	//  @param[out] result : The tile reference. (If the tile was succesfully added.) [opt]
	//	タイル参照。 （タイルが正常に追加された場合。）
	// @return The status flags for the operation.
	//	操作のステータスフラグ。
	dtStatus addTile(unsigned char* data, int dataSize, int flags, dtTileRef lastRef, dtTileRef* result);

	// Removes the specified tile from the navigation mesh.
	//  @param[in]		ref			The reference of the tile to remove.
	//  @param[out]	data		Data associated with deleted tile.
	//  @param[out]	dataSize	Size of the data associated with deleted tile.
	// @return The status flags for the operation.
	dtStatus removeTile(dtTileRef ref, unsigned char** data, int* dataSize);

	// @}

	// @{
	// @name Query Functions

	// Calculates the tile grid location for the specified world position.
	//  @param[in]	pos  The world position for the query. [(x, y, z)]
	//  @param[out]	tx		The tile's x-location. (x, y)
	//  @param[out]	ty		The tile's y-location. (x, y)
	void calcTileLoc(const float* pos, int* tx, int* ty) const;

	// Gets the tile at the specified grid location.
	//  @param[in]	x		The tile's x-location. (x, y, layer)
	//  @param[in]	y		The tile's y-location. (x, y, layer)
	//  @param[in]	layer	The tile's layer. (x, y, layer)
	// @return The tile, or null if the tile does not exist.
	const dtMeshTile* getTileAt(const int x, const int y, const int layer) const;

	// Gets all tiles at the specified grid location. (All layers.)
	//  @param[in]		x			The tile's x-location. (x, y)
	//  @param[in]		y			The tile's y-location. (x, y)
	//  @param[out]	tiles		A pointer to an array of tiles that will hold the result.
	//  @param[in]		maxTiles	The maximum tiles the tiles parameter can hold.
	// @return The number of tiles returned in the tiles array.
	int getTilesAt(const int x, const int y,
		dtMeshTile const** tiles, const int maxTiles) const;

	// Gets the tile reference for the tile at specified grid location.
	//  @param[in]	x		The tile's x-location. (x, y, layer)
	//  @param[in]	y		The tile's y-location. (x, y, layer)
	//  @param[in]	layer	The tile's layer. (x, y, layer)
	// @return The tile reference of the tile, or 0 if there is none.
	dtTileRef getTileRefAt(int x, int y, int layer) const;

	// Gets the tile reference for the specified tile.
	//  @param[in]	tile	The tile.
	// @return The tile reference of the tile.
	dtTileRef getTileRef(const dtMeshTile* tile) const;

	// Gets the tile for the specified tile reference.
	//  @param[in]	ref		The tile reference of the tile to retrieve.
	// @return The tile for the specified reference, or null if the
	//		reference is invalid.
	const dtMeshTile* getTileByRef(dtTileRef ref) const;

	// The maximum number of tiles supported by the navigation mesh.
	// @return The maximum number of tiles supported by the navigation mesh.
	int getMaxTiles() const;

	// Gets the tile at the specified index.
	//  @param[in]	i		The tile index. [Limit: 0 >= index < #getMaxTiles()]
	// @return The tile at the specified index.
	const dtMeshTile* getTile(int i) const;

	// Gets the tile and polygon for the specified polygon reference.
	//  @param[in]		ref		The reference for the a polygon.
	//  @param[out]	tile	The tile containing the polygon.
	//  @param[out]	poly	The polygon.
	// @return The status flags for the operation.
	dtStatus getTileAndPolyByRef(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;

	// Returns the tile and polygon for the specified polygon reference.
	//  @param[in]		ref		A known valid reference for a polygon.
	//  @param[out]	tile	The tile containing the polygon.
	//  @param[out]	poly	The polygon.
	// 指定されたポリゴン参照のタイルとポリゴンを返します。
	// Param[in]  ポリゴンの既知の有効な参照。
	// param[out] ポリゴンを含むタイル。
	// param[out] polyポリゴン。
	void getTileAndPolyByRefUnsafe(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;

	// Checks the validity of a polygon reference.							ポリゴン参照の有効性を確認します。
	//  @param[in]	ref		The polygon reference to check.					refチェックするポリゴン参照。
	// @return True if polygon reference is valid for the navigation mesh.	ナビゲーションメッシュでポリゴン参照が有効な場合はTrue。
	bool isValidPolyRef(dtPolyRef ref) const;

	// Gets the polygon reference for the tile's base polygon.
	//  @param[in]	tile		The tile.
	// @return The polygon reference for the base polygon in the specified tile.
	dtPolyRef getPolyRefBase(const dtMeshTile* tile) const;

	// Gets the endpoints for an off-mesh connection, ordered by "direction of travel".
	//  @param[in]		prevRef		The reference of the polygon before the connection.
	//  @param[in]		polyRef		The reference of the off-mesh connection polygon.
	//  @param[out]	startPos	The start position of the off-mesh connection. [(x, y, z)]
	//  @param[out]	endPos		The end position of the off-mesh connection. [(x, y, z)]
	// @return The status flags for the operation.
	dtStatus getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const;

	// Gets the specified off-mesh connection.
	//  @param[in]	ref		The polygon reference of the off-mesh connection.
	// @return The specified off-mesh connection, or null if the polygon reference is not valid.
	const dtOffMeshConnection* getOffMeshConnectionByRef(dtPolyRef ref) const;

	// @}

	// @{
	// @name State Management
	// These functions do not effect #dtTileRef or #dtPolyRef's.

	// Sets the user defined flags for the specified polygon.
	//  @param[in]	ref		The polygon reference.
	//  @param[in]	flags	The new flags for the polygon.
	// @return The status flags for the operation.
	dtStatus setPolyFlags(dtPolyRef ref, unsigned short flags);

	// Gets the user defined flags for the specified polygon.
	//  @param[in]		ref				The polygon reference.
	//  @param[out]	resultFlags		The polygon flags.
	// @return The status flags for the operation.
	dtStatus getPolyFlags(dtPolyRef ref, unsigned short* resultFlags) const;

	// Sets the user defined area for the specified polygon.
	//  @param[in]	ref		The polygon reference.
	//  @param[in]	area	The new area id for the polygon. [Limit: < #DT_MAX_AREAS]
	// @return The status flags for the operation.
	dtStatus setPolyArea(dtPolyRef ref, unsigned char area);

	// Gets the user defined area for the specified polygon.
	//  @param[in]		ref			The polygon reference.
	//  @param[out]	resultArea	The area id for the polygon.
	// @return The status flags for the operation.
	dtStatus getPolyArea(dtPolyRef ref, unsigned char* resultArea) const;

	// Gets the size of the buffer required by #storeTileState to store the specified tile's state.
	//  @param[in]	tile	The tile.
	// @return The size of the buffer required to store the state.
	int getTileStateSize(const dtMeshTile* tile) const;

	// Stores the non-structural state of the tile in the specified buffer. (Flags, area ids, etc.)
	//  @param[in]		tile			The tile.
	//  @param[out]	data			The buffer to store the tile's state in.
	//  @param[in]		maxDataSize		The size of the data buffer. [Limit: >= #getTileStateSize]
	// @return The status flags for the operation.
	dtStatus storeTileState(const dtMeshTile* tile, unsigned char* data, const int maxDataSize) const;

	// Restores the state of the tile.
	//  @param[in]	tile			The tile.
	//  @param[in]	data			The new state. (Obtained from #storeTileState.)
	//  @param[in]	maxDataSize		The size of the state within the data buffer.
	// @return The status flags for the operation.
	dtStatus restoreTileState(dtMeshTile* tile, const unsigned char* data, const int maxDataSize);

	// @}

	// @{
	// @name Encoding and Decoding
	// These functions are generally meant for internal use only.

	// Derives a standard polygon reference.
	//  @note This function is generally meant for internal use only.
	//  @param[in]	salt	The tile's salt value.
	//  @param[in]	it		The index of the tile.
	//  @param[in]	ip		The index of the polygon within the tile.
	inline dtPolyRef encodePolyId(unsigned int salt, unsigned int it, unsigned int ip) const
	{
#ifdef DT_POLYREF64
		return ((dtPolyRef)salt << (DT_POLY_BITS + DT_TILE_BITS)) | ((dtPolyRef)it << DT_POLY_BITS) | (dtPolyRef)ip;
#else
		return ((dtPolyRef)salt << (m_polyBits + m_tileBits)) | ((dtPolyRef)it << m_polyBits) | (dtPolyRef)ip;
#endif
	}

	// Decodes a standard polygon reference.								標準のポリゴン参照をデコードします。
	//  @note This function is generally meant for internal use only.	この関数は通常、内部使用のみを目的としています。
	//  @param[in]	ref   The polygon reference to decode.				デコードするポリゴン参照。
	//  @param[out]	salt	The tile's salt value.							タイルのソルト値。
	//  @param[out]	it		The index of the tile.						タイルのインデックス。
	//  @param[out]	ip		The index of the polygon within the tile.		タイル内のポリゴンのインデックス。
	//  @see #encodePolyId
	inline void decodePolyId(dtPolyRef ref, unsigned int& salt, unsigned int& it, unsigned int& ip) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef saltMask = ((dtPolyRef)1 << DT_SALT_BITS) - 1;
		const dtPolyRef tileMask = ((dtPolyRef)1 << DT_TILE_BITS) - 1;
		const dtPolyRef polyMask = ((dtPolyRef)1 << DT_POLY_BITS) - 1;
		salt = (unsigned int)((ref >> (DT_POLY_BITS + DT_TILE_BITS))& saltMask);
		it = (unsigned int)((ref >> DT_POLY_BITS)& tileMask);
		ip = (unsigned int)(ref & polyMask);
#else
		const dtPolyRef saltMask = ((dtPolyRef)1 << m_saltBits) - 1;
		const dtPolyRef tileMask = ((dtPolyRef)1 << m_tileBits) - 1;
		const dtPolyRef polyMask = ((dtPolyRef)1 << m_polyBits) - 1;
		salt = (unsigned int)((ref >> (m_polyBits + m_tileBits))& saltMask);
		it = (unsigned int)((ref >> m_polyBits)& tileMask);
		ip = (unsigned int)(ref & polyMask);
#endif
	}

	// Extracts a tile's salt value from the specified polygon reference.
	// 指定されたポリゴン参照からタイルのソルト値を抽出します。
	//  @note This function is generally meant for internal use only.
	//  @param[in]	ref		The polygon reference.
	//  @see #encodePolyId
	inline unsigned int decodePolyIdSalt(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef saltMask = ((dtPolyRef)1 << DT_SALT_BITS) - 1;
		return (unsigned int)((ref >> (DT_POLY_BITS + DT_TILE_BITS))& saltMask);
#else
		const dtPolyRef saltMask = ((dtPolyRef)1 << m_saltBits) - 1;
		return (unsigned int)((ref >> (m_polyBits + m_tileBits))& saltMask);
#endif
	}

	// Extracts the tile's index from the specified polygon reference.
	//  @note This function is generally meant for internal use only.
	//  @param[in]	ref		The polygon reference.
	//  @see #encodePolyId
	inline unsigned int decodePolyIdTile(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef tileMask = ((dtPolyRef)1 << DT_TILE_BITS) - 1;
		return (unsigned int)((ref >> DT_POLY_BITS)& tileMask);
#else
		const dtPolyRef tileMask = ((dtPolyRef)1 << m_tileBits) - 1;
		return (unsigned int)((ref >> m_polyBits)& tileMask);
#endif
	}

	// Extracts the polygon's index (within its tile) from the specified polygon reference.
	//  @note This function is generally meant for internal use only.
	//  @param[in]	ref		The polygon reference.
	//  @see #encodePolyId
	inline unsigned int decodePolyIdPoly(dtPolyRef ref) const
	{
#ifdef DT_POLYREF64
		const dtPolyRef polyMask = ((dtPolyRef)1 << DT_POLY_BITS) - 1;
		return (unsigned int)(ref & polyMask);
#else
		const dtPolyRef polyMask = ((dtPolyRef)1 << m_polyBits) - 1;
		return (unsigned int)(ref & polyMask);
#endif
	}

	// @}

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtNavMesh(const dtNavMesh&);
	dtNavMesh& operator=(const dtNavMesh&);

	// Returns pointer to tile in the tile array.
	dtMeshTile* getTile(int i);

	// Returns neighbour tile based on side.
	int getTilesAt(const int x, const int y,
		dtMeshTile** tiles, const int maxTiles) const;

	// Returns neighbour tile based on side.
	int getNeighbourTilesAt(const int x, const int y, const int side,
		dtMeshTile** tiles, const int maxTiles) const;

	// Returns all polygons in neighbour tile based on portal defined by the segment.
	int findConnectingPolys(const float* va, const float* vb,
		const dtMeshTile* tile, int side,
		dtPolyRef* con, float* conarea, int maxcon) const;

	// Builds internal polygons links for a tile.
	void connectIntLinks(dtMeshTile* tile);
	// Builds internal polygons links for a tile.
	void baseOffMeshLinks(dtMeshTile* tile);

	// Builds external polygon links for a tile.
	void connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side);
	// Builds external polygon links for a tile.
	void connectExtOffMeshLinks(dtMeshTile* tile, dtMeshTile* target, int side);

	// Removes external links at specified side.
	void unconnectLinks(dtMeshTile* tile, dtMeshTile* target);

	// TODO: These methods are duplicates from dtNavMeshQuery, but are needed for off-mesh connection finding.

	// Queries polygons within a tile.
	int queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
		dtPolyRef* polys, const int maxPolys) const;
	// Find nearest polygon within a tile.
	dtPolyRef findNearestPolyInTile(const dtMeshTile* tile, const float* center,
		const float* extents, float* nearestPt) const;
	// Returns closest point on polygon.
	void closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;

	dtNavMeshParams m_params;			//< Current initialization params. TODO: do not store this info twice.
	float m_orig[3];					//< Origin of the tile (0,0) タイルの原点
	float m_tileWidth, m_tileHeight;	//< Dimensions of each tile. 各タイルの寸法。
	int m_maxTiles;						//< Max number of tiles.
	int m_tileLutSize;					//< Tile hash lookup size (must be pot).
	int m_tileLutMask;					//< Tile hash lookup mask.

	dtMeshTile** m_posLookup;			//< Tile hash lookup.
	dtMeshTile* m_nextFree;				//< Freelist of tiles.
	dtMeshTile* m_tiles;				//< List of tiles.

#ifndef DT_POLYREF64
	unsigned int m_saltBits;			//< Number of salt bits in the tile ID.
	unsigned int m_tileBits;			//< Number of tile bits in the tile ID.
	unsigned int m_polyBits;			//< Number of poly bits in the tile ID.
#endif
};

// Allocates a navigation mesh object using the Detour allocator.
// @return A navigation mesh that is ready for initialization, or null on failure.
//  @ingroup detour
dtNavMesh* dtAllocNavMesh();

// Frees the specified navigation mesh object using the Detour allocator.
// Detourアロケーターを使用して、指定されたナビゲーションメッシュオブジェクトを解放します。
//  @param[in]	navmesh		A navigation mesh allocated using #dtAllocNavMesh
//  @ingroup detour
void dtFreeNavMesh(dtNavMesh* navmesh);

#endif // DETOURNAVMESH_H

//////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.
// このセクションには、ソースファイルを持たないメンバー向けの詳細なドキュメントが含まれています。
// ヘッダーのメインセクションの混乱を軽減します。

/**

@typedef dtPolyRef
@par

Polygon references are subject to the same invalidate/preserve/restore
rules that apply to #dtTileRef's.  If the #dtTileRef for the polygon's
tile changes, the polygon reference becomes invalid.
//ポリゴン参照は、＃dtTileRefに適用されるものと同じ無効化/保存/復元のルールに従います。
//ポリゴンのタイルの#dtTileRefが変更されると、ポリゴン参照は無効になります。

Changing a polygon's flags, area id, etc. does not impact its polygon
reference.
//ポリゴンのフラグ、エリアIDなどを変更しても、ポリゴンの参照には影響しません。

@typedef dtTileRef
@par

The following changes will invalidate a tile reference:
//次の変更により、タイル参照が無効になります。

- The referenced tile has been removed from the navigation mesh.
- The navigation mesh has been initialized using a different set
  of #dtNavMeshParams.
//-参照されたタイルがナビゲーションメッシュから削除されました。
//-ナビゲーションメッシュは、＃dtNavMeshParamsの異なるセットを使用して初期化されました。

A tile reference is preserved/restored if the tile is added to a navigation
mesh initialized with the original #dtNavMeshParams and is added at the
original reference location. (E.g. The lastRef parameter is used with
dtNavMesh::addTile.)
//タイルが元の#dtNavMeshParamsで初期化されたナビゲーションメッシュに追加され、元の参照位置に追加された場合、タイル参照は保持/復元されます。
//（たとえば、lastRefパラメーターはdtNavMesh::addTileで使用されます。）

Basically, if the storage structure of a tile changes, its associated
tile reference changes.
//基本的に、タイルのストレージ構造が変更されると、関連するタイル参照が変更されます。

@var unsigned short dtPoly::neis[DT_VERTS_PER_POLYGON]
@par

Each entry represents data for the edge starting at the vertex of the same index.
E.g. The entry at index n represents the edge data for vertex[n] to vertex[n+1].
// 各エントリは、同じインデックスの頂点から始まるエッジのデータを表します。
// 例えば、インデックスnのエントリは、頂点[n]から頂点[n + 1]のエッジデータを表します。

A value of zero indicates the edge has no polygon connection. (It makes up the
border of the navigation mesh.)
//値ゼロは、エッジにポリゴン接続がないことを示します。
//（ナビゲーションメッシュの境界線を構成します。）

The information can be extracted as follows:
//情報は次のように抽出できます。

@code
neighborRef = neis[n] & 0xff; // Get the neighbor polygon reference.
//近隣のポリゴン参照を取得します。

if (neis[n] & #DT_EX_LINK)
{
	// The edge is an external (portal) edge.
	//エッジは外部（ポータル）エッジです。
}
@endcode

@var float dtMeshHeader::bvQuantFactor
@par

This value is used for converting between world and bounding volume coordinates.
//この値は、ワールドと境界ボリューム座標の間の変換に使用されます。

For example:
@code
const float cs = 1.f / tile->header->bvQuantFactor;
const dtBVNode* n = &tile->bvTree[i];
if (n->i >= 0)
{
	// This is a leaf node.
	// これはリーフノードです。
	float worldMinX = tile->header->bmin[0] + n->bmin[0]*cs;
	float worldMinY = tile->header->bmin[0] + n->bmin[1]*cs;
	// Etc...
}
@endcode

@struct dtMeshTile
@par

Tiles generally only exist within the context of a dtNavMesh object.
//通常、タイルはdtNavMeshオブジェクトのコンテキスト内にのみ存在します。

Some tile content is optional.  For example, a tile may not contain any
off-mesh connections.  In this case the associated pointer will be null.
//一部のタイルコンテンツはオプションです。
//たとえば、タイルにオフメッシュ接続が含まれていない場合があります。
//この場合、関連付けられたポインターはnullになります。

If a detail mesh exists it will share vertices with the base polygon mesh.
Only the vertices unique to the detail mesh will be stored in #detailVerts.
//詳細メッシュが存在する場合、ベースポリゴンメッシュと頂点を共有します。
//詳細メッシュに固有の頂点のみが#detailVertsに保存されます。

@warning Tiles returned by a dtNavMesh object are not guarenteed to be populated.
For example: The tile at a location might not have been loaded yet, or may have been removed.
In this case, pointers will be null.  So if in doubt, check the polygon count in the
tile's header to determine if a tile has polygons defined.
//dtNavMeshオブジェクトによって返される@warning Tilesは、設定されることを保証されません。
//例：ロケーションのタイルはまだロードされていないか、削除されている可能性があります。
//この場合、ポインターはnullになります。
//疑わしい場合は、タイルのヘッダーのポリゴン数を確認して、タイルにポリゴンが定義されているかどうかを判断してください。

@var float dtOffMeshConnection::pos[6]
@par

For a properly built navigation mesh, vertex A will always be within the bounds of the mesh.
Vertex B is not required to be within the bounds of the mesh.
//適切に構築されたナビゲーションメッシュの場合、頂点Aは常にメッシュの境界内にあります。
//頂点Bはメッシュの境界内にある必要はありません。
*/

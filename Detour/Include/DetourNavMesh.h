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
//�i�r�Q�[�V�������b�V���^�C�����̃|���S���̃n���h���B
typedef unsigned int dtPolyRef;
#endif

// A handle to a tile within a navigation mesh.
// @ingroup detour
#ifdef DT_POLYREF64
typedef uint64_t dtTileRef;
#else
// �i�r�Q�[�V�������b�V�����̃^�C���ւ̃n���h���B
typedef unsigned int dtTileRef;
#endif

// The maximum number of vertices per navigation polygon.
// �i�r�Q�[�V�����|���S�����Ƃ̒��_�̍ő吔�B
// @ingroup detour
constexpr int DT_VERTS_PER_POLYGON = 6;

// @{
// @name Tile Serialization Constants
// These constants are used to detect whether a navigation tile's data and state format is compatible with the current build.
// �����̒萔�́A�i�r�Q�[�V�����^�C���̃f�[�^�Ə�Ԃ̌`�������݂̃r���h�ƌ݊��������邩�ǂ��������o���邽�߂Ɏg�p����܂��B
//

// A magic number used to detect compatibility of navigation tile data.
// �i�r�Q�[�V�����^�C���f�[�^�̌݊��������o���邽�߂Ɏg�p�����}�W�b�N�i���o�[�B
constexpr int DT_NAVMESH_MAGIC = 'D' << 24 | 'N' << 16 | 'A' << 8 | 'V';

// A version number used to detect compatibility of navigation tile data.
// �i�r�Q�[�V�����^�C���f�[�^�̌݊��������o���邽�߂Ɏg�p�����o�[�W�����ԍ��B
constexpr int DT_NAVMESH_VERSION = 7;

// A magic number used to detect the compatibility of navigation tile states.
// �i�r�Q�[�V�����^�C���̏�Ԃ̌݊��������o���邽�߂Ɏg�p�����}�W�b�N�i���o�[�B
constexpr int DT_NAVMESH_STATE_MAGIC = 'D' << 24 | 'N' << 16 | 'M' << 8 | 'S';

// A version number used to detect compatibility of navigation tile states.
// �i�r�Q�[�V�����^�C���̏�Ԃ̌݊��������o���邽�߂Ɏg�p�����o�[�W�����ԍ��B
constexpr int DT_NAVMESH_STATE_VERSION = 1;

// @}

// A flag that indicates that an entity links to an external entity.
// �G���e�B�e�B���O���G���e�B�e�B�Ƀ����N���Ă��邱�Ƃ������t���O�B
// (E.g. A polygon edge is a portal that links to another polygon.)
// �i���Ƃ��΁A�|���S���G�b�W�́A�ʂ̃|���S���Ƀ����N����|�[�^���ł��B�j
constexpr unsigned short DT_EXT_LINK = 0x8000;

// A value that indicates the entity does not link to anything.
// �G���e�B�e�B�����ɂ������N���Ă��Ȃ����Ƃ������l�B
constexpr unsigned int DT_NULL_LINK = 0xffffffff;

// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
// �I�t���b�V���ڑ��𗼕����ɉ��f�ł��邱�Ƃ������t���O�B�i�o�����ł��B�j
constexpr unsigned int DT_OFFMESH_CON_BIDIR = 1;

// The maximum number of user defined area ids.
// ���[�U�[��`�̃G���AID�̍ő吔�B
// @ingroup detour
constexpr int DT_MAX_AREAS = 64;

// Tile flags used for various functions and fields.
// ���܂��܂Ȋ֐��ƃt�B�[���h�Ɏg�p�����^�C���t���O�B
// For an example, see dtNavMesh::addTile().
// ��ɂ��ẮAdtNavMesh :: addTile�i�j���Q�Ƃ��Ă��������B
enum dtTileFlags
{
	// The navigation mesh owns the tile memory and is responsible for freeing it.
	// �i�r�Q�[�V�������b�V���̓^�C�������������L���A�^�C����������������܂��B
	DT_TILE_FREE_DATA = 0x01,
};

// Vertex flags returned by dtNavMeshQuery::findStraightPath.
// dtNavMeshQuery :: findStraightPath�ɂ���ĕԂ���钸�_�t���O�B
enum dtStraightPathFlags
{
	// The vertex is the start position in the path.
	// ���_�̓p�X�̊J�n�ʒu�ł��B
	DT_STRAIGHTPATH_START = 0x01,
	// The vertex is the end position in the path.
	// ���_�̓p�X���̏I���ʒu�ł��B
	DT_STRAIGHTPATH_END = 0x02,
	// The vertex is the start of an off-mesh connection.
	// ���_�̓I�t���b�V���ڑ��̊J�n�_�ł��B
	DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04,
};

// Options for dtNavMeshQuery::findStraightPath.
// dtNavMeshQuery :: findStraightPath�̃I�v�V�����B
enum dtStraightPathOptions
{
	// Add a vertex at every polygon edge crossing where area changes.
	// �ʐς��ς��ꏊ�Ō������邷�ׂẴ|���S���G�b�W�ɒ��_��ǉ����܂��B
	DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01,
	// Add a vertex at every polygon edge crossing.
	// �|���S���̃G�b�W���������邽�тɒ��_��ǉ����܂��B
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
// �i�r�Q�[�V�������b�V���|���S���̃^�C�v��\���t���O�B
enum dtPolyTypes
{
	// The polygon is a standard convex polygon that is part of the surface of the mesh.
	// �|���S���́A���b�V���̕\�ʂ̈ꕔ�ł���W���̓ʃ|���S���ł��B
	DT_POLYTYPE_GROUND = 0,
	// The polygon is an off-mesh connection consisting of two vertices.
	// �|���S���́A2�̒��_�ō\������郁�b�V���O�̐ڑ��ł��B
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
// dtMeshTile�I�u�W�F�N�g�Ɋ֘A���鍂���x���̏���񋟂��܂��B
// @ingroup detour
struct dtMeshHeader
{
	//< Tile magic number. (Used to identify the data format.)
	// �}�W�b�N�ԍ�����ׂĕ\�����܂��B �i�f�[�^�`�������ʂ��邽�߂Ɏg�p����܂��B�j
	int magic;
	//< Tile data format version number.
	// �^�C���f�[�^�`���̃o�[�W�����ԍ��B
	int version;
	//< The x-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	// dtNavMesh�^�C���O���b�h���̃^�C����x�ʒu�B �ix�Ay�A���C���[�j
	int x;
	//< The y-position of the tile within the dtNavMesh tile grid. (x, y, layer)
	// dtNavMesh�^�C���O���b�h���̃^�C����y�ʒu�B �ix�Ay�A���C���[�j
	int y;
	//< The layer of the tile within the dtNavMesh tile grid. (x, y, layer)
	// dtNavMesh�^�C���O���b�h���̃^�C���̃��C���[�B �ix�Ay�A���C���[�j
	int layer;
	//< The user defined id of the tile.
	// ���[�U�[��`�̃^�C����ID�B
	unsigned int userId;
	//< The number of polygons in the tile.
	// �^�C�����̃|���S���̐��B
	int polyCount;
	//< The number of vertices in the tile.
	// �^�C�����̒��_�̐��B
	int vertCount;
	//< The number of allocated links.
	// ���蓖�Ă�ꂽ�����N�̐��B
	int maxLinkCount;
	//< The number of sub-meshes in the detail mesh.
	// �f�B�e�[�����b�V���̃T�u���b�V���̐��B
	int detailMeshCount;


	// The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
	// �f�B�e�[�����b�V�����̈�ӂ̒��_�̐��B �i�|���S���̒��_�ɉ����āj
	int detailVertCount;

	//< The number of triangles in the detail mesh.
	// �f�B�e�[�����b�V���̎O�p�`�̐��B
	int detailTriCount;
	//< The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	// �o�E���f�B���O�{�����[���m�[�h�̐��B �i�o�E���f�B���O�{�����[���������ɂȂ��Ă���ꍇ�̓[���j�B
	int bvNodeCount;
	//< The number of off-mesh connections.
	// �I�t���b�V���ڑ��̐��B
	int offMeshConCount;
	//< The index of the first polygon which is an off-mesh connection.
	// �I�t���b�V���ڑ��ł���ŏ��̃|���S���̃C���f�b�N�X�B
	int offMeshBase;
	//< The height of the agents using the tile.
	// �^�C�����g�p����G�[�W�F���g�̍����B
	float walkableHeight;
	//< The radius of the agents using the tile.
	// �^�C�����g�p����G�[�W�F���g�̔��a�B
	float walkableRadius;
	//< The maximum climb height of the agents using the tile.
	// �^�C�����g�p����G�[�W�F���g�̍ő�㏸�����B
	float walkableClimb;
	//< The minimum bounds of the tile's AABB. [(x, y, z)]
	// �^�C����AABB�̍ŏ����E�B [�ix�Ay�Az�j]
	float bmin[3];
	//< The maximum bounds of the tile's AABB. [(x, y, z)]
	// �^�C����AABB�̍ő勫�E�B [�ix�Ay�Az�j]
	float bmax[3];

	// The bounding volume quantization factor.
	// �o�E���f�B���O�{�����[���̗ʎq���W���B
	float bvQuantFactor;
};

// Defines a navigation mesh tile.
// �i�r�Q�[�V�������b�V���^�C�����`���܂��B
// @ingroup detour
struct dtMeshTile
{
	//< Counter describing modifications to the tile.
	// �^�C���ւ̕ύX���������J�E���^�[�B
	unsigned int salt;
	//< Index to the next free link.
	// ���̋󂫃����N�ւ̃C���f�b�N�X�B
	unsigned int linksFreeList;
	//< The tile header.
	// �^�C���w�b�_�[�B
	dtMeshHeader* header;
	//< The tile polygons. [Size: dtMeshHeader::polyCount]
	// �^�C���|���S���B [�T�C�Y�FdtMeshHeader :: polyCount]
	dtPoly* polys;
	//< The tile vertices. [Size: dtMeshHeader::vertCount]
	// �^�C���̒��_�B [�T�C�Y�FdtMeshHeader :: vertCount]
	float* verts;
	//< The tile links. [Size: dtMeshHeader::maxLinkCount]
	// �^�C�������N�B [�T�C�Y�FdtMeshHeader :: maxLinkCount]
	dtLink* links;
	//< The tile's detail sub-meshes. [Size: dtMeshHeader::detailMeshCount]
	// �^�C���̏ڍ׃T�u���b�V���B [�T�C�Y�FdtMeshHeader :: detailMeshCount]
	dtPolyDetail* detailMeshes;


	// The detail mesh's unique vertices. [(x, y, z) * dtMeshHeader::detailVertCount]
	// �ڍ׃��b�V���̈�ӂ̒��_�B [�iX�Ay�Az�j* dtMeshHeader :: detailVertCount]
	float* detailVerts;

	// The detail mesh's triangles. [(vertA, vertB, vertC) * dtMeshHeader::detailTriCount]
	// �ڍ׃��b�V���̎O�p�`�B [�iVertA�AvertB�AvertC�j* dtMeshHeader :: detailTriCount]
	unsigned char* detailTris;

	// The tile bounding volume nodes. [Size: dtMeshHeader::bvNodeCount] (Will be null if bounding volumes are disabled.)
	// �^�C�����E�{�����[���m�[�h�B [�T�C�Y�FdtMeshHeader :: bvNodeCount]�i�o�E���f�B���O�{�����[���������ɂȂ��Ă���ꍇ��null�ɂȂ�܂��B�j
	dtBVNode* bvTree;

	//< The tile off-mesh connections. [Size: dtMeshHeader::offMeshConCount]
	// �^�C���̃I�t���b�V���ڑ��B [�T�C�Y�FdtMeshHeader :: offMeshConCount]
	dtOffMeshConnection* offMeshCons;

	//< The tile data. (Not directly accessed under normal situations.)
	// �^�C���f�[�^�B �i�ʏ�͒��ڃA�N�Z�X����܂���B�j
	unsigned char* data;
	//< Size of the tile data.
	// �^�C���f�[�^�̃T�C�Y�B
	int dataSize;
	//< Tile flags. (See: #dtTileFlags)
	// �^�C���t���O�B �i�Q�ƁF#dtTileFlags�j
	int flags;
	//< The next free tile, or the next tile in the spatial grid.
	// ���̃t���[�^�C���A�܂��͋�ԃO���b�h�̎��̃^�C���B
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
	// �������ƃ^�C���Ǘ��B

	// Initializes the navigation mesh for tiled use.
	// �^�C�������ꂽ�g�p�̂��߂Ƀi�r�Q�[�V�������b�V�������������܂��B
	// @param[in] params : Initialization parameters.
	// �������p�����[�^�[�B
	// @return The status flags for the operation.
	// ����̃X�e�[�^�X�t���O�B
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
	//	�i�r�Q�[�V�������b�V���Ƀ^�C����ǉ����܂��B
	//  @param[in] data : Data for the new tile mesh. (See: #dtCreateNavMeshData)
	//	�V�����^�C�����b�V���̃f�[�^�B �i#dtCreateNavMeshData���Q�Ɓj
	//  @param[in] dataSize : Data size of the new tile mesh.
	//	�V�����^�C�����b�V���̃f�[�^�T�C�Y�B
	//  @param[in] flags : Tile flags. (See: #dtTileFlags)
	//	�^�C���t���O�B �i�Q�ƁF#dtTileFlags�j
	//  @param[in] lastRef : The desired reference for the tile. (When reloading a tile.) [opt] [Default: 0]
	//	�^�C���ɕK�v�ȎQ�ƁB �i�^�C���������[�h����ꍇ�B�j[�f�t�H���g�F0]
	//  @param[out] result : The tile reference. (If the tile was succesfully added.) [opt]
	//	�^�C���Q�ƁB �i�^�C��������ɒǉ����ꂽ�ꍇ�B�j
	// @return The status flags for the operation.
	//	����̃X�e�[�^�X�t���O�B
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
	// �w�肳�ꂽ�|���S���Q�Ƃ̃^�C���ƃ|���S����Ԃ��܂��B
	// Param[in]  �|���S���̊��m�̗L���ȎQ�ƁB
	// param[out] �|���S�����܂ރ^�C���B
	// param[out] poly�|���S���B
	void getTileAndPolyByRefUnsafe(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const;

	// Checks the validity of a polygon reference.							�|���S���Q�Ƃ̗L�������m�F���܂��B
	//  @param[in]	ref		The polygon reference to check.					ref�`�F�b�N����|���S���Q�ƁB
	// @return True if polygon reference is valid for the navigation mesh.	�i�r�Q�[�V�������b�V���Ń|���S���Q�Ƃ��L���ȏꍇ��True�B
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

	// Decodes a standard polygon reference.								�W���̃|���S���Q�Ƃ��f�R�[�h���܂��B
	//  @note This function is generally meant for internal use only.	���̊֐��͒ʏ�A�����g�p�݂̂�ړI�Ƃ��Ă��܂��B
	//  @param[in]	ref   The polygon reference to decode.				�f�R�[�h����|���S���Q�ƁB
	//  @param[out]	salt	The tile's salt value.							�^�C���̃\���g�l�B
	//  @param[out]	it		The index of the tile.						�^�C���̃C���f�b�N�X�B
	//  @param[out]	ip		The index of the polygon within the tile.		�^�C�����̃|���S���̃C���f�b�N�X�B
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
	// �w�肳�ꂽ�|���S���Q�Ƃ���^�C���̃\���g�l�𒊏o���܂��B
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
	float m_orig[3];					//< Origin of the tile (0,0) �^�C���̌��_
	float m_tileWidth, m_tileHeight;	//< Dimensions of each tile. �e�^�C���̐��@�B
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
// Detour�A���P�[�^�[���g�p���āA�w�肳�ꂽ�i�r�Q�[�V�������b�V���I�u�W�F�N�g��������܂��B
//  @param[in]	navmesh		A navigation mesh allocated using #dtAllocNavMesh
//  @ingroup detour
void dtFreeNavMesh(dtNavMesh* navmesh);

#endif // DETOURNAVMESH_H

//////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.
// ���̃Z�N�V�����ɂ́A�\�[�X�t�@�C���������Ȃ������o�[�����̏ڍׂȃh�L�������g���܂܂�Ă��܂��B
// �w�b�_�[�̃��C���Z�N�V�����̍������y�����܂��B

/**

@typedef dtPolyRef
@par

Polygon references are subject to the same invalidate/preserve/restore
rules that apply to #dtTileRef's.  If the #dtTileRef for the polygon's
tile changes, the polygon reference becomes invalid.
//�|���S���Q�Ƃ́A��dtTileRef�ɓK�p�������̂Ɠ���������/�ۑ�/�����̃��[���ɏ]���܂��B
//�|���S���̃^�C����#dtTileRef���ύX�����ƁA�|���S���Q�Ƃ͖����ɂȂ�܂��B

Changing a polygon's flags, area id, etc. does not impact its polygon
reference.
//�|���S���̃t���O�A�G���AID�Ȃǂ�ύX���Ă��A�|���S���̎Q�Ƃɂ͉e�����܂���B

@typedef dtTileRef
@par

The following changes will invalidate a tile reference:
//���̕ύX�ɂ��A�^�C���Q�Ƃ������ɂȂ�܂��B

- The referenced tile has been removed from the navigation mesh.
- The navigation mesh has been initialized using a different set
  of #dtNavMeshParams.
//-�Q�Ƃ��ꂽ�^�C�����i�r�Q�[�V�������b�V������폜����܂����B
//-�i�r�Q�[�V�������b�V���́A��dtNavMeshParams�̈قȂ�Z�b�g���g�p���ď���������܂����B

A tile reference is preserved/restored if the tile is added to a navigation
mesh initialized with the original #dtNavMeshParams and is added at the
original reference location. (E.g. The lastRef parameter is used with
dtNavMesh::addTile.)
//�^�C��������#dtNavMeshParams�ŏ��������ꂽ�i�r�Q�[�V�������b�V���ɒǉ�����A���̎Q�ƈʒu�ɒǉ����ꂽ�ꍇ�A�^�C���Q�Ƃ͕ێ�/��������܂��B
//�i���Ƃ��΁AlastRef�p�����[�^�[��dtNavMesh::addTile�Ŏg�p����܂��B�j

Basically, if the storage structure of a tile changes, its associated
tile reference changes.
//��{�I�ɁA�^�C���̃X�g���[�W�\�����ύX�����ƁA�֘A����^�C���Q�Ƃ��ύX����܂��B

@var unsigned short dtPoly::neis[DT_VERTS_PER_POLYGON]
@par

Each entry represents data for the edge starting at the vertex of the same index.
E.g. The entry at index n represents the edge data for vertex[n] to vertex[n+1].
// �e�G���g���́A�����C���f�b�N�X�̒��_����n�܂�G�b�W�̃f�[�^��\���܂��B
// �Ⴆ�΁A�C���f�b�N�Xn�̃G���g���́A���_[n]���璸�_[n + 1]�̃G�b�W�f�[�^��\���܂��B

A value of zero indicates the edge has no polygon connection. (It makes up the
border of the navigation mesh.)
//�l�[���́A�G�b�W�Ƀ|���S���ڑ����Ȃ����Ƃ������܂��B
//�i�i�r�Q�[�V�������b�V���̋��E�����\�����܂��B�j

The information can be extracted as follows:
//���͎��̂悤�ɒ��o�ł��܂��B

@code
neighborRef = neis[n] & 0xff; // Get the neighbor polygon reference.
//�ߗׂ̃|���S���Q�Ƃ��擾���܂��B

if (neis[n] & #DT_EX_LINK)
{
	// The edge is an external (portal) edge.
	//�G�b�W�͊O���i�|�[�^���j�G�b�W�ł��B
}
@endcode

@var float dtMeshHeader::bvQuantFactor
@par

This value is used for converting between world and bounding volume coordinates.
//���̒l�́A���[���h�Ƌ��E�{�����[�����W�̊Ԃ̕ϊ��Ɏg�p����܂��B

For example:
@code
const float cs = 1.f / tile->header->bvQuantFactor;
const dtBVNode* n = &tile->bvTree[i];
if (n->i >= 0)
{
	// This is a leaf node.
	// ����̓��[�t�m�[�h�ł��B
	float worldMinX = tile->header->bmin[0] + n->bmin[0]*cs;
	float worldMinY = tile->header->bmin[0] + n->bmin[1]*cs;
	// Etc...
}
@endcode

@struct dtMeshTile
@par

Tiles generally only exist within the context of a dtNavMesh object.
//�ʏ�A�^�C����dtNavMesh�I�u�W�F�N�g�̃R���e�L�X�g���ɂ̂ݑ��݂��܂��B

Some tile content is optional.  For example, a tile may not contain any
off-mesh connections.  In this case the associated pointer will be null.
//�ꕔ�̃^�C���R���e���c�̓I�v�V�����ł��B
//���Ƃ��΁A�^�C���ɃI�t���b�V���ڑ����܂܂�Ă��Ȃ��ꍇ������܂��B
//���̏ꍇ�A�֘A�t����ꂽ�|�C���^�[��null�ɂȂ�܂��B

If a detail mesh exists it will share vertices with the base polygon mesh.
Only the vertices unique to the detail mesh will be stored in #detailVerts.
//�ڍ׃��b�V�������݂���ꍇ�A�x�[�X�|���S�����b�V���ƒ��_�����L���܂��B
//�ڍ׃��b�V���ɌŗL�̒��_�݂̂�#detailVerts�ɕۑ�����܂��B

@warning Tiles returned by a dtNavMesh object are not guarenteed to be populated.
For example: The tile at a location might not have been loaded yet, or may have been removed.
In this case, pointers will be null.  So if in doubt, check the polygon count in the
tile's header to determine if a tile has polygons defined.
//dtNavMesh�I�u�W�F�N�g�ɂ���ĕԂ����@warning Tiles�́A�ݒ肳��邱�Ƃ�ۏ؂���܂���B
//��F���P�[�V�����̃^�C���͂܂����[�h����Ă��Ȃ����A�폜����Ă���\��������܂��B
//���̏ꍇ�A�|�C���^�[��null�ɂȂ�܂��B
//�^�킵���ꍇ�́A�^�C���̃w�b�_�[�̃|���S�������m�F���āA�^�C���Ƀ|���S������`����Ă��邩�ǂ����𔻒f���Ă��������B

@var float dtOffMeshConnection::pos[6]
@par

For a properly built navigation mesh, vertex A will always be within the bounds of the mesh.
Vertex B is not required to be within the bounds of the mesh.
//�K�؂ɍ\�z���ꂽ�i�r�Q�[�V�������b�V���̏ꍇ�A���_A�͏�Ƀ��b�V���̋��E���ɂ���܂��B
//���_B�̓��b�V���̋��E���ɂ���K�v�͂���܂���B
*/

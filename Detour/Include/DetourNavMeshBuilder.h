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
// �i�r�Q�[�V�������b�V���^�C���̍\�z�Ɏg�p�����\�[�X�f�[�^��\���܂��B
// @ingroup detour
struct dtNavMeshCreateParams
{
	// @name Polygon Mesh Attributes // �|���S�����b�V���̑���
	// Used to create the base navigation graph.
	// �x�[�X�i�r�Q�[�V�����O���t�̍쐬�Ɏg�p����܂��B
	// See #rcPolyMesh for details related to these attributes.
	// @{
	// The polygon mesh vertices. [(x, y, z) * #vertCount] [Unit: vx]
	// �|���S�����b�V���̒��_�B [�ix�Ay�Az�j* #vertCount] [�P�ʁFvx]
	const unsigned short* verts;

	// The number vertices in the polygon mesh. [Limit: >= 3]
	// �|���S�����b�V���̒��_�̐��B [�����F> = 3]
	int vertCount;

	// The polygon data. [Size: #polyCount * 2 * #nvp]
	// �|���S���f�[�^�B [�T�C�Y�F#polyCount * 2 * #nvp]
	const unsigned short* polys;

	// The user defined flags assigned to each polygon. [Size: #polyCount]
	// �e�|���S���Ɋ��蓖�Ă�ꂽ���[�U�[��`�̃t���O�B [�T�C�Y�F#polyCount]
	const unsigned short* polyFlags;

	// The user defined area ids assigned to each polygon. [Size: #polyCount]
	// �e�|���S���Ɋ��蓖�Ă�ꂽ���[�U�[��`�̃G���AID�B [�T�C�Y�F#polyCount]
	const unsigned char* polyAreas;

	// Number of polygons in the mesh. [Limit: >= 1]
	// ���b�V�����̃|���S���̐��B [�����F> = 1]
	int polyCount;

	// Number maximum number of vertices per polygon. [Limit: >= 3]
	// �|���S�����Ƃ̒��_�̍ő吔�𐔂��܂��B [�����F> = 3]
	int nvp;

	// @}
	// @name Height Detail Attributes (Optional) // �����ڍב����i�I�v�V�����j
	// See #rcPolyMeshDetail for details related to these attributes.
	// @{
	// The height detail sub-mesh data. [Size: 4 * #polyCount]
	// �����̏ڍ׃T�u���b�V���f�[�^�B [�T�C�Y�F4 * #polyCount]
	const unsigned int* detailMeshes;

	// The detail mesh vertices. [Size: 3 * #detailVertsCount] [Unit: wu]
	// �ڍ׃��b�V���̒��_�B [�T�C�Y�F3 * #detailVertsCount] [�P�ʁFwu]
	const float* detailVerts;

	// The number of vertices in the detail mesh.
	// �ڍ׃��b�V���̒��_�̐��B
	int detailVertsCount;

	// The detail mesh triangles. [Size: 4 * #detailTriCount]
	// �ڍ׃��b�V���̎O�p�`�B [�T�C�Y�F4 * #detailTriCount]
	const unsigned char* detailTris;

	// The number of triangles in the detail mesh.
	// �ڍ׃��b�V���̎O�p�`�̐��B
	int detailTriCount;

	// @}
	// @name Off-Mesh Connections Attributes (Optional) // �I�t���b�V���ڑ������i�I�v�V�����j
	// Used to define a custom point-to-point edge within the navigation graph, an
	// off-mesh connection is a user defined traversable connection made up to two vertices,
	// at least one of which resides within a navigation mesh polygon.
	// �i�r�Q�[�V�����O���t���̃J�X�^���|�C���g�c�[�|�C���g�G�b�W���`���邽�߂Ɏg�p�����I�t���b�V���ڑ��́A
	// 2�܂ł̒��_�ō\������郆�[�U�[��`�̃g���o�[�X�\�Ȑڑ��ł���A
	// ���Ȃ��Ƃ�1�̓i�r�Q�[�V�������b�V���|���S�����ɑ��݂��܂��B
	// {
	// Off-mesh connection vertices. [(ax, ay, az, bx, by, bz) * #offMeshConCount] [Unit: wu]
	// ���b�V���O�̐ڑ����_�B [�iax�Aay�Aaz�Abx�Aby�Abz�j* #offMeshConCount] [�P�ʁFwu]
	const float* offMeshConVerts;

	// Off-mesh connection radii. [Size: #offMeshConCount] [Unit: wu]
	// �I�t���b�V���ڑ����a�B [�T�C�Y�F#offMeshConCount] [�P�ʁFwu]
	const float* offMeshConRad;

	// User defined flags assigned to the off-mesh connections. [Size: #offMeshConCount]
	// �I�t���b�V���ڑ��Ɋ��蓖�Ă�ꂽ���[�U�[��`�t���O�B [�T�C�Y�F#offMeshConCount]
	const unsigned short* offMeshConFlags;

	// User defined area ids assigned to the off-mesh connections. [Size: #offMeshConCount]
	// �I�t���b�V���ڑ��Ɋ��蓖�Ă�ꂽ���[�U�[��`�̃G���AID�B [�T�C�Y�F#offMeshConCount]
	const unsigned char* offMeshConAreas;

	// The permitted travel direction of the off-mesh connections. [Size: #offMeshConCount]
	// �I�t���b�V���ڑ��̋����ꂽ�ړ������B [�T�C�Y�F#offMeshConCount]
	//
	// 0 = Travel only from endpoint A to endpoint B.<br/>
	// 0 =�G���h�|�C���gA����G���h�|�C���gB�ւ̈ړ��̂݁B
	// #DT_OFFMESH_CON_BIDIR = Bidirectional travel.
	const unsigned char* offMeshConDir;

	// The user defined ids of the off-mesh connection. [Size: #offMeshConCount]
	// �I�t���b�V���ڑ��̃��[�U�[��`ID�B [�T�C�Y�F#offMeshConCount]
	const unsigned int* offMeshConUserID;

	// The number of off-mesh connections. [Limit: >= 0]
	// �I�t���b�V���ڑ��̐��B [�����F> = 0]
	int offMeshConCount;

	// @}
	// @name Tile Attributes // �^�C������
	// @note The tile grid/layer data can be left at zero if the destination is a single tile mesh.
	// ���悪�P��̃^�C�����b�V���̏ꍇ�A�^�C���O���b�h/���C���[�f�[�^�̓[���̂܂܂ɂ��邱�Ƃ��ł��܂��B
	// @{
	// The user defined id of the tile.
	// ���[�U�[����`�����^�C����ID�B
	unsigned int userId;

	// The tile's x-grid location within the multi-tile destination mesh. (Along the x-axis.)
	// �}���`�^�C���惁�b�V�����̃^�C����y�O���b�h�̈ʒu�B�ix���ɉ����j
	int tileX;

	// The tile's y-grid location within the multi-tile desitation mesh. (Along the z-axis.)
	// �}���`�^�C����̃��b�V�����̃^�C����y�O���b�h�̈ʒu�B�iz���ɉ����j
	int tileY;

	// The tile's layer within the layered destination mesh. [Limit: >= 0] (Along the y-axis.)
	// ���C���[�����ꂽ���惁�b�V�����̃^�C���̃��C���[�B [�����F> = 0]�iy���ɉ����āB�j
	int tileLayer;

	// The minimum bounds of the tile. [(x, y, z)] [Unit: wu]
	// �^�C���̍ŏ����E�B[�ix�Ay�Az�j] [�P�ʁFwu]
	float bmin[3];

	// The maximum bounds of the tile. [(x, y, z)] [Unit: wu]
	// �^�C���̍ő勫�E�B [�ix�Ay�Az�j] [�P�ʁFwu]
	float bmax[3];

	// @}
	// @name General Configuration Attributes // ��ʓI�ȍ\������
	// @{
	// The agent height. [Unit: wu]
	// �G�[�W�F���g�̍����B [�P�ʁFwu]
	float walkableHeight;

	// The agent radius. [Unit: wu]
	// �G�[�W�F���g�̔��a�B [�P�ʁFwu]
	float walkableRadius;

	// The agent maximum traversable ledge. (Up/Down) [Unit: wu]
	// �G�[�W�F���g�̍ő�ړ��\�I�B �i��/���j[�P�ʁFwu]
	float walkableClimb;

	// The xz-plane cell size of the polygon mesh. [Limit: > 0] [Unit: wu]
	// �|���S�����b�V����xz���ʂ̃Z���T�C�Y�B [�����F> 0] [�P�ʁFwu]
	float cs;

	// The y-axis cell height of the polygon mesh. [Limit: > 0] [Unit: wu]
	// �|���S�����b�V����y���̃Z���̍����B [�����F> 0] [�P�ʁFwu]
	float ch;

	// True if a bounding volume tree should be built for the tile.
	// @note The BVTree is not normally needed for layered navigation meshes.
	// ���E�{�����[���c���[���^�C���ɍ\�z����K�v������ꍇ��True�B
	// @note BVTree�͒ʏ�A�K�w�����ꂽ�i�r�Q�[�V�������b�V���ɂ͕K�v����܂���B
	bool buildBvTree;

	// @}
};

// Builds navigation mesh tile data from the provided tile creation data.
// �񋟂��ꂽ�^�C���쐬�f�[�^����i�r�Q�[�V�������b�V���^�C���f�[�^���\�z���܂��B
// @ingroup detour
// @param[in] params : Tile creation data. // �^�C���쐬�f�[�^�B
// @param[out] outData : The resulting tile data. // ���ʂ̃^�C���f�[�^�B
// @param[out] outDataSize : The size of the tile data array. // �^�C���f�[�^�z��̃T�C�Y�B
// @return True if the tile data was successfully created.
//  �^�C���f�[�^������ɍ쐬���ꂽ�ꍇ��True�B
bool dtCreateNavMeshData(dtNavMeshCreateParams* params, unsigned char** outData, int* outDataSize);

// Swaps the endianess of the tile data's header (#dtMeshHeader).
// �^�C���f�[�^�w�b�_�[�̃G���f�B�A�����������܂��i#dtMeshHeader�j�B
//  @param[in,out]	data		The tile data array.
//  @param[in]		dataSize	The size of the data array.
bool dtNavMeshHeaderSwapEndian(unsigned char* data, const int dataSize);

// Swaps endianess of the tile data.
// �^�C���f�[�^�̃G���f�B�A�������ւ��܂��B
//  @param[in,out]	data		The tile data array.
//  @param[in]		dataSize	The size of the data array.
bool dtNavMeshDataSwapEndian(unsigned char* data, const int dataSize);

#endif // DETOURNAVMESHBUILDER_H

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.
// ���̃Z�N�V�����ɂ́A�\�[�X�t�@�C���������Ȃ������o�[�̏ڍׂȃh�L�������g���܂܂�Ă��܂��B
// �w�b�_�[�̃��C���Z�N�V�����̍������y�����܂��B

/**

@struct dtNavMeshCreateParams
@par

This structure is used to marshal data between the Recast mesh generation pipeline and Detour navigation components.
���̍\���̂́ARecast���b�V�������p�C�v���C����Detour�i�r�Q�[�V�����R���|�[�l���g�ԂŃf�[�^���}�[�V�������O���邽�߂Ɏg�p����܂��B

See the rcPolyMesh and rcPolyMeshDetail documentation for detailed information related to mesh structure.
���b�V���\���Ɋւ���ڍׂɂ��ẮArcPolyMesh�����rcPolyMeshDetail�̃h�L�������g���Q�Ƃ��Ă��������B

Units are usually in voxels (vx) or world units (wu). The units for voxels, grid size, and cell size
are all based on the values of #cs and #ch.
�P�ʂ͒ʏ�A�{�N�Z���ivx�j�܂��̓��[���h�P�ʁiwu�j�ł��B
�{�N�Z���̒P�ʁA�O���b�h�T�C�Y�A����уZ���T�C�Y�͂��ׂāA��cs�����#ch�̒l�Ɋ�Â��Ă��܂��B

The standard navigation mesh build process is to create tile data using dtCreateNavMeshData, then add the tile
to a navigation mesh using either the dtNavMesh single tile <tt>init()</tt> function or the dtNavMesh::addTile()
function.
�W���̃i�r�Q�[�V�������b�V���r���h�v���Z�X�́AdtCreateNavMeshData���g�p���ă^�C���f�[�^���쐬���A
dtNavMesh�V���O���^�C��init�i�j�֐��܂���dtNavMesh :: addTile�i�j�֐����g�p���ă^�C�����i�r�Q�[�V�������b�V���ɒǉ����܂��B

@see dtCreateNavMeshData

*/

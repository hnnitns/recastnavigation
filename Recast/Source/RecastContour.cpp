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

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"

namespace
{
	// �R�[�i�[�̍������擾
	int getCornerHeight(
		int x, int y, int i, int dir,
		const rcCompactHeightfield& chf,
		bool& isBorderVertex)
	{
		const rcCompactSpan& s = chf.spans[i];
		int ch = (int)s.y;
		int dirp = (dir + 1) & 0x3;

		unsigned int regs[4] = { 0,0,0,0 };

		// Combine region and area codes in order to prevent
		// border vertices which are in between two areas to be removed.
		// ���[�W�����R�[�h�ƃG���A�R�[�h��g�ݍ��킹�āA2�̃G���A�̊Ԃɂ��鋫�E���_���폜����Ȃ��悤�ɂ��܂��B
		regs[0] = chf.spans[i].reg | (chf.areas[i] << 16);

		if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
		{
			const int ax = x + rcGetDirOffsetX(dir);
			const int ay = y + rcGetDirOffsetY(dir);
			const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(s, dir);
			const rcCompactSpan& as = chf.spans[ai];

			ch = rcMax(ch, (int)as.y);
			regs[1] = chf.spans[ai].reg | (chf.areas[ai] << 16);

			if (rcGetCon(as, dirp) != RC_NOT_CONNECTED)
			{
				const int ax2 = ax + rcGetDirOffsetX(dirp);
				const int ay2 = ay + rcGetDirOffsetY(dirp);
				const int ai2 = (int)chf.cells[ax2 + ay2 * chf.width].index + rcGetCon(as, dirp);
				const rcCompactSpan& as2 = chf.spans[ai2];

				ch = rcMax(ch, (int)as2.y);
				regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
			}
		}

		if (rcGetCon(s, dirp) != RC_NOT_CONNECTED)
		{
			const int ax = x + rcGetDirOffsetX(dirp);
			const int ay = y + rcGetDirOffsetY(dirp);
			const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(s, dirp);
			const rcCompactSpan& as = chf.spans[ai];

			ch = rcMax(ch, (int)as.y);
			regs[3] = chf.spans[ai].reg | (chf.areas[ai] << 16);

			if (rcGetCon(as, dir) != RC_NOT_CONNECTED)
			{
				const int ax2 = ax + rcGetDirOffsetX(dir);
				const int ay2 = ay + rcGetDirOffsetY(dir);
				const int ai2 = (int)chf.cells[ax2 + ay2 * chf.width].index + rcGetCon(as, dir);
				const rcCompactSpan& as2 = chf.spans[ai2];

				ch = rcMax(ch, (int)as2.y);
				regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
			}
		}

		// Check if the vertex is special edge vertex, these vertices will be removed later.
		// ���_�����ʂȃG�b�W���_���ǂ������m�F���܂��B�����̒��_�͌�ō폜����܂��B
		for (int j = 0; j < 4; ++j)
		{
			const int a = j;
			const int b = (j + 1) & 0x3;
			const int c = (j + 2) & 0x3;
			const int d = (j + 3) & 0x3;

			// The vertex is a border vertex there are two same exterior cells in a row,
			// followed by two interior cells and none of the regions are out of bounds.
			// ���_�͋��E�̒��_�ł���A2�̓����O���Z�����A�����đ��݂��A���̌��2�̓����Z���������A
			// �ǂ̗̈�����E�O�ł͂���܂���B
			const bool twoSameExts = (regs[a] & regs[b] & RC_BORDER_REG) != 0 && regs[a] == regs[b]; // ���_�͋��E�̒��_
			const bool twoInts = ((regs[c] | regs[d]) & RC_BORDER_REG) == 0; // 2�̓����O���Z�����A�����đ���
			const bool intsSameArea = (regs[c] >> 16) == (regs[d] >> 16);  // 2�̓����Z��������
			const bool noZeros = regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0; // �ǂ̗̈�����E�O�ł͂Ȃ�

			if (twoSameExts && twoInts && intsSameArea && noZeros)
			{
				isBorderVertex = true;
				break;
			}
		}

		return ch;
	}

	// �֊s�����
	void walkContour(int x, int y, int i,
		rcCompactHeightfield& chf,
		unsigned char* flags, rcIntArray& points)
	{
		// Choose the first non-connected edge
		// �ŏ��̔�ڑ��G�b�W��I�����܂�
		unsigned char dir{};
		while ((flags[i] & (1 << dir)) == 0)
			dir++;

		unsigned char startDir = dir;
		int starti = i;

		const unsigned char area = chf.areas[i];

		int iter{};
		while (++iter < 40000)
		{
			if (flags[i] & (1 << dir))
			{
				// Choose the edge corner
				// �G�b�W�R�[�i�[��I�����܂�
				bool isBorderVertex = false;
				bool isAreaBorder = false;
				int px = x;
				int py = getCornerHeight(x, y, i, dir, chf, isBorderVertex);
				int pz = y;

				switch (dir)
				{
					case 0: { pz++;			break; }
					case 1: { px++; pz++;	break; }
					case 2: { px++;			break; }
				}

				int r{};
				const rcCompactSpan& s = chf.spans[i];

				if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(dir);
					const int ay = y + rcGetDirOffsetY(dir);
					const int ai = (int)chf.cells[ax + ay * chf.width].index + rcGetCon(s, dir);
					r = (int)chf.spans[ai].reg;
					if (area != chf.areas[ai])
						isAreaBorder = true;
				}

				if (isBorderVertex) r |= RC_BORDER_VERTEX;

				if (isAreaBorder) r |= RC_AREA_BORDER;

				points.push(px);
				points.push(py);
				points.push(pz);
				points.push(r);

				flags[i] &= ~(1 << dir); // Remove visited edges // �K�₵���G�b�W���폜���܂�
				dir = (dir + 1) & 0x3;   // Rotate CW // ���v���ɉ�]
			}
			else
			{
				int ni = -1;
				const int nx = x + rcGetDirOffsetX(dir);
				const int ny = y + rcGetDirOffsetY(dir);
				const rcCompactSpan& s = chf.spans[i];

				if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
				{
					const rcCompactCell& nc = chf.cells[nx + ny * chf.width];
					ni = (int)nc.index + rcGetCon(s, dir);
				}

				// Should not happen. // �������Ȃ��͂��ł��B
				if (ni == -1) return;

				x = nx;
				y = ny;
				i = ni;
				dir = (dir + 3) & 0x3;	// Rotate CCW // �����v���ɉ�]
			}

			if (starti == i && startDir == dir)  break;
		}
	}

	float distancePtSeg(const int x, const int z,
		const int px, const int pz,
		const int qx, const int qz)
	{
		float pqx = (float)(qx - px);
		float pqz = (float)(qz - pz);
		float dx = (float)(x - px);
		float dz = (float)(z - pz);
		float d = pqx * pqx + pqz * pqz;
		float t = pqx * dx + pqz * dz;

		if (d > 0) t /= d;

		if (t < 0) t = 0;
		else
			if (t > 1) t = 1;

		dx = px + t * pqx - x;
		dz = pz + t * pqz - z;

		return dx * dx + dz * dz;
	}

	// �֊s��P��������
	void simplifyContour(rcIntArray& points, rcIntArray& simplified,
		const float maxError, const int maxEdgeLen, const int buildFlags)
	{
		// Add initial points.
		// �����_��ǉ����܂��B
		bool hasConnections = false;

		for (int i = 0; i < points.size(); i += 4)
		{
			if ((points[i + 3] & RC_CONTOUR_REG_MASK) != 0)
			{
				hasConnections = true;
				break;
			}
		}

		if (hasConnections)
		{
			// The contour has some portals to other regions.
			// �֊s�ɂ͑��̒n��ւ̂������̃|�[�^��������܂��B
			// Add a new point to every location where the region changes.
			// �̈悪�ύX����邷�ׂĂ̏ꏊ�ɐV�����|�C���g��ǉ����܂��B
			for (int i = 0, ni = points.size() / 4; i < ni; ++i)
			{
				int i_ni = (i + 1) % ni;
				const bool differentRegs
				{ (points[i * 4 + 3] & RC_CONTOUR_REG_MASK) != (points[i_ni * 4 + 3] & RC_CONTOUR_REG_MASK) };
				const bool areaBorders
				{ (points[i * 4 + 3] & RC_AREA_BORDER) != (points[i_ni * 4 + 3] & RC_AREA_BORDER) };

				if (differentRegs || areaBorders)
				{
					simplified.push(points[i * 4 + 0]);
					simplified.push(points[i * 4 + 1]);
					simplified.push(points[i * 4 + 2]);
					simplified.push(i);
				}
			}
		}

		if (simplified.size() == 0)
		{
			// If there is no connections at all, create some initial points for the simplification process.
			// �ڑ����܂������Ȃ��ꍇ�́A�P�����v���Z�X�̏����|�C���g���쐬���܂��B
			// Find lower-left and upper-right vertices of the contour.
			// �֊s�̍����ƉE��̒��_�������܂��B
			int llx = points[0];
			int lly = points[1];
			int llz = points[2];
			int lli{};
			int urx = points[0];
			int ury = points[1];
			int urz = points[2];
			int uri{};

			for (int i = 0; i < points.size(); i += 4)
			{
				int x = points[i + 0];
				int y = points[i + 1];
				int z = points[i + 2];

				if (x < llx || (x == llx && z < llz))
				{
					llx = x;
					lly = y;
					llz = z;
					lli = i / 4;
				}

				if (x > urx || (x == urx && z > urz))
				{
					urx = x;
					ury = y;
					urz = z;
					uri = i / 4;
				}
			}

			simplified.push(llx);
			simplified.push(lly);
			simplified.push(llz);
			simplified.push(lli);

			simplified.push(urx);
			simplified.push(ury);
			simplified.push(urz);
			simplified.push(uri);
		}

		// Add points until all raw points are within error tolerance to the simplified shape.
		// ���ׂĂ̐��̃|�C���g���P�������ꂽ�`��̋��e�덷���ɂȂ�܂Ń|�C���g��ǉ����܂��B
		const int pn = points.size() / 4;

		for (int i = 0; i < simplified.size() / 4; )
		{
			int ii = (i + 1) % (simplified.size() / 4);

			int ax = simplified[i * 4 + 0];
			int az = simplified[i * 4 + 2];
			int ai = simplified[i * 4 + 3];

			int bx = simplified[ii * 4 + 0];
			int bz = simplified[ii * 4 + 2];
			int bi = simplified[ii * 4 + 3];

			// Find maximum deviation from the segment.
			// �Z�O�����g����̍ő�΍��������܂��B
			float maxd = 0;
			int maxi = -1;
			int ci, cinc, endi;

			// Traverse the segment in lexilogical order so that the max deviation is calculated similarly when traversing
			// opposite segments.
			// �Z�O�����g����b���ɑ������āA���΂̃Z�O�����g�𑖍�����Ƃ��ɍő�΍������l�Ɍv�Z�����悤�ɂ��܂��B
			if (bx > ax || (bx == ax && bz > az))
			{
				cinc = 1;
				ci = (ai + cinc) % pn;
				endi = bi;
			}
			else
			{
				cinc = pn - 1;
				ci = (bi + cinc) % pn;
				endi = ai;
				rcSwap(ax, bx);
				rcSwap(az, bz);
			}

			// Tessellate only outer edges or edges between areas.
			// �O���̃G�b�W�܂��̓G���A�Ԃ̃G�b�W�݂̂��e�b�Z���[�V�������܂��B
			if ((points[ci * 4 + 3] & RC_CONTOUR_REG_MASK) == 0 || (points[ci * 4 + 3] & RC_AREA_BORDER))
			{
				while (ci != endi)
				{
					float d = distancePtSeg(points[ci * 4 + 0], points[ci * 4 + 2], ax, az, bx, bz);
					if (d > maxd)
					{
						maxd = d;
						maxi = ci;
					}
					ci = (ci + cinc) % pn;
				}
			}

			// If the max deviation is larger than accepted error, add new point, else continue to next segment.
			// �ő�΍������e�덷�����傫���ꍇ�A�V�����|�C���g��ǉ����邩�A���̃Z�O�����g�ɐi�݂܂��B
			if (maxi != -1 && maxd > (maxError * maxError))
			{
				// Add space for the new point.
				// �V�����|�C���g�ɃX�y�[�X��ǉ����܂��B
				simplified.resize(simplified.size() + 4);

				const int n = simplified.size() / 4;

				for (int j = n - 1; j > i; --j)
				{
					simplified[j * 4 + 0] = simplified[(j - 1) * 4 + 0];
					simplified[j * 4 + 1] = simplified[(j - 1) * 4 + 1];
					simplified[j * 4 + 2] = simplified[(j - 1) * 4 + 2];
					simplified[j * 4 + 3] = simplified[(j - 1) * 4 + 3];
				}

				// Add the point. // �|�C���g��ǉ����܂��B
				simplified[(i + 1) * 4 + 0] = points[maxi * 4 + 0];
				simplified[(i + 1) * 4 + 1] = points[maxi * 4 + 1];
				simplified[(i + 1) * 4 + 2] = points[maxi * 4 + 2];
				simplified[(i + 1) * 4 + 3] = maxi;
			}
			else
			{
				++i;
			}
		}

		// Split too long edges.
		// ��������G�b�W�𕪊����܂��B
		if (maxEdgeLen > 0 && (buildFlags & (RC_CONTOUR_TESS_WALL_EDGES | RC_CONTOUR_TESS_AREA_EDGES)) != 0)
		{
			for (int i = 0; i < simplified.size() / 4; )
			{
				const int ii = (i + 1) % (simplified.size() / 4);

				const int ax = simplified[i * 4 + 0];
				const int az = simplified[i * 4 + 2];
				const int ai = simplified[i * 4 + 3];

				const int bx = simplified[ii * 4 + 0];
				const int bz = simplified[ii * 4 + 2];
				const int bi = simplified[ii * 4 + 3];

				// Find maximum deviation from the segment.
				// �Z�O�����g����̍ő�΍��������܂��B
				int maxi = -1;
				int ci = (ai + 1) % pn;

				// Tessellate only outer edges or edges between areas.
				// �O���̃G�b�W�܂��̓G���A�Ԃ̃G�b�W�݂̂��e�b�Z���[�V�������܂��B
				bool tess = false;

				// Wall edges. // �ǂ̒[�B
				if ((buildFlags & RC_CONTOUR_TESS_WALL_EDGES) && (points[ci * 4 + 3] & RC_CONTOUR_REG_MASK) == 0)
					tess = true;

				// Edges between areas. //�G���A�Ԃ̃G�b�W�B
				if ((buildFlags & RC_CONTOUR_TESS_AREA_EDGES) && (points[ci * 4 + 3] & RC_AREA_BORDER))
					tess = true;

				if (tess)
				{
					int dx = bx - ax;
					int dz = bz - az;
					if (dx * dx + dz * dz > maxEdgeLen* maxEdgeLen)
					{
						// Round based on the segments in lexilogical order so that the
						// max tesselation is consistent regardles in which direction segments are traversed.
						// �Z�O�����g���ǂ̕����Ɉړ����邩�Ɋ֌W�Ȃ��A�ő�e�b�Z���[�V��������т���悤�ɁA
						// �������ŃZ�O�����g�Ɋ�Â��Ċۂ߂܂��B
						const int n = bi < ai ? (bi + pn - ai) : (bi - ai);

						if (n > 1)
						{
							if (bx > ax || (bx == ax && bz > az))
								maxi = (ai + n / 2) % pn;
							else
								maxi = (ai + (n + 1) / 2) % pn;
						}
					}
				}

				// If the max deviation is larger than accepted error, add new point, else continue to next segment.
				// �ő�΍������e�덷�����傫���ꍇ�A�V�����|�C���g��ǉ����邩�A���̃Z�O�����g�ɐi�݂܂��B
				if (maxi != -1)
				{
					// Add space for the new point.
					// �V�����|�C���g�ɃX�y�[�X��ǉ����܂��B
					simplified.resize(simplified.size() + 4);
					const int n = simplified.size() / 4;

					for (int j = n - 1; j > i; --j)
					{
						simplified[j * 4 + 0] = simplified[(j - 1) * 4 + 0];
						simplified[j * 4 + 1] = simplified[(j - 1) * 4 + 1];
						simplified[j * 4 + 2] = simplified[(j - 1) * 4 + 2];
						simplified[j * 4 + 3] = simplified[(j - 1) * 4 + 3];
					}

					// Add the point. // �|�C���g��ǉ����܂��B
					simplified[(i + 1) * 4 + 0] = points[maxi * 4 + 0];
					simplified[(i + 1) * 4 + 1] = points[maxi * 4 + 1];
					simplified[(i + 1) * 4 + 2] = points[maxi * 4 + 2];
					simplified[(i + 1) * 4 + 3] = maxi;
				}
				else
				{
					++i;
				}
			}
		}

		for (int i = 0; i < simplified.size() / 4; ++i)
		{
			// The edge vertex flag is take from the current raw point, and the neighbour region is take from the next raw point.
			// �G�b�W���_�t���O�͌��݂̐��̃|�C���g����擾����A�אڗ̈�͎��̐��̃|�C���g����擾����܂��B
			const int ai = (simplified[i * 4 + 3] + 1) % pn;
			const int bi = simplified[i * 4 + 3];

			simplified[i * 4 + 3] =
				(points[ai * 4 + 3] & (RC_CONTOUR_REG_MASK | RC_AREA_BORDER)) | (points[bi * 4 + 3] & RC_BORDER_VERTEX);
		}
	}

	int calcAreaOfPolygon2D(const int* verts, const int nverts)
	{
		int area = 0;
		for (int i = 0, j = nverts - 1; i < nverts; j = i++)
		{
			const int* vi = &verts[i * 4];
			const int* vj = &verts[j * 4];
			area += vi[0] * vj[2] - vj[0] * vi[2];
		}
		return (area + 1) / 2;
	}

	// TODO: these are the same as in RecastMesh.cpp, consider using the same. // ������RecastMesh.cpp�Ɠ����ł��B�������̂��g�p���邱�Ƃ��������Ă��������B
	// Last time I checked the if version got compiled using cmov, which was a lot faster than module (with idiv).
	// �O��Acmov���g�p���ăo�[�W�������R���p�C�����ꂽ���ǂ������m�F���܂����B
	// ����́A���W���[���iidiv���g�p�j�����͂邩�ɍ����ł����B
	inline constexpr int prev(const int i, const int n) { return i - 1 >= 0 ? i - 1 : n - 1; }
	inline constexpr int next(const int i, const int n) { return i + 1 < n ? i + 1 : 0; }

	inline constexpr int area2(const int* a, const int* b, const int* c)
	{
		return (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
	}

	//	Exclusive or: true iff exactly one argument is true.
	//	The arguments are negated to ensure that they are 0/1 values.
	//  Then the bitwise Xor operator may apply. (This idea is due to Michael Baldwin.)
	//	�r���I�܂��́F���m��1�̈�����true�̏ꍇ��true�B
	//	�����́A�l��0/1�ł��邱�Ƃ�ۏ؂��邽�߂ɔے肳��܂��B
	//	���ɁA�r�b�g�P�ʂ�Xor���Z�q���K�p����܂��B�i���̃A�C�f�A�́uMichael Baldwin�v�ɂ����̂ł��B�j
	inline constexpr bool xorb(const bool x, const bool y)
	{
		return !x ^ !y;
	}

	// Returns true iff c is strictly to the left of the directed line through a to b.
	// c��������a����b�܂ł̗L�����̍����ɂ���ꍇ�Atrue��Ԃ��܂��B
	inline constexpr bool left(const int* a, const int* b, const int* c)
	{
		return area2(a, b, c) < 0;
	}

	inline constexpr bool leftOn(const int* a, const int* b, const int* c)
	{
		return area2(a, b, c) <= 0;
	}

	inline constexpr bool collinear(const int* a, const int* b, const int* c)
	{
		return area2(a, b, c) == 0;
	}

	//	Returns true iff ab properly intersects cd: they share a point interior to both segments.
	//  The properness of the intersection is ensured by using strict leftness.
	//	ab��cd�ƓK�؂Ɍ�������ꍇ��true��Ԃ��܂��B�����͗����̃Z�O�����g�̓����|�C���g�����L���܂��B
	//	�����_�̓K�ؐ��́A�����ȍ����g�p���邱�Ƃɂ��ۏ؂���܂��B
	inline constexpr bool intersectProp(const int* a, const int* b, const int* c, const int* d)
	{
		// Eliminate improper cases.
		// �s�K�؂ȃP�[�X��r�����܂��B
		if (collinear(a, b, c) || collinear(a, b, d) || collinear(c, d, a) || collinear(c, d, b))
			return false;

		return xorb(left(a, b, c), left(a, b, d)) && xorb(left(c, d, a), left(c, d, b));
	}

	// Returns T iff (a,b,c) are collinear and point c lies on the closed segement ab.
	//�ia�Ab�Ac�j�����꒼����ɂ���A�|�C���gc�������Z�O�����gab�ɂ���ꍇ�AT��Ԃ��܂��B
	inline constexpr bool between(const int* a, const int* b, const int* c)
	{
		if (!collinear(a, b, c))
			return false;

		// If ab not vertical, check betweenness on x; else on y.
		// ab�������łȂ��ꍇ�Ax�̒��Ԃ��`�F�b�N���܂�; ����ȊO�̏ꍇ��y�B
		if (a[0] != b[0])
			return	((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
		else
			return	((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
	}

	// Returns true iff segments ab and cd intersect, properly or improperly.
	// �Z�O�����gab��cd���K�؂ɂ܂��͕s�K�؂Ɍ�������ꍇ�ɂ̂݁A�^��Ԃ��܂��B
	inline constexpr bool intersect(const int* a, const int* b, const int* c, const int* d)
	{
		if (intersectProp(a, b, c, d))
			return true;
		else if (between(a, b, c) || between(a, b, d) ||
			between(c, d, a) || between(c, d, b))
			return true;
		else
			return false;
	}

	// �������l�ł��邱�Ƃ��m���߂�
	inline constexpr bool vequal(const int* a, const int* b)
	{
		return a[0] == b[0] && a[2] == b[2];
	}

	bool intersectSegCountour(const int* d0, const int* d1, int i, int n, const int* verts)
	{
		// For each edge (k,k+1) of P
		for (int k = 0; k < n; k++)
		{
			int k1 = next(k, n);
			// Skip edges incident to i.
			if (i == k || i == k1)
				continue;
			const int* p0 = &verts[k * 4];
			const int* p1 = &verts[k1 * 4];
			if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1))
				continue;

			if (intersect(d0, d1, p0, p1))
				return true;
		}
		return false;
	}

	inline constexpr bool	inCone(int i, int n, const int* verts, const int* pj)
	{
		const int* pi = &verts[i * 4];
		const int* pi1 = &verts[next(i, n) * 4];
		const int* pin1 = &verts[prev(i, n) * 4];

		// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
		if (leftOn(pin1, pi, pi1))
			return left(pi, pj, pin1) && left(pj, pi, pi1);

		// Assume (i-1,i,i+1) not collinear.
		// else P[i] is reflex.
		return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
	}

	// �����Z�O�����g�̍폜
	void removeDegenerateSegments(rcIntArray& simplified)
	{
		// Remove adjacent vertices which are equal on xz-plane, or else the triangulator will get confused.
		// xz���ʂœ������אڂ��钸�_���폜���܂��B�������Ȃ��ƁA�O�p�`�����킪�������܂��B
		int npts = simplified.size() / 4;

		for (int i = 0; i < npts; ++i)
		{
			int ni = next(i, npts);

			if (vequal(&simplified[i * 4], &simplified[ni * 4]))
			{
				// Degenerate segment, remove.
				// �Z�O�����g���k�ނ��A�폜���܂��B
				for (int j = i; j < simplified.size() / 4 - 1; ++j)
				{
					simplified[j * 4 + 0] = simplified[(j + 1) * 4 + 0];
					simplified[j * 4 + 1] = simplified[(j + 1) * 4 + 1];
					simplified[j * 4 + 2] = simplified[(j + 1) * 4 + 2];
					simplified[j * 4 + 3] = simplified[(j + 1) * 4 + 3];
				}

				simplified.resize(simplified.size() - 4);
				npts--;
			}
		}
	}

	bool mergeContours(rcContour& ca, rcContour& cb, int ia, int ib)
	{
		const int maxVerts = ca.nverts + cb.nverts + 2;
		int* verts = (int*)rcAlloc(sizeof(int) * maxVerts * 4, RC_ALLOC_PERM);
		if (!verts)
			return false;

		int nv = 0;

		// Copy contour A.
		for (int i = 0; i <= ca.nverts; ++i)
		{
			int* dst = &verts[nv * 4];
			const int* src = &ca.verts[((ia + i) % ca.nverts) * 4];
			dst[0] = src[0];
			dst[1] = src[1];
			dst[2] = src[2];
			dst[3] = src[3];
			nv++;
		}

		// Copy contour B
		for (int i = 0; i <= cb.nverts; ++i)
		{
			int* dst = &verts[nv * 4];
			const int* src = &cb.verts[((ib + i) % cb.nverts) * 4];
			dst[0] = src[0];
			dst[1] = src[1];
			dst[2] = src[2];
			dst[3] = src[3];
			nv++;
		}

		rcFree(ca.verts);
		ca.verts = verts;
		ca.nverts = nv;

		rcFree(cb.verts);
		cb.verts = 0;
		cb.nverts = 0;

		return true;
	}

	struct rcContourHole
	{
		rcContour* contour;
		int minx, minz, leftmost;
	};

	struct rcContourRegion
	{
		rcContour* outline;
		rcContourHole* holes;
		int nholes;
	};

	struct rcPotentialDiagonal
	{
		int vert;
		int dist;
	};

	// Finds the lowest leftmost vertex of a contour.
	// �֊s�̈�ԍ����̒��_�������܂��B
	void findLeftMostVertex(rcContour* contour, int* minx, int* minz, int* leftmost)
	{
		*minx = contour->verts[0];
		*minz = contour->verts[2];
		*leftmost = 0;
		for (int i = 1; i < contour->nverts; i++)
		{
			const int x = contour->verts[i * 4 + 0];
			const int z = contour->verts[i * 4 + 2];
			if (x < *minx || (x == *minx && z < *minz))
			{
				*minx = x;
				*minz = z;
				*leftmost = i;
			}
		}
	}

	inline int compareHoles(const void* va, const void* vb)
	{
		const rcContourHole* a = (const rcContourHole*)va;
		const rcContourHole* b = (const rcContourHole*)vb;

		if (a->minx == b->minx)
		{
			return (a->minz > b->minz) - (a->minz < b->minz);
		}
		else
		{
			return (a->minx > b->minx) - (a->minx < b->minx);
		}
	}

	inline int compareDiagDist(const void* va, const void* vb)
	{
		const rcPotentialDiagonal* a = (const rcPotentialDiagonal*)va;
		const rcPotentialDiagonal* b = (const rcPotentialDiagonal*)vb;

		return (a->dist > b->dist) - (a->dist < b->dist);
	}

	void mergeRegionHoles(rcContext* ctx, rcContourRegion& region)
	{
		// Sort holes from left to right.
		// ����������E�ɕ��בւ��܂��B
		for (int i = 0; i < region.nholes; i++)
			findLeftMostVertex(region.holes[i].contour, &region.holes[i].minx, &region.holes[i].minz, &region.holes[i].leftmost);

		qsort(region.holes, region.nholes, sizeof(rcContourHole), compareHoles);

		int maxVerts = region.outline->nverts;
		for (int i = 0; i < region.nholes; i++)
			maxVerts += region.holes[i].contour->nverts;

		rcScopedDelete<rcPotentialDiagonal> diags((rcPotentialDiagonal*)rcAlloc(sizeof(rcPotentialDiagonal) * maxVerts, RC_ALLOC_TEMP));

		if (!diags)
		{
			// �f�f�̊��蓖�ĂɎ��s���܂����B
			ctx->log(RC_LOG_WARNING, "mergeRegionHoles: Failed to allocated diags %d.", maxVerts);
			return;
		}

		rcContour* outline = region.outline;

		// Merge holes into the outline one by one.
		// ����1���A�E�g���C���Ɍ������܂��B
		for (int i = 0; i < region.nholes; i++)
		{
			rcContour* hole = region.holes[i].contour;

			int index = -1;
			int bestVertex = region.holes[i].leftmost;
			for (int iter = 0; iter < hole->nverts; iter++)
			{
				// Find potential diagonals.
				// ���ݓI�ȑΊp���������܂��B
				// The 'best' vertex must be in the cone described by 3 cosequtive vertices of the outline.
				//�u�œK�ȁv���_�́A�A�E�g���C����3�̘A���������_�ɂ���ċL�q�����~�����ɂȂ���΂Ȃ�܂���B
				// ..o j-1
				//   |
				//   |   * best
				//   |
				// j o-----o j+1
				//         :
				int ndiags = 0;
				const int* corner = &hole->verts[bestVertex * 4];
				for (int j = 0; j < outline->nverts; j++)
				{
					if (inCone(j, outline->nverts, outline->verts, corner))
					{
						int dx = outline->verts[j * 4 + 0] - corner[0];
						int dz = outline->verts[j * 4 + 2] - corner[2];
						diags[ndiags].vert = j;
						diags[ndiags].dist = dx * dx + dz * dz;
						ndiags++;
					}
				}

				// Sort potential diagonals by distance, we want to make the connection as short as possible.
				// ���ݓI�ȑΊp���������ŕ��בւ��܂��B�ڑ����ł��邾���Z���������ł��B
				qsort(diags, ndiags, sizeof(rcPotentialDiagonal), compareDiagDist);

				// Find a diagonal that is not intersecting the outline not the remaining holes.
				// �c��̌��ł͂Ȃ��A�A�E�g���C���ƌ������Ȃ��Ίp���������܂��B
				index = -1;
				for (int j = 0; j < ndiags; j++)
				{
					const int* pt = &outline->verts[diags[j].vert * 4];
					bool intersect = intersectSegCountour(pt, corner, diags[i].vert, outline->nverts, outline->verts);
					for (int k = i; k < region.nholes && !intersect; k++)
						intersect |= intersectSegCountour(pt, corner, -1, region.holes[k].contour->nverts, region.holes[k].contour->verts);
					if (!intersect)
					{
						index = diags[j].vert;
						break;
					}
				}

				// If found non-intersecting diagonal, stop looking.
				// �������Ȃ��Ίp�������������ꍇ�A�������~���܂��B
				if (index != -1) break;

				// All the potential diagonals for the current vertex were intersecting, try next vertex.
				// ���݂̒��_�̂��ׂĂ̐��ݓI�ȑΊp�����������Ă����̂ŁA���̒��_�������Ă��������B
				bestVertex = (bestVertex + 1) % hole->nverts;
			}

			if (index == -1)
			{
				// �}�[�W�|�C���g��������܂���ł����B
				ctx->log(RC_LOG_WARNING, "mergeHoles: Failed to find merge points for %p and %p.", region.outline, hole);
				continue;
			}
			if (!mergeContours(*region.outline, *hole, index, bestVertex))
			{
				// �֊s�̃}�[�W�Ɏ��s���܂����B
				ctx->log(RC_LOG_WARNING, "mergeHoles: Failed to merge contours %p and %p.", region.outline, hole);
				continue;
			}
		}
	}
}

/// @par
///
/// The raw contours will match the region outlines exactly. The @p maxError and @p maxEdgeLen
/// parameters control how closely the simplified contours will match the raw contours.
/// ���̗֊s�́A�̈�̗֊s�Ɛ��m�Ɉ�v���܂��B @p maxError�����@p maxEdgeLen�p�����[�^�[�́A
/// �P�������ꂽ�֊s�����̗֊s�Ƃǂꂾ����v���邩�𐧌䂵�܂��B
///
/// Simplified contours are generated such that the vertices for portals between areas match up.
/// (They are considered mandatory vertices.)
/// �G���A�Ԃ̃|�[�^���̒��_����v����悤�ɁA�ȗ������ꂽ�֊s����������܂��B�i�����͕K�{�̒��_�ƌ��Ȃ���܂��B�j
///
/// Setting @p maxEdgeLength to zero will disabled the edge length feature.
/// @p maxEdgeLength���[���ɐݒ肷��ƁA�G�b�W���@�\�������ɂȂ�܂��B
///
/// See the #rcConfig documentation for more information on the configuration parameters.
/// �\���p�����[�^�[�̏ڍׂɂ��ẮA��rcConfig�̃h�L�������g���Q�Ƃ��Ă��������B
///
/// @see rcAllocContourSet, rcCompactHeightfield, rcContourSet, rcConfig
bool rcBuildContours(rcContext* ctx, rcCompactHeightfield& chf,
	const float maxError, const int maxEdgeLen,
	rcContourSet& cset, const int buildFlags)
{
	rcAssert(ctx);

	const int w = chf.width;
	const int h = chf.height;
	const int borderSize = chf.borderSize;

	rcScopedTimer timer(ctx, RC_TIMER_BUILD_CONTOURS);

	rcVcopy(cset.bmin, chf.bmin.data());
	rcVcopy(cset.bmax, chf.bmax.data());
	if (borderSize > 0)
	{
		// If the heightfield was build with bordersize, remove the offset.
		// �n�`��bordersize�ō\�z���ꂽ�ꍇ�A�I�t�Z�b�g���폜���܂��B
		const float pad = borderSize * chf.cs;
		cset.bmin[0] += pad;
		cset.bmin[2] += pad;
		cset.bmax[0] -= pad;
		cset.bmax[2] -= pad;
	}
	cset.cs = chf.cs;
	cset.ch = chf.ch;
	cset.width = chf.width - chf.borderSize * 2;
	cset.height = chf.height - chf.borderSize * 2;
	cset.borderSize = chf.borderSize;
	cset.maxError = maxError;

	int maxContours = rcMax((int)chf.maxRegions, 8);
	cset.conts = (rcContour*)rcAlloc(sizeof(rcContour) * maxContours, RC_ALLOC_PERM);
	if (!cset.conts)
		return false;
	cset.nconts = 0;

	rcScopedDelete<unsigned char> flags((unsigned char*)rcAlloc(sizeof(unsigned char) * chf.spanCount, RC_ALLOC_TEMP));
	if (!flags)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'flags' (%d).", chf.spanCount); // �������[�s���uflags�v
		return false;
	}

	ctx->startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

	// Mark boundaries.
	// ���E���}�[�N���܂��B
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				unsigned char res = 0;
				const rcCompactSpan& s = chf.spans[i];

				if (!chf.spans[i].reg || (chf.spans[i].reg & RC_BORDER_REG))
				{
					flags[i] = 0;
					continue;
				}

				for (int dir = 0; dir < 4; ++dir)
				{
					unsigned short r{};
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax + ay * w].index + rcGetCon(s, dir);
						r = chf.spans[ai].reg;
					}
					if (r == chf.spans[i].reg)
						res |= (1 << dir);
				}

				flags[i] = res ^ 0xf; // Inverse, mark non connected edges. // //�t�ɁA�ڑ�����Ă��Ȃ��G�b�W���}�[�N���܂��B
			}
		}
	}

	ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

	rcIntArray verts(256);
	rcIntArray simplified(64);

	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x + y * w];
			for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
			{
				if (flags[i] == 0 || flags[i] == 0xf)
				{
					flags[i] = 0;
					continue;
				}
				const unsigned short reg = chf.spans[i].reg;
				if (!reg || (reg & RC_BORDER_REG))
					continue;
				const unsigned char area = chf.areas[i];

				verts.resize(0);
				simplified.resize(0);

				ctx->startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
				walkContour(x, y, i, chf, flags, verts);
				ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);

				ctx->startTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
				simplifyContour(verts, simplified, maxError, maxEdgeLen, buildFlags);
				removeDegenerateSegments(simplified);
				ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);

				// Store region->contour remap info.
				// �uregion-> contour�v���}�b�v����ۑ����܂��B
				// Create contour. // �֊s���쐬���܂��B
				if (simplified.size() / 4 >= 3)
				{
					if (cset.nconts >= maxContours)
					{
						// Allocate more contours.
						// ��葽���̗֊s�����蓖�Ă܂��B
						// This happens when a region has holes.
						// ����́A�̈�Ɍ�������ꍇ�ɔ������܂��B
						const int oldMax = maxContours;
						maxContours *= 2;
						rcContour* newConts = (rcContour*)rcAlloc(sizeof(rcContour) * maxContours, RC_ALLOC_PERM);

						for (int j = 0; j < cset.nconts; ++j)
						{
							newConts[j] = cset.conts[j];
							// Reset source pointers to prevent data deletion.
							// �f�[�^�̍폜��h�����߂Ƀ\�[�X�|�C���^�[�����Z�b�g���܂��B
							cset.conts[j].verts = 0;
							cset.conts[j].rverts = 0;
						}
						rcFree(cset.conts);
						cset.conts = newConts;

						// �x���F�ő�֊s���g�����Ă��܂��B
						ctx->log(RC_LOG_WARNING, "rcBuildContours: Expanding max contours from %d to %d.", oldMax, maxContours);
					}

					rcContour* cont = &cset.conts[cset.nconts++];

					cont->nverts = simplified.size() / 4;
					cont->verts = (int*)rcAlloc(sizeof(int) * cont->nverts * 4, RC_ALLOC_PERM);

					if (!cont->verts)
					{
						// �������[�s���uverts�v
						ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'verts' (%d).", cont->nverts);
						return false;
					}

					memcpy(cont->verts, &simplified[0], sizeof(int) * cont->nverts * 4);

					if (borderSize > 0)
					{
						// If the heightfield was build with bordersize, remove the offset.
						// �n�`��bordersize�ō\�z���ꂽ�ꍇ�A�I�t�Z�b�g���폜���܂��B
						for (int j = 0; j < cont->nverts; ++j)
						{
							int* v = &cont->verts[j * 4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}

					cont->nrverts = verts.size() / 4;
					cont->rverts = (int*)rcAlloc(sizeof(int) * cont->nrverts * 4, RC_ALLOC_PERM);

					if (!cont->rverts)
					{
						// �������[�s���urverts�v
						ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'rverts' (%d).", cont->nrverts);
						return false;
					}

					memcpy(cont->rverts, &verts[0], sizeof(int) * cont->nrverts * 4);

					if (borderSize > 0)
					{
						// If the heightfield was build with bordersize, remove the offset.
						// �n�`��bordersize�ō\�z���ꂽ�ꍇ�A�I�t�Z�b�g���폜���܂��B
						for (int j = 0; j < cont->nrverts; ++j)
						{
							int* v = &cont->rverts[j * 4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}

					cont->reg = reg;
					cont->area = area;
				}
			}
		}
	}

	// Merge holes if needed.
	// �K�v�ɉ����Č����������܂��B
	if (cset.nconts > 0)
	{
		// Calculate winding of all polygons.
		// ���ׂẴ|���S���̊������v�Z���܂��B
		rcScopedDelete<char> winding((char*)rcAlloc(sizeof(char) * cset.nconts, RC_ALLOC_TEMP));

		if (!winding)
		{
			ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'hole' (%d).", cset.nconts); // �������[�s���uhole�v
			return false;
		}

		int nholes = 0;

		for (int i = 0; i < cset.nconts; ++i)
		{
			rcContour& cont = cset.conts[i];
			// If the contour is wound backwards, it is a hole.
			// �֊s������Ɋ�����Ă���ꍇ�A����͌��ł��B
			winding[i] = calcAreaOfPolygon2D(cont.verts, cont.nverts) < 0 ? -1 : 1;

			if (winding[i] < 0)
				nholes++;
		}

		if (nholes > 0)
		{
			// Collect outline contour and holes contours per region.
			// �̈悲�Ƃɗ֊s�ƌ��̗֊s�����W���܂��B
			// We assume that there is one outline and multiple holes.
			// 1�̃A�E�g���C���ƕ����̌�������Ɖ��肵�܂��B
			const int nregions = chf.maxRegions + 1;
			rcScopedDelete<rcContourRegion> regions
			{ (rcContourRegion*)rcAlloc(sizeof(rcContourRegion) * nregions, RC_ALLOC_TEMP) };

			if (!regions)
			{
				ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'regions' (%d).", nregions); // �������[�s���uregions�v
				return false;
			}

			memset(regions, 0, sizeof(rcContourRegion) * nregions);

			rcScopedDelete<rcContourHole> holes((rcContourHole*)rcAlloc(sizeof(rcContourHole) * cset.nconts, RC_ALLOC_TEMP));

			if (!holes)
			{
				ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'holes' (%d).", cset.nconts); // �������[�s���uholes�v
				return false;
			}

			memset(holes, 0, sizeof(rcContourHole) * cset.nconts);

			for (int i = 0; i < cset.nconts; ++i)
			{
				rcContour& cont = cset.conts[i];

				// Positively would contours are outlines, negative holes.
				// ���̗֊s�͗֊s�A���͌��ɂȂ�܂��B
				if (winding[i] > 0)
				{
					if (regions[cont.reg].outline)
						// �̈�̕����̃A�E�g���C���B
						ctx->log(RC_LOG_ERROR, "rcBuildContours: Multiple outlines for region %d.", cont.reg);

					regions[cont.reg].outline = &cont;
				}
				else
				{
					regions[cont.reg].nholes++;
				}
			}

			for (int i = 0, index = 0; i < nregions; i++)
			{
				if (regions[i].nholes > 0)
				{
					regions[i].holes = &holes[index];
					index += regions[i].nholes;
					regions[i].nholes = 0;
				}
			}

			for (int i = 0; i < cset.nconts; ++i)
			{
				rcContour& cont = cset.conts[i];
				rcContourRegion& reg = regions[cont.reg];
				if (winding[i] < 0)
					reg.holes[reg.nholes++].contour = &cont;
			}

			// Finally merge each regions holes into the outline.
			// �Ō�ɁA�e�̈�̌����A�E�g���C���Ƀ}�[�W���܂��B
			for (int i = 0; i < nregions; i++)
			{
				rcContourRegion& reg = regions[i];

				if (!reg.nholes) continue;

				if (reg.outline)
				{
					mergeRegionHoles(ctx, reg);
				}
				else
				{
					// The region does not have an outline.
					// �̈�ɂ̓A�E�g���C��������܂���B
					// This can happen if the contour becaomes selfoverlapping because of too aggressive simplification settings.
					// ����́A�P�����̐ݒ肪�������邽�߂ɗ֊s�����ȏd�Ȃ荇���ꍇ�ɔ�������\��������܂��B
					ctx->log(RC_LOG_ERROR,
						"rcBuildContours: Bad outline for region %d, contour simplification is likely too aggressive.", i);
					/// �̈�̗֊s���s�K�؂ł��B�֊s�̒P�����͂��܂�ɂ��ϋɓI�ł��B
				}
			}
		}
	}

	return true;
}
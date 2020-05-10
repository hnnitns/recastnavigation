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

#include "MeshLoaderObj.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>

rcMeshLoaderObj::rcMeshLoaderObj() :
	m_scale(1.f), m_vertCount{}, m_triCount{}
{}

void rcMeshLoaderObj::addVertex(float x, float y, float z, int& cap)
{
	if (m_vertCount + 1 > cap)
	{
		cap = !cap ? 8 : cap * 2;

		//float* nv = new float[cap * 3];
		std::vector<float> nv(cap * 3, 0.f);

		if (m_vertCount)
			//memcpy(nv, m_verts, m_vertCount * 3 * sizeof(float));
			for (int i = 0; i < m_vertCount * 3; i++)
			{
				nv[i] = m_verts[i];
			}

		//delete[] m_verts;

		m_verts = nv;
	}
	float* dst = &m_verts[m_vertCount * 3];
	*dst++ = x * m_scale;
	*dst++ = y * m_scale;
	*dst++ = z * m_scale;
	m_vertCount++;
}

void rcMeshLoaderObj::addTriangle(int a, int b, int c, int& cap)
{
	if (m_triCount + 1 > cap)
	{
		cap = !cap ? 8 : cap * 2;

		//int* nv = new int[cap * 3];
		std::vector<int> nv(cap * 3, 0);

		if (m_triCount)
			//memcpy(nv, m_tris, m_triCount * 3 * sizeof(int));
			for (int i = 0; i < m_triCount * 3; i++)
			{
				nv[i] = m_tris[i];
			}

		//delete[] m_tris;
		m_tris = nv;
	}

	int* dst = &m_tris[m_triCount * 3];
	*dst++ = a;
	*dst++ = b;
	*dst++ = c;
	m_triCount++;
}

namespace
{
	char* parseRow(char* buf, char* bufEnd, char* row, int len)
	{
		bool start = true;
		bool done = false;
		int n = 0;
		while (!done && buf < bufEnd)
		{
			char c = *buf;
			buf++;
			// multirow
			switch (c)
			{
				case '\\':
					break;
				case '\n':
					if (start) break;
					done = true;
					break;
				case '\r':
					break;
				case '\t':
				case ' ':
					if (start) break;
				default:
					start = false;
					row[n++] = c;
					if (n >= len - 1)
						done = true;
					break;
			}
		}
		row[n] = '\0';
		return buf;
	}

	int parseFace(char* row, int* data, int n, int vcnt)
	{
		int j{};

		while (*row != '\0')
		{
			// Skip initial white space
			// 最初の空白をスキップします
			while (*row != '\0' && (*row == ' ' || *row == '\t'))
				row++;

			char* s = row;

			// Find vertex delimiter and terminated the string there for conversion.
			// 頂点区切り文字を見つけ、変換のためにそこで文字列を終了しました。
			while (*row != '\0' && *row != ' ' && *row != '\t')
			{
				if (*row == '/') *row = '\0';
				row++;
			}

			if (*s == '\0')
				continue;

			int vi = atoi(s);
			data[j++] = vi < 0 ? vi + vcnt : vi - 1;

			if (j >= n) return j;
		}
		return j;
	}
}

bool rcMeshLoaderObj::load(const std::string& filename)
{
	char* buf{ nullptr };
	FILE* fp{ nullptr };

	// 開けない
	if (fopen_s(&fp, filename.c_str(), "rb") != 0) return false;

	// nullの可能性を排除
	if (!fp)	return false;

	if (fseek(fp, 0, SEEK_END) != 0)
	{
		fclose(fp);
		return false;
	}

	long bufSize = ftell(fp);
	if (bufSize < 0)
	{
		fclose(fp);
		return false;
	}

	if (fseek(fp, 0, SEEK_SET) != 0)
	{
		fclose(fp);
		return false;
	}

	buf = new char[bufSize];
	if (!buf)
	{
		fclose(fp);
		return false;
	}

	size_t readLen = fread(buf, bufSize, 1, fp);
	fclose(fp);

	if (readLen != 1)
	{
		delete[] buf;
		return false;
	}

	char* src = buf;
	char* srcEnd = buf + bufSize;
	char row[512]{};
	int face[32]{};
	float x{}, y{}, z{};
	int nv{};
	int vcap{};
	int tcap{};

	while (src < srcEnd)
	{
		// Parse one row
		// 1行を解析します
		row[0] = '\0';
		src = parseRow(src, srcEnd, row, sizeof(row) / sizeof(char));

		// Skip comments
		// コメントをスキップ
		if (row[0] == '#') continue;

		// 頂点の追加
		if (row[0] == 'v' && row[1] != 'n' && row[1] != 't')
		{
			// Vertex pos
			[[maybe_unused]]int temp{ sscanf_s(row + 1, "%f %f %f", &x, &y, &z) };
			addVertex(x, y, z, vcap);
		}

		// 三角形の追加
		if (row[0] == 'f')
		{
			// 法線
			nv = parseFace(row + 1, face, 32, m_vertCount);

			for (int i = 2; i < nv; ++i)
			{
				const int a = face[0];
				const int b = face[i - 1];
				const int c = face[i];

				if ((a < 0 || a >= m_vertCount) || (b < 0 || b >= m_vertCount) || (c < 0 || c >= m_vertCount))
					continue;

				addTriangle(a, b, c, tcap);
			}
		}
	}

	delete[] buf;

	// Calculate normals.
	// 法線を計算します。
	m_normals.resize(m_triCount * 3);

	for (int i = 0; i < m_triCount * 3; i += 3)
	{
		const float* v0 = &m_verts[m_tris[i] * 3];
		const float* v1 = &m_verts[m_tris[i + 1] * 3];
		const float* v2 = &m_verts[m_tris[i + 2] * 3];
		float e0[3], e1[3];

		for (int j = 0; j < 3; ++j)
		{
			e0[j] = v1[j] - v0[j];
			e1[j] = v2[j] - v0[j];
		}

		float* n = &m_normals[i];

		n[0] = e0[1] * e1[2] - e0[2] * e1[1];
		n[1] = e0[2] * e1[0] - e0[0] * e1[2];
		n[2] = e0[0] * e1[1] - e0[1] * e1[0];

		float d = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);

		if (d > 0)
		{
			d = 1.f / d;
			n[0] *= d;
			n[1] *= d;
			n[2] *= d;
		}
	}

	m_filename = filename;
	return true;
}
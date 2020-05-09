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

#ifndef MESHLOADER_OBJ
#define MESHLOADER_OBJ

#include <string>
#include <vector>

class rcMeshLoaderObj
{
public:
	rcMeshLoaderObj();
	~rcMeshLoaderObj() = default;

	rcMeshLoaderObj(rcMeshLoaderObj&& _rt) noexcept
	{
		using std::move;

		m_filename = move(_rt.m_filename);
		m_scale = (_rt.m_scale);
		m_verts = move(_rt.m_verts);
		m_tris = move(_rt.m_tris);
		m_normals = move(_rt.m_normals);
		m_vertCount = (_rt.m_vertCount);
		m_triCount = (_rt.m_triCount);
	}
	rcMeshLoaderObj& operator=(rcMeshLoaderObj&& _rt) noexcept
	{
		using std::move;

		if (this != &_rt)
		{
			m_filename = move(_rt.m_filename);
			m_scale = (_rt.m_scale);
			m_verts = move(_rt.m_verts);
			m_tris = move(_rt.m_tris);
			m_normals = move(_rt.m_normals);
			m_vertCount = (_rt.m_vertCount);
			m_triCount = (_rt.m_triCount);
		}

		return (*this);
	}

	rcMeshLoaderObj(const rcMeshLoaderObj& _rt) noexcept
	{
		using std::move;

		m_filename = (_rt.m_filename);
		m_scale = (_rt.m_scale);
		m_verts = (_rt.m_verts);
		m_tris = (_rt.m_tris);
		m_normals = (_rt.m_normals);
		m_vertCount = (_rt.m_vertCount);
		m_triCount = (_rt.m_triCount);
	}
	rcMeshLoaderObj& operator=(const rcMeshLoaderObj& _rt) noexcept
	{
		using std::move;

		if (this != &_rt)
		{
			m_filename = (_rt.m_filename);
			m_scale = (_rt.m_scale);
			m_verts = (_rt.m_verts);
			m_tris = (_rt.m_tris);
			m_normals = (_rt.m_normals);
			m_vertCount = (_rt.m_vertCount);
			m_triCount = (_rt.m_triCount);
		}

		return (*this);
	}

	bool load(const std::string& fileName);

	const auto& getVerts() const { return m_verts; }
	const float* getNormals() const { return m_normals.data(); }
	const int* getTris() const { return m_tris.data(); }
	int getVertCount() const { return m_vertCount; }
	int getTriCount() const { return m_triCount; }
	const std::string& getFileName() const { return m_filename; }

private:

	void addVertex(float x, float y, float z, int& cap);
	void addTriangle(int a, int b, int c, int& cap);

	std::string m_filename;
	float m_scale;
	std::vector<float> m_verts;
	std::vector<int> m_tris;
	std::vector<float> m_normals;
	int m_vertCount;
	int m_triCount;
};

#endif // MESHLOADER_OBJ

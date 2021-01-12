#ifndef NAVWORLD_H
#define NAVWORLD_H

#include <vector>
//#include "glimage.h"

struct GeomMesh
{
	GeomMesh() : verts(0), vnorms(0), nverts(0), tris(0), tnorms(0), ntris(0), edges(0), eflags(0) {}
	~GeomMesh() { delete [] verts; delete [] vnorms; delete [] tris; delete [] tnorms; delete [] tflags; delete [] edges; delete [] eflags; }

	float* verts;
	float* vnorms;
	int nverts;

	int* tris;
	unsigned char* tflags;
	float* tnorms;
	int ntris;

	unsigned short* edges;
	unsigned char* eflags;
	int nedges;

	float bmin[3], bmax[3];
	unsigned int color;
};

class NavWorld
{
	static constexpr int MAX_MESHES = 32;
	std::vector<GeomMesh> m_meshes;

	//GLImage m_tex;

public:
	NavWorld();
	~NavWorld();

	bool init();
	bool addMesh(const char* filename, const float* tm = 0);

	int getMeshCount() const { return m_meshes.size(); }
	const GeomMesh* getMesh(const int i) const { return &m_meshes[i]; }

	void getWorldBounds(float* bmin, float* bmax) const ;
	int getMaxMeshTriCount() const;

	bool raycast(const float* sp, const float* sq, float& tmin, unsigned char& tflag);

	void draw();

};

#endif // NAVWORLD_H

#include <algorithm>
#include <math.h>
#include <execution>

#include "SDL.h"
#include "SDL_Opengl.h"
#include "NavWorld.h"
#include "MeshLoaderObj.h"
#include "auxdraw.h"
#include "vecmath.h"


static bool raycastMesh(const float* p, const float* q, float& tmin, int& tri,
						const float* verts, const int nv,
						const int* tris, const int ntris)
{
	float dir[3];
	vsub(dir, q, p);

	tri = 0;
	tmin = 1;
	bool hit = false;

	for (int i = 0; i < ntris*3; i += 3)
	{
		float t = 1;
		const float* va = &verts[tris[i]*3];
		const float* vb = &verts[tris[i+1]*3];
		const float* vc = &verts[tris[i+2]*3];
		if (isectSegTriangle(p, q, va, vb, vc, t))
		{
			if (t < tmin)
			{
				tri = i/3;
				hit = true;
				tmin = t;
			}
		}
	}

	return hit;
}

static int buildMeshAdjacency(const int* tris, const int ntris, const int nverts,
							  unsigned short* outedges)
{
	struct Edge
	{
		unsigned short vert[2];
		unsigned short polyEdge[2];
		unsigned short poly[2];
	};

	// Based on code by Eric Lengyel from:
	// http://www.terathon.com/code/edges.php

	Edge* edges = new Edge[ntris*3];

	int maxEdgeCount = ntris*3;
	unsigned short* firstEdge = new unsigned short[nverts + maxEdgeCount];
	if (!firstEdge)
		return false;
	unsigned short* nextEdge = firstEdge + nverts;
	int edgeCount = 0;

	for (int i = 0; i < nverts; i++)
		firstEdge[i] = 0xffff;

	// Invalida indices are marked as 0xffff, the following code
	// handles them just fine.

	for (int i = 0; i < ntris; ++i)
	{
		const int* t = &tris[i*3];
		for (int j = 0; j < 3; ++j)
		{
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= 3) ? t[0] : t[j+1];
			if (v0 < v1)
			{
				Edge& edge = edges[edgeCount];
				edge.vert[0] = v0;
				edge.vert[1] = v1;
				edge.poly[0] = (unsigned short)i;
				edge.polyEdge[0] = (unsigned short)j;
				edge.poly[1] = (unsigned short)i;
				edge.polyEdge[1] = 0;
				// Insert edge
				nextEdge[edgeCount] = firstEdge[v0];
				firstEdge[v0] = edgeCount;
				edgeCount++;
			}
		}
	}

	for (int i = 0; i < ntris; ++i)
	{
		const int* t = &tris[i*3];
		for (int j = 0; j < 3; ++j)
		{
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= 3) ? t[0] : t[j+1];
			if (v0 > v1)
			{
				for (unsigned short e = firstEdge[v1]; e != 0xffff; e = nextEdge[e])
				{
					Edge& edge = edges[e];
					if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1])
					{
						edge.poly[1] = (unsigned short)i;
						edge.polyEdge[1] = (unsigned short)j;
						break;
					}
				}
			}
		}
	}

	// Store adjacency
	for (int i = 0; i < edgeCount; ++i)
	{
		outedges[i*4+0] = edges[i].poly[0];
		outedges[i*4+1] = edges[i].poly[1];
		outedges[i*4+2] = edges[i].vert[0];
		outedges[i*4+3] = edges[i].vert[1];
	}

	delete [] firstEdge;
	delete [] edges;

	return edgeCount;
}


NavWorld::NavWorld()
{
}

NavWorld::~NavWorld()
{
}

bool NavWorld::init()
{
	//if (!m_tex.create("tex.png", IMAGE_WRAP_REPEAT))
	//{
	//	return false;
	//}

	return true;
}

bool NavWorld::addMesh(const char* filename, const float* tm)
{
	if (m_meshes.size() >= MAX_MESHES)
	{
		printf("Too many meshes while loading '%s'\n", filename);
		return false;
	}

	rcMeshLoaderObj mesh;
	if (!mesh.load(filename))
	{
		printf("Could not load left: '%s'\n", filename);
		return false;
	}

	GeomMesh& m = m_meshes.emplace_back();

	m.ntris = mesh.getTriCount();
	m.tris = new int[m.ntris*3];
	if (!m.tris)
	{
		m_meshes.pop_back();
		return 0;
	}
	m.tnorms = new float[m.ntris*3];
	if (!m.tnorms)
	{
		m_meshes.pop_back();
		return 0;
	}
	m.tflags = new unsigned char[m.ntris];
	if (!m.tflags)
	{
		m_meshes.pop_back();
		return 0;
	}
	m.nverts = mesh.getVertCount();
	m.verts = new float[m.nverts*3];
	if (!m.verts)
	{
		m_meshes.pop_back();
		return 0;
	}
	m.vnorms = new float[m.nverts*3];
	if (!m.vnorms)
	{
		m_meshes.pop_back();
		return 0;
	}

	const int* tris = mesh.getTris().data();
	for (int i = 0; i < m.ntris*3; ++i)
		m.tris[i] = (int)tris[i];

	const float* tnorms = mesh.getNormals().data();
	memcpy(m.tnorms, tnorms, sizeof(float)*3*m.ntris);

	const float* verts = mesh.getVerts().data();
	memcpy(m.verts, verts, sizeof(float)*3*m.nverts);

	vcopy(m.bmin, m.verts);
	vcopy(m.bmax, m.verts);
	for (int i = 1; i < m.nverts; ++i)
	{
		vmin(m.bmin, &m.verts[i*3]);
		vmax(m.bmax, &m.verts[i*3]);
	}

	memset(m.vnorms, 0, sizeof(float)*3*m.nverts);
	for (int i = 0; i < m.ntris; ++i)
	{
		const int* t = &m.tris[i*3];
		const float* norm = &m.tnorms[i*3];
		vadd(&m.vnorms[t[0]*3], &m.vnorms[t[0]*3], norm);
		vadd(&m.vnorms[t[1]*3], &m.vnorms[t[1]*3], norm);
		vadd(&m.vnorms[t[2]*3], &m.vnorms[t[2]*3], norm);
	}
	for (int i = 0; i < m.nverts; ++i)
		_vnormalize(&m.vnorms[i*3]);

	m.edges = new unsigned short[m.ntris*3*4];
	m.nedges = buildMeshAdjacency(m.tris, m.ntris, m.nverts, m.edges);
	m.eflags = new unsigned char[m.nedges];

	const float thr = cosf(15.0f/180.0f*M_PI);
	for (int i = 0; i < m.nedges; ++i)
	{
		const unsigned short* e = &m.edges[i*4];
		m.eflags[i] = 0;
		if (e[0] != e[1])
		{
			const float dd = vdot(&m.tnorms[e[0]*3], &m.tnorms[e[1]*3]);
			if (dd < thr)
				m.eflags[i] = 1;
		}
	}

	const float thr2 = cosf(30.0f/180.0f*M_PI);
	for (int i = 0; i < m.ntris; ++i)
	{
		const float* norm = &m.tnorms[i*3];
		m.tflags[i] = 0;
		if (norm[1] > thr2)
			m.tflags[i] = 1;
	}

	m.color = RGBA(160,170,180,255);

	return true;
}

void NavWorld::getWorldBounds(float* bmin, float* bmax) const
{
	if (!m_meshes.size())
	{
		vset(bmin,0,0,0);
		vset(bmax,0,0,0);
		return;
	}
	vcopy(bmin, m_meshes.front().bmin);
	vcopy(bmax, m_meshes.front().bmax);
	for (auto& mesh : m_meshes)
	{
		vmin(bmin, mesh.bmin);
		vmax(bmax, mesh.bmax);
	}
}

int NavWorld::getMaxMeshTriCount() const
{
	if (m_meshes.empty())	return 0;

	return std::max_element(m_meshes.begin(), m_meshes.end(),
		[](const GeomMesh& left, const GeomMesh& right) { return left.ntris < right.ntris; })->ntris;
}

bool NavWorld::raycast(const float* sp, const float* sq, float& tmin, unsigned char& tflag)
{
	tmin = 1.0f;
	tflag = 0;
	bool hit = false;

	for (auto& mesh : m_meshes)
	{
		float t;
		int tri;
		if (raycastMesh(sp,sq,t,tri,mesh.verts,mesh.nverts,mesh.tris,mesh.ntris))
		{
			hit = true;
			if (t < tmin)
			{
				tflag = mesh.tflags[tri];
				tmin = t;
			}
		}
	}

	return hit;
}

static void drawMeshOutLine(const GeomMesh* m)
{
	const float offset = 0.01f;
	glBegin(GL_LINES);
	for (int i = 0; i < m->nedges; ++i)
	{
		if ((m->eflags[i] & 1) == 0) continue;
		const unsigned short* e = &m->edges[i*4];
		const float* na = &m->tnorms[e[0]*3];
		const float* nb = &m->tnorms[e[1]*3];
		const float* va = &m->verts[e[2]*3];
		const float* vb = &m->verts[e[3]*3];
		float n[3];
		vlerp(n,na,nb,0.5f);
		float pa[3], pb[3];
		vmad(pa,va,n,offset);
		vmad(pb,vb,n,offset);
		glNormal3fv(n);
		glVertex3fv(pa);
		glVertex3fv(pb);
	}
	glEnd();
}

static void drawMesh(const GeomMesh* m)
{
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < m->ntris; ++i)
	{
		const int* t = &m->tris[i*3];
		const float* norm = &m->tnorms[i*3];
		glNormal3fv(norm);
		glVertex3fv(&m->verts[t[0]*3]);
		glVertex3fv(&m->verts[t[1]*3]);
		glVertex3fv(&m->verts[t[2]*3]);
	}
	glEnd();
}

static void drawMeshTextured(const GeomMesh* m, unsigned int col)
{
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);

	unsigned int col2 = clerp(col, RGBA(220,0,0,255), 32);

	const float s = 1.0f / 4.0f;
	glBegin(GL_TRIANGLES);
	for (int i = 0; i < m->ntris; ++i)
	{
		const int* t = &m->tris[i*3];
		const float* norm = &m->tnorms[i*3];

		const float* va = &m->verts[t[0]*3];
		const float* vb = &m->verts[t[1]*3];
		const float* vc = &m->verts[t[2]*3];

		int axis = 0;
		if (fabsf(norm[1]) > fabsf(norm[axis]))
			axis = 1;
		if (fabsf(norm[2]) > fabsf(norm[axis]))
			axis = 2;
		int ta = (axis+1)%3;
		int tb = (axis+2)%3;

		if (m->tflags[i])
			glColor4ubv((GLubyte*)&col);
		else
			glColor4ubv((GLubyte*)&col2);

		glNormal3fv(norm);

		glTexCoord2f(va[ta]*s,va[tb]*s);
		glVertex3fv(va);

		glTexCoord2f(vb[ta]*s,vb[tb]*s);
		glVertex3fv(vb);

		glTexCoord2f(vc[ta]*s,vc[tb]*s);
		glVertex3fv(vc);
	}
	glEnd();

	glDisable(GL_COLOR_MATERIAL);
}

void NavWorld::draw()
{
	float lightDir[3];
	vset(lightDir, 1,2,1);
	_vnormalize(lightDir);

	glEnable(GL_NORMALIZE);

	const float ambientCol[4] = {0.4f, 0.45f, 0.45f, 1.0f};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientCol);

	const float fogCol[4] = {0.62f, 0.61f, 0.60f, 1.0f};
	float fogStart = 7.0f;
	float fogEnd = 40.0f;
	glFogi(GL_FOG_MODE, GL_LINEAR);
	glFogf(GL_FOG_START, fogStart);
	glFogf(GL_FOG_END, fogEnd);
	glFogfv(GL_FOG_COLOR, fogCol);
	glEnable(GL_FOG);

	setDirLight(0, lightDir, RGBA(255,240,200), 1.0f);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);

	glLineWidth(2.0f);

	glEnable(GL_TEXTURE_2D);
	//m_tex.bind();
	for (auto& mesh : m_meshes)
	{
		setMaterial(mesh.color, 0.1f, 0.0f);
		drawMeshTextured(&mesh, mesh.color);
	}
	glDisable(GL_TEXTURE_2D);

	for (auto& mesh : m_meshes)
	{
		setMaterial(clerp(mesh.color, RGBA(0,16,32,0), 16), 0.1f, 0.0f);
		drawMeshOutLine(&mesh);
	}

	glLineWidth(1.0f);

	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
	glDisable(GL_FOG);
}

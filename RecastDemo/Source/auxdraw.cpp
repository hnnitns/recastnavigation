#include "auxdraw.h"
#include <vector>
#include <float.h>
#include <math.h>
#include "vecmath.h"
#include "SDL_Opengl.h"


void drawSphere(const float* c, const float r, const int n)
{
	const unsigned nj = n/2+1;
	const unsigned ni = n;
	std::vector<float> verts(nj*ni*3), normals(nj*ni*3);
	std::vector<unsigned short> indices((nj-1)*ni*6);

	if (n < 4 || r <= 0) return;

	for (int j = 0; j < nj; j++)
	{
		float theta1 = (j * M_PI) / (float)(nj-1) - M_PI/2;
		for (int i = 0; i < ni; i++)
		{
			float theta3 = (i * M_PI*2) / (float)ni;

			float* e = &normals[(j*ni+i)*3];
			float* p = &verts[(j*ni+i)*3];

			e[0] = cosf(theta1) * cosf(theta3);
			e[1] = sinf(theta1);
			e[2] = cosf(theta1) * sinf(theta3);
			p[0] = c[0] + r * e[0];
			p[1] = c[1] + r * e[1];
			p[2] = c[2] + r * e[2];
		}
	}

	unsigned short* dst = indices.data();
	for (unsigned j = 0; j < nj-1; j++)
	{
		for (unsigned i = 0; i < ni; i++)
		{
			unsigned ii = (i+1) % ni;
			*dst++ = (unsigned short)(j*ni+i);
			*dst++ = (unsigned short)((j+1)*ni+i);
			*dst++ = (unsigned short)((j+1)*ni+ii);

			*dst++ = (unsigned short)(j*ni+i);
			*dst++ = (unsigned short)((j+1)*ni+ii);
			*dst++ = (unsigned short)(j*ni+ii);
		}
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glVertexPointer(3, GL_FLOAT, 0, verts.data());
	glNormalPointer(GL_FLOAT, 0, normals.data());

	glDrawElements(GL_TRIANGLES, (nj-1)*ni*6, GL_UNSIGNED_SHORT, indices.data());

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}

void drawEllipsoid(const float* c, const float* r, const int n)
{
	const unsigned nj = n/2+1;
	const unsigned ni = n;
	std::vector<float> verts(nj * ni * 3), normals(nj * ni * 3);
	std::vector<unsigned short> indices((nj - 1) * ni * 6);

	if (n < 4 || r <= 0) return;

	for (int j = 0; j < nj; j++)
	{
		float theta1 = (j * M_PI) / (float)(nj-1) - M_PI/2;
		for (int i = 0; i < ni; i++)
		{
			float theta3 = (i * M_PI*2) / (float)ni;

			float* e = &normals[(j*ni+i)*3];
			float* p = &verts[(j*ni+i)*3];

			e[0] = cosf(theta1) * cosf(theta3);
			e[1] = sinf(theta1);
			e[2] = cosf(theta1) * sinf(theta3);
			p[0] = c[0] + r[0] * e[0];
			p[1] = c[1] + r[1] * e[1];
			p[2] = c[2] + r[2] * e[2];
		}
	}

	unsigned short* dst = indices.data();
	for (unsigned j = 0; j < nj-1; j++)
	{
		for (unsigned i = 0; i < ni; i++)
		{
			unsigned ii = (i+1) % ni;
			*dst++ = (unsigned short)(j*ni+i);
			*dst++ = (unsigned short)((j+1)*ni+i);
			*dst++ = (unsigned short)((j+1)*ni+ii);

			*dst++ = (unsigned short)(j*ni+i);
			*dst++ = (unsigned short)((j+1)*ni+ii);
			*dst++ = (unsigned short)(j*ni+ii);
		}
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glVertexPointer(3, GL_FLOAT, 0, verts.data());
	glNormalPointer(GL_FLOAT, 0, normals.data());

	glDrawElements(GL_TRIANGLES, (nj-1)*ni*6, GL_UNSIGNED_SHORT, indices.data());

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}

void drawCapsule(const float* a, const float* b, const float rad, const int n)
{
	int i, j;
	float theta1,theta2,theta3;
	float ax[3], ay[3], az[3];
	vsub(ay, b,a);
	_vnormalize(ay);
	vset(az,0,0,1);
	if (fabsf(ay[2]) > 0.5f)
		vset(az,1,0,0);
	vcross(ax,ay,az);
	_vnormalize(ax);
	vcross(ay,az,ax);
	_vnormalize(ay);

	int ncap = n / 3;
	if (ncap < 2) ncap = 2;

	float e[3], te[3], p[3];

	for (j = 0; j < ncap; j++)
	{
		theta1 = (j * M_PI / 2.0f) / ncap - M_PI * 0.5f;
		theta2 = ((j + 1) * M_PI / 2.0f) / ncap - M_PI * 0.5f;

		const float cos1 = cosf(theta1);
		const float sin1 = sinf(theta1);
		const float cos2 = cosf(theta2);
		const float sin2 = sinf(theta2);

		glBegin(GL_TRIANGLE_STRIP);

		for (i = 0; i <= n; i++)
		{
			theta3 = (i * 2.0f * M_PI) / n;
			const float cos3 = cosf(theta3);
			const float sin3 = sinf(theta3);

			e[0] = cos1 * cos3;
			e[1] = sin1;
			e[2] = cos1 * sin3;
			vscale(te, ax, e[0]);
			vmad(te, te, ay, e[1]);
			vmad(te, te, az, e[2]);
			vmad(p, a, te, rad);

			glNormal3fv(e);
			glVertex3fv(p);

			e[0] = cos2 * cos3;
			e[1] = sin2;
			e[2] = cos2 * sin3;
			vscale(te, ax, e[0]);
			vmad(te, te, ay, e[1]);
			vmad(te, te, az, e[2]);
			vmad(p, a, te, rad);

			glNormal3fv(e);
			glVertex3fv(p);
		}
		glEnd();
	}

	// Mid section
	glBegin(GL_TRIANGLE_STRIP);

	for (i = 0; i <= n; i++)
	{
		theta3 = (i * 2.0f * M_PI) / n;

		e[0] = cosf(theta3);
		e[1] = 0;
		e[2] = sinf(theta3);
		vscale(te, ax, e[0]);
		vmad(te, te, ay, e[1]);
		vmad(te, te, az, e[2]);

		vmad(p, a, te, rad);
		glNormal3fv(e);
		glVertex3fv(p);

		vmad(p, b, te, rad);
		glNormal3fv(e);
		glVertex3fv(p);
	}
	glEnd();

	for (j = ncap; j < ncap*2; j++)
	{
		theta1 = (j * M_PI / 2.0f) / ncap - M_PI * 0.5f;
		theta2 = ((j + 1) * M_PI / 2.0f) / ncap - M_PI * 0.5f;

		const float cos1 = cosf(theta1);
		const float sin1 = sinf(theta1);
		const float cos2 = cosf(theta2);
		const float sin2 = sinf(theta2);

		glBegin(GL_TRIANGLE_STRIP);

		for (i = 0; i <= n; i++)
		{
			theta3 = (i * 2.0f * M_PI) / n;
			const float cos3 = cosf(theta3);
			const float sin3 = sinf(theta3);

			e[0] = cos1 * cos3;
			e[1] = sin1;
			e[2] = cos1 * sin3;
			vscale(te, ax, e[0]);
			vmad(te, te, ay, e[1]);
			vmad(te, te, az, e[2]);
			vmad(p, b, te, rad);

			glNormal3fv(e);
			glVertex3fv(p);

			e[0] = cos2 * cos3;
			e[1] = sin2;
			e[2] = cos2 * sin3;
			vscale(te, ax, e[0]);
			vmad(te, te, ay, e[1]);
			vmad(te, te, az, e[2]);
			vmad(p, b, te, rad);

			glNormal3fv(e);
			glVertex3fv(p);
		}
		glEnd();
	}
}

void drawHemiSphere(const float* c, const float r, const float h, const int n)
{
	const unsigned nj = n/2+1;
	const unsigned ni = n;
	std::vector<float> verts(nj * ni * 3), normals(nj * ni * 3);
	std::vector<unsigned short> indices((nj - 1) * ni * 6);

	if (n < 4 || r <= 0) return;

	for (int j = 0; j < nj; j++)
	{
		float theta1 = (j * M_PI/2) / (float)(nj-1); // - M_PI/2;
		for (int i = 0; i < ni; i++)
		{
			float theta3 = (i * M_PI*2) / (float)ni;

			float* e = &normals[(j*ni+i)*3];
			float* p = &verts[(j*ni+i)*3];

			e[0] = cosf(theta1) * cosf(theta3);
			e[1] = sinf(theta1);
			e[2] = cosf(theta1) * sinf(theta3);
			p[0] = c[0] + r * e[0];
			p[1] = c[1] + h * e[1];
			p[2] = c[2] + r * e[2];
		}
	}

	unsigned short* dst = indices.data();
	for (unsigned j = 0; j < nj-1; j++)
	{
		for (unsigned i = 0; i < ni; i++)
		{
			unsigned ii = (i+1) % ni;
			*dst++ = (unsigned short)(j*ni+i);
			*dst++ = (unsigned short)((j+1)*ni+i);
			*dst++ = (unsigned short)((j+1)*ni+ii);

			*dst++ = (unsigned short)(j*ni+i);
			*dst++ = (unsigned short)((j+1)*ni+ii);
			*dst++ = (unsigned short)(j*ni+ii);
		}
	}

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glVertexPointer(3, GL_FLOAT, 0, verts.data());
	glNormalPointer(GL_FLOAT, 0, normals.data());

	glDrawElements(GL_TRIANGLES, (nj-1)*ni*6, GL_UNSIGNED_SHORT, indices.data());

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}

void drawBox(const float* bmin, const float* bmax)
{
	float c[3], e[3];
	float s[3];
	vsub(s, bmax,bmin);
	vscale(e, s, 0.5f);
	vadd(c, bmin, e);
	const float ax[3] = {1,0,0}, ay[3] = {0,1,0}, az[3] = {0,0,1};
	drawBox(c, e, ax, ay, az);
}

void drawBox(const float* c, const float* e, const float* ax, const float* ay, const float* az)
{
	float corners[8*3];
	float* v = corners;

	// Create box corners
	vmad(v, c, ax, -e[0]);
	vmad(v, v, ay, -e[1]);
	vmad(v, v, az, -e[2]);
	v += 3;
	vmad(v, c, ax, +e[0]);
	vmad(v, v, ay, -e[1]);
	vmad(v, v, az, -e[2]);
	v += 3;
	vmad(v, c, ax, +e[0]);
	vmad(v, v, ay, -e[1]);
	vmad(v, v, az, +e[2]);
	v += 3;
	vmad(v, c, ax, -e[0]);
	vmad(v, v, ay, -e[1]);
	vmad(v, v, az, +e[2]);
	v += 3;

	vmad(v, c, ax, -e[0]);
	vmad(v, v, ay, +e[1]);
	vmad(v, v, az, -e[2]);
	v += 3;
	vmad(v, c, ax, +e[0]);
	vmad(v, v, ay, +e[1]);
	vmad(v, v, az, -e[2]);
	v += 3;
	vmad(v, c, ax, +e[0]);
	vmad(v, v, ay, +e[1]);
	vmad(v, v, az, +e[2]);
	v += 3;
	vmad(v, c, ax, -e[0]);
	vmad(v, v, ay, +e[1]);
	vmad(v, v, az, +e[2]);

	float normals[12*3*3];
	float verts[12*3*3];
	float* n = normals;
	v = verts;


	// Top
	vcopy(n, ay); n += 3;
	vcopy(v, &corners[4*3]); v += 3;
	vcopy(n, ay); n += 3;
	vcopy(v, &corners[6*3]); v += 3;
	vcopy(n, ay); n += 3;
	vcopy(v, &corners[5*3]); v += 3;

	vcopy(n, ay); n += 3;
	vcopy(v, &corners[4*3]); v += 3;
	vcopy(n, ay); n += 3;
	vcopy(v, &corners[7*3]); v += 3;
	vcopy(n, ay); n += 3;
	vcopy(v, &corners[6*3]); v += 3;

	// Bottom
	vneg(n, ay); n += 3;
	vcopy(v, &corners[0*3]); v += 3;
	vneg(n, ay); n += 3;
	vcopy(v, &corners[1*3]); v += 3;
	vneg(n, ay); n += 3;
	vcopy(v, &corners[2*3]); v += 3;

	vneg(n, ay); n += 3;
	vcopy(v, &corners[0*3]); v += 3;
	vneg(n, ay); n += 3;
	vcopy(v, &corners[2*3]); v += 3;
	vneg(n, ay); n += 3;
	vcopy(v, &corners[3*3]); v += 3;

	// Side0
	vcopy(n, ax); n += 3;
	vcopy(v, &corners[1*3]); v += 3;
	vcopy(n, ax); n += 3;
	vcopy(v, &corners[5*3]); v += 3;
	vcopy(n, ax); n += 3;
	vcopy(v, &corners[6*3]); v += 3;

	vcopy(n, ax); n += 3;
	vcopy(v, &corners[1*3]); v += 3;
	vcopy(n, ax); n += 3;
	vcopy(v, &corners[6*3]); v += 3;
	vcopy(n, ax); n += 3;
	vcopy(v, &corners[2*3]); v += 3;

	// Side1
	vneg(n, ax); n += 3;
	vcopy(v, &corners[3*3]); v += 3;
	vneg(n, ax); n += 3;
	vcopy(v, &corners[7*3]); v += 3;
	vneg(n, ax); n += 3;
	vcopy(v, &corners[4*3]); v += 3;

	vneg(n, ax); n += 3;
	vcopy(v, &corners[3*3]); v += 3;
	vneg(n, ax); n += 3;
	vcopy(v, &corners[4*3]); v += 3;
	vneg(n, ax); n += 3;
	vcopy(v, &corners[0*3]); v += 3;

	// Side2
	vcopy(n, az); n += 3;
	vcopy(v, &corners[2*3]); v += 3;
	vcopy(n, az); n += 3;
	vcopy(v, &corners[6*3]); v += 3;
	vcopy(n, az); n += 3;
	vcopy(v, &corners[7*3]); v += 3;

	vcopy(n, az); n += 3;
	vcopy(v, &corners[2*3]); v += 3;
	vcopy(n, az); n += 3;
	vcopy(v, &corners[7*3]); v += 3;
	vcopy(n, az); n += 3;
	vcopy(v, &corners[3*3]); v += 3;

	// Side3
	vneg(n, az); n += 3;
	vcopy(v, &corners[0*3]); v += 3;
	vneg(n, az); n += 3;
	vcopy(v, &corners[4*3]); v += 3;
	vneg(n, az); n += 3;
	vcopy(v, &corners[5*3]); v += 3;

	vneg(n, az); n += 3;
	vcopy(v, &corners[0*3]); v += 3;
	vneg(n, az); n += 3;
	vcopy(v, &corners[5*3]); v += 3;
	vneg(n, az); n += 3;
	vcopy(v, &corners[1*3]); v += 3;

	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);

	glVertexPointer(3, GL_FLOAT, 0, verts);
	glNormalPointer(GL_FLOAT, 0, normals);

	glDrawArrays(GL_TRIANGLES, 0, 12*3);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);
}

void drawBoxWire(const float* c, const float* e, const float* ax, const float* ay, const float* az)
{
	float corners[8*3];
	float* v = corners;

	// Create box corners
	vmad(v, c, ax, -e[0]);
	vmad(v, v, ay, -e[1]);
	vmad(v, v, az, -e[2]);
	v += 3;
	vmad(v, c, ax, +e[0]);
	vmad(v, v, ay, -e[1]);
	vmad(v, v, az, -e[2]);
	v += 3;
	vmad(v, c, ax, +e[0]);
	vmad(v, v, ay, -e[1]);
	vmad(v, v, az, +e[2]);
	v += 3;
	vmad(v, c, ax, -e[0]);
	vmad(v, v, ay, -e[1]);
	vmad(v, v, az, +e[2]);
	v += 3;

	vmad(v, c, ax, -e[0]);
	vmad(v, v, ay, +e[1]);
	vmad(v, v, az, -e[2]);
	v += 3;
	vmad(v, c, ax, +e[0]);
	vmad(v, v, ay, +e[1]);
	vmad(v, v, az, -e[2]);
	v += 3;
	vmad(v, c, ax, +e[0]);
	vmad(v, v, ay, +e[1]);
	vmad(v, v, az, +e[2]);
	v += 3;
	vmad(v, c, ax, -e[0]);
	vmad(v, v, ay, +e[1]);
	vmad(v, v, az, +e[2]);

	static const int inds[4*3*2] =
	{
		0,1, 1,2, 2,3, 3,0,	// bot
		4,5, 5,6, 6,7, 7,4,	// top
		0,4, 1,5, 2,6, 3,7,	// sides
	};

	glBegin(GL_LINES);
	for (int i = 0; i < 4*3*2; ++i)
		glVertex3fv(&corners[inds[i]*3]);
	glEnd();
}


void drawBoxWire(const float* bmin, const float* bmax)
{
	float corners[8*3];
	float* v = corners;

	// Create box corners
	vset(v, bmin[0],bmin[1],bmin[2]);
	v += 3;
	vset(v, bmax[0],bmin[1],bmin[2]);
	v += 3;
	vset(v, bmax[0],bmin[1],bmax[2]);
	v += 3;
	vset(v, bmin[0],bmin[1],bmax[2]);
	v += 3;

	vset(v, bmin[0],bmax[1],bmin[2]);
	v += 3;
	vset(v, bmax[0],bmax[1],bmin[2]);
	v += 3;
	vset(v, bmax[0],bmax[1],bmax[2]);
	v += 3;
	vset(v, bmin[0],bmax[1],bmax[2]);

	static const int inds[4*3*2] =
	{
		0,1, 1,2, 2,3, 3,0,	// bot
		4,5, 5,6, 6,7, 7,4,	// top
		0,4, 1,5, 2,6, 3,7,	// sides
	};

	glBegin(GL_LINES);
	for (int i = 0; i < 4*3*2; ++i)
		glVertex3fv(&corners[inds[i]*3]);
	glEnd();
}

void drawBoxWire(const float* c, const float* ax, const float* ay, const float* az,
				 const float* bmin, const float* bmax)
{
	float corners[8*3];
	float* v = corners;

	// Create box corners
	vmad(v, c, ax, bmin[0]);
	vmad(v, v, ay, bmin[1]);
	vmad(v, v, az, bmin[2]);
	v += 3;
	vmad(v, c, ax, bmax[0]);
	vmad(v, v, ay, bmin[1]);
	vmad(v, v, az, bmin[2]);
	v += 3;
	vmad(v, c, ax, bmax[0]);
	vmad(v, v, ay, bmin[1]);
	vmad(v, v, az, bmax[2]);
	v += 3;
	vmad(v, c, ax, bmin[0]);
	vmad(v, v, ay, bmin[1]);
	vmad(v, v, az, bmax[2]);
	v += 3;

	vmad(v, c, ax, bmin[0]);
	vmad(v, v, ay, bmax[1]);
	vmad(v, v, az, bmin[2]);
	v += 3;
	vmad(v, c, ax, bmax[0]);
	vmad(v, v, ay, bmax[1]);
	vmad(v, v, az, bmin[2]);
	v += 3;
	vmad(v, c, ax, bmax[0]);
	vmad(v, v, ay, bmax[1]);
	vmad(v, v, az, bmax[2]);
	v += 3;
	vmad(v, c, ax, bmin[0]);
	vmad(v, v, ay, bmax[1]);
	vmad(v, v, az, bmax[2]);

	static const int inds[4*3*2] =
	{
		0,1, 1,2, 2,3, 3,0,	// bot
		4,5, 5,6, 6,7, 7,4,	// top
		0,4, 1,5, 2,6, 3,7,	// sides
	};

	glBegin(GL_LINES);
	for (int i = 0; i < 4*3*2; ++i)
		glVertex3fv(&corners[inds[i]*3]);
	glEnd();
}

void setOmniLight(int iNum, const float* pos, unsigned int col, float s)
{
	GLfloat	diffuse[4] = { 1, 1, 1, 1 };
	GLfloat	spec[4] = { 1, 1, 1, 1 };
	GLfloat	lightPos[4] = { 0, 0, 0, 1 };

	// Store light color
	diffuse[0] = (cgetr(col)/255.0f) * s;
	diffuse[1] = (cgetg(col)/255.0f) * s;
	diffuse[2] = (cgetb(col)/255.0f) * s;

	lightPos[0] = pos[0];
	lightPos[1] = pos[1];
	lightPos[2] = pos[2];
	lightPos[3] = 1;

	// Set light
	glLightfv(GL_LIGHT0 + iNum, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0 + iNum, GL_SPECULAR, spec);
	glLightfv(GL_LIGHT0 + iNum, GL_POSITION, lightPos);
}

void setDirLight(int iNum, const float* dir, unsigned int col, float s)
{
	GLfloat	diffuse[4] = { 1, 1, 1, 1 };
	GLfloat	spec[4] = { 1, 1, 1, 1 };
	GLfloat	lightPos[4] = { 0, 0, 0, 1 };

	// Store light color
	diffuse[0] = (cgetr(col)/255.0f) * s;
	diffuse[1] = (cgetg(col)/255.0f) * s;
	diffuse[2] = (cgetb(col)/255.0f) * s;

	lightPos[0] = dir[0];
	lightPos[1] = dir[1];
	lightPos[2] = dir[2];
	lightPos[3] = 0;

	// Set light
	glLightfv(GL_LIGHT0 + iNum, GL_DIFFUSE, diffuse);
	glLightfv(GL_LIGHT0 + iNum, GL_SPECULAR, spec);
	glLightfv(GL_LIGHT0 + iNum, GL_POSITION, lightPos);
}

void setMaterial(unsigned int col, float specular, float em)
{
	//	GLfloat	black[4] = {0,0,0,1}; //0,0.05f,0.15f,1};
	GLfloat	spec[4] = {1,1,1,1};
	GLfloat	diffuse[4] = { cgetr(col)/255.0f, cgetg(col)/255.0f, cgetb(col)/255.0f, cgeta(col)/255.0f };
	GLfloat	emission[4] = { cgetr(col)/255.0f*em, cgetg(col)/255.0f*em, cgetb(col)/255.0f*em, 1.0f };

	spec[0] = specular;
	spec[1] = specular;
	spec[2] = specular;

	// If lighting is used, setup material
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec);
	glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emission);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 32.0f);
}

void drawLine(const float* a, const float* b)
{
	float verts[6];
	vcopy(&verts[0], a);
	vcopy(&verts[3], b);
	glVertexPointer(3, GL_FLOAT, 0, verts);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDrawArrays(GL_LINES, 0, 2);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void drawLineStrip(const float* pts, const int npts)
{
	glVertexPointer(3, GL_FLOAT, 0, pts);
	glEnableClientState(GL_VERTEX_ARRAY);
	glDrawArrays(GL_LINE_STRIP, 0, npts);
	glDisableClientState(GL_VERTEX_ARRAY);
}

void drawCircleXZ(float cx, float cy, float cr, float y)
{
	const int nseg = 30;
	float circle[3*nseg];
	for (int i = 0; i < nseg; ++i)
	{
		float a = ((float)i/(float)(nseg-1))*M_PI*2;
		circle[i*3+0] = cx + cosf(a)*cr;
		circle[i*3+1] = y;
		circle[i*3+2] = cy + sinf(a)*cr;
	}
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, circle);
	glDrawArrays(GL_LINE_STRIP, 0, nseg);
}

void drawFatCircleXZ(float cx, float cy, float r0, float r1, float y)
{
	const int nseg = 30;
	float circle[2*3*nseg];
	for (int i = 0; i < nseg; ++i)
	{
		float a = ((float)i/(float)(nseg-1))*M_PI*2;
		const float dx = cosf(a);
		const float dy = sinf(a);
		circle[i*6+0] = cx + dx*r1;
		circle[i*6+1] = y;
		circle[i*6+2] = cy + dy*r1;

		circle[i*6+3] = cx + dx*r0;
		circle[i*6+4] = y;
		circle[i*6+5] = cy + dy*r0;
	}
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, circle);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, nseg*2);
}

void drawFatStarXZ(float cx, float cy, float r0, float r1, float inset, float y, int n)
{
	const int nseg = 31;
	float circle[2*3*nseg];
	if (n > nseg) n = nseg;
	for (int i = 0; i < n; ++i)
	{
		float a = ((float)i/(float)(n-1))*M_PI*2;
		const float dx = cosf(a);
		const float dy = sinf(a);
		const float s = (i & 1) ? inset : 1;
		circle[i*6+0] = cx + dx*r1*s;
		circle[i*6+1] = y;
		circle[i*6+2] = cy + dy*r1*s;

		circle[i*6+3] = cx + dx*r0*s;
		circle[i*6+4] = y;
		circle[i*6+5] = cy + dy*r0*s;
	}
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, circle);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, n*2);
}

void drawRectXZ(float minx, float minz, float maxx, float maxz, float y)
{
	float rect[3*5];
	vset(&rect[0], minx,y,minz);
	vset(&rect[3], maxx,y,minz);
	vset(&rect[6], maxx,y,maxz);
	vset(&rect[9], minx,y,maxz);
	vset(&rect[12], minx,y,minz);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, rect);
	glDrawArrays(GL_LINE_STRIP, 0, 5);
}

void drawFatRectXZ(float minx, float minz, float maxx, float maxz, float r, float y)
{
	float rect[3*5*2];

	vset(&rect[0], minx-r,y,minz-r);
	vset(&rect[3], minx,y,minz);

	vset(&rect[6], maxx+r,y,minz-r);
	vset(&rect[9], maxx,y,minz);

	vset(&rect[12], maxx+r,y,maxz+r);
	vset(&rect[15], maxx,y,maxz);

	vset(&rect[18], minx-r,y,maxz+r);
	vset(&rect[21], minx,y,maxz);

	vset(&rect[24], minx-r,y,minz-r);
	vset(&rect[27], minx,y,minz);

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, rect);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 5*2);
}

void drawArrowXZ(float sx, float sy, float ex, float ey, float w, float y)
{
	float verts[3*6];
	float dx = ex-sx;
	float dy = ey-sy;
	float d = sqrtf(dx*dx+dy*dy);
	dx/=d; dy/=d;
	float nx = -dy;
	float ny = dx;

	vset(&verts[0], sx,y,sy);
	vset(&verts[3], ex,y,ey);

	vset(&verts[6], ex + dx*-w + nx*w/2, y, ey + dy*-w + ny*w/2);
	vset(&verts[9], ex,y,ey);

	vset(&verts[12], ex + dx*-w + nx*-w/2, y, ey + dy*-w + ny*-w/2);
	vset(&verts[15], ex,y,ey);

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, verts);
	glDrawArrays(GL_LINES, 0, 6);
}


void drawFatArrowXZ(float sx, float sy, float ex, float ey, float w, float y)
{
	float verts[3*7];
	float dx = ex-sx;
	float dy = ey-sy;
	float d = sqrtf(dx*dx+dy*dy);
	dx/=d; dy/=d;
	float nx = -dy;
	float ny = dx;

	vset(&verts[0], ex,y,ey);

	vset(&verts[3], ex-dx*w-nx*w*2,y,ey-dy*w-ny*w*2);
	vset(&verts[6], ex-dx*w-nx*w,y,ey-dy*w-ny*w);
	vset(&verts[9], sx-nx*w,y,sy-ny*w);

	vset(&verts[12], sx+nx*w,y,sy+ny*w);
	vset(&verts[15], ex-dx*w+nx*w,y,ey-dy*w+ny*w);
	vset(&verts[18], ex-dx*w+nx*w*2,y,ey-dy*w+ny*w*2);

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, verts);
	glDrawArrays(GL_TRIANGLE_FAN, 0, 7);
}

void drawRectXY(float minx, float miny, float maxx, float maxy, float z)
{
	float rect[3*5];
	vset(&rect[0], minx,miny,z);
	vset(&rect[3], maxx,miny,z);
	vset(&rect[6], maxx,maxy,z);
	vset(&rect[9], minx,maxy,z);
	vset(&rect[12], minx,miny,z);
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, rect);
	glDrawArrays(GL_LINE_STRIP, 0, 5);
}

void drawCircleXY(float cx, float cy, float cr, float z)
{
	const int nseg = 20;
	float circle[3*nseg];
	for (int i = 0; i < nseg; ++i)
	{
		float a = ((float)i/(float)(nseg-1))*M_PI*2;
		circle[i*3+0] = cx + cosf(a)*cr;
		circle[i*3+1] = cy + sinf(a)*cr;
		circle[i*3+2] = z;
	}
	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, circle);
	glDrawArrays(GL_LINE_STRIP, 0, nseg);
}

void drawArrowXY(float sx, float sy, float ex, float ey, float w, float z)
{
	float verts[3*6];
	float dx = ex-sx;
	float dy = ey-sy;
	float d = sqrtf(dx*dx+dy*dy);
	dx/=d; dy/=d;
	float nx = -dy;
	float ny = dx;

	vset(&verts[0], sx,sy,z);
	vset(&verts[3], ex,ey,z);

	vset(&verts[6], ex + dx*-w + nx*w/2, ey + dy*-w + ny*w/2, z);
	vset(&verts[9], ex,ey,z);

	vset(&verts[12], ex + dx*-w + nx*-w/2, ey + dy*-w + ny*-w/2, z);
	vset(&verts[15], ex,ey,z);

	glEnableClientState(GL_VERTEX_ARRAY);
	glVertexPointer(3, GL_FLOAT, 0, verts);
	glDrawArrays(GL_LINES, 0, 6);
}


inline int bit(int a, int b)
{
	return (a & (1 << b)) >> b;
}

unsigned int chashed(int i, int a)
{
	int	r = bit(i, 0) + bit(i, 3) * 2 + 1;
	int	g = bit(i, 1) + bit(i, 4) * 2 + 1;
	int	b = bit(i, 2) + bit(i, 5) * 2 + 1;
	return RGBA(r*63, g*63, b*63, a);
}


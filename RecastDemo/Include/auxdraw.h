#ifndef AUXDRAW_H
#define AUXDRAW_H

inline unsigned int RGBA(unsigned int r, unsigned int g, unsigned int b, unsigned int a = 255)
{
	return (a<<24) | (b<<16) | (g<<8) | r;
}

inline unsigned int cgetr(unsigned int c) { return c & 0xff; }
inline unsigned int cgetg(unsigned int c) { return (c>>8) & 0xff; }
inline unsigned int cgetb(unsigned int c) { return (c>>16) & 0xff; }
inline unsigned int cgeta(unsigned int c) { return (c>>24) & 0xff; }

inline unsigned int clerp(unsigned int ca, unsigned int cb, unsigned int u)
{
	unsigned int r = (cgetr(ca)*(255-u) + cgetr(cb)*u)/255;
	unsigned int g = (cgetg(ca)*(255-u) + cgetg(cb)*u)/255;
	unsigned int b = (cgetb(ca)*(255-u) + cgetb(cb)*u)/255;
	unsigned int a = (cgeta(ca)*(255-u) + cgeta(cb)*u)/255;
	return RGBA(r,g,b,a);
}

inline unsigned int ctrans(unsigned int c, unsigned int a)
{
	return (a<<24) | (c & 0x00ffffff);
}

unsigned int chashed(int i, int a);

void drawSphere(const float* c, const float r, const int n);
void drawEllipsoid(const float* c, const float* r, const int n);
void drawCapsule(const float* a, const float* b, const float rad, const int n);
void drawHemiSphere(const float* c, const float r, const float h, const int n);
void drawBox(const float* c, const float* e, const float* ax, const float* ay, const float* az);
void drawBox(const float* bmin, const float* bmax);
void drawBoxWire(const float* c, const float* e, const float* ax, const float* ay, const float* az);
void drawBoxWire(const float* bmin, const float* bmax);
void drawBoxWire(const float* c, const float* ax, const float* ay, const float* az, const float* bmin, const float* bmax);
void setOmniLight(int iNum, const float* pos, unsigned int col, float s);
void setDirLight(int iNum, const float* dir, unsigned int col, float s);
void setMaterial(unsigned int col, float specular = 0.0f, float em = 0.0f);
void drawLine(const float* a, const float* b);
void drawLineStrip(const float* pts, const int npts);



void drawCircleXZ(float cx, float cy, float cr, float y = 0);
void drawFatCircleXZ(float cx, float cy, float r0, float r1, float y = 0);
void drawFatStarXZ(float cx, float cy, float r0, float r1, float inset, float y = 0, int n = 31);
void drawRectXZ(float minx, float minz, float maxx, float maxz, float y = 0);
void drawFatRectXZ(float minx, float minz, float maxx, float maxz, float r, float y = 0);
void drawArrowXZ(float sx, float sy, float ex, float ey, float w, float y = 0);
void drawFatArrowXZ(float sx, float sy, float ex, float ey, float w, float y = 0);
void drawRectXY(float minx, float miny, float maxx, float maxy, float z = 0);
void drawCircleXY(float cx, float cy, float cr, float z = 0);
void drawArrowXY(float sx, float sy, float ex, float ey, float w, float z = 0);


#endif // AUXDRAW_H
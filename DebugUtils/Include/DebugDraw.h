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

#ifndef DEBUGDRAW_H
#define DEBUGDRAW_H

#include <cstdint>

// Some math headers don't have PI defined.
constexpr float DU_PI = 3.14159265f;

enum duDebugDrawPrimitives
{
	DU_DRAW_POINTS,
	DU_DRAW_LINES,
	DU_DRAW_TRIS,
	DU_DRAW_QUADS,
};

/// Abstract debug draw interface.
struct duDebugDraw
{
	virtual ~duDebugDraw() = 0;

	virtual void depthMask(bool state) = 0;

	virtual void texture(bool state) = 0;

	/// Begin drawing primitives.
	///  @param prim [in] primitive type to draw, one of rcDebugDrawPrimitives.
	///  @param size [in] size of a primitive, applies to point size and line width only.
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.f) = 0;

	/// Submit a vertex
	///  @param pos [in] position of the verts.
	///  @param color [in] color of the verts.
	virtual void vertex(const float* pos, uint32_t color) = 0;

	/// Submit a vertex
	///  @param x,y,z [in] position of the verts.
	///  @param color [in] color of the verts.
	virtual void vertex(const float x, const float y, const float z, uint32_t color) = 0;

	/// Submit a vertex
	///  @param pos [in] position of the verts.
	///  @param color [in] color of the verts.
	virtual void vertex(const float* pos, uint32_t color, const float* uv) = 0;

	/// Submit a vertex
	///  @param x,y,z [in] position of the verts.
	///  @param color [in] color of the verts.
	virtual void vertex(const float x, const float y, const float z, uint32_t color, const float u, const float v) = 0;

	/// End drawing primitives.
	virtual void end() = 0;

	/// Compute a color for given area.
	virtual uint32_t areaToCol(uint32_t area);
};

inline constexpr uint32_t duRGBA(int r, int g, int b, int a)
{
	return ((uint32_t)r) | ((uint32_t)g << 8) | ((uint32_t)b << 16) | ((uint32_t)a << 24);
}

inline constexpr uint32_t duRGBAf(float fr, float fg, float fb, float fa)
{
	uint8_t r = (uint8_t)(fr * 255.f);
	uint8_t g = (uint8_t)(fg * 255.f);
	uint8_t b = (uint8_t)(fb * 255.f);
	uint8_t a = (uint8_t)(fa * 255.f);
	return duRGBA(r, g, b, a);
}

uint32_t duIntToCol(int i, int a);
void duIntToCol(int i, float* col);

inline uint32_t duMultCol(const uint32_t col, const uint32_t d)
{
	const uint32_t r = col & 0xff;
	const uint32_t g = (col >> 8) & 0xff;
	const uint32_t b = (col >> 16) & 0xff;
	const uint32_t a = (col >> 24) & 0xff;
	return duRGBA((r * d) >> 8, (g * d) >> 8, (b * d) >> 8, a);
}

inline constexpr uint32_t duDarkenCol(uint32_t col)
{
	return ((col >> 1) & 0x007f7f7f) | (col & 0xff000000);
}

inline constexpr uint32_t duLerpCol(uint32_t ca, uint32_t cb, uint32_t u)
{
	const uint32_t ra = ca & 0xff;
	const uint32_t ga = (ca >> 8) & 0xff;
	const uint32_t ba = (ca >> 16) & 0xff;
	const uint32_t aa = (ca >> 24) & 0xff;
	const uint32_t rb = cb & 0xff;
	const uint32_t gb = (cb >> 8) & 0xff;
	const uint32_t bb = (cb >> 16) & 0xff;
	const uint32_t ab = (cb >> 24) & 0xff;

	uint32_t r = (ra * (255 - u) + rb * u) / 255;
	uint32_t g = (ga * (255 - u) + gb * u) / 255;
	uint32_t b = (ba * (255 - u) + bb * u) / 255;
	uint32_t a = (aa * (255 - u) + ab * u) / 255;
	return duRGBA(r, g, b, a);
}

inline constexpr uint32_t duTransCol(uint32_t c, uint32_t a)
{
	return (a << 24) | (c & 0x00ffffff);
}

void duCalcBoxColors(uint32_t* colors, uint32_t colTop, uint32_t colSide);

void duDebugDrawCylinderWire(struct duDebugDraw* dd, float minx, float miny, float minz,
	float maxx, float maxy, float maxz, uint32_t col, const float lineWidth);

void duDebugDrawBoxWire(struct duDebugDraw* dd, float minx, float miny, float minz,
	float maxx, float maxy, float maxz, uint32_t col, const float lineWidth);

void duDebugDrawArc(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
	const float x1, const float y1, const float z1, const float h,
	const float as0, const float as1, uint32_t col, const float lineWidth);

void duDebugDrawArrow(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
	const float x1, const float y1, const float z1,
	const float as0, const float as1, uint32_t col, const float lineWidth);

void duDebugDrawCircle(struct duDebugDraw* dd, const float x, const float y, const float z,
	const float r, uint32_t col, const float lineWidth);

void duDebugDrawCross(struct duDebugDraw* dd, const float x, const float y, const float z,
	const float size, uint32_t col, const float lineWidth);

void duDebugDrawBox(struct duDebugDraw* dd, float minx, float miny, float minz,
	float maxx, float maxy, float maxz, const uint32_t* fcol);

void duDebugDrawCylinder(struct duDebugDraw* dd, float minx, float miny, float minz,
	float maxx, float maxy, float maxz, uint32_t col);

void duDebugDrawGridXZ(struct duDebugDraw* dd, const float ox, const float oy, const float oz,
	const int w, const int h, const float size,
	const uint32_t col, const float lineWidth);

// Versions without begin/end, can be used to draw multiple primitives.
void duAppendCylinderWire(struct duDebugDraw* dd, float minx, float miny, float minz,
	float maxx, float maxy, float maxz, uint32_t col);

void duAppendBoxWire(struct duDebugDraw* dd, float minx, float miny, float minz,
	float maxx, float maxy, float maxz, uint32_t col);

void duAppendBoxPoints(struct duDebugDraw* dd, float minx, float miny, float minz,
	float maxx, float maxy, float maxz, uint32_t col);

void duAppendArc(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
	const float x1, const float y1, const float z1, const float h,
	const float as0, const float as1, uint32_t col);

void duAppendArrow(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
	const float x1, const float y1, const float z1,
	const float as0, const float as1, uint32_t col);

void duAppendCircle(struct duDebugDraw* dd, const float x, const float y, const float z,
	const float r, uint32_t col);

void duAppendCross(struct duDebugDraw* dd, const float x, const float y, const float z,
	const float size, uint32_t col);

void duAppendBox(struct duDebugDraw* dd, float minx, float miny, float minz,
	float maxx, float maxy, float maxz, const uint32_t* fcol);

void duAppendCylinder(struct duDebugDraw* dd, float minx, float miny, float minz,
	float maxx, float maxy, float maxz, uint32_t col);

class duDisplayList : public duDebugDraw
{
	float* m_pos;
	uint32_t* m_color;
	int m_size;
	int m_cap;

	bool m_depthMask;
	duDebugDrawPrimitives m_prim;
	float m_primSize;

	void resize(int cap);

public:
	duDisplayList(int cap = 512);
	~duDisplayList();
	virtual void depthMask(bool state);
	virtual void begin(duDebugDrawPrimitives prim, float size = 1.f);
	virtual void vertex(const float x, const float y, const float z, uint32_t color);
	virtual void vertex(const float* pos, uint32_t color);
	virtual void end();
	void clear();
	void draw(struct duDebugDraw* dd);
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	duDisplayList(const duDisplayList&);
	duDisplayList& operator=(const duDisplayList&);
};

#endif // DEBUGDRAW_H

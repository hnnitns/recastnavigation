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
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "Recast.h"
#include "RecastAlloc.h"
#include "RecastAssert.h"


static int getCornerHeight(int x, int y, int i, int dir,
						   const rcCompactHeightfield& chf)
{
	const rcCompactSpan& s = chf.spans[i];
	int ch = (int)s.y;
	int dirp = (dir+1) & 0x3;
	
	// Combine region and area codes in order to prevent
	// border vertices which are in between two areas to be removed. 
	if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
		const rcCompactSpan& as = chf.spans[ai];
		ch = rcMax(ch, (int)as.y);
		if (rcGetCon(as, dirp) != RC_NOT_CONNECTED)
		{
			const int ax2 = ax + rcGetDirOffsetX(dirp);
			const int ay2 = ay + rcGetDirOffsetY(dirp);
			const int ai2 = (int)chf.cells[ax2+ay2*chf.width].index + rcGetCon(as, dirp);
			const rcCompactSpan& as2 = chf.spans[ai2];
			ch = rcMax(ch, (int)as2.y);
		}
	}
	if (rcGetCon(s, dirp) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dirp);
		const int ay = y + rcGetDirOffsetY(dirp);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dirp);
		const rcCompactSpan& as = chf.spans[ai];
		ch = rcMax(ch, (int)as.y);
		if (rcGetCon(as, dir) != RC_NOT_CONNECTED)
		{
			const int ax2 = ax + rcGetDirOffsetX(dir);
			const int ay2 = ay + rcGetDirOffsetY(dir);
			const int ai2 = (int)chf.cells[ax2+ay2*chf.width].index + rcGetCon(as, dir);
			const rcCompactSpan& as2 = chf.spans[ai2];
			ch = rcMax(ch, (int)as2.y);
		}
	}
	
	return ch;
}

static void walkContour(int x, int y, int i,
						rcCompactHeightfield& chf,
						unsigned char* flags, rcIntArray& points)
{
	// Choose the first non-connected edge
	unsigned char dir = 0;
	while ((flags[i] & (1 << dir)) == 0)
		dir++;
	
	unsigned char startDir = dir;
	int starti = i;
	
	int iter = 0;
	while (++iter < 40000)
	{
		if (flags[i] & (1 << dir))
		{
			// Choose the edge corner
			int px = x;
			int py = getCornerHeight(x, y, i, dir, chf);
			int pz = y;
			switch(dir)
			{
				case 0: pz++; break;
				case 1: px++; pz++; break;
				case 2: px++; break;
			}
			
			points.push(px);
			points.push(py);
			points.push(pz);
			points.push(0);
			
			flags[i] &= ~(1 << dir); // Remove visited edges
			dir = (dir+1) & 0x3;  // Rotate CW
		}
		else
		{
			int ni = -1;
			const int nx = x + rcGetDirOffsetX(dir);
			const int ny = y + rcGetDirOffsetY(dir);
			const rcCompactSpan& s = chf.spans[i];
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const rcCompactCell& nc = chf.cells[nx+ny*chf.width];
				ni = (int)nc.index + rcGetCon(s, dir);
			}
			if (ni == -1)
			{
				// Should not happen.
				return;
			}
			x = nx;
			y = ny;
			i = ni;
			dir = (dir+3) & 0x3;	// Rotate CCW
		}
		
		if (starti == i && startDir == dir)
		{
			break;
		}
	}
}

static float distancePtSeg(const int x, const int y, const int z,
						   const int px, const int py, const int pz,
						   const int qx, const int qy, const int qz)
{
	float pqx = (float)(qx - px);
	float pqy = (float)(qy - py);
	float pqz = (float)(qz - pz);
	float dx = (float)(x - px);
	float dy = (float)(y - py);
	float dz = (float)(z - pz);
	float d = pqx*pqx + pqy*pqy + pqz*pqz;
	float t = pqx*dx + pqy*dy + pqz*dz;
	if (d > 0)
	t /= d;
	if (t < 0)
	t = 0;
	else if (t > 1)
	t = 1;

	dx = px + t*pqx - x;
	dy = py + t*pqy - y;
	dz = pz + t*pqz - z;

	return dx*dx + dy*dy + dz*dz;
	
/*	float pqx = (float)(qx - px);
	float pqz = (float)(qz - pz);
	float dx = (float)(x - px);
	float dz = (float)(z - pz);
	float d = pqx*pqx + pqz*pqz;
	float t = pqx*dx + pqz*dz;
	if (d > 0)
		t /= d;
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	
	dx = px + t*pqx - x;
	dz = pz + t*pqz - z;
	
	return dx*dx + dz*dz;*/
}

static void simplifyContour(rcIntArray& points, rcIntArray& simplified, const float maxError)
{
	// If there is no connections at all,
	// create some initial points for the simplification process. 
	// Find lower-left and upper-right vertices of the contour.
	int llx = points[0];
	int lly = points[1];
	int llz = points[2];
	int lli = 0;
	int urx = points[0];
	int ury = points[1];
	int urz = points[2];
	int uri = 0;
	for (int i = 0; i < points.size(); i += 4)
	{
		int x = points[i+0];
		int y = points[i+1];
		int z = points[i+2];
		if (x < llx || (x == llx && z < llz))
		{
			llx = x;
			lly = y;
			llz = z;
			lli = i/4;
		}
		if (x > urx || (x == urx && z > urz))
		{
			urx = x;
			ury = y;
			urz = z;
			uri = i/4;
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

	
	// Add points until all raw points are within
	// error tolerance to the simplified shape.
	const int pn = points.size()/4;
	for (int i = 0; i < simplified.size()/4; )
	{
		int ii = (i+1) % (simplified.size()/4);
		
		const int ax = simplified[i*4+0];
		const int ay = simplified[i*4+1];
		const int az = simplified[i*4+2];
		const int ai = simplified[i*4+3];
		
		const int bx = simplified[ii*4+0];
		const int by = simplified[ii*4+1];
		const int bz = simplified[ii*4+2];
		const int bi = simplified[ii*4+3];
		
		// Find maximum deviation from the segment.
		float maxd = 0;
		int maxi = -1;
		int ci, cinc, endi;
		
		// Traverse the segment in lexilogical order so that the
		// max deviation is calculated similarly when traversing
		// opposite segments.
		if (bx > ax || (bx == ax && bz > az))
		{
			cinc = 1;
			ci = (ai+cinc) % pn;
			endi = bi;
		}
		else
		{
			cinc = pn-1;
			ci = (bi+cinc) % pn;
			endi = ai;
		}
		
		while (ci != endi)
		{
			static const int s = 6;
			float d = distancePtSeg(points[ci*4+0], points[ci*4+1]/s, points[ci*4+2],
									ax,ay/s,az, bx,by/s,bz);
			if (d > maxd)
			{
				maxd = d;
				maxi = ci;
			}
			ci = (ci+cinc) % pn;
		}
		
		
		// If the max deviation is larger than accepted error,
		// add new point, else continue to next segment.
		if (maxi != -1 && maxd > (maxError*maxError))
		{
			// Add space for the new point.
			simplified.resize(simplified.size()+4);
			const int n = simplified.size()/4;
			for (int j = n-1; j > i; --j)
			{
				simplified[j*4+0] = simplified[(j-1)*4+0];
				simplified[j*4+1] = simplified[(j-1)*4+1];
				simplified[j*4+2] = simplified[(j-1)*4+2];
				simplified[j*4+3] = simplified[(j-1)*4+3];
			}
			// Add the point.
			simplified[(i+1)*4+0] = points[maxi*4+0];
			simplified[(i+1)*4+1] = points[maxi*4+1];
			simplified[(i+1)*4+2] = points[maxi*4+2];
			simplified[(i+1)*4+3] = maxi;
		}
		else
		{
			++i;
		}
	}
	
}

static void removeDegenerateSegments(rcIntArray& simplified)
{
	// Remove adjacent vertices which are equal on xz-plane,
	// or else the triangulator will get confused.
	for (int i = 0; i < simplified.size()/4; ++i)
	{
		int ni = i+1;
		if (ni >= (simplified.size()/4))
			ni = 0;
		
		if (simplified[i*4+0] == simplified[ni*4+0] &&
			simplified[i*4+2] == simplified[ni*4+2])
		{
			// Degenerate segment, remove.
			for (int j = i; j < simplified.size()/4-1; ++j)
			{
				simplified[j*4+0] = simplified[(j+1)*4+0];
				simplified[j*4+1] = simplified[(j+1)*4+1];
				simplified[j*4+2] = simplified[(j+1)*4+2];
				simplified[j*4+3] = simplified[(j+1)*4+3];
			}
			simplified.resize(simplified.size()-4);
		}
	}
}

bool buildContours(rcContext* ctx, rcCompactHeightfield& chf,
				   const float maxError,
				   rcContourSet& cset)
{
	rcAssert(ctx);
	
	const int w = chf.width;
	const int h = chf.height;
	
	ctx->startTimer(RC_TIMER_BUILD_CONTOURS);
	
	rcVcopy(cset.bmin, chf.bmin);
	rcVcopy(cset.bmax, chf.bmax);
	cset.cs = chf.cs;
	cset.ch = chf.ch;
	
	int maxContours = 256; // TODO: fix!
	cset.conts = (rcContour*)rcAlloc(sizeof(rcContour)*maxContours, RC_ALLOC_PERM);
	if (!cset.conts)
		return false;
	cset.nconts = 0;
	
	rcScopedDelete<unsigned char> flags = (unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_TEMP);
	if (!flags)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'flags' (%d).", chf.spanCount);
		return false;
	}
	
	ctx->startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
	
	// Mark boundaries.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				unsigned char res = 0;
				const rcCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA)
				{
					flags[i] = 0;
					continue;
				}
				for (int dir = 0; dir < 4; ++dir)
				{
					unsigned char area = 0;
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						area = chf.areas[ai];
					}
					if (area != RC_NULL_AREA)
						res |= (1 << dir);
				}
				flags[i] = res ^ 0xf; // Inverse, mark non connected edges.
			}
		}
	}
	
	ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
	
	ctx->startTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
	
	rcIntArray verts(256);
	rcIntArray simplified(64);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const rcCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (flags[i] == 0 || flags[i] == 0xf)
				{
					flags[i] = 0;
					continue;
				}
				const unsigned char area = chf.areas[i];
				if (area == RC_NULL_AREA)
					continue;
				
				verts.resize(0);
				simplified.resize(0);
				walkContour(x, y, i, chf, flags, verts);
				simplifyContour(verts, simplified, maxError);
				removeDegenerateSegments(simplified);
				
				// Store region->contour remap info.
				// Create contour.
				if (simplified.size()/4 >= 3)
				{
					if (cset.nconts >= maxContours)
					{
						// Allocate more contours.
						// This can happen when there are tiny holes in the heightfield.
						const int oldMax = maxContours;
						maxContours *= 2;
						rcContour* newConts = (rcContour*)rcAlloc(sizeof(rcContour)*maxContours, RC_ALLOC_PERM);
						for (int j = 0; j < cset.nconts; ++j)
						{
							newConts[j] = cset.conts[j];
							// Reset source pointers to prevent data deletion.
							cset.conts[j].verts = 0;
							cset.conts[j].rverts = 0;
						}
						rcFree(cset.conts);
						cset.conts = newConts;
						
						ctx->log(RC_LOG_WARNING, "rcBuildContours: Expanding max contours from %d to %d.", oldMax, maxContours);
					}
					
					rcContour* cont = &cset.conts[cset.nconts++];
					
					cont->nverts = simplified.size()/4;
					cont->verts = (int*)rcAlloc(sizeof(int)*cont->nverts*4, RC_ALLOC_PERM);
					if (!cont->verts)
					{
						ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'verts' (%d).", cont->nverts);
						return false;
					}
					memcpy(cont->verts, &simplified[0], sizeof(int)*cont->nverts*4);
					
					cont->nrverts = verts.size()/4;
					cont->rverts = (int*)rcAlloc(sizeof(int)*cont->nrverts*4, RC_ALLOC_PERM);
					if (!cont->rverts)
					{
						ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'rverts' (%d).", cont->nrverts);
						return false;
					}
					memcpy(cont->rverts, &verts[0], sizeof(int)*cont->nrverts*4);
					
					/*					cont->cx = cont->cy = cont->cz = 0;
					 for (int i = 0; i < cont->nverts; ++i)
					 {
					 cont->cx += cont->verts[i*4+0];
					 cont->cy += cont->verts[i*4+1];
					 cont->cz += cont->verts[i*4+2];
					 }
					 cont->cx /= cont->nverts;
					 cont->cy /= cont->nverts;
					 cont->cz /= cont->nverts;*/
					
					cont->reg = 0;
					cont->area = 0;
				}
			}
		}
	}
	
	ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
	
	ctx->stopTimer(RC_TIMER_BUILD_CONTOURS);
	
	return true;
}

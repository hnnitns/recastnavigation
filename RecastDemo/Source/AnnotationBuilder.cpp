
#include <string.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>
#include "AnnotationBuilder.h"
#include "NavWorld.h"
#include "SampleInterfaces.h"
#include "RecastDebugDraw.h"
#include "BuildContours.h"
#include "vecmath.h"


void drawBoxWire(struct duDebugDraw* dd,
				 const float* c, const float* ax, const float* ay, const float* az,
				 const float* bmin, const float* bmax,
				 const float lineWidth, unsigned int col)
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

	dd->begin(DU_DRAW_LINES, lineWidth);
	for (int i = 0; i < 4*3*2; ++i)
		dd->vertex(&corners[inds[i]*3], col);
	dd->end();
}

static void evalArc(float* pt, const float* sp, const float* sq, const float u, const float h)
{
	vlerp(pt, sp,sq, u);
	const float u2 = u*u*u; //1 - sqr(1-u);
	pt[1] = sp[1] + (sq[1] - sp[1]) * u2;
}

static void trans2d(float* dst, const float* ax, const float* ay, const float* pt)
{
	dst[0] = ax[0]*pt[0] + ay[0]*pt[1];
	dst[1] = ax[1]*pt[0] + ay[1]*pt[1];
	dst[2] = ax[2]*pt[0] + ay[2]*pt[1];
}

static void trans2dc(float* dst, const float* c, const float* ax, const float* ay, const float* pt)
{
	dst[0] = c[0] + ax[0]*pt[0] + ay[0]*pt[1];
	dst[1] = c[1] + ax[1]*pt[0] + ay[1]*pt[1];
	dst[2] = c[2] + ax[2]*pt[0] + ay[2]*pt[1];
}


AnnotationBuilder::AnnotationBuilder() :
	m_triareas(0),
	m_solid(0),
	m_chf(0),
	m_cset(0),
	m_edges(0),
	m_selEdge(-1),
	m_sampler(0),
	m_links(0),
	m_nlinks(0),
	m_clinks(0)
{
}

AnnotationBuilder::~AnnotationBuilder()
{
	cleanup();
}

void AnnotationBuilder::clearLinks()
{
	delete m_links;
	m_links = 0;
	m_nlinks = 0;
	m_clinks = 0;
}

void AnnotationBuilder::cleanup()
{
	delete [] m_triareas;
	m_triareas = 0;
	rcFreeHeightField(m_solid);
	m_solid = 0;
	rcFreeCompactHeightfield(m_chf);
	m_chf = 0;
	rcFreeContourSet(m_cset);
	m_cset = 0;
	delete [] m_edges;
	m_edges = 0;
	m_nedges = 0;
	delete m_sampler;
	m_sampler = 0;

	delete m_links;
	m_links = 0;
	m_nlinks = 0;
	m_clinks = 0;
}

bool AnnotationBuilder::build(const AnnotationBuilderConfig& acfg, class NavWorld* world)
{
	if (world->getMeshCount() < 1)
		return false;

	cleanup();

	m_acfg = acfg;

	BuildContext ctx;
	float bmin[3], bmax[3];
	world->getWorldBounds(bmin,bmax);
	const int maxMeshTris = world->getMaxMeshTriCount();

	// Init build configuration from GUI
	// GUIからビルド構成を初期化します
	memset(&m_cfg, 0, sizeof(m_cfg));
	m_cfg.cs = m_acfg.cellSize;
	m_cfg.ch = m_acfg.cellHeight;
	m_cfg.walkableSlopeAngle = 45.0f;
	m_cfg.walkableHeight = (int)ceilf(m_acfg.agentHeight / m_cfg.ch);
	m_cfg.walkableClimb = (int)floorf(m_acfg.agentClimb / m_cfg.ch);
	m_cfg.walkableRadius = (int)ceilf(m_acfg.agentRadius / m_cfg.cs);
	m_cfg.maxEdgeLen = (int)(12.0f / m_cfg.cs);
	m_cfg.maxSimplificationError = 1.3f;
	m_cfg.minRegionArea = (int)rcSqr(8);		// Note: area = size*size
	m_cfg.mergeRegionArea = (int)rcSqr(20);	// Note: area = size*size
	m_cfg.maxVertsPerPoly = (int)6;
	m_cfg.detailSampleDist = 6.0f * m_cfg.cs;
	m_cfg.detailSampleMaxError = 1.0 * m_cfg.ch;

	// Set the area where the navigation will be build.
	// Here the bounds of the input mesh are used, but the
	// area could be specified by an user defined box, etc.
	// ナビゲーションが構築される領域を設定します。
	// ここでは、入力メッシュの境界が使用されますが、領域はユーザー定義のボックスなどで指定できます。
	rcVcopy(m_cfg.bmin, bmin);
	rcVcopy(m_cfg.bmax, bmax);
	rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);

	// Reset build times gathering.
	// ビルド時間の収集をリセットします。
	ctx.resetTimers();

	// Start the build process.
	// ビルドプロセスを開始します。
	ctx.startTimer(RC_TIMER_TOTAL);

	ctx.log(RC_LOG_PROGRESS, "Building navigation:");
	ctx.log(RC_LOG_PROGRESS, " - %d x %d cells", m_cfg.width, m_cfg.height);

	//
	// Step 2. Rasterize input polygon soup.
	// ステップ2. 入力ポリゴンスープをラスタライズします。
	//

	// Allocate voxel heightfield where we rasterize our input data to.
	// 入力データをラスタライズするボクセルの高さフィールドを割り当てます。
	m_solid = rcAllocHeightfield();
	if (!m_solid)
	{
		ctx.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
		return false;
	}
	if (!rcCreateHeightfield(&ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
	{
		ctx.log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
		return false;
	}

	// Allocate array that can hold triangle area types.
	// If you have multiple meshes you need to process, allocate
	// and array which can hold the max number of triangles you need to process.
	// 三角形の領域タイプを保持できる配列を割り当てます。
	// 処理する必要のあるメッシュが複数ある場合は、
	// 処理する必要のある三角形の最大数を保持できる配列を割り当て、配列します。
	m_triareas = new unsigned char[maxMeshTris];
	if (!m_triareas)
	{
		ctx.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'm_triareas' (%d).", maxMeshTris);
		return false;
	}

	// Find triangles which are walkable based on their slope and rasterize them.
	// If your input data is multiple meshes, you can transform them here, calculate
	// the are type for each of the meshes and rasterize them.
	// 傾斜に基づいて歩行可能な三角形を見つけ、ラスタライズします。
	// 入力データが複数のメッシュである場合は、ここでそれらを変換し、
	// 各メッシュのareタイプを計算して、ラスタライズできます。

	for (int i = 0; i < world->getMeshCount(); ++i)
	{
		const GeomMesh* m = world->getMesh(i);
		memset(m_triareas, 0, maxMeshTris*sizeof(unsigned char));
		rcMarkWalkableTriangles(&ctx, m_cfg.walkableSlopeAngle, m->verts, m->nverts, m->tris, m->ntris, m_triareas);
		rcRasterizeTriangles(&ctx, m->verts, m->nverts, (uint16_t*)m->tris, m_triareas, m->ntris, *m_solid, m_cfg.walkableClimb);
	}

	//
	// Step 3. Filter walkables surfaces.
	// ステップ3. 歩行可能なサーフェスをフィルタリングします。
	//

	// Once all geoemtry is rasterized, we do initial pass of filtering to
	// remove unwanted overhangs caused by the conservative rasterization
	// as well as filter spans where the character cannot possibly stand.
	// すべてのジオエムトリーがラスタライズされたら、フィルタリングの最初のパスを実行して、
	// 保守的なラスタライズによって引き起こされた不要なオーバーハングと、
	// キャラクターが立つことができない可能性のあるフィルタースパンを削除します。
	rcFilterLowHangingWalkableObstacles(&ctx, m_cfg.walkableClimb, *m_solid);
	rcFilterLedgeSpans(&ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
	rcFilterWalkableLowHeightSpans(&ctx, m_cfg.walkableHeight, *m_solid);


	m_chf = rcAllocCompactHeightfield();
	if (!m_chf)
	{
		ctx.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
		return false;
	}
	if (!rcBuildCompactHeightfield(&ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
	{
		ctx.log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
		return false;
	}

	// Erode the walkable area by agent radius.
	// エージェントの半径で歩行可能エリアを侵食します。
	if (!rcErodeWalkableArea(&ctx, m_cfg.walkableRadius, *m_chf))
	{
		ctx.log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
		return false;
	}


	// Create contours.
	// 輪郭を作成します。
	m_cset = rcAllocContourSet();
	if (!m_cset)
	{
		ctx.log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
		return false;
	}
	if (!buildContours(&ctx, *m_chf, m_cfg.maxSimplificationError, *m_cset))
	{
		ctx.log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
		return false;
	}

	// Build edges.
	// エッジを構築します。
	m_nedges = 0;
	for (int i = 0; i < m_cset->nconts; ++i)
		m_nedges += m_cset->conts[i].nverts;

	if (!m_nedges)
	{
		ctx.log(RC_LOG_ERROR, "buildNavigation: No edges!");
		return false;
	}

	m_edges = new Edge[m_nedges];
	if (!m_edges)
	{
		ctx.log(RC_LOG_ERROR, "buildNavigation: Failed to alloc edges (%d)", m_nedges);
		return false;
	}

	const float* orig = m_cset->bmin;
	const float cs = m_cset->cs;
	const float ch = m_cset->ch;

	m_nedges = 0;
	for (int i = 0; i < m_cset->nconts; ++i)
	{
		const rcContour& c = m_cset->conts[i];
		if (!c.nverts)
			continue;

		for (int j = 0, k = c.nverts-1; j < c.nverts; k=j++)
		{
			//int k = (j+1) % c.nverts;
			const int* vj = &c.verts[j*4];
			const int* vk = &c.verts[k*4];

			Edge* e = &m_edges[m_nedges++];

			e->sp[0] = orig[0] + vk[0]*cs;
			e->sp[1] = orig[1] + (vk[1]+2)*ch;
			e->sp[2] = orig[2] + vk[2]*cs;

			e->sq[0] = orig[0] + vj[0]*cs;
			e->sq[1] = orig[1] + (vj[1]+2)*ch;
			e->sq[2] = orig[2] + vj[2]*cs;
		}
	}

	m_selEdge = -1;

	ctx.dumpLog("Log");

	return true;
}

AnnotationBuilder::JumpLink* AnnotationBuilder::addLink()
{
	if (m_nlinks+1 > m_clinks)
	{
		m_clinks = m_clinks ? m_clinks*2 : 8;
		JumpLink* n = new JumpLink[m_clinks];
		if (m_nlinks)
			memcpy(n, m_links, sizeof(JumpLink)*m_nlinks);
		delete [] m_links;
		m_links = n;
	}
	JumpLink* link = &m_links[m_nlinks++];
	return link;
}

static void insertSort(unsigned char* a, const int n)
{
	int i, j;
	for (i = 1; i < n; i++)
	{
		const unsigned char value = a[i];
		for (j = i - 1; j >= 0 && a[j] > value; j--)
			a[j+1] = a[j];
		a[j+1] = value;
	}
}
void AnnotationBuilder::addEdgeLinks(EdgeSampler* es)
{
	if (es->start.ngsamples != es->end.ngsamples)
		return;

	const int nsamples = es->start.ngsamples;

	// Filter small holes.
	const int RAD = 2;
	unsigned char kernel[RAD*2+1];

	unsigned char* nflags = new unsigned char[nsamples];

	for (int i = 0; i < nsamples; ++i)
	{
		const int a = max(0, i-RAD);
		const int b = min(nsamples-1, i+RAD);
		int nkernel = 0;
		for (int j = a; j <= b; ++j)
			kernel[nkernel++] = es->start.gsamples[i].flags & 4;
		insertSort(kernel, nkernel);
		nflags[i] = kernel[(nkernel+1)/2];
	}

	// Build segments
	int start = -1;
	for (int i = 0; i <= nsamples; ++i)
	{
		bool valid = i < nsamples && nflags[i] != 0;
		if (start == -1)
		{
			if (valid)
				start = i;
		}
		else
		{
			if (!valid)
			{
				if (i - start > 5)
				{
					const float u0 = (float)start/(float)nsamples;
					const float u1 = (float)i/(float)nsamples;

					float sp[3], sq[3], ep[3], eq[3];

					vlerp(sp, es->start.p,es->start.q, u0);
					vlerp(sq, es->start.p,es->start.q, u1);
					vlerp(ep, es->end.p,es->end.q, u0);
					vlerp(eq, es->end.p,es->end.q, u1);
					sp[1] = es->start.gsamples[start].height;
					sq[1] = es->start.gsamples[i-1].height;
					ep[1] = es->end.gsamples[start].height;
					eq[1] = es->end.gsamples[i-1].height;

					JumpLink* link = addLink();

					link->flags = 1;
					link->nspine = es->trajectory.nspine;

					const float startx = es->trajectory.spine[0];
					const float endx = es->trajectory.spine[(es->trajectory.nspine-1)*2];
					const float deltax = endx - startx;

					const float starty = es->trajectory.spine[1];
					const float endy = es->trajectory.spine[(es->trajectory.nspine-1)*2+1];

					// Build start spine.
					for (int j = 0; j < es->trajectory.nspine; ++j)
					{
						const float* spt = &es->trajectory.spine[j*2];
						const float u = (spt[0] - startx)/deltax;
						const float dy = spt[1] - lerp(starty,endy,u) + m_acfg.agentClimb;
						float* p = &link->spine0[j*3];
						vlerp(p, sp,ep,u);
						vmad(p, p, es->ay, dy);
					}

					for (int j = 0; j < es->trajectory.nspine; ++j)
					{
						const float* spt = &es->trajectory.spine[j*2];
						const float u = (spt[0] - startx)/deltax;
						const float dy = spt[1] - lerp(starty,endy,u) + m_acfg.agentClimb;
						float* p = &link->spine1[j*3];
						vlerp(p, sq,eq,u);
						vmad(p, p, es->ay, dy);
					}
				}

				start = -1;
			}
		}
	}

	delete [] nflags;
}

void AnnotationBuilder::filterJumpOverLinks()
{
	// Filter out links which overlap
	const float thr = m_chf->cs*5.0f;

	for (int i = 0; i < m_nlinks-1; ++i)
	{
		JumpLink* li = &m_links[i];
		if (li->flags == 0)
			continue;
		const float* spi = &li->spine0[0];
		const float* sqi = &li->spine1[0];
		const float* epi = &li->spine0[(li->nspine-1)*3];
		const float* eqi = &li->spine1[(li->nspine-1)*3];

		for (int j = i+1; j < m_nlinks; ++j)
		{
			JumpLink* lj = &m_links[j];
			if (lj->flags == 0)
				continue;
			const float* spj = &lj->spine0[0];
			const float* sqj = &lj->spine1[0];
			const float* epj = &lj->spine0[(lj->nspine-1)*3];
			const float* eqj = &lj->spine1[(lj->nspine-1)*3];

			const float d0 = distSegSegSqr(spi,sqi, epj,eqj);
			const float d1 = distSegSegSqr(epi,eqi, spj,sqj);
//			const float d0 = min(distSegSegSqr(spi,sqi, spj,sqj), distSegSegSqr(spi,sqi, epj,eqj));
//			const float d1 = min(distSegSegSqr(epi,eqi, epj,eqj), distSegSegSqr(epi,eqi, spj,sqj));

//			printf("%d/%d  d0=%f d1=%f (%f)\n", i,j, sqrtf(d0), sqrtf(d1), thr);

			if (d0 < sqr(thr) && d1 < sqr(thr))
			{
//				printf(" - merge!\n");
				if (vdistSqr(spi,sqi) > vdistSqr(spj,sqj))
				{
					lj->flags = 0;
				}
				else
				{
					li->flags = 0;
					break;
				}
			}
		}
	}

}

void AnnotationBuilder::buildAllEdges(int type)
{
	for (int i = 0; i < m_nedges; ++i)
	{
		delete m_sampler;
		m_sampler = sampleEdge(type, m_edges[i].sp, m_edges[i].sq);
		if (m_sampler)
			addEdgeLinks(m_sampler);
	}

	filterJumpOverLinks();
}

bool AnnotationBuilder::getFloorPosition(const float* pt, const float hrange, float* height)
{
	const int ix = (int)floorf((pt[0] - m_chf->bmin[0])/m_chf->cs);
	const int iz = (int)floorf((pt[2] - m_chf->bmin[2])/m_chf->cs);

	if (ix < 0 || iz < 0 || ix >= m_chf->width || iz >= m_chf->height)
		return false;

	float bestDist = FLT_MAX;
	float bestHeight = FLT_MAX;
	bool found = false;

	const rcCompactCell& c = m_chf->cells[ix+iz*m_chf->width];
	for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
	{
		const rcCompactSpan& s = m_chf->spans[i];
		if (m_chf->areas[i] == RC_NULL_AREA)
			continue;
		const float y = m_chf->bmin[1] + s.y * m_chf->ch;
		const float dist = abs(y - pt[1]);
		if (dist < hrange && dist < bestDist)
		{
			bestDist = dist;
			bestHeight = y;
			found = true;
		}
	}

	*height = bestHeight;

	return found;
}

bool AnnotationBuilder::getCompactHeightfieldHeigh(const float* pt, const float hrange, float* height)
{
	const float range = m_chf->cs;
	const int ix0 = clamp((int)floorf((pt[0]-range - m_chf->bmin[0])/m_chf->cs), 0, m_chf->width-1);
	const int iz0 = clamp((int)floorf((pt[2]-range - m_chf->bmin[2])/m_chf->cs), 0, m_chf->height-1);
	const int ix1 = clamp((int)floorf((pt[0]+range - m_chf->bmin[0])/m_chf->cs), 0, m_chf->width-1);
	const int iz1 = clamp((int)floorf((pt[2]+range - m_chf->bmin[2])/m_chf->cs), 0, m_chf->height-1);

	float bestDist = FLT_MAX;
	float bestHeight = FLT_MAX;
	bool found = false;

	for (int z = iz0; z <= iz1; ++z)
	{
		for (int x = ix0; x <= ix1; ++x)
		{
			const rcCompactCell& c = m_chf->cells[x+z*m_chf->width];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const rcCompactSpan& s = m_chf->spans[i];
				if (m_chf->areas[i] == RC_NULL_AREA)
					continue;
				const float y = m_chf->bmin[1] + s.y * m_chf->ch;
				const float dist = abs(y - pt[1]);
				if (dist < hrange && dist < bestDist)
				{
					bestDist = dist;
					bestHeight = y;
					found = true;
				}
			}
		}
	}

	if (found)
		*height = bestHeight;
	else
		*height = pt[1];

	return found;
}

inline bool overlapRange(const float amin, const float amax, const float bmin, const float bmax)
{
	return (amin > bmax || amax < bmin) ? false : true;
}

bool AnnotationBuilder::checkHeightfieldCollision(const float x, const float ymin, const float ymax, const float z)
{
	const int w = m_solid->width;
	const int h = m_solid->height;
	const float cs = m_solid->cs;
	const float ch = m_solid->ch;
	const float* orig = m_solid->bmin;
	const int ix = (int)floorf((x - orig[0])/cs);
	const int iz = (int)floorf((z - orig[2])/cs);

	if (ix < 0 || iz < 0 || ix > w || iz > h)
		return false;

	const rcSpan* s = m_solid->spans[ix + iz*w];
	if (!s) return false;

	while (s)
	{
		const float symin = orig[1] + s->smin*ch;
		const float symax = orig[1] + s->smax*ch;
		if (overlapRange(ymin, ymax, symin, symax))
			return true;
		s = s->next;
	}

	return false;
}

bool AnnotationBuilder::sampleTrajectory(const float* pa, const float* pb, Trajectory2D* tra)
{
	float p[3];

	for (int i = 0; i < tra->nsamples; ++i)
	{
		const TrajectorySample* s = &tra->samples[i];
		const float u = (float)i / (float)(tra->nsamples-1);
		vlerp(p, pa,pb, u);
		if (checkHeightfieldCollision(p[0], p[1] + s->ymin, p[1] + s->ymax, p[2]))
			return false;
	}

	return true;
}

void AnnotationBuilder::sampleGroundSegment(GroundSegment* seg, const float nsamples, const float groundRange)
{
	float delta[3];
	vsub(delta, seg->p, seg->q);

	seg->ngsamples = nsamples;
	seg->gsamples = new GroundSample[seg->ngsamples];
	seg->npass = 0;

	for (int i = 0; i < seg->ngsamples; ++i)
	{
		const float u = (float)i/(float)(seg->ngsamples-1);
		float pt[3];

		GroundSample* s = &seg->gsamples[i];
		vlerp(pt, seg->p, seg->q, u);
		s->flags = 0;
		if (!getCompactHeightfieldHeigh(pt, groundRange, &s->height))
			continue;
		s->flags |= 1;
		seg->npass++;
	}
}

void AnnotationBuilder::sampleAction(EdgeSampler* es)
{
	if (es->start.ngsamples != es->end.ngsamples)
		return;

	const int nsamples = es->start.ngsamples;

	for (int i = 0; i < nsamples; ++i)
	{
		GroundSample* ssmp = &es->start.gsamples[i];
		GroundSample* esmp = &es->end.gsamples[i];

		if ((ssmp->flags & 1) == 0 || (esmp->flags & 1) == 0)
			continue;

		const float u = (float)i/(float)(nsamples-1);
		float spt[3], ept[3];
		vlerp(spt, es->start.p, es->start.q,u);
		vlerp(ept, es->end.p, es->end.q, u);


		spt[1] = ssmp->height;
		ept[1] = esmp->height;

		if (!sampleTrajectory(spt, ept, &es->trajectory))
			continue;
		ssmp->flags |= 4;
	}
}

static float getHeight(const float x, const float* pts, const int npts)
{
	if (x <= pts[0])
		return pts[1];
	if (x >= pts[(npts-1)*2])
		return pts[(npts-1)*2+1];

	for (int i = 1; i < npts; ++i)
	{
		const float* q = &pts[i*2];
		if (x <= q[0])
		{
			const float* p = &pts[(i-1)*2];
			const float u = (x-p[0]) / (q[0]-p[0]);
			return lerp(p[1],q[1],u);
		}
	}

	return pts[(npts-1)*2+1];
}

void AnnotationBuilder::initTrajectory(Trajectory2D* tra)
{
	const float cs = m_acfg.cellSize;

	const float* pa = &tra->spine[0];
	const float* pb = &tra->spine[(tra->nspine-1)*2];

	const float dx = pb[0] - pa[0];
	tra->nsamples = max(2, (int)ceilf(dx/cs));
	tra->samples = new TrajectorySample[tra->nsamples];

	for (int i = 0; i < tra->nsamples; ++i)
	{
		const float u = (float)i / (float)(tra->nsamples-1);
		TrajectorySample* s = &tra->samples[i];
		s->x = lerp(pa[0], pb[0], u);

		const float y0 = getHeight(s->x-m_acfg.agentRadius, tra->spine, tra->nspine);
		const float y1 = getHeight(s->x+m_acfg.agentRadius, tra->spine, tra->nspine);

		const float y = lerp(pa[1], pb[1], u);

		s->ymin = min(y0,y1) + m_acfg.agentClimb - y;
		s->ymax = max(y0,y1) + m_acfg.agentHeight - y;
	}
}

int AnnotationBuilder::findPotentialJumpOverEdges(const float* sp, const float* sq,
												  const float depthRange, const float heightRange,
												  float* outSegs, const int maxOutSegs)
{
	// Find potential edges to join to.
	const float widthRange = sqrtf(vdistSqr(sp,sq));
	const float amin[3] = {0,-heightRange*0.5f,0}, amax[3] = {widthRange,heightRange*0.5f,depthRange};

	const float thr = cosf((180.0f - 45.0f)/180.0f*3.1415f);

	float ax[3], ay[3], az[3];
	vsub(ax, sq,sp);
	_vnormalize(ax);
	vset(az, ax[2], 0, -ax[0]);
	_vnormalize(az);
	vset(ay, 0,1,0);

	static const int MAX_SEGS = 64;
	PotentialSeg segs[MAX_SEGS];
	int nsegs = 0;

	for (int i = 0; i < m_nedges; ++i)
	{
		float p[3], lsp[3], lsq[3];
		vsub(p, m_edges[i].sp, sp);
		lsp[0] = ax[0]*p[0] + ay[0]*p[1] + az[0]*p[2];
		lsp[1] = ax[1]*p[0] + ay[1]*p[1] + az[1]*p[2];
		lsp[2] = ax[2]*p[0] + ay[2]*p[1] + az[2]*p[2];

		vsub(p, m_edges[i].sq, sp);
		lsq[0] = ax[0]*p[0] + ay[0]*p[1] + az[0]*p[2];
		lsq[1] = ax[1]*p[0] + ay[1]*p[1] + az[1]*p[2];
		lsq[2] = ax[2]*p[0] + ay[2]*p[1] + az[2]*p[2];

		float tmin, tmax;
		if (isectSegAABB(lsp,lsq, amin,amax, tmin,tmax))
		{
			if (tmin > 1.0f)
				continue;
			if (tmax < 0.0f)
				continue;

			float edir[3];
			vsub(edir, m_edges[i].sq, m_edges[i].sp);
			edir[1] = 0;
			_vnormalize(edir);

			if (vdot(ax, edir) > thr)
				continue;

			if (nsegs < MAX_SEGS)
			{
				segs[nsegs].umin = clamp(tmin, 0.0f, 1.0f);
				segs[nsegs].umax = clamp(tmax, 0.0f, 1.0f);
				segs[nsegs].dmin = min(lsp[2],lsq[2]);
				segs[nsegs].dmax = max(lsp[2],lsq[2]);
				segs[nsegs].idx = i;
				segs[nsegs].mark = 0;
				nsegs++;
			}
		}
	}

	const float eps = m_chf->cs;
	unsigned char mark = 1;
	for (int i = 0; i < nsegs; ++i)
	{
		if (segs[i].mark != 0)
			continue;
		segs[i].mark = mark;

		for (int j = i+1; j < nsegs; ++j)
		{
			if (overlapRange(segs[i].dmin-eps, segs[i].dmax+eps,
							 segs[j].dmin-eps, segs[j].dmax+eps))
			{
				segs[j].mark = mark;
			}
		}

		mark++;
	}


	int nout = 0;

	for (int i = 1; i < mark; ++i)
	{
		// Find destination mid point.
		float umin = 10.0f, umax = -10.0;
		float ptmin[3], ptmax[3];

		for (int j = 0; j < nsegs; ++j)
		{
			PotentialSeg* seg = &segs[j];
			if (seg->mark != (unsigned char)i)
				continue;

			float pa[3], pb[3];
			vlerp(pa, m_edges[seg->idx].sp,m_edges[seg->idx].sq, seg->umin);
			vlerp(pb, m_edges[seg->idx].sp,m_edges[seg->idx].sq, seg->umax);

			const float ua = getClosestPtPtSeg(pa, sp, sq);
			const float ub = getClosestPtPtSeg(pb, sp, sq);

			if (ua < umin)
			{
				vcopy(ptmin, pa);
				umin = ua;
			}
			if (ua > umax)
			{
				vcopy(ptmax, pa);
				umax = ua;
			}

			if (ub < umin)
			{
				vcopy(ptmin, pb);
				umin = ub;
			}
			if (ub > umax)
			{
				vcopy(ptmax, pb);
				umax = ub;
			}
		}

		if (umin > umax)
			continue;

		float end[3];
		vlerp(end, ptmin,ptmax,0.5f);

		float start[3];
		vlerp(start, sp,sq,(umin+umax)*0.5f);

		float orig[3];
		vlerp(orig, start,end,0.5f);

		float dir[3], norm[3];
		vsub(dir, end,start);
		dir[1] = 0;
		_vnormalize(dir);
		vset(norm, dir[2],0,-dir[0]);

		float ssp[3], ssq[3];

		const float width = widthRange * (umax-umin);
		vmad(ssp, orig, norm, width*0.5f);
		vmad(ssq, orig, norm, -width*0.5f);

		if (nout < maxOutSegs)
		{
			vcopy(&outSegs[nout*6+0], ssp);
			vcopy(&outSegs[nout*6+3], ssq);
			nout++;
		}

/*		for (int j = 0; j < nsegs; ++j)
		{
			PotentialSeg* seg = &segs[j];
			if (seg->mark != (unsigned char)i)
				continue;
			vcopy(seg->sp, ssp);
			vcopy(seg->sq, ssq);
		}*/
	}

	return nout;
}


void AnnotationBuilder::initJumpDownRig(EdgeSampler* es, const float* sp, const float* sq,
										const float jumpStartDist, const float jumpEndDist,
										const float jumpDownDist, const float groundRange)
{
	vcopy(es->rigp, sp);
	vcopy(es->rigq, sq);

	vsub(es->ax, sq,sp);
	_vnormalize(es->ax);
	vset(es->az, es->ax[2], 0, -es->ax[0]);
	_vnormalize(es->az);
	vset(es->ay, 0,1,0);

	// Build action sampling spine.
	es->trajectory.nspine = MAX_SPINE;
	for (int i = 0; i < MAX_SPINE; ++i)
	{
		float* pt = &es->trajectory.spine[i*2];
		const float u = (float)i/(float)(MAX_SPINE-1);
		pt[0] = jumpStartDist + u * (jumpEndDist - jumpStartDist);
		pt[1] = u*u*u * jumpDownDist;
	}

	es->groundRange = groundRange;
}

void AnnotationBuilder::initJumpOverRig(EdgeSampler* es, const float* sp, const float* sq,
										const float jumpStartDist, const float jumpEndDist,
										const float jumpHeight, const float groundRange)
{
	vcopy(es->rigp, sp);
	vcopy(es->rigq, sq);

	vsub(es->ax, sq,sp);
	_vnormalize(es->ax);
	vset(es->az, es->ax[2], 0, -es->ax[0]);
	_vnormalize(es->az);
	vset(es->ay, 0,1,0);

	// Build action sampling spine.
	es->trajectory.nspine = MAX_SPINE;
	for (int i = 0; i < MAX_SPINE; ++i)
	{
		float* pt = &es->trajectory.spine[i*2];
		const float u = (float)i/(float)(MAX_SPINE-1);
		pt[0] = jumpStartDist + u * (jumpEndDist - jumpStartDist);
		pt[1] = (1-sqr(u*2-1)) * jumpHeight;
	}

	es->groundRange = groundRange;
}

AnnotationBuilder::EdgeSampler* AnnotationBuilder::sampleEdge(int type, const float* sp, const float* sq)
{
	EdgeSampler* es = new EdgeSampler;
	if (!es)
		return 0;

	if (type == EDGE_JUMP_DOWN)
	{
		initJumpDownRig(es, sp, sq, -0.25f, 2.0f, -3.0f, 0.5f);
	}
	else if (type == EDGE_JUMP_OVER)
	{
		const float jumpDist = 4.0f;
		static const int NSEGS = 8;
		float segs[NSEGS*6];
		int nsegs = findPotentialJumpOverEdges(sp, sq, jumpDist, 1.0f, segs, NSEGS);
		int ibest = -1;
		float dbest = 0;
		for (int i = 0; i < nsegs; ++i)
		{
			const float* seg = &segs[i*6];
			const float d = vdistSqr(seg,seg+3);
			if (d > dbest)
			{
				dbest = d;
				ibest = i;
			}
		}
		if (ibest == -1)
		{
			delete es;
			return 0;
		}

		initJumpOverRig(es, &segs[ibest*6+0], &segs[ibest*6+3], -jumpDist*0.5f, jumpDist*0.5f, 1.0f, 0.5f);
	}

	initTrajectory(&es->trajectory);

	// Init start end segments.
	float offset[3];
	trans2d(offset, es->az, es->ay, &es->trajectory.spine[0]);
	vadd(es->start.p, es->rigp, offset);
	vadd(es->start.q, es->rigq, offset);
	trans2d(offset, es->az, es->ay, &es->trajectory.spine[(es->trajectory.nspine-1)*2]);
	vadd(es->end.p, es->rigp, offset);
	vadd(es->end.q, es->rigq, offset);

	// Sample start and end ground segments.
	const float dist = sqrtf(vdistSqr(es->rigp,es->rigq));
	const float cs = m_cfg.cs;
	const int ngsamples = max(2, (int)ceilf(dist/cs));

	sampleGroundSegment(&es->start, ngsamples, es->groundRange);
	sampleGroundSegment(&es->end, ngsamples, es->groundRange);

	sampleAction(es);

	return es;
}

void AnnotationBuilder::buildNearestEdge(int type, const float* pos)
{
	float dmin = FLT_MAX;
	int imin = -1;
	for (int i = 0; i < m_nedges; ++i)
	{
		float t = getClosestPtPtSeg(pos, m_edges[i].sp, m_edges[i].sq);
		float pp[3];
		vlerp(pp, m_edges[i].sp, m_edges[i].sq, t);
		float d = vdistSqr(pos,pp);
		if (d < dmin)
		{
			dmin = d;
			imin = i;
		}
	}
	if (imin != -1)
	{
		m_selEdge = imin;
		delete m_sampler;
		m_sampler = sampleEdge(type, m_edges[m_selEdge].sp, m_edges[m_selEdge].sq);
	}
}

static void drawSpine(struct duDebugDraw* dd,
					  const float* pt, const float* ax, const float* ay, const float ah,
					  const float* spine, const int nspine,
					  const unsigned int col)
{
	float pa[3], pb[3];
	float sa[2], sb[2];

	for (int i = 0; i < nspine-1; ++i)
	{
		sa[0] = spine[i*2+0];
		sa[1] = spine[i*2+1] + ah/2;
		sb[0] = spine[(i+1)*2+0];
		sb[1] = spine[(i+1)*2+1] + ah/2;

		trans2dc(pa, pt, ax, ay, sa);
		trans2dc(pb, pt, ax, ay, sb);
		dd->vertex(pa, col);
		dd->vertex(pb, col);
	}
}

void AnnotationBuilder::drawTrajectory(struct duDebugDraw* dd, const float* pa, const float* pb,
									   Trajectory2D* tra, const unsigned int col)
{
//	float pa[3], pb[3];
//	float sa[2], sb[2];

	const float startx = tra->spine[0];
	const float endx = tra->spine[(tra->nspine-1)*2];
	const float deltax = endx - startx;

	const float starty = tra->spine[1];
	const float endy = tra->spine[(tra->nspine-1)*2+1];


	float pts[3*MAX_SPINE];
	int npts = tra->nspine;
	for (int i = 0; i < tra->nspine; ++i)
	{
		const float* spt = &tra->spine[i*2];
		const float u = (spt[0] - startx)/deltax;
		const float dy = spt[1] - lerp(starty,endy,u);
		float* p = &pts[i*3];
		vlerp(p, pa,pb,u);
		p[1] += dy;
	}

	const float len = getPathLen(pts, npts);
	const int nsegs = ceilf(len / 0.3f);

	for (int i = 0; i < nsegs*2; ++i)
	{
		float u = (float)i / (float)(nsegs*2);
		float pt[3];
		getPointAlongPath(u*len, pts, npts, pt);
		dd->vertex(pt, col);
	}



	// Build start spine.
/*	float p0[3], p1[3];
	for (int i = 0; i < tra->nspine; ++i)
	{
		const float* spt = &tra->spine[i*2];
		const float u = (spt[0] - startx)/deltax;
		const float dy = spt[1] - lerp(starty,endy,u);
		vlerp(p1, pa,pb,u);
		p1[1] += dy;
//		if (i > 0)
		{
			dd->vertex(p0, col);
//			dd->vertex(p1, col);
		}
		vcopy(p0,p1);
	}*/
}

void AnnotationBuilder::drawTrajectorySlice(struct duDebugDraw* dd, const float* pa, const float* pb,
											Trajectory2D* tra, const unsigned int col)
{
	//	float pa[3], pb[3];
	//	float sa[2], sb[2];

	const float startx = tra->spine[0];
	const float endx = tra->spine[(tra->nspine-1)*2];
//	const float deltax = endx - startx;

//	const float starty = tra->spine[1];
//	const float endy = tra->spine[(tra->nspine-1)*2+1];

	// Build start spine.
/*	float p0[3], p1[3];
	for (int i = 0; i < tra->nspine; ++i)
	{
		const float* spt = &tra->spine[i*2];
		const float u = (spt[0] - startx)/deltax;
		const float dy = spt[1] - lerp(starty,endy,u);
		vlerp(p1, pa,pb,u);
		p1[1] += dy;
		if (i > 0)
		{
			dd->vertex(p0, duTransCol(col,128));
			dd->vertex(p1, duTransCol(col,128));
		}
		vcopy(p0,p1);
	}*/

	unsigned int colt = duTransCol(col, 255);
	unsigned int colb = duTransCol(duLerpCol(col,duRGBA(0,0,0,255),96), 255);

	dd->begin(DU_DRAW_QUADS);
	float p0[3], p1[3], p2[3], p3[3];
	for (int i = 0; i < tra->nsamples; ++i)
	{
		const TrajectorySample* s = &tra->samples[i];
		const float u = (float)i / (float)(tra->nsamples-1);
		vlerp(p1, pa,pb,u);
		vcopy(p0,p1);
		p0[1] += s->ymin;
		p1[1] += s->ymax;

		if (i > 0)
		{
			dd->vertex(p0, colb);
			dd->vertex(p1, colt);
			dd->vertex(p3, colt);
			dd->vertex(p2, colb);

			dd->vertex(p2, colb);
			dd->vertex(p3, colt);
			dd->vertex(p1, colt);
			dd->vertex(p0, colb);
		}

		vcopy(p2,p0);
		vcopy(p3,p1);
	}
	dd->end();

	colb = duDarkenCol(colb);
	colt = duDarkenCol(colt);

	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int i = 0; i < tra->nsamples; ++i)
	{
		const TrajectorySample* s = &tra->samples[i];
		const float u = (float)i / (float)(tra->nsamples-1);
		vlerp(p1, pa,pb,u);
		vcopy(p0,p1);
		p0[1] += s->ymin;
		p1[1] += s->ymax;

		if (i == 0 || i == (tra->nsamples-1))
		{
			dd->vertex(p0, colb);
			dd->vertex(p1, colt);
		}
		if (i > 0)
		{
			dd->vertex(p0, colb);
			dd->vertex(p2, colb);
			dd->vertex(p1, colt);
			dd->vertex(p3, colt);
		}

		vcopy(p2,p0);
		vcopy(p3,p1);
	}
	dd->end();
}

static void drawSpineVolume(struct duDebugDraw* dd,
							const float* pt, const float* ax, const float* ay,
							const float ah, const float ar,
							const float* spine, const int nspine,
							const unsigned int col)
{
	float pa[3], pb[3], pc[3], pd[3];
	float sa[2], sb[2], sc[2], sd[2];

/*	sa[0] = spine[0*2+0] - ar;
	sa[1] = spine[0*2+1];
	sb[0] = spine[0*2+0] - ar;
	sb[1] = spine[0*2+1] + ah;
	sc[0] = spine[0*2+0] + ar;
	sc[1] = spine[0*2+1] + ah;
	trans2dc(pa, pt, ax, ay, sa);
	trans2dc(pb, pt, ax, ay, sb);
	trans2dc(pc, pt, ax, ay, sc);
	dd->vertex(pa, col);
	dd->vertex(pb, col);
	dd->vertex(pb, col);
	dd->vertex(pc, col);*/

	sa[0] = spine[0*2+0];
	sa[1] = spine[0*2+1];
	sb[0] = spine[0*2+0] + ar;
	sb[1] = spine[0*2+1] + ah;

	sc[0] = spine[(nspine-1)*2+0];
	sc[1] = spine[(nspine-1)*2+1];
	sd[0] = spine[(nspine-1)*2+0] + ar;
	sd[1] = spine[(nspine-1)*2+1] + ah;

	trans2dc(pa, pt, ax, ay, sa);
	trans2dc(pb, pt, ax, ay, sb);
	trans2dc(pc, pt, ax, ay, sc);
	trans2dc(pd, pt, ax, ay, sd);
	dd->vertex(pa, col);
	dd->vertex(pb, col);
	dd->vertex(pc, col);
	dd->vertex(pd, col);

	for (int i = 0; i < nspine-1; ++i)
	{
		sa[0] = spine[i*2+0];
		sa[1] = spine[i*2+1];
		sb[0] = spine[(i+1)*2+0];
		sb[1] = spine[(i+1)*2+1];
		sc[0] = spine[i*2+0] + ar;
		sc[1] = spine[i*2+1] + ah;
		sd[0] = spine[(i+1)*2+0] + ar;
		sd[1] = spine[(i+1)*2+1] + ah;
		trans2dc(pa, pt, ax, ay, sa);
		trans2dc(pb, pt, ax, ay, sb);
		trans2dc(pc, pt, ax, ay, sc);
		trans2dc(pd, pt, ax, ay, sd);
		dd->vertex(pa, col);
		dd->vertex(pb, col);
		dd->vertex(pc, col);
		dd->vertex(pd, col);
	}

/*	for (int i = 0; i < nspine-1; ++i)
	{
		float sa[2], sb[2];
		sa[0] = spine[i*2+0] - ar;
		sa[1] = spine[i*2+1];
		sb[0] = spine[(i+1)*2+0] - ar;
		sb[1] = spine[(i+1)*2+1];
		float pa[3], pb[3], da[3], db[3];
		trans2d(da, ax, ay, sa);
		trans2d(db, ax, ay, sb);

		vadd(pa, pt, da);
		vadd(pb, pt, db);
		dd->vertex(pa, col);
		dd->vertex(pb, col);
	}

	sa[0] = spine[(nspine-1)*2+0] - ar;
	sa[1] = spine[(nspine-1)*2+1];
	sb[0] = spine[(nspine-1)*2+0] + ar;
	sb[1] = spine[(nspine-1)*2+1];
	sc[0] = spine[(nspine-1)*2+0] + ar;
	sc[1] = spine[(nspine-1)*2+1] + ah;
	trans2dc(pa, pt, ax, ay, sa);
	trans2dc(pb, pt, ax, ay, sb);
	trans2dc(pc, pt, ax, ay, sc);
	dd->vertex(pa, col);
	dd->vertex(pb, col);
	dd->vertex(pb, col);
	dd->vertex(pc, col);*/
}

void AnnotationBuilder::draw(unsigned int flags)
{
	DebugDrawGL dd;

	dd.depthMask(false);

	if (flags & DRAW_WALKABLE_SURFACE)
	{
		if (m_chf != 0)
		{
			duDebugDrawCompactHeightfieldSolid(&dd, *m_chf);
		}
	}

/*	if (m_cset != 0)
	{
		duDebugDrawContours(&dd, *m_cset);
	}*/

	if (flags & DRAW_WALKABLE_BORDER)
	{
		if (m_nedges)
		{
			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int i = 0; i < m_nedges; ++i)
			{
				unsigned int col = duRGBA(0,96,128,255);
				if (i == m_selEdge)
					continue;
				dd.vertex(m_edges[i].sp, col);
				dd.vertex(m_edges[i].sq, col);
			}
			dd.end();

			dd.begin(DU_DRAW_POINTS, 8.0f);
			for (int i = 0; i < m_nedges; ++i)
			{
				unsigned int col = duRGBA(0,96,128,255);
				if (i == m_selEdge)
					continue;
				dd.vertex(m_edges[i].sp, col);
				dd.vertex(m_edges[i].sq, col);
			}
			dd.end();

			if (m_selEdge >= 0 && m_selEdge < m_nedges)
			{
				unsigned int col = duRGBA(48,16,16,255); //duRGBA(255,192,0,255);
				dd.begin(DU_DRAW_LINES, 3.0f);
				dd.vertex(m_edges[m_selEdge].sp, col);
				dd.vertex(m_edges[m_selEdge].sq, col);
				dd.end();
				dd.begin(DU_DRAW_POINTS, 8.0f);
				dd.vertex(m_edges[m_selEdge].sp, col);
				dd.vertex(m_edges[m_selEdge].sq, col);
				dd.end();
			}

			dd.begin(DU_DRAW_POINTS, 4.0f);
			for (int i = 0; i < m_nedges; ++i)
			{
				unsigned int col = duRGBA(190,190,190,255);
				dd.vertex(m_edges[i].sp, col);
				dd.vertex(m_edges[i].sq, col);
			}
			dd.end();
		}
	}


	if (flags & DRAW_ANNOTATIONS)
	{
		if (m_nlinks)
		{
			unsigned int col0 = duLerpCol(duRGBA(32,255,96,255),duRGBA(255,255,255,255), 200);
			unsigned int col1 = duRGBA(32,255,96,255);

			dd.begin(DU_DRAW_QUADS);
			for (int i = 0; i < m_nlinks; ++i)
			{
				const JumpLink* link = &m_links[i];
				if (link->flags == 0) continue;
				for (int j = 0; j < link->nspine-1; ++j)
				{
					int u = (j*255)/link->nspine;
					unsigned int col = duTransCol(duLerpCol(col0,col1,u),128);
					if (link->flags == 0)
						col = duRGBA(255,0,0,64);

					dd.vertex(&link->spine1[j*3], col);
					dd.vertex(&link->spine1[(j+1)*3], col);
					dd.vertex(&link->spine0[(j+1)*3], col);
					dd.vertex(&link->spine0[j*3], col);
				}
			}
			dd.end();

			dd.begin(DU_DRAW_LINES, 3.0f);
			for (int i = 0; i < m_nlinks; ++i)
			{
				const JumpLink* link = &m_links[i];
				if (link->flags == 0) continue;
				for (int j = 0; j < link->nspine-1; ++j)
				{
//					int u = (j*255)/link->nspine;
					unsigned int col = duTransCol(duDarkenCol(col1)/*duDarkenCol(duLerpCol(col0,col1,u))*/,128);

					dd.vertex(&link->spine0[j*3], col);
					dd.vertex(&link->spine0[(j+1)*3], col);
					dd.vertex(&link->spine1[j*3], col);
					dd.vertex(&link->spine1[(j+1)*3], col);
				}

				dd.vertex(&link->spine0[0], duDarkenCol(col1));
				dd.vertex(&link->spine1[0], duDarkenCol(col1));

				dd.vertex(&link->spine0[(link->nspine-1)*3], duDarkenCol(col1));
				dd.vertex(&link->spine1[(link->nspine-1)*3], duDarkenCol(col1));
			}
			dd.end();

			dd.begin(DU_DRAW_POINTS, 8.0f);
			for (int i = 0; i < m_nlinks; ++i)
			{
				const JumpLink* link = &m_links[i];
				if (link->flags == 0) continue;
				dd.vertex(&link->spine0[0], duDarkenCol(col1));
				dd.vertex(&link->spine1[0], duDarkenCol(col1));
				dd.vertex(&link->spine0[(link->nspine-1)*3], duDarkenCol(col1));
				dd.vertex(&link->spine1[(link->nspine-1)*3], duDarkenCol(col1));
			}
			dd.end();
			dd.begin(DU_DRAW_POINTS, 4.0f);
			for (int i = 0; i < m_nlinks; ++i)
			{
				const JumpLink* link = &m_links[i];
				if (link->flags == 0) continue;
				dd.vertex(&link->spine0[0], duRGBA(220,220,220,255));
				dd.vertex(&link->spine1[0], duRGBA(220,220,220,255));
				dd.vertex(&link->spine0[(link->nspine-1)*3], duRGBA(220,220,220,255));
				dd.vertex(&link->spine1[(link->nspine-1)*3], duRGBA(220,220,220,255));
			}
			dd.end();
		}
	}

	if (flags & DRAW_SELECTED_EDGE)
	{
		if (m_sampler)
		{
			EdgeSampler* es = m_sampler;


			dd.begin(DU_DRAW_LINES, 2.0f);

			// Axis
			const float d = 0.5f;
			dd.vertex(es->rigp, duRGBA(255,0,0,255));
			dd.vertex(es->rigp[0]+es->ax[0]*d, es->rigp[1]+es->ax[1]*d, es->rigp[2]+es->ax[2]*d, duRGBA(255,0,0,255));

			dd.vertex(es->rigp, duRGBA(0,255,0,255));
			dd.vertex(es->rigp[0]+es->ay[0]*d, es->rigp[1]+es->ay[1]*d, es->rigp[2]+es->ay[2]*d, duRGBA(0,255,0,255));

			dd.vertex(es->rigp, duRGBA(0,0,255,255));
			dd.vertex(es->rigp[0]+es->az[0]*d, es->rigp[1]+es->az[1]*d, es->rigp[2]+es->az[2]*d, duRGBA(0,0,255,255));

			dd.end();

			// Action spine
	/*		drawSpine(&dd, es->sp, es->az, es->ay, m_acfg.agentHeight, es->spine, es->nspine, duRGBA(255,0,0,128));
			drawSpine(&dd, es->sq, es->az, es->ay, m_acfg.agentHeight, es->spine, es->nspine, duRGBA(255,0,0,128));
			drawSpineVolume(&dd, es->sp, es->az, es->ay, m_acfg.agentHeight, m_acfg.agentRadius, es->spine, es->nspine, duRGBA(255,255,0,128));
			drawSpineVolume(&dd, es->sq, es->az, es->ay, m_acfg.agentHeight, m_acfg.agentRadius, es->spine, es->nspine, duRGBA(255,255,0,128));*/

	//		drawTrajectory(&dd, es->sp, es->az, es->ay, &es->trajectory, duRGBA(255,255,0,128));


			{
	/*			dd.vertex(es->sp[0],es->sp[1],es->sp[2], duRGBA(255,255,255,255));
				dd.vertex(es->sp[0]+es->az[0]*es->depth,es->sp[1]+es->az[1]*es->depth,es->sp[2]+es->az[2]*es->depth, duRGBA(255,255,255,255));

				dd.vertex(es->sq[0],es->sq[1],es->sq[2], duRGBA(255,255,255,255));
				dd.vertex(es->sq[0]+es->az[0]*es->depth,es->sq[1]+es->az[1]*es->depth,es->sq[2]+es->az[2]*es->depth, duRGBA(255,255,255,255));

				dd.vertex(es->sp[0]+es->az[0]*es->depth,es->sp[1]+es->az[1]*es->depth,es->sp[2]+es->az[2]*es->depth, duRGBA(255,255,255,255));
				dd.vertex(es->sq[0]+es->az[0]*es->depth,es->sq[1]+es->az[1]*es->depth,es->sq[2]+es->az[2]*es->depth, duRGBA(255,255,255,255));

				dd.vertex(es->sp[0]+es->ay[0]*-es->height,
						  es->sp[1]+es->ay[1]*-es->height,
						  es->sp[2]+es->ay[2]*-es->height, duRGBA(255,255,255,255));
				dd.vertex(es->sp[0]+es->ay[0]*es->height,
						  es->sp[1]+es->ay[1]*es->height,
						  es->sp[2]+es->ay[2]*es->height, duRGBA(255,255,255,255));
				dd.vertex(es->sq[0]+es->ay[0]*-es->height,
						  es->sq[1]+es->ay[1]*-es->height,
						  es->sq[2]+es->ay[2]*-es->height, duRGBA(255,255,255,255));
				dd.vertex(es->sq[0]+es->ay[0]*es->height,
						  es->sq[1]+es->ay[1]*es->height,
						  es->sq[2]+es->ay[2]*es->height, duRGBA(255,255,255,255));*/


	/*			for (int i = 0; i < es->nsegs; ++i)
				{
					unsigned int col = duIntToCol(es->segs[i].mark, 255);

					float pa[3], pb[3];
					vlerp(pa, m_edges[es->segs[i].idx].sp, m_edges[es->segs[i].idx].sq, es->segs[i].umin);
					vlerp(pb, m_edges[es->segs[i].idx].sp, m_edges[es->segs[i].idx].sq, es->segs[i].umax);
					dd.vertex(pa, col);
					dd.vertex(pb, col);

					dd.vertex(es->segs[i].sp, duRGBA(255,255,255,255));
					dd.vertex(es->segs[i].sq, duRGBA(0,0,0,255));

					float ax[3];
					float az[3];

					vsub(ax, es->segs[i].sq,es->segs[i].sp);
					vnormalize(ax);
					vset(az, ax[2], 0, -ax[0]);
					vnormalize(az);

					dd.vertex(es->segs[i].sp[0] + az[0]*es->depth*-0.5f,
							  es->segs[i].sp[1] + az[1]*es->depth*-0.5f,
							  es->segs[i].sp[2] + az[2]*es->depth*-0.5f, duRGBA(255,255,255,128));
					dd.vertex(es->segs[i].sp[0] + az[0]*es->depth*0.5f,
							  es->segs[i].sp[1] + az[1]*es->depth*0.5f,
							  es->segs[i].sp[2] + az[2]*es->depth*0.5f, duRGBA(255,255,255,128));

					dd.vertex(es->segs[i].sp[0] + az[0]*es->depth*0.5f,
							  es->segs[i].sp[1] + az[1]*es->depth*0.5f,
							  es->segs[i].sp[2] + az[2]*es->depth*0.5f, duRGBA(255,255,255,128));
					dd.vertex(es->segs[i].sq[0] + az[0]*es->depth*0.5f,
							  es->segs[i].sq[1] + az[1]*es->depth*0.5f,
							  es->segs[i].sq[2] + az[2]*es->depth*0.5f, duRGBA(255,255,255,128));

					dd.vertex(es->segs[i].sq[0] + az[0]*es->depth*-0.5f,
							  es->segs[i].sq[1] + az[1]*es->depth*-0.5f,
							  es->segs[i].sq[2] + az[2]*es->depth*-0.5f, duRGBA(255,255,255,128));
					dd.vertex(es->segs[i].sq[0] + az[0]*es->depth*0.5f,
							  es->segs[i].sq[1] + az[1]*es->depth*0.5f,
							  es->segs[i].sq[2] + az[2]*es->depth*0.5f, duRGBA(255,255,255,128));

					dd.vertex(es->segs[i].sp[0] + az[0]*es->depth*-0.5f,
							  es->segs[i].sp[1] + az[1]*es->depth*-0.5f,
							  es->segs[i].sp[2] + az[2]*es->depth*-0.5f, duRGBA(255,255,255,128));
					dd.vertex(es->segs[i].sq[0] + az[0]*es->depth*-0.5f,
							  es->segs[i].sq[1] + az[1]*es->depth*-0.5f,
							  es->segs[i].sq[2] + az[2]*es->depth*-0.5f, duRGBA(255,255,255,128));

				}*/
			}


			if (flags & DRAW_ANIM_TRAJECTORY)
			{

				const float r = es->groundRange;

				unsigned int col = duLerpCol(duRGBA(255,192,0,255), duRGBA(255,255,255,255), 64);
				unsigned int cola = duTransCol(col,192);
				unsigned int colb = duRGBA(255,255,255,255);

				// Start segment.
				dd.begin(DU_DRAW_LINES, 3.0f);
				dd.vertex(es->start.p[0],es->start.p[1],es->start.p[2], col);
				dd.vertex(es->start.q[0],es->start.q[1],es->start.q[2], col);
				dd.end();

				dd.begin(DU_DRAW_LINES, 1.0f);
				dd.vertex(es->start.p[0],es->start.p[1]-r,es->start.p[2], colb);
				dd.vertex(es->start.p[0],es->start.p[1]+r,es->start.p[2], colb);
				dd.vertex(es->start.p[0],es->start.p[1]+r,es->start.p[2], colb);
				dd.vertex(es->start.q[0],es->start.q[1]+r,es->start.q[2], colb);
				dd.vertex(es->start.q[0],es->start.q[1]+r,es->start.q[2], colb);
				dd.vertex(es->start.q[0],es->start.q[1]-r,es->start.q[2], colb);
				dd.vertex(es->start.q[0],es->start.q[1]-r,es->start.q[2], colb);
				dd.vertex(es->start.p[0],es->start.p[1]-r,es->start.p[2], colb);
				dd.end();

				// End segment.
				dd.begin(DU_DRAW_LINES, 3.0f);
				dd.vertex(es->end.p[0],es->end.p[1],es->end.p[2], col);
				dd.vertex(es->end.q[0],es->end.q[1],es->end.q[2], col);
				dd.end();

				dd.begin(DU_DRAW_LINES, 1.0f);
				dd.vertex(es->end.p[0],es->end.p[1]-r,es->end.p[2], colb);
				dd.vertex(es->end.p[0],es->end.p[1]+r,es->end.p[2], colb);
				dd.vertex(es->end.p[0],es->end.p[1]+r,es->end.p[2], colb);
				dd.vertex(es->end.q[0],es->end.q[1]+r,es->end.q[2], colb);
				dd.vertex(es->end.q[0],es->end.q[1]+r,es->end.q[2], colb);
				dd.vertex(es->end.q[0],es->end.q[1]-r,es->end.q[2], colb);
				dd.vertex(es->end.q[0],es->end.q[1]-r,es->end.q[2], colb);
				dd.vertex(es->end.p[0],es->end.p[1]-r,es->end.p[2], colb);
				dd.end();

				dd.begin(DU_DRAW_LINES, 4.0f);
				drawTrajectory(&dd, es->start.p, es->end.p, &es->trajectory, cola);
				drawTrajectory(&dd, es->start.q, es->end.q, &es->trajectory, cola);
				dd.end();


				dd.begin(DU_DRAW_POINTS, 8.0f);
				dd.vertex(es->start.p[0],es->start.p[1],es->start.p[2], duDarkenCol(col));
				dd.vertex(es->start.q[0],es->start.q[1],es->start.q[2], duDarkenCol(col));
				dd.vertex(es->end.p[0],es->end.p[1],es->end.p[2], duDarkenCol(col));
				dd.vertex(es->end.q[0],es->end.q[1],es->end.q[2], duDarkenCol(col));
				dd.end();

				unsigned int colm = duRGBA(255,255,255,255);
				dd.begin(DU_DRAW_POINTS, 3.0f);
				dd.vertex(es->start.p[0],es->start.p[1],es->start.p[2], colm);
				dd.vertex(es->start.q[0],es->start.q[1],es->start.q[2], colm);
				dd.vertex(es->end.p[0],es->end.p[1],es->end.p[2], colm);
				dd.vertex(es->end.q[0],es->end.q[1],es->end.q[2], colm);
				dd.end();

			}


			if (flags & DRAW_LAND_SAMPLES)
			{

				if (es->start.ngsamples)
				{
					dd.begin(DU_DRAW_POINTS, 8.0f);
					for (int i = 0; i < es->start.ngsamples; ++i)
					{
						const GroundSample* s = &es->start.gsamples[i];
//						if (!s->flags) continue;
						const float u = (float)i/(float)(es->start.ngsamples-1);
						float spt[3], ept[3];
						vlerp(spt, es->start.p, es->start.q,u);
						vlerp(ept, es->start.p, es->start.q,u);

						unsigned int col = duRGBA(48,16,16,255); //duRGBA(255,(s->flags & 4)?255:0,0,255);
						float off = 0.1f;
						if ((s->flags & 1) == 0)
						{
							off = 0;
							col = duRGBA(220,32,32,255);
						}

//						if (s->flags & 1)
						{
							spt[1] = s->height + off;
							dd.vertex(spt, col);
						}
					}
					dd.end();

					dd.begin(DU_DRAW_POINTS, 4.0f);
					for (int i = 0; i < es->start.ngsamples; ++i)
					{
						const GroundSample* s = &es->start.gsamples[i];
//						if (!s->flags) continue;
						const float u = (float)i/(float)(es->start.ngsamples-1);
						float spt[3], ept[3];
						vlerp(spt, es->start.p, es->start.q,u);
						vlerp(ept, es->start.p, es->start.q,u);
						float off = 0;
						if (s->flags & 1)
						{
							off = 0.1f;
						}
						spt[1] = s->height + off;
						dd.vertex(spt, duRGBA(255,255,255,255));
					}
					dd.end();

				}

				if (es->end.ngsamples)
				{
					dd.begin(DU_DRAW_POINTS, 8.0f);
					for (int i = 0; i < es->end.ngsamples; ++i)
					{
						const GroundSample* s = &es->end.gsamples[i];
						//						if (!s->flags) continue;
						const float u = (float)i/(float)(es->end.ngsamples-1);
						float spt[3], ept[3];
						vlerp(spt, es->end.p, es->end.q,u);
						vlerp(ept, es->end.p, es->end.q,u);

						unsigned int col = duRGBA(48,16,16,255); //duRGBA(255,(s->flags & 4)?255:0,0,255);
						float off = 0.1f;
						if ((s->flags & 1) == 0)
						{
							off = 0;
							col = duRGBA(220,32,32,255);
						}

						//						if (s->flags & 1)
						{
							spt[1] = s->height + off;
							dd.vertex(spt, col);
						}
					}
					dd.end();

					dd.begin(DU_DRAW_POINTS, 4.0f);
					for (int i = 0; i < es->end.ngsamples; ++i)
					{
						const GroundSample* s = &es->end.gsamples[i];
						//						if (!s->flags) continue;
						const float u = (float)i/(float)(es->end.ngsamples-1);
						float spt[3], ept[3];
						vlerp(spt, es->end.p, es->end.q,u);
						vlerp(ept, es->end.p, es->end.q,u);

						float off = 0;
						if (s->flags & 1)
						{
							off = 0.1f;
						}
						spt[1] = s->height + off;
						dd.vertex(spt, duRGBA(255,255,255,255));
					}
					dd.end();


/*					dd.begin(DU_DRAW_POINTS, 4.0f);
					for (int i = 0; i < es->end.ngsamples; ++i)
					{
						const GroundSample* s = &es->end.gsamples[i];
//						if (!s->flags) continue;
						const float u = (float)i/(float)(es->end.ngsamples-1);
						float spt[3], ept[3];
						vlerp(spt, es->end.p, es->end.q,u);
						vlerp(ept, es->end.p, es->end.q,u);

						unsigned int col = duRGBA(255,255,0,255); //duRGBA(255,(s->flags & 4)?255:0,0,255);

//						if (s->flags & 1)
						{
							spt[1] = s->height + 0.1f;
							dd.vertex(spt, col);
						}
					}
					dd.end();*/
				}
			}

			if (flags & DRAW_COLLISION_SLICES)
			{
				if (es->start.ngsamples && es->start.ngsamples == es->end.ngsamples)
				{
					const int nsamples = es->start.ngsamples;
					dd.depthMask(true);
//					dd.begin(DU_DRAW_LINES, 2.0f);
					for (int i = 0; i < nsamples; ++i)
					{
						const GroundSample* ssmp = &es->start.gsamples[i];
						const GroundSample* esmp = &es->end.gsamples[i];
						if ((ssmp->flags & 1) == 0 || (esmp->flags & 1) == 0)
							continue;

						const float u = (float)i/(float)(nsamples-1);
						float spt[3], ept[3];
						vlerp(spt, es->start.p, es->start.q, u);
						vlerp(ept, es->end.p, es->end.q, u);

						spt[1] = ssmp->height;
						ept[1] = esmp->height;

						if (ssmp->flags & 4)
							drawTrajectorySlice(&dd, spt, ept, &es->trajectory, duRGBA(32,255,96,255));
						else
							drawTrajectorySlice(&dd, spt, ept, &es->trajectory, duRGBA(255,32,32,255));

					}
//					dd.end();
					dd.depthMask(true);
				}
			}


	//		drawBoxWire(&dd, es->sp, es->ax, es->ay, es->az, es->bmin, es->bmax, 1.0f, duRGBA(255,255,255,64));

		}
	}

/*		if (m_selEdge >= 0 && m_selEdge < m_nedges)
		{
			const float* sp = m_edges[m_selEdge].sp;
			const float* sq = m_edges[m_selEdge].sq;
			float edge[3], norm[3], up[3];
			vsub(edge, sq,sp);
			vnormalize(edge);
			vset(norm, edge[2], 0, -edge[0]);
			vnormalize(norm);
			vset(up, 0,1,0);

			const float s = 2.0f;

			dd.begin(DU_DRAW_LINES, 2.0f);

			dd.vertex(sp, duRGBA(255,0,0,255));
			dd.vertex(sp[0]+edge[0]*s, sp[1]+edge[1]*s, sp[2]+edge[2]*s, duRGBA(255,0,0,255));

			dd.vertex(sp, duRGBA(0,255,0,255));
			dd.vertex(sp[0]+norm[0]*s, sp[1]+norm[1]*s, sp[2]+norm[2]*s, duRGBA(0,255,0,255));

			dd.vertex(sp, duRGBA(0,0,255,255));
			dd.vertex(sp[0]+up[0]*s, sp[1]+up[1]*s, sp[2]+up[2]*s, duRGBA(0,0,255,255));

			dd.end();
		}
	}*/

	dd.depthMask(true);

}

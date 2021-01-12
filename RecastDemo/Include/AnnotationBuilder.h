#ifndef ANNOTATIONBUILDER_H
#define ANNOTATIONBUILDER_H

#include "Recast.h"

enum DrawFlags
{
	DRAW_WALKABLE_SURFACE =	1 << 0,
	DRAW_WALKABLE_BORDER =	1 << 1,
	DRAW_SELECTED_EDGE =	1 << 2,
	DRAW_ANIM_TRAJECTORY =	1 << 3,
	DRAW_LAND_SAMPLES =		1 << 4,
	DRAW_COLLISION_SLICES =	1 << 5,
	DRAW_ANNOTATIONS =		1 << 6,
};

enum SampleType
{
	EDGE_JUMP_DOWN,
	EDGE_JUMP_OVER,
};

struct AnnotationBuilderConfig
{
	float agentRadius;
	float agentHeight;
	float agentClimb;
	float cellSize;
	float cellHeight;
};

class AnnotationBuilder
{
	AnnotationBuilderConfig m_acfg;

	rcConfig m_cfg;
	unsigned char* m_triareas;
	rcHeightfield* m_solid;
	rcCompactHeightfield* m_chf;
	rcContourSet* m_cset;

	struct Edge
	{
		float sp[3], sq[3];
	};

	Edge* m_edges;
	int m_nedges;

	void cleanup();

	static const int MAX_SPINE = 8;

	struct TrajectorySample
	{
		float x, ymin, ymax;
	};

	struct Trajectory2D
	{
		Trajectory2D() : nspine(0), samples(0), nsamples(0) {}
		~Trajectory2D() { delete [] samples; }
		float spine[2*MAX_SPINE];
		int nspine;
		TrajectorySample* samples;
		int nsamples;
	};

	void initTrajectory(Trajectory2D* tra);
	void drawTrajectory(struct duDebugDraw* dd, const float* pa, const float* pb, Trajectory2D* tra, const unsigned int col);
	void drawTrajectorySlice(struct duDebugDraw* dd, const float* pa, const float* pb, Trajectory2D* tra, const unsigned int col);
	bool sampleTrajectory(const float* pa, const float* pb, Trajectory2D* tra);

	struct GroundSample
	{
		float height;
		unsigned char flags;
	};

	struct PotentialSeg
	{
		unsigned char mark;
		int idx;
		float umin, umax;
		float dmin, dmax;
		float sp[3], sq[3];
	};

	struct GroundSegment
	{
		inline GroundSegment() : gsamples(0), ngsamples(0) {}
		inline ~GroundSegment() { delete [] gsamples; }

		float p[3], q[3];
		GroundSample* gsamples;
		int ngsamples;
		int npass;
	};

	struct EdgeSampler
	{
		GroundSegment start;
		GroundSegment end;

		float groundRange;

		Trajectory2D trajectory;

		float rigp[3], rigq[3];
		float ax[3], ay[3], az[3];
	};
	EdgeSampler* m_sampler;


	int findPotentialJumpOverEdges(const float* sp, const float* sq,
								   const float depthRange, const float heightRange,
								   float* outSegs, const int maxOutSegs);

	void initJumpDownRig(EdgeSampler* es, const float* sp, const float* sq,
						 const float jumpStartDist, const float jumpEndDist,
						 const float jumpDownDist, const float groundRange);

	void initJumpOverRig(EdgeSampler* es, const float* sp, const float* sq,
						 const float jumpStartDist, const float jumpEndDist,
						 const float jumpHeight, const float groundRange);

	bool getFloorPosition(const float* pt, const float hrange, float* height);

	bool getCompactHeightfieldHeigh(const float* pt, const float hrange, float* height);
	bool checkHeightfieldCollision(const float x, const float ymin, const float ymax, const float z);

	void sampleGroundSegment(GroundSegment* seg, const float nsamples, const float groundRange);

	void sampleAction(EdgeSampler* es);

	void filterJumpOverLinks();

	EdgeSampler* sampleEdge(int type, const float* sp, const float* sq);
	void addEdgeLinks(EdgeSampler* es);

	int m_selEdge;


	struct JumpLink
	{
		float spine0[MAX_SPINE*3];
		float spine1[MAX_SPINE*3];
		int nspine;
		unsigned char flags;
	};
	JumpLink* m_links;
	int m_nlinks;
	int m_clinks;

	JumpLink* addLink();


public:
	AnnotationBuilder();
	~AnnotationBuilder();

	void clearLinks();

	bool build(const AnnotationBuilderConfig& acfg, class NavWorld* world);

	void buildAllEdges(int type);
	void buildNearestEdge(int type, const float* pos);

	void draw(unsigned int flags);
};

#endif // ANNOTATIONBUILDER_H
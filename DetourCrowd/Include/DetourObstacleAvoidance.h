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

#ifndef DETOUROBSTACLEAVOIDANCE_H
#define DETOUROBSTACLEAVOIDANCE_H

#include <vector>

struct dtObstacleCircle
{
	float p[3];				///< Position of the obstacle
	float vel[3];			///< Velocity of the obstacle
	float dvel[3];			///< Velocity of the obstacle
	float rad;				///< Radius of the obstacle
	float dp[3], np[3];		///< Use for side selection during sampling.
};

struct dtObstacleSegment
{
	float p[3], q[3];		///< End points of the obstacle segment
	bool touch;
};

class dtObstacleAvoidanceDebugData
{
public:
	dtObstacleAvoidanceDebugData();
	~dtObstacleAvoidanceDebugData();

	bool init(const int maxSamples);
	void reset();
	void addSample(const float* vel, const float ssize, const float pen,
		const float vpen, const float vcpen, const float spen, const float tpen);

	void normalizeSamples();

	inline int getSampleCount() const { return m_nsamples; }
	inline const float* getSampleVelocity(const int i) const { return &m_vel[i * 3]; }
	inline float getSampleSize(const int i) const { return m_ssize[i]; }
	inline float getSamplePenalty(const int i) const { return m_pen[i]; }
	inline float getSampleDesiredVelocityPenalty(const int i) const { return m_vpen[i]; }
	inline float getSampleCurrentVelocityPenalty(const int i) const { return m_vcpen[i]; }
	inline float getSamplePreferredSidePenalty(const int i) const { return m_spen[i]; }
	inline float getSampleCollisionTimePenalty(const int i) const { return m_tpen[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtObstacleAvoidanceDebugData(const dtObstacleAvoidanceDebugData&) = delete;
	dtObstacleAvoidanceDebugData& operator=(const dtObstacleAvoidanceDebugData&) = delete;

	int m_nsamples;
	int m_maxSamples;
	std::vector<float> m_vel;
	std::vector<float> m_ssize;
	std::vector<float> m_pen; // 合計ペナルティー
	std::vector<float> m_vpen; // 望まれる速度ペナルティー
	std::vector<float> m_vcpen; // 現在の速度ペナルティー
	std::vector<float> m_spen; // 優先サイドペナルティ（はみ出た分のペナルティー？）
	std::vector<float> m_tpen; // 衝突時間ペナルティ
};

dtObstacleAvoidanceDebugData* dtAllocObstacleAvoidanceDebugData();
void dtFreeObstacleAvoidanceDebugData(dtObstacleAvoidanceDebugData* ptr);

constexpr int DT_MAX_PATTERN_DIVS = 32;	///< Max numver of adaptive divs.
constexpr int DT_MAX_PATTERN_RINGS = 4;	///< Max number of adaptive rings.

struct dtObstacleAvoidanceParams
{
	float velBias;
	float weightDesVel;
	float weightCurVel;
	float weightSide;
	float weightToi;
	float horizTime;
	unsigned char gridSize;	///< grid
	unsigned char adaptiveDivs;	///< adaptive
	unsigned char adaptiveRings;	///< adaptive
	unsigned char adaptiveDepth;	///< adaptive
};

class dtObstacleAvoidanceQuery
{
public:
	dtObstacleAvoidanceQuery();
	~dtObstacleAvoidanceQuery();

	bool init(const int maxCircles, const int maxSegments);

	void reset();

	void addCircle(const float* pos, const float rad,
		const float* vel, const float* dvel);

	void addSegment(const float* p, const float* q);

	int sampleVelocityGrid(const float* pos, const float rad, const float vmax,
		const float* vel, const float* dvel, float* nvel,
		const dtObstacleAvoidanceParams* params,
		dtObstacleAvoidanceDebugData* debug = 0);

	int sampleVelocityAdaptive(const float* pos, const float rad, const float vmax,
		const float* vel, const float* dvel, float* nvel,
		const dtObstacleAvoidanceParams* params,
		dtObstacleAvoidanceDebugData* debug = 0);

	inline int getObstacleCircleCount() const { return m_ncircles; }
	const dtObstacleCircle* getObstacleCircle(const int i) { return &m_circles[i]; }

	inline int getObstacleSegmentCount() const { return m_nsegments; }
	const dtObstacleSegment* getObstacleSegment(const int i) { return &m_segments[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtObstacleAvoidanceQuery(const dtObstacleAvoidanceQuery&);
	dtObstacleAvoidanceQuery& operator=(const dtObstacleAvoidanceQuery&);

	void prepare(const float* pos, const float* dvel);

	// Calculate the collision penalty for a given velocity vector
	// 与えられた速度ベクトルの衝突ペナルティを計算します
	//  @param vcand sampled velocity
	//  サンプリングされた速度
	//  @param dvel desired velocity
	//  望ましい速度
	//  @param minPenalty threshold penalty for early out
	//  アーリーアウトのペナルティのしきい値
	float processSample(const float* vcand, const float cs,
		const float* pos, const float rad,
		const float* vel, const float* dvel,
		const float minPenalty,
		dtObstacleAvoidanceDebugData* debug);

	dtObstacleAvoidanceParams m_params;
	float m_invHorizTime;
	float m_vmax;
	float m_invVmax;

	int m_maxCircles;
	dtObstacleCircle* m_circles;
	int m_ncircles;

	int m_maxSegments;
	dtObstacleSegment* m_segments;
	int m_nsegments;
};

dtObstacleAvoidanceQuery* dtAllocObstacleAvoidanceQuery();
void dtFreeObstacleAvoidanceQuery(dtObstacleAvoidanceQuery* ptr);

#endif // DETOUROBSTACLEAVOIDANCE_H

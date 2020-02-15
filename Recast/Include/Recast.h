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

#ifndef RECAST_H
#define RECAST_H

#include <utility>
#include <algorithm>
#include <array>
#include <memory>
#include "DebugNewDef.h"
#include "RecastConfig.h"

// The value of PI used by Recast.
// Recast���g�p����PI�̒l�B
constexpr float RC_PI = 3.14159265f;

// Recast log categories.
// ���O�J�e�S�������L���X�g���܂��B
// @see rcContext
enum rcLogCategory
{
	RC_LOG_PROGRESS = 1,	//< A progress log entry. // �i�����O�G���g���B
	RC_LOG_WARNING,			//< A warning log entry.  // �x�����O�G���g���B
	RC_LOG_ERROR,			//< An error log entry.   // �G���[���O�G���g���B
};

// Recast performance timer categories.
// �p�t�H�[�}���X�^�C�}�[�J�e�S�������L���X�g���܂��B
// @see rcContext
enum rcTimerLabel
{
	// The user defined total time of the build.
	// ���[�U�[��`�̃r���h�̍��v���ԁB
	RC_TIMER_TOTAL,
	// A user defined build time.
	// ���[�U�[��`�̃r���h���ԁB
	RC_TIMER_TEMP,
	// The time to rasterize the triangles. (See: #rcRasterizeTriangle)
	// �O�p�`�����X�^���C�Y���鎞�ԁB
	RC_TIMER_RASTERIZE_TRIANGLES,
	// The time to build the compact heightfield. (See: #rcBuildCompactHeightfield)
	// �R���p�N�g�Ȓn�`���\�z���鎞�ԁB
	RC_TIMER_BUILD_COMPACTHEIGHTFIELD,
	// The total time to build the contours. (See: #rcBuildContours)
	// �֊s���\�z���邽�߂̍��v���ԁB
	RC_TIMER_BUILD_CONTOURS,
	// The time to trace the boundaries of the contours. (See: #rcBuildContours)
	// �������̋��E���g���[�X���鎞�ԁB
	RC_TIMER_BUILD_CONTOURS_TRACE,
	// The time to simplify the contours. (See: #rcBuildContours)
	// �֊s��P�������鎞�ԁB
	RC_TIMER_BUILD_CONTOURS_SIMPLIFY,
	// The time to filter ledge spans. (See: #rcFilterLedgeSpans)
	// ���b�W�X�p�����t�B���^�����O���鎞�ԁB
	RC_TIMER_FILTER_BORDER,
	// The time to filter low height spans. (See: #rcFilterWalkableLowHeightSpans)
	// �Ⴂ�����̃X�p�����t�B���^�����O���鎞�ԁB
	RC_TIMER_FILTER_WALKABLE,
	// The time to apply the median filter. (See: #rcMedianFilterWalkableArea)
	// �����l�t�B���^�[��K�p���鎞�ԁB
	RC_TIMER_MEDIAN_AREA,
	// The time to filter low obstacles. (See: #rcFilterLowHangingWalkableObstacles)
	// ���Q�����t�B���^�����O���鎞�ԁB
	RC_TIMER_FILTER_LOW_OBSTACLES,
	// The time to build the polygon mesh. (See: #rcBuildPolyMesh)
	// �|���S�����b�V�����\�z���鎞�ԁB
	RC_TIMER_BUILD_POLYMESH,
	// The time to merge polygon meshes. (See: #rcMergePolyMeshes)
	// �|���S�����b�V�����}�[�W���鎞�ԁB
	RC_TIMER_MERGE_POLYMESH,
	// The time to erode the walkable area. (See: #rcErodeWalkableArea)
	// ���s�\�G���A��N�H���鎞�ԁB
	RC_TIMER_ERODE_AREA,
	// The time to mark a box area. (See: #rcMarkBoxArea)
	// �{�b�N�X�̈���}�[�N���鎞�ԁB
	RC_TIMER_MARK_BOX_AREA,
	// The time to mark a cylinder area. (See: #rcMarkCylinderArea)
	// �~���̈���}�[�N���鎞�ԁB
	RC_TIMER_MARK_CYLINDER_AREA,
	// The time to mark a convex polygon area. (See: #rcMarkConvexPolyArea)
	// �ʑ��p�`�̈���}�[�N���鎞�ԁB
	RC_TIMER_MARK_CONVEXPOLY_AREA,
	// The total time to build the distance field. (See: #rcBuildDistanceField)
	// �����t�B�[���h���\�z���邽�߂̍��v���ԁB
	RC_TIMER_BUILD_DISTANCEFIELD,
	// The time to build the distances of the distance field. (See: #rcBuildDistanceField)
	// �����t�B�[���h�̋������\�z���鎞�ԁB
	RC_TIMER_BUILD_DISTANCEFIELD_DIST,
	// The time to blur the distance field. (See: #rcBuildDistanceField)
	// �����t�B�[���h���ڂ������ԁB
	RC_TIMER_BUILD_DISTANCEFIELD_BLUR,
	// The total time to build the regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
	// �̈���\�z���邽�߂̍��v���ԁB
	RC_TIMER_BUILD_REGIONS,
	// The total time to apply the watershed algorithm. (See: #rcBuildRegions)
	// ����A���S���Y����K�p���鍇�v���ԁB
	RC_TIMER_BUILD_REGIONS_WATERSHED,
	// The time to expand regions while applying the watershed algorithm. (See: #rcBuildRegions)
	// ����A���S���Y����K�p���Ȃ���̈���g�傷�鎞�ԁB
	RC_TIMER_BUILD_REGIONS_EXPAND,
	// The time to flood regions while applying the watershed algorithm. (See: #rcBuildRegions)
	// ����A���S���Y����K�p���Ȃ���̈���t���b�f�B���O���鎞�ԁB
	RC_TIMER_BUILD_REGIONS_FLOOD,
	// The time to filter out small regions. (See: #rcBuildRegions, #rcBuildRegionsMonotone)
	// �����ȗ̈�����O���鎞�ԁB
	RC_TIMER_BUILD_REGIONS_FILTER,
	// The time to build heightfield layers. (See: #rcBuildHeightfieldLayers)
	// �n�`���C���[���\�z���鎞�ԁB
	RC_TIMER_BUILD_LAYERS,
	// The time to build the polygon mesh detail. (See: #rcBuildPolyMeshDetail)
	// �|���S�����b�V���̏ڍׂ��쐬���鎞�ԁB
	RC_TIMER_BUILD_POLYMESHDETAIL,
	// The time to merge polygon mesh details. (See: #rcMergePolyMeshDetails)
	// �|���S�����b�V���̏ڍׂ��}�[�W���鎞�ԁB
	RC_TIMER_MERGE_POLYMESHDETAIL,
	// The maximum number of timers.  (Used for iterating timers.)
	// �^�C�}�[�̍ő吔�B �i�^�C�}�[�̔����Ɏg�p����܂��B�j
	RC_MAX_TIMERS
};

// Provides an interface for optional logging and performance tracking of the Recast
// build process.
// Recast�r���h�v���Z�X�̃I�v�V�����̃��O�ƃp�t�H�[�}���X�ǐ՗p�̃C���^�[�t�F�C�X��񋟂��܂��B
// @ingroup recast
class rcContext
{
public:

	// Contructor.
	//  @param[in]		state	TRUE if the logging and performance timers should be enabled.  [Default: true]
	// ���O����уp�t�H�[�}���X�^�C�}�[��L���ɂ���K�v������ꍇ��TRUE�B[�f�t�H���g�Ftrue]
	inline rcContext(bool state = true) : m_logEnabled(state), m_timerEnabled(state) {}
	virtual ~rcContext() {}

	// Enables or disables logging.
	// ���O��L���܂��͖����ɂ��܂��B
	//  @param[in]		state	TRUE if logging should be enabled. // ���O��L���ɂ���K�v������ꍇ��TRUE�B
	inline void enableLog(bool state) { m_logEnabled = state; }

	// Clears all log entries.
	// ���ׂẴ��O�G���g�����N���A���܂��B
	inline void resetLog() { if (m_logEnabled) doResetLog(); }

	// Logs a message.
	// ���b�Z�[�W���L�^���܂��B
	//  @param[in]		category	The category of the message. // ���b�Z�[�W�̃J�e�S���B
	//  @param[in]		format		The message.
	void log(const rcLogCategory category, const char* format, ...);

	// Enables or disables the performance timers.
	// �p�t�H�[�}���X�^�C�}�[��L���܂��͖����ɂ��܂��B
	//  @param[in]		state	TRUE if timers should be enabled. // �^�C�}�[��L���ɂ���K�v������ꍇ��TRUE�B
	inline void enableTimer(bool state) { m_timerEnabled = state; }

	// Clears all peformance timers. (Resets all to unused.)
	// ���ׂẴp�t�H�[�}���X�^�C�}�[���N���A���܂��B�i���ׂĂ𖢎g�p�Ƀ��Z�b�g���܂��B�j
	inline void resetTimers() { if (m_timerEnabled) doResetTimers(); }

	// Starts the specified performance timer.
	// �w�肳�ꂽ�p�t�H�[�}���X�^�C�}�[���J�n���܂��B
	//  @param	label	The category of the timer. // �^�C�}�[�̃J�e�S���B
	inline void startTimer(const rcTimerLabel label) { if (m_timerEnabled) doStartTimer(label); }

	// Stops the specified performance timer.
	// �w�肳�ꂽ�p�t�H�[�}���X�^�C�}�[���~���܂��B
	//  @param	label	The category of the timer. // �^�C�}�[�̃J�e�S���B
	inline void stopTimer(const rcTimerLabel label) { if (m_timerEnabled) doStopTimer(label); }

	// Returns the total accumulated time of the specified performance timer.
	// �w�肵���p�t�H�[�}���X�^�C�}�[�̍��v�ݐώ��Ԃ�Ԃ��܂��B
	// @param	label	The category of the timer. // �^�C�}�[�̃J�e�S���B
	// @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
	// �^�C�}�[�̗ݐώ��ԁB�^�C�}�[�������ɂȂ��Ă��邩�A�^�C�}�[���J�n����Ă��Ȃ��ꍇ��-1�B
	inline int getAccumulatedTime(const rcTimerLabel label) const { return m_timerEnabled ? doGetAccumulatedTime(label) : -1; }

protected:

	// Clears all log entries.
	// ���ׂẴ��O�G���g�����N���A���܂��B
	virtual void doResetLog() {}

	// Logs a message.
	// ���b�Z�[�W���L�^���܂��B
	//  @param[in]		category	The category of the message. // ���b�Z�[�W�̃J�e�S���B
	//  @param[in]		msg			The formatted message. // �t�H�[�}�b�g���ꂽ���b�Z�[�W�B
	//  @param[in]		len			The length of the formatted message. // �t�H�[�}�b�g���ꂽ���b�Z�[�W�̒����B
	virtual void doLog(const rcLogCategory /*category*/, const char* /*msg*/, const int /*len*/) {}

	// Clears all timers. (Resets all to unused.)
	// ���ׂẴ^�C�}�[���N���A���܂��B�i���ׂĂ𖢎g�p�Ƀ��Z�b�g���܂��B�j
	virtual void doResetTimers() {}

	// Starts the specified performance timer.
	// �w�肳�ꂽ�p�t�H�[�}���X�^�C�}�[���J�n���܂��B
	//  @param[in]		label	The category of timer. // �^�C�}�[�̃J�e�S���B
	virtual void doStartTimer(const rcTimerLabel /*label*/) {}

	// Stops the specified performance timer.
	// �w�肳�ꂽ�p�t�H�[�}���X�^�C�}�[���~���܂��B
	//  @param[in]		label	The category of the timer. // �^�C�}�[�̃J�e�S���B
	virtual void doStopTimer(const rcTimerLabel /*label*/) {}

	// Returns the total accumulated time of the specified performance timer.
	// �w�肵���p�t�H�[�}���X�^�C�}�[�̍��v�ݐώ��Ԃ�Ԃ��܂��B
	//  @param[in]		label	The category of the timer. // �^�C�}�[�̃J�e�S���B
	//  @return The accumulated time of the timer, or -1 if timers are disabled or the timer has never been started.
	//  �^�C�}�[�̗ݐώ��ԁB�^�C�}�[�������ɂȂ��Ă��邩�A�^�C�}�[���J�n����Ă��Ȃ��ꍇ��-1�B
	virtual int doGetAccumulatedTime(const rcTimerLabel /*label*/) const { return -1; }

	// True if logging is enabled.
	// ���O���L���ȏꍇ��True�B
	bool m_logEnabled;

	// True if the performance timers are enabled.
	// �p�t�H�[�}���X�^�C�}�[���L���ɂȂ��Ă���ꍇ��true�B
	bool m_timerEnabled;
};

// A helper to first start a timer and then stop it when this helper goes out of scope.
// �ŏ��Ƀ^�C�}�[���J�n���A���̃w���p�[���͈͊O�ɂȂ����Ƃ��Ƀ^�C�}�[���~����w���p�[�B
// @see rcContext
class rcScopedTimer
{
public:
	// Constructs an instance and starts the timer.
	// �C���X�^���X���\�z���A�^�C�}�[���J�n���܂��B
	//  @param[in]		ctx		The context to use. �g�p����R���e�L�X�g�B
	//  @param[in]		label	The category of the timer. �^�C�}�[�̃J�e�S���[�B
	inline rcScopedTimer(rcContext* ctx, const rcTimerLabel label) : m_ctx(ctx), m_label(label) { m_ctx->startTimer(m_label); }
	inline ~rcScopedTimer() { m_ctx->stopTimer(m_label); }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcScopedTimer(const rcScopedTimer&) = delete;
	rcScopedTimer& operator=(const rcScopedTimer&) = delete;
	rcScopedTimer(rcScopedTimer&&) noexcept = delete;
	rcScopedTimer& operator=(rcScopedTimer&&) noexcept = delete;

	rcContext* const m_ctx;
	const rcTimerLabel m_label;
};

// Specifies a configuration to use when performing Recast builds.
// ���L���X�g�r���h�����s����Ƃ��Ɏg�p����\�����w�肵�܂��B
// @ingroup recast
struct rcConfig
{
	// The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
	// x���ɉ������t�B�[���h�̕��B[�����F> = 0] [�P�ʁFvx]
	int width;

	// The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
	// z���ɉ������t�B�[���h�̍����B[�����F> = 0] [�P�ʁFvx]
	int height;

	// The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx]
	// xz���ʏ�̃^�C���̕�/�����̃T�C�Y�B[�����F> = 0] [�P�ʁFvx]
	int tileSize;

	// The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
	// �n�`�̎���̃i�r�Q�[�g�ł��Ȃ����E�̃T�C�Y�B[�����F> = 0] [�P�ʁFvx]
	int borderSize;

	// The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu]
	// �t�B�[���h�Ɏg�p����xz���ʂ̃Z���T�C�Y�B[�����F> 0] [�P�ʁFwu]
	float cs;

	// The y-axis cell size to use for fields. [Limit: > 0] [Units: wu]
	// �t�B�[���h�Ɏg�p����y���̃Z���T�C�Y�B[�����F> 0] [�P�ʁFwu]
	float ch;

	// The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	// �t�B�[���h��AABB�̍ŏ����E�B[�ix�Ay�Az�j] [�P�ʁFwu]
	std::array<float, 3> bmin;

	// The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
	// �t�B�[���h��AABB�̍ő勫�E�B[�ix�Ay�Az�j] [�P�ʁFwu]
	std::array<float, 3> bmax;

	// The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees]
	// ���s�\�ƌ��Ȃ����ő���z�B[�����F0 <=�l<90] [�P�ʁF�x]
	float walkableSlopeAngle;

	// Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3] [Units: vx]
	// ���ʐς����s�\�ƌ��Ȃ����悤�ɂ���ŏ�������u�V��v�܂ł̍����B [�����F> = 3] [�P�ʁFvx]
	int walkableHeight;

	// Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx]
	// �܂��ʉ߉\�ł���ƌ��Ȃ����ő�I�̍����B[�����F> = 0] [�P�ʁFvx]
	int walkableClimb;

	// The distance to erode/shrink the walkable area of the heightfield away from obstructions.  [Limit: >=0] [Units: vx]
	// �n�`�̕��s�\�̈����Q������N�H/���k���鋗���B [�����F> = 0] [�P�ʁFvx]
	int walkableRadius;

	// The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx]
	// ���b�V���̋��E�ɉ������֊s�G�b�W�̍ő勖�e���B[�����F> = 0] [�P�ʁFvx]
	int maxEdgeLen;

	// The maximum distance a simplfied contour's border edges should deviate the original raw contour. [Limit: >=0] [Units: vx]
	// �P�������ꂽ�֊s�̋��E�G�b�W�̍ő勗���́A���̐��̗֊s�����E����͂��ł��B [�����F> = 0] [�P�ʁFvx]
	float maxSimplificationError;

	// The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx]
	// �Ǘ��������̗̈���`���ł���Z���̍ŏ����B[�����F> = 0] [�P�ʁFvx]
	int minRegionArea;

	// Any regions with a span count smaller than this value will, if possible,
	// be merged with larger regions. [Limit: >=0] [Units: vx]
	// �\�ȏꍇ�A�X�p���J�E���g�����̒l�����������̈�́A���傫�ȗ̈�ƃ}�[�W����܂��B [�����F> = 0] [�P�ʁFvx]
	int mergeRegionArea;

	// The maximum number of vertices allowed for polygons generated during the
	// contour to polygon conversion process. [Limit: >= 3]
	// �֊s����|���S���ւ̕ϊ��v���Z�X���ɐ��������|���S���ɋ�����钸�_�̍ő吔�B [�����F> = 3]
	int maxVertsPerPoly;

	// Sets the sampling distance to use when generating the detail mesh.
	// (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu]
	//�ڍ׃��b�V���𐶐�����Ƃ��Ɏg�p����T���v�����O������ݒ肵�܂��B�i�����̏ڍׂ̂݁j[�����F0�܂���> = 0.9] [�P�ʁFwu]
	float detailSampleDist;

	// The maximum distance the detail mesh surface should deviate from heightfield
	// data. (For height detail only.) [Limit: >=0] [Units: wu]
	// �ڍ׃��b�V���̕\�ʂ��n�`�f�[�^�����E����ő勗���B�i�����̏ڍׂ̂݁j[�����F> = 0] [�P�ʁFwu]
	float detailSampleMaxError;
};

// Defines the maximum value for rcSpan::smin and rcSpan::smax.
// rcSpan :: smin�����rcSpan :: smax�̍ő�l���`���܂��B
constexpr int RC_SPAN_MAX_HEIGHT = (1 << RC_SPAN_HEIGHT_BITS) - 1;

// The number of spans allocated per span spool.
// �X�p���X�v�[�����ƂɊ��蓖�Ă�ꂽ�X�p���̐��B
// @see rcSpanPool
constexpr int RC_SPANS_PER_POOL = 2048;

// Represents a span in a heightfield.
// �X�p���X�v�[�����ƂɊ��蓖�Ă�ꂽ�X�p���̐��B
// @see rcHeightfield
struct rcSpan
{
	// The lower limit of the span. [Limit: < #smax]
	// �X�p���̉����B[�����F<#smax]
	uint32_t smin : RC_SPAN_HEIGHT_BITS;

	// The upper limit of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT]
	// �X�p���̏���B[�����F<= #RC_SPAN_MAX_HEIGHT]
	uint32_t smax : RC_SPAN_HEIGHT_BITS;

	// The area id assigned to the span.
	// �X�p���Ɋ��蓖�Ă�ꂽ�G���AID�B
	uint32_t area : 6;

	// The next span higher up in column.
	// ��̏�̎��̃X�p���B
	rcSpan* next;
};

// A memory pool used for quick allocation of spans within a heightfield.
// �n�`���̃X�p���̐v���Ȋ��蓖�ĂɎg�p����郁�����v�[���B
// @see rcHeightfield
struct rcSpanPool
{
	rcSpanPool* next;					// The next span pool. // ���̃X�p���v�[���B
	std::array<rcSpan, RC_SPANS_PER_POOL> items;	// Array of spans in the pool. // �v�[�����̃X�p���̔z��B
};

// A dynamic heightfield representing obstructed space.
// �Ղ�ꂽ��Ԃ�\�����I�Ȓn�`�B
// @ingroup recast
struct rcHeightfield
{
	rcHeightfield();
	~rcHeightfield();

	// The width of the heightfield. (Along the x-axis in cell units.)
	// �n�`�̕��B�i�Z���P�ʂ�x���ɉ����āB�j
	int width;

	// The height of the heightfield. (Along the z-axis in cell units.)
	// �n�`�̍����B�i�Z���P�ʂ�z���ɉ����āB�j
	int height;

	// The minimum bounds in world space. [(x, y, z)]
	// ���[���h��Ԃ̍ŏ����E�B[�ix�Ay�Az�j]
	std::array<float, 3> bmin;

	// The maximum bounds in world space. [(x, y, z)]
	// ���[���h��Ԃ̍ő勫�E�B[�ix�Ay�Az�j]
	std::array<float, 3> bmax;

	// The size of each cell. (On the xz-plane.)
	// �e�Z���̃T�C�Y�B�ixz���ʏ�B�j
	float cs;

	// The height of each cell. (The minimum increment along the y-axis.)
	// �e�Z���̍����B�iy���ɉ������ŏ������B�j
	float ch;

	// Heightfield of spans (width*height).
	// �X�p���̒n�`�iwidth * height�j�B
	rcSpan** spans;

	// Linked list of span pools.
	// �X�p���v�[���̃����N���X�g�B
	rcSpanPool* pools;

	// The next free span.
	// ���̋󂫃X�p���B
	rcSpan* freelist;

private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	// �����I�ɖ��������ꂽ�R�s�[�R���X�g���N�^�[�ƃR�s�[���蓖�ĉ��Z�q�B
	rcHeightfield(const rcHeightfield&) = delete;
	rcHeightfield& operator=(const rcHeightfield&) = delete;
};

// Provides information on the content of a cell column in a compact heightfield.
// �R���p�N�g�Ȓn�`�̃Z����̃R���e���c�Ɋւ������񋟂��܂��B
struct rcCompactCell
{
	uint32_t index : 24;	// Index to the first span in the column. // ��̍ŏ��̃X�p���ւ̃C���f�b�N�X�B
	uint32_t count : 8;		// Number of spans in the column. // ����̃X�p���̐��B
};

// Represents a span of unobstructed space within a compact heightfield.
// �R���p�N�g�Ȓn�`���̎Ղ���̂̂Ȃ���Ԃ͈̔͂�\���܂��B
struct rcCompactSpan
{
	// The lower extent of the span. (Measured from the heightfield's base.)
	// �X�p���̉����B�i�n�C�g�t�B�[���h�̃x�[�X���瑪��B�j
	uint16_t y;

	// The id of the region the span belongs to. (Or zero if not in a region.)
	// �X�p����������̈��ID�B�i�܂��́A�n��ɂȂ��ꍇ�̓[���B�j
	uint16_t reg;

	// Packed neighbor connection data.
	// �p�b�N���ꂽ�l�C�o�[�ڑ��f�[�^�B
	uint32_t con : 24;

	// The height of the span.  (Measured from #y.)
	// �X�p���̍����B�i#y���瑪��j
	uint32_t h : 8;
};

// �Ղ��Ă��Ȃ���Ԃ�\���R���p�N�g�ŐÓI�Ȓn�`�B
// A compact, static heightfield representing unobstructed space.
// @ingroup recast
struct rcCompactHeightfield
{
	// The width of the heightfield. (Along the x-axis in cell units.)
	// �n�`�̕��B�i�Z���P�ʂ�x���ɉ����āj
	int width;

	// The height of the heightfield. (Along the z-axis in cell units.)
	// �n�`�̍����B�i�Z���P�ʂ�z���ɉ����āj
	int height;

	// The number of spans in the heightfield.
	// �n�`�̃X�p��(�l�Ԃ̎����Ƃ��������̒P��)�̐�
	int spanCount;

	// The walkable height used during the build of the field.  (See: rcConfig::walkableHeight)
	// �t�B�[���h�̍\�z���Ɏg�p�������s�\�ȍ���
	int walkableHeight;

	// The walkable climb used during the build of the field. (See: rcConfig::walkableClimb)
	// �t�B�[���h�̍\�z���Ɏg�p�������s�\�ȏ㏸
	int walkableClimb;

	// The AABB border size used during the build of the field. (See: rcConfig::borderSize)
	// �t�B�[���h�̃r���h���Ɏg�p�����AABB���E�T�C�Y
	int borderSize;

	// The maximum distance value of any span within the field.
	// �t�B�[���h���̔C�ӂ̃X�p���̍ő勗���l
	uint16_t maxDistance;

	// The maximum region id of any span within the field.
	// �t�B�[���h���̔C�ӂ̃X�p���̍ő�̈�ID
	uint16_t maxRegions;

	// The minimum bounds in world space. [(x, y, z)]
	// ���[���h��Ԃ̍ŏ����E�B[�ix�Ay�Az�j]
	std::array<float, 3> bmin;

	// The maximum bounds in world space. [(x, y, z)]
	// ���[���h��Ԃ̍ő勫�E�B[�ix�Ay�Az�j]
	std::array<float, 3> bmax;

	// The size of each cell. (On the xz-plane.)
	// �e�Z���̃T�C�Y�B�ixz���ʏ�B�j
	float cs;

	// The height ofeach cell. (The minimum increment along the y-axis.)
	// �e�Z���̍����B�iy���ɉ������ŏ������B�j
	float ch;

	// Array of cells. [Size: #width*#height]
	// �Z���̔z��
	rcCompactCell* cells;

	// Array of spans. [Size: #spanCount]
	// �X�p���̔z��
	rcCompactSpan* spans;

	// Array containing border distance data. [Size: #spanCount]
	// ���E�����f�[�^���܂ޔz��
	uint16_t* dist;

	// Array containing area id data. [Size: #spanCount]
	// �G���AID�f�[�^���܂ޔz��
	uint8_t* areas;
};

// Represents a heightfield layer within a layer set.
// ���C���[�Z�b�g���̒n�`���C���[��\���܂��B
// @see rcHeightfieldLayerSet
struct rcHeightfieldLayer
{
	// The minimum bounds in world space. [(x, y, z)]
	// ���[���h��Ԃ̍ŏ����E�B[�ix�Ay�Az�j]
	std::array<float, 3> bmin;

	// The maximum bounds in world space. [(x, y, z)]
	// ���[���h��Ԃ̍ő勫�E�B[�ix�Ay�Az�j]
	std::array<float, 3> bmax;

	// The size of each cell. (On the xz-plane.)
	// �e�Z���̃T�C�Y�B�ixz���ʏ�j
	float cs;

	// The height of each cell. (The minimum increment along the y-axis.)
	// �e�Z���̍����B�iy���ɉ������ŏ������j
	float ch;

	// The width of the heightfield. (Along the x-axis in cell units.)
	// �n�`�̕��B�i�Z���P�ʂ�x���ɉ����j
	int width;

	// The height of the heightfield. (Along the z-axis in cell units.)
	// �n�`�̍����B�i�Z���P�ʂ�z���ɉ����j
	int height;

	// The minimum x-bounds of usable data.
	// �g�p�\�ȃf�[�^�̍ŏ�x���E�B
	int minx;

	// The maximum x-bounds of usable data.
	// �g�p�\�ȃf�[�^�̍ő�x���E�B
	int maxx;

	// The minimum y-bounds of usable data. (Along the z-axis.)
	// �g�p�\�ȃf�[�^�̍ŏ�y���E�B�iz���ɉ����j
	int miny;

	// The maximum y-bounds of usable data. (Along the z-axis.)
	// �g�p�\�ȃf�[�^�̍ő�y���E�B�iz���ɉ����j
	int maxy;

	// The minimum height bounds of usable data. (Along the y-axis.)
	// �g�p�\�ȃf�[�^�̍ŏ��̍����̋��E�B�iy���ɉ����j
	int hmin;

	// The maximum height bounds of usable data. (Along the y-axis.)
	// �g�p�\�ȃf�[�^�̍ő卂���̋��E�B�iy���ɉ����j
	int hmax;

	// The heightfield. [Size: width * height]
	// �n�`�B[�T�C�Y�F�� * ����]
	uint8_t* heights;

	// Area ids. [Size: Same as #heights]
	// �G���AID�B[�T�C�Y�F#heights�Ɠ���]
	uint8_t* areas;

	// Packed neighbor connection information. [Size: Same as #heights]
	// �p�b�N���ꂽ�l�C�o�[�ڑ����B[�T�C�Y�F#heights�Ɠ���]
	uint8_t* cons;
};

// Represents a set of heightfield layers.
// �n�`���C���[�̃Z�b�g��\���܂��B
// @ingroup recast
// @see rcAllocHeightfieldLayerSet, rcFreeHeightfieldLayerSet
struct rcHeightfieldLayerSet
{
	rcHeightfieldLayer* layers;			// The layers in the set. [Size: #nlayers] // �Z�b�g���̃��C���[�B[�T�C�Y�F#nlayers]
	int nlayers;						// The number of layers in the set. // �Z�b�g���̃��C���[�̐��B
};

// Represents a simple, non-overlapping contour in field space.
// �t�B�[���h��ԓ��̒P���ȏd�����Ȃ��֊s��\���܂��B
struct rcContour
{
	// Simplified contour vertex and connection data. [Size: 4 * #nverts]
	// �ȗ������ꂽ�֊s�̒��_�Ɛڑ��f�[�^�B[�T�C�Y�F4 * #nverts]
	int* verts;

	// The number of vertices in the simplified contour.
	// �P�������ꂽ�֊s�̒��_�̐��B
	int nverts;

	// Raw contour vertex and connection data. [Size: 4 * #nrverts]
	// ���̗֊s�̒��_�Ɛڑ��f�[�^�B[�T�C�Y�F4 * #nrverts]
	int* rverts;

	// The number of vertices in the raw contour.
	// ���̗֊s�̒��_�̐��B
	int nrverts;

	// The region id of the contour.
	// �֊s�̗̈�ID�B
	uint16_t reg;

	// The area id of the contour.
	// �֊s�̃G���AID�B
	uint8_t area;
};

// Represents a group of related contours.
// �֘A����֊s�̃O���[�v��\���܂��B
// @ingroup recast
struct rcContourSet
{
	// An array of the contours in the set. [Size: #nconts]
	// �Z�b�g���̗֊s�̔z��B[�T�C�Y�F#nconts]
	rcContour* conts;

	// The number of contours in the set.
	// �Z�b�g���̗֊s�̐��B
	int nconts;

	// The minimum bounds in world space. [(x, y, z)]
	// ���[���h��Ԃ̍ŏ����E�B[�ix�Ay�Az�j]
	std::array<float, 3> bmin;

	// The maximum bounds in world space. [(x, y, z)]
	// ���[���h��Ԃ̍ő勫�E�B[�ix�Ay�Az�j]
	std::array<float, 3> bmax;

	// The size of each cell. (On the xz-plane.)
	// �e�Z���̃T�C�Y�B�ixz���ʏ�j
	float cs;

	// The height of each cell. (The minimum increment along the y-axis.)
	// �e�Z���̍����B�iy���ɉ������ŏ������j
	float ch;

	// The width of the set. (Along the x-axis in cell units.)
	// �Z�b�g�̕��B�i�Z���P�ʂ�x���ɉ����j
	int width;

	// The height of the set. (Along the z-axis in cell units.)
	// �Z�b�g�̍����B�i�Z���P�ʂ�z���ɉ����j
	int height;

	// The AABB border size used to generate the source data from which the contours were derived.
	// �֊s�̔h�����̃\�[�X�f�[�^�𐶐����邽�߂Ɏg�p�����AABB���E�T�C�Y�B
	int borderSize;

	// The max edge error that this contour set was simplified with.
	// ���̗֊s�Z�b�g���P�������ꂽ�ő�G�b�W�G���[�B
	float maxError;
};

// Represents a polygon mesh suitable for use in building a navigation mesh.
// �i�r�Q�[�V�������b�V���̍\�z�Ɏg�p����̂ɓK�����|���S�����b�V����\���܂��B
// @ingroup recast
struct rcPolyMesh
{
	// The mesh vertices. [Form: (x, y, z) * #nverts]
	// ���b�V���̒��_�B[�`���F�ix�Ay�Az�j* #nverts]
	uint16_t* verts;

	// Polygon and neighbor data. [Length: #maxpolys * 2 * #nvp]
	// �|���S���ƋߖT�f�[�^�B[�����F#maxpolys * 2 * #nvp]
	uint16_t* polys;

	// The region id assigned to each polygon. [Length: #maxpolys]
	// �e�|���S���Ɋ��蓖�Ă�ꂽ���[�W����ID�B[�����F#maxpolys]
	uint16_t* regs;

	// The user defined flags for each polygon. [Length: #maxpolys]
	// �e�|���S���̃��[�U�[��`�t���O�B[�����F#maxpolys]
	uint16_t* flags;

	// The area id assigned to each polygon. [Length: #maxpolys]
	// �e�|���S���Ɋ��蓖�Ă�ꂽ�G���AID�B[�����F#maxpolys]
	uint8_t* areas;

	// The number of vertices.
	// ���_�̐��B
	int nverts;

	// The number of polygons.
	// �|���S���̐��B
	int npolys;

	// The number of allocated polygons.
	// ���蓖�Ă�ꂽ�|���S���̐��B
	int maxpolys;

	// The maximum number of vertices per polygon.
	// �|���S�����Ƃ̒��_�̍ő吔�B
	int nvp;

	// The minimum bounds in world space. [(x, y, z)]
	// ���[���h��Ԃ̍ŏ����E�B[�ix�Ay�Az�j]
	std::array<float, 3> bmin;

	// The maximum bounds in world space. [(x, y, z)]
	// ���[���h��Ԃ̍ő勫�E�B[�ix�Ay�Az�j]
	std::array<float, 3> bmax;

	// The size of each cell. (On the xz-plane.)
	// �e�Z���̃T�C�Y�B�ixz���ʏ�B�j
	float cs;

	// The height of each cell. (The minimum increment along the y-axis.)
	// �e�Z���̍����B�iy���ɉ������ŏ������B�j
	float ch;

	// The AABB border size used to generate the source data from which the mesh was derived.
	// ���b�V���̔h�����̃\�[�X�f�[�^�𐶐����邽�߂Ɏg�p�����AABB���E�T�C�Y�B
	int borderSize;

	// The max error of the polygon edges in the mesh.
	// ���b�V�����̃|���S���G�b�W�̍ő�덷�B
	float maxEdgeError;
};

// Contains triangle meshes that represent detailed height data associated
// with the polygons in its associated polygon mesh object.
// �֘A�t����ꂽ�|���S�����b�V���I�u�W�F�N�g���̃|���S���Ɋ֘A�t����ꂽ�ڍׂȍ����f�[�^��\���O�p�`���b�V�����܂܂�܂��B
// @ingroup recast
struct rcPolyMeshDetail
{
	// The sub-mesh data. [Size: 4*#nmeshes]
	// �T�u���b�V���f�[�^�B[�T�C�Y�F4 *��nmeshes]
	uint32_t* meshes;

	// The mesh vertices. [Size: 3*#nverts]
	// ���b�V���̒��_�B[�T�C�Y�F3 *��nverts]
	float* verts;

	// The mesh triangles. [Size: 4*#ntris]
	// ���b�V���̎O�p�`�B[�T�C�Y�F4 *��ntris]
	uint8_t* tris;

	// The number of sub-meshes defined by #meshes.
	// #meshes�Œ�`���ꂽ�T�u���b�V���̐��B
	int nmeshes;

	// The number of vertices in #verts.
	// #verts�̒��_�̐��B
	int nverts;

	// The number of triangles in #tris.
	// #tris�̎O�p�`�̐��B
	int ntris;
};

constexpr uint8_t RC_AREA_FLAGS_MASK = 0x3F;

// @ingroup recast
class rcAreaModification
{
public:
	// Mask is set to all available bits, which means value is fully applied
	// �}�X�N�͎g�p�\�Ȃ��ׂẴr�b�g�ɐݒ肳��܂��B�܂�A�l�͊��S�ɓK�p����܂�
	//  @param[in] value	The area id to apply. [Limit: <= #RC_AREA_FLAGS_MASK]
	//  @param [in] value�K�p����G���AID�B [�����F<= #RC_AREA_FLAGS_MASK]
	rcAreaModification(uint8_t value);
	//  @param[in] value	The area id to apply. [Limit: <= #RC_AREA_FLAGS_MASK]
	//  @param [in] value�K�p����G���AID�B [�����F<= #RC_AREA_FLAGS_MASK]
	//  @param[in] mask	Bitwise mask used when applying value. [Limit: <= #RC_AREA_FLAGS_MASK]
	//  @param [in] mask�l��K�p����Ƃ��Ɏg�p�����r�b�g�P�ʂ̃}�X�N�B [�����F<= #RC_AREA_FLAGS_MASK]
	rcAreaModification(uint8_t value, uint8_t mask);
	rcAreaModification(const rcAreaModification& other);
	void operator = (const rcAreaModification& other);
	bool operator == (const rcAreaModification& other) const;
	bool operator != (const rcAreaModification& other) const;
	void apply(uint8_t& area) const;
	uint8_t getMaskedValue() const;

	uint8_t m_value;	// Value to apply to target area id // �^�[�Q�b�g�G���AID�ɓK�p����l
	uint8_t m_mask;	// Bitwise mask used when applying value to target area id //�^�[�Q�b�g�G���AID�ɒl��K�p����Ƃ��Ɏg�p�����r�b�g�P�ʂ̃}�X�N
};

// @name Allocation Functions
// Functions used to allocate and de-allocate Recast objects.
// Recast�I�u�W�F�N�g�̊��蓖�ĂƊ��蓖�ĉ����Ɏg�p�����֐��B
// @see rcAllocSetCustom
// @{
// Allocates a heightfield object using the Recast allocator.
// Recast�A���P�[�^�[���g�p���āAheightfield�I�u�W�F�N�g�����蓖�Ă܂��B
//  @return A heightfield that is ready for initialization, or null on failure.
//	�������̏������ł��Ă��鍂���t�B�[���h�A�܂��͎��s�����ꍇ��null�B
//  @ingroup recast
//  @see rcCreateHeightfield, rcFreeHeightField
rcHeightfield* rcAllocHeightfield();

// Frees the specified heightfield object using the Recast allocator.
// Recast�A���P�[�^�[���g�p���āA�w�肳�ꂽheightfield�I�u�W�F�N�g��������܂��B
//  @param[in]		hf	A heightfield allocated using #rcAllocHeightfield
//  #rcAllocHeightfield���g�p���Ċ��蓖�Ă�ꂽ�����t�B�[���h
//  @ingroup recast
//  @see rcAllocHeightfield
void rcFreeHeightField(rcHeightfield* hf);

// Allocates a compact heightfield object using the Recast allocator.
// Recast�A���P�[�^�[���g�p���āA�R���p�N�g��heightfield�I�u�W�F�N�g�����蓖�Ă܂��B
// @return A compact heightfield that is ready for initialization, or null on failure.
// �������̏������ł��Ă���R���p�N�g�ȍ����t�B�[���h�A�܂��͎��s�����ꍇ��null�B
// @ingroup recast
// @see rcBuildCompactHeightfield, rcFreeCompactHeightfield
rcCompactHeightfield* rcAllocCompactHeightfield();

// Frees the specified compact heightfield object using the Recast allocator.
// Recast�A���P�[�^�[���g�p���āA�w�肳�ꂽ�R���p�N�g��heightfield�I�u�W�F�N�g��������܂��B
//  @param[in]		chf		A compact heightfield allocated using #rcAllocCompactHeightfield
//  #rcAllocCompactHeightfield���g�p���Ċ��蓖�Ă�ꂽ�R���p�N�g�ȍ����t�B�[���h
//  @ingroup recast
//  @see rcAllocCompactHeightfield
void rcFreeCompactHeightfield(rcCompactHeightfield* chf);

// Allocates a heightfield layer set using the Recast allocator.
// Recast�A���P�[�^�[���g�p���āA�n�`���C���[�Z�b�g�����蓖�Ă܂��B
// @return A heightfield layer set that is ready for initialization, or null on failure.
// �������̏������ł��Ă���n�`���C���[�Z�b�g�A�܂��͎��s�����ꍇ��null�B
// @ingroup recast
// @see rcBuildHeightfieldLayers, rcFreeHeightfieldLayerSet
rcHeightfieldLayerSet* rcAllocHeightfieldLayerSet();

// Frees the specified heightfield layer set using the Recast allocator.
// Recast�A���P�[�^�[���g�p���āA�w�肳�ꂽheightfield���C���[�Z�b�g��������܂��B
//  @param[in]		lset	A heightfield layer set allocated using #rcAllocHeightfieldLayerSet
//  #rcAllocHeightfieldLayerSet���g�p���Ċ��蓖�Ă�ꂽ�����t�B�[���h���C���[�Z�b�g
//  @ingroup recast
//  @see rcAllocHeightfieldLayerSet
void rcFreeHeightfieldLayerSet(rcHeightfieldLayerSet* lset);

// Allocates a contour set object using the Recast allocator.
// Recast�A���P�[�^�[���g�p���ė֊s�Z�b�g�I�u�W�F�N�g�����蓖�Ă܂��B
//  @return A contour set that is ready for initialization, or null on failure.
//  �������̏������ł��Ă���֊s�Z�b�g�A�܂��͎��s�����ꍇ��null�B
//  @ingroup recast
//  @see rcBuildContours, rcFreeContourSet
rcContourSet* rcAllocContourSet();

// Frees the specified contour set using the Recast allocator.
// Recast�A���P�[�^�[���g�p���āA�w�肳�ꂽ�֊s�Z�b�g��������܂��B
//  @param[in]		cset	A contour set allocated using #rcAllocContourSet
//  @param[in]		#rcAllocContourSet���g�p���Ċ��蓖�Ă�ꂽ�֊s�Z�b�g
//  @ingroup recast
//  @see rcAllocContourSet
void rcFreeContourSet(rcContourSet* cset);

// Allocates a polygon mesh object using the Recast allocator.
// Recast�A���P�[�^�[���g�p���ă|���S�����b�V���I�u�W�F�N�g�����蓖�Ă܂��B
//  @return A polygon mesh that is ready for initialization, or null on failure.
//  �������̏������ł��Ă���|���S�����b�V���A�܂��͎��s�����ꍇ��null�B
//  @ingroup recast
//  @see rcBuildPolyMesh, rcFreePolyMesh
rcPolyMesh* rcAllocPolyMesh();

// Frees the specified polygon mesh using the Recast allocator.
// Recast�A���P�[�^�[���g�p���āA�w�肳�ꂽ�|���S�����b�V����������܂��B
//  @param[in]		pmesh	A polygon mesh allocated using #rcAllocPolyMesh
//  #rcAllocPolyMesh���g�p���Ċ��蓖�Ă�ꂽ�|���S�����b�V��
//  @ingroup recast
//  @see rcAllocPolyMesh
void rcFreePolyMesh(rcPolyMesh* pmesh);

// Allocates a detail mesh object using the Recast allocator.
// Recast�A���P�[�^�[���g�p���ďڍ׃��b�V���I�u�W�F�N�g�����蓖�Ă܂��B
//  @return A detail mesh that is ready for initialization, or null on failure.
//  �������̏������ł��Ă���ڍ׃��b�V���A�܂��͎��s�����ꍇ��null�B
//  @ingroup recast
//  @see rcBuildPolyMeshDetail, rcFreePolyMeshDetail
rcPolyMeshDetail* rcAllocPolyMeshDetail();

// Frees the specified detail mesh using the Recast allocator.
// Recast�A���P�[�^�[���g�p���āA�w�肳�ꂽ�ڍ׃��b�V����������܂��B
//  @param[in]		dmesh	A detail mesh allocated using #rcAllocPolyMeshDetail
//  #rcAllocPolyMeshDetail���g�p���Ċ��蓖�Ă�ꂽ�ڍ׃��b�V��
//  @ingroup recast
//  @see rcAllocPolyMeshDetail
void rcFreePolyMeshDetail(rcPolyMeshDetail* dmesh);

// @}

// Heighfield border flag.
// �n�`���E�t���O�B
// If a heightfield region ID has this bit set, then the region is a border region and its spans are considered unwalkable.
// �n�`�̗̈�ID�ɂ��̃r�b�g���ݒ肳��Ă���ꍇ�A���̗̈�͋��E�̈�ł���A���̃X�p���͕��s�s�\�ƌ��Ȃ���܂��B
// (Used during the region and contour build process.)
//�i�̈您��ї֊s�̍\�z�v���Z�X���Ɏg�p����܂��B�j
// @see rcCompactSpan::reg
constexpr uint16_t RC_BORDER_REG = 0x8000;

// Polygon touches multiple regions.
// �|���S���͕����̗̈�ɐڐG���܂��B
// If a polygon has this region ID it was merged with or created from polygons of different regions during the polymesh
// build step that removes redundant border vertices.
// �|���S���ɂ��̗̈�ID������ꍇ�A�璷�ȋ��E���_���폜����polymesh�r���h�X�e�b�v���ɁA�قȂ�̈�̃|���S���ƃ}�[�W�܂��͍쐬����܂����B
// (Used during the polymesh and detail polymesh build processes)
//�ipolymesh����яڍ�polymesh�r���h�v���Z�X���Ɏg�p�j
// @see rcPolyMesh::regs
constexpr uint16_t RC_MULTIPLE_REGS = 0;

// Border vertex flag.
// ���E���_�t���O�B
// If a region ID has this bit set, then the associated element lies on a tile border.
// �̈�ID�ɂ��̃r�b�g���ݒ肳��Ă���ꍇ�A�֘A����v�f�̓^�C���̋��E����ɂ���܂��B
// If a contour vertex's region ID has this bit set,
// �������̒��_�̗̈�ID�ɂ��̃r�b�g���ݒ肳��Ă���ꍇ�A
// the vertex will later be removed in order to match the segments and vertices at tile boundaries.
// �^�C���̋��E�ŃZ�O�����g�ƒ��_����v�����邽�߂ɁA��Œ��_���폜����܂��B
// (Used during the build process.)
//�i�r���h�v���Z�X���Ɏg�p����܂��B�j
// @see rcCompactSpan::reg, #rcContour::verts, #rcContour::rverts
constexpr int RC_BORDER_VERTEX = 0x10000;

// Area border flag.
// �G���A���E�t���O�B
// If a region ID has this bit set, then the associated element lies on the border of an area.
// �̈�ID�ɂ��̃r�b�g���ݒ肳��Ă���ꍇ�A�֘A����v�f�͗̈�̋��E�ɂ���܂��B
// (Used during the region and contour build process.)
//�i�̈您��ї֊s�̍\�z�v���Z�X���Ɏg�p����܂��B�j
// @see rcCompactSpan::reg, #rcContour::verts, #rcContour::rverts
constexpr int RC_AREA_BORDER = 0x20000;

// Contour build flags. // �֊s�\�z�t���O�B
// @see rcBuildContours
enum rcBuildContoursFlags
{
	// Tessellate solid (impassable) edges during contour simplification.
	// �֊s�̒P�������Ƀ\���b�h�i�ʉ߂ł��Ȃ��j�G�b�W���e�b�Z���[�V�������܂��B
	RC_CONTOUR_TESS_WALL_EDGES = 0x01,
	// Tessellate edges between areas during contour simplification.
	// �֊s�̒P�������ɗ̈�Ԃ̃G�b�W���e�b�Z���[�V�������܂��B
	RC_CONTOUR_TESS_AREA_EDGES = 0x02,
};

// Applied to the region id field of contour vertices in order to extract the region id.
// �̈�ID�𒊏o���邽�߂ɁA�֊s���_�̗̈�ID�t�B�[���h�ɓK�p����܂��B
// The region id field of a vertex may have several flags applied to it.
// ���_�̗̈�ID�t�B�[���h�ɂ́A�������̃t���O���K�p�����ꍇ������܂��B
// So the fields value can't be used directly.
// ���̂��߁A�t�B�[���h�̒l�𒼐ڎg�p���邱�Ƃ͂ł��܂���B
// @see rcContour::verts, rcContour::rverts
constexpr int RC_CONTOUR_REG_MASK = 0xffff;

// An value which indicates an invalid index within a mesh.
// ���b�V�����̖����ȃC���f�b�N�X�������l�B
// @note This does not necessarily indicate an error.
// ����͕K�������G���[�������Ă���킯�ł͂���܂���B
// @see rcPolyMesh::polys
constexpr uint16_t RC_MESH_NULL_IDX = 0xffff;

// Represents the null area. // null�G���A��\���܂��B
// When a data element is given this value it is considered to no longer be
// assigned to a usable area.  (E.g. It is unwalkable.)
//�f�[�^�v�f�ɂ��̒l���^������ƁA�g�p�\�ȗ̈�Ɋ��蓖�Ă��Ȃ��Ȃ����ƌ��Ȃ���܂��B(��F���s�s�\�j
constexpr uint8_t RC_NULL_AREA = 0;

// The default area id used to indicate a walkable polygon.
// This is also the maximum allowed area id, and the only non-null area id
// recognized by some steps in the build process.
// �f�t�H���g�̗̈�́A���s�\�ȃ|���S�����������߂Ɏg�p����܂��B
// ����́A�������ő�G���AID�ł���A�r���h�v���Z�X�̂������̃X�e�b�v�ŔF�������B��̔�k���G���AID�ł��B
constexpr uint8_t RC_WALKABLE_AREA = 63;

// The value returned by #rcGetCon if the specified direction is not connected
// to another span. (Has no neighbor.)
// �w�肳�ꂽ�������ʂ̃X�p���ɐڑ�����Ă��Ȃ��ꍇ��#rcGetCon�ɂ���ĕԂ����l�B(�t�߂ɑ��݂��܂���j
constexpr int RC_NOT_CONNECTED = 0x3f;

// @name General helper functions
// @{
// Used to ignore a function parameter. VS complains about unused parameters and this silences the warning.
// �֐��p�����[�^�[�𖳎����邽�߂Ɏg�p����܂��B VS�͖��g�p�̃p�����[�^�[�ɂ��ĕs���������Ă���A����͌x����ق点�܂��B
//  @param [in] _ Unused parameter
template<class T> void rcIgnoreUnused(const T&) { }

// Swaps the values of the two parameters.
// 2�̃p�����[�^�[�̒l���������܂��B
//  @param[in,out]	a	Value A
//  @param[in,out]	b	Value B
template<class T> inline void rcSwap(T& a, T& b) { std::swap(a, b); }

// Returns the minimum of two values.
// �ŏ���2�̒l��Ԃ��܂��B
//  @param[in]		a	Value A
//  @param[in]		b	Value B
//  @return The minimum of the two values.
template<class T> inline constexpr T rcMin(T a, T b) { return (std::min)(a, b); }

// Returns the maximum of two values.
// �ő�2�̒l��Ԃ��܂��B
//  @param[in]		a	Value A
//  @param[in]		b	Value B
//  @return The maximum of the two values.
template<class T> inline constexpr T rcMax(T a, T b) { return (std::max)(a, b); }

// Returns the absolute value.
// ��Βl��Ԃ��܂��B
//  @param[in]		a	The value.
//  @return The absolute value of the specified value.
template<class T> inline constexpr T rcAbs(T a) { return a < 0 ? -a : a; }

// Returns the square of the value.
// �l�̓���Ԃ��܂��B
//  @param[in]		a	The value.
//  @return The square of the value.
template<class T> inline constexpr T rcSqr(T a) { return a * a; }

// Clamps the value to the specified range.
// �w�肵���͈͂ɒl���N�����v���܂��B
//  @param[in]		v	The value to clamp.
//  @param[in]		mn	The minimum permitted return value.
//  @param[in]		mx	The maximum permitted return value.
//  @return The value, clamped to the specified range.
template<class T> inline constexpr T rcClamp(T v, T mn, T mx) { return v < mn ? mn : (v > mx ? mx : v); }

// Returns the square root of the value.
// �l�̕�������Ԃ��܂��B
//  @param[in]		x	The value.
//  @return The square root of the vlaue.
inline float rcSqrt(float x)
{
	return sqrtf(x);
}

// @}
// @name Vector helper functions.
// @{

// Derives the cross product of two vectors. (@p v1 x @p v2)
// 2�̃x�N�g���̊O�ς𓱏o���܂��B
//  @param[out]	dest	The cross product. [(x, y, z)]
//  @param[in]		v1		A Vector [(x, y, z)]
//  @param[in]		v2		A vector [(x, y, z)]
inline constexpr void rcVcross(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[1] * v2[2] - v1[2] * v2[1];
	dest[1] = v1[2] * v2[0] - v1[0] * v2[2];
	dest[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// Derives the cross product of two vectors. (@p v1 x @p v2)
// 2�̃x�N�g���̊O�ς𓱏o���܂��B
//  @param[out]	dest	The cross product. [(x, y, z)]
//  @param[in]		v1		A Vector [(x, y, z)]
//  @param[in]		v2		A vector [(x, y, z)]
inline constexpr void rcVcross(
	std::array<float, 3>* dest, const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	dest->at(0) = v1[1] * v2[2] - v1[2] * v2[1];
	dest->at(1) = v1[2] * v2[0] - v1[0] * v2[2];
	dest->at(2) = v1[0] * v2[1] - v1[1] * v2[0];
}

// Derives the dot product of two vectors. (@p v1 . @p v2)
// 2�̃x�N�g���̃h�b�g�ς𓱏o���܂��B
//  @param[in]		v1	A Vector [(x, y, z)]
//  @param[in]		v2	A vector [(x, y, z)]
// @return The dot product.
inline constexpr float rcVdot(const float* v1, const float* v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// Derives the dot product of two vectors. (@p v1 . @p v2)
// 2�̃x�N�g���̃h�b�g�ς𓱏o���܂��B
//  @param[in]		v1	A Vector [(x, y, z)]
//  @param[in]		v2	A vector [(x, y, z)]
// @return The dot product.
inline constexpr float rcVdot(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
// �X�P�[�����O���ꂽ�x�N�g�����Z�����s���܂��B
//  @param[out]	dest	The result vector. [(x, y, z)]
//  ���ʃx�N�g���B[�ix�Ay�Az�j]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  �x�[�X�x�N�g���B[�ix�Ay�Az�j]
//  @param[in]		v2		The vector to scale and add to @p v1. [(x, y, z)]
//  �X�P�[�����O����@p v1�ɒǉ�����x�N�g���B[�ix�Ay�Az�j]
//  @param[in]		s		The amount to scale @p v2 by before adding to @p v1.
//  v1�ɒǉ�����O��@p v2���X�P�[�����O����ʁB
inline constexpr void rcVmad(float* dest, const float* v1, const float* v2, const float s)
{
	dest[0] = v1[0] + v2[0] * s;
	dest[1] = v1[1] + v2[1] * s;
	dest[2] = v1[2] + v2[2] * s;
}

// Performs a scaled vector addition. (@p v1 + (@p v2 * @p s))
// �X�P�[�����O���ꂽ�x�N�g�����Z�����s���܂��B
//  @param[out]	dest	The result vector. [(x, y, z)]
//  ���ʃx�N�g���B[�ix�Ay�Az�j]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  �x�[�X�x�N�g���B[�ix�Ay�Az�j]
//  @param[in]		v2		The vector to scale and add to @p v1. [(x, y, z)]
//  �X�P�[�����O����@p v1�ɒǉ�����x�N�g���B[�ix�Ay�Az�j]
//  @param[in]		s		The amount to scale @p v2 by before adding to @p v1.
//  v1�ɒǉ�����O��@p v2���X�P�[�����O����ʁB
inline constexpr void rcVmad(
	std::array<float, 3>* dest, const std::array<float, 3>& v1, const std::array<float, 3>& v2, const float s)
{
	dest->at(0) = v1[0] + v2[0] * s;
	dest->at(1) = v1[1] + v2[1] * s;
	dest->at(2) = v1[2] + v2[2] * s;
}

// Performs a vector addition. (@p v1 + @p v2)
// �x�N�g���̉��Z�����s���܂��B�i@p v1 + @p v2�j
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  @param[in]		v2		The vector to add to @p v1. [(x, y, z)]
inline constexpr void rcVadd(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0] + v2[0];
	dest[1] = v1[1] + v2[1];
	dest[2] = v1[2] + v2[2];
}

// Performs a vector subtraction. (@p v1 - @p v2)
// �x�N�g�����Z�����s���܂��B�i@p v1-@p v2�j
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		v1		The base vector. [(x, y, z)]
//  @param[in]		v2		The vector to subtract from @p v1. [(x, y, z)]
inline constexpr void rcVsub(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0] - v2[0];
	dest[1] = v1[1] - v2[1];
	dest[2] = v1[2] - v2[2];
}

namespace RcOperator
{
	using ArrayF = std::array<float, 3>;

	// Performs a vector addition. (@p v1 + @p v2)
	// �x�N�g���̉��Z�����s���܂��B �iv1 + v2�j
	//  @param[out]	dest	The result vector. [(x, y, z)]
	//  @param[in]		v1		The base vector. [(x, y, z)]
	//  @param[in]		v2		The vector to add to @p v1. [(x, y, z)]
	inline auto operator+(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		std::array<float, 3> dest;

		dest[0] = v1[0] + v2[0];
		dest[1] = v1[1] + v2[1];
		dest[2] = v1[2] + v2[2];

		return dest;
	}

	inline void operator+=(std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		v1[0] += v2[0];
		v1[1] += v2[1];
		v1[2] += v2[2];
	}

	// Performs a vector subtraction. (@p v1 - @p v2)
	// �x�N�g�����Z�����s���܂��B�iv1 - v2�j
	//  @param[out]	dest	The result vector. [(x, y, z)]
	//  @param[in]		v1		The base vector. [(x, y, z)]
	//  @param[in]		v2		The vector to subtract from @p v1. [(x, y, z)]
	inline auto operator-(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		std::array<float, 3> dest;

		dest[0] = v1[0] - v2[0];
		dest[1] = v1[1] - v2[1];
		dest[2] = v1[2] - v2[2];

		return dest;
	}

	inline void operator-=(std::array<float, 3>& v1, const std::array<float, 3>& v2)
	{
		v1[0] -= v2[0];
		v1[1] -= v2[1];
		v1[2] -= v2[2];
	}

	inline auto operator*(const std::array<float, 3>& v1, const float num)
	{
		std::array<float, 3> dest;

		dest[0] = v1[0] * num;
		dest[1] = v1[1] * num;
		dest[2] = v1[2] * num;

		return dest;
	}

	inline void operator*=(std::array<float, 3>& v1, const float num)
	{
		v1[0] *= num;
		v1[1] *= num;
		v1[2] *= num;
	}
}

// Selects the minimum value of each element from the specified vectors.
// �w�肳�ꂽ�x�N�g������e�v�f�̍ŏ��l��I�����܂��B
//  @param[in,out]	mn	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]		v	A vector. [(x, y, z)]
inline constexpr void rcVmin(float* mn, const float* v)
{
	mn[0] = rcMin(mn[0], v[0]);
	mn[1] = rcMin(mn[1], v[1]);
	mn[2] = rcMin(mn[2], v[2]);
}

// Selects the minimum value of each element from the specified vectors.
// �w�肳�ꂽ�x�N�g������e�v�f�̍ŏ��l��I�����܂��B
//  @param[in,out]	mn	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]		v	A vector. [(x, y, z)]
inline constexpr void rcVmin(std::array<float, 3>* mn, const std::array<float, 3>& v)
{
	mn->at(0) = rcMin(mn->at(0), v[0]);
	mn->at(1) = rcMin(mn->at(1), v[1]);
	mn->at(2) = rcMin(mn->at(2), v[2]);
}

// Selects the maximum value of each element from the specified vectors.
// �w�肳�ꂽ�x�N�g������e�v�f�̍ő�l��I�����܂��B
//  @param[in,out]	mx	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]		v	A vector. [(x, y, z)]
inline constexpr void rcVmax(float* mx, const float* v)
{
	mx[0] = rcMax(mx[0], v[0]);
	mx[1] = rcMax(mx[1], v[1]);
	mx[2] = rcMax(mx[2], v[2]);
}

// Selects the maximum value of each element from the specified vectors.
// �w�肳�ꂽ�x�N�g������e�v�f�̍ő�l��I�����܂��B
//  @param[in,out]	mx	A vector.  (Will be updated with the result.) [(x, y, z)]
//  @param[in]		v	A vector. [(x, y, z)]
inline constexpr void rcVmax(std::array<float, 3>* mn, const std::array<float, 3>& v)
{
	mn->at(0) = rcMax(mn->at(0), v[0]);
	mn->at(1) = rcMax(mn->at(1), v[1]);
	mn->at(2) = rcMax(mn->at(2), v[2]);
}

// Performs a vector copy.
// �x�N�^�[�R�s�[�����s���܂��B
//  @param[out]	dest	The result. [(x, y, z)]
//  @param[in]		v		The vector to copy. [(x, y, z)]
inline constexpr void rcVcopy(float* dest, const float* v)
{
	dest[0] = v[0];
	dest[1] = v[1];
	dest[2] = v[2];
}

// Returns the distance between two points.
// 2�_�Ԃ̋�����Ԃ��܂��B
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The distance between the two points.
inline float rcVdist(const float* v1, const float* v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return rcSqrt(dx * dx + dy * dy + dz * dz);
}

// Returns the distance between two points.
// 2�_�Ԃ̋�����Ԃ��܂��B
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The distance between the two points.
inline float rcVdist(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return rcSqrt(dx * dx + dy * dy + dz * dz);
}

// Returns the square of the distance between two points.
// 2�_�Ԃ̋�����2���Ԃ��܂��B
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The square of the distance between the two points.
inline constexpr float rcVdistSqr(const float* v1, const float* v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return dx * dx + dy * dy + dz * dz;
}

// Returns the square of the distance between two points.
// 2�_�Ԃ̋�����2���Ԃ��܂��B
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The square of the distance between the two points.
inline constexpr float rcVdistSqr(const std::array<float, 3>& v1, const std::array<float, 3>& v2)
{
	float dx = v2[0] - v1[0];
	float dy = v2[1] - v1[1];
	float dz = v2[2] - v1[2];
	return dx * dx + dy * dy + dz * dz;
}

// Normalizes the vector.
// �x�N�g���𐳋K�����܂��B
//  @param[in,out]	v	The vector to normalize. [(x, y, z)]
inline void rcVnormalize(float* v)
{
	float d = 1.f / rcSqrt(rcSqr(v[0]) + rcSqr(v[1]) + rcSqr(v[2]));
	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

// Normalizes the vector.
// �x�N�g���𐳋K�����܂��B
//  @param[in,out]	v	The vector to normalize. [(x, y, z)]
inline void rcVnormalize(std::array<float, 3>* v)
{
	float d = 1.f / rcSqrt(rcSqr(v->at(0)) + rcSqr(v->at(1)) + rcSqr(v->at(2)));
	v->at(0) *= d;
	v->at(1) *= d;
	v->at(2) *= d;
}

// @}
// @name Heightfield Functions
// @see rcHeightfield
// @{
// Calculates the bounding box of an array of vertices.
// ���_�̔z��̋��E�{�b�N�X���v�Z���܂��B
//  @ingroup recast
//  @param[in]		verts	An array of vertices. [(x, y, z) * @p nv]
//  verts���_�̔z��B[�ix�Ay�Az�j* @p nv]
//  @param[in]		nv		The number of vertices in the @p verts array.
//  verts�z����̒��_�̐��B
//  @param[out]	bmin	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABB�̍ŏ����E�B[�ix�Ay�Az�j] [�P�ʁFwu]
//  @param[out]	bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABB�̍ő勫�E�B[�ix�Ay�Az�j] [�P�ʁFwu]
void rcCalcBounds(const float* verts, int nv, float* bmin, float* bmax);

// Calculates the grid size based on the bounding box and grid cell size.
// ���E�{�b�N�X�ƃO���b�h�Z���T�C�Y�Ɋ�Â��ăO���b�h�T�C�Y���v�Z���܂��B
//  @ingroup recast
//  @param[in] bmin : The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABB�̍ŏ����E�B [�ix�Ay�Az�j] [�P�ʁFwu]
//  @param[in] bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABB�̍ő勫�E�B [�ix�Ay�Az�j] [�P�ʁFwu]
//  @param[in] cs : The xz-plane cell size. [Limit: > 0] [Units: wu]
//  xz���ʂ̃Z���T�C�Y�B [�����F> 0] [�P�ʁFwu]
//  @param[out] w : The width along the x-axis. [Limit: >= 0] [Units: vx]
//  x���ɉ��������B [�����F> = 0] [�P�ʁFvx]
//  @param[out] h : The height along the z-axis. [Limit: >= 0] [Units: vx]
//  z���ɉ����������B [�����F> = 0] [�P�ʁFvx]
void rcCalcGridSize(const std::array<float, 3>& bmin, const std::array<float, 3>& bmax, float cs, int* w, int* h);

// Calculates the grid size based on the bounding box and grid cell size.
// ���E�{�b�N�X�ƃO���b�h�Z���T�C�Y�Ɋ�Â��ăO���b�h�T�C�Y���v�Z���܂��B
//  @ingroup recast
//  @param[in] bmin : The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABB�̍ŏ����E�B [�ix�Ay�Az�j] [�P�ʁFwu]
//  @param[in] bmax	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
//  AABB�̍ő勫�E�B [�ix�Ay�Az�j] [�P�ʁFwu]
//  @param[in] cs : The xz-plane cell size. [Limit: > 0] [Units: wu]
//  xz���ʂ̃Z���T�C�Y�B [�����F> 0] [�P�ʁFwu]
//  @param[out] w : The width along the x-axis. [Limit: >= 0] [Units: vx]
//  x���ɉ��������B [�����F> = 0] [�P�ʁFvx]
//  @param[out] h : The height along the z-axis. [Limit: >= 0] [Units: vx]
//  z���ɉ����������B [�����F> = 0] [�P�ʁFvx]
void rcCalcGridSize(const float* bmin, const float* bmax, float cs, int* w, int* h);

// Initializes a new heightfield. // �V�����n�`�����������܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in,out] hf : The allocated heightfield to initialize.
//	���������邽�߂Ɋ��蓖�Ă�ꂽ�����t�B�[���h�B
//  @param[in] width : The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
//	x���ɉ������t�B�[���h�̕��B [�����F> = 0] [�P�ʁFvx]
//  @param[in] height : The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
//	z���ɉ������t�B�[���h�̍����B [�����F> = 0] [�P�ʁFvx]
//  @param[in] bmin : The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
//	�t�B�[���h��AABB�̍ŏ����E�B [�ix�Ay�Az�j] [�P�ʁFwu]
//  @param[in] bmax : The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
//	�t�B�[���h��AABB�̍ő勫�E�B [�ix�Ay�Az�j] [�P�ʁFwu]
//  @param[in] cs : The xz-plane cell size to use for the field. [Limit: > 0] [Units: wu]
//	�t�B�[���h�Ɏg�p����xz���ʂ̃Z���T�C�Y�B [�����F> 0] [�P�ʁFwu]
//  @param[in] ch : The y-axis cell size to use for field. [Limit: > 0] [Units: wu]
//	�t�B�[���h�Ɏg�p����y���̃Z���T�C�Y�B [�����F> 0] [�P�ʁFwu]
//  @returns True if the operation completed successfully.
//	���삪����Ɋ��������ꍇ��true�B
bool rcCreateHeightfield(rcContext* ctx, rcHeightfield& hf, int width, int height,
	const float* bmin, const float* bmax,
	float cs, float ch);

// Modifies the area id of all triangles with a slope below the specified value.
//	�w�肳�ꂽ�l���Ⴂ���z�ł��ׂĂ̎O�p�`�̃G���AID��ύX���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in walkableSlopeAngle : The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees]
//	���s�\�ƌ��Ȃ����ő���z�B [�����F0 <=�l<90] [�P�ʁF�x]
//  @param[in] verts	 : The vertices. [(x, y, z) * @p nv]
//	���_�B [�ix�Ay�Az�j* @p nv]
//  @param[in] nv : The number of vertices.
//	���_�̐��B
//  @param[in] tris : The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
//	�O�p�`�̒��_�̃C���f�b�N�X�B [�ivertA�AvertB�AvertC�j* @p nt]
//  @param[in] nt : The number of triangles.
//	�O�p�`�̐��B
//  @param[out] areas : The triangle area ids. [Length: >= @p nt]
//	�O�p�`�̃G���AID�B [�����F> = @p nt]
//  @param[in] areaMod : The area modification to apply.
//	�K�p����G���A�̕ύX�B
// ���s�\�ȎO�p�`���}�[�N
void rcMarkWalkableTriangles(rcContext* ctx, const float walkableSlopeAngle, const float* verts, int nv,
	const int* tris, int nt, uint8_t* areas, rcAreaModification areaMod);

// Modifies the area id of all triangles with a slope greater than or equal to the specified value.
//  @ingroup recast
//  @param[in,out]	ctx					The build context to use during the operation.
//  @param[in]		walkableSlopeAngle	The maximum slope that is considered walkable.
//  									[Limits: 0 <= value < 90] [Units: Degrees]
//  @param[in]		verts				The vertices. [(x, y, z) * @p nv]
//  @param[in]		nv					The number of vertices.
//  @param[in]		tris				The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
//  @param[in]		nt					The number of triangles.
//  @param[out]	areas				The triangle area ids. [Length: >= @p nt]
void rcClearUnwalkableTriangles(rcContext* ctx, const float walkableSlopeAngle, const float* verts, int nv,
	const int* tris, int nt, uint8_t* areas);

// Adds a span to the specified heightfield.
//	�w�肳�ꂽ�����t�B�[���h�ɃX�p����ǉ����܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in,out] hf : An initialized heightfield.
//	���������ꂽ�����t�B�[���h�B
//  @param[in] x	 : The width index where the span is to be added. [Limits: 0 <= value < rcHeightfield::width]
//	�X�p�����ǉ�����镝�C���f�b�N�X�B [�����F0 <=�l<rcHeightfield :: width]
//  @param[in] y	 : The height index where the span is to be added. [Limits: 0 <= value < rcHeightfield::height]
//	�X�p�����ǉ�����鍂���C���f�b�N�X�B [�����F0 <=�l<rcHeightfield :: height]
//  @param[in] smin : The minimum height of the span. [Limit: < @p smax] [Units: vx]
//	�X�p���̍ŏ��̍����B [�����F<@p smax] [�P�ʁFvx]
//  @param[in] smax : The maximum height of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT] [Units: vx]
//	�X�p���̍ő�̍����B [�����F<= #RC_SPAN_MAX_HEIGHT] [�P�ʁFvx]
//  @param[in] area : The area id of the span. [Limit: <= #RC_WALKABLE_AREA)
//	�X�p���̃G���AID�B [�����F<= #RC_WALKABLE_AREA�j
//  @param[in] flagMergeThr : The merge theshold. [Limit: >= 0] [Units: vx]
//	�}�[�W�̂������l�B [�����F> = 0] [�P�ʁFvx]
//  @returns True if the operation completed successfully.
//	���삪����Ɋ��������ꍇ��true�B
bool rcAddSpan(rcContext* ctx, rcHeightfield& hf, const int x, const int y,
	const uint16_t smin, const uint16_t smax,
	const uint8_t area, const int flagMergeThr);

// Rasterizes a triangle into the specified heightfield.
//  @ingroup recast
//  @param[in,out]	ctx				The build context to use during the operation.
//  @param[in]		v0				Triangle vertex 0 [(x, y, z)]
//  @param[in]		v1				Triangle vertex 1 [(x, y, z)]
//  @param[in]		v2				Triangle vertex 2 [(x, y, z)]
//  @param[in]		area			The area id of the triangle. [Limit: <= #RC_WALKABLE_AREA]
//  @param[in,out]	solid			An initialized heightfield.
//  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag.
//  								[Limit: >= 0] [Units: vx]
//  @returns True if the operation completed successfully.
bool rcRasterizeTriangle(rcContext* ctx, const float* v0, const float* v1, const float* v2,
	const uint8_t area, rcHeightfield& solid,
	const int flagMergeThr = 1);

// Rasterizes an indexed triangle mesh into the specified heightfield.
//	�C���f�b�N�X�t���O�p�`���b�V�����w�肳�ꂽ�n�`�Ƀ��X�^���C�Y���܂��B
//  @ingroup recast
//  @param[in,out] ctx	 : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in] verts	 : The vertices. [(x, y, z) * @p nv]
//	���_�B [�ix�Ay�Az�j* @p nv]
//  @param[in] nv	 : The number of vertices.
//	���_�̐��B
//  @param[in] tris	 : The triangle indices. [(vertA, vertB, vertC) * @p nt]
//	�O�p�`�̃C���f�b�N�X�B [�ivertA�AvertB�AvertC�j* @p nt]
//  @param[in] areas	 : The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
//	�O�p�`�̃G���AID�B [�����F<= #RC_WALKABLE_AREA] [�T�C�Y�F@p nt]
//  @param[in] nt	 : The number of triangles.
//	�O�p�`�̐��B
//  @param[in,out] solid	 : An initialized heightfield.
//	���������ꂽ�n�`�B
//  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag. [Limit: >= 0] [Units: vx]
//	���s�s�\�t���O�������s�\�t���O���D�悳��鋗���B [�����F> = 0] [�P�ʁFvx]
//  @returns True if the operation completed successfully.
//	���삪����Ɋ��������ꍇ��true�B
// �O�p�`�̃��X�^���C�Y
bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const int nv,
	const int* tris, const uint8_t* areas, const int nt,
	rcHeightfield& solid, const int flagMergeThr = 1);

// Rasterizes an indexed triangle mesh into the specified heightfield.
//  @ingroup recast
//  @param[in,out]	ctx			The build context to use during the operation.
//  @param[in]		verts		The vertices. [(x, y, z) * @p nv]
//  @param[in]		nv			The number of vertices.
//  @param[in]		tris		The triangle indices. [(vertA, vertB, vertC) * @p nt]
//  @param[in]		areas		The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
//  @param[in]		nt			The number of triangles.
//  @param[in,out]	solid		An initialized heightfield.
//  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag.
//  							[Limit: >= 0] [Units: vx]
//  @returns True if the operation completed successfully.
bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const int nv,
	const uint16_t* tris, const uint8_t* areas, const int nt,
	rcHeightfield& solid, const int flagMergeThr = 1);

// Rasterizes triangles into the specified heightfield.
//  @ingroup recast
//  @param[in,out]	ctx				The build context to use during the operation.
//  @param[in]		verts			The triangle vertices. [(ax, ay, az, bx, by, bz, cx, by, cx) * @p nt]
//  @param[in]		areas			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
//  @param[in]		nt				The number of triangles.
//  @param[in,out]	solid			An initialized heightfield.
//  @param[in]		flagMergeThr	The distance where the walkable flag is favored over the non-walkable flag.
//  								[Limit: >= 0] [Units: vx]
//  @returns True if the operation completed successfully.
bool rcRasterizeTriangles(rcContext* ctx, const float* verts, const uint8_t* areas, const int nt,
	rcHeightfield& solid, const int flagMergeThr = 1);

// Marks non-walkable spans as walkable if their maximum is within @p walkableClimp of a walkable neighbor.
// �ő�l�����s�\�ȗאڂ̕��s�\��Climp���ɂ���ꍇ�A���s�s�\�Ƃ��ăX�p�����}�[�N���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in] walkableClimb : Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx]
//	�܂��ʉ߉\�ł���ƌ��Ȃ����ő�̏o����̍����B [�����F> = 0] [�P�ʁFvx]
//  @param[in,out] solid : A fully built heightfield.  (All spans have been added.)
//	���S�ɍ\�z���ꂽ�n�`�B�i���ׂẴX�p�����ǉ����ꂽ�j
void rcFilterLowHangingWalkableObstacles(rcContext* ctx, const int walkableClimb, rcHeightfield& solid);

// Marks spans that are ledges as not-walkable.
// �o����ł���X�p������s�s�\�Ƃ��ă}�[�N���܂��B
//
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in] walkableHeight : Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3] [Units: vx]
//	���ʐς����s�\�ƌ��Ȃ����悤�ɂ���ŏ�������u�V��v�܂ł̍����B[�����F> = 3] [�P�ʁFvx]
//  @param[in] walkableClimb : Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx]
//	�܂��ʉ߉\�ł���ƌ��Ȃ����ő�̏o����̍����B[�����F> = 0] [�P�ʁFvx]
//  @param[in,out] solid : A fully built heightfield. (All spans have been added.)
//	���S�ɍ\�z���ꂽ�n�`�B�i���ׂẴX�p�����ǉ�����܂����B�j
void rcFilterLedgeSpans(rcContext* ctx, const int walkableHeight,
	const int walkableClimb, rcHeightfield& solid);

// Marks walkable spans as not walkable if the clearence above the span is less than the specified height.
// �X�p���̏�̃N���A�����X���w�肳�ꂽ���������������ꍇ�A���s�\�X�p������s�s�Ƃ��ă}�[�N���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation. // ���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in] walkableHeight : Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3] [Units: vx]
// ���ʐς����s�\�ƌ��Ȃ����悤�ɂ���ŏ�������u�V��v�܂ł̍����B [�����F> = 3] [�P�ʁFvx]
//  @param[in,out] solid : A fully built heightfield.(All spans have been added.) // ���S�ɍ\�z���ꂽ�n�`�B(���ׂẴX�p�����ǉ�����܂����B�j
void rcFilterWalkableLowHeightSpans(rcContext* ctx, int walkableHeight, rcHeightfield& solid);

// Returns the number of spans contained in the specified heightfield.
//�@�w�肳�ꂽ�����t�B�[���h�Ɋ܂܂��X�p���̐���Ԃ��܂��B
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//�@���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in]		hf		An initialized heightfield.
//�@���������ꂽ�����t�B�[���h�B
//  @returns The number of spans in the heightfield.
//�@�����t�B�[���h�̃X�p���̐��B
int rcGetHeightFieldSpanCount(rcContext* ctx, rcHeightfield& hf);

// @}
// @name Compact Heightfield Functions
// @see rcCompactHeightfield
// @{
// Builds a compact heightfield representing open space, from a heightfield representing solid space.
// �\���b�h�X�y�[�X��\���n�`����A�I�[�v���X�y�[�X��\���R���p�N�g�Ȓn�`���\�z���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in] walkableHeight : Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3] [Units: vx]
//	���ʐς����s�\�ƌ��Ȃ����悤�ɂ���ŏ�������u�V��v�܂ł̍����B [�����F> = 3] [�P�ʁFvx]
//  @param[in] walkableClimb : Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx]
//	�܂��ʉ߉\�ł���ƌ��Ȃ����ő�̏o����̍����B �����F> = 0] [�P�ʁFvx]
//  @param[in] hf : The heightfield to be compacted. // ���k����鍂���t�B�[���h�B
//  @param[out] chf : The resulting compact heightfield. (Must be pre-allocated.)
//	���ʂ̃R���p�N�g�ȍ����t�B�[���h�B(���O�Ɋ��蓖�Ă�K�v������܂��B�j
//  @returns True if the operation completed successfully. // ���삪����Ɋ��������ꍇ��true�B
//  �R���p�N�g�Ȓn�`�̍\�z
bool rcBuildCompactHeightfield(rcContext* ctx, const int walkableHeight, const int walkableClimb,
	rcHeightfield& hf, rcCompactHeightfield& chf);

// Erodes the walkable area within the heightfield by the specified radius.
// �w�肳�ꂽ���a�����n�`���̕��s�\�̈��N�H���܂��B
// @ingroup recast
// @param[in,out] ctx : The build context to use during the operation.
// ���쒆�Ɏg�p����r���h�R���e�L�X�g�B
// @param[in] radius : The radius of erosion. [Limits: 0 < value < 255] [Units: vx]
// �N�H�̔��a�B [�����F0 <�l<255] [�P�ʁFvx]
// @param[in,out] chf : The populated compact heightfield to erode.
// �N�H���邽�߂ɓǂݍ��܂ꂽ�R���p�N�g�ȍ����t�B�[���h�B
// @returns True if the operation completed successfully. // ���삪����Ɋ��������ꍇ��true�B
bool rcErodeWalkableArea(rcContext* ctx, int radius, rcCompactHeightfield& chf);

// Applies a median filter to walkable area types (based on area id), removing noise.
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//  @param[in,out]	chf		A populated compact heightfield.
//  @returns True if the operation completed successfully.
bool rcMedianFilterWalkableArea(rcContext* ctx, rcCompactHeightfield& chf);

// Applies an area id to all spans within the specified bounding box. (AABB)
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//  @param[in]		bmin	The minimum of the bounding box. [(x, y, z)]
//  @param[in]		bmax	The maximum of the bounding box. [(x, y, z)]
//  @param[in]		areaMod	The area modification to apply.
//  @param[in,out]	chf		A populated compact heightfield.
void rcMarkBoxArea(rcContext* ctx, const float* bmin, const float* bmax, rcAreaModification areaMod,
	rcCompactHeightfield& chf);

// Applies the area id to the all spans within the specified convex polygon.
// �w�肳�ꂽ�ʑ��p�`���̂��ׂẴX�p���ɃG���AID��K�p���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in] verts : The vertices of the polygon [Fomr: (x, y, z) * @p nverts]
//	�|���S���̒��_[Fomr�F�ix�Ay�Az�j* @p nverts]
//  @param[in] nverts : The number of vertices in the polygon.
//	�|���S�����̒��_�̐��B
//  @param[in] hmin : The height of the base of the polygon.
//	�|���S���̃x�[�X�̍����B
//  @param[in] hmax : The height of the top of the polygon.
//	���p�`�̏㕔�̍����B
//  @param[in] areaMod : The area modification to apply.
//	�K�p����G���A�̕ύX�B
//  @param[in,out] chf : A populated compact heightfield.
//	�ǂݍ��܂ꂽ�R���p�N�g�Ȓn�`�B
void rcMarkConvexPolyArea(rcContext* ctx, const float* verts, const int nverts,
	const float hmin, const float hmax, rcAreaModification areaMod,
	rcCompactHeightfield& chf);

// Helper function to offset voncex polygons for rcMarkConvexPolyArea.
//  @ingroup recast
//  @param[in]		verts		The vertices of the polygon [Form: (x, y, z) * @p nverts]
//  @param[in]		nverts		The number of vertices in the polygon.
//  @param[out]	outVerts	The offset vertices (should hold up to 2 * @p nverts) [Form: (x, y, z) * return value]
//  @param[in]		maxOutVerts	The max number of vertices that can be stored to @p outVerts.
//  @returns Number of vertices in the offset polygon or 0 if too few vertices in @p outVerts.
int rcOffsetPoly(const float* verts, const int nverts, const float offset,
	float* outVerts, const int maxOutVerts);

// Applies the area id to all spans within the specified cylinder.
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//  @param[in]		pos		The center of the base of the cylinder. [Form: (x, y, z)]
//  @param[in]		r		The radius of the cylinder.
//  @param[in]		h		The height of the cylinder.
//  @param[in]		areaMod	The area modification to apply.
//  @param[in,out]	chf	A populated compact heightfield.
void rcMarkCylinderArea(rcContext* ctx, const float* pos,
	const float r, const float h, rcAreaModification areaMod,
	rcCompactHeightfield& chf);

// Builds the distance field for the specified compact heightfield.
// �w�肵���R���p�N�g�Ȓn�`�̋����t�B�[���h���\�z���܂��B
// @ingroup recast
// @param[in,out] ctx : The build context to use during the operation. ���쒆�Ɏg�p����r���h�R���e�L�X�g�B
// @param[in,out] chf : A populated compact heightfield. �ǂݍ��܂ꂽ�R���p�N�g�Ȓn�`�B
// @returns True if the operation completed successfully. ���삪����Ɋ��������ꍇ��True�B
bool rcBuildDistanceField(rcContext* ctx, rcCompactHeightfield& chf);

// ���敪�����g�p���āA�n�`�̗̈�f�[�^���\�z���܂��B
//  @ingroup recast
//  @param[in,out]	ctx				���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in,out]	chf				�ǂݍ��܂ꂽ�R���p�N�g�Ȓn�`�B
//  @param[in]		borderSize		�n�`�̎���̃i�r�Q�[�V�����s�̋��E���̃T�C�Y�B[Limit: >=0] [Units: vx]
//  @param[in]		minRegionArea	�Ǘ��������̗̈���`���ł���Z���̍ŏ����B[Limit: >=0] [Units: vx].
//  @param[in]		mergeRegionArea		�X�p���J�E���g�����̒l�����������̈�́A�\�ł���΂��傫�ȗ̈�ƃ}�[�W����܂��B [Limit: >=0] [Units: vx]
//  @returns ���삪����Ɋ��������ꍇ��True�B
// Builds region data for the heightfield using watershed partitioning.
//  @ingroup recast
//  @param[in,out]	ctx				The build context to use during the operation.
//  @param[in,out]	chf				A populated compact heightfield.
//  @param[in]		borderSize		The size of the non-navigable border around the heightfield.
//  								[Limit: >=0] [Units: vx]
//  @param[in]		minRegionArea	The minimum number of cells allowed to form isolated island areas.
//  								[Limit: >=0] [Units: vx].
//  @param[in]		mergeRegionArea		Any regions with a span count smaller than this value will, if possible,
//  								be merged with larger regions. [Limit: >=0] [Units: vx]
//  @returns True if the operation completed successfully.
bool rcBuildRegions(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea, const int mergeRegionArea);

// Builds region data for the heightfield by partitioning the heightfield in non-overlapping layers.
// �d�Ȃ荇��Ȃ����C���[�Œn�`�𕪊����邱�Ƃɂ��A�n�`�̗̈�f�[�^���\�z���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in,out] chf : A populated compact heightfield.
//	�ǂݍ��܂ꂽ�R���p�N�g�Ȓn�`�B
//  @param[in] borderSize : The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
//	�n�`�̎���̃i�r�Q�[�V�����s�\�ȋ��E���̃T�C�Y�B [�����F> = 0] [�P�ʁFvx]
//  @param[in] minRegionArea : The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx].
//	�Ǘ��������̗̈���`���ł���Z���̍ŏ����B [�����F> = 0] [�P�ʁFvx]�B
//  @returns True if the operation completed successfully.
//	���삪����Ɋ��������ꍇ��true�B
bool rcBuildLayerRegions(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea);

// Builds region data for the heightfield using simple monotone partitioning.
// �P���ȃ��m�g�[���������g�p���āA�n�`�̗̈�f�[�^���\�z���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in,out] chf : A populated compact heightfield.
//	�ǂݍ��܂ꂽ�R���p�N�g�ȍ����t�B�[���h�B
//  @param[in] borderSize : The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
//	heightfield�̎���̃i�r�Q�[�V�����s�\�ȋ��E���̃T�C�Y�B [�����F> = 0] [�P�ʁFvx]
//  @param[in] minRegionArea : The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx].
//	�Ǘ��������̗̈���`���ł���Z���̍ŏ����B [�����F> = 0] [�P�ʁFvx]�B
//  @param[in] mergeRegionArea : Any regions with a span count smaller than this value will, if possible, be merged with larger regions. [Limit: >=0] [Units: vx]
//	�\�ȏꍇ�A�X�p���J�E���g�����̒l�����������̈�́A���傫�ȗ̈�ƃ}�[�W����܂��B [�����F> = 0] [�P�ʁFvx]
//  @returns True if the operation completed successfully.
//	���삪����Ɋ��������ꍇ��true�B
bool rcBuildRegionsMonotone(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int minRegionArea, const int mergeRegionArea);

// Sets the neighbor connection data for the specified direction.
//  @param[in]		s		The span to update.
//  @param[in]		dir		The direction to set. [Limits: 0 <= value < 4]
//  @param[in]		i		The index of the neighbor span.
inline void rcSetCon(rcCompactSpan& s, int dir, int i)
{
	const uint32_t shift = (uint32_t)dir * 6;
	uint32_t con = s.con;
	s.con = (con & ~(0x3f << shift)) | (((uint32_t)i & 0x3f) << shift);
}

// Gets neighbor connection data for the specified direction.
// �w�肳�ꂽ�����̋ߗאڑ��f�[�^���擾���܂��B
//  @param[in]		s		The span to check. �`�F�b�N����X�p��
//  @param[in]		dir		The direction to check. �`�F�b�N������� [Limits: 0 <= value < 4]
//  @return The neighbor connection data for the specified direction,
//  	or #RC_NOT_CONNECTED if there is no connection.
// �w�肳�ꂽ�����̗אڐڑ��f�[�^�A�ڑ����Ȃ��ꍇ��#RC_NOT_CONNECTED�B
inline int rcGetCon(const rcCompactSpan& s, int dir)
{
	const uint32_t shift = (uint32_t)dir * 6;
	return (s.con >> shift) & 0x3f;
}

// Gets the standard width (x-axis) offset for the specified direction.
// �w�肳�ꂽ�����̕W�����ix���j�I�t�Z�b�g���擾���܂��B
//  @param[in]		dir		The direction. [Limits: 0 <= value < 4]
//  @return The width offset to apply to the current cell position to move
//  	in the direction.
//�����Ɉړ����邽�߂Ɍ��݂̃Z���ʒu�ɓK�p���镝�I�t�Z�b�g�B
inline int rcGetDirOffsetX(int dir)
{
	constexpr int offset[4] = { -1, 0, 1, 0, };
	return offset[dir & 0x03];
}

// Gets the standard height (z-axis) offset for the specified direction.
// �w�肳�ꂽ�����̕W���̍����iz���j�I�t�Z�b�g���擾���܂��B
//  @param[in]		dir		The direction. [Limits: 0 <= value < 4]
//  @return The height offset to apply to the current cell position to move
//  	in the direction.
// �����Ɉړ����邽�߂Ɍ��݂̃Z���ʒu�ɓK�p���鍂���I�t�Z�b�g�B
inline int rcGetDirOffsetY(int dir)
{
	constexpr int offset[4] = { 0, 1, 0, -1 };
	return offset[dir & 0x03];
}

// Gets the direction for the specified offset. One of x and y should be 0.
//  @param[in]		x		The x offset. [Limits: -1 <= value <= 1]
//  @param[in]		y		The y offset. [Limits: -1 <= value <= 1]
//  @return The direction that represents the offset.
inline int rcGetDirForOffset(int x, int y)
{
	constexpr int dirs[5] = { 3, 0, -1, 2, 1 };
	return dirs[((y + 1) << 1) + x];
}

// @}
// @name Layer, Contour, Polymesh, and Detail Mesh Functions
// @see rcHeightfieldLayer, rcContourSet, rcPolyMesh, rcPolyMeshDetail
// @{
// Builds a layer set from the specified compact heightfield.
// �w�肳�ꂽ�R���p�N�g�Ȓn�`���烌�C���[�Z�b�g���\�z���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in] chf : A fully built compact heightfield.
//	���S�ɍ\�z���ꂽ�R���p�N�g�Ȓn�`�B
//  @param[in] borderSize : The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
//  heightfield�̎���̃i�r�Q�[�V�����s�\�ȋ��E���̃T�C�Y�B [�����F> = 0] [�P�ʁFvx]
//  @param[in] walkableHeight : Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable. [Limit: >= 3] [Units: vx]
// ���ʐς����s�\�ƌ��Ȃ����悤�ɂ���ŏ�������u�V��v�܂ł̍����B [�����F> = 3] [�P�ʁFvx]
//  @param[out] lset : The resulting layer set. (Must be pre-allocated.)
// ���ʂ̃��C���[�Z�b�g�B �i���O�Ɋ��蓖�Ă�K�v������܂��B�j
//  @returns True if the operation completed successfully.
// ���삪����Ɋ��������ꍇ��true�B
bool rcBuildHeightfieldLayers(rcContext* ctx, rcCompactHeightfield& chf,
	const int borderSize, const int walkableHeight,
	rcHeightfieldLayerSet& lset);

// Builds a contour set from the region outlines in the provided compact heightfield.
//	�w�肳�ꂽ�R���p�N�g�ȍ����t�B�[���h�̗̈�A�E�g���C������֊s�Z�b�g���\�z���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in] chf : A fully built compact heightfield.
//	���S�ɍ\�z���ꂽ�R���p�N�g�ȃn�C�g�t�B�[���h�B
//  @param[in] maxError : The maximum distance a simplfied contour's border edges should deviate the original raw contour. [Limit: >=0] [Units: wu]
//	�P�������ꂽ�֊s�̋��E�G�b�W�����̐��̗֊s�����E����ő勗���B [�����F> = 0] [�P�ʁFwu]
//  @param[in] maxEdgeLen : The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx]
//	���b�V���̋��E�ɉ������֊s�G�b�W�̍ő勖�e���B [�����F> = 0] [�P�ʁFvx]
//  @param[out] cset : The resulting contour set. (Must be pre-allocated.)
//	���ʂ̗֊s�Z�b�g�B �i���O�Ɋ��蓖�Ă�K�v������܂��B�j
//  @param[in] buildFlags : The build flags. (See: #rcBuildContoursFlags)
//	�r���h�t���O�B �i#rcBuildContoursFlags���Q�Ɓj
//  @returns True if the operation completed successfully.
//	���삪����Ɋ��������ꍇ��true�B
bool rcBuildContours(rcContext* ctx, rcCompactHeightfield& chf,
	const float maxError, const int maxEdgeLen,
	rcContourSet& cset, const int buildFlags = RC_CONTOUR_TESS_WALL_EDGES);

// Builds a polygon mesh from the provided contours.
//	�w�肳�ꂽ�֊s����|���S�����b�V�����\�z���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in] cset : A fully built contour set.
//	���S�ɍ\�z���ꂽ�֊s�Z�b�g�B
//  @param[in] nvp : The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process. [Limit: >= 3]
//	�֊s����|���S���ւ̕ϊ��v���Z�X���ɐ��������|���S���ɋ�����钸�_�̍ő吔�B [�����F> = 3]
//  @param[out] mesh : The resulting polygon mesh. (Must be re-allocated.)
//	���ʂ̃|���S�����b�V���B �i�Ċ��蓖�Ă���K�v������܂��B�j
//  @returns True if the operation completed successfully.
//	���삪����Ɋ��������ꍇ��true�B
bool rcBuildPolyMesh(rcContext* ctx, rcContourSet& cset, const int nvp, rcPolyMesh& mesh);

// Merges multiple polygon meshes into a single mesh.
//	�����̃|���S�����b�V����P��̃��b�V���Ƀ}�[�W���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in] meshes : An array of polygon meshes to merge. [Size: @p nmeshes]
//	�}�[�W����|���S�����b�V���̔z��B [�T�C�Y�F@p nmeshes]
//  @param[in] nmeshes : The number of polygon meshes in the meshes array.
//	���b�V���z����̃|���S�����b�V���̐��B
//  @param[in] mesh : The resulting polygon mesh. (Must be pre-allocated.)
//	���ʂ̃|���S�����b�V���B �i���O�Ɋ��蓖�Ă�K�v������܂��B�j
//  @returns True if the operation completed successfully.
//	���삪����Ɋ��������ꍇ��true�B
bool rcMergePolyMeshes(rcContext* ctx, rcPolyMesh** meshes, const int nmeshes, rcPolyMesh& mesh);

// Builds a detail mesh from the provided polygon mesh.
//	�w�肳�ꂽ�|���S�����b�V������ڍ׃��b�V�����\�z���܂��B
//  @ingroup recast
//  @param[in,out] ctx : The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in] mesh : A fully built polygon mesh.
//	���S�ɍ\�z���ꂽ�|���S�����b�V���B
//  @param[in] chf : The compact heightfield used to build the polygon mesh.
//	�|���S�����b�V���̍\�z�Ɏg�p�����R���p�N�g�ȍ����t�B�[���h�B
//  @param[in] sampleDist : Sets the distance to use when samping the heightfield. [Limit: >=0] [Units: wu]
//	�n�`���T���v�����O����Ƃ��Ɏg�p���鋗����ݒ肵�܂��B [�����F> = 0] [�P�ʁFwu]
//  @param[in] sampleMaxError : The maximum distance the detail mesh surface should deviate from heightfield data. [Limit: >=0] [Units: wu]
//	�ڍ׃��b�V���\�ʂ��n�`�f�[�^�����E����ő勗���B [�����F> = 0] [�P�ʁFwu]
//  @param[out]	dmesh : The resulting detail mesh.  (Must be pre-allocated.)
//	���ʂ̏ڍ׃��b�V���B �i���O�Ɋ��蓖�Ă�K�v������܂��B�j
//  @returns True if the operation completed successfully.
//	���삪����Ɋ��������ꍇ��true�B
bool rcBuildPolyMeshDetail(rcContext* ctx, const rcPolyMesh& mesh, const rcCompactHeightfield& chf,
	const float sampleDist, const float sampleMaxError,
	rcPolyMeshDetail& dmesh);

// Copies the poly mesh data from src to dst.
//	�|�����b�V���f�[�^��src����dst�ɃR�s�[���܂��B
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in]		src		The source mesh to copy from.
//	�R�s�[���̃\�[�X���b�V���B
//  @param[out]	dst		The resulting detail mesh. (Must be pre-allocated, must be empty mesh.)
//	���ʂ̏ڍ׃��b�V���B �i���O�Ɋ��蓖�Ă��Ă���K�v������A��̃��b�V���ł���K�v������܂��B�j
//  @returns True if the operation completed successfully.
//	���삪����Ɋ��������ꍇ��true�B
bool rcCopyPolyMesh(rcContext* ctx, const rcPolyMesh& src, rcPolyMesh& dst);

// Merges multiple detail meshes into a single detail mesh.
//	�����̏ڍ׃��b�V����P��̏ڍ׃��b�V���Ƀ}�[�W���܂��B
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//	���쒆�Ɏg�p����r���h�R���e�L�X�g�B
//  @param[in]		meshes	An array of detail meshes to merge. [Size: @p nmeshes]
//	�}�[�W����ڍ׃��b�V���̔z��B [�T�C�Y�F@p nmeshes]
//  @param[in]		nmeshes	The number of detail meshes in the meshes array.
//	���b�V���z����̏ڍ׃��b�V���̐��B
//  @param[out]	mesh	The resulting detail mesh. (Must be pre-allocated.)
//	���ʂ̏ڍ׃��b�V���B �i���O�Ɋ��蓖�Ă�K�v������܂��B�j
//  @returns True if the operation completed successfully.
//	���삪����Ɋ��������ꍇ��true�B
bool rcMergePolyMeshDetails(rcContext* ctx, rcPolyMeshDetail** meshes, const int nmeshes, rcPolyMeshDetail& mesh);

// @}

#endif // RECAST_H

//////////////////////////////////////////////////

// Due to the large amount of detail documentation for this file,
// the content normally located at the end of the header file has been separated
// out to a file in /Docs/Extern.
//���̃t�@�C���̏ڍׂȃh�L�������g����ʂɂ��邽�߁A�ʏ�w�b�_�[�t�@�C���̍Ō�ɂ���R���e���c��/ Docs / Extern�̃t�@�C���ɕ�������Ă��܂��B
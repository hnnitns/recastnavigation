#ifndef BUILDCONTOURS_H
#define BUILDCONTOURS_H

bool buildContours(rcContext* ctx, rcCompactHeightfield& chf,
				   const float maxError,
				   rcContourSet& cset);

#endif // BUILDCONTOURS_H
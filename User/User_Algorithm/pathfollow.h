#ifndef _PATHFOLLOW_H
#define _PATHFOLLOW_H

#include "motion.h"
#include "chassis.h"

#define FALSE (0)
#define TRUE	(1)
//void BufferSizeInit();
//void ClearRingBuffer();
//void InputPoint2RingBuffer();
//void ClearPathLen();

void PathFollowing(float LookAheadDis, float Kp);

uint8_t FindLastPassedPoint(float PassedLen, float *TheoryLength, uint8_t PathNum);

//VirtualPos
void CalcProjectionPoint(Pose_t *Path, uint8_t PathNum, Pose_t CurrentPos, uint8_t projectionLineIndex);

//void CalcLookAheadPoint(PointU_t virtualPos, PointU_t virtualTarget);						//VirtualTarget
void CalcLookAheadPoint(Pose_t *Path, uint8_t PathNum, Pose_t projetionPos, uint8_t projectionIndex);

Pose_t CloestPointOnline(Pose_t startPos, Pose_t endPos, Pose_t curPos);

uint8_t JudgeLengthLessEqual(float lengthInput, float lengthCompared, float lengthLimit);

float Calc2PointsDistance(Pose_t *firstPos, Pose_t *secondPos);
#endif

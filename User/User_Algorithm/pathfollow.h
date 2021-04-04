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

void CalcProjectionPoint(Pose_t *Path, PointU_t virtualPos);	//VirtualPos
void CalcLookAheadPoint(PointU_t virtualPos, PointU_t virtualTarget);						//VirtualTarget

//Pose_t CloestPointOnline(Pose_t startPos, Pose_t endPos, Pose_t curPos);
Pose_t CloestPointOnline(Pose_t startPos, Pose_t endPos, float curX, float curY);

uint8_t JudgeLengthLessEqual(float lengthInput, float lengthCompared, float lengthLimit);


#endif

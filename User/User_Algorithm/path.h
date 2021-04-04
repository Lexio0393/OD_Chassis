#ifndef __PATH_H
#define __PATH_H

#include "motion.h"

//¹ì¼£³¤¶È
#define TEST_PATH_NUM (6)

extern Pose_t testPath[TEST_PATH_NUM];


void PathLineInterpolation(Pose_t *path , Pose_t startPos , Pose_t endPos ,\
						   float percent1 , float percent2 , float posture1 , float posture2);

//void PathInit(uint8_t courtId);
#endif

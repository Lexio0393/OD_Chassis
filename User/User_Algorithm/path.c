#include "path.h"
#include "chassis.h"

#define MAX_PLAN_VEL (8192)	//changed

void PathLineInterpolation(Pose_t *path , Pose_t startPos , Pose_t endPos ,\
						   float percent1 , float percent2 , float posture1 , float posture2)
{
	Pose_t inter1 , inter2 = {0};
	path[0] = startPos;
	path[3] = endPos;
	
	inter1.point.x = startPos.point.x + percent1 * (endPos.point.x - startPos.point.x);
	inter1.point.y = startPos.point.y + percent1 * (endPos.point.y - startPos.point.y);
	inter1.direction = posture1;
	inter1.vel = MAX_PLAN_VEL;
	
	inter2.point.x = startPos.point.x + percent2 * (endPos.point.x - startPos.point.x);
	inter2.point.y = startPos.point.y + percent2 * (endPos.point.y - startPos.point.y);
	inter2.direction = posture2;
	inter2.vel = MAX_PLAN_VEL;	
	
	path[1] = inter1;
	path[2] = inter2;

}

//void PathInit(uint8_t courtId)
//{
//}

Pose_t testPath[TEST_PATH_NUM]=
{
{	0.f	,	0.f	,	0.0f	,	0.0f	},
{	-500.0f	,	0.f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1000.0f	,	0.f	,	0.0f	,	MAX_PLAN_VEL	},
{	-1500.0f	,	0.f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2000.0f	,	0.f	,	0.0f	,	MAX_PLAN_VEL	},
{	-2500.0f	,	0.f	,	0.0f	,	0.0f	}
};

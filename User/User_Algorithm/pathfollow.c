#include "pathfollow.h"

#include "motion.h"
#include "path.h"

#include "chassis.h"

#include "pps.h"


void PathFollowing(float LookAheadDis, float Kp);



uint8_t FindLastPassedPoint(float PassedLen, float *TheoryLength, uint8_t PathNum)
{
	static uint8_t n = 0;
	uint8_t PointNum = 0;
	
	gChassis.PathInfo.totalLength = TheoryLength[PathNum - 1];
	
	if(JudgeLengthLessEqual(PassedLen, gChassis.PathInfo.totalLength, 500))
	{
		for(; n < PathNum; n++)
		{
			if(TheoryLength[n] > PassedLen)
			{
				PointNum = n;
				gChassis.PathInfo.pathLength = TheoryLength[n];
				gChassis.PathInfo.lengthIncrement = TheoryLength[n] - TheoryLength[n - 1];	
				break;
			}
		}
	}
	else
	{
		PointNum = PathNum;
		gChassis.PathInfo.pathLength = TheoryLength[PointNum];
		gChassis.PathInfo.lengthIncrement = TheoryLength[PointNum] - TheoryLength[PointNum - 1];
	}
	
	return PointNum;
}


void CalcProjectionPoint(Pose_t *Path, PointU_t virtualPos)
{
	static uint8_t searchFlag = FALSE;
	
	Pose_t ProjectionPoint;
	float disRealPos2VirPos;
	
	uint8_t i;
	
	if(gChassis.PathInfo.projectionIndex == 0)
	{
		searchFlag = TRUE;
		gChassis.PathInfo.projextionIndexNew = 0;
	}
	else
	{
		gChassis.PathInfo.projextionIndexNew = gChassis.PathInfo.projectionIndex;
	}
	
	i = gChassis.PathInfo.projextionIndexNew;
	
	ProjectionPoint = CloestPointOnline((Path + i), (Path + (i+1)), GetX(), GetY());
//	disRealPos2VirPos = sqrtf((ProjectionPoint.point.x - endPos.point.x) * (ProjectionPoint.point.x - endPos.point.x) +\
//															(ProjectionPoint.point.y - endPos.point.y) * (ProjectionPoint.point.y - endPos.point.y));
    
//    for i = ProjectionLineIndexNew+1:size(waypoints,1)-1

//        if ~searchFlag && dist > LookaheadDistance
//            break;
//        end
//        dist = dist + norm(waypoints(i,1:2) - waypoints(i+1,1:2));
//        % Check the remaining waypoints
//        [tempPoint, tempDistance] = ...
//            closestPointOnLine(waypoints(i,1:2), ...
//            waypoints(i+1,1:2), pose(1:2));

//        if tempDistance < minDistance
//            minDistance = tempDistance;
//            ProjectionPoint = tempPoint;
//            ProjectionLineIndexNew = cast(i, 'like', pose);
	
}

void CalcLookAheadPoint(PointU_t virtualPos, PointU_t virtualTarget);						//VirtualTarget


Pose_t CloestPointOnline(Pose_t startPos, Pose_t endPos, float curX, float curY)
{
	Pose_t cloestPoint;
	
//	vector_t v12;
//	vector_t vr2;
	
	Point_t v12;
	Point_t vr2;
	
	float alpha;
	
	if(startPos.point.x == endPos.point.x &&\
			startPos.point.y == endPos.point.y)
	{
		cloestPoint = startPos;
	}
	
//	v12.module = sqrt((startPos.point.x - endPos.point.x)*(startPos.point.x - endPos.point.x)+\
//											(startPos.point.y - endPos.point.y)*(startPos.point.y - endPos.point.y));
//	v12.direction = atan2f((startPos.point.y - endPos.point.y), (startPos.point.x - endPos.point.x))*CHANGE_TO_ANGLE;
//	
//	vr2.module = sqrt((curPos.point.x - endPos.point.x)*(curPos.point.x - endPos.point.x)+\
//											(curPos.point.y - endPos.point.y)*(curPos.point.y - endPos.point.y));
//	vr2.direction = atan2f((curPos.point.y - endPos.point.y), (curPos.point.x - endPos.point.x))*CHANGE_TO_ANGLE;
		
//	vr2.x = curPos.point.x - endPos.point.x;
//	vr2.y = curPos.point.y - endPos.point.y;
	
	v12.x = startPos.point.x - endPos.point.x;
	v12.y = startPos.point.y - endPos.point.y;
	vr2.x = curX - endPos.point.x;
	vr2.y = curY - endPos.point.y;
	
	alpha = (v12.x * vr2.x + v12.y * vr2.y) / sqrtf(v12.x * v12.x + v12.y * v12.y);
	
	if(alpha > 1)
	{
		cloestPoint = startPos;
	}
	else if(alpha < 0)
	{
		cloestPoint = endPos;
	}
	else
	{
		cloestPoint.point.x = alpha * startPos.point.x + (1 - alpha) * endPos.point.x;
		cloestPoint.point.y = alpha * startPos.point.y + (1 - alpha) * endPos.point.y;
		cloestPoint.direction = atan2f((startPos.point.y - endPos.point.y), (startPos.point.x - endPos.point.x)) * CHANGE_TO_ANGLE;
		cloestPoint.vel = endPos.vel;
	}
	
	return cloestPoint;
}

uint8_t JudgeLengthLessEqual(float lengthInput, float lengthCompared, float lengthLimit)
{
	float actLength = lengthInput;
	float targetLength = lengthCompared;
	
	if(actLength > targetLength - lengthLimit && \
			actLength < targetLength + lengthLimit)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

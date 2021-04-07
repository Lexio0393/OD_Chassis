#include "pathfollow.h"

#include "motion.h"
#include "path.h"

#include "chassis.h"

#include "pps.h"


void PathFollowing(float LookAheadDis, float Kp)
{
	/*
	 * float len = GetPassedLength();
	 * gChassis.PathInfo.projectionIndex = FindLastPassedPoint(float len, float *TheotyLength, uint8_t PathNum);
	 * CalcProjectionPoint(Pose_t *Path, PointU_t virtualPos, uint8_t projectionLineIndex)
	 * CalcLookAheadPoint
	 */
}



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

//CalcProjectionPoint(testPath, GetCurrentPoint, gChassis.PathInfo.projectionIndex)
void CalcProjectionPoint(Pose_t *Path, uint8_t PathNum, Pose_t CurrentPos, uint8_t projectionLineIndex)
{
	static uint8_t searchFlag = FALSE;
	
	Pose_t ProjectionPoint, tempPoint;
	Pose_t startPoint, endPoint;
	float disRealPos2VirPos, tempDis, dist;
	
	uint8_t index_num;
	uint8_t i = 0;
	
	if(projectionLineIndex == 0)
	{
		searchFlag = TRUE;
		gChassis.PathInfo.projextionIndexNew = 0;
	}
	else
	{
		gChassis.PathInfo.projextionIndexNew = projectionLineIndex;
	}
	
	index_num = gChassis.PathInfo.projextionIndexNew;
	
	startPoint = *(Path + index_num);
	endPoint = *(Path + (index_num+1));
	
	ProjectionPoint = CloestPointOnline(startPoint, endPoint, CurrentPos);
	
	disRealPos2VirPos = Calc2PointsDistance(&ProjectionPoint, &CurrentPos);
	
	dist = Calc2PointsDistance(&ProjectionPoint, &endPoint);

    
	for(i = index_num + 1;  i < PathNum; i++)
	{
		if(~searchFlag && disRealPos2VirPos > gChassis.PathInfo.lookaheadDis)
		{
			break;
		}
		
		dist = dist + Calc2PointsDistance(Path + i, (Path + (i+1)));
		
		tempPoint = CloestPointOnline(*(Path+i), *(Path + (i+1)), CurrentPos);
		tempDis = Calc2PointsDistance(&tempPoint, &CurrentPos);
		
		if(tempDis < disRealPos2VirPos)
		{
			disRealPos2VirPos = tempDis;
			ProjectionPoint = tempPoint;
			gChassis.PathInfo.projextionIndexNew = i;
		}
	}	
}

void CalcLookAheadPoint(Pose_t *Path, uint8_t PathNum, Pose_t projetionPos, uint8_t projectionIndex)				//VirtualTarget
{
	Pose_t LookaheadStartPoint, LookaheadEndPoint;
	Pose_t LookaheadPoint;
	
	uint8_t index_num = 0;
	uint8_t lookaheadIndex = projectionIndex;
	
	float dis, overshootDis;
	float alpha;
	
	index_num = projectionIndex + 1;
	dis = Calc2PointsDistance(&projetionPos, (Path+index_num));
	
	LookaheadStartPoint = projetionPos;
	LookaheadEndPoint = *(Path + index_num);
	
	overshootDis = dis - gChassis.PathInfo.lookaheadDis;
	
	while(overshootDis <0 && lookaheadIndex < PathNum)
	{
		lookaheadIndex++;
		
		LookaheadStartPoint = *(Path + lookaheadIndex);
		LookaheadEndPoint = *(Path +(lookaheadIndex + 1));
		dis = dis + Calc2PointsDistance(&LookaheadStartPoint, &LookaheadEndPoint);
		
		overshootDis = dis - gChassis.PathInfo.lookaheadDis;
	}
	
	
	alpha = overshootDis / Calc2PointsDistance(&LookaheadStartPoint, &LookaheadEndPoint);
	
	if(alpha > 0)
	{
		LookaheadPoint.point.x = alpha * LookaheadStartPoint.point.x + (1 - alpha) * LookaheadEndPoint.point.x;
		LookaheadPoint.point.y = alpha * LookaheadStartPoint.point.y + (1 - alpha) * LookaheadEndPoint.point.y;
	}
	else
	{
		LookaheadPoint = LookaheadEndPoint;
	}
}

Pose_t CloestPointOnline(Pose_t startPos, Pose_t endPos, Pose_t currentPos)
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
	vr2.x = currentPos.point.x - endPos.point.x;
	vr2.y = currentPos.point.y - endPos.point.y;
	
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

float Calc2PointsDistance(Pose_t *firstPoint, Pose_t *secondPoint)
{
	Pose_t firstPos = *firstPoint;
	Pose_t secondPos = *secondPoint;
	
	return sqrtf((firstPos.point.x - secondPos.point.x) * (firstPos.point.x - secondPos.point.x) +\
															(firstPos.point.y - secondPos.point.y) * (firstPos.point.y - secondPos.point.y));
}


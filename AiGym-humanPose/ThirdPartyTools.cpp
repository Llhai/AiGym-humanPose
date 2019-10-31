#include "ThirdPartyTools.h"

bool ThirdPartyTools::Check3PointCommonLine(Point2f p1, Point2f p2, Point2f p3, float thresholds)
{
	Point2f v1 = p2 - p1;
	Point2f v2 = p3 - p1;
	float angle = fabs(Get2VecAngle(v1.y, v1.x, v2.y, v2.x));
	//std::cout << "angle " << angle << std::endl;

	if (angle < thresholds)
	{
		return true;
	}
	return false;
}

float ThirdPartyTools::Get2VecAngle(float p1x, float p1y, float p2x, float p2y)
{
	float angle = atan2f(p2y, p2x)*180/M_PI - atan2f(p1y, p1x) * 180 / M_PI;
	return angle;
}

float ThirdPartyTools::GetDisBetween2Point(Point p1, Point p2)
{
	float dis = sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
	return dis;
 }
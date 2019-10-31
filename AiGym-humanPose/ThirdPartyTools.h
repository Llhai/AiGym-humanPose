#pragma once

#define M_PI 3.14159265358979323846

#include<opencv.hpp>
#include<cmath>
using namespace cv;

static class ThirdPartyTools
{

public:
	// points should be in order p1 p2 p3 
	//thresholds angle (0,180)
	static bool Check3PointCommonLine(Point2f p1, Point2f p2, Point2f p3, float thresholds);

	// return angle from v2 to v1 (-180, 180)
	static float Get2VecAngle(float p1x, float p1y, float p2x, float p2y);

	//return dis between p1 and p2
	static float GetDisBetween2Point(Point p1, Point p2);
};


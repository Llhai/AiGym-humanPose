
#include <opencv.hpp>
#include "human_pose_estimator.hpp"
#include "ThirdPartyTools.h"
#include "thresholdDefine.h"
using namespace std;

enum poseEnum { otherPose, pullStart, pullForward, pullBack, pullStraight };
bool CheckPoseSit(human_pose_estimation::HumanPose pose, cv::Mat frame)
{
	bool lumbarInRange = false;
	//check lumbar Ñü pos
	cv::Point leftUp = cv::Point(0, 0);
	cv::Point rightDown = cv::Point(1000, 1000);
	cv::Point lumar = cv::Point((pose.keypoints[8].x + pose.keypoints[11].x) / 2, (pose.keypoints[8].y + pose.keypoints[11].y) / 2);
	bool poseSit = false;
	if ((lumar.x<rightDown.x && lumar.x>leftUp.x) && (lumar.y<rightDown.y && lumar.y>leftUp.y))
	{
		lumbarInRange = true;
	}

	//check knee foot
	if (pose.keypoints[8].x< pose.keypoints[9].x || pose.keypoints[11].x>pose.keypoints[12].x)
	{
		poseSit = false;
	}
	if (pose.keypoints[8].x< pose.keypoints[10].x || pose.keypoints[11].x>pose.keypoints[13].x)
	{
		poseSit = false;
	}
	poseSit = true;
	std::string putString = poseSit == true ? "true" : "false";
	//cv::putText(frame, putString, cv::Point(100, 100), 2, 2.0f, cv::Scalar(255, 255, 0));

	return poseSit;
}

bool CheckPoseSymmetric(human_pose_estimation::HumanPose pose, cv::Mat frame)
{
	int score = 0;
	int pointPair[6][2] = { {2,5},{3,6},{4,7},{8,11},{9,12},{10,13} };
	for (int i = 0; i < 6; i++)
	{
		int disY = fabs(pose.keypoints[pointPair[i][0]].y - pose.keypoints[pointPair[i][1]].y);
		//cv::putText(frame, to_string(disY), pose.keypoints[pointPair[i][0]], 2, 1.0f, cv::Scalar(255, 255, 0));

		if (fabs(pose.keypoints[pointPair[i][0]].y - pose.keypoints[pointPair[i][1]].y) < PIXEL_DIFFER_THRESHOLD)
		{
			score++;
			//cv::line(frame, pose.keypoints[pointPair[i][0]], pose.keypoints[pointPair[i][1]], cv::Scalar(255, 255, 0));
		}
	}

	//string putString = symm == true ? "true" : "false";
	//cv::putText(frame, to_string(score), cv::Point(100, 100), 2, 2.0f, cv::Scalar(255, 255, 0));

	if (score >= SYMM_COUNT_THRESHOLD)
	{
		return true;
	}
	return false;

}

poseEnum getCurretPoseState(human_pose_estimation::HumanPose pose, cv::Mat frame)
{
	poseEnum poseenu = otherPose;

	bool leftArmCommonLine = ThirdPartyTools::Check3PointCommonLine(pose.keypoints[2], pose.keypoints[3], pose.keypoints[4], COMMON_LINE_COUNT);
	bool rightArmCommonLine = ThirdPartyTools::Check3PointCommonLine(pose.keypoints[5], pose.keypoints[6], pose.keypoints[7], COMMON_LINE_COUNT);
	string putText = "Not Common Line ";
	if (leftArmCommonLine&&rightArmCommonLine)
	{
		float leftBigArmLen =ThirdPartyTools::GetDisBetween2Point(pose.keypoints[2], pose.keypoints[3]) ;
		float leftSmallArmLen = ThirdPartyTools::GetDisBetween2Point(pose.keypoints[3], pose.keypoints[4]);

		float rightBigArmLen = ThirdPartyTools::GetDisBetween2Point(pose.keypoints[5], pose.keypoints[6]);
		float rightSmallArmLen = ThirdPartyTools::GetDisBetween2Point(pose.keypoints[6], pose.keypoints[7]);

		float ArmRate = (leftBigArmLen + rightBigArmLen) / (leftSmallArmLen + rightSmallArmLen);
		if (ArmRate <= PULL_STRAIGHT_RATE)
		{
			poseenu = pullStraight;
		}
		else if(ArmRate >= PULL_START_RATE)
		{
			poseenu = pullStart;
		}
		putText = to_string(ArmRate);
	}
	else
	{
		float leftAngle = ThirdPartyTools::Get2VecAngle();
		float rightAngle = ThirdPartyTools::Get2VecAngle();
	}
	/*else if (leftArmCommonLine)
	{
		putText = "Right Not Common Line ";
	}
	else if (rightArmCommonLine)
	{
		putText = "Left Not Common Line ";
	}
	else
	{
		putText = "Both Not Common Line ";

	}*/
	poseenu =  otherPose;
	cv::putText(frame, putText, cv::Point(100, 100), 2, 1.0f, cv::Scalar(0, 0, 0));
	return poseenu;
}

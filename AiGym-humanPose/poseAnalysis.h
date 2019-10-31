
#include <opencv.hpp>
#include "human_pose_estimator.hpp"
#include "ThirdPartyTools.h"
#include "thresholdDefine.h"
using namespace std;

enum poseEnum { otherPose, pullStart, pullForward, pullBack, pullStraight };
void countNum(int& num, cv::Mat frame);
string poseEnu2String(poseEnum pose);
bool CheckPoseSit(human_pose_estimation::HumanPose pose, cv::Mat frame);
bool CheckPoseSymmetric(human_pose_estimation::HumanPose pose, cv::Mat frame);
poseEnum getCurretPoseState(human_pose_estimation::HumanPose pose, cv::Mat frame);

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

	if (lumbarInRange)
	{
		//check knee foot
		if (pose.keypoints[8].x> pose.keypoints[9].x && pose.keypoints[11].x<pose.keypoints[12].x)
		{
			poseSit = true;
		}
	}
	
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
	string putText = "Message ";

	// height differ between shouler and elbow
	float leftHeightDiffer = fabs (pose.keypoints[2].y - pose.keypoints[3].y);
	float rightHeightDiffer = fabs(pose.keypoints[5].y - pose.keypoints[6].y);
	//putText = "left "+to_string(int (leftHeightDiffer))+" "+ "right " + to_string(int(rightHeightDiffer));
	if (leftHeightDiffer < PULL_START_HEIGHT_BETWEEN_SHOULDER_AND_ELBOW
		&& rightHeightDiffer < PULL_START_HEIGHT_BETWEEN_SHOULDER_AND_ELBOW)
	{
		poseenu = pullStart;
	}


	bool leftArmCommonLine = ThirdPartyTools::Check3PointCommonLine(pose.keypoints[2], pose.keypoints[3], pose.keypoints[4], COMMON_LINE_COUNT);
	bool rightArmCommonLine = ThirdPartyTools::Check3PointCommonLine(pose.keypoints[5], pose.keypoints[6], pose.keypoints[7], COMMON_LINE_COUNT);
	
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
	}
	else
	{
		//ÅÐ¶Ï ÊÖ±Û½Ç¶È 
		//2->3 -----> 3->4
		Point leftV1 = pose.keypoints[4] - pose.keypoints[3];
		Point leftV2 = pose.keypoints[3] - pose.keypoints[2];
		float leftAngle = ThirdPartyTools::Get2VecAngle(leftV1.x, leftV1.y, leftV2.x, leftV2.y);		
		//6->7 -----> 5->6
		Point rightV1 = pose.keypoints[6] - pose.keypoints[5];
		Point rightV2 = pose.keypoints[7] - pose.keypoints[6];
		float rightAngle = ThirdPartyTools::Get2VecAngle(rightV1.x,rightV1.y,rightV2.x,rightV2.y);
		if (leftAngle > PULL_START_ANGLE_MIN&& leftAngle<PULL_START_ANGLE_MAX 
			&& rightAngle>PULL_START_ANGLE_MIN&& rightAngle < PULL_START_ANGLE_MAX)
		{
			poseenu = pullStart;
		}
		//putText = "left "+to_string(int(leftAngle))+"   ""right " + to_string(int(rightAngle));
		//putText = to_string(rightAngle);
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
	putText = poseEnu2String(poseenu);
	//cv::putText(frame, putText, cv::Point(100, 40), 2, 1.0f, cv::Scalar(255, 0, 0));
	return poseenu;
}


void countNum(int& num,poseEnum& lastPose, poseEnum currentPose)
{
	//cout << poseEnu2String(lastPose) << endl;
	switch (lastPose)
	{
	case pullStart:
		if (currentPose == pullStraight)
		{
			lastPose = pullStraight;
			num++;
			cout << "+++++++++++" << endl;
		}
		break;
	case pullStraight:
		if (currentPose == pullStart)
		{
			lastPose = pullStart;
		}
	case otherPose:
		if (currentPose == pullStart)
		{
			lastPose = pullStart;
		}
	default :
		break;
	}
	return;
}

string poseEnu2String(poseEnum pose)
{
	string temp = " ";
	switch (pose)
	{
	case otherPose:
		temp = "otherPose";
		break;
	case pullStart:
		temp = "pull Start";
		break;
	case pullForward:
		temp = "pull Forward";
		break;
	case pullBack:
		temp = "pull Back";
		break;
	case pullStraight:
		temp = "pull Straight";
		break;
	}
	return temp;
}

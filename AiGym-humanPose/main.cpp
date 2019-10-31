#include <vector>
#include <inference_engine.hpp>
#include <samples/ocv_common.hpp>
#include "human_pose_estimator.hpp"
#include "render_human_pose.hpp"
#include  "opencv2/highgui.hpp"
#include  "opencv2/imgproc.hpp"
#include "poseAnalysis.h"
#include "ThirdPartyTools.h"

using namespace InferenceEngine;
using namespace human_pose_estimation;
using namespace std;



void openCamera(int cameraId);
void initMonocularestimator();


human_pose_estimation::HumanPoseEstimator* estimator;
cv::VideoCapture cap;
int cameraHeight;
int cameraWidth;



void openCamera(int cameraId) {
	cap.open("D:/Project/AiGym/videos/4.mp4");
	//cap.open(1);
}

void initMonocularestimator() {
	estimator = new human_pose_estimation::HumanPoseEstimator("human-pose-estimation.xml", "CPU", false);

}

cv::Mat imagePreProcess(cv::Mat mat)
{
	//resize
	cv::Mat frame;
	cv::resize(mat, frame, cv::Size(1440, 810));

	//cut
	int x = 250;
	int y = 0;
	int width = 700;
	int height = 810;
	cv::Rect roi = cv::Rect(x, y, width, height);
	if ( x + width > frame.cols || y + height > frame.rows )
	{
		cout << "imagePreProcess error" << endl;
	}
	frame = frame(roi);
	return frame;
	//cv::rectangle(frame, roi,cv::Scalar(255,0,0));
}


human_pose_estimation::HumanPose getHumanPose(cv::Mat frame)
{
	std::vector<human_pose_estimation::HumanPose> poses = estimator->estimate(frame);
	human_pose_estimation::renderHumanPose(poses, frame);
	human_pose_estimation::HumanPose pose;
	if (poses.size() > 1)
	{
		float maxScore = 0;
		for (auto human : poses)
		{
			if (human.score > maxScore)
			{
				pose = human;
			}
		}
	}
	else
	{
		pose = poses[0];
	}
	//bool poseSymm = CheckPoseSymmetric(pose,frame);
	
	return pose;
}


int main(int argc, char* argv[]) {

	
	initMonocularestimator();
	openCamera(0);
	double lastTime = 0;
	cv::Mat frame;
	
	int count = 0;
	poseEnum lastpose = otherPose;
	int sitFrameCount = 0;
	while (true) {
		cap >> frame;
		frame = imagePreProcess(frame);
		human_pose_estimation::HumanPose pose = getHumanPose(frame);
		bool poseSit = CheckPoseSit(pose, frame);
		if (poseSit)
		{
			sitFrameCount++;
		}
		else
		{
			sitFrameCount = 0;
		}
		if (sitFrameCount > SIT_FRAME_COUNT)
		{
			poseEnum currentPose = getCurretPoseState(pose, frame);
			countNum(count, lastpose, currentPose);
			cv::putText(frame, poseEnu2String(lastpose), cv::Point(100, 100), 2, 1.0f, cv::Scalar(0, 0, 255));
			cv::putText(frame, to_string(count), cv::Point(100, 60), 2, 2.0f, cv::Scalar(0, 0, 255));
		}
		else
		{
			cv::putText(frame, "not sit", cv::Point(100, 100), 2, 1.0f, cv::Scalar(0, 0, 255));
		}
		cv::imshow("frame", frame);
		cv::waitKey(5);
	}
	return 0;
}

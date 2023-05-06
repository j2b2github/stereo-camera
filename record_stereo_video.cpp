#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include "common.hpp"

int main(int argc, char **argv)
{
	Settings sets;
	if (!ReadCommandLine(argc, argv, sets))
	{
		return -1;
	}

	cv::VideoCapture capL(sets.CamL_id, cv::CAP_V4L2), capR(sets.CamR_id, cv::CAP_V4L2);

	if (!capL.isOpened() || !capR.isOpened())
	{
		std::cerr << "Camera open failed!" << std::endl;
		return -1;
	}

	SetCameraProperties(capL, sets);
	SetCameraProperties(capR, sets);

	int w = cvRound(capL.get(cv::CAP_PROP_FRAME_WIDTH));
	int h = cvRound(capL.get(cv::CAP_PROP_FRAME_HEIGHT));
	double fps = capL.get(cv::CAP_PROP_FPS);

	int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
	int delay = cvRound(1000 / fps);

	auto now = std::chrono::system_clock::now();
	std::time_t end_time = std::chrono::system_clock::to_time_t(now);
	std::cout << "Current Time and Date: " << std::ctime(&end_time) << std::endl;

	cv::VideoWriter outputVideoL(sets.strOutPath + "stereoL-" + std::ctime(&end_time) + ".mp4", fourcc, fps, cv::Size(w, h));
	cv::VideoWriter outputVideoR(sets.strOutPath + "stereoR-" + std::ctime(&end_time) + ".mp4", fourcc, fps, cv::Size(w, h));

	if (!outputVideoL.isOpened() || !outputVideoR.isOpened())
	{
		std::cout << "File open failed!" << std::endl;
		return -1;
	}

	cv::Mat frameL, frameR, tmp;
	while (true)
	{
		capL >> frameL;
		capR >> frameR;
		if (frameL.empty() || frameR.empty())
			break;

		outputVideoL << frameL;
		outputVideoR << frameR;

		cv::hconcat(frameL, frameR, tmp);
		imshow("frame", tmp);

		if (cv::waitKey(delay) == 27)
			break;
	}

	cv::destroyAllWindows();

	return 0;
}

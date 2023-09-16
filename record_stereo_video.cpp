#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include "common.hpp"

int main(int argc, char **argv)
{
	cv::CommandLineParser parser(argc, argv,
		"{ help || show help message }"
		"{ stereo_setting || .yml file for stereo camera setting }"
	);

	parser.about("Use this script to run calibrateZ");

    if (argc == 1 || parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }

    Settings sets;
	std::string strSettingPath(parser.get<std::string>("stereo_setting"));
    if (!ReadSettings(strSettingPath, sets))
	{
		std::cout << "read setting error." << std::endl;
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
	std::time_t timeCurrent = std::chrono::system_clock::to_time_t(now);
	std::string strCurTime = std::ctime(&timeCurrent);
	strCurTime = strCurTime.substr(0, strCurTime.length()-1);
	std::cout << "Current Time and Date: " << strCurTime << std::endl;

	std::string strFilenameL = "stereoL-" + strCurTime + ".mp4"; 
	std::string strFilenameR = "stereoR-" + strCurTime + ".mp4";
	cv::VideoWriter outputVideoL(sets.strOutPath + strFilenameL, fourcc, fps, cv::Size(w, h));
	cv::VideoWriter outputVideoR(sets.strOutPath + strFilenameR, fourcc, fps, cv::Size(w, h));

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

	sets.estimateZ_videoFileL = strFilenameL;
	sets.estimateZ_videoFileR = strFilenameR;
	if (!WriteSettings(strSettingPath, sets))
		return -1;

	return 0;
}

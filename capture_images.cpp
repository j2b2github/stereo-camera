#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include "common.hpp"

bool CheckLeftRight(int &CalL_id, int &CamR_id, bool &swapped);
bool Check_n_CreateFolder(std::string strPath);

int main(int argc, char **argv)
{
    Settings sets;

    if (argc != 2)
    {
        std::cout << "need setting file(.yml)" << std::endl;
        return -1;
    }

	std::string strSettingPath(argv[1]);
    if (!ReadSettings(strSettingPath, sets))
	{
		std::cout << "read setting error." << std::endl;
		return -1;
	}

	bool swapped(false);
	if (!CheckLeftRight(sets.CamL_id, sets.CamR_id, swapped))
	{
		return -1;
	}
	else
	{
		if (swapped)
		{
			if (!WriteSettings(strSettingPath, sets))
				return -1;
		}
	}

	if (!Check_n_CreateFolder(sets.strOutPath))
	{
		std::cout << "create folder error." << std::endl;
		return -1;
	}

	cv::Mat frameL, frameR, temp;
	cv::VideoCapture camL_valid(sets.CamL_id, cv::CAP_V4L2), camR_valid(sets.CamR_id, cv::CAP_V4L2);

	SetCameraProperties(camL_valid, sets);
	SetCameraProperties(camR_valid, sets);

	int64 start{cv::getTickCount()};
	float time{0};

	cv::Mat grayL, grayR;

	std::vector<cv::Point2f> cornersL, cornersR;
	cv::Size board = cv::Size(sets.board_w, sets.board_h);
	bool foundL, foundR;
	int count{0};

	while (true)
	{
		camL_valid >> frameL;
		camR_valid >> frameR;

		if (frameL.empty() || frameR.empty())
		{
			std::cout << "cam open failed: L:" << frameL.empty() << " R:" << frameR.empty() << std::endl;
			break;
		}

		time = (cv::getTickCount() - start) / cv::getTickFrequency();

		cv::hconcat(frameL, frameR, temp);
		cv::putText(temp, std::to_string(time), cv::Point(50, 100), 2, 1, cv::Scalar(0, 200, 200), 2);
		cv::imshow("left and right frames", temp);
		cv::waitKey(1);

		cv::cvtColor(frameL, grayL, cv::COLOR_BGR2GRAY);
		cv::cvtColor(frameR, grayR, cv::COLOR_BGR2GRAY);

		foundL = cv::findChessboardCorners(grayL, board, cornersL, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		foundR = cv::findChessboardCorners(grayR, board, cornersR, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

		std::cout << foundL << "," << foundR << std::endl;

		if ((foundL == true) && (foundR == true) && (time > 10))
		{
			count++;
			cv::imwrite(sets.strOutPath + "stereoL/img_" + cv::format("%03d", count) + ".jpg", frameL);
			cv::imwrite(sets.strOutPath + "stereoR/img_" + cv::format("%03d", count) + ".jpg", frameR);
			std::cout << sets.strOutPath + "stereoL/img_" + cv::format("%03d", count) + ".jpg" << std::endl;
		}

		if (time > 10)
			start = cv::getTickCount();

		if (cv::waitKey(1) == 27)
			break;
	}

	cv::destroyAllWindows();
	return 0;
}

bool CheckLeftRight(int &CamL_id, int &CamR_id, bool &swapped)
{
	// Check for left and right camera IDs
	cv::VideoCapture camL(CamL_id), camR(CamR_id);

	// Check if left camera is attched
	if (!camL.isOpened())
	{
		std::cout << "Could not open camera with index : " << CamL_id << std::endl;
		return false;
	}

	// Check if right camera is attached
	if (!camL.isOpened())
	{
		std::cout << "Could not open camera with index : " << CamL_id << std::endl;
		return false;
	}

	cv::Mat frameL, frameR, temp;
	cv::namedWindow("left and right frames", cv::WINDOW_NORMAL);
	cv::resizeWindow("left and right frames", 1200, 600);

	camL >> frameL;
	camR >> frameR;
	cv::putText(frameL, "L", cv::Point(frameL.cols / 2, frameL.rows / 2), cv::FONT_HERSHEY_SIMPLEX, 10.0, cv::Scalar(0, 255, 0), 10);
	cv::putText(frameR, "R", cv::Point(frameR.cols / 2, frameR.rows / 2), cv::FONT_HERSHEY_SIMPLEX, 10.0, cv::Scalar(0, 255, 0), 10);

	cv::hconcat(frameL, frameR, temp);
	cv::putText(temp, "Checking the right and left camera IDs:", cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
	cv::putText(temp, "Press (y) if IDs are correct and (n) to swap the IDs", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
	cv::imshow("left and right frames", temp);
	std::cout << "Checking the right and left camera IDs:" << std::endl;
	std::cout << "Press (y) if IDs are correct and (n) to swap the IDs" << std::endl;
	char c = (char)cv::waitKey(0);

	if (c == 'y')
	{
		std::cout << "camera IDs retained" << std::endl;
	}
	else if (c == 'n')
	{
		int temp;
		temp = CamL_id;
		CamL_id = CamR_id;
		CamR_id = temp;
		std::cout << "camera IDs swapped" << std::endl;
		swapped = true;
	}
	else
	{
		std::cout << "Wrong response!!!" << std::endl;
		return false;
	}

	return true;
}

bool Check_n_CreateFolder(std::string strPath)
{
	bool ret(true);

	if (std::filesystem::exists(strPath))
	{
		if (!std::filesystem::exists(strPath + "/stereoL/"))
			ret &= std::filesystem::create_directory(strPath + "/stereoL/");

		if (!std::filesystem::exists(strPath + "/stereoR/"))
			ret &= std::filesystem::create_directory(strPath + "/stereoR/");
	}
	else
	{
		ret &= std::filesystem::create_directory(strPath);
		ret &= std::filesystem::create_directory(strPath + "/stereoL/");
		ret &= std::filesystem::create_directory(strPath + "/stereoR/");
	}

	return ret;
}

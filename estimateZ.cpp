#include <opencv2/opencv.hpp>
#include <opencv2/cudastereo.hpp>

#include <stdio.h>
#include <iostream>
#include <filesystem>
#include <math.h>
#include <fstream>
#include <map>
#include <cmath>

#include "common.hpp"

double GetAvgZ_rect(const cv::Mat & matXYZ, cv::Point2d lt, cv::Point2d rb, const cv::Mat& mask = cv::Mat());
bool Check_n_CreateFolder_Z_calibration(std::string strPath);
bool GetRectifiedMap(const Settings &sets,
					 StereoCameraParameter &param,
					 cv::Mat &Left_Stereo_Map1, cv::Mat &Left_Stereo_Map2,
					 cv::Mat &Right_Stereo_Map1, cv::Mat &Right_Stereo_Map2,
					 cv::Rect &validRoiL, cv::Rect &validRoiR);
double GetAvgZ_circle(const cv::Mat &matXYZ, const cv::Vec3f &circle);
void houghtcircle(cv::Mat &img, const cv::Mat & matXYZ, std::map<int, double> table);

int main(int argc, char** argv)
{
    Settings sets;
	if (!ReadCommandLine(argc, argv, sets))
	{
		return -1;
	}

    cv::VideoCapture cam_0, cam_1;
	
	if (sets.estimateZ_mode == 0) {
		cam_0.open(sets.strOutPath + sets.estimateZ_videoPathL);
		cam_1.open(sets.strOutPath + sets.estimateZ_videoPathR);
	} else {
		cam_0.open(sets.CamL_id, cv::CAP_V4L2);
		cam_1.open(sets.CamR_id, cv::CAP_V4L2);

		SetCameraProperties(cam_0, sets);
		SetCameraProperties(cam_1, sets);

		int FPS_0 = cam_0.get(cv::CAP_PROP_FPS);
		int FPS_1 = cam_1.get(cv::CAP_PROP_FPS);
		std::cout << "cam_0 FPS: " << FPS_0 << std::endl;
		std::cout << "cam_1 FPS: " << FPS_1 << std::endl;
	}

    if (!cam_0.isOpened() || !cam_1.isOpened())
    {
        std::cout << "cam/video open fail"; 
        return -1;
    }

	// z 축 캘리브레이션
	std::map<int, double> table;
	if (sets.estimateZ_mode == 2)
	{
		Check_n_CreateFolder_Z_calibration(sets.strOutPath);
	}
	else
	{
		// read Z calibration table
		std::ifstream readFile;
		readFile.open(sets.strOutPath + "Zcal.txt");

		if (readFile.is_open())
		{
			while (!readFile.eof())
			{
				std::string str;
				std::getline(readFile, str);
				if (str.length() > 0)
				{
					int diss = str.find(':');
					std::string strZ = str.substr(0, diss);
					int iZ = std::stoi(strZ);
					std::string strCMperPIXEL = str.substr(diss + 1, str.length() - diss);
					double dCMperPIXEL = std::stod(strCMperPIXEL);
					table.insert({ iZ, dCMperPIXEL });
					std::cout << str << std::endl;
				}
			}
			readFile.close();
		}
		else {
			std::cout << "z calibraion read fail" << std::endl;
		}
	}

	
	StereoCameraParameter param;
	cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
    cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;
	cv::Rect validRoiL, validRoiR;
	if (!GetRectifiedMap(sets, param, Left_Stereo_Map1, Left_Stereo_Map2, Right_Stereo_Map1, Right_Stereo_Map2, validRoiL, validRoiR)) {
		std::cout << "GetRectifiedMap fail" << std::endl;
		return -1;
	}


	// depth map 표현
    cv::Ptr<cv::cuda::StereoSGM> sgm;
    sgm = cv::cuda::createStereoSGM(0, 256);

	cv::Mat frame_0, frame_1;


	int w = cvRound(cam_0.get(cv::CAP_PROP_FRAME_WIDTH));
	int h = cvRound(cam_0.get(cv::CAP_PROP_FRAME_HEIGHT));
	cv::Size sizeOutput(w*2, h*2);

	cv::VideoWriter demoVideo;
	if (sets.estimateZ_record)
	{
		// 결과 비디오 저장
		double fps = cam_0.get(cv::CAP_PROP_FPS);
		int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
		int delay = cvRound(1000 / fps);
		demoVideo.open(sets.strOutPath + "demo.mp4", fourcc, fps, sizeOutput);
	}
	

    while (true)
    {
		cam_0 >> frame_0;
		cam_1 >> frame_1;

		if (frame_0.empty() || frame_1.empty())
			break;

		// 보정
		cv::Mat img1r, img2r;
		cv::remap(frame_0, img1r, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LINEAR);
		cv::remap(frame_1, img2r, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LINEAR);

		frame_0 = img1r;
		frame_1 = img2r;

		// GPU로 데이터 복사 & depth map 계산
		cv::Mat left, right;
		cvtColor(frame_0, left, cv::COLOR_BGR2GRAY);
		cvtColor(frame_1, right, cv::COLOR_BGR2GRAY);
		cv::cuda::GpuMat d_left, d_right, d_disp;
		cv::Mat disp, dispColor;
		d_left.upload(left);
		d_right.upload(right);
		sgm->compute(d_left, d_right, d_disp);
		d_disp.download(disp);
		cv::normalize(disp, disp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
		cv::applyColorMap(disp, dispColor, cv::COLORMAP_JET);


		// disparity map
		cv::Mat xyz;
		cv::Mat floatDisp;
		float disparity_multiplier = 1.0f;
		if (disp.type() == CV_16S)
		{
			disparity_multiplier = 16.0f;
		}
		disp.convertTo(floatDisp, CV_32F, 1.0f / disparity_multiplier);
		reprojectImageTo3D(floatDisp, xyz, param.Q, true);
		
		// Z 캘리브레이션용 이미지 취득
		if (sets.estimateZ_mode == 2)
		{
			// 사각형 안의 평균 Z 값
			cv::Point2d lt(frame_0.cols / 2 - 10, frame_0.rows / 2 - 10), rb(frame_0.cols / 2 + 10, frame_0.rows / 2 + 10);
			double avgZ = GetAvgZ_rect(xyz, lt, rb);

			if ( std::fmod(avgZ, 1.0) < 1.0 )
			{
				// 테스트 결과 30cm ~ 110cm
				// 카메라 해상도를 높이고 L,R 카메라 사이를 넓히면 더 멀리까지 가능할 듯...
				// 110cm를 넘어가면 chess board 인식을 정확히 못한다.
				if (int(avgZ) > 30 && int(avgZ) < 110)
				{
					cv::imwrite(sets.strOutPath + "ZcalL/img_" + cv::format("%03d", int(avgZ)) + ".jpg", frame_0);
					std::cout << sets.strOutPath + "ZcalL/img_" + cv::format("%03d", int(avgZ)) + ".jpg" << std::endl;
				}
			}

			// 사각형
			cv::rectangle(frame_0, lt, rb, cv::Scalar(255, 255, 255));
			putText(frame_0, cv::format("%.2f cm", avgZ), cv::Point(lt.x, rb.y + 20), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
		}

		cv::Mat circle = frame_0.clone();
		houghtcircle(circle, xyz, table);
		

		// result display
		cv::Rect validRoiDisp = validRoiL & validRoiR;
		rectangle(frame_0, validRoiL, cv::Scalar(0, 0, 255), 3, 8);
		cv::putText(frame_0, "left", cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 0), 2);
		rectangle(frame_1, validRoiR, cv::Scalar(0, 0, 255), 3, 8);
		cv::putText(frame_1, "right", cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 0), 2);
		rectangle(dispColor, validRoiDisp, cv::Scalar(0, 0, 255), 3, 8);
		cv::putText(dispColor, "disparity", cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 0), 2);
		rectangle(circle, validRoiDisp, cv::Scalar(0, 0, 255), 3, 8);
		cv::putText(circle, "circle", cv::Point(20, 40), cv::FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(0, 255, 0), 2);

		cv::Mat canvas(sizeOutput, CV_8UC3);
		cv::Mat canvasPartL = canvas(cv::Rect(0, 0, w, h));
		frame_0.copyTo(canvasPartL);
		cv::Mat canvasPartR = canvas(cv::Rect(w, 0, w, h));
		frame_1.copyTo(canvasPartR);
		cv::Mat canvasPartDispColor = canvas(cv::Rect(0, h, w, h));
		dispColor.copyTo(canvasPartDispColor);
		cv::Mat canvasPartRB = canvas(cv::Rect(w, h, w, h));
		circle.copyTo(canvasPartRB);

		cv::imshow("result", canvas);


		// 결과 비디오 저장
		if (sets.estimateZ_record)
		{
			demoVideo << canvas;
		}

		if (cv::waitKey(1) == 27)
			break;
	}


	cv::destroyAllWindows();
    return 0;
}

// xyz 행렬에서 주어진 left-top(x, y) ~ right-bottom(x, y) 사이의 z축 평균 값을 반환한다.
double GetAvgZ_rect(const cv::Mat & matXYZ, cv::Point2d lt, cv::Point2d rb, const cv::Mat& mask)
{
	const double max_z = 1.0e4;
	double sum(0.0);
	int cnt(0);

	for (int y = lt.y; y < rb.y; y++)
	{
		for (int x = lt.x; x < rb.x; x++)
		{
			if (x < 0 || x >= matXYZ.cols)
				continue;
			if (y < 0 || y >= matXYZ.rows)
				continue;

			cv::Vec3f point = matXYZ.at<cv::Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
				continue;

			if (!mask.empty() && !mask.at<bool>(y - lt.y, x - lt.x))
				continue;

			sum += point[2];
			cnt++;
		}
	}

	if (cnt == 0)
		return 0;

	return sum / cnt;
}

bool Check_n_CreateFolder_Z_calibration(std::string strPath)
{
	bool ret(true);

	if (std::filesystem::exists(strPath))
	{
		if (!std::filesystem::exists(strPath + "/ZcalL/"))
			ret &= std::filesystem::create_directory(strPath + "/ZcalL/");
	}
	else
	{
		ret &= std::filesystem::create_directory(strPath);
		ret &= std::filesystem::create_directory(strPath + "/ZcalL/");
	}

	return ret;
}

bool GetRectifiedMap(const Settings &sets,
					 StereoCameraParameter &param,
					 cv::Mat &Left_Stereo_Map1, cv::Mat &Left_Stereo_Map2,
					 cv::Mat &Right_Stereo_Map1, cv::Mat &Right_Stereo_Map2,
					 cv::Rect &validRoiL, cv::Rect &validRoiR)
{
	// stereo calibrate 결과 값으로 보정맵 계산
	if (!ReadStereoCameraParameter(sets.strOutPath + "stereoCameraParameter.yml", param))
		return false;

	cv::stereoRectify(
        param.M1, param.D1,
        param.M2, param.D2,
        param.sizeImage1, param.R, param.T, param.R1, param.R2, param.P1, param.P2, param.Q,
            cv::CALIB_ZERO_DISPARITY, 1, param.sizeImage1, &validRoiL, &validRoiR);

    cv::initUndistortRectifyMap(param.M1,
                                param.D1,
                                param.R1,
                                param.P1,
                                param.sizeImage1,
                                CV_16SC2,
                                Left_Stereo_Map1,
                                Left_Stereo_Map2);

    cv::initUndistortRectifyMap(param.M2,
                                param.D2,
                                param.R2,
                                param.P2,
                                param.sizeImage2,
                                CV_16SC2,
                                Right_Stereo_Map1,
                                Right_Stereo_Map2);

	return true;	
}

double GetAvgZ_circle(const cv::Mat &matXYZ, const cv::Vec3f &circle)
{
	// 원의 외접 사각형에서 원의 내부 점의 Z값만 더하고,
	// 더한 점의 수로 나누면 평균 Z값이 나온다.
	const double max_z = 1.0e4;
	double sum(0.0);
	int cnt(0);

	cv::Point center = cv::Point(circle[0], circle[1]);
	int radius = circle[2];
	cv::Point2i rect_lt(center.x-radius, center.y-radius);
	cv::Point2i rect_rb(center.x+radius, center.y+radius);
	for (int y = rect_lt.y; y < rect_rb.y; y++)
	{
		for (int x = rect_lt.x; x < rect_rb.x; x++)
		{
			if (x < 0 || x >= matXYZ.cols)
				continue;
			if (y < 0 || y >= matXYZ.rows)
				continue;

			cv::Vec3f point = matXYZ.at<cv::Vec3f>(y, x);
			if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z)
				continue;

			// 중심이 (a, b)이고 반지름이 r인 원의 방정식
			// pow(x - a, 2) + pow(y - b, 2) = pow(r, 2)
			// pow는 제곱연산함수
			if (std::pow(x-center.x,2) + std::pow(y-center.y,2) > std::pow(radius, 2))
				continue;

			sum += point[2];
			cnt++;
		}
	}

	if (cnt == 0)
		return 0.0;

	return sum / cnt;
}

void houghtcircle(cv::Mat &img, const cv::Mat & matXYZ, std::map<int, double> table)
{
	cv::Mat gray;
    cvtColor(img, gray, cv::COLOR_BGR2GRAY);

    medianBlur(gray, gray, 5);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 gray.rows/16,  // change this value to detect circles with different distances to each other
                 100, 50, 10, 50 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );

    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
		double avgZ = GetAvgZ_circle(matXYZ, c);

		

		if (int(avgZ) > 30 && int(avgZ) < 110)
		{
			// circle center
			cv::circle( img, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
			// circle outline
			int radius = c[2];
			cv::circle( img, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
			
			double cmPpixel = table.find(int(avgZ))->second;
			double diameter = radius * 2 * cmPpixel;
			double area = CV_PI * std::pow(radius* cmPpixel, 2);

			int left = center.x - radius;
			int top = center.y - radius;
			int right = center.x + radius;
			int bottom = center.y + radius;

			std::string stravgZ = cv::format("avgZ=%.2f", avgZ);
			std::string strarea = cv::format("diameter=%.2f, area=%.2f", diameter, area);

			int baseLine;
			cv::Size labelSize = cv::getTextSize(stravgZ, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
			top = std::max(top - 10, labelSize.height);
			cv::rectangle(img, cv::Point(left, top - labelSize.height),
					cv::Point(left + labelSize.width, top + baseLine), cv::Scalar::all(255), cv::FILLED);
			cv::putText(img, stravgZ, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar());

			labelSize = cv::getTextSize(strarea, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
			top = std::min(bottom + 20, labelSize.height);
			cv::rectangle(img, cv::Point(left, bottom + labelSize.height + 10),
					cv::Point(left + labelSize.width, bottom - baseLine + 10), cv::Scalar::all(255), cv::FILLED);
			cv::putText(img, strarea, cv::Point(left, bottom + 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar());
		}
    }

}
#ifndef COMMON_HPP
#define COMMON_HPP

#include <opencv2/opencv.hpp>
#include <iostream>

struct Settings
{
	int Cam_Width;
	int Cam_Height;
	int CamL_id;
	int CamR_id;
	int board_w;
	int board_h;
	float square_size;				  	// chess board square size, cm
	std::string strOutPath;			  	// data path
	std::string estimateZ_videoFileL; 	// if estimateZ_mode == 0 than need file
	std::string estimateZ_videoFileR; 	// if estimateZ_mode == 0 than need file
	int estimateZ_validDistanceStart;	// logitech c920에서는 30cm ~ 110cm 범위에서 쓸만함, camera resolution과 chess board 크기에 따라 변경 가능
	int estimateZ_validDistanceEnd;		// logitech c920에서는 30cm ~ 110cm 범위에서 쓸만함, camera resolution과 chess board 크기에 따라 변경 가능
};

struct StereoCameraParameter
{
	cv::Size sizeImage1;
	cv::Size sizeImage2;

	// intrinsics
	cv::Mat M1; // cameraMatrix1;
	cv::Mat D1; // distCoeffs1;
	cv::Mat M2; // cameraMatrix2;
	cv::Mat D2; // distCoeffs2;

	// extrinsics
	cv::Mat R;	// RotationMatrix;
	cv::Mat T;	// TranslationVector;
	cv::Mat R1; // RotationMatrix1;
	cv::Mat R2; // RotationMatrix2;
	cv::Mat P1; // ProjectionMatrix1;
	cv::Mat P2; // ProjectionMatrix2;
	cv::Mat Q;	// PerspectiveTransformationMatrix;
};

bool ReadSettings(std::string strFile, Settings &setting)
{
	cv::FileStorage fs(strFile, cv::FileStorage::READ);

	if (!fs.isOpened())
	{
		std::cerr << "File open failed!" << std::endl;
		return false;
	}

	fs["Cam_Width"] >> setting.Cam_Width;
	fs["Cam_Height"] >> setting.Cam_Height;
	fs["CamL_id"] >> setting.CamL_id;
	fs["CamR_id"] >> setting.CamR_id;
	fs["board_w"] >> setting.board_w;
	fs["board_h"] >> setting.board_h;
	fs["square_size"] >> setting.square_size;
	fs["output_path"] >> setting.strOutPath;
	fs["estimateZ_videoFileL"] >> setting.estimateZ_videoFileL;
	fs["estimateZ_videoFileR"] >> setting.estimateZ_videoFileR;
	fs["estimateZ_validDistanceStart"] >> setting.estimateZ_validDistanceStart;
	fs["estimateZ_validDistanceEnd"] >> setting.estimateZ_validDistanceEnd;

	fs.release();

	std::cout << "read settings" << std::endl;
	std::cout << "Cam_Width:" << setting.Cam_Width << std::endl;
	std::cout << "Cam_Height:" << setting.Cam_Height << std::endl;
	std::cout << "CamL_id:" << setting.CamL_id << std::endl;
	std::cout << "CamR_id:" << setting.CamR_id << std::endl;
	std::cout << "board_w:" << setting.board_w << std::endl;
	std::cout << "board_h:" << setting.board_h << std::endl;
	std::cout << "square_size:" << setting.square_size << std::endl;
	std::cout << "output_path:" << setting.strOutPath << std::endl;
	std::cout << "estimateZ_videoFileL:" << setting.estimateZ_videoFileL << std::endl;
	std::cout << "estimateZ_videoFileR:" << setting.estimateZ_videoFileR << std::endl;
	std::cout << "estimateZ_validDistanceStart:" << setting.estimateZ_validDistanceStart << " cm" << std::endl;
	std::cout << "estimateZ_validDistanceEnd:" << setting.estimateZ_validDistanceEnd << " cm" << std::endl;

	return true;
}

bool WriteSettings(std::string strFile, Settings &setting)
{
	cv::FileStorage fs(strFile, cv::FileStorage::WRITE);

	if (!fs.isOpened())
	{
		std::cerr << "File open failed!" << std::endl;
		return false;
	}

	fs << "Cam_Width" << setting.Cam_Width;
	fs << "Cam_Height" << setting.Cam_Height;
	fs << "CamL_id" << setting.CamL_id;
	fs << "CamR_id" << setting.CamR_id;
	fs << "board_w" << setting.board_w;
	fs << "board_h" << setting.board_h;
	fs << "square_size" << setting.square_size;
	fs << "output_path" << setting.strOutPath;
	fs << "estimateZ_videoFileL" << setting.estimateZ_videoFileL;
	fs << "estimateZ_videoFileR" << setting.estimateZ_videoFileR;
	fs << "estimateZ_validDistanceStart" << setting.estimateZ_validDistanceStart;
	fs << "estimateZ_validDistanceEnd" << setting.estimateZ_validDistanceEnd;

	fs.release();

	std::cout << "write settings" << std::endl;
	std::cout << "Cam_Width:" << setting.Cam_Width << std::endl;
	std::cout << "Cam_Height:" << setting.Cam_Height << std::endl;
	std::cout << "CamL_id:" << setting.CamL_id << std::endl;
	std::cout << "CamR_id:" << setting.CamR_id << std::endl;
	std::cout << "board_w:" << setting.board_w << std::endl;
	std::cout << "board_h:" << setting.board_h << std::endl;
	std::cout << "square_size:" << setting.square_size << std::endl;
	std::cout << "output_path:" << setting.strOutPath << std::endl;
	std::cout << "estimateZ_videoFileL:" << setting.estimateZ_videoFileL << std::endl;
	std::cout << "estimateZ_videoFileR:" << setting.estimateZ_videoFileR << std::endl;
	std::cout << "estimateZ_validDistanceStart:" << setting.estimateZ_validDistanceStart << " cm" << std::endl;
	std::cout << "estimateZ_validDistanceEnd:" << setting.estimateZ_validDistanceEnd << " cm" << std::endl;

	return true;
}

bool ReadStereoCameraParameter(std::string strFile, StereoCameraParameter &param)
{
	cv::FileStorage fs(strFile, cv::FileStorage::READ);

	if (!fs.isOpened())
	{
		std::cerr << "Error: can not load the stereo camera parameters" << std::endl;
		return false;
	}

	fs["sizeImage1"] >> param.sizeImage1;
	fs["sizeImage2"] >> param.sizeImage2;

	// intrinsics
	fs["M1"] >> param.M1;
	fs["D1"] >> param.D1;
	fs["M2"] >> param.M2;
	fs["D2"] >> param.D2;

	// extrinsics
	fs["R"] >> param.R;
	fs["T"] >> param.T;
	fs["R1"] >> param.R1;
	fs["R2"] >> param.R2;
	fs["P1"] >> param.P1;
	fs["P2"] >> param.P2;
	fs["Q"] >> param.Q;

	fs.release();

	std::cout << "load stereo camera parameters" << std::endl;

	return true;
}

bool WriteStereoCameraParameter(std::string strFile, StereoCameraParameter &param)
{
	cv::FileStorage fs(strFile, cv::FileStorage::WRITE);

	if (!fs.isOpened())
	{
		std::cerr << "Error: can not save the stereo camera parameters" << std::endl;
		return false;
	}

	fs << "sizeImage1" << param.sizeImage1;
	fs << "sizeImage2" << param.sizeImage2;

	// intrinsics
	fs << "M1" << param.M1;
	fs << "D1" << param.D1;
	fs << "M2" << param.M2;
	fs << "D2" << param.D2;

	// extrinsics
	fs << "R" << param.R;
	fs << "T" << param.T;
	fs << "R1" << param.R1;
	fs << "R2" << param.R2;
	fs << "P1" << param.P1;
	fs << "P2" << param.P2;
	fs << "Q" << param.Q;

	fs.release();

	std::cout << "save stereo camera parameters" << std::endl;

	return true;
}

void SetCameraProperties(cv::VideoCapture &cap, const Settings &setting)
{
	// c920은 해상도에 따라 FPS가 자동으로 설정된다. V4L로는 임의 설정이 안된다.
	// 640x480 -> 30FPS
	// 800x600 -> 25FPS
	// 1280x960 -> 10FPS
	// 1920x1080 -> 5FPS
	if (!cap.set(cv::CAP_PROP_FRAME_WIDTH, setting.Cam_Width)){			std::cout << cap.getBackendName() << " not set " << cv::CAP_PROP_FRAME_WIDTH << std::endl;}
	if (!cap.set(cv::CAP_PROP_FRAME_HEIGHT, setting.Cam_Height)){		std::cout << cap.getBackendName() << " not set " << cv::CAP_PROP_FRAME_HEIGHT << std::endl;}
	if (!cap.set(cv::CAP_PROP_FPS, 30.0/1.0)){							std::cout << cap.getBackendName() << " not set " << cv::CAP_PROP_FPS << std::endl;}
	if (!cap.set(cv::CAP_PROP_EXPOSURE, 100)){							std::cout << cap.getBackendName() << " not set " << cv::CAP_PROP_EXPOSURE << std::endl;}
	if (!cap.set(cv::CAP_PROP_AUTOFOCUS, 0)){							std::cout << cap.getBackendName() << " not set " << cv::CAP_PROP_AUTOFOCUS << std::endl;}
	if (!cap.set(cv::CAP_PROP_FOCUS, 0)){								std::cout << cap.getBackendName() << " not set " << cv::CAP_PROP_FOCUS << std::endl;}
	if (!cap.set(cv::CAP_PROP_AUTO_WB, 0)){								std::cout << cap.getBackendName() << " not set " << cv::CAP_PROP_AUTO_WB << std::endl;}
	if (!cap.set(cv::CAP_PROP_WB_TEMPERATURE, 3800)){					std::cout << cap.getBackendName() << " not set " << cv::CAP_PROP_WB_TEMPERATURE << std::endl;}
}

#endif // !COMMON_HPP

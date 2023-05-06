#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <iostream>
#include "common.hpp"

void GetPts(Settings sets, std::vector<std::vector<cv::Point3f>> &objpoints,
            std::vector<std::vector<cv::Point2f>> &imgpointsL, std::vector<std::vector<cv::Point2f>> &imgpointsR,
            std::vector<cv::String> &imagesL, std::vector<cv::String> &imagesR,
            cv::Size &sizeImageL, cv::Size &sizeImageR);

double CheckCalibrationQuality(const std::vector<cv::String> &imagesL,
                             const std::vector<cv::String> &imagesR,
                             const std::vector<std::vector<cv::Point2f>> &imgpointsL,
                             const std::vector<std::vector<cv::Point2f>> &imgpointsR,
                             const cv::Mat &new_mtxL, const cv::Mat &new_mtxR,
                             const cv::Mat &distL, const cv::Mat &distR,
                             const cv::Mat &Fmat);

int main(int argc, char** argv)
{
    Settings sets;
	if (!ReadCommandLine(argc, argv, sets))
	{
		return -1;
	}

    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f>> objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f>> imgpointsL, imgpointsR;

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> imagesL, imagesR;

    cv::Size sizeImageL, sizeImageR;
    GetPts(sets, objpoints, imgpointsL, imgpointsR, imagesL, imagesR, sizeImageL, sizeImageR);

    

    /*
     * Performing camera calibration by
     * passing the value of known 3D points (objpoints)
     * and corresponding pixel coordinates of the
     * detected corners (imgpoints)
     */

    StereoCameraParameter param;
    param.sizeImage1 = sizeImageL;
    param.sizeImage2 = sizeImageR;

    param.M1 = cv::initCameraMatrix2D(objpoints, imgpointsL, sizeImageL, 0);
    param.M2 = cv::initCameraMatrix2D(objpoints, imgpointsR, sizeImageR, 0);


    // Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat
    // are calculated. Hence intrinsic parameters are the same.
    cv::Mat Emat, Fmat;

    int flag = 0;
    flag = cv::CALIB_FIX_ASPECT_RATIO +
        cv::CALIB_ZERO_TANGENT_DIST +
        cv::CALIB_USE_INTRINSIC_GUESS +
        cv::CALIB_SAME_FOCAL_LENGTH +
        cv::CALIB_RATIONAL_MODEL +
        cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5;

    // This step is performed to transformation between the two cameras and calculate Essential and
    // Fundamenatl matrix
    cv::stereoCalibrate(
        objpoints,
        imgpointsL, imgpointsR,
        param.M1, param.D1,
        param.M2, param.D2,
        sizeImageR, param.R, param.T, Emat, Fmat,
        flag,
        cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));
    
    CheckCalibrationQuality(imagesL,
                            imagesR,
                            imgpointsL,
                            imgpointsR,
                            param.M1, param.M2,
                            param.D1, param.D2,
                            Fmat);


    cv::Rect validRoiL, validRoiR;

    // Once we know the transformation between the two cameras we can perform
    // stereo rectification
    int flag2 = 0;
    flag2 = cv::CALIB_ZERO_DISPARITY;
    cv::stereoRectify(
        param.M1, param.D1,
        param.M2, param.D2,
        sizeImageR, param.R, param.T, param.R1, param.R2, param.P1, param.P2, param.Q,
            flag2, 1, sizeImageR, &validRoiL, &validRoiR);
    
    WriteStereoCameraParameter(sets.strOutPath + "stereoCameraParameter.yml", param);
    

    // Use the rotation matrixes for stereo rectification and camera intrinsics for undistorting the image
    // Compute the rectification map (mapping between the original image pixels and
    // their transformed values after applying rectification and undistortion) for left and right camera frames
    cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
    cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;

    cv::initUndistortRectifyMap(param.M1,
                                param.D1,
                                param.R1,
                                param.P1,
                                sizeImageR,
                                CV_16SC2,
                                Left_Stereo_Map1,
                                Left_Stereo_Map2);

    cv::initUndistortRectifyMap(param.M2,
                                param.D2,
                                param.R2,
                                param.P2,
                                sizeImageR,
                                CV_16SC2,
                                Right_Stereo_Map1,
                                Right_Stereo_Map2);


    // 영상을 조정하고 시차 지도를 구한다.
	cv::Mat canvas;
	double sf;
	int w, h;
    sf = 600. / std::max(sizeImageR.width, sizeImageR.height);
    w = cvRound(sizeImageR.width * sf);
    h = cvRound(sizeImageR.height * sf);
    canvas.create(h, w * 2, CV_8UC3);

	for (int i = 0; i < imagesL.size(); i++)
	{
        cv::Mat imgL = cv::imread(imagesL[i], 0), rimgL, cimgL;
        cv::remap(imgL, rimgL, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LINEAR);
        cvtColor(rimgL, cimgL, cv::COLOR_GRAY2BGR);
        cv::Mat canvasPartL = canvas(cv::Rect(0, 0, w, h));
        resize(cimgL, canvasPartL, canvasPartL.size(), 0, 0, cv::INTER_AREA);
        cv::Rect vroiL(cvRound(validRoiL.x * sf), cvRound(validRoiL.y * sf), cvRound(validRoiL.width * sf), cvRound(validRoiL.height * sf));
        rectangle(canvasPartL, vroiL, cv::Scalar(0, 0, 255), 3, 8);

        cv::Mat imgR = cv::imread(imagesR[i], 0), rimgR, cimgR;
        cv::remap(imgR, rimgR, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LINEAR);
        cvtColor(rimgR, cimgR, cv::COLOR_GRAY2BGR);
        cv::Mat canvasPartR = canvas(cv::Rect(w, 0, w, h));
        resize(cimgR, canvasPartR, canvasPartR.size(), 0, 0, cv::INTER_AREA);
        cv::Rect vroiR(cvRound(validRoiR.x * sf), cvRound(validRoiR.y * sf), cvRound(validRoiR.width * sf), cvRound(validRoiR.height * sf));
        rectangle(canvasPartR, vroiR, cv::Scalar(0, 0, 255), 3, 8);

        for (int j = 0; j < canvas.rows; j += 16)
        {
            line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
        }

		imshow("rectified", canvas);

		char c = (char)cv::waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}

    return 0;
}

void GetPts(Settings sets, std::vector<std::vector<cv::Point3f>> &objpoints,
            std::vector<std::vector<cv::Point2f>> &imgpointsL, std::vector<std::vector<cv::Point2f>> &imgpointsR,
            std::vector<cv::String> &imagesL, std::vector<cv::String> &imagesR,
            cv::Size &sizeImageL, cv::Size &sizeImageR)
{
    // Defining the dimensions of checkerboard
    int CHECKERBOARD[2]{sets.board_h, sets.board_w};

    // Defining the world coordinates for 3D points
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < CHECKERBOARD[1]; i++)
    {
        for (int j = 0; j < CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j * sets.square_size, i * sets.square_size, 0));
    }

    // Extracting path of individual image stored in a given directory
    // Path of the folder containing checkerboard images
    std::string pathL = sets.strOutPath + "stereoL/*.jpg";
    std::string pathR = sets.strOutPath + "stereoR/*.jpg";

    cv::glob(pathL, imagesL);
    cv::glob(pathR, imagesR);

    cv::Mat frameL, frameR, grayL, grayR;
    // vector to store the pixel coordinates of detected checker board corners
    std::vector<cv::Point2f> corner_ptsL, corner_ptsR;
    bool successL, successR;

    // Looping over all the images in the directory
    for (int i = 0; i < imagesL.size(); i++)
    {
        frameL = cv::imread(imagesL[i]);
        cv::cvtColor(frameL, grayL, cv::COLOR_BGR2GRAY);
        sizeImageL = grayL.size();

        frameR = cv::imread(imagesR[i]);
        cv::cvtColor(frameR, grayR, cv::COLOR_BGR2GRAY);
        sizeImageR = grayR.size();

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true
        successL = cv::findChessboardCorners(
            grayL,
            cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]),
            corner_ptsL);
        // cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        successR = cv::findChessboardCorners(
            grayR,
            cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]),
            corner_ptsR);
        // cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        /*
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display
         * them on the images of checker board
         */
        if ((successL) && (successR))
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(grayL, corner_ptsL, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            cv::cornerSubPix(grayR, corner_ptsR, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            cv::drawChessboardCorners(frameL, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_ptsL, successL);
            cv::drawChessboardCorners(frameR, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_ptsR, successR);

            objpoints.push_back(objp);
            imgpointsL.push_back(corner_ptsL);
            imgpointsR.push_back(corner_ptsR);
        }

        cv::imshow("ImageL", frameL);
        cv::imshow("ImageR", frameR);
        cv::waitKey(0);
    }

    cv::destroyAllWindows();
}

double CheckCalibrationQuality(const std::vector<cv::String> &imagesL,
                             const std::vector<cv::String> &imagesR,
                             const std::vector<std::vector<cv::Point2f>> &imgpointsL,
                             const std::vector<std::vector<cv::Point2f>> &imgpointsR,
                             const cv::Mat &new_mtxL, const cv::Mat &new_mtxR,
                             const cv::Mat &distL, const cv::Mat &distR,
                             const cv::Mat &Fmat)
{
    //////////////////////////////////////////////////////////////////
    // 보정 품질 검사:
	// 출력 기본 행렬은 사실상 모든 출력 정보를 포함하기 때문에
	// 에피폴라 기하 제약을 이용하여 보정 품질을 검사할 수 있다.
	// 에피폴라 기하 제약: m2^t*F*m1=0
	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
    std::vector<cv::Vec3f> linesL, linesR;
	for (int i = 0; i < imagesL.size(); i++)
	{
        // 왜곡이 제거된 상태에서 동작한다.
        cv::Mat imgPtL = cv::Mat(imgpointsL[i]);
        cv::undistortPoints(imgPtL, imgPtL, new_mtxL, distL, cv::Mat(), new_mtxL);
        cv::computeCorrespondEpilines(imgPtL, 1, Fmat, linesL);

        cv::Mat imgPtR = cv::Mat(imgpointsR[i]);
        cv::undistortPoints(imgPtR, imgPtR, new_mtxR, distR, cv::Mat(), new_mtxR);
        cv::computeCorrespondEpilines(imgPtR, 2, Fmat, linesR);

		for (int j = 0; j < imgpointsL.size(); j++)
		{
			double errij = fabs(imgpointsL[i][j].x * linesR[j][0] +
								imgpointsL[i][j].y * linesR[j][1] + linesR[j][2]) +
						   fabs(imgpointsR[i][j].x * linesL[j][0] +
								imgpointsR[i][j].y * linesL[j][1] + linesL[j][2]);
			err += errij;
		}
		npoints += imgpointsL.size();
	}
    std::cout << "average epipolar err = " << err / npoints << std::endl;

    return err / npoints;
}
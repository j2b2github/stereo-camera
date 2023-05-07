#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <fstream>

#include "common.hpp"

double Distance(const cv::Point &p1, const cv::Point &p2);

int main(int argc, char **argv)
{
    Settings sets;

    if (argc != 2)
    {
        std::cout << "need setting file(.yml)" << std::endl;
        return -1;
    }

    if (!ReadSettings(argv[1], sets))
	{
		std::cout << "read setting error." << std::endl;
		return -1;
	}

    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector<std::vector<cv::Point3f>> objpoints;

    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector<std::vector<cv::Point2f>> imgpointsL, imgpointsR;

    // Extracting path of individual image stored in a given directory
    std::vector<cv::String> imagesL, imagesR;

    cv::Size sizeImageL, sizeImageR;

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
    std::string pathL = sets.strOutPath + "ZcalL/*.jpg";
    // std::string pathL = sets.strOutPath + "ZcalL/img_180.jpg";

    cv::glob(pathL, imagesL);
    // cv::glob(pathR, imagesR);

    cv::Mat frameL, frameR, grayL, grayR;
    // vector to store the pixel coordinates of detected checker board corners
    std::vector<cv::Point2f> corner_ptsL, corner_ptsR;
    bool successL, successR;

    std::string fullString;

    // Looping over all the images in the directory
    for (int i = 0; i < imagesL.size(); i++)
    {
        frameL = cv::imread(imagesL[i]);
        cv::cvtColor(frameL, grayL, cv::COLOR_BGR2GRAY);
        sizeImageL = grayL.size();

        // Finding checker board corners
        // If desired number of corners are found in the image then success = true
        successL = cv::findChessboardCorners(
            grayL,
            cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]),
            corner_ptsL);

        /*
         * If desired number of corner are detected,
         * we refine the pixel coordinates and display
         * them on the images of checker board
         */
        if (successL)
        {
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            // refining pixel coordinates for given 2d points.
            cv::cornerSubPix(grayL, corner_ptsL, cv::Size(11, 11), cv::Size(-1, -1), criteria);

            // Displaying the detected corner points on the checker board
            // cv::drawChessboardCorners(frameL, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_ptsL, successL);

            objpoints.push_back(objp);
            imgpointsL.push_back(corner_ptsL);

            cv::Point iLB = cv::Point(imgpointsL[i][0].x, imgpointsL[i][0].y);
            cv::Point iLT = cv::Point(imgpointsL[i][CHECKERBOARD[0] - 1].x, imgpointsL[i][CHECKERBOARD[0] - 1].y);
            cv::Point iRT = cv::Point(imgpointsL[i][CHECKERBOARD[0] * CHECKERBOARD[1] - 1].x, imgpointsL[i][CHECKERBOARD[0] * CHECKERBOARD[1] - 1].y);
            cv::Point iRB = cv::Point(imgpointsL[i][CHECKERBOARD[0] * CHECKERBOARD[1] - CHECKERBOARD[0]].x, imgpointsL[i][CHECKERBOARD[0] * CHECKERBOARD[1] - CHECKERBOARD[0]].y);
            cv::circle(frameL, iLB, 2, cv::Scalar(0, 255, 0), 2);
            cv::circle(frameL, iLT, 2, cv::Scalar(0, 255, 255), 2);
            cv::circle(frameL, iRT, 2, cv::Scalar(255, 255, 255), 2);
            cv::circle(frameL, iRB, 2, cv::Scalar(255, 255, 0), 2);

            double iwidth = Distance(iLT, iRT);
            double iheight = Distance(iLT, iLB);

            cv::Point oLB = cv::Point(objpoints[i][0].x, objpoints[i][0].y);
            cv::Point oLT = cv::Point(objpoints[i][CHECKERBOARD[0] - 1].x, objpoints[i][CHECKERBOARD[0] - 1].y);
            cv::Point oRT = cv::Point(objpoints[i][CHECKERBOARD[0] * CHECKERBOARD[1] - 1].x, objpoints[i][CHECKERBOARD[0] * CHECKERBOARD[1] - 1].y);
            cv::Point oRB = cv::Point(objpoints[i][CHECKERBOARD[0] * CHECKERBOARD[1] - CHECKERBOARD[0]].x, objpoints[i][CHECKERBOARD[0] * CHECKERBOARD[1] - CHECKERBOARD[0]].y);

            double owidth = Distance(oLT, oRT);
            double oheight = Distance(oLT, oLB);

            std::cout << imagesL[i] << ", iwidth=" << iwidth << ", iheight=" << iheight << ", owidth=" << owidth << ", oheight=" << oheight
                      << ", cm / pixel = (" << owidth / iwidth << ", " << oheight / iheight << ")"
                      << ", (width + height) / 2 = [" << ((owidth / iwidth) + (oheight / iheight)) / 2 << "] cm/pixel" << std::endl;

            size_t start = imagesL[i].find_last_of('_') + 1;
            size_t end = imagesL[i].find_last_of('.');
            std::string strDistance = imagesL[i].substr(start, 3);
            std::string out = strDistance + cv::format(":%f\n", ((owidth / iwidth) + (oheight / iheight)) / 2);
            std::cout << out << std::endl;
            fullString += out;
        }

        cv::imshow(imagesL[i], frameL);

        cv::waitKey(0);
        cv::destroyWindow(imagesL[i]);
    }

    std::ofstream writeFile;
    writeFile.open(sets.strOutPath + "Zcal.txt");
    writeFile.write(fullString.c_str(), fullString.size());
    writeFile.close();
    return 0;

    cv::destroyAllWindows();
}

double Distance(const cv::Point &p1, const cv::Point &p2)
{

    double distance;

    // 피타고라스의 정리
    // pow(x,2) x의 2승,  sqrt() 제곱근
    distance = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));

    return distance;
}
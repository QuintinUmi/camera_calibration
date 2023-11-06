#ifndef _UNDISTORTION_H_
#define _UNDISTORTION_H_


#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "camera_caliberation_chessboard.h"


class Undistortion{

    public:

        Undistortion();
        Undistortion(cv::Mat cameraMatrix, cv::Mat disCoffes);
        bool undistortion(cv::Mat* srcImage, cv::Mat* undistortedImage);

    private:

        cv::Mat cameraMatrix;
        cv::Mat disCoffes;
};




#endif
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "camera_caliberation_chessboard.h"
#include "undistortion.h"


Undistortion::Undistortion(){

}
Undistortion::Undistortion(cv::Mat cameraMatrix, cv::Mat disCoffes){
    this->cameraMatrix = cameraMatrix;
    this->disCoffes = disCoffes;
}

bool Undistortion::undistortion(cv::Mat* srcImage, cv::Mat* undistortedImage)
{
    cv::undistort(*srcImage, *undistortedImage, this->cameraMatrix, this->disCoffes);
    return true;
}
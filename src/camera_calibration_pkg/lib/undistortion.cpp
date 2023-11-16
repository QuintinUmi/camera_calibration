#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "camera_calibration_tool.h"
#include "undistortion.h"

using namespace cct;

Undistortion::Undistortion(){

}
Undistortion::Undistortion(cv::Mat cameraMatrix, cv::Mat disCoffes, cv::Size image_size, double alpha){
    this->cameraMatrix = cameraMatrix;
    this->disCoffes = disCoffes;
    this->image_size = image_size;
    this->alpha = alpha;


    cv::Mat map1, map2;
    cv::Mat newMap1, newMap2;

    cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, disCoffes, image_size, alpha);
    cv::initUndistortRectifyMap(cameraMatrix, disCoffes, cv::Mat(), newCameraMatrix, image_size, CV_32FC2, this->map1, this->map2);
}

bool Undistortion::undistortion_process(cv::Mat &srcImage, cv::Mat &undistortedImage)
{
    cv::remap(srcImage, undistortedImage, this->map1, this->map2, cv::INTER_LINEAR);
    return true;
}
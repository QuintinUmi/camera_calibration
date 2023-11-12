#ifndef _UNDISTORTION_H_
#define _UNDISTORTION_H_


#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "camera_caliberation_tool.h"


class Undistortion{

    public:

        Undistortion();
        Undistortion(cv::Mat cameraMatrix, cv::Mat disCoffes, cv::Size image_size, double alpha = 1.0);
        bool undistortion_process(cv::Mat &srcImage, cv::Mat &undistortedImage);

    private:

        cv::Mat cameraMatrix;
        cv::Mat disCoffes;
        cv::Size image_size;
        double alpha;

        cv::Mat map1, map2;
        cv::Mat newCameraMatrix;
        cv::Mat newDisCoffes; 
};




#endif
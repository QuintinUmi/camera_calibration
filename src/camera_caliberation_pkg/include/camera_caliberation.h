#ifndef _CALIBERATION_TOOL_H_
#define _CALIBERATION_TOOL_H_

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"        

#include "image_transport/image_transport.h"

#include "param_code.h"


using namespace std;

class CameraCaliberation
{
    public:

        bool get_images_from_path(string path);
        bool find_chessboard_corners(cv::Mat* srcImage);
        bool objPoint2;

    private:

        vector<cv::String> imagePaths;
        vector<cv::Point3f> worldPoints;
        vector<vector<cv::Point3f>> objectPoints;
        vector<vector<cv::Point2f>> imagePoints;
};


#endif
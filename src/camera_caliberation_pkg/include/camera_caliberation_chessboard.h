#ifndef _CALIBERATION_TOOL_H_
#define _CALIBERATION_TOOL_H_

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <fstream>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"        

#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "param_code.h"


using namespace std;

class CamCalChessboard
{
    public:

        CamCalChessboard(cv::Size chessboardSize, double squareSize);
        bool get_images_from_path(string path);
        vector<cv::Point2f> find_image_chessboard_corners(cv::Mat* srcImage, bool cornerShow = false, int criteriaIterTimes = 100, double iterDifference = 0.001);
        vector<cv::Point3f> find_object_chessboard_corners();
        bool caliberation_process(bool cornerShow = false, int criteriaIterTimes = 100, double iterDifference = 0.001);
        bool save_caliberation_parm_yaml(string path);

    private:

        vector<cv::String> imagePaths;
        cv::Size imgSize;

        cv::Size chessboardSize;
        double squareSize;

        vector<cv::Point3f> worldPoints;
        vector<vector<cv::Point3f>> objectPoints;
        vector<vector<cv::Point2f>> imagePoints;

        cv::Mat cameraMatrix; 
        cv::Mat disCoffes; 
        cv::Mat rvecs; 
        cv::Mat tvecs; 
};



#endif
#ifndef _calibration_CHESSBOARD_H_
#define _calibration_CHESSBOARD_H_

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


namespace cct{


    class CamCalChessboard
    {
        public:

            CamCalChessboard(cv::Size chessboardSize, double squareSize);
            CamCalChessboard();
            bool get_images_from_path(string path, string image_format);
            int get_images_num();
            void show_src_image(int index);
            cv::Mat get_src_image(int index, int flags = 1); 
            vector<cv::Point2f> find_image_chessboard_corners(cv::Mat* srcImage, bool cornerShow = false, int criteriaIterTimes = 100, double iterDifference = 0.001);
            vector<cv::Point3f> find_object_chessboard_corners();
            bool calibration_process(bool cornerShow = false, int criteriaIterTimes = 100, double iterDifference = 0.001);
            bool save_calibration_parm_yaml(string path);

        private:

            vector<cv::String> imagePaths;
            std::string searchPath;
            string image_format;
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

            cv::Mat newCameraMatrixAlpha1;
            cv::Mat newDisCoffesAlpha1; 
            cv::Mat newCameraMatrixAlpha0;
            cv::Mat newDisCoffesAlpha0; 
            vector<vector<cv::Point3f>> newObjectPoints;
            vector<vector<cv::Point2f>> newImagePoints;
            
    };



    class CamCalExt
    {
        
        public:

            CamCalExt();
            CamCalExt(CamCalChessboard camCalCB);
            CamCalExt(cv::Mat cameraMatrix, cv::Mat disCoffes);

            bool set_points_n_points(vector<cv::Point3f> worldPoints, vector<cv::Point2f> imagePoints);

            bool set_intrinsics(cv::Mat cameraMatrix, cv::Mat disCoffes);
            bool set_extrinsics(cv::Mat rvec, cv::Mat tvec);

            vector<cv::Mat> ext_cal_one_frame();

            void mapping_3d_to_2d_one_frame(vector<cv::Point3f> &worldPoints, vector<cv::Point2f> &imagePoints, cv::Mat rvec = cv::Mat(), cv::Mat tvec = cv::Mat(), 
                                            cv::Mat cameraMatrix = cv::Mat(), cv::Mat disCoffes = cv::Mat());
            // void mapping_points_3d_to_2d(vector<cv::Point3f> &worldPoints, vector<cv::Point2f> &imagePoints, cv::Mat rvec, cv::Mat tvec, 
            //                                 cv::Mat cameraMatrix, cv::Mat disCoffes = cv::Mat());

        private:

            vector<cv::Point3f> worldPoints;
            vector<cv::Point2f> imagePoints;
            cv::Size imgSize;



            cv::Mat cameraMatrix;
            cv::Mat disCoffes;
            cv::Mat rvec;
            cv::Mat tvec;


    };

    vector<cv::String> get_images_from_path(string path, string image_format);
    
}

#endif
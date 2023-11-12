#ifndef SPACE_3D_DRAWING_H_
#define SPACE_3D_DRAWING_H_

#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <unistd.h>
#include <stdlib.h>

#include "opencv2/opencv.hpp"        

#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "param_code.h"


using namespace std;


class Draw3D{

    public:

        Draw3D(float unitLength = 1.0 ,float scaleX = 1.0, float scaleY = 1.0, float scaleZ = 1.0);

        void write_in(cv::Point3f &dst, float x, float y, float z);
        void write_in(vector<cv::Point3f> &dst, float x, float y, float z);


        void set_scale(float scaleX = 1.0, float scaleY = 1.0, float scaleZ = 1.0);
        void set_unitlen(float unitLength);

        vector<vector<cv::Point3f>> draw_ortho_coordinate_3d(float cx = 0.0, float cy = 0.0, float cz = 0.0, float density = 0.1);
        void draw_ortho_coordinate_2d(cv::Mat &imgInputOutput, cv::Mat cameraMatrix, cv::Mat disCoffes, cv::Mat rvec, cv::Mat tvec,
                                        float cx = 0.0, float cy = 0.0, float cz = 0.0);

        void transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Mat rvec, cv::Mat tvec);
        void transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, 
                                        float rx, float ry, float rz, float tx, float ty, float tz);

        void mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Point3f surfaceNorVec);
        void mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, float surNorX, float surNorY, float surNorZ);

    private:

        float unitLength;
        float scaleX;
        float scaleY;
        float scaleZ;

        cv::Mat scaleMatrix = (cv::Mat_<float>(3, 3) << 1.0, 0.0, 0.0,
                                                        0.0, 1.0, 0.0,
                                                        0.0, 0.0, 1.0);
};

#endif
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
// #include "apriltag/apriltag.h" 
#include "opencv2/aruco/charuco.hpp"    

#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "param_code.h"


using namespace std;


namespace drt{

    class Draw3D{

        public:
            
            Draw3D(float unitLength = 1.0 ,float scaleX = 1.0, float scaleY = 1.0, float scaleZ = 1.0, cv::Mat cameraMatrix = cv::Mat(), cv::Mat disCoffes = cv::Mat());
            Draw3D(cv::Mat cameraMatrix, cv::Mat disCoffes);

            void write_in(cv::Point3f &dst, float x, float y, float z);
            void write_in(vector<cv::Point3f> &dst, float x, float y, float z);


            void set_scale(float scaleX = 1.0, float scaleY = 1.0, float scaleZ = 1.0);
            void set_unitlen(float unitLength);

            vector<vector<cv::Point3f>> draw_ortho_coordinate_3d(cv::Point3f centerPoint, float density = 0.1);
            vector<vector<cv::Point3f>> draw_ortho_coordinate_3d(float cx = 0.0, float cy = 0.0, float cz = 0.0, float density = 0.1);
            void draw_ortho_coordinate_2d(cv::Mat &imgInputOutput, cv::Mat cameraMatrix, cv::Mat disCoffes, cv::Mat rvec, cv::Mat tvec,
                                            float cx = 0.0, float cy = 0.0, float cz = 0.0);

            void transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Mat rvec, cv::Mat tvec);
            void transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, 
                                            float rx, float ry, float rz, float tx, float ty, float tz);

            void mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Point3f surfaceNorVec);
            void mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, float surNorX, float surNorY, float surNorZ);

            void setparam_image_perspective_3d(cv::Mat cameraMatrix, cv::Mat disCoffes,
                                    cv::Point3f imgOriPoint, cv::Size imgSizeIn3d, cv::Mat offsetRvec = cv::Mat(), cv::Mat offsetTvec = cv::Mat());
            void paste_image_perspective_3d(cv::Mat &srcImage, cv::Mat &dstImage, bool remove_background_color, bool center_image_axis, cv::Mat rvec, cv::Mat tvec);
            void paste_image_perspective_3d(cv::Mat &srcImage, cv::Mat &dstImage, bool remove_background_color, bool center_image_axis, vector<cv::Mat> rvecs, vector<cv::Mat> tvecs);
            void paste_image_perspective_3d(cv::Mat &srcImage, cv::Mat &dstImage, bool remove_background_color, bool center_image_axis, cv::Mat cameraMatrix, cv::Mat disCoffes, 
                                            cv::Mat rvec, cv::Mat tvec, cv::Point3f imgOriPoint, cv::Size imgSizeIn3d, cv::Mat offsetRvec = cv::Mat(), cv::Mat offsetTvec = cv::Mat());
            void paste_image_perspective_3d(cv::Mat &srcImage, cv::Mat &dstImage, bool remove_background_color, bool center_image_axis, cv::Mat cameraMatrix, cv::Mat disCoffes, 
                                            vector<cv::Mat> rvecs, vector<cv::Mat> tvecs, cv::Point3f imgOriPoint, cv::Size imgSizeIn3d, cv::Mat offsetRvec = cv::Mat(), cv::Mat offsetTvec = cv::Mat());

            void center_image_scale(cv::Mat &srcImage, cv::Mat &dstImage);
            void center_image_scale(cv::Mat &srcImage, cv::Mat &dstImage, float scaleX, float scaleY, int flags = 1, int borderMode = 0, const cv::Scalar &borderValue = cv::Scalar());



            // cv::Mat cal_2vec_rvec(cv::Point3f vecOri, cv::Point3f vecDst);

        private:

            float unitLength;
            float scaleX;
            float scaleY;
            float scaleZ;

            cv::Mat scaleMatrix = (cv::Mat_<float>(3, 3) << 1.0, 0.0, 0.0,
                                                            0.0, 1.0, 0.0,
                                                            0.0, 0.0, 1.0);

            // cv::Mat cameraMatrix;
            // cv::Mat disCoffes;


            cv::Mat setCameraMatrix;
            cv::Mat setDisCoffes;
            cv::Mat setRvec;
            cv::Mat setTvec;                             
            cv::Point3f setImgOriPoint = cv::Point3f(0.0, 0.0, 0.0);
            cv::Size setImgSizeIn3d = cv::Size(1.0, 1.0);
            cv::Mat setOffsetRvec = cv::Mat();
            cv::Mat setOffsetTvec = cv::Mat();
    };


    class ArucoM{

        public:

            ArucoM();
            ArucoM(int dictionaryName, vector<int> selectedIds, vector<float> markerRealLength = vector<float>{1.0}, cv::Mat cameraMatrix = cv::Mat(), cv::Mat disCoffes = cv::Mat());
            ArucoM(cv::Ptr<cv::aruco::Dictionary> markerDictionary, vector<int> selectedIds, vector<float> markerRealLength = vector<float>{1.0}, cv::Mat cameraMatrix = cv::Mat(), cv::Mat disCoffes = cv::Mat());
            ~ArucoM();

            void create();
            void aruco_hash_init();
            void release();

               
            void set_aruco(int dictionaryName, vector<int> selectedIds, vector<float> markerRealLength = vector<float>{1.0});
            void set_aruco(cv::Ptr<cv::aruco::Dictionary> markerDictionary, vector<int> selectedIds, vector<float> markerRealLength = vector<float>{1.0});
            void sel_aruco_ids(vector<int> selectedIds);
            void set_aruco_real_length(vector<float> markerRealLength);
            void set_camera_intrinsics(cv::Mat cameraMatrix, cv::Mat disCoffes);

            vector<cv::Mat> generate_aruco_marker(int markerSize);
            vector<cv::Mat> generate_aruco_marker(int dictionaryName, vector<int> selectedIds, int markerSize);
            void generate_aruco_inner(int markerSize);
            void generate_aruco_inner(int dictionaryName, vector<int> selectedIds, int markerSize);

            void detect_aruco(cv::Mat &inputImage, cv::OutputArrayOfArrays markerCorners, vector<int> markerIds);

            void ext_calib_single_arucos(cv::Mat &inputImage, int targetId, 
                                        vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs);
            
            void set_target_id_hash(vector<int> targetIds);
            void release_target_id_hash();
            void ext_calib_multipul_arucos(cv::Mat &inputImage, vector<cv::Mat> &rvecs, vector<cv::Mat> &tvecs, vector<int> detectedIds);

            void aruco_marker_save(cv::String imageSavePath, cv::String imageFormat, vector<cv::Mat> arucoMarkerImages, int dictionaryName, bool showImage);

        private:
            
            int *aruco_hash = NULL;
            bool *targetId_hash = NULL;
            vector<int> targetIds; 

	        cv::Ptr<cv::aruco::Dictionary> markerDictionary;
            vector<int> selectedIds;
            cv::Ptr<cv::aruco::DetectorParameters> dParameters;
            vector<cv::Mat> markerImage;

            vector<float> markerRealLength;

            cv::Mat cameraMatrix;
            cv::Mat disCoffes;

    };

}

#endif
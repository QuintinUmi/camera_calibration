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

#include "camera_calibration_tool.h"
#include "drawing_tool.h"
#include "param_code.h"

#define PI 3.14159265358979324


using namespace std;
using namespace cct;
using namespace drt;


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Aruco_Ext_Calib");
    ros::NodeHandle rosHandle;
    
    cv::String yamlPath;
    std::string imageFormat;
    cv::String imageSavePath;
    cv::String imageLoadPath;

    cv::Size chessboardSize;
    double squareSize;

    rosHandle.param("yaml_save_path", yamlPath, cv::String("~/"));
    rosHandle.param("image_save_path", imageSavePath, cv::String("~/"));
    rosHandle.param("image_load_path", imageLoadPath, cv::String("~/"));
    rosHandle.param("image_format", imageFormat, cv::String("png"));

    int dictionaryName;
    vector<int> ids;
    int markerSize;
    vector<float> arucoRealLength;

    rosHandle.param("dictionary_name", dictionaryName, 10);
    rosHandle.param("selected_ids", ids, vector<int>{});
    rosHandle.param("aruco_marker_size", markerSize, 500);
    rosHandle.param("aruco_real_length", arucoRealLength, vector<float>{1.0});
    

    cv::String intrinsicsPath = yamlPath + "calibration_param.yaml";
    cv::FileStorage fs(intrinsicsPath, cv::FileStorage::READ);
    int image_width{0}, image_height{0};
    fs["imageWidth"] >> image_width;
    fs["imageHeight"] >> image_height;

    cv::Size image_size = cv::Size(image_width, image_height);

    cv::Mat cameraMatrix, disCoffes, newCameraMatrix, newDisCoffes;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["disCoffes"] >> disCoffes;
    fs["newCameraMatrix"] >> newCameraMatrix;
    fs["newDisCoffes"] >> newDisCoffes;
    fs.release();
    std::cout << cameraMatrix << std::endl;
    std::cout << disCoffes << std::endl;
    std::cout << image_size << std::endl;


    cv::aruco::DICT_6X6_250;
    ArucoM arucoMarker(dictionaryName, ids, arucoRealLength, newCameraMatrix, newDisCoffes);
    vector<cv::Mat> arucoMarkerImgs;


    Draw3D d3d(47.62, 1, 1, 1, newCameraMatrix, newDisCoffes);
    d3d.setparam_image_perspective_3d(newCameraMatrix, newDisCoffes, cv::Point3f(0, 0, 0), cv::Size(47.62 * 2, 47.62 * 2), 
                                        (cv::Mat_<float>(3, 1) << 0, 0, -PI/2));

    vector<cv::String> imagePaths;
    imagePaths = cct::get_images_from_path(imageLoadPath, imageFormat);


    cv::Mat pasteImage = cv::imread("/home/quintinumi/test.jpeg");
    for(int i = 0; i < imagePaths.size(); i++){

        std::cout << imagePaths[i] << std::endl;
        cv::Mat imgDetect = cv::imread(imagePaths[i]);
        vector<cv::Mat> rvecs, tvecs;
        
        vector<vector<cv::Point2f>> testPoint;
        vector<int> testIds;
        // arucoMarker.detect_aruco(imgDetect, testPoint, testIds);
        arucoMarker.ext_calib_single_arucos(imgDetect, 150, rvecs, tvecs);

        std::cout << rvecs.empty() << std::endl;
        
        d3d.paste_image_perspective_3d(pasteImage, imgDetect, true, true, rvecs, tvecs);

        cv::imshow("Show paste image", imgDetect);
        cv::waitKey(0);

    }
    // cv::namedWindow("test1", cv::WINDOW_NORMAL);

    // cv::VideoCapture inputVideo;
    // inputVideo.open(0);
    // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // for(int i = 0; i < imagePaths.size(); i++) {
    //     cv::Mat image, imageCopy;
    //     // inputVideo.retrieve(image);
    //     image = cv::imread(imagePaths[i]);
    //     image.copyTo(imageCopy);
    //     std::vector<int> ids;
    //     std::vector<std::vector<cv::Point2f> > corners;
    //     cv::aruco::detectMarkers(image, dictionary, corners, ids);
    //     // if at least one marker detected
    //     if (ids.size() > 0)
    //         cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
    //     cv::imshow("test1", imageCopy);
    //     cv::waitKey(0);
    //     char key = (char) cv::waitKey(0);
    //     if (key == 27)
    //         break;
    //     }

    
    // cv::namedWindow("test1", cv::WINDOW_NORMAL);

    // cv::VideoCapture inputVideo;
    // inputVideo.open(0);
    // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // while (inputVideo.grab()) {
    //     cv::Mat image, imageCopy;
    //     inputVideo.retrieve(image);
    //     image.copyTo(imageCopy);
    //     std::vector<int> ids;
    //     std::vector<std::vector<cv::Point2f> > corners;
    //     cv::aruco::detectMarkers(image, dictionary, corners, ids);
    //     // if at least one marker detected
    //     if (ids.size() > 0)
    //         cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
    //     cv::imshow("test1", imageCopy);
    //     cv::waitKey(1);
    //     char key = (char) cv::waitKey(1);
    //     if (key == 27)
    //         break;
    //     } 

    return 0;
}

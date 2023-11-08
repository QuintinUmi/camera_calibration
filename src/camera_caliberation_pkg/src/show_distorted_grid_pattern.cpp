#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h>

#include "camera_caliberation_chessboard.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "undistortion");
    ros::NodeHandle rosHandle;
    
    cv::String yamlPath;
    cv::String instrinsicsFilePath;

    rosHandle.param("yaml_save_path", yamlPath, cv::String("~/"));

    instrinsicsFilePath = yamlPath + "caliberation_param.yaml";
    cv::FileStorage fs(instrinsicsFilePath, cv::FileStorage::READ);
    int image_width{0}, image_height{0};
    fs["imageWidth"] >> image_width;
    fs["imageHeight"] >> image_height;

    cv::Size image_size = cv::Size(image_width, image_height);

    cv::Mat cameraMatrix, disCoffes;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["disCoffes"] >> disCoffes;
    fs.release();


    printf("%d, %d\n", image_width, image_height);

    cv::Mat gridPattern(image_height, image_width, CV_8UC3, cv::Scalar(255, 255, 255));

    int cols = 41;
    for(int i = 0; i <= (cols - 1) / 2; i++){

        cv::Point2i pt1(image_width / 2 + image_width / cols * i, 0), pt2(image_width / 2 + image_width / cols * i, image_height);
        if(i == 0){
            cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 255), 4);
        }
        cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 255), 2);
        pt1.x = image_width / 2 - image_width / cols * i, 0; pt2.x = image_width / 2 - image_width / cols * i;
        cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 255), 2);
    }
    int rows = image_height / (image_width / cols);
    for(int i = 0; i <= (rows - 1) / 2; i++){

        cv::Point2i pt1(0, image_height / 2 + image_width / cols * i), pt2(image_width, image_height / 2 + image_width / cols * i);
        if(i == 0){
            cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 255), 4);
        }
        cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 255), 2);
        pt1.y = image_height / 2 - image_width / cols * i, 0; pt2.y = image_height / 2 - image_width / cols * i;
        cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 255), 2);
    }


    cv::Mat distortedGridPattern(image_height, image_width, CV_8UC3, cv::Scalar(255, 255, 255));

    cv::Mat map1, map2;
    cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, disCoffes, image_size, 1.0);
    cv::initUndistortRectifyMap(cameraMatrix, disCoffes, cv::Mat(), newCameraMatrix, image_size, CV_16SC2, map1, map2);
    cv::remap(gridPattern, distortedGridPattern, map2, map1, cv::INTER_LINEAR);



    gridPattern = cv::Mat(image_height, image_width, CV_8UC3, cv::Scalar(255, 255, 255));

    for(int i = 0; i <= (cols - 1) / 2; i++){

        cv::Point2i pt1(image_width / 2 + image_width / cols * i, 0), pt2(image_width / 2 + image_width / cols * i, image_height);
        if(i == 0){
            cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 0), 4);
        }
        cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 0), 2);
        pt1.x = image_width / 2 - image_width / cols * i, 0; pt2.x = image_width / 2 - image_width / cols * i;
        cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 0), 2);
    }
    for(int i = 0; i <= (rows - 1) / 2; i++){

        cv::Point2i pt1(0, image_height / 2 + image_width / cols * i), pt2(image_width, image_height / 2 + image_width / cols * i);
        if(i == 0){
            cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 0), 4);
        }
        cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 0), 2);
        pt1.y = image_height / 2 - image_width / cols * i, 0; pt2.y = image_height / 2 - image_width / cols * i;
        cv::line(gridPattern, pt1, pt2, cv::Scalar(0, 0, 0), 2);
    }

    cv::Mat outputImage2;
    cv::addWeighted(gridPattern, 0.5, distortedGridPattern, 0.5, 3.0, outputImage2);


    for(int i = 0; i <= (cols - 1) / 2; i++){

        cv::Point2i pt1(image_width / 2 + image_width / cols * i, 0), pt2(image_width / 2 + image_width / cols * i, image_height);
        if(i == 0){
            cv::line(distortedGridPattern, pt1, pt2, cv::Scalar(0, 0, 255), 4);
        }
        cv::line(distortedGridPattern, pt1, pt2, cv::Scalar(0, 0, 0), 2);
        pt1.x = image_width / 2 - image_width / cols * i, 0; pt2.x = image_width / 2 - image_width / cols * i;
        cv::line(distortedGridPattern, pt1, pt2, cv::Scalar(0, 0, 0), 2);
    }
    for(int i = 0; i <= (rows - 1) / 2; i++){

        cv::Point2i pt1(0, image_height / 2 + image_width / cols * i), pt2(image_width, image_height / 2 + image_width / cols * i);
        if(i == 0){
            cv::line(distortedGridPattern, pt1, pt2, cv::Scalar(0, 0, 255), 4);
        }
        cv::line(distortedGridPattern, pt1, pt2, cv::Scalar(0, 0, 0), 2);
        pt1.y = image_height / 2 - image_width / cols * i, 0; pt2.y = image_height / 2 - image_width / cols * i;
        cv::line(distortedGridPattern, pt1, pt2, cv::Scalar(0, 0, 0), 2);
    }

    

    cv::Mat outputImage1 = distortedGridPattern;
    cv::imshow("Distorted Grid Pattern1", outputImage1);
    cv::imshow("Distorted Grid Pattern2", outputImage2);
    cv::waitKey(0);

    printf("Save file to yaml path? (Y/Other)\n");
    char ch = getchar();
    if(ch == 'Y' || ch == 'y'){
        cv::imwrite(yamlPath + cv::String("distortion_gridpattern_compare1.png"), outputImage1);
        cv::imwrite(yamlPath + cv::String("distortion_gridpattern_compare2.png"), outputImage2);
    }

    return 0;
}

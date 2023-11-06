#include <ros/ros.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <iostream>
#include <fstream>
#include <ctime>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"        

#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "camera_caliberation_chessboard.h"
#include "param_code.h"


using namespace std;


CamCalChessboard::CamCalChessboard(cv::Size chessboardSize, double squareSize)
{
    this->chessboardSize = chessboardSize;
    this->squareSize = squareSize;
}
CamCalChessboard::CamCalChessboard(){

}

bool CamCalChessboard::get_images_from_path(string path, string image_format)
{
    std::cout << "Path: " << path <<std::endl;
    cv::String searchPath = path + string("*.") + image_format;
    
    cv::glob(searchPath, this->imagePaths);

    if(this->imagePaths.size()){
        this->imgSize.width = cv::imread(imagePaths[0]).cols;
        this->imgSize.height = cv::imread(imagePaths[0]).rows;
        printf("Image width: %d, height: %d\n", this->imgSize.width, this->imgSize.height);
        return SUCCESS_PROCESS;
    }
    else
        return FAILD_PROCESS;
}

int CamCalChessboard::get_images_num()
{
    return this->imagePaths.size();
}

void CamCalChessboard::show_src_image(int index)
{
    cv::Mat img = cv::imread(this->imagePaths[index]);
    cv::imshow("imgShow", img);
    cv::waitKey(0);
}

cv::Mat CamCalChessboard::get_src_image(int index, int flags)
{
    return cv::imread(this->imagePaths[index], flags);
} 

vector<cv::Point2f> CamCalChessboard::find_image_chessboard_corners(cv::Mat* srcImage, bool cornerShow, int criteriaIterTimes, double iterDifference)
{
    cv::Mat grayImage;
    cv::cvtColor(*srcImage, grayImage, cv::COLOR_BGR2GRAY);
    vector<cv::Point2f> imagePoints;

    bool patternWasFound = cv::findChessboardCorners(grayImage, this->chessboardSize, imagePoints);
    if(patternWasFound)
    {
        cv::cornerSubPix(grayImage, imagePoints, cv::Size(11, 11), cv::Size(-1, -1),
                        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, criteriaIterTimes, iterDifference));
    }
    else
    {
        printf("Faild to find corners");
        imagePoints.clear();
        return imagePoints;
    }

    if(cornerShow)
    {
        cv::drawChessboardCorners(*srcImage, this->chessboardSize, imagePoints, patternWasFound);
        cv::imshow("cornerShow", *srcImage);
        cv::waitKey(0);
    }

    return imagePoints;     
}

vector<cv::Point3f> CamCalChessboard::find_object_chessboard_corners()
{
    vector<cv::Point3f> objectPoints;
    for (int i = 0; i < this->chessboardSize.height; i++)
        for (int j = 0; j < this->chessboardSize.width; j++)
            objectPoints.emplace_back(j * this->squareSize, i * this->squareSize, 0);

    return objectPoints;
}


bool CamCalChessboard::caliberation_process(bool cornerShow, int criteriaIterTimes, double iterDifference)
{   
    cv::Mat grayImage;
    vector<cv::Point2f> imagePoints;
    vector<cv::Point3f> objectPoints;

    printf("Size of imagePaths: %ld\n", this->imagePaths.size());
    for(int i = 0; i < this->imagePaths.size(); i++)
    {
        grayImage = cv::imread(this->imagePaths[i]);

        printf("Proccess image %d: ", i + 1);

        imagePoints = this->find_image_chessboard_corners(&grayImage, cornerShow, criteriaIterTimes, iterDifference);
        if(imagePoints.size())
        {
            objectPoints = this->find_object_chessboard_corners();

            this->imagePoints.emplace_back(imagePoints);
            this->objectPoints.emplace_back(objectPoints);
            printf("Success!\n");

        }
        else
        {
            printf("Faild!\n");
            continue;
        }
        
    }

    printf("Camera Caliberating...");

    cv::calibrateCamera(this->objectPoints, 
                        this->imagePoints, 
                        this->imgSize, 

                        this->cameraMatrix, 
                        this->disCoffes, 
                        this->rvecs, 
                        this->tvecs);

    printf("Finished!\n");

    return true;
}

bool CamCalChessboard::save_caliberation_parm_yaml(string savePath)
{

    printf("Intrinsics & Extrinsics Parameters Saving...");
    
    savePath = savePath + "caliberation_param.yaml";

    cv::FileStorage fs(savePath ,cv::FileStorage::WRITE); 

    time_t now = time(nullptr);
    fs << "caliberationTimeChar" << ctime(&now);
    fs << "caliberationTime_t" << int(now);
    fs << "imageWidth" << this->imgSize.width;
    fs << "imageHeight" << this->imgSize.height;
    fs << "cameraMatrix" << this->cameraMatrix;
    fs << "disCoffes" << this->disCoffes;
    fs << "rvecs" << this->rvecs;
    fs << "tvecs" << this->tvecs;

    fs.release(); 

    printf("Finished!\n");
     
    return true;
}
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

#include "camera_caliberation_chessboard.h"
#include "param_code.h"


using namespace std;


CamCalChessboard::CamCalChessboard(cv::Size chessboardSize, double squareSize)
{
    this->chessboardSize = chessboardSize;
    this->squareSize = squareSize;
}

bool CamCalChessboard::get_images_from_path(string path)
{
    cv::String searchPath = path + "*.png";
    cv::glob(searchPath, this->imagePaths);

    if(!this->imagePaths.size())
        return SUCCESS_PROCESS;
    else
        return FAILD_PROCESS;
}

vector<cv::Point2f> CamCalChessboard::find_image_chessboard_corners(cv::Mat* srcImage, bool cornerShow, int criteriaIterTimes, double iterDifference)
{
    
    vector<cv::Point2f> imagePoints;

    bool patternWasFound = cv::findChessboardCorners(*srcImage, this->chessboardSize, imagePoints);
    if(patternWasFound)
    {
        cv::cornerSubPix(*srcImage, imagePoints, cv::Size(11, 11), cv::Size(-1, -1),
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

    for(int i = 0; i < this->imagePaths.size(); i++)
    {
        grayImage = cv::imread(this->imagePaths[i], cv::IMREAD_GRAYSCALE);

        printf("Proccess image %d: ", i + 1);

        imagePoints = this->find_image_chessboard_corners(&grayImage, cornerShow, criteriaIterTimes, iterDifference);
        if(imagePoints.size())
        {
            printf("Success!\n");

            objectPoints = this->find_object_chessboard_corners();

            this->imagePoints.emplace_back(imagePoints);
            this->objectPoints.emplace_back(objectPoints);

        }
        else
        {
            printf("Faild!\n");
            continue;
        }
        
    }

    cv::calibrateCamera(this->objectPoints, 
                        this->imagePoints, 
                        this->imgSize, 

                        this->cameraMatrix, 
                        this->disCoffes, 
                        this->rvecs, 
                        this->tvecs);

    return true;
}

bool CamCalChessboard::save_caliberation_parm_yaml(string savePath)
{
    
    savePath = savePath + "caliberation_parm.yaml";

    cv::FileStorage fs(savePath ,cv::FileStorage::WRITE); 

    fs << "cameraMatrix" << this->cameraMatrix;
    fs << "disCoffes" << this->disCoffes;
    fs << "rvecs" << this->rvecs;
    fs << "tvecs" << this->tvecs;

    fs.release(); 
     
    return true;
}
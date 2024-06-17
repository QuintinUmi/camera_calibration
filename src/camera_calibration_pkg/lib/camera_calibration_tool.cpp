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

#include "camera_calibration_tool.h"
#include "param_code.h"


using namespace std;
using namespace cct;

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
    this->searchPath = searchPath;
    
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
            objectPoints.emplace_back(j * this->squareSize, i * this->squareSize, 0.0);

    return objectPoints;
}


bool CamCalChessboard::calibration_process(bool cornerShow, int criteriaIterTimes, double iterDifference)
{   
    cv::Mat grayImage;
    vector<cv::Point2f> imagePoints;
    vector<cv::Point3f> objectPoints;
    cv::Mat cache1, cache2;

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
    fflush(stdout); 

    cv::calibrateCamera(this->objectPoints, 
                        this->imagePoints, 
                        this->imgSize, 

                        this->cameraMatrix, 
                        this->disCoffes,
                        cache1, cache2);

    printf("Finished!\n");

    printf("Camera Undistortion alpha = 1 Caliberating...\n");
    fflush(stdout); 

    cv::Mat srcImage, undistortedImage;
    cv::Mat newCamMat, map1, map2;

    newCamMat = cv::getOptimalNewCameraMatrix(this->cameraMatrix, this->disCoffes, this->imgSize, 1.0);
    cv::initUndistortRectifyMap(this->cameraMatrix, this->disCoffes, cv::Mat(), newCamMat, this->imgSize, CV_32FC2, map1, map2);

    for(int i = 0; i < this->imagePaths.size(); i++)
    {
        srcImage = cv::imread(this->imagePaths[i]);

        printf("Undistort and process image %d: ", i + 1);

        cv::remap(srcImage, undistortedImage, map1, map2, cv::INTER_LINEAR);

        imagePoints = this->find_image_chessboard_corners(&undistortedImage, cornerShow, criteriaIterTimes, iterDifference);
        if(imagePoints.size())
        {
            objectPoints = this->find_object_chessboard_corners();

            this->newImagePoints.emplace_back(imagePoints);
            this->newObjectPoints.emplace_back(objectPoints);
            printf("Success!\n");

        }
        else
        {
            printf("Faild!\n");
            continue;
        }
        
    }


    printf("Processing...");
    fflush(stdout); 
    cv::calibrateCamera(this->newObjectPoints, 
                        this->newImagePoints, 
                        this->imgSize, 

                        this->newCameraMatrixAlpha1, 
                        this->newDisCoffesAlpha1,
                        cache1, cache1);

    printf("Finished!\n");


    this->newImagePoints.clear();
    this->newObjectPoints.clear();
    printf("Camera Undistortion alpha = 0 Caliberating...\n");
    fflush(stdout); 

    srcImage = cv::Mat(), undistortedImage = cv::Mat();
    newCamMat = cv::Mat(), map1 = cv::Mat(), map2 = cv::Mat();

    newCamMat = cv::getOptimalNewCameraMatrix(this->cameraMatrix, this->disCoffes, this->imgSize, 0.0);
    cv::initUndistortRectifyMap(this->cameraMatrix, this->disCoffes, cv::Mat(), newCamMat, this->imgSize, CV_32FC2, map1, map2);

    for(int i = 0; i < this->imagePaths.size(); i++)
    {
        srcImage = cv::imread(this->imagePaths[i]);

        printf("Undistort and process image %d: ", i + 1);

        cv::remap(srcImage, undistortedImage, map1, map2, cv::INTER_LINEAR);

        imagePoints = this->find_image_chessboard_corners(&undistortedImage, cornerShow, criteriaIterTimes, iterDifference);
        if(imagePoints.size())
        {
            objectPoints = this->find_object_chessboard_corners();

            this->newImagePoints.emplace_back(imagePoints);
            this->newObjectPoints.emplace_back(objectPoints);
            printf("Success!\n");

        }
        else
        {
            printf("Faild!\n");
            continue;
        }
        
    }

     printf("Processing...");
    fflush(stdout); 
    cv::calibrateCamera(this->newObjectPoints, 
                        this->newImagePoints, 
                        this->imgSize, 

                        this->newCameraMatrixAlpha0, 
                        this->newDisCoffesAlpha0,
                        cache1, cache1);

    printf("Finished!\n");



    return true;
}

bool CamCalChessboard::save_calibration_parm_yaml(string savePath)
{

    printf("Intrinsics & Extrinsics Parameters Saving...");
    fflush(stdout); 
    
    savePath = savePath + "calibration_param.yaml";

    cv::FileStorage fs(savePath ,cv::FileStorage::WRITE); 

    time_t now = time(nullptr);
    fs << "calibrationTimeChar" << ctime(&now);
    fs << "calibrationTime_t" << int(now);
    fs << "imageWidth" << this->imgSize.width;
    fs << "imageHeight" << this->imgSize.height;
    fs << "cameraMatrix" << this->cameraMatrix;
    fs << "disCoffes" << this->disCoffes;
    // fs << "rvecs" << this->rvecs;
    // fs << "tvecs" << this->tvecs;

    fs << "newCameraMatrixAlpha1" << this->newCameraMatrixAlpha1;
    fs << "newDisCoffesAlpha1" << this->newDisCoffesAlpha1;
    fs << "newCameraMatrixAlpha0" << this->newCameraMatrixAlpha0;
    fs << "newDisCoffesAlpha0" << this->newDisCoffesAlpha0;

    fs.release(); 

    printf("Finished!\n");
     
    return true;
}






/************************************************************************************/


CamCalExt::CamCalExt(){

}
CamCalExt::CamCalExt(CamCalChessboard camCalCB){

}
CamCalExt::CamCalExt(cv::Mat cameraMatrix, cv::Mat disCoffes){
    this->set_intrinsics(cameraMatrix, disCoffes);
}

bool CamCalExt::set_points_n_points(vector<cv::Point3f> worldPoints, vector<cv::Point2f> imagePoints)
{
    this->worldPoints = worldPoints;
    this->imagePoints = imagePoints;
    return true;
}

bool CamCalExt::set_intrinsics(cv::Mat cameraMatrix, cv::Mat disCoffes)
{
    this->cameraMatrix = cameraMatrix;
    this->disCoffes = disCoffes;
    return true;
}
bool CamCalExt::set_extrinsics(cv::Mat rvec, cv::Mat tvec)
{
    this->rvec = rvec;
    this->tvec = tvec;
    return true;
}

vector<cv::Mat> CamCalExt::ext_cal_one_frame()
{
    cv::Mat rvec, tvec;
    cv::solvePnP(this->worldPoints, this->imagePoints, this->cameraMatrix, this->disCoffes, rvec, tvec);
    vector<cv::Mat> extrinsics_matrix;
    extrinsics_matrix.emplace_back(rvec);
    extrinsics_matrix.emplace_back(tvec);
    return extrinsics_matrix;
}

void CamCalExt::mapping_3d_to_2d_one_frame(vector<cv::Point3f> &worldPoints, vector<cv::Point2f> &imagePoints, cv::Mat rvec, cv::Mat tvec, 
                                            cv::Mat cameraMatrix, cv::Mat disCoffes)
{
    if(rvec.empty()){
        rvec = this->rvec;
    }
    if(tvec.empty()){
        tvec = this->tvec;
    }
    if(cameraMatrix.empty()){
        cameraMatrix = this->cameraMatrix;
    }
    if(disCoffes.empty()){
        disCoffes = this->disCoffes;
    }
    
    cv::projectPoints(worldPoints, rvec, tvec, cameraMatrix, cv::Mat(), imagePoints);
    // CamCalExt::mapping_points_3d_to_2d(worldPoints, imagePoints, rvec, tvec, cameraMatrix);
    
    
}




// void CamCalExt::mapping_points_3d_to_2d(vector<cv::Point3f> &worldPoints, vector<cv::Point2f> &pixelPoints, cv::Mat rvec, cv::Mat tvec, 
//                                             cv::Mat cameraMatrix, cv::Mat disCoffes)
// {   
//     double thetaX = rvec.at<double>(0), thetaY = rvec.at<double>(1), thetaZ = rvec.at<double>(2);
    
//     cv::Mat rX = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0,
//                                             0.0, cos(thetaX), -sin(thetaX),
//                                             0.0, sin(thetaX), cos(thetaX));
//     cv::Mat rY = (cv::Mat_<double>(3, 3) << cos(thetaY), 0.0, sin(thetaY),
//                                             0.0, 1.0, 0.0,
//                                             -sin(thetaY), 0.0, cos(thetaY));     
//     cv::Mat rZ = (cv::Mat_<double>(3, 3) << cos(thetaZ), -sin(thetaZ), 0.0,
//                                             sin(thetaZ), cos(thetaZ), 0.0,
//                                             0.0, 0.0, 1.0);    

//     cv::Mat testMatrix = (cv::Mat_<double>(4, 4) << 1.0, 0.0, 0.0, 0.0, 
//                                                     0.0, 1.0, 0.0, 0.0,
//                                                     0.0, 0.0, -1.0, 0.0,
//                                                     0.0, 0.0, 0.0, 1.0);
    
//     cv::Mat rM = rZ 
// remote: Resolving deltas: 100% (7/7), compnts = (cv::Mat_<double>(4, 1)); 
//     cv::Mat imagePoints = (cv::Mat_<double>(3, 1));
//     cv::Mat _pixelPoints = (cv::Mat_<double>(3, 1));

//     // printf("------------------------------------------------------------------------\n");
//     for(int p = 0; p < worldPoints.size(); p++){

//         _worldPoints = (cv::Mat_<double>(4, 1) << worldPoints[p].x, worldPoints[p].y, worldPoints[p].z, 1.0);
//         _imagePoints = wvec * _worldPoints;
//         imagePoints = (cv::Mat_<double>(3, 1) << _imagePoints.at<double>(0), _imagePoints.at<double>(1), _imagePoints.at<double>(2));
//         _pixelPoints = cameraMatrix * (imagePoints / imagePoints.at<double>(2));

//         pixelPoints.emplace_back(_pixelPoints.at<double>(0), _pixelPoints.at<double>(1));
//     }

// }


vector<cv::String> cct::get_images_from_path(cv::String path, cv::String image_format)
{
    std::cout << "Path: " << path <<std::endl;
    cv::String searchPath = path + string("*.") + image_format;
    vector<cv::String> imagePaths;
    cv::glob(searchPath, imagePaths);

    if(imagePaths.size()){
        printf("Find %ld images in path: %s\n", imagePaths.size(), searchPath.c_str());
        return imagePaths;
    }
    else{
        printf("Failed to load images!\n");
        return vector<cv::String>{};
    }
}

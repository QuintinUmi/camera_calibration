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
#include "apriltag/apriltag.h"    
#include "opencv2/aruco/charuco.hpp"  
  


#include <yaml-cpp/yaml.h>

#include "image_transport/image_transport.h"

#include "drawing_tool.h"
#include "param_code.h"


using namespace std;
using namespace drt;


ArucoM::ArucoM()
{
    this->dParameters = cv::aruco::DetectorParameters::create();
}
ArucoM::ArucoM(int dictionaryName, vector<int> selectedIds, vector<float> markerRealLength)
{
    this->markerDictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    this->selectedIds = selectedIds;
    this->dParameters = cv::aruco::DetectorParameters::create();
    this->markerRealLength = markerRealLength;
}
ArucoM::ArucoM(cv::Ptr<cv::aruco::Dictionary> markerDictionary, vector<int> selectedIds, vector<float> markerRealLength)
{
    this->markerDictionary = markerDictionary;
    this->selectedIds = selectedIds;
    this->dParameters = cv::aruco::DetectorParameters::create();
    this->markerRealLength = markerRealLength;
}

void ArucoM::set_aruco(int dictionaryName, vector<int> selectedIds, vector<float> markerRealLength)
{
    this->markerDictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    this->selectedIds = selectedIds;
    this->dParameters = cv::aruco::DetectorParameters::create();
    this->markerRealLength = markerRealLength;
}
void ArucoM::set_aruco(cv::Ptr<cv::aruco::Dictionary> markerDictionary, vector<int> selectedIds, vector<float> markerRealLength)
{
    this->markerDictionary = markerDictionary;
    this->selectedIds = selectedIds;
    this->dParameters = cv::aruco::DetectorParameters::create();
    this->markerRealLength = markerRealLength;
}
void ArucoM::sel_aruco_ids(vector<int> selectedIds)
{
    this->selectedIds = selectedIds;
}
void ArucoM::set_aruco_real_length(vector<float> markerRealLength)
{
    this->markerRealLength = markerRealLength;
}
void ArucoM::set_camera_intrinsics(cv::Mat cameraMatrix, cv::Mat disCoffes)
{
    this->cameraMatrix = cameraMatrix;
    this->disCoffes = disCoffes;
}


vector<cv::Mat> ArucoM::generate_aruco_marker(int markerSize)
{
    vector<cv::Mat> imgArucoMarker;
    for(int i = 0; i < this->selectedIds.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(this->markerDictionary, this->selectedIds[i], markerSize, temp);
        imgArucoMarker.emplace_back(temp);
    }
    
    return imgArucoMarker;
}
vector<cv::Mat> ArucoM::generate_aruco_marker(int dictionaryName, vector<int> selectedIds, int markerSize)
{
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    vector<cv::Mat> imgArucoMarker;
    for(int i = 0; i < selectedIds.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(markerDictionary, selectedIds[i], markerSize, temp);
        imgArucoMarker.emplace_back(temp);
    }
    
    return imgArucoMarker;
}
void ArucoM::generate_aruco_inner(int markerSize)
{
    for(int i = 0; i < this->selectedIds.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(this->markerDictionary, this->selectedIds[i], markerSize, temp);
        this->markerImage.emplace_back(temp);
    }
}
void ArucoM::generate_aruco_inner(int dictionaryName, vector<int> selectedIds, int markerSize)
{
    this->markerDictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    for(int i = 0; i < this->selectedIds.size(); i++)
    { 
        cv::Mat temp;
        cv::aruco::drawMarker(this->markerDictionary, this->selectedIds[i], markerSize, temp);
        this->markerImage.emplace_back(temp);
    }
}


void ArucoM::detect_aruco(cv::Mat &inputImage, cv::OutputArrayOfArrays markerCorners, cv::OutputArray markerIds)
{
    cv::aruco::detectMarkers(inputImage, this->markerDictionary, markerCorners, markerIds, this->dParameters);
}

void ArucoM::ext_calib_single_aruco(cv::Mat &inputImage, int targetId, 
                                    cv::Mat rvecs, cv::Mat tvecs)
{
    vector<cv::Point2f> markerCorners;
    vector<int> markerIds;
    int indexId;

    cv::aruco::detectMarkers(inputImage, this->markerDictionary, markerCorners, markerIds, this->dParameters);
    for(indexId = 0; indexId < markerIds.size() && markerIds[indexId] != targetId; indexId++);
    if(indexId != markerIds.size())
    {
        cv::aruco::estimatePoseSingleMarkers(markerCorners, this->markerRealLength[indexId], this->cameraMatrix, this->disCoffes, rvecs, tvecs);
    }
    else
    {
        rvecs = cv::Mat();
        tvecs = cv::Mat();
    }
    
}


void ArucoM::aruco_marker_save(cv::String imageSavePath, cv::String imageFormat, vector<cv::Mat> arucoMarkerImages, 
                                int dictionaryName, bool showImage)
{
    cv::Mat processImg(arucoMarkerImages[0].size(), CV_8U, cv::Scalar(255));
    cv::Ptr<cv::aruco::Dictionary> markerDictionary = cv::aruco::getPredefinedDictionary(dictionaryName);
    cv::String saveFileName;
    vector<vector<cv::Point2f>> markerCorners;
    vector<int> markerIds;
    Draw3D d3d;
    vector<int> markerIdss;
    markerIdss.emplace_back(5);
    // arucoMarkerImages = this->generate_aruco_marker(dictionaryName, markerIdss, 500);
    for(int i = 0; i < arucoMarkerImages.size(); i++)
    { 
        processImg = cv::Mat(arucoMarkerImages[i].size(), CV_8U, cv::Scalar(255));
        
        d3d.center_image_scale(arucoMarkerImages[i], processImg, 0.5, 0.5, 1, 0, cv::Scalar(255));
        cv::aruco::detectMarkers(processImg, markerDictionary, markerCorners, markerIds);
        saveFileName = imageSavePath + cv::String("id_") + std::to_string(markerIds[0]) + 
                                        cv::String("--dictName_") + std::to_string(dictionaryName) + 
                                        cv::String(".") + imageFormat;
        // saveFileName = to_string(i) + cv::String(".png");
        cv::imwrite(saveFileName, arucoMarkerImages[i]);
        if(showImage){
            cv::imshow(cv::String("id_") + std::to_string(markerIds[0]) + 
                        cv::String("--dictName_") + std::to_string(dictionaryName) + 
                        cv::String(".") + imageFormat, 
                        arucoMarkerImages[i]);
            cv::waitKey(0);
        }
    }
}
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

#include "drawing_tool.h"
#include "param_code.h"


#define PI 3.14159265358979324

using namespace std;



// Draw3D::Draw3D(){
//     this->scaleX = 1.0;
//     this->scaleY = 1.0;
//     this->scaleZ = 1.0;
// }
Draw3D::Draw3D(float unitLength ,float scaleX, float scaleY, float scaleZ)
{
    this->unitLength = unitLength;
    this->scaleX = scaleX;
    this->scaleY = scaleY;
    this->scaleZ = scaleZ;
}
void Draw3D::set_unitlen(float unitLength){
    this->unitLength;
}
void Draw3D::set_scale(float scaleX, float scaleY, float scaleZ)
{
    this->scaleX = scaleX;
    this->scaleY = scaleY;
    this->scaleZ = scaleZ;
}


void Draw3D::write_in(cv::Point3f &dst, float x, float y, float z)
{
    dst.x = x * this->scaleX;
    dst.y = y * this->scaleY;
    dst.z = z * this->scaleZ;
}
void Draw3D::write_in(vector<cv::Point3f> &dst, float x, float y, float z)
{
    dst.emplace_back(x * this->scaleX, y * this->scaleY, z * this->scaleZ);
}


vector<vector<cv::Point3f>> Draw3D::draw_ortho_coordinate_3d(float cx, float cy, float cz, float density)
{
    vector<vector<cv::Point3f>> coordinate(4);
    this->write_in(coordinate[0], cx, cy, cz);
    int pointNum;
    pointNum = this->unitLength / density;
    for(int i = 1; i <= pointNum; i++)
        this->write_in(coordinate[1], cx + i * density, cy, cz);
    for(int i = 1; i <= pointNum; i++)
        this->write_in(coordinate[2], cx, cy + i * density, cz);
    for(int i = 1; i <= pointNum; i++)
        this->write_in(coordinate[3], cx, cy, cz + i * density);

    return coordinate;
}
void Draw3D::draw_ortho_coordinate_2d(cv::Mat &imgInputOutput, cv::Mat cameraMatrix, cv::Mat disCoffes, cv::Mat rvec, cv::Mat tvec,
                                        float cx, float cy, float cz)
{
    vector<cv::Point3f> p3d;
    this->write_in(p3d, cx, cy, cz);
    this->write_in(p3d, cx + this->unitLength, cy, cz);
    this->write_in(p3d, cx, cy + this->unitLength, cz);
    this->write_in(p3d, cx, cy, cz + this->unitLength);

    vector<cv::Point2f> p2d;
    cv::projectPoints(p3d, rvec, tvec, cameraMatrix, disCoffes, p2d);
    
    cv::line(imgInputOutput, p2d[0], p2d[1], cv::Scalar(0, 0, 255), 3);
    cv::line(imgInputOutput, p2d[0], p2d[2], cv::Scalar(0, 255, 0), 3);
    cv::line(imgInputOutput, p2d[0], p2d[3], cv::Scalar(255, 0, 0), 3);

}


void Draw3D::transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Mat rvec, cv::Mat tvec)
{
    if(rvec.empty()){
        rvec = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    }
    if(tvec.empty()){
        tvec = (cv::Mat_<float>(3, 1) << 0, 0, 0);
    }
    cv::Mat R;
    cv::Mat srcMatrix, dstMatrix;
    cv::Rodrigues(rvec, R);
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = R * srcMatrix + tvec;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
    
}
void Draw3D::transform_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, 
                                        float rx, float ry, float rz, float tx, float ty, float tz)
{
    cv::Mat rvec = (cv::Mat_<float>(3, 1) << rx, ry, rz), tvec = (cv::Mat_<float>(3, 1) << tx, ty, tz);
    cv::Mat R;
    cv::Mat srcMatrix, dstMatrix;
    cv::Rodrigues(rvec, R);
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = R * srcMatrix + tvec;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
}


void Draw3D::mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, cv::Point3f surfaceNorVec)
{
    double len = pow(surfaceNorVec.x, 2) + pow(surfaceNorVec.y, 2) + pow(surfaceNorVec.z, 2);
    float uX = surfaceNorVec.x / len;
    float uY = surfaceNorVec.y / len;
    float uZ = surfaceNorVec.z / len;

    cv::Mat q = (cv::Mat_<float>(3, 3) <<   1-2*pow(uX, 2), -2*uX*uY, -2*uX*uZ,
                                            -2*uX*uY, 1-2*pow(uY, 2), -2*uY*uZ,
                                            -2*uX*uZ, -2*uY*uZ, 1-2*pow(uZ, 2));

    cv::Mat srcMatrix, dstMatrix;
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = q * srcMatrix;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
            
    }
}
void Draw3D::mirror_3d_points(vector<cv::Point3f> &srcWorldPoints, vector<cv::Point3f> &newWorldPoints, float surNorX, float surNorY, float surNorZ)
{
    cv::Mat surfaceNorVec;
    double len = pow(surNorX, 2) + pow(surNorY, 2) + pow(surNorZ, 2);
    float uX = surNorX / len;
    float uY = surNorY / len;
    float uZ = surNorZ / len;

    cv::Mat q = (cv::Mat_<float>(3, 3) <<   1-2*pow(uX, 2), -2*uX*uY, -2*uX*uZ,
                                            -2*uX*uY, 1-2*pow(uY, 2), -2*uY*uZ,
                                            -2*uX*uZ, -2*uY*uZ, 1-2*pow(uZ, 2));

    cv::Mat srcMatrix, dstMatrix;
    size_t t = srcWorldPoints.size();
    for(int i = 0; i < t; i++)
    {
        srcMatrix = cv::Mat(srcWorldPoints[i]);
        dstMatrix = q * srcMatrix;
        if(srcWorldPoints == newWorldPoints){
            newWorldPoints[i] = cv::Point3f(dstMatrix);
        }
        else{
            newWorldPoints.emplace_back(dstMatrix);
        }
    }
}

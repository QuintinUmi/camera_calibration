#include <ros/ros.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
// #include "apriltag/apriltag.h" 
// #include "apriltag/tagStandard41h12.h"


#include "camera_calibration_tool.h"
#include "drawing_tool.h"
#include "aruco_tool.h"
#include "undistortion.h"


#define PI 3.14159265358979324

using namespace cct;
using namespace Eigen;
using namespace drt;

vector<cv::Point3f> draw_cylindar_x(float cy, float cz, float start_x, float end_x, float radium, float precision = 0.1){

    vector<cv::Point3f> points3D;
    float x = 0.0, y, z, d_theta = 2 * PI / (float)int(2 * PI * radium / precision);

    points3D.emplace_back(0.0, cy, cz);
    for(int i = 0; i < int((end_x - start_x) / precision); i++){
        x = start_x + i * precision;
        for(int j = 0; j < int(2 * PI * radium / precision); j++){
            y = cy + radium * cos(j * d_theta);
            z = cz + radium * sin(j * d_theta);
            // std::cout << cv::Point3f(x, y, z) << std::endl;
            points3D.emplace_back(x, y, z);
        }
    }
    printf("Finish Drawing Cylindar!\n");
    return points3D;
}
vector<cv::Point3f> draw_cylindar_y(float cx, float cz, float start_y, float end_y, float radium, float precision = 0.1){

    vector<cv::Point3f> points3D;
    float x, y= 0.0, z, d_theta = 2 * PI / (float)int(2 * PI * radium / precision);

    points3D.emplace_back(cx, 0.0, cz);
    for(int i = 0; i < int((end_y - start_y) / precision); i++){
        y = start_y + i * precision;
        for(int j = 0; j < int(2 * PI * radium / precision); j++){
            x = cx + radium * cos(j * d_theta);
            z = cz + radium * sin(j * d_theta);
            // std::cout << cv::Point3f(x, y, z) << std::endl;
            points3D.emplace_back(x, y, z);
        }
    }
    printf("Finish Drawing Cylindar!\n");
    return points3D;
}
vector<cv::Point3f> draw_cylindar_z(float cx, float cy, float start_z, float end_z, float radium, float precision = 0.1){

    vector<cv::Point3f> points3D;
    float x, y, z = 0.0, d_theta = 2 * PI / (float)int(2 * PI * radium / precision);

    points3D.emplace_back(cx, cy, 0.0);
    for(int i = 0; i < int((end_z - start_z) / precision); i++){
        z = start_z + i * precision;
        for(int j = 0; j < int(2 * PI * radium / precision); j++){
            x = cx + radium * cos(j * d_theta);
            y = cy + radium * sin(j * d_theta);
            // std::cout << cv::Point3f(x, y, z) << std::endl;
            points3D.emplace_back(x, y, z);
        }
    }
    printf("Finish Drawing Cylindar!\n");
    return points3D;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Chessboard_Ext_Calib");
    ros::NodeHandle rosHandle;
    
    cv::String yamlPath;
    std::string imageFormat;
    cv::String imagePath;

    cv::Size chessboardSize;
    double squareSize;

    rosHandle.param("yaml_save_path", yamlPath, cv::String("~/"));
    rosHandle.param("image_load_path", imagePath, cv::String("~/"));
    rosHandle.param("image_format", imageFormat, std::string("png"));
    rosHandle.param("chessboard_width", chessboardSize.width, 8);
    rosHandle.param("chessboard_height", chessboardSize.height, 11);
    rosHandle.param("square_size", squareSize, 25.44);

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


    CamCalChessboard camCal(chessboardSize, squareSize);
    camCal.get_images_from_path(imagePath, imageFormat);
    CamCalExt Cce(newCameraMatrix, newDisCoffes);
    Draw3D d3d(28.9, 1, 1, 1);
    cv::Mat testImg = cv::imread("/home/quintinumi/project/camera_calibration_img/1.png"), testImgOp;
    d3d.center_image_scale(testImg, testImg, 1, -1);
    // CamCalExt Cce(cameraMatrix, disCoffes);
    // vector<vector<cv::Point3f>> wp = d3d.draw_ortho_coordinate_3d();
    // vector<cv::Point3f> worldPoints1 = wp[1];
    // vector<cv::Point3f> worldPoints2 = wp[2];
    // vector<cv::Point3f> worldPoints3 = wp[3];
    // d3d.transform_3d_coordinate(w3, worldPoints3, 0, PI, 0, 0, 0, 0);
    // d3d.mirror_3d_points(worldPoints3, worldPoints3, 0, 0, 1);
    vector<int> id;
    id.emplace_back(5);
    id.emplace_back(50);
    ArucoM test(cv::aruco::DICT_6X6_100, id);
    test.generate_aruco_marker(500);

    for(int index = 0; index < camCal.get_images_num(); index++)
    {
        cv::Mat srcImage, undistortedImage, tarImage;
        srcImage = camCal.get_src_image(index);

        Undistortion Und(cameraMatrix, disCoffes, image_size);
        Und.undistortion_process(srcImage, undistortedImage);

        
        
        vector<cv::Point2f> imagePoints;

        vector<cv::Point3f> objP = camCal.find_object_chessboard_corners();
        vector<cv::Point2f> corP = camCal.find_image_chessboard_corners(&undistortedImage);
        // vector<cv::Point2f> corP = camCal.find_image_chessboard_corners(&srcImage);
        Cce.set_points_n_points(objP, corP);
        vector<cv::Mat> ext = Cce.ext_cal_one_frame();

        d3d.draw_ortho_coordinate_2d(undistortedImage, newCameraMatrix, cv::Mat(), ext[0], ext[1]);


        // imagePoints.clear();
        // Cce.mapping_3d_to_2d_one_frame(worldPoints1, imagePoints, ext[0], ext[1], newCameraMatrix, newDisCoffes);
        // for(int i = 0; i < imagePoints.size(); i++)
        // {
        //     cv::circle(undistortedImage, imagePoints[i], 2, cv::Scalar(0, 0, 255), -1);
        //     // cv::circle(srcImage, imagePoints[i], 2, cv::Scalar(0, 255, 120), -1);
        // }
        // imagePoints.clear();
        // Cce.mapping_3d_to_2d_one_frame(worldPoints2, imagePoints, ext[0], ext[1], newCameraMatrix, newDisCoffes);      
        // for(int i = 0; i < imagePoints.size(); i++)
        // {
        //     cv::circle(undistortedImage, imagePoints[i], 2, cv::Scalar(0, 255, 0), -1);
        //     // cv::circle(srcImage, imagePoints[i], 2, cv::Scalar(0, 255, 120), -1);
        // }
        // imagePoints.clear();
        // Cce.mapping_3d_to_2d_one_frame(worldPoints3, imagePoints, ext[0], ext[1], newCameraMatrix, newDisCoffes);
        // for(int i = 0; i < imagePoints.size(); i++)
        // {
        //     cv::circle(undistortedImage, imagePoints[i], 2, cv::Scalar(255, 0, 0), -1);
        //     // cv::circle(srcImage, imagePoints[i], 2, cv::Scalar(0, 255, 120), -1);
        // }
        
        // Cce.mapping_3d_to_2d_one_frame(worldPoints, imagePoints, ext[0], ext[1], cameraMatrix, disCoffes);

        cv::drawChessboardCorners(undistortedImage, chessboardSize, corP, true);

        // d3d.paste_image_perspective_3d(testImg, undistortedImage, true,
        //                                 newCameraMatrix, cv::Mat(), ext[0], ext[1], 
        //                                 cv::Point3f(28.9, 28.9, 28.9), cv::Size(28.9 * 5, 28.9 * 5), 
        //                                 (cv::Mat_<float>(3, 1) << 0, 0, -PI), (cv::Mat_<float>(3, 1) << 28.9 * 3, 28.9 * 3, 0));
        d3d.paste_image_perspective_3d(testImg, undistortedImage, true, false,
                                        newCameraMatrix, cv::Mat(), ext[0], ext[1], 
                                        cv::Point3f(-28.9, -28.9, 0), cv::Size(28.9 * 10, 28.9 * 10), 
                                        (cv::Mat_<float>(3, 1) << 0, 0, 0), (cv::Mat_<float>(3, 1) << 0, 0, 0));

        
        
        cv::imshow("Draw 3D Object on Image", undistortedImage);
        // cv::imshow("Draw 3D Object on Image", srcImage);
        cv::waitKey(0);

    }

    
    

    return 0;
}

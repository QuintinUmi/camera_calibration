#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include "camera_caliberation_tool.h"
#include "undistortion.h"


#define PI 3.1415926535

using namespace cct;


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
    ros::init(argc, argv, "undistortion");
    ros::NodeHandle rosHandle;
    
    cv::String yamlPath;
    std::string imageFormat;
    cv::String imagePath;

    cv::Size chessboardSize;
    double squareSize;

    rosHandle.param("yaml_save_path", yamlPath, cv::String("~/"));
    rosHandle.param("image_load_path", imagePath, cv::String("~/"));
    rosHandle.param("image_format", imageFormat, std::string("png"));
    rosHandle.param("chessboard_width", chessboardSize.width, 6);
    rosHandle.param("chessboard_height", chessboardSize.height, 8);
    rosHandle.param("square_size", squareSize, 28.9);

    cv::String intrinsicsPath = yamlPath + "caliberation_param.yaml";
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
    vector<cv::Point3f> worldPoints1 = draw_cylindar_z(28.9, 28.9, 0.2, 4.8, 15.0, 0.1);
    vector<cv::Point3f> worldPoints2 = draw_cylindar_z(28.9, 28.9, 0.0, 0.2, 15.0, 0.1);
    vector<cv::Point3f> worldPoints3 = draw_cylindar_z(28.9, 28.9, 4.8, 5.0, 15.0, 0.1);

    for(int index = 0; index < camCal.get_images_num(); index++)
    {
        cv::Mat srcImage, undistortedImage, tarImage;
        srcImage = camCal.get_src_image(index);

        Undistortion Und(cameraMatrix, disCoffes, image_size);
        Und.undistortion_process(srcImage, undistortedImage);

        CamCalExt Cce(newCameraMatrix, newDisCoffes);
        // CamCalExt Cce(cameraMatrix, disCoffes);
        
        vector<cv::Point2f> imagePoints;

        vector<cv::Point3f> objP = camCal.find_object_chessboard_corners();
        vector<cv::Point2f> corP = camCal.find_image_chessboard_corners(&undistortedImage);
        // vector<cv::Point2f> corP = camCal.find_image_chessboard_corners(&srcImage);
        Cce.set_points_n_points(objP, corP);
        vector<cv::Mat> ext = Cce.ext_cal_one_frame();

        
        // cv::Mat inverseZ_matrix(cv::Size(1, 3), CV_64F, _inverseZ_Matrix);
        float _inverseZ_Matrix[3][3] = {   1, 0, 0, 
                                            0, -1, 0,
                                            0, 0, 1};
        cv::Mat inverseZ_matrix(cv::Size(3, 3), CV_32F, _inverseZ_Matrix);
        

        inverseZ_matrix.cross(inverseZ_matrix);
        // std::cout << inverseZ_matrix << std::endl;
        // std::cout << R << std::endl;
        // std::cout << inverseZ_matrix.size() << std::endl;
        // std::cout << R.size() << std::endl;
        // // std::cout << inverseZ_matrix.cross(R) << std::endl;
        // R.at<double>(1, 0) = -R.at<double>(1, 0);
        // R.at<double>(1, 0) = -R.at<double>(1, 1);
        // R.at<double>(1, 0) = -R.at<double>(1, 2);
        // std::cout << R << std::endl;
        // cv::Rodrigues(R, ext[0]);
        
        imagePoints.clear();
        Cce.mapping_3d_to_2d_one_frame(worldPoints1, imagePoints, ext[0], ext[1], newCameraMatrix, newDisCoffes);
        for(int i = 0; i < imagePoints.size(); i++)
        {
            cv::circle(undistortedImage, imagePoints[i], 2, cv::Scalar(0, 255, 120), -1);
            // cv::circle(srcImage, imagePoints[i], 2, cv::Scalar(0, 255, 120), -1);
        }
        imagePoints.clear();
        Cce.mapping_3d_to_2d_one_frame(worldPoints2, imagePoints, inverseZ_matrix + ext[0], ext[1], newCameraMatrix, newDisCoffes);      
        for(int i = 0; i < imagePoints.size(); i++)
        {
            cv::circle(undistortedImage, imagePoints[i], 2, cv::Scalar(255, 0, 0), -1);
            // cv::circle(srcImage, imagePoints[i], 2, cv::Scalar(0, 255, 120), -1);
        }
        imagePoints.clear();
        Cce.mapping_3d_to_2d_one_frame(worldPoints3, imagePoints, inverseZ_matrix + ext[0], ext[1], newCameraMatrix, newDisCoffes);
        for(int i = 0; i < imagePoints.size(); i++)
        {
            cv::circle(undistortedImage, imagePoints[i], 2, cv::Scalar(0, 0, 255), -1);
            // cv::circle(srcImage, imagePoints[i], 2, cv::Scalar(0, 255, 120), -1);
        }
        
        
        
        // Cce.mapping_3d_to_2d_one_frame(worldPoints, imagePoints, ext[0], ext[1], cameraMatrix, disCoffes);

        cv::drawChessboardCorners(undistortedImage, chessboardSize, corP, true);
        
        
        cv::imshow("Draw 3D Object on Image", undistortedImage);
        // cv::imshow("Draw 3D Object on Image", srcImage);
        cv::waitKey();
    }
    

    return 0;
}




#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "camera_caliberation_tool.h"

using namespace cct;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "undistortion");
    ros::NodeHandle rosHandle;
    
    cv::String yamlPath;
    std::string imageFormat;
    cv::String imagePath;


    rosHandle.param("yaml_save_path", yamlPath, cv::String("~/"));
    rosHandle.param("image_load_path", imagePath, cv::String("~/"));
    rosHandle.param("image_format", imageFormat, std::string("png"));

    yamlPath = yamlPath + "caliberation_param.yaml";
    cv::FileStorage fs(yamlPath, cv::FileStorage::READ);
    int image_width{0}, image_height{0};
    fs["imageWidth"] >> image_width;
    fs["imageHeight"] >> image_height;

    cv::Size image_size = cv::Size(image_width, image_height);

    cv::Mat cameraMatrix, disCoffes;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["disCoffes"] >> disCoffes;
    fs.release();
    std::cout << cameraMatrix << std::endl;
    std::cout << disCoffes << std::endl;
    std::cout << image_size << std::endl;


    CamCalChessboard camCal;
    camCal.get_images_from_path(imagePath, imageFormat);

    cv::Mat srcImg, undistortImg;
    for(int i = 0; i < camCal.get_images_num(); i++){

        srcImg = camCal.get_src_image(i);

        undistort(srcImg, undistortImg, cameraMatrix, disCoffes);

        imshow("srcImg", srcImg);
        imshow("undistortImg", undistortImg);

        cv::waitKey();
    }



    return 0;
}

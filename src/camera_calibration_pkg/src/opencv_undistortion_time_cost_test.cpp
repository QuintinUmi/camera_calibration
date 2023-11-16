#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <time.h>

#include "camera_calibration_tool.h"

using namespace cv;
using namespace std;
using namespace cct;

int main(int argc, char **argv) {

    ros::init(argc, argv, "undistortion");
    ros::NodeHandle rosHandle;
    
    cv::String yamlPath;
    cv::String InstrinsicsFilePath;
    std::string imageFormat;
    cv::String imagePath;


    rosHandle.param("yaml_save_path", yamlPath, cv::String("~/"));
    rosHandle.param("image_load_path", imagePath, cv::String("~/"));
    rosHandle.param("image_format", imageFormat, std::string("png"));

    InstrinsicsFilePath = yamlPath + "calibration_param.yaml";
    cv::FileStorage fs(InstrinsicsFilePath, FileStorage::READ);
    int image_width{0}, image_height{0};
    fs["imageWidth"] >> image_width;
    fs["imageHeight"] >> image_height;

    Size image_size = Size(image_width, image_height);

    Mat cameraMatrix, disCoffes;
    fs["cameraMatrix"] >> cameraMatrix;
    fs["disCoffes"] >> disCoffes;
    fs.release();
    std::cout << cameraMatrix << std::endl;
    std::cout << disCoffes << std::endl;
    std::cout << image_size << std::endl;


    CamCalChessboard camCal;
    camCal.get_images_from_path(imagePath, imageFormat);

    Mat srcImg, undistortImg;

    int j = 0;
    clock_t start, end;

    start = clock();
    for(int i = 0; j < 1000; i++){

        if(i == camCal.get_images_num()) i = 0;
        j++;

        srcImg = camCal.get_src_image(i);

        undistort(srcImg, undistortImg, cameraMatrix, disCoffes);

        printf("Count: %d\n", j);
    }
    end = clock();

    printf("Time comsuming: %.5f\n", double((end - start) / CLOCKS_PER_SEC));

    
    return 0;
}

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {

    // 读取相机矩阵、畸变系数
    cv::FileStorage fs("./calibration_in_params.yml", FileStorage::READ);
    int image_width{0}, image_height{0};
    fs["image_width"] >> image_width;
    fs["image_height"] >> image_height;
    Size image_size = Size(image_width, image_height);

    Mat intrinsic_matrix, distortion_coeffs;
    fs["cameraMatrix"] >> intrinsic_matrix;
    fs["distCoeffs"] >> distortion_coeffs;
    fs.release();
    std::cout << intrinsic_matrix << std::endl;
    std::cout << distortion_coeffs << std::endl;
    std::cout << image_size << std::endl;

    const Mat &image0 = imread("./calib_chess_img/image_0.jpg", IMREAD_COLOR);
    Mat image;

    undistort(image0, image, intrinsic_matrix, distortion_coeffs);

    imshow("original", image0);
    imshow("undistorted", image);

    waitKey();
    return 0;
}
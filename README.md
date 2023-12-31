# Camera_Calibration

This repository is for camera calibration in C++.

[![GitHub](https://img.shields.io/badge/dynamic/json?url=https%3A%2F%2Fapi.swo.moe%2Fstats%2Fgithub%2FQuintinUmi&query=count&color=181717&label=GitHub&labelColor=282c34&logo=github&suffix=+follows&cacheSeconds=3600)](https://github.com/QuintinUmi)


<br />


## Files Structure Description

```
camera_calibration_pkg
    │  CMakeLists.txt
    │  package.xml
    │
    ├─config
    │      calibration_param.yaml
    │      distortion_gridpattern_compare1.png
    │      distortion_gridpattern_compare2.png
    │      setup_aruco.yaml
    │      setup_chessboard.yaml
    │      setup_ros.yaml
    │
    ├─include
    │      camera_calibration_tool.h
    │      drawing_tool.h
    │      param_code.h
    │      undistortion.h
    │
    ├─launch
    │      aruco_ext_calib.launch
    │      aruco_marker_generate.launch
    │      aruco_video_ext_calib.launch
    │      camera_calibration.launch
    │      chessboard_cam_ext_calibration.launch
    │      opencv_undistortion_time_cost_test.launch
    │      show_distorted_grid_pattern.launch
    │      undistortion.launch
    │
    ├─lib
    │      aruco_tool.cpp
    │      camera_calibration_tool.cpp
    │      drawing_tool.cpp
    │      undistortion.cpp
    │
    └─src
            aruco_ext_calib.cpp
            aruco_marker_generate.cpp
            aruco_video_ext_calib.cpp
            camera_calibration.cpp
            chessboard_cam_ext_calibration.cpp
            opencv_undistortion_time_cost_test.cpp
            show_distorted_grid_pattern.cpp
            undistortion_sample.cpp
```

<br />

## Node File Example

To use these nodes, please use launch file for transmitting server parameters.

### Example for Node "show_distorted_grid_pattern"

<br />
<p align="center">
        <a href="https://github.com/QuintinUmi/camera_calibration/">
            <img src="https://github.com/QuintinUmi/camera_calibration/blob/main/src/camera_calibration_pkg/config/distortion_gridpattern_compare1.png?raw=true" alt="distortion_gridpattern_compare1"/>
            <img src="https://github.com/QuintinUmi/camera_calibration/blob/main/src/camera_calibration_pkg/config/distortion_gridpattern_compare2.png?raw=true" alt="distortion_gridpattern_compare1"/>     
        </a>
</p>

#### Method of Distorted GridPattern Generation

1. Use ```cv::getOptimalNewCameraMatrix()``` and ```cv::initUndistortRectifyMap()``` to get map1 and map2. Map1 contains a mapping table on (x, y) which refer to the source gridpattern image pixel, and map2 is a mapping table on y for refinement of the image. Map1 and map2 is a mapping from distorted image to undistorted image.
2. Find a mapping from undistorted image to distorted image for drawing a distorted gridpattern from a uniform grid pattern:
   Assume $\ X$ is the original image matrix, $\ X'$ is the target image processed by undistortion, $\ X''$ is the distorted gridpattern image we need to generate, and $\ T$ is the undistortion transform matrix. We have

   $\ X' = TX$  
   $\ ΔX = X - X' = X - TX = (I - T)X$  
   $\ X'' = X + ΔX = X + (I - T)X = (2I - T)X$  

   Therefore, the new transform matrix is $\ G = 2I - T$
   
   **Implementation**
   ```
   newMap1.at<cv::Vec2f>(y, x) = static_cast<cv::Vec2f>(cv::Vec2f(2.0 * x, 2.0 * y) - map1.at<cv::Vec2f>(y, x));
   #This is for map1 CV_32FC2 format.

   newMap2.at<uchar>(y, x) = static_cast<uchar>(2 * y - map2.at<uchar>(y, x));
   #This is for map2 CV_8U format.

   #For other format, please check the format table for cv::Mat::at() method to edit it. You can use cv::Mat::type() function to find the format code. 
   ```
   

4. Remapping by ```cv::remap()``` function.

### Example for Node "aruco_video_ext_calib"

After the camera intrinsics calibrating, real-time external calibration can be performed using an Aruco Marker with known true dimensions. 
The resulting rotation and translation matrices enable mapping from 3d space to 2d space. 
A simple function of AR can be realized by inserting objects (pictures, point clouds, etc.) into the 3d space.

<div align=center>
        <img src="https://github.com/QuintinUmi/camera_caliberation/blob/source/src/image/output1.gif?raw=true" width="400"/><img src="https://github.com/QuintinUmi/camera_caliberation/blob/source/src/image/output3.gif?raw=true" width="400"/>
</div>

## Launch File Description

Please use launch command to run the nodes. Before use it, please edit the yaml file in ./config folder.

<br />

## Contributor:   
QuintinUmi



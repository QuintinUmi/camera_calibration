cmake_minimum_required(VERSION 3.0.2)
project(camera_calibration_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(aruco REQUIRED)
# find_package(apriltag REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES camera_calibration_pkg
 CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ./include/
  /usr/local/include/
)

## Declare a C++ library
add_library(camera_calibration_lib
  include include/camera_calibration_tool.h
  include include/param_code.h
  lib/camera_calibration_tool.cpp
)
add_library(undistortion_lib
  include include/camera_calibration_tool.h
  include include/undistortion.h
  lib/undistortion.cpp
)
add_library(drawing_tool_lib
  include include/drawing_tool.h
  lib/drawing_tool.cpp
)
add_library(aruco_tool_lib
  include include/aruco_tool.h
  lib/aruco_tool.cpp
)

add_dependencies(camera_calibration_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(undistortion_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(drawing_tool_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(aruco_tool_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(camera_calibration_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(undistortion_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(drawing_tool_lib
  ${catkin_LIBRARIES}
)
target_link_libraries(aruco_tool_lib
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(camera_calibration src/camera_calibration.cpp)
add_executable(opencv_undistortion_time_cost_test src/opencv_undistortion_time_cost_test.cpp)
add_executable(undistortion_sample src/undistortion_sample.cpp)
add_executable(show_distorted_grid_pattern src/show_distorted_grid_pattern.cpp)
add_executable(chessboard_cam_ext_calibration src/chessboard_cam_ext_calibration.cpp)
add_executable(aruco_marker_generate src/aruco_marker_generate.cpp)
add_executable(aruco_ext_calib src/aruco_ext_calib.cpp)
add_executable(aruco_video_ext_calib src/aruco_video_ext_calib.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above


## Specify libraries to link a library or executable target against
target_link_libraries(camera_calibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  camera_calibration_lib
  yaml-cpp
)

target_link_libraries(undistortion_sample
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  camera_calibration_lib
  undistortion_lib
  yaml-cpp
)

target_link_libraries(opencv_undistortion_time_cost_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  camera_calibration_lib
  undistortion_lib
  yaml-cpp
)

target_link_libraries(show_distorted_grid_pattern
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  camera_calibration_lib
  undistortion_lib
  yaml-cpp
)

target_link_libraries(chessboard_cam_ext_calibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  camera_calibration_lib
  drawing_tool_lib
  aruco_tool_lib
  undistortion_lib
  yaml-cpp
)

target_link_libraries(aruco_marker_generate
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  camera_calibration_lib
  drawing_tool_lib
  aruco_tool_lib
  undistortion_lib
  yaml-cpp
)

target_link_libraries(aruco_ext_calib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  camera_calibration_lib
  drawing_tool_lib
  aruco_tool_lib
  undistortion_lib
  yaml-cpp
)

target_link_libraries(aruco_video_ext_calib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  camera_calibration_lib
  drawing_tool_lib
  aruco_tool_lib
  undistortion_lib
  yaml-cpp
)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_camera_calibration_pkg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

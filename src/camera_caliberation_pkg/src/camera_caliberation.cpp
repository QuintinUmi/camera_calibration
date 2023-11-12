#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "camera_caliberation_tool.h"

using namespace cct;

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "camera_caliberation");

	ros::NodeHandle rosHandle;

	std::string imagePath;
	std::string imageFormat;
	std::string yamlPath;

	int chessboardWidth, chessboardHeight;
	double squareSize;
	bool showCornersImage;

	int criteriaIterTimes;
	double iterDifference;

	rosHandle.param("image_load_path", imagePath, std::string("~/"));
	rosHandle.param("yaml_save_path", yamlPath, std::string("~/"));
	rosHandle.param("chessboard_width", chessboardWidth, 6);
	rosHandle.param("chessboard_height", chessboardHeight, 9);
	rosHandle.param("square_size", squareSize, 30.0);
	rosHandle.param("image_format", imageFormat, std::string("png"));
	rosHandle.param("show_corners_image", showCornersImage, false);
	rosHandle.param("criteria_iter_times", criteriaIterTimes, 100);
	rosHandle.param("iter_difference", iterDifference, 0.001);


	cv::Size chessboardSize(chessboardWidth, chessboardHeight);

	CamCalChessboard camCal(chessboardSize, squareSize);
	
	if(!camCal.get_images_from_path(imagePath, imageFormat)){
		printf("Failed to load image");
		return 0;
	}
	// camCal.show_src_image(0);
	// cv::Mat srcImage = camCal.get_src_image(0);
	// camCal.find_image_chessboard_corners(&srcImage, true);
	printf("criteriaIterTimes: %d, iterDifference: %.10f", criteriaIterTimes, iterDifference);
	camCal.caliberation_process(showCornersImage, criteriaIterTimes, iterDifference);
	camCal.save_caliberation_parm_yaml(yamlPath);
	

	return 0;
}

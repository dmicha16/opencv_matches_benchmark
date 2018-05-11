#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <filesystem>
#include<cstdlib>
#include<ctime>

#include "opencv2/opencv_modules.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/utility.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching/detail/autocalib.hpp"
#include "opencv2/stitching/detail/blenders.hpp"
#include "opencv2/stitching/detail/timelapsers.hpp"
#include "opencv2/stitching/detail/camera.hpp"
#include "opencv2/stitching/detail/exposure_compensate.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/stitching/detail/motion_estimators.hpp"
#include "opencv2/stitching/detail/seam_finders.hpp"
#include "opencv2/stitching/detail/warpers.hpp"
#include "opencv2/stitching/warpers.hpp"
#include "opencv2/core/ocl.hpp"

#pragma region namespaces
using namespace std;
using namespace cv;
using namespace cv::detail;
#pragma endregion

#ifdef _WIN32
#define WINPAUSE system("pause")
#endif

class ImageHandler {
public:
	ImageHandler(string path);
	~ImageHandler();
	vector<Mat> get_images();
	vector<String> get_image_names();
	vector<String> get_short_names();
	
private:

	int num_images_;
	vector<String> img_names_;
	vector<String> short_img_names_;
	vector<Mat> images_;

	void read_images_(string path);
	vector<Mat> upload_images_(vector<Mat> images);
	void shorten_image_names_(vector<String> img_names);
};
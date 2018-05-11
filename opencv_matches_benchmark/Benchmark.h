#pragma once
#pragma warning(disable : 4996)
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <filesystem>
#include <cstdlib>
#include <ctime>
#include <vector>

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

#include "clogging/Logger.h"

#pragma region namespaces
using namespace std;
using namespace cv;
using namespace cv::detail;
#pragma endregion

#ifdef _WIN32
#define WINPAUSE system("pause")
#endif

enum ResultsType {ORB_R = 1, AKAZE_R = 2, BRISK_R = 3};

struct ImageParams {
	vector<Mat> images;
	vector<ImageFeatures> image_features;
	vector<MatchesInfo> pairwise_matches;
	string matcher_type;
	int image_index;
	ResultsType results_type;
};

class Benchmark {
public:
	INIT_CLOGGING;
	Benchmark();
	~Benchmark();
	void draw_keypoints(vector<Mat> images, vector<ImageFeatures> image_features, vector<MatchesInfo> pairwise_matches);	
	void matcher(ImageParams image_params);

private:
	float calculate_deviation_(float avarage, vector<float> distances, int num_below_thresh);
	string construct_file_name_(string matcher_type, ResultsType results_type, int image_index);
	void draw_matches_(ImageParams image_params);
	void threshold_calculator_(const ImageParams image_params);
};


#include "../stdafx.h"
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

#include "Warping.h"
#include "Stitcher.h"

#pragma region namespaces
using namespace std;
using namespace cv;
using namespace cv::detail;
#pragma endregion

#ifdef _WIN32
#define WINPAUSE system("pause")
#endif

typedef struct RowDefiner {
	Point2f left;
	Point2f right;
	Point2f top_left;
};

typedef struct MatchedKeyPoint {
	vector<Point2f> image_1;
	vector<Point2f> image_2;
};

typedef struct Rectengales {
	vector<Rect> rectangles;
	vector<RowDefiner> row_definitions;
	vector<KeyPointList> keypoint_list;
	void populate_keypoint_list(MatchedKeyPoint matched_keypoints);
	void desginate_rectangles(int desired_rect);
	void populate_rectangles(int height_offset, int desired_cols);
	void reset_rectangles();
};

typedef struct KeyPointList {
	vector<Point2f> image_2;
};

typedef enum RectangleCases { PER16 = 0, PER32 = 1, PER48 = 2, PER64 = 3, PER80 = 4 };

class RoiCalculator {

public:	
	RoiCalculator();
	~RoiCalculator();

	void set_image(Mat inc_image_1, Mat inc_image);
	void calculate_roi(int desired_cols, int desired_rows, float overlap);
	bool check_keypoint();
	void set_matched_keypoints(MatchedKeyPoint inc_matched_keypoints, int threshold_percentage);
	int num_occupied_rects();

private:
	int num_rect_;
	Mat image_;
	Mat image_1_;
	int num_images_;
	int threshold_percentage_;

	Rectengales rectangles_s_;	
	MatchedKeyPoint matched_keypoints_;

	RowDefiner populate_row_definer_(int img_width, unsigned int start_height, int offset);
	vector<RowDefiner> row_definitions_;
	void rectangle_cases_(int desired_cols);
	void write_roi_(float min_height);
	vector<KeyPointList> sort_keypoints_list_(vector<KeyPointList> inc_list);
};


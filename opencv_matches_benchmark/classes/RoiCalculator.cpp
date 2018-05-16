#include "RoiCalculator.h"

RoiCalculator::RoiCalculator() {
}

RoiCalculator::~RoiCalculator() {
}

void RoiCalculator::calculate_roi(int desired_cols, int desired_rows, float overlap) {

	rectangles_s_.desginate_rectangles(desired_cols * desired_rows);
	num_rect_ = rectangles_s_.rectangles.size();
	
	const unsigned int img_width = image_.cols - 1;
	const unsigned int img_height = image_.rows - 1;
	row_definitions_.resize(desired_rows);
	vector<int> starting_row_heights(desired_rows);

	int height_offset = img_height * overlap / desired_rows;

	for (size_t i = 0; i < starting_row_heights.size(); i++)
		starting_row_heights[i] = img_height - height_offset * i;		

	for (size_t i = 0; i < row_definitions_.size(); i++)
		row_definitions_[i] = populate_row_definer_(img_width, starting_row_heights[i], height_offset);

	rectangles_s_.row_definitions = row_definitions_;
	rectangles_s_.populate_rectangles(height_offset, desired_cols);
	float min_height = (img_height * overlap);

	rectangles_s_.populate_keypoint_list(matched_keypoints_);
	//write_roi_(min_height);
	rectangle_cases_(desired_cols);	
}

//bool RoiCalculator::check_keypoint() {
//
//	for (size_t i = 0; i < num_images_; i++) {
//		for (size_t j = 0; j < num_rect_; j++) {
//			if (i == 1) {
//				for (size_t k = 0; k < matched_keypoints_.image_1.size(); k++) {
//					if (rectangles_s_.rectangles[j].contains(matched_keypoints_.image_1[k])) {
//						
//					}
//				}
//			}
//			else {
//				for (size_t k = 0; i < matched_keypoints_.image_2.size(); k++) {
//
//				}
//			}
//		}
//	}
//
//	return false;
//}

void RoiCalculator::rectangle_cases_(int desired_cols) {

	float rect_percent = 0;
	int rect_number = 0;

	vector<KeyPointList> sorted_keypoint_list = sort_keypoints_list_(rectangles_s_.keypoint_list);

	for (size_t i = 0; i < 5; i++) {

		MatchedKeyPoint shuffled_keypoints;

		switch (i) {
		case PER16:
			rect_percent = 0.17;
			break;
		case PER32:
			rect_percent = 0.34;
			break;
		case PER48:
			rect_percent = 0.51;
			break;
		case PER64:
			rect_percent = 0.68;
			break;
		case PER80:
			rect_percent = 0.85;
			break;
		}

		rect_number = rectangles_s_.rectangles.size() * rect_percent;		

		LOGLN("rect number: " << rect_number);

		for (size_t j = 0; j < rect_number; j++) {
			for (size_t k = 0; k < sorted_keypoint_list[j].image_2.size(); k ++) {
				shuffled_keypoints.image_2.push_back(sorted_keypoint_list[j].image_2[k]);
			}
		}

		for (size_t i = 0; i < shuffled_keypoints.image_2.size(); i++) {
			for (size_t j = 0; j < matched_keypoints_.image_2.size(); j++) {
				if (shuffled_keypoints.image_2[i].x == matched_keypoints_.image_2[j].x && 
						shuffled_keypoints.image_2[i].y == matched_keypoints_.image_2[j].y) {

					shuffled_keypoints.image_1.push_back(matched_keypoints_.image_1[j]);
					break;
				}
			}
		}

		Warping warper;
		Stitcher stitcher;

		LOGLN("shuffled_keypoints img1 size: " << shuffled_keypoints.image_1.size());
		LOGLN("shuffled_keypoints img2 size: " << shuffled_keypoints.image_2.size());

		Mat warped_img = warper.warp(image_1_, shuffled_keypoints);
		Mat stitched_img = stitcher.customMerger(image_, warped_img);

		String folder = "";

		switch(desired_cols) {
			case 3:
				folder = "case_low/";
				break;
			case 6:
				folder = "case_medium/";
				break;
			case 12:
				folder = "case_high/";
				break;
		}

		folder += to_string(threshold_percentage_) + "/";
		String name = "stitched_" + to_string(rect_percent * 100) + ".jpg";

		String output_location = "../opencv_matches_benchmark/images/results/" + folder + name;

		cv::imwrite(output_location, stitched_img);

		rect_number = 0;
		
	}
	
}

void RoiCalculator::write_roi_(float min_height) {

	String output_location = "../opencv_matches_benchmark/images/results/roi1.jpg"; 
	Point coord_to_display;

	for (size_t i = 0; i < rectangles_s_.rectangles.size(); i++)
		rectangle(image_, rectangles_s_.rectangles[i], Scalar(0, 0, 255), 3, LINE_8, 0);	
	
	for (size_t i = 0; i < matched_keypoints_.image_1.size(); i++) {
		coord_to_display.x = matched_keypoints_.image_1[i].x;
		coord_to_display.y = matched_keypoints_.image_1[i].y;		
		
		if (coord_to_display.y > (image_.rows - min_height))
			drawMarker(image_, coord_to_display, Scalar(0, 0, 255), 100, 50, LINE_8);			
		else
			drawMarker(image_, coord_to_display, Scalar(0, 255, 0), 100, 50, LINE_8);

		//cout << "ELSE display coords : (" << matched_keypoints_.image_1[i].x << ", " << matched_keypoints_.image_1[i].y << ")" << endl;
		coord_to_display.x = 0;
		coord_to_display.y = 0;
	}
	
	imwrite(output_location, image_);
	exit(0);
}

vector<KeyPointList> RoiCalculator::sort_keypoints_list_(vector<KeyPointList> inc_list) {

	vector<KeyPointList> temp_list = inc_list;	
	vector<Point2f> temp_point_holder;	

	for (size_t i = 0; i < temp_list.size(); i++) {
		LOGLN("temp list size before: " << i << " " << temp_list[i].image_2.size());
	}	
	
	for (int i = 0; i < temp_list.size(); i++) {

		temp_point_holder = temp_list[i].image_2;

		int counter = 0;
		for (int j = i; j < temp_list.size(); j++) {
			if (temp_list[j].image_2.size() > temp_point_holder.size() ) {
				temp_point_holder = temp_list[j].image_2;
				counter = j;		
			}
		}

		iter_swap(temp_list.begin() + i, temp_list.begin() + counter);
	}
	
	for (size_t i = 0; i < temp_list.size(); i++) {
		LOGLN("temp list size after sort: " << i << " " << temp_list[i].image_2.size());
	}
	WINPAUSE;
	return temp_list;
}

int RoiCalculator::num_occupied_rects() {

	int num_occupied_rects = 0;
	Point2f coord_to_check;

	for (size_t i = 0; i < rectangles_s_.rectangles.size(); i++) {
		for (size_t j = 0; j < matched_keypoints_.image_1.size(); j++) {
			coord_to_check.x = matched_keypoints_.image_1[j].x;
			coord_to_check.y = matched_keypoints_.image_1[j].y;
			if (rectangles_s_.rectangles[i].contains(coord_to_check)) {
				num_occupied_rects++;
				break;
			}
		}
	}

	rectangles_s_.reset_rectangles();
	return num_occupied_rects;
}

RowDefiner RoiCalculator::populate_row_definer_(int img_width, unsigned int start_height, int offset) {

	RowDefiner row_defition;
	const Vec3b kBLACKPIXEL = { 0, 0, 0 };	

	int noise_pixel_count = 0;

	row_defition.left.y = start_height;
	row_defition.right.y = start_height;	

	for (int pixel_idx = 0; pixel_idx < img_width; pixel_idx++) {
		if (image_.at<Vec3b>(start_height, pixel_idx) != kBLACKPIXEL) {
			noise_pixel_count = 0;
			for (int j = 1; j <= 5; j++) {
				if (image_.at<Vec3b>(start_height, (pixel_idx + j)) != kBLACKPIXEL) {					
					noise_pixel_count++;
				}	
			}
			if (noise_pixel_count == 5) {
				row_defition.left.x = pixel_idx;
				row_defition.top_left.y = start_height - offset;
				row_defition.top_left.x = pixel_idx;
				noise_pixel_count = 0;
				break;
			}	
		}
	}

	for (int pixel_idx = img_width; pixel_idx > 0; pixel_idx--) {
		if (image_.at<Vec3b>(start_height, pixel_idx) != kBLACKPIXEL) {
			noise_pixel_count = 0;
			for (int j = 1; j <= 5; j++) {
				if (image_.at<Vec3b>(start_height, (pixel_idx - j)) != kBLACKPIXEL)
					noise_pixel_count++;
			}
			if (noise_pixel_count == 5) {
				row_defition.right.x = pixel_idx;
				break;
			}
		}
	}

	return row_defition;
}

void RoiCalculator::set_image(Mat inc_image_1, Mat inc_image) {
	image_ = inc_image;
	image_1_ = inc_image_1;
	LOGLN("RoiCalculator::Images set.");
}

void RoiCalculator::set_matched_keypoints(MatchedKeyPoint inc_matched_keypoints, int threshold_percentage) {
	matched_keypoints_ = inc_matched_keypoints;
	threshold_percentage_ = (threshold_percentage + 1) * 10;
	LOGLN("RoiCalculator::Keypoints set.");
	LOGLN("Matched keypoints img1 size: " << matched_keypoints_.image_1.size());
	LOGLN("Matched keypoints img2 size: " << matched_keypoints_.image_2.size());
	LOGLN("RoiCalculator::threshold_percentage: " << threshold_percentage_);
}

void Rectengales::populate_rectangles(int height_offset, int desired_cols) {
	
	int start_x = 0;

	for (size_t j = 0; j < row_definitions.size(); j++) {
		for (size_t i = j * desired_cols; i < (j * desired_cols + desired_cols); i++) {
			rectangles[i].width = (row_definitions[j].right.x - row_definitions[j].left.x) / desired_cols;
			rectangles[i].height = height_offset;
			 
			 if (i == j * desired_cols) {
				 start_x = row_definitions[j].top_left.x;
			 }

			 rectangles[i].x = start_x;
			 rectangles[i].y = row_definitions[j].top_left.y;

			 start_x += rectangles[i].width;			
		}
		start_x = 0;
	}	
}

void Rectengales::reset_rectangles() {	
	rectangles.clear();
}

void Rectengales::populate_keypoint_list(MatchedKeyPoint matched_keypoints) {	

	Point2f coord_to_check;	

	for (size_t i = 0; i < matched_keypoints.image_2.size(); i++) {

		coord_to_check.x = matched_keypoints.image_2[i].x;
		coord_to_check.y = matched_keypoints.image_2[i].y;

		for (size_t j = 0; j < keypoint_list.size(); j++) {
			
			if (rectangles[j].contains(coord_to_check)) {
				keypoint_list[j].image_2.push_back(coord_to_check);				
				break;
			}
		}
	}
}

void Rectengales::desginate_rectangles(int desired_rect) {

	if (desired_rect % 2 != 0)
		desired_rect = desired_rect + 1;
	
	Rect rect_filler;	
	for (size_t i = 0; i < desired_rect; i++)
		rectangles.push_back(rect_filler);	
	
	keypoint_list.resize(rectangles.size());
}

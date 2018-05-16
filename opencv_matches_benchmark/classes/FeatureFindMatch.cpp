#include "FeatureFindMatch.h"

struct SortOperator {
	bool operator() (int i, int j) {
		return (i < j);
	}
} sort_operator_;

FeatureFindMatch::FeatureFindMatch() {
}

FeatureFindMatch::~FeatureFindMatch() {
}

void FeatureFindMatch::find_features(const vector<Mat> inc_images) {

	threshold_ = 1;
	num_images_ = static_cast <int>(inc_images.size());
	image_features_.resize(num_images_);
	string features_out;

	float scaleFactor = 1.2f;
	int nlevels = 8;
	int edgeThreshold = 31;
	int firstLevel = 0;
	int WTA_K = 2;
	int scoreType = ORB::HARRIS_SCORE;
	int patchSize = 31;
	int fastThreshold = 20;	

	InputArray mask = noArray();
	Ptr<ORB> detector_desciptor;
	detector_desciptor = ORB::create(150000, scaleFactor, nlevels, edgeThreshold, 
		firstLevel, WTA_K, scoreType, patchSize, fastThreshold);	

	for (int i = 0; i < num_images_; ++i) {

		features_out = "Features in image ";

		try {
			detector_desciptor->detectAndCompute(inc_images[i], mask,
				image_features_[i].keypoints, image_features_[i].descriptors);
		}
		catch (const std::exception& e) {
			cout << e.what() << endl;
		}		

		image_features_[i].img_idx = i;
		features_out += to_string(i + 1) + ": " + to_string(image_features_[i].keypoints.size());		
		LOGLN(features_out);

	}
	//for (size_t j = 0; j < image_features_[1].keypoints.size(); j++) {
	//	LOGLN("Features img2: " << image_features_[1].keypoints[j].pt);
	//	/*string msg = "Features img2: " + to_string(image_features_[1].keypoints[j].pt.x) + " " + to_string(image_features_[1].keypoints[j].pt.y);
	//	CLOG(msg);*/
	//}	

	match_features_(inc_images);
}

MatchedKeyPoint FeatureFindMatch::get_matched_coordinates() {
	return matched_keypoints_;
}

bool FeatureFindMatch::keypoint_area_check_(vector<Mat> inc_images) {

	desired_rectangle_.image_overlap = 0.8;

	for (size_t i = 0; i < 3; i++) {

		switch (i) {
		case Cases::LOW:
			desired_rectangle_.columns = 3;
			desired_rectangle_.rows = 2;			
			break;
		case Cases::MEDIUM:
			desired_rectangle_.columns = 6;
			desired_rectangle_.rows = 4;
			break;
		case Cases::HIGH:
			desired_rectangle_.columns = 12;
			desired_rectangle_.rows = 8;
			break;
		}

		LOGLN("Desired columns: " << desired_rectangle_.columns);
		LOGLN("Desired rows: " << desired_rectangle_.rows);
		
		RoiCalculator roi_calculator;
		roi_calculator.set_image(inc_images[0], inc_images[1]);

		for (size_t j = 0; j < 4; j++) {
			roi_calculator.set_matched_keypoints(keypoints_by_percentage_[j], j);
			roi_calculator.calculate_roi(desired_rectangle_.columns,
				desired_rectangle_.rows, desired_rectangle_.image_overlap);
		}	

		desired_rectangle_.columns = 0;
		desired_rectangle_.rows = 0;
	}	
	return true;
}

void FeatureFindMatch::match_features_(const vector<Mat> inc_images) {

	float match_conf = 0.3f;
	bool try_cuda = false;

	vector<MatchesInfo> pairwise_matches;
	image_data_.img_1 = inc_images[0];
	image_data_.img_2 = inc_images[1];
	image_data_.keypoints_1 = image_features_[0].keypoints;
	image_data_.keypoints_2 = image_features_[1].keypoints;

#pragma region logging

	string keypoints_features_1 = "Keypoints 1 from features i: " + to_string(image_data_.keypoints_1.size());
	string keypoints_features_2 = "Keypoints 2 from features i: " + to_string(image_data_.keypoints_2.size());	

	string image_length = "Images length: " + to_string(inc_images.size());
	string keypoints_length_1 = "Keypoints_1 length: " + to_string(image_data_.keypoints_1.size());
	string keypoints_length_2 = "Keypoints_2 length: " + to_string(image_data_.keypoints_2.size());	

	LOG("Pairwise matching\n");	

#pragma endregion //local_logging

	Ptr<FeaturesMatcher> current_matcher = makePtr<AffineBestOf2NearestMatcher>(false, try_cuda, match_conf);

	try {
		(*current_matcher)(image_features_, pairwise_matches);
		//(*current_matcher)(image_features_[0], image_features_[1], pairwise_matches);
	}
	catch (const std::exception& e) {
		cout << e.what() << endl;		
		WINPAUSE;
	}

	image_data_.all_matches = pairwise_matches[1].matches;

	LOGLN("Matches found: " << image_data_.all_matches.size());
	/*matches_drawer_(image_data_.all_matches);
	WINPAUSE;*/
	
	filter_matches_(inc_images);
	keypoint_area_check_(inc_images);
	
	current_matcher->collectGarbage();
}

void FeatureFindMatch::filter_matches_(const vector<Mat> inc_images) {

	vector<DMatch> filtered_matches;
	bool enough_occupied = false;
	int calculated_threshold = 0;
	int desirec_occupied_rects = desired_rectangle_.desired_occupied;

	for (float i = 0.1; i < 0.5; i += 0.1) {

		calculated_threshold = calculate_treshold_(image_data_.all_matches, i);		

		for (size_t j = 0; j < image_data_.all_matches.size(); j++) {

			if (image_data_.all_matches[j].distance <= calculated_threshold)
				filtered_matches.push_back(image_data_.all_matches[j]);
		}

		int idx = i * 10 - 1;
		/*if (idx == 0) {
			for (size_t z = 0; z < filtered_matches.size(); z++) {
				LOGLN("filtered_matches z: " << z << " " << image_data_.keypoints_2[filtered_matches[z].trainIdx].pt);
			}
		}*/

		keypoints_by_percentage_[idx].image_1.resize(filtered_matches.size());
		keypoints_by_percentage_[idx].image_2.resize(filtered_matches.size());

		//LOGLN("Filtered matches size: " << filtered_matches.size());

		for (size_t k = 0; k < filtered_matches.size(); k++) {

			keypoints_by_percentage_[idx].image_2[k].x = (image_data_.keypoints_1[filtered_matches[k].queryIdx].pt.x);
			keypoints_by_percentage_[idx].image_2[k].y = (image_data_.keypoints_1[filtered_matches[k].queryIdx].pt.y);

			keypoints_by_percentage_[idx].image_1[k].x = (image_data_.keypoints_2[filtered_matches[k].trainIdx].pt.x);
			keypoints_by_percentage_[idx].image_1[k].y = (image_data_.keypoints_2[filtered_matches[k].trainIdx].pt.y);
		}
		//LOGLN("Populated keypoints: " << i << " " << keypoints_by_percentage_[i].image_2);
	}
	/*for (size_t i = 0; i < keypoints_by_percentage_[0].image_2.size(); i++) {
		LOGLN("Keypoints by percentage: " << i << " " << keypoints_by_percentage_[0].image_2[i]);
	}*/
	
	//matches_drawer_(filtered_matches);
}

int FeatureFindMatch::calculate_treshold_(vector<DMatch> my_matches, float desired_percentage) {

	float calculated_tresh = 0;
	vector<float> distances;
	int avarage = 0;

	for (size_t i = 0; i < my_matches.size(); i++)
		distances.push_back(my_matches[i].distance);

	std::sort(distances.begin(), distances.end(), sort_operator_);

	for (size_t i = 0; i < my_matches.size(); i++)
		avarage = avarage + my_matches[i].distance;

	avarage = avarage / my_matches.size();

	//for (size_t i = 0; i < my_matches.size(); i++) {
	//	//LOGLN("Keypoints by percentage: " << i << " " << my_matches[i].queryIdx);
	//	LOGLN("my_matches: " << i << " " << my_matches[i].);
	//}

	calculated_tresh = distances[(int)distances.size() * desired_percentage];
	return calculated_tresh;
}

void FeatureFindMatch::matches_drawer_(vector<DMatch> filtered_matches) {

	String output_location = "../opencv_matches_benchmark/images/results/test_2.jpg";
	vector<char> mask(filtered_matches.size(), 1);
	Mat output_img;
	
	drawMatches(image_data_.img_1, image_data_.keypoints_1, image_data_.img_2, image_data_.keypoints_2, filtered_matches, output_img, Scalar::all(-1),
		Scalar::all(-1), mask, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	imwrite(output_location, output_img);
	exit(0);
}

void FeatureFindMatch::set_rectangle_info(int rows, int columns, float overlap, int desired_occupied) {
	desired_rectangle_.rows = rows;
	desired_rectangle_.columns = columns;
	desired_rectangle_.image_overlap = overlap;
	desired_rectangle_.desired_occupied = desired_occupied;
}

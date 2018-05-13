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

void FeatureFindMatch::find_features(const vector<Mat> inc_images, const float inc_threshold) {

	threshold_ = inc_threshold;
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
	detector_desciptor = ORB::create(50000, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);

	vector<Mat> local_image_holder;
	Mat temp_image_holder;
	for (size_t i = 0; i < inc_images.size(); i++)
		local_image_holder.push_back(inc_images[i]);	

	if (inc_images[0].rows > inc_images[1].rows) {
		temp_image_holder = local_image_holder[0];
		temp_image_holder.resize(inc_images[1].rows);
		local_image_holder[0] = temp_image_holder;

		temp_image_holder.empty();
	}

	for (int i = 0; i < num_images_; ++i) {

		features_out = "Features in image #";

		try {
			detector_desciptor->detectAndCompute(local_image_holder[i], mask, image_features_[i].keypoints, image_features_[i].descriptors);
		}
		catch (const std::exception& e) {
			cout << e.what() << endl;
		}

		cout << "image 1 size at features finding: " << local_image_holder[0].size << endl;
		cout << "image 2 size at features finding: " << local_image_holder[1].size << endl;

		image_features_[i].img_idx = i;
		features_out += to_string(i + 1) + ": " + to_string(image_features_[i].keypoints.size());		
		LOGLN(features_out);

	}
	local_image_holder.clear();
	match_features_(inc_images, image_features_);
}

MatchedKeyPoint FeatureFindMatch::get_matched_coordinates() {
	return matched_keypoints_;
}

bool FeatureFindMatch::keypoint_area_check_(vector<Mat> inc_images, int desired_occ_rects) {

	RoiCalculator roi_calculator;
	roi_calculator.set_image(inc_images[1]);
	roi_calculator.set_matched_keypoints(matched_keypoints_);
	roi_calculator.calculate_roi(desired_rectangle_.columns,
		desired_rectangle_.rows, desired_rectangle_.image_overlap);

	if (roi_calculator.num_occupied_rects() >= desired_occ_rects)
		return true;
	else
		return false;
}

void FeatureFindMatch::match_features_(const vector<Mat> inc_images, const vector<ImageFeatures> image_features_) {

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
	//display_pairwise_matches_(pairwise_matches);
	filter_matches_(inc_images);
	
	current_matcher->collectGarbage();
}

void FeatureFindMatch::filter_matches_(const vector<Mat> inc_images) {

	vector<DMatch> filtered_matches;
	bool enough_occupied = false;
	int calculated_threshold = 0;
	int desirec_occupied_rects = desired_rectangle_.desired_occupied;

	do {
		calculated_threshold = 0;
		matched_keypoints_.image_1.clear();
		matched_keypoints_.image_2.clear();
		filtered_matches.clear();

		calculated_threshold = calculate_treshold_(image_data_.all_matches, threshold_);
		//cout << "calculated_threshold: " << calculated_threshold << endl;

		for (size_t i = 0; i < image_data_.all_matches.size(); i++) {

			if (image_data_.all_matches[i].distance <= calculated_threshold)
				filtered_matches.push_back(image_data_.all_matches[i]);
		}
		//cout << "filtered_matches.size: " << filtered_matches.size() << endl;

		matched_keypoints_.image_1.resize(filtered_matches.size());
		matched_keypoints_.image_2.resize(filtered_matches.size());

		for (size_t i = 0; i < filtered_matches.size(); i++) {
			matched_keypoints_.image_1[i].x = (image_data_.keypoints_1[filtered_matches[i].queryIdx].pt.x);
			matched_keypoints_.image_1[i].y = (image_data_.keypoints_1[filtered_matches[i].queryIdx].pt.y);

			matched_keypoints_.image_2[i].x = (image_data_.keypoints_2[filtered_matches[i].trainIdx].pt.x);
			matched_keypoints_.image_2[i].y = (image_data_.keypoints_2[filtered_matches[i].trainIdx].pt.y);
			//cout << matched_keypoints_.image_1[i] << endl;
			//cout << matched_keypoints_.image_2[i] << endl;
		}

		enough_occupied = keypoint_area_check_(inc_images, desirec_occupied_rects);
		/*cout << "enough occupied: " << boolalpha << enough_occupied << endl;		
		cout << "current_threshold: " << calculated_threshold << endl;
		cout << "threshold_: " << threshold_ << endl;*/
		threshold_ += 0.1;

	} while ((!enough_occupied) && (threshold_ <= 1));

	cout << "Threshold has been set to: " << threshold_ << endl;	
	cout << "Good matches #:" << filtered_matches.size() << endl;
	
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

	calculated_tresh = distances[(int)distances.size() * desired_percentage];
	return calculated_tresh;
}

void FeatureFindMatch::matches_drawer_(vector<DMatch> filtered_matches) {

	String output_location = "../opencv_image_stitching/Images/Results/test_1.jpg";
	vector<char> mask(filtered_matches.size(), 1);
	Mat output_img;
	
	drawMatches(image_data_.img_1, image_data_.keypoints_1, image_data_.img_2, image_data_.keypoints_2, filtered_matches, output_img, Scalar::all(-1),
		Scalar::all(-1), mask, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	imwrite(output_location, output_img);
}

void FeatureFindMatch::display_pairwise_matches_(const vector<MatchesInfo> pairwise_matches) {

	cout << "\n-----------------------------" << endl;
	cout << "display_pairwise_matches_() {" << endl;
	cout << "\t-------------------" << endl;

	for (size_t i = 1; i < pairwise_matches.size() - 1; i++) {
		cout << "\tdst_img_indx: " << pairwise_matches[i].dst_img_idx << endl;
		cout << "\tconfidence: " << pairwise_matches[i].confidence << endl;
		cout << "\tH: " << pairwise_matches[i].H << endl;
		//cout << "inliers_mask[i]: " << pairwise_matches[i].inliers_mask[i] << endl;
		cout << "\tnum_inliers: " << pairwise_matches[i].num_inliers << endl;
		cout << "\t-------------------" << endl;
	}
	cout << "}" << endl;
	cout << "-----------------------------" << endl << endl;
}


void FeatureFindMatch::set_rectangle_info(int rows, int columns, float overlap, int desired_occupied) {
	desired_rectangle_.rows = rows;
	desired_rectangle_.columns = columns;
	desired_rectangle_.image_overlap = overlap;
	desired_rectangle_.desired_occupied = desired_occupied;
}

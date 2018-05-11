#include "stdafx.h"
#include "Benchmark.h"

struct SortOperator {
	bool operator() (int i, int j) {
		return (i < j);
	}
} sort_operator_;

Benchmark::Benchmark() {
	ADD_FILE("clogging.csv");
}

Benchmark::~Benchmark() {
}
void Benchmark::draw_keypoints(vector<Mat> images, vector<ImageFeatures> image_features, vector<MatchesInfo> pairwise_matches) {
	
}

void Benchmark::draw_matches_(const ImageParams image_params) {

	string output_location = construct_file_name_(image_params.matcher_type, image_params.results_type, image_params.image_index);
	size_t image_idx = image_params.image_index;
	vector<Mat> images = image_params.images;
	vector<DMatch> matches = image_params.pairwise_matches[1].matches;
	vector<char> mask(matches.size(), 1);

	Mat output_img;

	try {
		drawMatches(images[0], image_params.image_features[0].keypoints,
			images[1], image_params.image_features[1].keypoints, matches, output_img, Scalar::all(-1),
			Scalar::all(-1), mask, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	}
	catch (const std::exception& e) {
		cout << e.what() << endl;
	}

	Mat resized_image;
	resize(output_img, resized_image, cv::Size(), 1, 1);

	if (imwrite(output_location, resized_image)) {
		//printf("Image result written.\n");
		//CLOG("Image result successfully written.", INFO);
	}		
	else {
		//printf("Image failed to write.\n");
		CLOG("Image failed to write.", ERR, CSV);
	}
}

float Benchmark::calculate_deviation_(float avarage, vector<float> distances, int num_below_thresh) {

	vector<float> distances_below_thresh;
	float standard_deviation = 0;
	float sum = 0;
	int devider = 0;

	if (num_below_thresh == 0) {
		for (size_t i = 0; i < distances.size(); i++)
			distances_below_thresh.push_back(distances[i]);
		devider = distances.size();
	}
	else {
		for (size_t i = 0; i < distances[num_below_thresh]; i++)
			distances_below_thresh.push_back(distances[i]);
		devider = num_below_thresh;
	}	

	for (size_t i = 0; i < distances_below_thresh.size(); i++)
		standard_deviation += pow(distances_below_thresh[i] - avarage, 2);		

	return sqrt(standard_deviation / devider);
}

string Benchmark::construct_file_name_(string matcher_type, ResultsType results_type, int image_index) {

	string output_location = "../opencv_features_benchmark/Images";

	switch (results_type) {
	case ORB_R:
		output_location += "/ORB/" + matcher_type + "_" + to_string(image_index) + "-" + to_string(image_index + 1) + ".jpg";
		break;
	case BRISK_R:
		output_location += "/BRISK/" + matcher_type + "_" + to_string(image_index) + "-" + to_string(image_index + 1) + ".jpg";
		break;
	case AKAZE_R:
		output_location += "/AKAZE/" + matcher_type + "_" + to_string(image_index) + "-" + to_string(image_index + 1) + ".jpg";
		break;
	}
	return output_location;
}

void Benchmark::threshold_calculator_(const ImageParams image_params) {

	vector<DMatch> matches = image_params.pairwise_matches[1].matches;
	vector<float> distances;
	vector<float> thresholds;	
	float thresh = 0;

	for (size_t i = 0; i < matches.size(); i++)
		distances.push_back(matches[i].distance);

	std::sort (distances.begin(), distances.end(), sort_operator_);
	for (float i = 0.1; i < 0.5; i += 0.1) {		
		thresh = distances[(int)distances.size() * i];
		thresholds.push_back(thresh);
	}

	string msg;
	float avarage = 0;
	int num_below_thresh = 0;
	stringstream ss;

	for (size_t i = 0; i < distances.size(); i++)
		avarage += distances[i];

	avarage = avarage / distances.size();
	string av = to_string((int)avarage);
	CLOG(av, INFO, CSV_A);

	float deviation = calculate_deviation_(avarage, distances, num_below_thresh);
	ss << setprecision(3) << deviation;
	CLOG(ss.str(), INFO, CSV_A);
	ss.str(string());

	CLOG(" ", INFO, CSV_A);

	avarage = 0;
	deviation = 0;

	for (size_t i = 0; i < thresholds.size(); i++) {		
		for (size_t j = 0; j < distances.size(); j++) {
			if (distances[j] <= thresholds[i]) {
				avarage += distances[j];
				num_below_thresh++;
			}
		}

		// current threshold		
		msg = to_string(thresholds[i]);
		CLOG(msg, INFO, CSV_A);
		msg.clear();

		//number of matches under the first threshold		
		msg = to_string(num_below_thresh);
		CLOG(msg, INFO, CSV_A);		
		msg.clear();
		
		//average of matches below threshold
		avarage = avarage / num_below_thresh;
		msg = to_string((int)avarage);
		CLOG(msg, INFO, CSV_A);

		deviation = calculate_deviation_(avarage, distances, num_below_thresh);		
		ss << setprecision(3) << deviation;		
		CLOG(ss.str(), INFO, CSV_A);
		CLOG(" ", INFO, CSV_A);
		
		ss.str(string());
		msg.clear();
		num_below_thresh = 0;
		avarage = 0;
	}		
}

void Benchmark::matcher(ImageParams image_params) {

	vector<ImageFeatures> image_features = image_params.image_features;

	float match_conf = 0.3f;
	bool try_cuda = false;
	int range_width = -1;

	vector<MatchesInfo> pairwise_matches;
	Ptr<FeaturesMatcher> current_matcher;
	string matcher_type;
	string number_of_matches;
	string number_of_matches_clog;

	for (size_t i = 2; i < 4; i++) {

		switch (i) {
		case 1:
			current_matcher = makePtr<BestOf2NearestMatcher>(false, try_cuda, match_conf);
			matcher_type = "DEFAULT";
			image_params.matcher_type = matcher_type;
			CLOG(matcher_type, INFO, CSV);
			printf("DEFAULT.\n");
			break;
		case 2:
			current_matcher = makePtr<BestOf2NearestRangeMatcher>(false, try_cuda, match_conf);
			matcher_type = "RANGE";
			image_params.matcher_type = matcher_type;
			CLOG(matcher_type, INFO, CSV);
			printf("RANGE.\n");
			break;
		case 3:
			current_matcher = makePtr<AffineBestOf2NearestMatcher>(false, try_cuda, match_conf);
			matcher_type = "AFFINE";
			image_params.matcher_type = matcher_type;
			CLOG(matcher_type, INFO, CSV);
			printf("AFFINE.\n");
			break;
		}

		try {
			(*current_matcher)(image_features, pairwise_matches);
		}
		catch (const std::exception& e) {
			cout << e.what() << endl;
		}

		number_of_matches = "Total number of matches:, " + to_string(pairwise_matches[1].matches.size());
		number_of_matches_clog = to_string(pairwise_matches[1].matches.size());
		CLOG(" ", INFO, CSV_A);
		CLOG(" ", INFO, CSV_A);
		CLOG(number_of_matches_clog, INFO, CSV_A);

		image_params.pairwise_matches = pairwise_matches;

		threshold_calculator_(image_params);
		draw_matches_(image_params);
		pairwise_matches.clear();
		number_of_matches = "";
	}
}

#include "stdafx.h"
#include "Orb.h"

Orb::Orb() {
}

Orb::~Orb() {
}

void Orb::find_features(vector<Mat> images, int idx) {

	image_params_.image_index = idx;

	num_images = static_cast<int> (images.size());
	image_features.resize(num_images);
	string features_out;

	for (int i = 0; i < num_images; i++) {		

		features_out.clear();
		float scaleFactor = 1.2f;
		int nlevels = 8;
		int edgeThreshold = 31;
		int firstLevel = 0;
		int WTA_K = 2;
		int scoreType = ORB::HARRIS_SCORE;
		int patchSize = 31;
		int fastThreshold = 20;

		Ptr<ORB> detector_desciptor;

		detector_desciptor = ORB::create(100000, scaleFactor, nlevels, edgeThreshold,
			firstLevel, WTA_K, scoreType, patchSize, fastThreshold);

		try {
			InputArray mask = noArray();
			detector_desciptor->detectAndCompute(images[i], mask, image_features[i].keypoints, image_features[i].descriptors);
		}
		catch (const std::exception& e) {
			cout << e.what() << endl;
		}

		cout << "Features in image: " << idx << ": " << image_features[i].keypoints.size() << endl;
		features_out += to_string(image_features[i].keypoints.size());
		CLOG(features_out, INFO, CSV_A);

		image_features[i].img_idx = i;
	}

	image_params_.image_features = image_features;
	image_params_.results_type = ORB_R;
	image_params_.images = images;

	matcher(image_params_);
}

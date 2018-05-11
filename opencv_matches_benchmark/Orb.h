#pragma once
#include "Benchmark.h"

class Orb :
	public Benchmark {
public:
	Orb();
	void find_features(vector<Mat> images, int idx);
	~Orb();

private:
	vector<ImageFeatures> image_features;
	ImageParams image_params_;
	int num_images;
};


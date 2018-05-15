// opencv_matches_benchmark.cpp : Defines the entry point for the console application.
#include "classes/Wrapper.h"
#include "classes/FeatureFindMatch.h"
#include "ImageHandler.h"
#include "classes/Warping.h"

int main()
{

	String path = "../opencv_features_benchmark/Images/";
	ImageHandler image_handler(path);
	vector<Mat> all_images = image_handler.get_images();
	vector<String> image_names = image_handler.get_short_names();	
	vector<Mat> images_to_stitch;

	Warping warper_translater;
	FeatureFindMatch finder;	

	images_to_stitch[0] = warper_translater.translate(images_to_stitch[0], 2000, 0);
	images_to_stitch[1] = warper_translater.translate(images_to_stitch[1], 2000, 0);	

	finder.find_features(images_to_stitch);

	cout << endl << "------------ MISSION COMPLETE ------------" << endl;
	waitKey(0);
	
    return 0;
}


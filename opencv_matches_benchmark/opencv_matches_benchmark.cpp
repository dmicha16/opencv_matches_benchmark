// opencv_matches_benchmark.cpp : Defines the entry point for the console application.
#include "classes/Wrapper.h"
#include "classes/FeatureFindMatch.h"
#include "ImageHandler.h"
#include "classes/Warping.h"

int main()
{
	INIT_CLOGGING;
	CLOGGING_TIMER;
	ADD_FILE("clogging.log");
	cv::ocl::setUseOpenCL(false);
	LOGLN("Application started");
	String path = "../opencv_matches_benchmark/images/";
	ImageHandler image_handler(path);
	vector<Mat> all_images = image_handler.get_images();
	vector<String> image_names = image_handler.get_short_names();	

	Warping warper_translater;
	FeatureFindMatch finder;

	LOGLN("Objects constructed.");

	all_images[0] = warper_translater.translate(all_images[0], 2000, 0);
	all_images[1] = warper_translater.translate(all_images[1], 2000, 0);

	LOGLN("Images translated.");

	finder.find_features(all_images);

	cout << endl << "------------ MISSION COMPLETE ------------" << endl;
	WINPAUSE;
	
    return 0;
}


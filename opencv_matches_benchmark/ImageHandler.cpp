#include "ImageHandler.h"


ImageHandler::ImageHandler(string path) {
	read_images_(path);
}

ImageHandler::~ImageHandler() {
}

vector<Mat> ImageHandler::get_images() {
	return images_;
}

vector<String> ImageHandler::get_image_names() {
	return img_names_;
}

vector<String> ImageHandler::get_short_names() {
	return short_img_names_;
}

void ImageHandler::read_images_(string path) {
	vector<String> photos;

	/*for (auto & file : experimental::filesystem::directory_iterator(path))
	cout << file << endl;*/

	glob(path, photos, false);

	cout << "Images read: " << photos.size() << endl;
	//WINPAUSE;
	for (int i = 0; i < photos.size(); i++) {
		img_names_.push_back(photos[i]);
	}

	num_images_ = static_cast <int> (img_names_.size());
	images_.resize(num_images_);
	images_ = upload_images_(images_);
}

vector<Mat> ImageHandler::upload_images_(vector<Mat> images) {

	Mat full_img, img;
	for (int i = 0; i < num_images_; ++i)
		images_[i] = imread(img_names_[i]);

	return images_;	
}

void ImageHandler::shorten_image_names_(vector<String> img_names) {

	string img_name;
	string temp;
	for (size_t i = 0; i < img_names.size(); i++) {
		temp.assign(img_names[i]);
		for (size_t j = 0; j < temp.size(); j++) {
			if (temp.at(j) == '\\') {
				for (size_t k = (j + 1); k < temp.size(); k++) {
					img_name += temp.at(k);
				}
				break;
			}
		}
		short_img_names_.push_back(img_name);
		img_name.clear();
		temp.clear();
	}
}

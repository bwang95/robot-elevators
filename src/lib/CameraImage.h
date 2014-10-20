#ifndef CAMERA_IMAGE_H
#define CAMERA_IMAGE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

struct CameraImage {
	int width;
	int height;
	Mat image;

	CameraImage(vector<unsigned char> data, int width, int height):
		image(width, height, CV_8U, &data, width),
		width(width),
		height(height) {
	}
};

#endif //CAMERA_IMAGE_H
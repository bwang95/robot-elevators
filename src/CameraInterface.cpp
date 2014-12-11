#include "lib/CameraInterface.h"
#include <cmath>

#define SCALE 2

double area(vector<Point> *);
int isCloserTo(double, double, double);
bool isReasonablyIsosceles(vector<Point> *);
double distance(double, double, double, double);

void CameraInterface::image_callback(
  const sensor_msgs::Image::ConstPtr &msg) {
	CameraImage *ptr = lastImage;
	lastImage = new CameraImage(msg);
	process();
	if (ptr != NULL)
		delete ptr;
}

void CameraInterface::process() {
	int cols = lastImage->width;
	int rows = lastImage->height;

	Mat hsv_frame(cols, rows, CV_8UC4);
	Mat thresholded(cols, rows, CV_8UC1);

	Mat image = lastImage -> image;

	cvtColor(image, hsv_frame, CV_RGB2HSV);
	inRange(hsv_frame, hsv_min, hsv_max, thresholded);

	vector <vector <Point> > contours, approximation;
	vector <Vec4i> hierarchy;
	findContours(thresholded, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

	approximation.resize(contours.size());
	for (int k = 0; k < contours.size(); k++) {
		approxPolyDP(Mat(contours[k]), approximation[k], 8, true);
	}
	for ( int i = 0; i < approximation.size(); i++ ) {
		if (approximation[i].size() == 3 && verifyTriangle(&approximation[i])) {
			Scalar color = Scalar(255, 255, 255);
			drawContours( thresholded, approximation, i, color, 2, 8, hierarchy, 0, Point() );
		}
	}
	namedWindow("contours", 1);
	imshow("contours", thresholded);
	waitKey(1);
}

void CameraInterface::generateMinMax(Scalar ideal, Scalar range, enum hsv_type type) {
	double h = max((int)(ideal.val[H] - range.val[H]), 0);
	double s = max((int)(ideal.val[S] - range.val[S]), 0);
	double v = max((int)(ideal.val[V] - range.val[V]), 0);

	hsv_min = Scalar(h, s, v);

	h = min((int)(ideal.val[H] + range.val[H]), 255);
	s = min((int)(ideal.val[S] + range.val[S]), 255);
	v = min((int)(ideal.val[V] + range.val[V]), 255);

	hsv_max = Scalar(h, s, v);
}

enum Direction CameraInterface::getDirection(vector<Point> *vertices) {
	double A[3] = {
		(*vertices)[0].y,
		(*vertices)[1].y,
		(*vertices)[2].y
	};

	double minimum = min(A[0], min(A[1], A[2]));
	double maximum = max(A[0], max(A[1], A[2]));

	int closeToMin = 0;

	closeToMin += isCloserTo(A[0], minimum, maximum) == 1 ? 1 : 0;
	closeToMin += isCloserTo(A[1], minimum, maximum) == 1 ? 1 : 0;
	closeToMin += isCloserTo(A[2], minimum, maximum) == 1 ? 1 : 0;
	closeToMin += isCloserTo(A[2], minimum, maximum) == 1 ? 1 : 0;

	if (closeToMin == 1)
		return DIR_UP;

	return DIR_DOWN;
}

double area(vector<Point> *vertices) {
	if (vertices->size() != 3)
		return -1;
	double A[2] = {(*vertices)[0].x, (*vertices)[0].y};
	double B[2] = {(*vertices)[1].x, (*vertices)[1].y};
	double C[2] = {(*vertices)[2].x, (*vertices)[2].y};

	return abs(A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1])) / 2.0;
}

int isCloserTo(double subject, double one, double two) {
	double diffOne = abs(one - subject);
	double diffTwo = abs(two - subject);
	return diffTwo < diffOne ? 2 : 1;
}

bool isReasonablyIsosceles(vector<Point> *vertices) {
	double A[2] = {(*vertices)[0].x, (*vertices)[0].y};
	double B[2] = {(*vertices)[1].x, (*vertices)[1].y};
	double C[2] = {(*vertices)[2].x, (*vertices)[2].y};

	double sides[3] = {
		distance(A[0], A[1], B[0], B[1]),
		distance(B[0], B[1], A[0], A[1]),
		distance(B[0], B[1], C[0], C[1])
	};

	sides[1] /= sides[0];
	sides[2] /= sides[0];
	sides[0] = 1;

	return (abs(sides[1] - sides[2]) <= 0.1 || abs(sides[1] - sides[0]) <= 0.1 || abs(sides[2] - sides[0]) <= 0.1);
}

double distance(double x1, double y1, double x2, double y2) {
	return sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
}

bool CameraInterface::verifyTriangle(vector<Point> *vertices) {
	double areaNum = area(vertices);
	if (areaNum < 500 && areaNum > 100 && isReasonablyIsosceles(vertices)) {
		if (verbose) {
			cout << *vertices << endl;
			cout << "Found triangle of area: " << areaNum << endl;
		}

		if (controller->getCurrentPosition() < 0)
			cout << "Elevator on the left" << endl;
		else
			cout << "Elevator on the right" << endl;

		enum Direction dir = getDirection(vertices);
		cout << "Triangle pointing: ";
		switch (dir) {
		case DIR_UP : cout << "Up" << endl;
			break;
		case DIR_DOWN : cout << "Down" << endl;
			break;
		case DIR_UNKNOWN : cout << "Not enough info!" << endl;
		}

		return true;

	}
	return false;
}
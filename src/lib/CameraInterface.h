#ifndef CAMERA_INTERFACE_H
#define CAMERA_INTERFACE_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <cstdlib>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.h>
#include <cv_bridge/cv_bridge.h>
#include "CameraImage.h"
#include "CameraController.h"
#include <ctime>

#define H 0
#define S 1
#define V 2

enum hsv_type {
    HSV_STANDARD,
    HSV_255
};

enum Direction {
    DIR_UNKNOWN = -1,
    DIR_UP = 0,
    DIR_DOWN = 1
};

using namespace cv;
using namespace std;
using namespace ros;

class CameraInterface {
    CameraImage *lastImage;
    Scalar hsv_min, hsv_max;
    CameraController *controller;
    bool verbose;
    bool flip;
    int sigma;
    int tri[2];
    int found;
public:
    CameraInterface(Scalar ideal, Scalar range, CameraController *cont, int area[2], bool f, int strict, enum hsv_type type) {
        
        lastImage = NULL;
        generateMinMax(ideal, range, type);
        srand(time(NULL));
        controller = cont;

        tri[0] = area[0];
        tri[1] = area[1];

        sigma = 12;
        flip = true;

        verbose = false;
        found = 0;
    }

    ~CameraInterface() {
        if (lastImage != NULL)
            delete lastImage;
    }

    /**
     * Image callback for subscriber defined in main.
     */
    void image_callback(const sensor_msgs::Image::ConstPtr &msg);

    /**
     * Runs a number of processing steps on lastImage.
     */
    void process();

    /**
     * Generates the minimum and maximum scalar for thresholding.
     */
    void generateMinMax(Scalar ideal, Scalar range, enum hsv_type type);

    /**
     * Get direction of triangle, based vertically.
     */
    enum Direction getDirection(vector<Point> *vertices);

    /**
     * Verifies the triangle is suitable.
     */
    bool verifyTriangle(vector<Point> *vertices);

    /**
     * Returns whether the triangle is fully enclosed by certain bounds.
     */
    bool inRegion(vector<Point> *vertices);

    //Getters
    Scalar getMin() {
        return hsv_min;
    }
    Scalar getMax() {
        return hsv_max;
    }
};

#endif //CAMERA_INTERFACE_H

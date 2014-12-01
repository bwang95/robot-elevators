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
#include <ctime>

#define H 0
#define S 1
#define V 2

enum hsv_type {
    HSV_STANDARD,
    HSV_255
};

using namespace cv;
using namespace std;
using namespace ros;

class CameraInterface {
    CameraImage *lastImage;
    Scalar hsv_min, hsv_max;
public:
    CameraInterface(Scalar ideal, Scalar range, enum hsv_type type) {
        lastImage = NULL;
        generateMinMax(ideal, range, type);
        srand(time(NULL));
    }
    ~CameraInterface() {
        if (lastImage != NULL)
            delete lastImage;
    }
    void image_callback(const sensor_msgs::Image::ConstPtr &msg);
    void process();
    void generateMinMax(Scalar ideal, Scalar range, enum hsv_type type);
    Scalar getMin() {
        return hsv_min;
    }
    Scalar getMax() {
        return hsv_max;
    }
};

#endif //CAMERA_INTERFACE_H
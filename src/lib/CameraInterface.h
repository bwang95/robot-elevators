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

using namespace cv;
using namespace std;
using namespace ros;

class CameraInterface {
    CameraImage *lastImage;
public:
    CameraInterface() {
        lastImage = NULL;
    }
    ~CameraInterface() {
        if (lastImage != NULL)
            delete lastImage;
    }
    void image_callback(const sensor_msgs::Image::ConstPtr &msg);
    void process();
};

#endif //CAMERA_INTERFACE_H
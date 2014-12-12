#ifndef CAMERA_IMAGE_H
#define CAMERA_IMAGE_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;

struct CameraImage {
    int width;
    int height;
    Mat image;

    CameraImage(const sensor_msgs::Image::ConstPtr &msg):
        image(cv_bridge::toCvCopy(msg,
                                  sensor_msgs::image_encodings::TYPE_8UC3)->image),
        width(msg->width),
        height(msg->height) {
    }
};

#endif //CAMERA_IMAGE_H
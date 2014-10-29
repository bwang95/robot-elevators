#include "lib/CameraInterface.h"

void CameraInterface::image_callback(
    const sensor_msgs::Image::ConstPtr &msg) {
    CameraImage *ptr = lastImage;
    lastImage = new CameraImage(msg->data, msg->step, msg->height);
    process();
    delete ptr;
}

void CameraInterface::process() {
	ROS_INFO("Processing image with dimensions %d x %d\n", 
		lastImage->width, lastImage->height);
}
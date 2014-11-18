#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include "lib/CameraInterface.h"
#include "lib/SoundInterface.h"

using namespace std;
using namespace ros;

static int current_floor = 3;

int main(int argc, char **argv) {
	init(argc, argv, "elevator_listener");
	
	NodeHandle node;

	SoundInterface sinter;
	CameraInterface cinter;

	Subscriber listener = node.subscribe("/fft_topic", 100, &SoundInterface::fft_callback, &sinter);
	Subscriber image = node.subscribe(argc > 1 ? argv[1] : "camera/image_raw", 100, &CameraInterface::image_callback, &cinter);

	spin();

	return 0;
}
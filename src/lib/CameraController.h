#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <ros/ros.h>
#include <unistd.h>

#define SLEEP_TIME (10 * 1000)

using namespace std;
using namespace ros;

class CameraController {
	float currentPosition;
	Publisher *publisher;
public:
	CameraController(Publisher *p){
		publisher = p;
		moveTo(0);
	}
	void moveTo(float f);
	float getCurrentPosition(){
		return currentPosition;
	}
};

#endif //CAMERA_CONTROLLER_H
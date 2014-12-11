#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Float32.h>

#define SLEEP_TIME 1500

using namespace std;
using namespace ros;

class CameraController {
	float currentPosition;
	Publisher *publisher;
  int counter;
public:
	CameraController(Publisher *p){
		publisher = p;
		moveTo(0);
    counter = 0;
	}
	void moveTo(float f);
	void wait(int ms);
	void alternate();
  void status_callback(const std_msgs::Float32::ConstPtr &msg);
 
	float getCurrentPosition(){
		return currentPosition;
	}
};

#endif //CAMERA_CONTROLLER_H

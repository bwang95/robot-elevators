#include "lib/CameraController.h"

void CameraController::moveTo(float f){
	std_msgs::Float32 msg;
	msg.data = f;
	currentPosition = f;

	publisher->publish(msg);

	wait(SLEEP_TIME);
}

void CameraController::wait(int ms){
	usleep(1000 * ms);
}

void CameraController::alternate(){
	if(currentPosition >= 0)
		moveTo(-1);
	else
		moveTo(1);
}

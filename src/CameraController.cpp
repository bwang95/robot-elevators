#include "lib/CameraController.h"

void CameraController::moveTo(float f){
	msg;
	msg->data = f;

	publisher->publish(msg);

	usleep(SLEEP_TIME);
}
#include "lib/CameraController.h"

void CameraController::moveTo(float f) {
    std_msgs::Float32 msg;
    msg.data = f;
    
    publisher->publish(msg);
}

void CameraController::wait(int ms) {
    usleep(1000 * ms);
}

void CameraController::alternate() {
    if (currentPosition >= 0)
        moveTo(-0.5);
    else
        moveTo(0.5);
}

void CameraController::status_callback(const std_msgs::Float32::ConstPtr &msg) {
    currentPosition = msg->data;
    if (currentPosition >= 0.45 || currentPosition <= -0.45)
        counter++;
    if (counter >= 60) {
        counter = 0;
        alternate();
    }
}

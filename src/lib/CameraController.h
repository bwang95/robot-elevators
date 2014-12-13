#ifndef CAMERA_CONTROLLER_H
#define CAMERA_CONTROLLER_H

#include <ros/ros.h>
#include <unistd.h>
#include <std_msgs/Float32.h>

using namespace std;
using namespace ros;

class CameraController {
    //Current position of camera. Governed by servo itself.
    float currentPosition;

    //Publisher to publish destination messages to.
    Publisher *publisher;

    //Counter for how long servo has been at destination.
    int counter;

    //Range of servo movement.
    double range[2];
public:
    //Initialize camera and reset servo to position 0.
    CameraController(Publisher *p) {
        publisher = p;
        moveTo(0);
        counter = 0;

        range[0] = -0.5;
        range[1] = 0.5;
    }

    /**
     * Sends a move command. This method is asynchronous,
     * and should be verified by polling getCurrentPosition.
     */
    void moveTo(float f);

    /**
     * Waits *ms* amount of milliseconds. Puts thread to sleep.
     */
    void wait(int ms);

    /**
     * Alternates the servo back and forth.
     */
    void alternate();

    /**
     * Status callback for Subscriber defined in main.cpp
     */
    void status_callback(const std_msgs::Float32::ConstPtr &msg);

    /**
     * Returns the current position.
     */
    float getCurrentPosition() {
        return currentPosition;
    }
};

#endif //CAMERA_CONTROLLER_H

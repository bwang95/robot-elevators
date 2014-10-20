#ifndef SOUND_INTERFACE_H
#define SOUND_INTERFACE_H

#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include "elevators/fft_col.h"

using namespace std;
using namespace ros;

class SoundInterface {
	vector<double> *lastData;
public:
    SoundInterface() {
    	lastData = NULL;
    }
    ~SoundInterface() {
    	if(lastData != NULL)
    		delete lastData;
    }
    void fft_callback(const elevators::fft_col::ConstPtr &msg);
};

#endif //SOUND_INTERFACE_H
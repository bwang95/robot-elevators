#ifndef SOUND_INTERFACE_H
#define SOUND_INTERFACE_H

#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include "elevators/fft_col.h"

using namespace std;
using namespace ros;

class SoundInterface {
public:
    SoundInterface() {}
    void fft_callback(const elevators::fft_col::ConstPtr &msg);
    void process_audio(vector<double> data);
};

#endif //SOUND_INTERFACE_H
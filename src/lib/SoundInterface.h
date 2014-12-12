#ifndef SOUND_INTERFACE_H
#define SOUND_INTERFACE_H

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include "elevators/fft_col.h"
#include <dirent.h>

#define PEAK_MIN 2

using namespace std;
using namespace ros;

class SoundInterface {
    vector< vector<float> * > sound_db;
    int *peaks;
public:
    SoundInterface() {
        load_ref_data();
        peaks = NULL;
    }
    ~SoundInterface() {
        if (peaks != NULL)
            delete peaks;
        while (sound_db.size() > 0) {
            vector <float> *ptr = sound_db.back();
            sound_db.erase(sound_db.end());
            delete ptr;
        }
    }
    void fft_callback(const elevators::fft_col::ConstPtr &msg);
    void process_audio(vector<double> data);
private:
    void load_ref_data();
    int match_audio(vector<double> data);
    int peak(int index, vector<double> *dataptr);
};

#endif //SOUND_INTERFACE_H
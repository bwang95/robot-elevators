#include "lib/SoundInterface.h"

void SoundInterface::fft_callback(const elevators::fft_col::ConstPtr &msg){
	vector<double> data = msg->data;
	lastData = &data;
}
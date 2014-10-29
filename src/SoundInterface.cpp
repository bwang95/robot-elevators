#include "lib/SoundInterface.h"

void SoundInterface::fft_callback(const elevators::fft_col::ConstPtr &msg){
	vector<double> data = msg->data;
	process_audio(data);
}

void SoundInterface::process_audio(std::vector<double> data){
	ROS_INFO("Processing audio data with length %lu", data.size());
}
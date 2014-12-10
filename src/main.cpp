#include <cstdlib>
#include <iostream>
#include <ros/ros.h>
#include <string>
#include <fstream>
#include "lib/CameraInterface.h"
#include "lib/SoundInterface.h"
#include "lib/CameraController.h"
#include <std_msgs/Float32.h>

using namespace std;
using namespace ros;

static int current_floor = 3;

struct config
{
  int ideal[3];
  int range[3];
  string topic;
};

void parse(ifstream *, struct config *);

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    cout << "Usage: <config file>" << endl;
    return EXIT_FAILURE;
  }

  ifstream config_file (argv[1]);
  if (!config_file.is_open())
  {
    cout << "Config file not found!" << endl;
    return EXIT_FAILURE;
  }

  struct config conf;
  parse(&config_file, &conf);

  init(argc, argv, "elevator_listener");

  NodeHandle node;

  Scalar ideal = Scalar(conf.ideal[0], conf.ideal[1], conf.ideal[2]);
  Scalar range = Scalar(conf.range[0], conf.range[1], conf.range[2]);

  SoundInterface sinter;
  CameraInterface cinter(ideal, range, HSV_STANDARD);

  cout << cinter.getMin() << endl;
  cout << cinter.getMax() << endl;

  Subscriber listener = node.subscribe("/fft_topic", 100, &SoundInterface::fft_callback, &sinter);
  Subscriber image = node.subscribe(conf.topic, 100, &CameraInterface::image_callback, &cinter);

  Publisher cpublisherh = node.advertise<std_msgs::Float32>("/servo0_cmd", 1000);
  //Publisher cpublisherv = node.advertise<std_msgs::Float32>("camera/servo1", 1000);

  //std_msgs::Float32 msg;
  //msg.data = 1;
  //cpublisherv.publish(msg);
  //TODO Accelerometer

  CameraController camera(&cpublisherh);
  camera.wait(1000);

  while(ok()){
    spinOnce();
    camera.wait(100);
    camera.alternate();
  }

  return 0;
}

void parse(ifstream *file, struct config *configuration)
{
  string line;
  bool done = false;
  while (getline(*file, line, '='))
  {
    if (line == "image_topic")
      getline(*file, configuration->topic);
    else if (line == "ideal"){
        for(int k = 0; k < 2; k++){
            getline(*file, line, ' ');
            configuration->ideal[k] = atoi(line.c_str());
        }
        getline(*file, line);
        configuration->ideal[2] = atoi(line.c_str());
    }
    else if (line == "range"){
        for(int k = 0; k < 2; k++){
            getline(*file, line, ' ');
            configuration->range[k] = atoi(line.c_str());
        }
        getline(*file, line);
        configuration->range[2] = atoi(line.c_str());
    }
  }
}

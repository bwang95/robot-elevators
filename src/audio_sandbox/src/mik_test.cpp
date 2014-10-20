#include <cstdio>
#include <string.h>
#include <iostream>
#include <signal.h>
#include <portaudio.h>
#include <boost/thread/thread.hpp>

const static int SAMPLE_RATE = 44100; // todo: make this a parameter.

bool g_caught_sigint = false;

int main(int argc, char **argv)
{

/* INITIALIZING PORTAUDIO */
  PaStream *stream;
  PaError err;
  if (Pa_Initialize() != paNoError)
  {
    printf("unable to initialize portaudio\n");
    //ROS_BREAK();
  }
  
}

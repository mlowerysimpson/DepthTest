#include "HelixDepth.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <memory>

//example program that lets you test out the depth output of a Helix fish finder
#define NUM_SAMPLES 10

using namespace std;
static char * HELIX_COM_PORT = "/dev/ttyUSBPort2";//serial port used for communications with the fish finder

int main(int argc, char * argv[])
{
    HelixDepth fishDepth(HELIX_COM_PORT);
    for (int i = 0; i < NUM_SAMPLES; i++) {
        usleep(1000000);
        double dDepth = fishDepth.GetDepth();
        double dTemperature = fishDepth.GetTemperature();
        printf("Depth = %.1f m, Temperature = %.1f deg C\n", dDepth, dTemperature);
    }
    return 0 ;
}

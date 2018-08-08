/*
   PositionEstimation.ino : Arduino sketch to perform altitude estimation using
   the provided library
 */

// Drivers for the sensors
#include "Bitcraze_PMW3901.h"

#include <Arduino.h>

#include "position.h"

// Define variables and create objects
float Q[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
PositionEstimator position = PositionEstimator(1.0f, R, Q);
// Using digital pin 10 for chip select
Bitcraze_PMW3901 flow(10);

void setup(void)
{
    Serial.begin(9600);
    if (!flow.begin())
    {
      Serial.println("Initialization of the flow sensor failed");
    }
    position.reset();
}

void loop(void)
{

}

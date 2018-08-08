/*
   PositionEstimation.ino : Arduino sketch to perform altitude estimation using
   the provided library
 */

#include <Arduino.h>

#include "position.h"

float Q[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
float R[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

PositionEstimator position = PositionEstimator(1.0f, R, Q);

void setup(void)
{
    position.reset();
}

void loop(void)
{

}

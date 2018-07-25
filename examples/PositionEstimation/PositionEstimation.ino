/*
   PositionEstimation.ino : Arduino sketch to perform altitude estimation using
   the provided library
 */

#include <Arduino.h>

#include "position.h"

PositionEstimator position = PositionEstimator(1.0f);

void setup(void)
{
    position.begin();
}

void loop(void)
{

}

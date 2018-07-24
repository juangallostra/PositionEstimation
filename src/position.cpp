/*
    position.cpp: Position estimation from optical flow
*/

# include "position.h"

# include <Arduino.h> // XXX For micros; eventually need to compute micros() elsewhere

PositionEstimator::PositionEstimator(float KOpticalFlow)
{
    this->KOpticalFlow = KOpticalFlow;
}

void PositionEstimator::angularCompensation(float velocity[2], float flow[2], float gyro[3],
                                            float height, float currentTime)
{
    float K = KOpticalFlow * height;
    float deltaT = (currentTime - lastUpdateTime) / 1000000.0f;
    velocity[0] = K * (-flow[0]) / deltaT + height * (-gyro[1]);
    velocity[1] = K * (-flow[1]) / deltaT + height * gyro[0];
}

void PositionEstimator::begin()
{
    lastUpdateTime = micros();
}

void PositionEstimator::estimate()
{

}

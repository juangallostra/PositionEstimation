/*
    position.cpp: Position estimation from optical flow
*/

# include "position.h"

# include <Arduino.h> // XXX For micros; eventually need to compute micros() elsewhere

PositionEstimator::PositionEstimator(float KOpticalFlow)
{
    this->KOpticalFlow = KOpticalFlow;
}

void PositionEstimator::angularCompensation(float flow[2], float gyro[3],
                                            float height, uint32_t currentTime)
{
    float K = KOpticalFlow * height;
    float deltaT = (currentTime - lastUpdateTime) / 1000000.0f;
    xVel = K * (-flow[0]) / deltaT + height * (-gyro[1]);
    yVel = K * (-flow[1]) / deltaT + height * gyro[0];
}

void PositionEstimator::reset()
{
    // reset time
    lastUpdateTime = micros();
    // reset estimated velocity
    xVel = 0;
    yVel = 0;
    // reset estimated position
    xPos = 0;
    yPos = 0;

}

void PositionEstimator::estimate()
{

}

void PositionEstimator::getEstimatedVelocity(float velocity[2])
{
    velocity[0] = xVel;
    velocity[1] = yVel;
}

void PositionEstimator::getEstimatedPosition(float position[2])
{
    position[0] = xPos;
    position[1] = yPos;
}

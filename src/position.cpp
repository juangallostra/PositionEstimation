/*
    position.cpp: Position estimation from optical flow
*/

# include "position.h"

PositionEstimator::PositionEstimator(float KOpticalFlow)
{
  this->KOpticalFlow = KOpticalFlow;
}

void PositionEstimator::angularCompensation(float[2] flow, float[3] gyro, float height)
{

}

void PositionEstimator::estimate()
{

}

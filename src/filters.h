/*
   filters.h: Filter class declarations
 */

#pragma once

#include <cmath>
#include <stdint.h>

#include "algebra.h"

class KalmanFilter {
  private:
    float currentState[3] = {0, 0, 0};
    float currErrorCovariance[3][3] = {{100, 0, 0},{0, 100, 0},{0, 0, 100}};
    float H[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float previousAccelSensor[3] = {0, 0, 0};

    void predictState(float predictedState[3], float gyro[3], float deltat);

    void predictErrorCovariance(float covariance[3][3], float gyro[3], float deltat);

    void updateGain(float gain[3][3], float errorCovariance[3][3]);

    void updateState(float updatedState[3], float predictedState[3], float gain[3][3], float accel[3]);

    void updateErrorCovariance(float covariance[3][3], float errorCovariance[3][3], float gain[3][3]);

  public:

    KalmanFilter(float ca, float sigmaGyro, float sigmaAccel);

    float estimate(float gyro[3], float accel[3], float deltat);

}; // Class KalmanFilter

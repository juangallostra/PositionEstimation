/*
    position.h: Position estimation from optical flow
*/

# pragma once

# include <stdint.h>

#include "filters.h"
#include "algebra.h"

namespace pe {

  class PositionEstimator {

      private:
          // Estimator parameters
          float KOpticalFlow;
          uint32_t lastUpdateTime;
          KalmanFilter kalman;

          // Estimated values
          float xVel;
          float yVel;

          void angularCompensation(float velocities[2], float flow[2], float gyro[3], float height, float deltaT);

      public:

          PositionEstimator(float KOpticalFlow, float R[3][3], float Q[3][3]);

          void reset();

          void estimate(float flow[2], float gyro[3], float accel[3], float height, uint32_t currentTime);

          float getEstimatedXVelocity();

          float getEstimatedYVelocity();

  }; // class PositionEstimator

} // namespace pe

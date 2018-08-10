/*
    position.cpp: Position estimation from optical flow
*/

# include <cmath>
# include <Arduino.h> // XXX For micros; eventually need to compute micros() elsewhere

# include "position.h"
# include "filters.h"

namespace pe {

  PositionEstimator::PositionEstimator(float KOpticalFlow, float R[3][3], float Q[3][3])
  :kalman(R, Q)
  {
      this->KOpticalFlow = KOpticalFlow;
  }

  void PositionEstimator::angularCompensation(float velocities[2], float flow[2],
                                              float gyro[3], float height, float deltaT)
  {
      float K = KOpticalFlow * height;
      velocities[0] = K * (-flow[0]) / deltaT + height * (-gyro[1]);
      velocities[1] = K * (-flow[1]) / deltaT + height * gyro[0];
  }

  void PositionEstimator::reset()
  {
      // reset time
      lastUpdateTime = micros();
      // reset estimated velocity
      xVel = 0;
      yVel = 0;
  }

  void PositionEstimator::estimate(float flow[2], float gyro[3], float accel[3],
                                   float height, uint32_t currentTime)
  {
    float deltaT = (currentTime - lastUpdateTime) / 1000000.0f;
    // Perform angular compensation
    float velocities[2];
    PositionEstimator::angularCompensation(velocities, flow, gyro, height, deltaT);
    // estimate via kalman filter
    float velocityEstimate[3];
    kalman.estimate(velocityEstimate, gyro, accel, velocities, deltaT);
    // Store estimated values
    xVel = velocityEstimate[0];
    yVel = velocityEstimate[1];
    lastUpdateTime = currentTime;
  }

  float PositionEstimator::getEstimatedXVelocity()
  {
      return xVel;
  }
  float PositionEstimator::getEstimatedYVelocity()
  {
      return yVel;
  }

}

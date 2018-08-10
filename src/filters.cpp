/*
   filters.cpp: Filter class implementations
 */

#include <cmath>
#include <stdlib.h> // XXX eventually use fabs() instead of abs() ?

#include "filters.h"

namespace pe {

  KalmanFilter::KalmanFilter(float R[3][3], float Q[3][3])
  {
    copyMatrix3x3(this->R, R);
    copyMatrix3x3(this->Q, Q);
  }

  void KalmanFilter::predictState(float predictedState[3], float gyro[3], float deltat)
  {
      // helper matrices
      float identity[3][3];
      identityMatrix3x3(identity);
      float skewFromGyro[3][3];
      skew(skewFromGyro, gyro);
      // Predict state
      scaleAndAccumulateMatrix3x3(identity, -deltat, skewFromGyro);
      matrixDotVector3x3(predictedState, identity, currentState);
  }

  void KalmanFilter::predictErrorCovariance(float covariance[3][3], float gyro[3], float deltat)
  {
      // required matrices
      float identity[3][3];
      identityMatrix3x3(identity);
      float skewFromGyro[3][3];
      skew(skewFromGyro, gyro);
      float tmp[3][3];
      float tmpTransposed[3][3];
      float tmp2[3][3];
      // predict error covariance
      scaleAndAccumulateMatrix3x3(identity, -deltat, skewFromGyro);
      copyMatrix3x3(tmp, identity);
      transposeMatrix3x3(tmpTransposed, tmp);
      matrixProduct3x3(tmp2, tmp, currErrorCovariance);
      matrixProduct3x3(covariance, tmp2, tmpTransposed);
      scaleAndAccumulateMatrix3x3(covariance, 1.0, Q);
  }

  void KalmanFilter::updateGain(float gain[3][3], float errorCovariance[3][3])
  {
      // required matrices
      float HTransposed[3][3];
      transposeMatrix3x3(HTransposed, H);
      float tmp[3][3];
      float tmp2[3][3];
      float tmp2Inverse[3][3];
      // update kalman gain
      // P.dot(H.T).dot(inv(H.dot(P).dot(H.T) + R))
      matrixProduct3x3(tmp, errorCovariance, HTransposed);
      matrixProduct3x3(tmp2, H, tmp);
      scaleAndAccumulateMatrix3x3(tmp2, 1.0, R);
      invert3x3(tmp2Inverse, tmp2);
      matrixProduct3x3(gain, tmp, tmp2Inverse);
  }

  void KalmanFilter::updateState(float updatedState[3], float predictedState[3], float gain[3][3], float measurement[3])
  {
      // required matrices
      float tmp[3];
      float tmp2[3];
      // update state with measurement
      // predicted_state + K.dot(measurement - H.dot(predicted_state))
      matrixDotVector3x3(tmp, H, predictedState);
      subtractVectors(tmp, measurement, tmp);
      matrixDotVector3x3(tmp2, gain, tmp);
      sumVectors(updatedState, predictedState, tmp2);
      normalizeVector(updatedState);
  }

  void KalmanFilter::updateErrorCovariance(float covariance[3][3], float errorCovariance[3][3], float gain[3][3])
  {
      // required matrices
      float identity[3][3];
      identityMatrix3x3(identity);
      float tmp[3][3];
      float tmp2[3][3];
      // update error covariance with measurement
      matrixProduct3x3(tmp, gain, H);
      matrixProduct3x3(tmp2, tmp, errorCovariance);
      scaleAndAccumulateMatrix3x3(identity, -1.0, tmp2);
      copyMatrix3x3(covariance, tmp2);
  }

  float KalmanFilter::estimate(float velocityEstimate[3], float gyro[3], float accel[3], float VelFromFlow[2], float deltat)
  {
      float predictedState[3];
      float updatedState[3];
      float errorCovariance[3][3];
      float updatedErrorCovariance[3][3];
      float gain[3][3];
      float measurement[3] = {VelFromFlow[0], VelFromFlow[1], 0};
      scaleVector(accel, 9.81, accel); // Scale accel readings since they are measured in gs
      // perform estimation
      predictState(predictedState, gyro, deltat);
      predictErrorCovariance(errorCovariance, gyro, deltat);
      updateGain(gain, errorCovariance);
      updateState(updatedState, predictedState, gain, measurement);
      updateErrorCovariance(updatedErrorCovariance, errorCovariance, gain);
      // Store required values for next iteration
      copyVector(currentState, updatedState);
      copyMatrix3x3(currErrorCovariance, updatedErrorCovariance);
      // return translational velocity estimate
      copyVector(velocityEstimate, currentState);

  }

} // namespace pe

/*
    position.h: Position estimation from optical flow
*/

# pragma once

class PositionEstimator {

    private:
        // Estimator parameters
        float KOpticalFlow;

        void angularCompensation(float[2] flow, float[3] gyro, float height);

    public:

        PositionEstimator(float KOpticalFlow);

        void estimate();

} // class PositionEstimator

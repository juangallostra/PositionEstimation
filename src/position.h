/*
    position.h: Position estimation from optical flow
*/

# pragma once

class PositionEstimator {

    private:
        // Estimator parameters
        float KOpticalFlow;

        float lastUpdateTime;

        void angularCompensation(float velocity[2], float flow[2], float gyro[3],
                                 float height, float currentTime);

    public:

        PositionEstimator(float KOpticalFlow);

        void begin();

        void estimate();

}; // class PositionEstimator

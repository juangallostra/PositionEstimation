/*
    position.h: Position estimation from optical flow
*/

# pragma once

class PositionEstimator {

    private:
        // Estimator parameters
        float KOpticalFlow;
        float lastUpdateTime;

        // Estimated values
        float xVel;
        float yVel;
        float xPos;
        float yPos;

        void angularCompensation(float flow[2], float gyro[3],
                                 float height, float currentTime);

    public:

        PositionEstimator(float KOpticalFlow);

        void reset();

        void estimate();

        void getEstimatedVelocity(float velocity[2]);

        void getEstimatedPosition(float position[2]);

}; // class PositionEstimator

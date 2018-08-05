/*
    position.h: Position estimation from optical flow
*/

# pragma once

# include <stdint.h>

class PositionEstimator {

    private:
        // Estimator parameters
        float KOpticalFlow;
        uint32_t lastUpdateTime;

        // Estimated values
        float xVel;
        float yVel;
        float xVelPast;
        float yVelPast;
        float xPos;
        float yPos;

        void angularCompensation(float flow[2], float gyro[3],
                                 float height, float deltaT);

    public:

        PositionEstimator(float KOpticalFlow);

        void reset();

        void estimate(float flow[2], float gyro[3],
                      float height, uint32_t currentTime);

        void getEstimatedVelocity(float velocity[2]);

        void getEstimatedPosition(float position[2]);

}; // class PositionEstimator

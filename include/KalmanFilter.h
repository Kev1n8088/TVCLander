#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#include <Arduino.h>

class KalmanFilter {
private:
    // State vectors
    float x[3]; // State vector [position, velocity, acceleration]
    float P[3][3]; // Error covariance matrix 
    float Q[3][3]; // Process noise covariance matrix
    float R_accel; // Measurement noise covariance for acceleration
    float R_gps_pos; // Measurement noise covariance for GPS position
    float R_gps_vel; // Measurement noise covariance for GPS velocity

    float q_pos;
    float q_vel;
    float q_accel;

    uint64_t lastUpdateMicros; // Last update time in microseconds
    bool initialized; // Flag to check if the filter is initialized

    // Basic matrix operations
    void matrixMultiply3x3(float A[3][3], float B[3][3], float result[3][3]);
    void matrixAdd3x3(float A[3][3], float B[3][3], float result[3][3]);
    void matrixSubtract3x3(float A[3][3], float B[3][3], float result[3][3]);
    void matrixTranspose3x1(float A[3], float result[3]);
    void matrixInvert2x2(float A[2][2], float result[2][2]);
    void matrixInvert1x1(float A, float &result);

    // Kalman filter operations
    void predict(float dt);
    void updateWithAccel(float accel_measurement);
    void updateWithGPS(float pos_measurement, float vel_measurement);

public:
    KalmanFilter();

    void begin(float initial_pos = 0.0, float initial_vel = 0.0, float initial_accel = 0.0);

    // setup
    void setProcessNoise(float pos_noise, float vel_noise, float accel_noise);

    // update
    void updateAccelerometer(float acceleration);
    void updateGPS(float position, float velocity);

    // get state
    float getPosition();
    float getVelocity();
    float getAcceleration();

    // Get uncertainty estimates (standard deviation)
    float getPositionUncertainty();
    float getVelocityUncertainty();
    float getAccelerationUncertainty();

    //set noises
    void setAccelNoise(float accel_noise) {
        R_accel = accel_noise;
    }
    void setGPSPosNoise(float pos_noise) {
        R_gps_pos = pos_noise;
    }
    void setGPSVelNoise(float vel_noise) {
        R_gps_vel = vel_noise;
    }

    // Reset the filter
    void reset();

};

#endif // KALMANFILTER_H


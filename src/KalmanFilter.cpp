#include "KalmanFilter.h"
#include <Arduino.h>

KalmanFilter::KalmanFilter(){
    initialized = false;
    lastUpdateMicros = 0;

    // Default noise parameters
    R_accel = 0.5;    // Accelerometer variance (m/s^2)^2
    R_gps_pos = 0.0006;  // GPS position variance (m)^2
    R_gps_vel = 0.001;  // GPS velocity variance (m/s)^2
    
    // Initialize matrices to zero
    for(int i = 0; i < 3; i++) {
        x[i] = 0.0;
        for(int j = 0; j < 3; j++) {
            P[i][j] = 0.0;
            Q[i][j] = 0.0;
        }
    }
}


void KalmanFilter::begin(float initial_pos, float initial_vel, float initial_accel) {
    // Initialize state vector
    x[0] = initial_pos;      // Position
    x[1] = initial_vel;      // Velocity
    x[2] = initial_accel;    // Acceleration

    // Initialize error covariance matrix with high uncertainty
    P[0][0] = 100.0;  // Position uncertainty
    P[1][1] = 10.0;   // Velocity uncertainty
    P[2][2] = 1.0;    // Acceleration uncertainty
    
    // Set default process noise
    setProcessNoise(0.02, 0.1, 0.5);
    
    lastUpdateMicros = micros();
    initialized = true;
}

void KalmanFilter::setProcessNoise(float pos_noise, float vel_noise, float accel_noise) {
    // Process noise covariance matrix
    Q[0][0] = pos_noise * pos_noise;
    Q[1][1] = vel_noise * vel_noise;
    Q[2][2] = accel_noise * accel_noise;
}

void KalmanFilter::predict(float dt) {
    // State transition model: x_k = F * x_{k-1}
    // F = [1, dt, 0.5*dt^2]
    //     [0, 1,  dt     ]
    //     [0, 0,  1      ]
    
    float dt2 = dt * dt;
    
    // Update state estimate
    float new_x[3];
    new_x[0] = x[0] + x[1] * dt + 0.5 * x[2] * dt2;  // position
    new_x[1] = x[1] + x[2] * dt;                      // velocity
    new_x[2] = x[2];                                  // acceleration
    
    // Copy back to state vector
    for(int i = 0; i < 3; i++) {
        x[i] = new_x[i];
    }
    
    // Update error covariance: P = F * P * F^T + Q
    float F[3][3] = {
        {1.0, dt, 0.5 * dt2},
        {0.0, 1.0, dt},
        {0.0, 0.0, 1.0}
    };
    
    float FT[3][3] = {
        {1.0, 0.0, 0.0},
        {dt, 1.0, 0.0},
        {0.5 * dt2, dt, 1.0}
    };
    
    float temp[3][3];
    matrixMultiply3x3(F, P, temp);
    matrixMultiply3x3(temp, FT, P);
    matrixAdd3x3(P, Q, P);
}

void KalmanFilter::updateWithAccel(float accel_measurement) {
    // Measurement model: z = H * x
    // H = [0, 0, 1] accel observed directly (world frame)
    
    // Innovation: y = z - H * x
    float innovation = accel_measurement - x[2];
    
    // Innovation covariance: S = H * P * H^T + R
    float S = P[2][2] + R_accel;
    
    // Kalman gain: K = P * H^T * S^-1
    float K[3];
    K[0] = P[0][2] / S;
    K[1] = P[1][2] / S;
    K[2] = P[2][2] / S;
    
    // Update state estimate: x = x + K * y
    x[0] += K[0] * innovation;
    x[1] += K[1] * innovation;
    x[2] += K[2] * innovation;
    
    // Update error covariance: P = (I - K * H) * P
    float temp_P[3][3];
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            temp_P[i][j] = P[i][j] - K[i] * P[2][j];
        }
    }
    
    // Copy back to P
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            P[i][j] = temp_P[i][j];
        }
    }
}

void KalmanFilter::updateWithGPS(float pos_measurement, float vel_measurement) {
    // Combined GPS update using both position and velocity measurements
    // Measurement model: z = H * x
    // H = [1, 0, 0]  for position
    //     [0, 1, 0]  for velocity
    
    // Innovation vector: y = z - H * x
    float innovation[2];
    innovation[0] = pos_measurement - x[0];  // position innovation
    innovation[1] = vel_measurement - x[1];  // velocity innovation
    
    // Measurement matrix H (2x3)
    float H[2][3] = {
        {1.0, 0.0, 0.0},  // position measurement
        {0.0, 1.0, 0.0}   // velocity measurement
    };
    
    // Innovation covariance: S = H * P * H^T + R
    float S[2][2];
    S[0][0] = P[0][0] + R_gps_pos;              // position variance
    S[0][1] = P[0][1];                          // position-velocity covariance
    S[1][0] = P[1][0];                          // velocity-position covariance
    S[1][1] = P[1][1] + R_gps_vel;              // velocity variance
    
    // Invert S (2x2 matrix inversion)
    float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
    if (abs(det) < 1e-6) return;  // Singular matrix, skip update
    
    float S_inv[2][2];
    S_inv[0][0] = S[1][1] / det;
    S_inv[0][1] = -S[0][1] / det;
    S_inv[1][0] = -S[1][0] / det;
    S_inv[1][1] = S[0][0] / det;
    
    // Kalman gain: K = P * H^T * S^-1 (3x2 matrix)
    float K[3][2];
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 2; j++) {
            K[i][j] = 0;
            for(int k = 0; k < 2; k++) {
                K[i][j] += P[i][k] * S_inv[k][j];
            }
        }
    }
    
    // Update state estimate: x = x + K * y
    for(int i = 0; i < 3; i++) {
        x[i] += K[i][0] * innovation[0] + K[i][1] * innovation[1];
    }
    
    // Update error covariance: P = P - K * H * P
    float temp_P[3][3];
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            temp_P[i][j] = P[i][j] - (K[i][0] * P[0][j] + K[i][1] * P[1][j]);
        }
    }
    
    // Copy back to P
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            P[i][j] = temp_P[i][j];
        }
    }
}

void KalmanFilter::updateAccelerometer(float acceleration) {
    if (!initialized) return;
    
    uint64_t currentTime = micros();
    float dt = (currentTime - lastUpdateMicros) / 1000000.0;  // Convert to seconds
    
    updateWithAccel(acceleration);
    if (dt > 0.001) {  // Minimum time step of 1ms
        predict(dt);
        lastUpdateMicros = currentTime;
    }
}

void KalmanFilter::updateGPS(float position, float velocity) {
    if (!initialized) return;
    
    uint64_t currentTime = micros();
    float dt = (currentTime - lastUpdateMicros) / 1000000.0;  // Convert to seconds
    
    updateWithGPS(position, velocity);
    
    if (dt > 0.001) {  // Minimum time step of 1ms
        predict(dt);
        lastUpdateMicros = currentTime;
    }
}

float KalmanFilter::getPosition() {
    return x[0];
}

float KalmanFilter::getVelocity() {
    return x[1];
}

float KalmanFilter::getAcceleration() {
    return x[2];
}

float KalmanFilter::getPositionUncertainty() {
    return sqrt(P[0][0]);
}

float KalmanFilter::getVelocityUncertainty() {
    return sqrt(P[1][1]);
}

float KalmanFilter::getAccelerationUncertainty() {
    return sqrt(P[2][2]);
}


void KalmanFilter::reset() {
    initialized = false;
    lastUpdateMicros = 0;
    
    for(int i = 0; i < 3; i++) {
        x[i] = 0.0;
        for(int j = 0; j < 3; j++) {
            P[i][j] = 0.0;
        }
    }
}

// Matrix operation implementations
void KalmanFilter::matrixMultiply3x3(float A[3][3], float B[3][3], float result[3][3]) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            result[i][j] = 0;
            for(int k = 0; k < 3; k++) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void KalmanFilter::matrixAdd3x3(float A[3][3], float B[3][3], float result[3][3]) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}

void KalmanFilter::matrixSubtract3x3(float A[3][3], float B[3][3], float result[3][3]) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            result[i][j] = A[i][j] - B[i][j];
        }
    }
}
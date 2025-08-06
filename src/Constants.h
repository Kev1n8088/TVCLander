#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>

//SCH16T constants
#define IMU0_RST_PIN A7 // Define the reset pin for SCH1
#define IMU0_CS_PIN A8 
#define IMU0_DRY_PIN A6

//Neopixel constants
#define RGB_PIN A9 
#define NUMPIXELS 2

//ICM456xx constants
#define IMU1_CS_PIN 4

//BMP390 constants
#define BMP_CS A2
#define SEALEVELPRESSURE_HPA (1013.25)
#define BARO_ADJUST_N 10 //number of samples for baro adjustment
#define BARO_ADJUST_INTERVAL 500 // milliseconds between baro samples

//LIS2MDL constants
#define LIS2MDL_CS A14

//Servo Constants
#define SRV0 30
#define SRV1 29
#define SRV2 25
#define SRV3 24
#define SRV4 33
#define SRV5 36
#define SRV6 37

// Gimbal Pins
#define YAW_SERVO SRV0
#define PITCH_SERVO SRV1

// Roll motor controller pins
#define ROLL_MINUS SERV5
#define ROLL_PLUS SERV6

#define LAND_PYRO 32
#define LAND_CONTINUITY 26

#define WORLD_GRAVITY_X -9.81f
#define WORLD_GRAVITY_Y -0.0f
#define WORLD_GRAVITY_Z -0.0f

#define PRELAUNCH_UPDATE_INTERVAL 3000 // Update interval in milliseconds before launch
#define PRELAUNCH_AVERAGE_COUNT 10 // Number of samples to average IMU before launch
#define PRELAUNCH_AVERAGE_INTERVAL 30 // Interval in milliseconds between samples before launch

#define STATE_ESTIMATION_INTERVAL_US 500 // Interval in microseconds for state estimation updates

#define TELEMETRY_BAUD 57600 // Baud rate for telemetry communication
#define DEBUG_BAUD 115200 // Baud rate for debug communication
#define DEBUG_SERIAL Serial
#define TELEMETRY_SERIAL Serial1 // Serial port for telemetry communication
#define DEBUG_MODE 1 // Set to 1 to enable debug mode, 0 to disable

#define DATALOG_INTERVAL 20 // Interval in milliseconds for data logging
#define TELEMETRY_INTERVAL 100 // Interval in milliseconds for telemetry updates

#define FLOATS_PER_LOG (1 + 4 + 3 + 3 + 3 + 3 + 3 + 3 + 2 + 2 + 1 + 1) // 1 time + 4 quat + 3 accel + 3 vel + 3 pos + 3 rawAccel + 3 rawGyro + 3 gyroBias + 2 attSet + 2 servoCmd + 1 thrust + 1 rwSpeed
#define BYTES_PER_LOG (sizeof(uint32_t) + (FLOATS_PER_LOG * sizeof(float)) + 3 * sizeof(int)) // separator + floats + 2 * int
#define MAX_DATA_LOGS 2000 // Maximum number of data logs to store

#define LOG_SEPARATOR 0xDEADBEEF

#define Y_TARGET 2.0f // Target Y position in meters for landing
#define X_TARGET 0.0f // Target X position in meters for landing

#define GIMBAL_LIMIT_RAD 5.0 * DEG_TO_RAD // 5 degrees in radians

#define PYRO_LOCKOUT_ALT 5.0

#endif // CONSTANTS_H
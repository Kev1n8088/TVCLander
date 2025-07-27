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
#define PRELAUNCH_AVERAGE_COUNT 10 // Number of samples to average before launch
#define PRELAUNCH_AVERAGE_INTERVAL 30 // Interval in milliseconds between samples before launch

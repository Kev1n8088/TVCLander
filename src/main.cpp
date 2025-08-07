#include <Arduino.h>
#include <SPI.h>
#include <SCH1.h>
#include <Adafruit_NeoPixel.h>
#include "ICM45686.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LIS2MDL.h>
#include <Constants.h>
#include "StateEstimation.h"
#include "Telemetry.h"
#include "PID.h"
#include "Servo.h"

// Instantiate an ICM456XX with SPI interface and CS on pin 8



Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

StateEstimation StateEstimate;
Telemetry TelemetrySystem;

Servo PitchServo;
Servo YawServo;

void setup() {
  // put your setup code here, to run once
  PitchServo.attach(PITCH_SERVO); // Attach the pitch servo to the specified pin
  YawServo.attach(YAW_SERVO); // Attach the yaw servo to the specified pin

  if (DEBUG_MODE) {
      DEBUG_SERIAL.begin(DEBUG_BAUD); // Initialize debug serial port with specified baud rate
      while (!DEBUG_SERIAL) {
        delay(1); // Wait for serial port to connect. Needed for native USB port only
      }
  }

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(10);
  pixels.clear();
  pixels.show();

  DEBUG_SERIAL.println(StateEstimate.begin()); // Initialize state estimation
  TelemetrySystem.begin(); // Initialize telemetry system
  StateEstimate.setVehicleState(1);

}

void loop() {
  PitchServo.writeMicroseconds(1500);
  YawServo.writeMicroseconds(1500); // Set servos to neutral position
  delay(1000); // Wait for servos to stabilize

  //StateEstimate.estimateState();
  //TelemetrySystem.telemetryLoop(StateEstimate); // Call telemetry loop to log and send data=
}

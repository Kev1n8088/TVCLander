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
#include "Servo.h"

// Instantiate an ICM456XX with SPI interface and CS on pin 8



Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

StateEstimation StateEstimate;
Telemetry TelemetrySystem;


void setup() {
  // put your setup code here, to run once

  if (DEBUG_MODE) {
      DEBUG_SERIAL.begin(DEBUG_BAUD); // Initialize debug serial port with specified baud rate
      // while (!DEBUG_SERIAL) {
      //   delay(1); // Wait for serial port to connect. Needed for native USB port only
      // }
  }

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(10);
  pixels.clear();
  pixels.show();

  DEBUG_SERIAL.println(StateEstimate.begin()); // Initialize state estimation
  TelemetrySystem.begin(); // Initialize telemetry system
  StateEstimate.setVehicleState(0); // Set initial vehicle state to disarmed

}

void loop() {
  if (millis() > 10000){
    StateEstimate.setVehicleState(1); // Set vehicle state to armed after 10 seconds
  }
  StateEstimate.estimateState();
  TelemetrySystem.telemetryLoop(StateEstimate); // Call telemetry loop to log and send data=
}

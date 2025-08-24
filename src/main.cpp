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


uint64_t lastNeopixTime = 0; // Variable to store the end time of NeoPixel updates

void setup() {
  // put your setup code here, to run once

  if (DEBUG_MODE) {
      DEBUG_SERIAL.begin(DEBUG_BAUD); // Initialize debug serial port with specified baud rate
      while (!DEBUG_SERIAL) {
        delay(1); // Wait for serial port to connect. Needed for native USB port only
      }
      DEBUG_SERIAL.println("Debug mode enabled");
  }

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(10);
  pixels.clear();
  pixels.show();

  StateEstimate.begin(); // Initialize state estimation
  TelemetrySystem.begin(); // Initialize telemetry system
  //StateEstimate.forceVehicleState(1); // Set initial vehicle state to disarmed

  lastNeopixTime = millis(); // Record the end time of setup for debugging purposes
}

void loop() {
  StateEstimate.estimateState();
  TelemetrySystem.telemetryLoop(StateEstimate); // Call telemetry loop to log and send data

  if (millis() - lastNeopixTime > 100) {
    lastNeopixTime = millis();
    // Cycle through hues over time (0-65535 range for Adafruit NeoPixel HSV)
    uint16_t hue = (millis() * 5) % 65536; // Complete cycle every ~22 minutes
    pixels.fill(pixels.ColorHSV(hue, 255, 255)); // Full saturation and brightness
    pixels.show();
  }

  // if (millis() - endSetupTime > 10000){
  //   if (StateEstimate.getVehicleState() == 0) {
  //     //StateEstimate.setVehicleState(69); // Set vehicle state to armed after 10 seconds
  //     StateEstimate.forceVehicleState(69);
  //     DEBUG_SERIAL.println("Vehicle state set to armed test");
  //   }
  // }
}

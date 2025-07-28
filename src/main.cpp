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

// Instantiate an ICM456XX with SPI interface and CS on pin 8



Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);


void setup() {
  // put your setup code here, to run once

  if (DEBUG_MODE) {
      DEBUG_SERIAL.begin(DEBUG_BAUD); // Initialize debug serial port with specified baud rate
  }

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(10);
  pixels.clear();
  pixels.show();
}

void loop() {
}

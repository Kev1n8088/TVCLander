#include <Arduino.h>
#include <SPI.h>
#include <SCH16T/SCH1.h>
#include <Adafruit_NeoPixel.h>
#include "ICM45686.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LIS2MDL.h>

// Instantiate an ICM456XX with SPI interface and CS on pin 8

#define MAIN_IMU_RST_PIN A7 // Define the reset pin for SCH1
#define MAIN_IMU_CS_PIN A8


#define RGB_PIN        A9 // On Trinket or Gemma, suggest changing this to 1
#define NUMPIXELS 2 // Popular NeoPixel ring size

#define BACK_IMU_CS_PIN 4

SCH1 main_imu(MAIN_IMU_RST_PIN); // Create an instance of SCH1 with the reset pin
ICM456xx IMU(SPI, BACK_IMU_CS_PIN); // Create an instance of ICM456xx with SPI and CS pin

Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

#define BMP_CS A2

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;


Adafruit_LIS2MDL lis2mdl = Adafruit_LIS2MDL(12345);
#define LIS2MDL_CS A14

#define SRV0 30
#define SRV1 29
#define SRV2 25
#define SRV3 24
#define SRV4 33
#define SRV5 36
#define SRV6 37

void setup() {
  // put your setup code here, to run once:

  pinMode(SRV0, OUTPUT);
  pinMode(SRV1, OUTPUT);
  pinMode(SRV2, OUTPUT);
  pinMode(SRV3, OUTPUT);
  pinMode(SRV4, OUTPUT);
  pinMode(SRV5, OUTPUT);
  pinMode(SRV6, OUTPUT);

  digitalWrite(SRV0, HIGH);
  digitalWrite(SRV1, HIGH);
  digitalWrite(SRV2, HIGH);
  digitalWrite(SRV3, HIGH);
  digitalWrite(SRV4, HIGH);
  digitalWrite(SRV5, HIGH);
  digitalWrite(SRV6, HIGH);
  

  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for serial port to connect. Needed for native USB
  }


  SPI.begin();

  int ret;
  
  // ret = IMU.begin();
  // if (ret != 0) {
  //   Serial.print("ICM456xx initialization failed: ");
  //   Serial.println(ret);
  //   while(1);
  // }else{
  //   Serial.println("ICM456xx initialized successfully!");
  // }

  Serial.println("Adafruit BMP388 / BMP390 test");

  if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }else{
    Serial.println("BMP3 sensor initialized successfully!");
  }

  lis2mdl.enableAutoRange(true);

  /* Initialise the sensor */
  if (! lis2mdl.begin_SPI(LIS2MDL_CS)) {  // hardware SPI mode
  //if (! lis2mdl.begin_SPI(LIS2MDL_CS, LIS2MDL_CLK, LIS2MDL_MISO, LIS2MDL_MOSI)) { // soft SPI
    /* There was a problem detecting the LIS2MDL ... check your connections */
    Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
    while (1) delay(10);
  }

  /* Display some basic information on this sensor */
  lis2mdl.printSensorDetails();

  char serial_num[15];
  int init_status;
  SCH1_filter Filter;
  SCH1_sensitivity Sensitivity;
  SCH1_decimation Decimation;
// SCH1600 settings and initialization
  //------------------------------------

  // SCH1600 filter settings
  Filter.Rate12 = FILTER_RATE;
  Filter.Acc12 = FILTER_ACC12;
  Filter.Acc3 = FILTER_ACC3;

  // SCH1600 sensitivity settings
  Sensitivity.Rate1 = SENSITIVITY_RATE1;
  Sensitivity.Rate2 = SENSITIVITY_RATE2; 
  Sensitivity.Acc1 = SENSITIVITY_ACC1;
  Sensitivity.Acc2 = SENSITIVITY_ACC2;
  Sensitivity.Acc3 = SENSITIVITY_ACC3;

  // SCH1600 decimation settings (for Rate2 and Acc2 channels).
  Decimation.Rate2 = DECIMATION_RATE;
  Decimation.Acc2 = DECIMATION_ACC;

  //main_imu = SCH1(RST_PIN); // Initialize SCH1 with reset pin 4
  // put your setup code here, to run once:
  init_status = main_imu.SCH1_init(MAIN_IMU_CS_PIN, Filter, Sensitivity, Decimation, true, 10000000, &SPI);
  Serial.println("Got past init");
  if(init_status != SCH1_OK) {
    Serial.println("SCH1 initialization failed!");
    // while(1) { 
    //   Serial.println("SCH1 initialization failed!");
    //   delay(500);
    // }; // Halt the program if initialization fails
  } else {
    Serial.println("SCH1 initialized successfully!");
  }

  strcpy(serial_num, main_imu.SCH1_getSnbr());
  Serial.print("Serial number: "); 
  Serial.println(serial_num);


  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.setBrightness(10);
  pixels.clear();
  pixels.show();
}

void loop() {
}

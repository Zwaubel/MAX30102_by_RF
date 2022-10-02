/********************************************************

  Project: MAXREFDES117#
  Filename: RD117_ARDUINO.ino
  Description: This module contains the Main application for the MAXREFDES117 example program.

  Revision History:
  \n 1-18-2016 Rev 01.00 GL Initial release.
  \n 12-22-2017 Rev 02.00 Significantlly modified by Robert Fraczkiewicz
  \n 08-22-2018 Rev 02.01 Added conditional compilation of the code related to ADALOGGER SD card operations

  --------------------------------------------------------------------

  This code follows the following naming conventions:

  char              ch_pmod_value
  char (array)      s_pmod_s_string[16]
  float             f_pmod_value
  int32_t           n_pmod_value
  int32_t (array)   an_pmod_value[16]
  int16_t           w_pmod_value
  int16_t (array)   aw_pmod_value[16]
  uint16_t          uw_pmod_value
  uint16_t (array)  auw_pmod_value[16]
  uint8_t           uch_pmod_value
  uint8_t (array)   auch_pmod_buffer[16]
  uint32_t          un_pmod_value
  int32_t *         pn_pmod_value

  ------------------------------------------------------------------------- */

#define FW_VERSION "0.0.1"

#include <Arduino.h>
#include "algorithm_by_RF.h"
#include "MAX3010x.h"

//#define DEBUG // Uncomment for debug output to the Serial stream
//#define DEBUG_INCL_RAW // Uncomment to include raw data in debug output to the Serial stream
#define SD_CARD_LOGGING // Comment out if you don't have ADALOGGER itself but your MCU still can handle this code
//#define TEST_MAXIM_ALGORITHM // Uncomment if you want to include results returned by the original MAXIM algorithm
//#define SAVE_RAW_DATA // Uncomment if you want raw data coming out of the sensor saved to SD card. Red signal first, IR second.

#ifdef TEST_MAXIM_ALGORITHM
#include "algorithm.h"
#endif

// Interrupt pin
#define PIN_OXI_INT     19 // pin connected to MAX30102 INT

// ADALOGGER pins
#ifdef SD_CARD_LOGGING
#include <SPI.h>
#include <mySD.h>
#define PIN_SD_CS       13
#define PIN_SD_MOSI     15
#define PIN_SD_MISO     2
#define PIN_SD_CLK      14
#define PIN_SD_DETECT   7
#define PIN_TOGGLE_LED  33 // Red LED on ADALOGGER
ext::File dataFile;
bool sdCardOk;
#endif

// battery voltage
#define PIN_VBAT        35

MAX3010x oxi;
// MAX30102 Config
struct {
  uint8_t uch_oxiLedBrightness = 0xAF;  // Options: 0=Off to 255=50mA
  uint8_t uch_oxiSampleAverage = 8;     // Options: 1, 2, 4, 8, 16, 32
  uint8_t uch_oxiLedMode = 2;           // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  uint16_t uw_oxiSampleRate = 400;      // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  uint16_t uw_oxiPulseWidth = 411;      // Options: 69, 118, 215, 411
  uint16_t uw_oxiAdcRange = 16384;      // Options: 2048, 4096, 8192, 16384
} const oxiConfig;

uint32_t un_elapsed_t_s, un_start_t_ms; // variables to keep track of runtime
uint32_t aun_ir_buffer[BUFFER_SIZE];    //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];   //red LED sensor data
float old_n_spo2 = 0.0;                 // Previous SPO2 value
uint8_t uch_sdCardDataLines = 0;        // counter for sd card data flushing
#define SD_DATA_LINE_FLUSH_THRESH 1     // sd card data line threshold to flush data


#ifdef SD_CARD_LOGGING
// blink three times if isOK is true, otherwise blink continuously
void toggleLED(const byte led, bool isOK)
{
  byte i;
  if (isOK) {
    for (i = 0; i < 3; ++i) {
      digitalWrite(led, HIGH);
      delay(200);
      digitalWrite(led, LOW);
      delay(200);
    }
  } else {
    while (1) {
      for (i = 0; i < 2; ++i) {
        digitalWrite(led, HIGH);
        delay(50);
        digitalWrite(led, LOW);
        delay(50);
      }
      delay(500);
    }
  }
}
#endif

float readVBat(void) {
  float vBat = analogRead(PIN_VBAT) * 2.0 * 4.2 / 4096.0;
  // This returns the battery voltage (adc reading * voltage divider * max battery voltage / max range )
  // voltage divider = in the PCB of the T8 you can see there is 2 equal resistors, hence 50% division
  return vBat;
}

void millis_to_hours(uint32_t ms, char* hr_str)
{
  char istr[6];
  uint32_t secs, mins, hrs;
  secs = ms / 1000; // time in seconds
  mins = secs / 60; // time in minutes
  secs -= 60 * mins; // leftover seconds
  hrs = mins / 60; // time in hours
  mins -= 60 * hrs; // leftover minutes
  itoa(hrs, hr_str, 10);
  strcat(hr_str, ":");
  itoa(mins, istr, 10);
  strcat(hr_str, istr);
  strcat(hr_str, ":");
  itoa(secs, istr, 10);
  strcat(hr_str, istr);
}


void setup() {
  pinMode(PIN_OXI_INT, INPUT);  // connects to the interrupt output pin of the MAX30102

#ifdef SD_CARD_LOGGING
  //pinMode(PIN_SD_DETECT, INPUT_PULLUP);
  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_TOGGLE_LED, OUTPUT);
  digitalWrite(PIN_TOGGLE_LED, LOW);

  // set spi interface pins according to the sd card slot pinout
  //SPI.begin(PIN_SD_CS, PIN_SD_MOSI, PIN_SD_MISO, PIN_SD_CLK);
#endif

#if defined(DEBUG) || !defined(SD_CARD_LOGGING)
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.println(F("Sensor init started..."));
#endif

  // Initialize sensor
  if (!oxi.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
#if defined(DEBUG)
    Serial.println(F(" - MAX30102 failed!"));
#endif
    while (1);
  }
#if defined(DEBUG)
  Serial.println(F(" - MAX30102 initialized!"));
#endif

#ifdef SD_CARD_LOGGING
  char systemStatus[20];
  //if (HIGH == digitalRead(cardDetect)) {
  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!SD.begin(PIN_SD_CS, PIN_SD_MOSI, PIN_SD_MISO, PIN_SD_CLK)) {
    sdCardOk = false;
    strncpy(systemStatus, "SD card init failed!", 20);
  } else sdCardOk = true;
  //}

#if defined(DEBUG)
  if (sdCardOk)
    Serial.println(F(" - SD card init successful!"));
  else
    Serial.println(F(" - SD card init failed!"));
#endif

  if (sdCardOk) {
    long count = 0;
    char fname[20];
    do {
      //      if(useClock && now.month()<13 && now.day()<32) {
      //        sprintf(fname,"%d-%d_%d.txt",now.month(),now.day(),++count);
      //      } else {
      sprintf(fname, "data_%d.txt", ++count);
      //      }
    } while (SD.exists(fname));
    dataFile = SD.open(fname, FILE_WRITE);
    strncpy(systemStatus, "waiting for user", 16);
  }

  if (!SD.exists(dataFile.name())) {
#ifdef DEBUG
    while (Serial.available() == 0) //wait until user presses a key
    {
      Serial.print(F("Firmware version: "));
      Serial.println(FW_VERSION);
      Serial.print(F("System status: "));
      Serial.println(systemStatus);
      Serial.print(F("SD Card file: "));
      Serial.println("NOT SET");
      Serial.print(F("VBat: "));
      Serial.println(readVBat());
      delay(1000);
    }
    Serial.read();
#else
    while (1);
#endif
  }

  //Configure sensor with these settings
  oxi.setup(
    oxiConfig.uch_oxiLedBrightness,
    oxiConfig.uch_oxiSampleAverage,
    oxiConfig.uch_oxiLedMode,
    oxiConfig.uw_oxiSampleRate,
    oxiConfig.uw_oxiPulseWidth,
    oxiConfig.uw_oxiAdcRange
  );

  //oxi.enableAFULL(); //Enable the almost full interrupt (default is 32 samples)
  //oxi.setFIFOAlmostFull(0x0F); //Set almost full int to fire at 29 samples

#if defined(DEBUG)
  Serial.println(F("Sensor init finished!"));

  while (Serial.available() == 0) //wait until user presses a key
  {
    Serial.print(F("Firmware version: "));
    Serial.println(FW_VERSION);
    Serial.print(F("System status: "));
    Serial.println(systemStatus);
    Serial.print(F("SD Card file: "));
    Serial.println(dataFile.name());
    Serial.print(F("VBat: "));
    Serial.println(readVBat());
    Serial.println(F("Press any key to start conversion"));
    delay(1000);
  }
  Serial.read();
#endif

  toggleLED(PIN_TOGGLE_LED, sdCardOk);

  dataFile.print(F("Firmware version: "));
  dataFile.println(FW_VERSION);
  dataFile.print(F("Sensor state: "));
  dataFile.println(systemStatus);
#ifdef TEST_MAXIM_ALGORITHM
  dataFile.println(F("Time[s]\tSpO2\tHR\tSpO2_MX\tHR_MX\tClock\tRatio\tCorr\tTemp[C]\tVBat"));
#else // TEST_MAXIM_ALGORITHM
  dataFile.println(F("Time[s]\tSpO2\tHR\tClock\tRatio\tCorr\tTemp[C]\tVBat"));
#endif // TEST_MAXIM_ALGORITHM
#ifdef SAVE_RAW_DATA
  int32_t i;
  // These are headers for the red signal
  for (i = 0; i < BUFFER_SIZE; ++i) {
    dataFile.print(F("\t"));
    dataFile.print(i);
  }
  // These are headers for the infrared signal
  for (i = 0; i < BUFFER_SIZE; ++i) {
    dataFile.print(F("\t"));
    dataFile.print(i);
  }
  dataFile.println();
#endif // SAVE_RAW_DATA

#else // SD_CARD_LOGGING

  while (Serial.available() == 0) //wait until user presses a key
  {
    Serial.println(F("Press any key to start conversion"));
    delay(1000);
  }
  Serial.read();
#ifdef TEST_MAXIM_ALGORITHM
  Serial.println(F("Time[s]\tSpO2\tHR\tSpO2_MX\tHR_MX\tClock\tRatio\tCorr\tTemp[C]\tVBat"));
#else // TEST_MAXIM_ALGORITHM
  Serial.println(F("Time[s]\tSpO2\tHR\tClock\tRatio\tCorr\tTemp[C]\tVBat"));
#endif // TEST_MAXIM_ALGORITHM
#ifdef SAVE_RAW_DATA
  int32_t i;
  // These are headers for the red signal
  for (i = 0; i < BUFFER_SIZE; ++i) {
    Serial.print(F("\t"));
    Serial.print(i);
  }
  // These are headers for the infrared signal
  for (i = 0; i < BUFFER_SIZE; ++i) {
    Serial.print(F("\t"));
    Serial.print(i);
  }
  Serial.println();
#endif // SAVE_RAW_DATA

#endif // SD_CARD_LOGGING

  un_start_t_ms = millis();
}

// Continuously taking samples from MAX30102.
// Heart rate and SpO2 are calculated every ST seconds
void loop() {
  float n_spo2, ratio, correl;  //SPO2 value
  int8_t ch_spo2_valid;         //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate;         //heart rate value
  int8_t  ch_hr_valid;          //indicator to show if the heart rate calculation is valid
  int32_t i;
  char hr_str[10];

  //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  //read BUFFER_SIZE samples, and determine the signal range
  for (i = 0; i < BUFFER_SIZE; i++)
  {
    // while (digitalRead(PIN_OXI_INT) == 1); //wait until the interrupt pin asserts
    //oxi.check();

    // oxi.check();
    // aun_ir_buffer[i] = oxi.getFIFOIR();
    // aun_red_buffer[i] = oxi.getFIFORed();
    aun_ir_buffer[i] = oxi.getIR();
    aun_red_buffer[i] = oxi.getRed();

#ifdef DEBUG_INCL_RAW
    Serial.print(i, DEC);
    Serial.print(F("\t"));
    Serial.print(aun_red_buffer[i], DEC);
    Serial.print(F("\t"));
    Serial.println(aun_ir_buffer[i], DEC);
#endif // DEBUG_INCL_RAW

    //oxi.nextSample();
  }

  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);
  un_elapsed_t_s = millis() - un_start_t_ms;
  millis_to_hours(un_elapsed_t_s, hr_str); // Time in hh:mm:ss format
  un_elapsed_t_s /= 1000; // Time in seconds

  // Read the _chip_ temperature in degrees Celsius
  float temperature = oxi.readTemperature();
  float vBatReading = readVBat();

#ifdef DEBUG
  Serial.println(F("--RF--"));
  Serial.print(un_elapsed_t_s, DEC);
  Serial.print(F("\t"));
  Serial.print(n_spo2, 2);
  Serial.print(F("\t"));
  Serial.print(n_heart_rate, DEC);
  Serial.print(F("\t"));
  Serial.print(hr_str);
  Serial.print(F("\t"));
  Serial.print(temperature, 2);
  Serial.print(F("\t"));
  Serial.println(vBatReading, 2);
  Serial.println(F("------"));
#endif // DEBUG

#ifdef TEST_MAXIM_ALGORITHM
  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using MAXIM's method
  float n_spo2_maxim;  //SPO2 value
  int8_t ch_spo2_valid_maxim;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate_maxim; //heart rate value
  int8_t  ch_hr_valid_maxim;  //indicator to show if the heart rate calculation is valid
  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2_maxim, &ch_spo2_valid_maxim, &n_heart_rate_maxim, &ch_hr_valid_maxim);
#ifdef DEBUG
  Serial.println(F("--MX--"));
  Serial.print(un_elapsed_t_s, DEC);
  Serial.print(F("\t"));
  Serial.print(n_spo2_maxim, 2);
  Serial.print(F("\t"));
  Serial.print(n_heart_rate_maxim, DEC);
  Serial.print(F("\t"));
  Serial.print(hr_str);
  Serial.print(F("\t"));
  Serial.print(temperature, 2);
  Serial.print(F("\t"));
  Serial.println(vBatReading, 2);
  Serial.println(F("------"));
#endif // DEBUG
#endif // TEST_MAXIM_ALGORITHM

  //save samples and calculation result to SD card
#ifdef TEST_MAXIM_ALGORITHM
  if (ch_hr_valid && ch_spo2_valid || ch_hr_valid_maxim && ch_spo2_valid_maxim) {
#else   // TEST_MAXIM_ALGORITHM
  if (ch_hr_valid && ch_spo2_valid) {
#endif // TEST_MAXIM_ALGORITHM
#ifdef SD_CARD_LOGGING
    ++uch_sdCardDataLines;
    dataFile.print(un_elapsed_t_s, DEC);
    dataFile.print(F("\t"));
    dataFile.print(n_spo2, 2);
    dataFile.print(F("\t"));
    dataFile.print(n_heart_rate, DEC);
    dataFile.print(F("\t"));
#ifdef TEST_MAXIM_ALGORITHM
    dataFile.print(n_spo2_maxim);
    dataFile.print(F("\t"));
    dataFile.print(n_heart_rate_maxim, DEC);
    dataFile.print(F("\t"));
#endif // TEST_MAXIM_ALGORITHM
    dataFile.print(hr_str);
    dataFile.print(F("\t"));
    dataFile.print(ratio, 2);
    dataFile.print(F("\t"));
    dataFile.print(correl, 2);
    dataFile.print(F("\t"));
    dataFile.print(temperature, 2);
    dataFile.print(F("\t"));
    dataFile.println(vBatReading, 2);
#ifdef SAVE_RAW_DATA
    // Save raw data for unusual O2 levels
    for (i = 0; i < BUFFER_SIZE; ++i)
    {
      dataFile.print(F("\t"));
      dataFile.print(aun_red_buffer[i], DEC);
    }
    for (i = 0; i < BUFFER_SIZE; ++i)
    {
      dataFile.print(F("\t"));
      dataFile.print(aun_ir_buffer[i], DEC);
    }
    dataFile.println();
#endif // SAVE_RAW_DATA
    // Blink green LED to indicate save event
    //digitalWrite(sdIndicatorPin, HIGH);
    //delay(10);
    //digitalWrite(sdIndicatorPin, LOW);
    // FLush SD buffer every SD_CARD_DATA_LINE_FLUSH_THRESH points
    if (uch_sdCardDataLines >= SD_DATA_LINE_FLUSH_THRESH) {
      dataFile.flush();
      uch_sdCardDataLines = 0;
    }
#else // SD_CARD_LOGGING
    Serial.print(un_elapsed_t_s, DEC);
    Serial.print(F("\t"));
    Serial.print(n_spo2, 2);
    Serial.print(F("\t"));
    Serial.print(n_heart_rate, DEC);
    Serial.print(F("\t"));
#ifdef TEST_MAXIM_ALGORITHM
    Serial.print(n_spo2_maxim, 2);
    Serial.print(F("\t"));
    Serial.print(n_heart_rate_maxim, DEC);
    Serial.print(F("\t"));
#endif //TEST_MAXIM_ALGORITHM
    Serial.print(hr_str);
    Serial.print(F("\t"));
    Serial.print(ratio, 2);
    Serial.print(F("\t"));
    Serial.print(correl, 2);
    Serial.print(F("\t"));
    Serial.print(temperature, 2);
    Serial.print(F("\t"));
    Serial.println(vBatReading, 2);
#ifdef DEBUG_INCL_RAW
    // Save raw data for unusual O2 levels
    for (i = 0; i < BUFFER_SIZE; ++i)
    {
      Serial.print(F("\t"));
      Serial.print(aun_red_buffer[i], DEC);
    }
    for (i = 0; i < BUFFER_SIZE; ++i)
    {
      Serial.print(F("\t"));
      Serial.print(aun_ir_buffer[i], DEC);
    }
    Serial.println();
#endif // DEBUG_INCL_RAW
#endif // SD_CARD_LOGGING
    old_n_spo2 = n_spo2;
  }
}

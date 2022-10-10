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

  bool              b_pmod_value
  char              ch_pmod_value
  char (array)      s_pmod_s_string[16]
  char *            ps_pmod_s_string
  int16_t           w_pmod_value
  int16_t (array)   aw_pmod_value[16]
  int32_t           n_pmod_value
  int32_t (array)   an_pmod_value[16]
  int32_t *         pn_pmod_value
  uint8_t           uch_pmod_value
  uint8_t (array)   auch_pmod_buffer[16]
  uint16_t          uw_pmod_value
  uint16_t (array)  auw_pmod_value[16]
  uint32_t          un_pmod_value
  uint32_t (array)  aun_pmod_value
  float             f_pmod_value
  float (array)     af_pmod_value

  ------------------------------------------------------------------------- */

#define FW_VERSION "0.0.1"
#define HW_VERSION "0.0.1-ESP32"

#define MEASUERMENT_DATA_HEADER F("Time[s]\tSpO2\tHR\tTemp[C]\tClock\tRatio\tCorr\tVBat\tBatCap")

#include <Arduino.h>
#include "algorithm_by_RF.h"
#include "MAX3010x.h"

//#define DEBUG // Uncomment for debug output to the Serial stream
//#define DEBUG_INCL_RAW // Uncomment to include raw data in debug output to the Serial stream
#define SD_CARD_LOGGING // Comment out if you don't have ADALOGGER itself but your MCU still can handle this code
//#define SAVE_RAW_DATA // Uncomment if you want raw data coming out of the sensor saved to SD card. Red signal first, IR second.
#define BLE_COMM // disable/enable bluetooth low-energy communication

// Interrupt pin
#define PIN_OXI_INT         23 // pin connected to MAX30102 INT
#define PIN_OXI_SDA         19 // pin connected to MAX30102 SDA
#define PIN_OXI_SCL         22 // pin connected to MAX30102 SCL
#define OXI_CONNECT_RETRIES 5

// sd card logging
#if defined(SD_CARD_LOGGING)
#include <SPI.h>
#include <mySD.h>
#define PIN_SD_CS       13
#define PIN_SD_MOSI     15
#define PIN_SD_MISO     2
#define PIN_SD_CLK      14
#define PIN_SD_DETECT   7
#define PIN_TOGGLE_LED  21 // Red LED on ADALOGGER
ext::File dataFile;
bool b_sdCardOk;
#endif

// battery voltage and capacity
#define PIN_VBAT                    35
#define VBAT_VOLT_DIV_RATIO         2.0
#define VBAT_ADC_REF_MILLI_VOLTAGE  3.6
#define BAT_LEVEL_UPDATE_MS         10000
#define VBAT_MIN_MILLI_VOLT         3000
#define VBAT_MAX_MILLI_VOLT         4200
uint32_t un_lastBatLevelUpdate = 0; // variables for polling timing

MAX3010x oxi;
// MAX30102 Config
struct {
  uint8_t uch_oxiLedBrightness = 0x2F;  // Options: 0=Off to 255=50mA
  uint8_t uch_oxiSampleAverage = 8;     // Options: 1, 2, 4, 8, 16, 32
  uint8_t uch_oxiLedMode = 2;           // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  uint16_t uw_oxiSampleRate = 400;      // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  uint16_t uw_oxiPulseWidth = 411;      // Options: 69, 118, 215, 411
  uint16_t uw_oxiAdcRange = 16384;      // Options: 2048, 4096, 8192, 16384
} const oxiConfig;

uint32_t un_elapsedTime_s, un_start_t_ms; // variables to keep track of runtime
uint32_t aun_ir_buffer[BUFFER_SIZE];    //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];   //red LED sensor data
float f_oldSpo2 = 0.0;                 // Previous SPO2 value
uint8_t uch_sdCardDataLines = 0;        // counter for sd card data flushing
#define SD_DATA_LINE_FLUSH_THRESH 1     // sd card data line threshold to flush data


#if defined(BLE_COMM)

#include <esp_wifi.h>
#include <esp_pm.h>
//#include <esp_err.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// BLE advertisement name
#define BLE_ADVERTISING_NAME "LibreOx ESP32"

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
// Device Info Service
#define DEVICE_INFO_SERVICE_UUID "0000180A-0000-1000-8000-00805F9B34FB"
#define DEVICE_INFO_CHAR_MODEL_NR_UUID "00002A24-0000-1000-8000-00805F9B34FB"
#define DEVICE_INFO_CHAR_SERIAL_NR_UUID "00002A25-0000-1000-8000-00805F9B34FB"
#define DEVICE_INFO_CHAR_FIRMWARE_REV_UUID "00002A26-0000-1000-8000-00805F9B34FB"
#define DEVICE_INFO_CHAR_HARDWARE_REV_UUID "00002A27-0000-1000-8000-00805F9B34FB"
#define DEVICE_INFO_CHAR_MANUFACTURER_UUID "00002A29-0000-1000-8000-00805F9B34FB"

// Nordic UART Service
#define NORDIC_UART_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NORDIC_UART_CHAR_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NORDIC_UART_CHAR_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
BLECharacteristic* p_bleUartTxCharacteristic = NULL;

// Battery Service
#define BATTERY_SERVICE_UUID "0000180F-0000-1000-8000-00805F9B34FB"
#define BATTERY_LEVEL_CHAR_UUID "00002A19-0000-1000-8000-00805F9B34FB"
BLECharacteristic* p_bleBatCharacteristic = NULL;

// Heart Rate Service
#define HEART_RATE_SERVICE_UUID "0000180D-0000-1000-8000-00805F9B34FB"
#define HEART_RATE_MEASUREMENT_CHAR_UUID "00002A37-0000-1000-8000-00805F9B34FB"
BLECharacteristic* p_bleHeartRateCharacteristic = NULL;

// Blood Oxi Service
#define PULSE_OXIMETER_SERVICE_UUID "00001822-0000-1000-8000-00805F9B34FB"
#define PULSE_OXIMETER_CONT_MEAS_CHAR_UUID "00002A5F-0000-1000-8000-00805F9B34FB"
BLECharacteristic* p_blePulseOxiCharacteristic = NULL;

// Thermomter Service
#define THERMOMETER_SERVICE_UUID "00001809-0000-1000-8000-00805F9B34FB"
#define TEMPERATURE_MEASUREMENT_CHAR_UUID "00002A1C-0000-1000-8000-00805F9B34FB"
BLECharacteristic* p_bleTemperatureMeasCharacteristic = NULL;

// BLE Server pointer
BLEServer * p_bleServer = NULL;
bool b_bleConnected = false;  // connected flag

class BleServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      b_bleConnected = true;
#if defined(DEBUG)
      Serial.println(F("Device connected."));
#endif
    };

    void onDisconnect(BLEServer* pServer) {
      b_bleConnected = false;
      //delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising();
#if defined(DEBUG)
      Serial.println(F("Device disconnected. Start advertising."));
#endif
    }
};

class BleUARTCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
#if defined(DEBUG)
        Serial.println(F("*********"));
        Serial.print(F("Received Value: "));

        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println(F("*********"));
#endif
        rxValue = "ECHO: " + rxValue + "\n";
        p_bleUartTxCharacteristic->setValue(rxValue);
        p_bleUartTxCharacteristic->notify();
      }
    }
};

void bleInit(void) {
  // create the BLE Device
  BLEDevice::init(BLE_ADVERTISING_NAME);

  // set BLE power
  /**
    Set the transmission power.
    The power level can be one of:
  * * ESP_PWR_LVL_N12 = 0, Corresponding to -12dbm
  * * ESP_PWR_LVL_N9  = 1, Corresponding to  -9dbm
  * * ESP_PWR_LVL_N6  = 2, Corresponding to  -6dbm
  * * ESP_PWR_LVL_N3  = 3, Corresponding to  -3dbm
  * * ESP_PWR_LVL_N0  = 4, Corresponding to   0dbm
  * * ESP_PWR_LVL_P3  = 5, Corresponding to  +3dbm
  * * ESP_PWR_LVL_P6  = 6, Corresponding to  +6dbm
  * * ESP_PWR_LVL_P9  = 7, Corresponding to  +9dbm

    The power types can be one of:
  * * ESP_BLE_PWR_TYPE_CONN_HDL0
  * * ESP_BLE_PWR_TYPE_CONN_HDL1
  * * ESP_BLE_PWR_TYPE_CONN_HDL2
  * * ESP_BLE_PWR_TYPE_CONN_HDL3
  * * ESP_BLE_PWR_TYPE_CONN_HDL4
  * * ESP_BLE_PWR_TYPE_CONN_HDL5
  * * ESP_BLE_PWR_TYPE_CONN_HDL6
  * * ESP_BLE_PWR_TYPE_CONN_HDL7
  * * ESP_BLE_PWR_TYPE_CONN_HDL8
  * * ESP_BLE_PWR_TYPE_ADV
  * * ESP_BLE_PWR_TYPE_SCAN
  * * ESP_BLE_PWR_TYPE_DEFAULT
  */
  BLEDevice::setPower(ESP_PWR_LVL_N12, ESP_BLE_PWR_TYPE_ADV);

  // create the BLE Server
  p_bleServer = BLEDevice::createServer();
  p_bleServer->setCallbacks(new BleServerCallbacks());

  // add UART service
  bleAddUARTService(p_bleServer);

  // add device info service
  bleAddDeviceInfoService(p_bleServer);

  // add Battery service
  bleAddBatteryService(p_bleServer);

  // add Heart Rate service
  bleAddHeartRateService(p_bleServer);

  // add Pulseoximetry service
  bleAddPulseOximeterService(p_bleServer);

  // add Thermometer Service
  bleAddThermometerService(p_bleServer);

  // Start advertising
  p_bleServer->startAdvertising();

  // disable WiFi
  esp_wifi_stop();

  // enable automatic light sleep
  esp_pm_config_esp32_t pm_config = {
    .max_freq_mhz = 240,
    .min_freq_mhz = 80,
    .light_sleep_enable = true,
  };
  //ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
  esp_pm_configure(&pm_config);
}

void bleAddUARTService(BLEServer* pServer) {
  // create a BLE service
  BLEService* p_bleService = pServer->createService(NORDIC_UART_SERVICE_UUID);
  pServer->getAdvertising()->addServiceUUID(NORDIC_UART_SERVICE_UUID);

  // add BLE UART characteristics to the service;
  // UART RX
  BLECharacteristic* p_uartCharacteristic = p_bleService->createCharacteristic(
        NORDIC_UART_CHAR_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE
      );
  p_uartCharacteristic->setCallbacks(new BleUARTCallbacks());

  // UART TX
  p_bleUartTxCharacteristic = p_bleService->createCharacteristic(
                                NORDIC_UART_CHAR_TX_UUID,
                                BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
                              );
  p_bleUartTxCharacteristic->addDescriptor(new BLE2902());

  // start the service
  p_bleService->start();
}

void bleAddDeviceInfoService(BLEServer* pServer) {
  // create a BLE service
  BLEService* p_bleService = pServer->createService(DEVICE_INFO_SERVICE_UUID);
  pServer->getAdvertising()->addServiceUUID(DEVICE_INFO_SERVICE_UUID);

  // add Device Info service
  BLECharacteristic* p_devInfoCharacteristic = p_bleService->createCharacteristic(
        DEVICE_INFO_CHAR_MODEL_NR_UUID,
        BLECharacteristic::PROPERTY_READ
      );
  p_devInfoCharacteristic->setValue("LibreOx-ESP32");

  p_devInfoCharacteristic = p_bleService->createCharacteristic(
                              DEVICE_INFO_CHAR_SERIAL_NR_UUID,
                              BLECharacteristic::PROPERTY_READ
                            );
  p_devInfoCharacteristic->setValue("LibreOx-A-A-0-0-1");

  p_devInfoCharacteristic = p_bleService->createCharacteristic(
                              DEVICE_INFO_CHAR_FIRMWARE_REV_UUID,
                              BLECharacteristic::PROPERTY_READ
                            );
  p_devInfoCharacteristic->setValue(FW_VERSION);

  p_devInfoCharacteristic = p_bleService->createCharacteristic(
                              DEVICE_INFO_CHAR_HARDWARE_REV_UUID,
                              BLECharacteristic::PROPERTY_READ
                            );
  p_devInfoCharacteristic->setValue(HW_VERSION);

  p_devInfoCharacteristic = p_bleService->createCharacteristic(
                              DEVICE_INFO_CHAR_MANUFACTURER_UUID,
                              BLECharacteristic::PROPERTY_READ
                            );
  p_devInfoCharacteristic->setValue("LibreOx-Org");


  // start the service
  p_bleService->start();
}

void bleAddBatteryService(BLEServer* pServer) {
  // create a BLE service
  BLEService* p_bleService = pServer->createService(BATTERY_SERVICE_UUID);
  pServer->getAdvertising()->addServiceUUID(BATTERY_SERVICE_UUID);

  // add Device Info service
  p_bleBatCharacteristic = p_bleService->createCharacteristic(
                             BATTERY_LEVEL_CHAR_UUID,
                             BLECharacteristic::PROPERTY_READ |
                             BLECharacteristic::PROPERTY_NOTIFY
                           );
  uint8_t uc_batLevel = getRemainingBatCap();
  p_bleBatCharacteristic->setValue(&uc_batLevel, 1);

  // start the service
  p_bleService->start();
}

void bleAddHeartRateService(BLEServer* pServer) {
  // create a BLE service
  BLEService* p_bleService = pServer->createService(HEART_RATE_SERVICE_UUID);

  // add Device Info service
  p_bleHeartRateCharacteristic = p_bleService->createCharacteristic(
                                   HEART_RATE_MEASUREMENT_CHAR_UUID,
                                   BLECharacteristic::PROPERTY_READ |
                                   BLECharacteristic::PROPERTY_NOTIFY
                                 );
  int32_t n_initHr = 0;
  p_bleHeartRateCharacteristic->setValue(n_initHr);

  // start the service
  p_bleService->start();
}

void bleAddPulseOximeterService(BLEServer* pServer) {
  // create a BLE service
  BLEService* p_bleService = pServer->createService(PULSE_OXIMETER_SERVICE_UUID);
  pServer->getAdvertising()->addServiceUUID(PULSE_OXIMETER_SERVICE_UUID);

  // add Device Info service
  p_blePulseOxiCharacteristic = p_bleService->createCharacteristic(
                                  PULSE_OXIMETER_CONT_MEAS_CHAR_UUID,
                                  BLECharacteristic::PROPERTY_READ |
                                  BLECharacteristic::PROPERTY_NOTIFY
                                );
  float f_initSpo2 = 0.0;
  p_blePulseOxiCharacteristic->setValue(f_initSpo2);

  // start the service
  p_bleService->start();
}

void bleAddThermometerService(BLEServer* pServer) {
  // create a BLE service
  BLEService* p_bleService = pServer->createService(THERMOMETER_SERVICE_UUID);
  pServer->getAdvertising()->addServiceUUID(THERMOMETER_SERVICE_UUID);

  // add Device Info service
  p_bleTemperatureMeasCharacteristic = p_bleService->createCharacteristic(
                                         TEMPERATURE_MEASUREMENT_CHAR_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  float f_initTemp = 0.0;
  p_bleTemperatureMeasCharacteristic->setValue(f_initTemp);

  // start the service
  p_bleService->start();
}

void bleUartWritePPGData(uint32_t irReading, uint32_t redReading, uint32_t timeStampMillis) {
#ifdef UART_DATASTYLE_BLUEFRUIT_APP
  p_bleUartTxCharacteristic->setValue(String(String(irReading) + "," + String(redReading) + "," + String(tRun)).c_str());
#else
  uint8_t ble_uart_data[] = {uint8_t((0x00ff0000 & irReading) >> 16), uint8_t((0x0000ff00 & irReading) >> 8), uint8_t(0x000000ff & irReading),
                             uint8_t((0x00ff0000 & redReading) >> 16), uint8_t((0x0000ff00 & redReading) >> 8), uint8_t(0x000000ff & redReading),
                             uint8_t((0xff000000 & timeStampMillis) >> 24), uint8_t((0x00ff0000 & timeStampMillis) >> 16), uint8_t((0x0000ff00 & timeStampMillis) >> 8), uint8_t(0x000000ff & timeStampMillis),
                            };
  p_bleUartTxCharacteristic->setValue(ble_uart_data, 10);
#endif
  p_bleUartTxCharacteristic->notify();
  // delay is should not be necessary, as the ppg sensor call is blockink and running at 50 Hz
  delay(12); // bluetooth stack will go into congestion, if too many packets are sent
}

void bleUartReadData(void) {
  // implementation needed
}

void bleWriteBatteryState(void) {
  uint8_t uc_batLevel = getRemainingBatCap();
  float f_batVoltage = getBatVolt();
#if defined(DEBUG)
  Serial.print(F("Battery level: ")); Serial.println(uc_batLevel);
  Serial.print(F("Battery voltage: ")); Serial.println(f_batVoltage);
#endif
  p_bleBatCharacteristic->setValue(&uc_batLevel, 1);
}

#endif // BLE_COMM


#if defined(SD_CARD_LOGGING)
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

/**
   Symmetric sigmoidal approximation
   https://www.desmos.com/calculator/7m9lu26vpy

   c - c / (1 + k*x/v)^3
*/
uint8_t batteryCapSigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
  // slow
  // uint8_t result = 110 - (110 / (1 + pow(1.468 * (voltage - minVoltage)/(maxVoltage - minVoltage), 6)));

  // steep
  // uint8_t result = 102 - (102 / (1 + pow(1.621 * (voltage - minVoltage)/(maxVoltage - minVoltage), 8.1)));

  // normal
  uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage) / (maxVoltage - minVoltage), 5.5)));
  return result >= 100 ? 100 : result;
}

/**
   Asymmetric sigmoidal approximation
   https://www.desmos.com/calculator/oyhpsu8jnw

   c - c / [1 + (k*x/v)^4.5]^3
*/
uint8_t batteryCapAsigmoidal(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage) {
  uint8_t result = 101 - (101 / pow(1 + pow(1.33 * (voltage - minVoltage) / (maxVoltage - minVoltage) , 4.5), 3));
  return result >= 100 ? 100 : result;
}

float getBatVolt(void) {
  // float f_vBat = analogRead(PIN_VBAT);
  // f_vBat *= VBAT_VOLT_DIV_RATIO;
  // f_vBat *= VBAT_ADC_REF_MILLI_VOLTAGE / 1000.0;
  // f_vBat /= 4096.0;

  // this uses a ESP-API function, which "knows" (probably measures) the internal voltage reference better
  float f_vBat = analogReadMilliVolts(PIN_VBAT);
  f_vBat /= 1000.0;
  f_vBat *= VBAT_VOLT_DIV_RATIO;
  return f_vBat;
}

uint8_t getRemainingBatCap(void) {
  float f_vBat = getBatVolt();
  uint8_t uc_batLvl = batteryCapAsigmoidal(uint16_t(f_vBat * 1000), VBAT_MIN_MILLI_VOLT, VBAT_MAX_MILLI_VOLT);
  return uc_batLvl;
}

void millis_to_hours(uint32_t un_ms, char* ps_timeStr)
{
  char s_iStr[6];
  uint32_t un_secs, un_mins, un_hrs;
  un_secs = un_ms / 1000; // time in seconds
  un_mins = un_secs / 60; // time in minutes
  un_secs -= 60 * un_mins; // leftover seconds
  un_hrs = un_mins / 60; // time in hours
  un_mins -= 60 * un_hrs; // leftover minutes
  itoa(un_hrs, ps_timeStr, 10);
  strcat(ps_timeStr, ":");
  itoa(un_mins, s_iStr, 10);
  strcat(ps_timeStr, s_iStr);
  strcat(ps_timeStr, ":");
  itoa(un_secs, s_iStr, 10);
  strcat(ps_timeStr, s_iStr);
}


void setup() {
  pinMode(PIN_OXI_INT, INPUT);  // connects to the interrupt output pin of the MAX30102

#if defined(SD_CARD_LOGGING)
  //pinMode(PIN_SD_DETECT, INPUT_PULLUP);
  pinMode(PIN_VBAT, INPUT);
  pinMode(PIN_TOGGLE_LED, OUTPUT);
  digitalWrite(PIN_TOGGLE_LED, LOW);

  // set spi interface pins according to the sd card slot pinout
  //SPI.begin(PIN_SD_CS, PIN_SD_MOSI, PIN_SD_MISO, PIN_SD_CLK);
#endif

#if defined(DEBUG)
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.println(F("Sensor init started..."));
#endif

  // initialize the oximeter
  Wire.setPins(PIN_OXI_SDA, PIN_OXI_SCL);
  for (uint8_t i = 0; i < OXI_CONNECT_RETRIES; i++) {
    if (oxi.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
    {
#if defined(DEBUG)
      Serial.println(F(" - MAX30102 initialized!"));
#endif
      break;
    }
    else {
#if defined(DEBUG)
      Serial.print(F(" - "));
      Serial.print(i + 1);
      Serial.println(F(". MAX30102 connection attempt failed... retrying"));
#endif
      delay(500);
      if (i + 1 == OXI_CONNECT_RETRIES) {
#if defined(DEBUG)
        Serial.println(F(" - MAX30102 failed!"));
#endif
        while (1) {
          delay(1000);
        }
      }
    }
  }

#if defined(BLE_COMM)
  // initialize BLE
  bleInit();
#if defined(DEBUG)
  Serial.println(F(" - BLE communication initialized"));
#endif
#endif

  // Configure the oximeter
  oxi.softReset();
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

#if defined(SD_CARD_LOGGING)
  char s_systemStatus[40];
  //if (HIGH == digitalRead(cardDetect)) {
  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!SD.begin(PIN_SD_CS, PIN_SD_MOSI, PIN_SD_MISO, PIN_SD_CLK)) {
    b_sdCardOk = false;
    strncpy(s_systemStatus, "SD card init failed", 19);
  } else {
    b_sdCardOk = true;
    strncpy(s_systemStatus, "SD card init successfull", 24);
  }
  //}

#if defined(DEBUG)
  if (b_sdCardOk)
    Serial.println(F(" - SD card init successful!"));
  else
    Serial.println(F(" - SD card init failed!"));
#endif

  if (b_sdCardOk) {
    uint16_t uw_fileNameCounter = 0;
    char s_fileName[20];
    do {
      //      if(useClock && now.month()<13 && now.day()<32) {
      //        sprintf(fname,"%d-%d_%d.txt",now.month(),now.day(),++count);
      //      } else {
      sprintf(s_fileName, "data_%d.txt", ++uw_fileNameCounter);
      //      }
    } while (SD.exists(s_fileName));
    dataFile = SD.open(s_fileName, FILE_WRITE);
  }

  if (!SD.exists(dataFile.name())) {
#if defined(DEBUG)
    Serial.println(F(" - SD card file creation failed!"));
#endif
    strncpy(s_systemStatus, "SD log file creation failed", 27);
    while (1) // stay here forever
    {
      delay(1000);
    }
  }
  else {
#if defined(DEBUG)
    Serial.print(F(" - SD card file created successfully: "));
    Serial.println(dataFile.name());
#endif
    strncpy(s_systemStatus, "SD log file created", 19);
  }
#endif // SD_CARD_LOGGING

#if defined(SD_CARD_LOGGING)

#if defined(DEBUG)
  Serial.println(F("Sensor init finished!"));
  Serial.println(F("====================="));
  Serial.println();
  strncpy(s_systemStatus, "init successfull - waiting for user", 35);

  Serial.print(F("Firmware version: "));
  Serial.println(FW_VERSION);
  Serial.print(F("System status: "));
  Serial.println(s_systemStatus);
  Serial.print(F("SD Card file: "));
  Serial.println(dataFile.name());
  Serial.print(F("VBat: "));
  Serial.println(getBatVolt());
  Serial.println(F("Press any key to start conversion"));

  while (Serial.available() == 0) //wait until user presses a key
  {
    delay(2000);
  }
  Serial.read();
#endif

  toggleLED(PIN_TOGGLE_LED, b_sdCardOk);
  dataFile.print(F("Firmware version: "));
  dataFile.println(FW_VERSION);
  dataFile.print(F("Sensor state: "));
  dataFile.println(s_systemStatus);
  dataFile.println(MEASUERMENT_DATA_HEADER);

#if defined(SAVE_RAW_DATA)
  uint32_t un_iter;
  // These are headers for the red signal
  for (un_iter = 0; un_iter < BUFFER_SIZE; ++un_iter) {
    dataFile.print(F("\tr-"));
    dataFile.print(un_iter);
  }
  // These are headers for the infrared signal
  for (un_iter = 0; un_iter < BUFFER_SIZE; ++un_iter) {
    dataFile.print(F("\tir-"));
    dataFile.print(un_iter);
  }
  dataFile.println();
#endif // SAVE_RAW_DATA

#endif // SD_CARD_LOGGING 

#if defined(DEBUG) & !defined(SD_CARD_LOGGING) // DEBUG

  while (Serial.available() == 0) //wait until user presses a key
  {
    Serial.println(F("Press any key to start conversion"));
    delay(1000);
  }
  Serial.read();
#endif

#if defined(DEBUG)
  Serial.println(MEASUERMENT_DATA_HEADER);

#if defined(DEBUG_INCL_RAW)
  // These are headers for the red signal
  for (un_iter = 0; un_iter < BUFFER_SIZE; ++un_iter) {
    Serial.print(F("\tr-"));
    Serial.print(un_iter);
  }
  // These are headers for the infrared signal
  for (un_iter = 0; un_iter < BUFFER_SIZE; ++un_iter) {
    Serial.print(F("\tir-"));
    Serial.print(un_iter);
  }
  Serial.println();
#endif // DEBUG_INCL_RAW

#endif // DEBUG

  un_start_t_ms = millis();
}

// Continuously taking samples from MAX30102.
// Heart rate and SpO2 are calculated every ST seconds
void loop() {
  char s_timeStr[10];
  int8_t ch_spo2Valid;         //indicator to show if the SPO2 calculation is valid
  int8_t  ch_hrValid;          //indicator to show if the heart rate calculation is valid
  int32_t n_heartrate;         //heart rate value
  uint32_t un_iter;
  float f_spo2, f_ratio, f_correl;  //SPO2 value

  //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  //read BUFFER_SIZE samples, and determine the signal range
  for (un_iter = 0; un_iter < BUFFER_SIZE; un_iter++)
  {
    // while (digitalRead(PIN_OXI_INT) == 1); //wait until the interrupt pin asserts
    //oxi.check();

    // oxi.check();
    // aun_ir_buffer[un_iter] = oxi.getFIFOIR();
    // aun_red_buffer[un_iter] = oxi.getFIFORed();
    aun_ir_buffer[un_iter] = oxi.getIR();
    aun_red_buffer[un_iter] = oxi.getRed();

#if defined(DEBUG_INCL_RAW)
    Serial.print(un_iter, DEC);
    Serial.print(F("\t"));
    Serial.print(aun_red_buffer[un_iter], DEC);
    Serial.print(F("\t"));
    Serial.println(aun_ir_buffer[un_iter], DEC);
#endif // DEBUG_INCL_RAW

    //oxi.nextSample();
  }

  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &f_spo2, &ch_spo2Valid, &n_heartrate, &ch_hrValid, &f_ratio, &f_correl);
  un_elapsedTime_s = millis() - un_start_t_ms;
  millis_to_hours(un_elapsedTime_s, s_timeStr); // Time in hh:mm:ss format
  un_elapsedTime_s /= 1000; // Time in seconds

  // Read the _chip_ temperature in degrees Celsius
  float f_temperature = oxi.readTemperature();
  float f_batVolt = getBatVolt();
  uint8_t uch_batCap = getRemainingBatCap();

  //save samples and calculation result to SD card
  if (ch_hrValid && ch_spo2Valid) {
#if defined(BLE_COMM)
    p_bleHeartRateCharacteristic->setValue(n_heartrate);
    p_bleHeartRateCharacteristic->notify();
    p_blePulseOxiCharacteristic->setValue(f_spo2);
    p_blePulseOxiCharacteristic->notify();
    p_bleTemperatureMeasCharacteristic->setValue(f_temperature);
    p_bleTemperatureMeasCharacteristic->notify();
#endif

#if defined(SD_CARD_LOGGING)
    ++uch_sdCardDataLines;
    dataFile.print(un_elapsedTime_s, DEC);
    dataFile.print(F("\t"));
    dataFile.print(f_spo2, 2);
    dataFile.print(F("\t"));
    dataFile.print(n_heartrate, DEC);
    dataFile.print(F("\t"));
    dataFile.print(f_temperature, 2);
    dataFile.print(F("\t"));
    dataFile.print(s_timeStr);
    dataFile.print(F("\t"));
    dataFile.print(f_ratio, 2);
    dataFile.print(F("\t"));
    dataFile.print(f_correl, 2);
    dataFile.print(F("\t"));
    dataFile.print(f_batVolt, 2);
    dataFile.print(F("\t"));
    dataFile.println(uch_batCap, DEC);
#if defined(SAVE_RAW_DATA)
    // Save raw data for unusual O2 levels
    for (un_iter = 0; un_iter < BUFFER_SIZE; ++un_iter)
    {
      dataFile.print(F("\t"));
      dataFile.print(aun_red_buffer[un_iter], DEC);
    }
    for (un_iter = 0; un_iter < BUFFER_SIZE; ++un_iter)
    {
      dataFile.print(F("\t"));
      dataFile.print(aun_ir_buffer[un_iter], DEC);
    }
    dataFile.println();
#endif // SAVE_RAW_DATA

    // FLush SD buffer every SD_CARD_DATA_LINE_FLUSH_THRESH points
    if (uch_sdCardDataLines >= SD_DATA_LINE_FLUSH_THRESH) {
      dataFile.flush();
      uch_sdCardDataLines = 0;
    }
#endif // SD_CARD_LOGGING

#if defined(DEBUG) // DEBUG
    Serial.print(un_elapsedTime_s, DEC);
    Serial.print(F("\t"));
    Serial.print(f_spo2, 2);
    Serial.print(F("\t"));
    Serial.print(n_heartrate, DEC);
    Serial.print(F("\t"));
    Serial.print(f_temperature, 2);
    Serial.print(F("\t"));
    Serial.print(s_timeStr);
    Serial.print(F("\t"));
    Serial.print(f_ratio, 2);
    Serial.print(F("\t"));
    Serial.print(f_correl, 2);
    Serial.print(F("\t"));
    Serial.print(f_batVolt, 2);
    Serial.print(F("\t"));
    Serial.println(uch_batCap, DEC);

#if defined(DEBUG_INCL_RAW)
    // Save raw data for unusual O2 levels
    for (un_iter = 0; un_iter < BUFFER_SIZE; ++un_iter)
    {
      Serial.print(F("\t"));
      Serial.print(aun_red_buffer[un_iter], DEC);
    }
    for (un_iter = 0; un_iter < BUFFER_SIZE; ++un_iter)
    {
      Serial.print(F("\t"));
      Serial.print(aun_ir_buffer[un_iter], DEC);
    }
    Serial.println();
#endif // DEBUG_INCL_RAW

#endif // DEBUG
    f_oldSpo2 = f_spo2;
  }

#if defined(BLE_COMM)
  // update the advertised remaining battery capacity
  if (millis() - un_lastBatLevelUpdate > BAT_LEVEL_UPDATE_MS) {
    un_lastBatLevelUpdate = millis();
    bleWriteBatteryState();
  }
#endif
}

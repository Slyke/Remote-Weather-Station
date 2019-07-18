
  
// #include <Esp.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <BME280I2C.h>
#include <Adafruit_ADS1015.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

// EEPROM Memory Addresses and Lengths
#define EE_MAX_WIFI_TRIES_LOC 1
#define EE_MAX_WIFI_TRIES_LEN 1

#define EE_SERIAL_OUT_REQUEST_DETAILS_LOC 2
#define EE_SERIAL_OUT_REQUEST_DETAILS_LEN 1

#define EE_ENABLE_SERIAL_LOC 3
#define EE_ENABLE_SERIAL_LEN 1

#define EE_ENABLE_OLED_LOC 4
#define EE_ENABLE_OLED_LEN 1

#define EE_I2C_SCAN_LOC 5
#define EE_I2C_SCAN_LEN 1

#define EE_DISABLE_TRANSCE_LOC 6
#define EE_DISABLE_TRANSCE_LEN 1

#define EE_SLEEP_MODE_LOC 7
#define EE_SLEEP_MODE_LEN 1

#define EE_DISABLE_POWER_WIFI_LED_LOC 8
#define EE_DISABLE_POWER_WIFI_LED_LEN 1

#define EE_SCROLL_ENABABLED_LOC 9
#define EE_SCROLL_ENABABLED_LEN 1

#define EE_SERIAL_DATA_MODE_LOC 10
#define EE_SERIAL_DATA_MODE_LEN 1

#define EE_SLEEP_TIMER_LOC 14
#define EE_SLEEP_TIMER_LEN 4

#define EE_HTTP_RX_CODE_LOC 20
#define EE_HTTP_RX_CODE_LEN 4

#define EE_SERIAL_CON_SPD_LOC 28
#define EE_SERIAL_CON_SPD_LEN 8

#define EE_STATION_ID_LOC 42
#define EE_STATION_ID_LEN 16

#define EE_SSID_LOC 128
#define EE_SSID_LEN 64

#define EE_STAPSK_LOC 256
#define EE_STAPSK_LEN 64

#define EE_TELEMERTY_POST_URL_LOC 384
#define EE_TELEMERTY_POST_URL_LEN 128

#define EE_I2C_POST_URL_LOC 512
#define EE_I2C_POST_URL_LEN 128

#define EE_GPS_LAT_LOC 1024
#define EE_GPS_LAT_LEN 12

#define EE_GPS_LNG_LOC 1036
#define EE_GPS_LNG_LEN 12

#define EE_GPS_ALT_LOC 1048
#define EE_GPS_ALT_LEN 8

#define EE_READ_GOOD_LOC 777
#define EE_READ_GOOD_LEN 1
#define EE_PROM_VERSION 0x77 // Don't set this to EEPROM_READ_FAILURE

// Defaults (No EEPROM)
#define STATION_ID 1

// Default GPS
#define GPS_LAT 0 // 43.6560079
#define GPS_LNG 0 // -79.3813297
#define GPS_ALT 0

#define TX_SOLUTION 1 // 0 = No Radio, 1 = Integrated Wifi, 2 = NRF24L01

// TX (Wifi):
#define STASSID "" // SSID
#define STAPSK  "" // SSID PSK
#define POST_URL ""
#define I2C_SCAN_POST_URL ""
#define GOOD_HTTP_RESP_CODE_DEFAULT HTTP_CODE_OK // Good HTTP response. Anything other than this number will result in an error being reported.
#define MAX_WIFI_TRIES_DEFAULT 10 // How many times to tr to connect to WIFI
#define SERIAL_OUT_REQUEST_DETAILS_DEFAULT true // Turn on/off transmission debugging. Errors will always serial out.

#define DEEP_SLEEP_TIMER_DEFAULT 2
#define DEEP_SLEEP_MODE_DEFAULT 0

#define SERIAL_CON_SPD_DEFAULT 19200
#define SERIAL_DEBUG 0 // have the ESP8266 debug serial outs

#define DISABLE_POWER_WIFI_LED true

#define SCROLL_ENABABLED true // Have the attached screen automatically scroll


// Pins
#define SW_ENABLE_SERIAL D5
#define SW_I2C_SCAN D6
#define SW_DISABLE_TRANSCE D7

#define EEPROM_ADDR 0x50 // I2C Address
#define EEPROM_READ_FAILURE 0xFF // Returned value on fail. Don't set to 0, 1 or EE_PROM_VERSION
#define OLED_ADDR 0x3C // OLED I2C Address

#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_ADS1115 adc1115;

unsigned int scrollCounter = 0;
unsigned int scrollFrame = 0;
unsigned int lastTxResponse = 0;
unsigned int errorList = 0;
boolean oledOn = false;

BME280I2C::Settings settings(
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::OSR_X1,
  BME280::Mode_Forced,
  BME280::StandbyTime_1000ms,
  BME280::Filter_Off,
  BME280::SpiEnable_False,
  BME280I2C::I2CAddr_0x76
);

BME280I2C bme(settings);

struct RunTimeVariables_s {
  char enableSerialConn;
  char enableOled;
  char enableI2CMode;
  char disableTransceiver;
  unsigned int serialConnSpd;
  unsigned char serialDataMode;
  unsigned char wifiRetryTimes;
  bool powerWifiLed;
  bool serialOutRequestDetails;
  bool scrollEnabled;
  double gpsLat;
  double gpsLng;
  float gpsAlt;
  unsigned int sleepMode;
  unsigned int sleepTimerus;
  unsigned int httpGoodResponse;
  String SSID;
  String PSK;
  String telemetryPostUrl;
  String i2cPostUrl;
} RunTimeVariables_UnInit {
  EEPROM_READ_FAILURE,
  EEPROM_READ_FAILURE,
  EEPROM_READ_FAILURE,
  EEPROM_READ_FAILURE,
  SERIAL_CON_SPD_DEFAULT,
  SERIAL_DEBUG,
  MAX_WIFI_TRIES_DEFAULT,
  DISABLE_POWER_WIFI_LED,
  SERIAL_OUT_REQUEST_DETAILS_DEFAULT,
  SCROLL_ENABABLED,
  GPS_LAT,
  GPS_LNG,
  GPS_ALT,
  DEEP_SLEEP_MODE_DEFAULT,
  DEEP_SLEEP_TIMER_DEFAULT,
  GOOD_HTTP_RESP_CODE_DEFAULT,
  STASSID,
  STAPSK,
  POST_URL,
  I2C_SCAN_POST_URL
};

struct SensorReading_s {
  unsigned char runMode; // 1 = Sleep Enabled, 2 = Serial Enabled, 4 = OLED Enabled, 8 = Disabled Tx, 16 = I2C Scan Mode
  float temp;
  float hum;
  float pres;
  unsigned int lightLevel;
  unsigned int uv;
  unsigned int vIn;
  unsigned int vBat;
  unsigned int vAux;
  unsigned int windDirection;
  unsigned int windSpeed;
  unsigned int cps;
  unsigned int rAlpha;
  unsigned int rBeta;
  unsigned int rGamma;
  double lat;
  double lng;
  float alt;
  int rssi;
  bool rxtxConn;
  unsigned int stationId;
} SensorReading_UnInit {
  0,
  NAN,
  NAN,
  NAN,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  0,
  GPS_LAT,
  GPS_LNG,
  GPS_ALT,
  0,
  false,
  STATION_ID
};

typedef struct SensorReading_s SensorReading;
SensorReading currentReading;

typedef struct RunTimeVariables_s RunTimeVariables;
RunTimeVariables runTimeVariables;

void Reset() {
//  asm volatile ("  jmp 0");
  ESP.reset();
}

void ICACHE_FLASH_ATTR exEepromWriteByte(int deviceAddress, unsigned int memAddress, byte data) {
  int rdata = data;
  delay(25);
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(memAddress >> 8)); // MSB
  Wire.write((int)(memAddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
  delay(25);
}

void ICACHE_FLASH_ATTR exEepromWriteBlock(int deviceAddress, unsigned int memStartAddress, byte* data, byte dataLength) {
  delay(25);
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(memStartAddress >> 8)); // MSB
  Wire.write((int)(memStartAddress & 0xFF)); // LSB
  byte c;
  for ( c = 0; c < dataLength; c++) {
    Wire.write(data[c]);
  }
  delay(25);
  Wire.endTransmission();
}

byte ICACHE_FLASH_ATTR exEepromReadByte(int deviceAddress, unsigned int memAddress) {
  byte rdata = EEPROM_READ_FAILURE;
  delay(25);
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(memAddress >> 8)); // MSB
  Wire.write((int)(memAddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 1);
  if (Wire.available()) rdata = Wire.read();
  delay(25);
  return rdata;
}

byte ICACHE_FLASH_ATTR exEepromReadByte(int deviceAddress, unsigned int memAddress, byte defaultValue) {
  byte rdata = defaultValue;
  delay(25);
  Wire.beginTransmission(deviceAddress);
  Wire.write((int)(memAddress >> 8)); // MSB
  Wire.write((int)(memAddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, 1);
  if (Wire.available()) rdata = Wire.read();
  delay(25);
  return rdata;
}

String ICACHE_FLASH_ATTR readStringFromEeprom(int deviceAddress, unsigned int startAddress, byte stringMaxLength) {
  byte stringIndex = 0;
  String output = "";
  while(stringIndex < stringMaxLength) {
    char buf = exEepromReadByte(deviceAddress, startAddress + stringIndex, EEPROM_READ_FAILURE);
    String temp(buf);
    if (buf < 32 || buf > 126) {
      break;
    }
    output.concat(buf);
    stringIndex++;
  }
  return output;
}

void ICACHE_FLASH_ATTR oledMsgDisplayDelay() {
  if (oledOn) {
    delay(2000);
  }
}

bool ICACHE_FLASH_ATTR findI2CDevice(int deviceAddress) {
  Wire.beginTransmission(deviceAddress);
  byte error = Wire.endTransmission();

  if (error == 0) {
    return true;
  }

  return false;
}

void ICACHE_FLASH_ATTR printSensorDataVerbose(Stream* client) {
  if ((currentReading.runMode & 2) == 2 && client) {
    client->print(F("\nTemp: "));
    client->print(currentReading.temp);
    client->print(F("°C"));
    client->print(F("\tHum: "));
    client->print(currentReading.hum);
    client->print(F("% RH"));
    client->print(F("\tPres: "));
    client->print(currentReading.pres);
    client->print(F("Pa"));
    client->print(F("\tLum: "));
    client->print(currentReading.lightLevel);
    client->print(F("Lux"));
    client->print(F("\tuv: "));
    client->print(currentReading.uv);
    client->print(F("i"));
    client->print(F("\tvIn: "));
    client->print(currentReading.vIn);
    client->print(F("v"));
    client->print(F("\tvBat: "));
    client->print(currentReading.vBat);
    client->print(F("v"));
    client->print(F("\tvAux: "));
    client->print(currentReading.vAux);
    client->print(F("v"));
    client->print(F("\twDir: "));
    client->print(currentReading.windDirection);
    client->print(F("°"));
    client->print(F("\twSpd: "));
    client->print(currentReading.windSpeed);
    client->print(F("km/h"));
    client->print(F("\tcps: "));
    client->print(currentReading.cps);
    client->print(F("c/s"));
    client->print(F("\tAlpha: "));
    client->print(currentReading.rAlpha);
    client->print(F("uSv/h"));
    client->print(F("\tBeta: "));
    client->print(currentReading.rBeta);
    client->print(F("uSv/h"));
    client->print(F("\trGamma: "));
    client->print(currentReading.rGamma);
    client->print(F("uSv/h"));
    client->print(F("\tlat: "));
    client->print(currentReading.lat);
    client->print(F("\tlng: "));
    client->print(currentReading.lng);
    client->print(F("\talt: "));
    client->print(currentReading.alt);
    client->print(F("\tRunM: 0x"));
    if (currentReading.runMode < 0x10) {
      client->print("0");
    }
    client->print(currentReading.runMode, HEX);
    client->print(F("\tRXTX: "));
    client->print(currentReading.rxtxConn) ? F("true") : F("false");
    client->print(F("\tRSSI: "));
    client->print(currentReading.rssi);
    client->print(F("\ttxCode: "));
    client->print(lastTxResponse);
    client->print(F("\tStnId: "));
    client->print(currentReading.stationId);
    client->print(F("."));
  }
}

void ICACHE_FLASH_ATTR printSensorData(Stream* client) {
  if ((currentReading.runMode & 2) == 2 && client) {
    client->print(F("\nCSV: "));
    client->print(currentReading.temp);
    client->print(",");
    client->print(currentReading.hum);
    client->print(",");
    client->print(currentReading.pres);
    client->print(",");
    client->print(currentReading.lightLevel);
    client->print(",");
    client->print(currentReading.uv);
    client->print(",");
    client->print(currentReading.vIn);
    client->print(",");
    client->print(currentReading.vBat);
    client->print(",");
    client->print(currentReading.vAux);
    client->print(",");
    client->print(currentReading.windDirection);
    client->print(",");
    client->print(currentReading.windSpeed);
    client->print(",");
    client->print(currentReading.cps);
    client->print(",");
    client->print(currentReading.lat);
    client->print(",");
    client->print(currentReading.lng);
    client->print(",");
    client->print(currentReading.alt);
    client->print(",");
    client->print(currentReading.cps);
    client->print(",");
    client->print(currentReading.rAlpha);
    client->print(",");
    client->print(currentReading.rBeta);
    client->print(",");
    client->print(currentReading.rGamma);
    client->print(",");
    client->print(currentReading.runMode, HEX);
    client->print(",");
    client->print(currentReading.rxtxConn) ? F("true") : F("false");
    client->print(",");
    client->print(currentReading.rssi);
    client->print(",");
    client->print(lastTxResponse);
    client->print(",");
    client->print(currentReading.stationId);
  }
}

void ICACHE_FLASH_ATTR serialPrint(Stream* client, String dataToPrint) {
  if ((currentReading.runMode & 2) == 2 && client) {
    client->print(dataToPrint);
  }
}

void ICACHE_FLASH_ATTR serialPrint(Stream* client, long dataToPrint) {
  if ((currentReading.runMode & 2) == 2 && client) {
    client->print(dataToPrint);
  }
}

void ICACHE_FLASH_ATTR serialPrint(Stream* client, int dataToPrint, int dataType) {
  if ((currentReading.runMode & 2) == 2 && client) {
    client->print(dataToPrint, dataType);
  }
}

void ICACHE_FLASH_ATTR displayPrint(String dataToPrint) {
  if ((currentReading.runMode & 4) == 4) {
    if (oledOn) {
      display.print(dataToPrint);
      display.display();
    }
  }
}

void ICACHE_FLASH_ATTR displayPrint(long dataToPrint) {
  if ((currentReading.runMode & 4) == 4) {
    if (oledOn) {
      display.print(dataToPrint);
      display.display();
    }
  }
}
void ICACHE_FLASH_ATTR displayPrint(String dataToPrint, bool clearScreen) {
  if ((currentReading.runMode & 4) == 4) {
    if (clearScreen) {
      if (oledOn) {
        display.clearDisplay();
        display.setCursor(0,0);
      }
    }
    displayPrint(dataToPrint);
  }
}

void ICACHE_FLASH_ATTR displayPrint(int dataToPrint, bool clearScreen, bool formatHex) {
  if ((currentReading.runMode & 4) == 4) {
    if (oledOn) {
      if (formatHex) {
        char hexOut[5];
        sprintf(hexOut, "%x", dataToPrint);
        displayPrint(hexOut, clearScreen);
      } else {
        displayPrint((String)dataToPrint, clearScreen);
      }
    }
  }
}

void ICACHE_FLASH_ATTR setupOled(bool startMode) {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    oledOn = false;
  } else {
    if (findI2CDevice(OLED_ADDR)) {
      oledOn = true;
      if (startMode) {
        serialPrint(&Serial, F("\n OLED device found at address 0x3C"));
      }
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.clearDisplay();
      display.setCursor(0,0);
      if (startMode) {
        display.println(F("Booting..."));
        if ((currentReading.runMode & 2) == 2) {
          display.print(F("Serial Speed: "));
          display.println(runTimeVariables.serialConnSpd);
          display.display();
          delay(500);
        }
        display.println(F("Run Mode: "));
        displayPrint(currentReading.runMode, false, true);
      }
      display.display();
    } else {
      oledOn = false;
    }
  }
}

void ICACHE_FLASH_ATTR setupOled() {
  setupOled(false);
}

void ICACHE_FLASH_ATTR displaySensorOled(unsigned int dataToShow) {
  if ((currentReading.runMode & 4) == 4) {
    if (!oledOn) {
      setupOled();
    }

    if (!oledOn) {
      return;
    }
    
    display.clearDisplay();
    display.setCursor(0,0);

    unsigned int stepper = 1;

    if ((dataToShow & stepper) != 0) {
      display.print(F("Temp: "));
      display.print(currentReading.temp);
      display.println("C");
    }
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("Hum: "));
      display.print(currentReading.hum);
      display.println(F("% RH"));
    }
    stepper *= 2;
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("Pres: "));
      display.print(currentReading.pres);
      display.println(F("Pa"));
    }
      if ((dataToShow & stepper) != 0) {
      display.print(F("vAux: "));
      display.print(currentReading.vAux);
      display.println(F("v"));
    }
    stepper *= 2;
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("vIn: "));
      display.print(currentReading.vIn);
      display.println(F("v"));
    }
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("vBat: "));
      display.print(currentReading.vBat);
      display.println(F("v"));
    }
    stepper *= 2;
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("Light: "));
      display.println(currentReading.lightLevel);
    }
    
    if ((dataToShow & stepper) != 0) {
      display.print(F("uv: "));
      display.print(currentReading.uv);
      display.println(F("i"));
    }
    stepper *= 2;
    
    if ((dataToShow & stepper) != 0) {
      display.print(F("wDir: "));
      display.println(currentReading.windDirection);
    }
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("wSpd: "));
      display.print(currentReading.windSpeed);
      display.println(F("km/h"));
    }
    stepper *= 2;
    
    if ((dataToShow & stepper) != 0) {
      display.print(F("cps: "));
      display.print(currentReading.cps);
      display.println(F("c/s"));
    }
    
    if ((dataToShow & stepper) != 0) {
      display.print(F("Lat: "));
      display.println(currentReading.lat);
    }
    stepper *= 2;
    
    if ((dataToShow & stepper) != 0) {
      display.print(F("Lng: "));
      display.println(currentReading.lng);
    }
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("Alt: "));
      display.println(currentReading.alt);
    }
    stepper *= 2;
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("RunM: "));
      char hexOut[5];
      sprintf(hexOut, "%x", currentReading.runMode);
      display.println(hexOut);
    }
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("RXTXConn: "));
      display.println(currentReading.rxtxConn ? F("true") : F("false"));
    }
    stepper *= 2;
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("RSSI: "));
      display.println(currentReading.rssi);
    }
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("txCode: "));
      display.println(lastTxResponse);
    }
    stepper *= 2;
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("StnId: "));
      display.println(currentReading.stationId);
    }
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("\nTemp: "));
      display.print(currentReading.temp);
      display.println(F("C"));
    }
    
    display.display();
  }
}

bool ICACHE_FLASH_ATTR connectToWifi() {
  WiFi.mode(WIFI_STA);
  runTimeVariables.SSID = readStringFromEeprom(EEPROM_ADDR, EE_SSID_LOC, EE_SSID_LEN).c_str();
  runTimeVariables.PSK = readStringFromEeprom(EEPROM_ADDR, EE_STAPSK_LOC, EE_STAPSK_LEN).c_str();
  
  WiFi.begin(runTimeVariables.SSID.c_str(), runTimeVariables.PSK.c_str());
  int wifiTries = 0;

  displayPrint(F("SSID: "), true);
  displayPrint(runTimeVariables.SSID.c_str(), false);
//  displayPrint(F("\nPSK: "), true);
//  displayPrint(runTimeVariables.PSK.c_str(), false);
  displayPrint(F("\nConnecting RX/TX..."), false);
  serialPrint(&Serial, F("\nConnecting to: "));
  serialPrint(&Serial, runTimeVariables.SSID.c_str());
  serialPrint(&Serial, F("\nRX/TX..."));

  while (WiFi.status() != WL_CONNECTED && wifiTries < runTimeVariables.wifiRetryTimes) {
    wifiTries++;
    delay(750 * wifiTries);
    serialPrint(&Serial, ".");
    displayPrint(".");
  }
  
  if (wifiTries >= runTimeVariables.wifiRetryTimes) {
    serialPrint(&Serial, F("\nFailed to connect to wifi"));
    errorList |= 1;
  
    displayPrint(F("\nFailed to connect to wifi"));
    oledMsgDisplayDelay();
    return false;
  } else {
    serialPrint(&Serial, F("\nWifi connected: "));
    serialPrint(&Serial, (String)WiFi.localIP().toString());
    serialPrint(&Serial, F("\tRSSI: "));
    serialPrint(&Serial, (long)WiFi.RSSI());
  
    displayPrint(F("Wifi connected: \n"), true);
    displayPrint((String)WiFi.localIP().toString());
    displayPrint(F("\nRSSI: "));
    displayPrint((long)WiFi.RSSI());
    oledMsgDisplayDelay();
    return true;
  }
}

void ICACHE_FLASH_ATTR getBME280Data() {
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(currentReading.pres, currentReading.temp, currentReading.hum, tempUnit, presUnit);
}

void ICACHE_FLASH_ATTR getGpsData() {
  currentReading.lat = runTimeVariables.gpsLat;
  currentReading.lng = runTimeVariables.gpsLng;
  currentReading.alt = runTimeVariables.gpsAlt;
}

void ICACHE_FLASH_ATTR getGeigerMullerData() {
  currentReading.cps = currentReading.cps;
}

void ICACHE_FLASH_ATTR getAnalogData() {
  currentReading.vIn = adc1115.readADC_SingleEnded(0);
  currentReading.vBat = adc1115.readADC_SingleEnded(1);
  currentReading.lightLevel = adc1115.readADC_SingleEnded(2);
  currentReading.uv = adc1115.readADC_SingleEnded(3);
  currentReading.vAux = analogRead(A0);
}

void ICACHE_FLASH_ATTR getWindData() {
  currentReading.windDirection = currentReading.windDirection;
  currentReading.windSpeed = currentReading.windSpeed;
}

void ICACHE_FLASH_ATTR getRxTxData() {
  currentReading.rxtxConn = WiFi.status() == WL_CONNECTED;
  currentReading.rssi = (long)WiFi.RSSI();
}

void ICACHE_FLASH_ATTR updateCurrentReading() {
  getBME280Data();
  getGpsData();
  getGeigerMullerData();
  getAnalogData();
  getWindData();
  getRxTxData();
}

void ICACHE_FLASH_ATTR txSensorData() {
  if ((currentReading.runMode & 8) == 0) {
    if ((WiFi.status() == WL_CONNECTED)) {
      WiFiClient client;
      HTTPClient http;

      if (http.begin(client, runTimeVariables.telemetryPostUrl.c_str())) {
        http.addHeader(F("Content-Type"), F("application/json"));
        
        StaticJsonDocument<1024> JSONbuffer;
        char JSONmessageBuffer[1024];
        
        JsonObject JSONencoder = JSONbuffer.to<JsonObject>();
        JSONencoder["payload"] = JSONbuffer.to<JsonObject>();
        JSONencoder["payload"][F("rads")] = JSONbuffer.to<JsonObject>();
        JSONencoder["payload"][F("light")] = JSONbuffer.to<JsonObject>();
        JSONencoder["payload"][F("atmosphere")] = JSONbuffer.to<JsonObject>();
        JSONencoder["payload"][F("power")] = JSONbuffer.to<JsonObject>();
        JSONencoder["payload"][F("location")] = JSONbuffer.to<JsonObject>();
        JSONencoder["payload"][F("meta")] = JSONbuffer.to<JsonObject>();
        
        JSONencoder["payload"][F("atmosphere")][F("temperature")] = currentReading.temp;
        JSONencoder["payload"][F("atmosphere")][F("humidity")] = currentReading.hum;
        JSONencoder["payload"][F("atmosphere")][F("pressure")] = currentReading.pres;
        
        JSONencoder["payload"][F("atmosphere")][F("windDirection")] = currentReading.windDirection;
        JSONencoder["payload"][F("atmosphere")][F("windSpeed")] = currentReading.windSpeed;

        JSONencoder["payload"][F("power")][F("voltageIn")] = currentReading.vIn;
        JSONencoder["payload"][F("power")][F("voltageBattery")] = currentReading.vBat;
        JSONencoder["payload"][F("power")][F("voltageAuxiliary")] = currentReading.vAux;

        JSONencoder["payload"][F("light")][F("visible")] = currentReading.lightLevel;
        JSONencoder["payload"][F("light")][F("uv")] = currentReading.uv;
        
        JSONencoder["payload"][F("rads")][F("countsPerSecond")] = currentReading.cps;
        JSONencoder["payload"][F("rads")][F("alpha")] = currentReading.rAlpha;
        JSONencoder["payload"][F("rads")][F("beta")] = currentReading.rBeta;
        JSONencoder["payload"][F("rads")][F("gamma")] = currentReading.rGamma;
        
        JSONencoder["payload"][F("meta")][F("rssi")] = currentReading.rssi;
        JSONencoder["payload"][F("meta")][F("transmitCode")] = lastTxResponse;
        JSONencoder["payload"][F("meta")][F("stationId")] = currentReading.stationId;
        JSONencoder["payload"][F("meta")][F("runMode")] = currentReading.runMode;
        JSONencoder["payload"][F("meta")][F("errorList")] = errorList;

        JSONencoder["payload"][F("location")][F("latitude")] = currentReading.lat;
        JSONencoder["payload"][F("location")][F("longitude")] = currentReading.lng;
        JSONencoder["payload"][F("location")][F("altitude")] = currentReading.alt;
        
        serializeJsonPretty(JSONencoder, JSONmessageBuffer);
        
        lastTxResponse = http.POST(JSONmessageBuffer);
        if (runTimeVariables.serialOutRequestDetails == 1) {
          serialPrint(&Serial, F("\nSending POST Request: "));
          serialPrint(&Serial, runTimeVariables.telemetryPostUrl.c_str());
        }
        
        if (lastTxResponse > 0) {
          if (runTimeVariables.serialOutRequestDetails == 1) {
            serialPrint(&Serial, F(" HTTP Response: "));
            serialPrint(&Serial, (String)lastTxResponse);
          }
          
          if (lastTxResponse == runTimeVariables.httpGoodResponse) {
            if (runTimeVariables.serialOutRequestDetails == 1) {
              String payload = http.getString();
              serialPrint(&Serial, F("\nResponse Body: \n"));
              serialPrint(&Serial, payload);
            }
          }
        } else {
          serialPrint(&Serial, F("\nHTTP Error: "));
          serialPrint(&Serial, http.errorToString(lastTxResponse).c_str());
        }
        http.end();
      }
    }
  }
}

void ICACHE_FLASH_ATTR txI2CData(int i2cAddresses[], int i2cAdressSize, int i2cAddressesErrors[], int i2cAdressErrorSize) {
  if ((currentReading.runMode & 8) == 0) {
    if ((WiFi.status() == WL_CONNECTED)) {
      WiFiClient client;
      HTTPClient http;

      if (http.begin(client, runTimeVariables.i2cPostUrl.c_str())) {
        http.addHeader(F("Content-Type"), F("application/json"));

        StaticJsonDocument<512> JSONbuffer;
        char JSONmessageBuffer[512];

        JsonObject JSONencoder = JSONbuffer.to<JsonObject>();
        JsonArray foundDevicesArray = JSONbuffer.to<JsonArray>();
        JsonArray errorDevicesArray = JSONbuffer.to<JsonArray>();
        
        JSONencoder["scanResults"] = JSONbuffer.to<JsonObject>();
        
        for (int i = 0; i < i2cAdressSize ; i++) {
          foundDevicesArray.add(i2cAddresses[i]);
        }

        for (int i = 0; i < i2cAdressErrorSize ; i++) {
          errorDevicesArray.add(i2cAddressesErrors[i]);
        }

        JSONencoder["scanResults"]["foundDevices"] = foundDevicesArray;
        JSONencoder["scanResults"]["errorDevices"] = errorDevicesArray;

        serializeJsonPretty(JSONencoder, JSONmessageBuffer);
        
        lastTxResponse = http.POST(JSONmessageBuffer);
        if (runTimeVariables.serialOutRequestDetails == 1) {
          serialPrint(&Serial, F("\nSending POST Request: "));
          serialPrint(&Serial, runTimeVariables.i2cPostUrl.c_str());
        }
        
        if (lastTxResponse > 0) {
          if (runTimeVariables.serialOutRequestDetails == 1) {
            serialPrint(&Serial, F(" HTTP Response: "));
            serialPrint(&Serial, (String)lastTxResponse);
          }
          
          if (lastTxResponse == runTimeVariables.httpGoodResponse) {
            if (runTimeVariables.serialOutRequestDetails == 1) {
              String payload = http.getString();
              serialPrint(&Serial, F("\nResponse Body: \n"));
              serialPrint(&Serial, payload);
            }
          }
        } else {
          serialPrint(&Serial, F("\nHTTP Error: "));
          serialPrint(&Serial, http.errorToString(lastTxResponse).c_str());
        }
        http.end();
      }
    }
  }
}

void ICACHE_FLASH_ATTR scanI2CDevices() {
  byte error, address;
  int nDevices;
  int devicesList[127];
  int devicesListLength = 0;
  int devicesErrorList[127];
  int devicesErrorListLength = 0;
  
  serialPrint(&Serial, F("\nScanning..."));

  if (currentReading.runMode & 16 == 16) {
    displayPrint(" ", true);
  }
  
  displayPrint(F("I2C Scan...\n"), true);
  delay(500);
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      serialPrint(&Serial, F("\nI2C device found at address 0x"));
      displayPrint("0x");
      if (address < 16) {
        serialPrint(&Serial, "0");
        displayPrint("0");
      }
      devicesList[devicesListLength] = address;
      devicesListLength++;
      serialPrint(&Serial, address, HEX);
      displayPrint(address, false, true);
      displayPrint(", ");
      nDevices++;
    }
    else if (error == 4) {
      devicesErrorList[devicesErrorListLength] = address;
      devicesErrorListLength++;
      serialPrint(&Serial, F("\nUnknown error at address 0x"));
      displayPrint("ERx");
      if (address < 16) {
        serialPrint(&Serial, "0");
        displayPrint("0");
      }
      serialPrint(&Serial, address, HEX);
      displayPrint(address, false, true);
      displayPrint(", ");
    }
  }
  if (nDevices == 0) {
    serialPrint(&Serial, F("\nNo I2C devices found"));
    displayPrint(F("No I2C devices found"));
    delay(5000);
  } else {
    serialPrint(&Serial, F("\nDone\n"));
    displayPrint(F("Fin~"));
    txI2CData(devicesList, devicesListLength, devicesErrorList, devicesErrorListLength);
    delay(5000);
  }
}

bool ICACHE_FLASH_ATTR setupSerial() {
  if (Serial) {
    return true;
  } else {
    Serial.begin(runTimeVariables.serialConnSpd);
    Serial.setDebugOutput(1);
    delay(10);
    Serial.setDebugOutput(SERIAL_DEBUG);
    if (!SERIAL_DEBUG) {
      system_set_os_print(0);
    }
  }

  if ((currentReading.runMode & 2) == 2) {
    int i = 0;
    while (!Serial) {
      i++;
      if (i >= 10) {
        return false;
      }
    }
  }

  return true;
}

bool ICACHE_FLASH_ATTR readEepromSettings() {
  if (exEepromReadByte(EEPROM_ADDR, EE_READ_GOOD_LOC, EEPROM_READ_FAILURE) == EE_PROM_VERSION) {
    runTimeVariables.wifiRetryTimes = exEepromReadByte(EEPROM_ADDR, EE_MAX_WIFI_TRIES_LOC, (byte)runTimeVariables.wifiRetryTimes);
    runTimeVariables.enableOled = exEepromReadByte(EEPROM_ADDR, EE_ENABLE_OLED_LOC, runTimeVariables.enableOled);
    runTimeVariables.enableSerialConn = exEepromReadByte(EEPROM_ADDR, EE_ENABLE_SERIAL_LOC, runTimeVariables.enableSerialConn);
    runTimeVariables.enableI2CMode = exEepromReadByte(EEPROM_ADDR, EE_I2C_SCAN_LOC, runTimeVariables.enableI2CMode);
    runTimeVariables.disableTransceiver = exEepromReadByte(EEPROM_ADDR, EE_DISABLE_TRANSCE_LOC, runTimeVariables.disableTransceiver);
    runTimeVariables.serialDataMode = exEepromReadByte(EEPROM_ADDR, EE_SERIAL_DATA_MODE_LOC, runTimeVariables.serialDataMode);
    runTimeVariables.powerWifiLed = exEepromReadByte(EEPROM_ADDR, EE_DISABLE_POWER_WIFI_LED_LOC, runTimeVariables.powerWifiLed);
    runTimeVariables.serialOutRequestDetails = exEepromReadByte(EEPROM_ADDR, EE_SERIAL_OUT_REQUEST_DETAILS_LOC, runTimeVariables.serialOutRequestDetails);
    runTimeVariables.scrollEnabled = exEepromReadByte(EEPROM_ADDR, EE_SCROLL_ENABABLED_LOC, runTimeVariables.scrollEnabled);

    runTimeVariables.PSK = readStringFromEeprom(EEPROM_ADDR, EE_STAPSK_LOC, EE_STAPSK_LEN).c_str();
    runTimeVariables.telemetryPostUrl = readStringFromEeprom(EEPROM_ADDR, EE_TELEMERTY_POST_URL_LOC, EE_TELEMERTY_POST_URL_LEN).c_str();
    runTimeVariables.i2cPostUrl = readStringFromEeprom(EEPROM_ADDR, EE_I2C_POST_URL_LOC, EE_I2C_POST_URL_LEN).c_str();

    runTimeVariables.powerWifiLed = exEepromReadByte(EEPROM_ADDR, EE_DISABLE_POWER_WIFI_LED_LOC, EEPROM_READ_FAILURE);

    runTimeVariables.serialConnSpd = (int)strtol(readStringFromEeprom(EEPROM_ADDR, EE_SERIAL_CON_SPD_LOC, EE_SERIAL_CON_SPD_LEN).c_str(), 0, 10);

    runTimeVariables.gpsLat = (double)strtod(readStringFromEeprom(EEPROM_ADDR, EE_GPS_LAT_LOC, EE_GPS_LAT_LEN).c_str(), 0);
    runTimeVariables.gpsLng = (double)strtod(readStringFromEeprom(EEPROM_ADDR, EE_GPS_LNG_LOC, EE_GPS_LNG_LEN).c_str(), 0);
    runTimeVariables.gpsAlt = (float)strtof(readStringFromEeprom(EEPROM_ADDR, EE_GPS_ALT_LOC, EE_GPS_ALT_LEN).c_str(), 0);

    runTimeVariables.sleepMode = exEepromReadByte(EEPROM_ADDR, EE_SLEEP_MODE_LOC, (byte)runTimeVariables.sleepMode);

    runTimeVariables.sleepTimerus = (int)strtol(readStringFromEeprom(EEPROM_ADDR, EE_SLEEP_TIMER_LOC, EE_SLEEP_TIMER_LEN).c_str(), 0, 10);
    Serial.println("\n res");
    Serial.println(readStringFromEeprom(EEPROM_ADDR, EE_SLEEP_TIMER_LOC, EE_SLEEP_TIMER_LEN));

    runTimeVariables.httpGoodResponse = (int)strtol(readStringFromEeprom(EEPROM_ADDR, EE_HTTP_RX_CODE_LOC, EE_HTTP_RX_CODE_LEN).c_str(), 0, 10);

    return true;
  }

  return false;
}

void ICACHE_FLASH_ATTR setRunMode() {
  bool enableSerialComm = digitalRead(SW_ENABLE_SERIAL);
  bool disableTransceiver = digitalRead(SW_DISABLE_TRANSCE);
  bool enableI2CScanner = digitalRead(SW_I2C_SCAN);

  bool enableOledOutput = true;
  
  if (readEepromSettings()) {
    if (runTimeVariables.enableOled == 1) {
      enableOledOutput = true;
    } else if (runTimeVariables.enableOled == 0) {
      enableOledOutput = false;
    }
    
    if (runTimeVariables.disableTransceiver == 1) {
      disableTransceiver = true;
    } else if (runTimeVariables.disableTransceiver == 0) {
      disableTransceiver = false;
    }
  }

  if (enableSerialComm) {
    currentReading.runMode |= 2;
  } else {
    currentReading.runMode &= ~(1 << 1);
  }
  
  if (enableOledOutput) {
    currentReading.runMode |= 4;
  } else {
    currentReading.runMode &= ~(1 << 2);
  }
  
  if (disableTransceiver) {
    currentReading.runMode |= 8;
  } else {
    currentReading.runMode &= ~(1 << 3);
  }
  
  if (enableI2CScanner) {
    currentReading.runMode |= 16;
  } else {
    currentReading.runMode &= ~(1 << 4);
  }

  if ((currentReading.runMode & 2) == 0 && (currentReading.runMode & 4) == 0) {
    currentReading.runMode &= ~(1 << 0);
  }
}

void ICACHE_FLASH_ATTR sleepExec() {
  if ((currentReading.runMode & 1) == 0) {
    
    if (oledOn) {
      delay(500);
      displayPrint(F("Entering sleep mode.."), true);
      delay(2000);
      displayPrint(F(" "), true);
    }
    
    if (runTimeVariables.sleepMode == 1) {
      if (TX_SOLUTION == 1) {
        serialPrint(&Serial, F("\n Entering deep sleep mode..."));
//        ESP.deepSleep(runTimeVariables.sleepTimerus * 1e6); // deepSleep is in us
Serial.println("\n timer: ");
Serial.println(runTimeVariables.sleepTimerus);
Serial.println(runTimeVariables.sleepTimerus * 1e6);
        ESP.deepSleep(5e6); // deepSleep is in us
      } else {
        serialPrint(&Serial, F("\n Deep sleep mode (no RF)..."));
        ESP.deepSleep(runTimeVariables.sleepTimerus * 1e6, RF_DISABLED); // deepSleep is in us
      }
    } else if (runTimeVariables.sleepMode == 2) {
      serialPrint(&Serial, F("\n Entering light sleep mode..."));
      if (TX_SOLUTION == 1) {
        WiFi.forceSleepBegin();
      }
      
      delay(runTimeVariables.sleepTimerus * 1000); // Delay is in ms
      
      if (TX_SOLUTION == 1) {
        WiFi.forceSleepWake();
      }
      Reset();
    } else if (runTimeVariables.sleepMode == 0) {
      serialPrint(&Serial, F("\n NOPing no sleep mode"));
      delay(50);
    }
  }
}

void ICACHE_FLASH_ATTR setup() {

  Wire.begin();

  currentReading = SensorReading_UnInit;
  runTimeVariables = RunTimeVariables_UnInit;

  setRunMode();
  
  if (runTimeVariables.powerWifiLed == 1) {
    wifi_status_led_uninstall();
    pinMode(LED_BUILTIN, INPUT);
  } else {
    pinMode(LED_BUILTIN, OUTPUT);
  }
  
  pinMode(SW_ENABLE_SERIAL, INPUT);
  pinMode(SW_DISABLE_TRANSCE, INPUT);
  pinMode(SW_I2C_SCAN, INPUT);

  if ((currentReading.runMode & 2) == 2) {
    if (setupSerial()) {
      serialPrint(&Serial, F("\nBooting weather station..."));
      serialPrint(&Serial, F("\nRun Mode: "));
      serialPrint(&Serial, (int)currentReading.runMode, HEX);

      serialPrint(&Serial, F("\nEEPROM Version: "));
      serialPrint(&Serial, EE_PROM_VERSION, HEX);
      serialPrint(&Serial, F("\nEEPROM Read: "));
      const char eePromResult = exEepromReadByte(EEPROM_ADDR, EE_READ_GOOD_LOC);
      if (eePromResult == EE_PROM_VERSION) {
        serialPrint(&Serial, F(" Success:  "));
      } else {
        serialPrint(&Serial, F(" Failure:  "));
      }
      serialPrint(&Serial, (int)eePromResult, HEX);
    }
  }

  if ((currentReading.runMode & 4) == 4) {
    setupOled(true);
  }

  if ((currentReading.runMode & 8) == 0) {
    bool wifiConnected;

    if ((currentReading.runMode & 1) == 1) {
      delay(500);
    }
    
    wifiConnected = connectToWifi();

    if ((currentReading.runMode & 1) == 1) {
      delay(1000);
    }
    
    if (!wifiConnected && (currentReading.runMode & 1) == 0) {
      serialPrint(&Serial, F("\n Error initialising connectivity."));
      displayPrint(F("Wifi Error."), true);
      errorList |= 4;
      sleepExec();
    }
  }

  int rslt;
  while ((rslt = bme.begin()) != true) {
    serialPrint(&Serial, F("\n Error initialising BME280 device. "));
    serialPrint(&Serial, F("I2C Slave Address: "));
    serialPrint(&Serial, (int)settings.bme280Addr, HEX);
    displayPrint(F("BME280 Error."), true);
    errorList |= 8;
    oledMsgDisplayDelay();
  }

  if (bme.chipModel() == BME280::ChipModel_BME280) {
    serialPrint(&Serial, F("\n BME280 device found with chip model: "));
    serialPrint(&Serial, (int)bme.chipModel(), HEX);
  } else {
    serialPrint(&Serial, F("\n Invalid chip model found: "));
    serialPrint(&Serial, (String)bme.chipModel());
    serialPrint(&Serial, F("  Will attempt to use, but garbage may ensue. "));
    displayPrint(F("Bad BME280 Chip Model"), true);
    oledMsgDisplayDelay();
    errorList |= 16;
  }

  settings.tempOSR = BME280::OSR_X4;
  bme.setSettings(settings);

  adc1115.begin();
  
  updateCurrentReading();

  if ((currentReading.runMode & 8) == 0 && (currentReading.runMode & 16) == 0) {
    displayPrint("TXing telemetry...", true);
    txSensorData();
    if (TX_SOLUTION == 1) {
      if (lastTxResponse == runTimeVariables.httpGoodResponse) {
        displayPrint("\nTX Send Acknowledged.");
      } else {
        displayPrint("\nTX Failed: ");
        displayPrint(lastTxResponse);
      }
    } else if (TX_SOLUTION == 2) {
      displayPrint("\nTelemetry Sent");
    }
    oledMsgDisplayDelay();
  }

  if ((currentReading.runMode & 16) == 0 && (currentReading.runMode & 8) == 0) {
    sleepExec();
  }
}

void ICACHE_FLASH_ATTR loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  
  setRunMode();

  if ((currentReading.runMode & 2) == 2) {
    setupSerial();
  } else {
    if (Serial) {
      Serial.end();
    }
  }
  
  if ((currentReading.runMode & 16) == 16) {
    scrollFrame = 0;
    scrollCounter = 0;
    scanI2CDevices();
  } else {
    if ((currentReading.runMode & 4) == 4) {
      scrollFrame++;
      if (scrollFrame > 6) {
        scrollCounter++;
        scrollFrame = 0;
  
        if (scrollCounter > 8) {
          scrollCounter = 0;
        }
      }
      displaySensorOled((unsigned int)(3 << scrollCounter));
    } else {
      scrollFrame = 0;
      scrollCounter = 0;
    }
    updateCurrentReading();

    if (runTimeVariables.serialDataMode == 0) {
      printSensorDataVerbose(&Serial);
    } else {
      printSensorData(&Serial);
    }
    
    if ((currentReading.runMode & 8) == 0) {
      txSensorData();
    }

    if ((currentReading.runMode & 16) == 0 && (currentReading.runMode & 8) == 0) {
      sleepExec();
    }
  }
}

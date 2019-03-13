
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

#define EE_SLEEP_TIMER_LOC 7
#define EE_SLEEP_TIMER_LEN 1

#define EE_SLEEP_MODE_LOC 8
#define EE_SLEEP_MODE_LEN 1

#define EE_DISABLE_POWER_WIFI_LED_LOC 9
#define EE_DISABLE_POWER_WIFI_LED_LEN 1

#define EE_SCROLL_ENABABLED_LOC 10
#define EE_SCROLL_ENABABLED_LEN 1

#define EE_SERIAL_CON_SPD_LOC 11
#define EE_SERIAL_CON_SPD_LEN 8

#define EE_STATION_ID_LOC 21
#define EE_STATION_ID_LEN 4

#define EE_SSID_LOC 30
#define EE_SSID_LEN 30

#define EE_STAPSK_LOC 60
#define EE_STAPSK_LEN 30

#define EE_TELEMERTY_POST_URL_LOC 90
#define EE_TELEMERTY_POST_URL_LEN 128

#define EE_I2C_POST_URL_LOC 218
#define EE_I2C_POST_URL_LEN 128

#define EE_READ_GOOD_LOC 777
#define EE_READ_GOOD_LEN 1

// Defaults (No EEPROM)
#define STATION_ID 1

// Default GPS
#define GPS_LAT 43.6560079
#define GPS_LNG -79.3813297
#define GPS_ALT 0

// TX (Wifi):
#define STASSID "BELL457" // SSID
#define STAPSK  "6DF2572754DF" // SSID PSK
#define POST_URL "http://192.168.2.23:3000/update"
#define I2C_SCAN_POST_URL "http://192.168.2.23:3000/i2c"
#define GOOD_HTTP_RESP_CODE HTTP_CODE_OK // Good HTTP response. Anything other than this number will result in an error being reported.
#define MAX_WIFI_TRIES 10 // How many times to tr to connect to WIFI
#define SERIAL_OUT_REQUEST_DETAILS false // Turn on/off transmission debugging. Errors will always serial out.

#define DEEP_SLEEP_TIMER 5e6
#define DEEP_SLEEP_MODE_1 deepSleep

#define SERIAL_CON_SPD 19200
#define SERIAL_DEBUG 0 // have the ESP8266 debug serial outs

#define DISABLE_POWER_WIFI_LED true

#define SCROLL_ENABABLED true // Have the attached screen automatically scroll


// Pins
#define SW_ENABLE_SERIAL D5
#define SW_ENABLE_OLED D6
#define SW_I2C_SCAN D7
#define SW_DISABLE_TRANSCE D4

#define EEPROM_ADDR 0x50 // I2C Address
#define EEPROM_READ_FAILURE 0xFF // Returned value on fail

#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Adafruit_ADS1115 adc1115;

unsigned int scrollCounter = 0;
unsigned int scrollFrame = 0;
unsigned int lastTxResponse = 0;
unsigned int errorList = 0;

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
  unsigned char serialDebugOut;
  unsigned char wifiRetryTimes;
  bool powerWifiLed;
  bool serialOutRequestDetails;
  bool scrollEnabled;
  float gpsLat;
  float gpsLng;
  int alt;
  unsigned int sleepMode;
  unsigned int sleepTimerus;
  unsigned int httpGoodResponse;
  unsigned char SSID [EE_SSID_LEN];
  unsigned char PSK [EE_STAPSK_LEN];
  unsigned char telemetryPostUrl [EE_I2C_POST_URL_LEN];
  char i2cPostUrl [EE_TELEMERTY_POST_URL_LEN];
} RunTimeVariables_UnInit {
  EEPROM_READ_FAILURE,
  EEPROM_READ_FAILURE,
  EEPROM_READ_FAILURE,
  EEPROM_READ_FAILURE,
  SERIAL_CON_SPD,
  SERIAL_DEBUG,
  MAX_WIFI_TRIES,
  DISABLE_POWER_WIFI_LED,
  SERIAL_OUT_REQUEST_DETAILS,
  SCROLL_ENABABLED,
  GPS_LAT,
  GPS_LNG,
  GPS_ALT,
  5,
  1,
  GOOD_HTTP_RESP_CODE,
  STASSID,
  STAPSK,
  POST_URL,
  I2C_SCAN_POST_URL
};

struct SensorReading_s {
  unsigned char runMode;
  float temp;
  float hum;
  float pres;
  unsigned int lightLevel;
  unsigned int vIn;
  unsigned int vBat;
  unsigned int vAux;
  unsigned int vAux2;
  unsigned int windDirection;
  unsigned int windSpeed;
  unsigned int cps;
  double lat;
  double lng;
  int alt;
  unsigned int rssi;
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

void exEepromWriteBlock(int deviceAddress, unsigned int memStartAddress, byte* data, byte dataLength) {
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

byte exEepromReadByte(int deviceAddress, unsigned int memAddress) {
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

byte exEepromReadByte(int deviceAddress, unsigned int memAddress, byte defaultValue) {
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
    client->print(F("\tvIn: "));
    client->print(currentReading.vIn);
    client->print(F("v"));
    client->print(F("\tvBat: "));
    client->print(currentReading.vBat);
    client->print(F("v"));
    client->print(F("\tvAux: "));
    client->print(currentReading.vAux);
    client->print(F("v"));
    client->print(F("\tvAux2: "));
    client->print(currentReading.vAux2);
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
    client->print(F("\n"));
    client->print(currentReading.temp);
    client->print(",");
    client->print(currentReading.hum);
    client->print(",");
    client->print(currentReading.pres);
    client->print(",");
    client->print(currentReading.lightLevel);
    client->print(",");
    client->print(currentReading.vIn);
    client->print(",");
    client->print(currentReading.vBat);
    client->print(",");
    client->print(currentReading.vAux);
    client->print(",");
    client->print(currentReading.vAux2);
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
    display.print(dataToPrint);
    display.display();
  }
}

void ICACHE_FLASH_ATTR displayPrint(long dataToPrint) {
  if ((currentReading.runMode & 4) == 4) {
    display.print(dataToPrint);
    display.display();
  }
}
void ICACHE_FLASH_ATTR displayPrint(String dataToPrint, bool clearScreen) {
  if ((currentReading.runMode & 4) == 4) {
    if (clearScreen) {
      display.clearDisplay();
      display.setCursor(0,0);
    }
    displayPrint(dataToPrint);
  }
}

void ICACHE_FLASH_ATTR displayPrint(int dataToPrint, bool clearScreen, bool formatHex) {
  if ((currentReading.runMode & 4) == 4) {
    if (formatHex) {
      char hexOut[5];
      sprintf(hexOut, "%x", dataToPrint);
      displayPrint(hexOut, clearScreen);
    } else {
      displayPrint((String)dataToPrint, clearScreen);
    }
  }
}

void ICACHE_FLASH_ATTR setupOled(bool startMode) {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    if (startMode) {
      serialPrint(&Serial, F("\n Error initialising OLED Display."));
    }
    errorList |= 2;
  } else {
    if (startMode) {
      serialPrint(&Serial, F("\n OLED device found at address 0x3C"));
    }
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.setCursor(0,0);
    if (startMode) {
      display.println(F("Booting...\n"));
      display.println(F("Run Mode: "));
      displayPrint(currentReading.runMode, false, true);
    }
    display.display();
  }
}

void ICACHE_FLASH_ATTR setupOled() {
  setupOled(false);
}

void ICACHE_FLASH_ATTR displaySensorOled(unsigned int dataToShow) {
  if ((currentReading.runMode & 4) == 4) {
    setupOled();
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
      Serial.print("\n112");
    }
    stepper *= 2;
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("Pres: "));
      display.print(currentReading.pres);
      display.println(F("Pa"));
    }
  
    if ((dataToShow & stepper) != 0) {
      display.print(F("Light: "));
      display.println(currentReading.lightLevel);
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
      display.print(F("vAux: "));
      display.print(currentReading.vAux);
      display.println(F("v"));
    }
    
    if ((dataToShow & stepper) != 0) {
      display.print(F("vAux2: "));
      display.print(currentReading.vAux2);
      display.println(F("v"));
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
  WiFi.begin(STASSID, STAPSK);

  int wifiTries = 0;

  displayPrint(F("Connecting RX/TX...\n"), true);
  serialPrint(&Serial, F("\nConnecting RX/TX...\n"));

  while (WiFi.status() != WL_CONNECTED && wifiTries < MAX_WIFI_TRIES) {
    wifiTries++;
    delay(1000 * wifiTries);
    serialPrint(&Serial, ".");
    displayPrint(".");
  }
  
  if (wifiTries >= MAX_WIFI_TRIES) {
    serialPrint(&Serial, F("\nFailed to connect to wifi"));
    errorList |= 1;
  
    displayPrint(F("\nFailed to connect to wifi"));
    delay(2000);
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

    return true;
  }
}

void ICACHE_FLASH_ATTR getBME280Data() {
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(currentReading.pres, currentReading.temp, currentReading.hum, tempUnit, presUnit);
}

void ICACHE_FLASH_ATTR getLightLevelData() {
  currentReading.lightLevel = analogRead(A0);
}

void ICACHE_FLASH_ATTR getGpsData() {
  currentReading.lat = currentReading.lat;
  currentReading.lng = currentReading.lng;
  currentReading.alt = currentReading.alt;
}

void ICACHE_FLASH_ATTR getGeigerMullerData() {
  currentReading.cps = currentReading.cps;
}

void ICACHE_FLASH_ATTR getAnalogData() {
  currentReading.vIn = adc1115.readADC_SingleEnded(0);
  currentReading.vBat = adc1115.readADC_SingleEnded(1);
  currentReading.vAux = adc1115.readADC_SingleEnded(2);
  currentReading.vAux2 = adc1115.readADC_SingleEnded(3);
}

void ICACHE_FLASH_ATTR getWindData() {
  currentReading.windDirection = currentReading.windDirection;
  currentReading.windSpeed = currentReading.windSpeed;
}

void ICACHE_FLASH_ATTR getRxTxData() {
  currentReading.rxtxConn = WiFi.status() == WL_CONNECTED;
  currentReading.rssi = WiFi.RSSI();
}

void ICACHE_FLASH_ATTR updateCurrentReading() {
  getLightLevelData();
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
    
      if (http.begin(client, POST_URL)) {
        http.addHeader(F("Content-Type"), F("application/json"));
        
        StaticJsonBuffer<512> JSONbuffer;
        char JSONmessageBuffer[512];
        
        JsonObject& JSONencoder = JSONbuffer.createObject();
        JSONencoder["payload"] = JSONbuffer.createObject();
        JSONencoder["payload"][F("temperature")] = currentReading.temp;
        JSONencoder["payload"][F("humidity")] = currentReading.hum;
        JSONencoder["payload"][F("pressure")] = currentReading.pres;
        JSONencoder["payload"][F("light")] = currentReading.lightLevel;
        JSONencoder["payload"][F("voltageIn")] = currentReading.vIn;
        JSONencoder["payload"][F("voltageBattery")] = currentReading.vBat;
        JSONencoder["payload"][F("voltageAuxiliary")] = currentReading.vAux;
        JSONencoder["payload"][F("voltageAuxiliary2")] = currentReading.vAux2;
        JSONencoder["payload"][F("windDirection")] = currentReading.windDirection;
        JSONencoder["payload"][F("windSpeed")] = currentReading.windSpeed;
        JSONencoder["payload"][F("countsPerSecond")] = currentReading.cps;
        JSONencoder["payload"][F("latitude")] = currentReading.lat;
        JSONencoder["payload"][F("longitude")] = currentReading.lng;
        JSONencoder["payload"][F("altitude")] = currentReading.alt;
        JSONencoder["payload"][F("rssi")] = currentReading.rssi;
        JSONencoder["payload"][F("transmitCode")] = lastTxResponse;
        JSONencoder["payload"][F("stationId")] = currentReading.stationId;
        JSONencoder["payload"][F("runMode")] = currentReading.runMode;
        JSONencoder["payload"][F("errorList")] = errorList;

        JSONencoder.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
        
        lastTxResponse = http.POST(JSONmessageBuffer);
        if (SERIAL_OUT_REQUEST_DETAILS) {
          serialPrint(&Serial, F("\nSending POST Request..."));
        }
        
        if (lastTxResponse > 0) {
          if (SERIAL_OUT_REQUEST_DETAILS) {
            serialPrint(&Serial, F(" HTTP Response: "));
            serialPrint(&Serial, (String)lastTxResponse);
          }
          
          if (lastTxResponse == GOOD_HTTP_RESP_CODE) {
            if (SERIAL_OUT_REQUEST_DETAILS) {
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
    
      if (http.begin(client, I2C_SCAN_POST_URL)) {
        http.addHeader(F("Content-Type"), F("application/json"));

        StaticJsonBuffer<512> JSONbuffer;
        char JSONmessageBuffer[512];

        JsonObject& JSONencoder = JSONbuffer.createObject();
        JsonArray& foundDevicesArray = JSONbuffer.createArray();
        JsonArray& errorDevicesArray = JSONbuffer.createArray();
        
        JSONencoder["scanResults"] = JSONbuffer.createObject();
        
        for (int i = 0; i < i2cAdressSize ; i++) {
          foundDevicesArray.add(i2cAddresses[i]);
        }

        for (int i = 0; i < i2cAdressErrorSize ; i++) {
          errorDevicesArray.add(i2cAddressesErrors[i]);
        }

        JSONencoder["scanResults"]["foundDevices"] = foundDevicesArray;
        JSONencoder["scanResults"]["errorDevices"] = errorDevicesArray;

        JSONencoder.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
        
        lastTxResponse = http.POST(JSONmessageBuffer);
        if (SERIAL_OUT_REQUEST_DETAILS) {
          serialPrint(&Serial, F("\nSending POST Request..."));
        }
        
        if (lastTxResponse > 0) {
          if (SERIAL_OUT_REQUEST_DETAILS) {
            serialPrint(&Serial, F(" HTTP Response: "));
            serialPrint(&Serial, (String)lastTxResponse);
          }
          
          if (lastTxResponse == GOOD_HTTP_RESP_CODE) {
            if (SERIAL_OUT_REQUEST_DETAILS) {
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
    display.clearDisplay();
    display.setCursor(0,0);
  }
  displayPrint(F("I2C Scan...\n"));
  delay(500);
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
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
  } else {
    serialPrint(&Serial, F("\nDone\n"));
    displayPrint(F("Fin~"));
    txI2CData(devicesList, devicesListLength, devicesErrorList, devicesErrorListLength);
  }
}

bool ICACHE_FLASH_ATTR setupSerial() {
  if (Serial) {
    return true;
  } else {
    Serial.begin(SERIAL_CON_SPD);
    Serial.setDebugOutput(SERIAL_DEBUG);
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

void ICACHE_FLASH_ATTR setRunMode() {
  bool enableSerialComm = digitalRead(SW_ENABLE_SERIAL);
  bool enableOledOutput = digitalRead(SW_ENABLE_OLED);
  bool disableTransceiver = digitalRead(SW_DISABLE_TRANSCE);
  bool enableI2CScanner = digitalRead(SW_I2C_SCAN);

  runTimeVariables.wifiRetryTimes = exEepromReadByte(EEPROM_ADDR, EE_MAX_WIFI_TRIES_LOC, (byte)runTimeVariables.wifiRetryTimes);
  
  runTimeVariables.enableOled = exEepromReadByte(EEPROM_ADDR, EE_ENABLE_OLED_LOC, EEPROM_READ_FAILURE);
//  delay(50);
  
  if (runTimeVariables.enableOled == 0) {
    enableOledOutput = true;
  } else if (runTimeVariables.enableOled == 1) {
    enableOledOutput = false;
  }

  if (((currentReading.runMode & 4) == 4 && !enableOledOutput)) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.display();
  }

  if (enableSerialComm) {
    currentReading.runMode |= 3;
  } else {
    currentReading.runMode &= ~(1 << 1);
  }
  
  if (enableOledOutput) {
    currentReading.runMode |= 5;
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

void ICACHE_FLASH_ATTR setup() {
  if (DISABLE_POWER_WIFI_LED) {
    wifi_status_led_uninstall();
    pinMode(LED_BUILTIN, INPUT);
  } else {
    pinMode(LED_BUILTIN, OUTPUT);
  }
  
  pinMode(SW_ENABLE_SERIAL, INPUT);
  pinMode(SW_ENABLE_OLED, INPUT);
  pinMode(SW_DISABLE_TRANSCE, INPUT);
  pinMode(SW_I2C_SCAN, INPUT);

  currentReading = SensorReading_UnInit;
  runTimeVariables = RunTimeVariables_UnInit;

  Wire.begin();
  exEepromWriteByte(EEPROM_ADDR, EE_ENABLE_OLED_LOC, 0x00);

  setRunMode();

  if ((currentReading.runMode & 2) == 2) {
    if (setupSerial()) {
      serialPrint(&Serial, F("\nBooting weather station..."));
      serialPrint(&Serial, F("\nRun Mode: "));
      serialPrint(&Serial, (int)currentReading.runMode, HEX);
      serialPrint(&Serial, F("\nEEPROM Read: "));
      serialPrint(&Serial, (int)exEepromReadByte(EEPROM_ADDR, EE_READ_GOOD_LOC), HEX);
    }
  }

  if ((currentReading.runMode & 4) == 4) {
    setupOled(true);
  }

  if ((currentReading.runMode & 8) == 0) {
    bool wifiConnected;
    
    if ((currentReading.runMode & 1) == 1) {
      delay(1000);
    }
    
    wifiConnected = connectToWifi();

    if ((currentReading.runMode & 1) == 1) {
      delay(2000);
    }
    if (!wifiConnected && (currentReading.runMode & 1) == 0) {
      serialPrint(&Serial, F("\n Error initialising connectivity. Entering Sleep mode."));
      display.println(F("Error. Entering Sleep mode."));
      errorList |= 4;
      ESP.DEEP_SLEEP_MODE_1(DEEP_SLEEP_TIMER);
    }
  }

  int rslt;
  while ((rslt = bme.begin()) != true) {
    serialPrint(&Serial, F("\n Error initialising BME280 device. "));
    serialPrint(&Serial, F("I2C Slave Address: "));
    serialPrint(&Serial, (int)settings.bme280Addr, HEX);
    errorList |= 8;
    delay(5000);
  }

  if (bme.chipModel() == BME280::ChipModel_BME280) {
    serialPrint(&Serial, F("\n BME280 device found with chip model: "));
    serialPrint(&Serial, (int)bme.chipModel(), HEX);
  } else {
    serialPrint(&Serial, F("\n Invalid chip model found: "));
    serialPrint(&Serial, (String)bme.chipModel());
    serialPrint(&Serial, F("  Will attempt to use, but garbage may ensue. "));
    errorList |= 16;
  }

  settings.tempOSR = BME280::OSR_X4;
  bme.setSettings(settings);

  adc1115.begin();
  
  updateCurrentReading();

  if ((currentReading.runMode & 8) == 0) {
    txSensorData();
  }
  
  if ((currentReading.runMode & 1) == 0) {
    ESP.DEEP_SLEEP_MODE_1(DEEP_SLEEP_TIMER);
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
    
    printSensorDataVerbose(&Serial);
    
    if ((currentReading.runMode & 8) == 0) {
      txSensorData();
    }

    if ((currentReading.runMode & 1) == 0) {
      ESP.DEEP_SLEEP_MODE_1(DEEP_SLEEP_TIMER);
    }
  }
}

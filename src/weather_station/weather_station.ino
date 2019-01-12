
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <BME280I2C.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

#define STATION_ID 1

// Wifi:
#define STASSID "BELL457" // SSID
#define STAPSK  "6DF2572754DF" // SSID PSK
#define POST_URL "http://192.168.2.23:3000/update"
#define GOOD_HTTP_RESP_CODE HTTP_CODE_OK // Good HTTP response. Anything other than this number will result in an error being reported.
#define MAX_WIFI_TRIES 10 // How many times to tr to connect to WIFI
#define SERIAL_OUT_REQUEST_DETAILS false // Turn on/off transmission debugging. Errors will always serial out.

// Default GPS
#define GPS_LAT 43.6560079
#define GPS_LNG -79.3813297
#define GPS_ALT 0

// Pins
#define SW_ENABLE_SERIAL D5
#define SW_ENABLE_OLED D6
#define SW_I2C_SCAN D7
#define SW_DISABLE_TRANSCE D8

#define DEEP_SLEEP_TIMER 5e6
#define DEEP_SLEEP_MODE deepSleep

#define SERIAL_CON_SPD 19200
#define SERIAL_DEBUG 0 // have the ESP8266 debug serial outs

#define SCROLL_ENABABLED true // Have the attached screen automatically scroll

#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int scrollCounter = 0;
int scrollFrame = 0;
int lastTxResponse = 0;

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

struct SensorReading_s {
  char runMode;
  float temp;
  float hum;
  float pres;
  int lightLevel;
  int vIn;
  int vBat;
  int windDirection;
  int windSpeed;
  int cps;
  double lat;
  double lng;
  int alt;
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
  GPS_LAT,
  GPS_LNG,
  GPS_ALT,
  0,
  false,
  STATION_ID
};

typedef struct SensorReading_s SensorReading;

SensorReading currentReading;

void ICACHE_FLASH_ATTR displaySensorOled(unsigned int dataToShow) {
  display.clearDisplay();
  display.setCursor(0,0);

  if ((dataToShow & 1) == 1) {
    display.print(F("Temp: "));
    display.print(currentReading.temp);
    display.println("C");
  }

  if ((dataToShow & 2) == 2) {
    display.print(F("Hum: "));
    display.print(currentReading.hum);
    display.println(F("% RH"));
  }

  if ((dataToShow & 4) == 4) {
    display.print(F("Pres: "));
    display.print(currentReading.pres);
    display.println(F("Pa"));
  }

  if ((dataToShow & 8) == 8) {
    display.print(F("Light: "));
    display.println(currentReading.lightLevel);
  }

  if ((dataToShow & 16) == 16) {
    display.print(F("vIn: "));
    display.print(currentReading.vIn);
    display.println(F("v"));
  }

  if ((dataToShow & 32) == 32) {
    display.print(F("vBat: "));
    display.print(currentReading.vIn);
    display.println(F("v"));
  }
  
  if ((dataToShow & 64) == 64) {
    display.print(F("wDir: "));
    display.println(currentReading.windDirection);
  }

  if ((dataToShow & 128) == 128) {
    display.print(F("wSpd: "));
    display.print(currentReading.windSpeed);
    display.println(F("km/h"));
  }
  
  if ((dataToShow & 256) == 256) {
    display.print(F("cps: "));
    display.print(currentReading.cps);
    display.println(F("c/s"));
  }
  
  if ((dataToShow & 512) == 512) {
    display.print(F("Lat: "));
    display.println(currentReading.lat);
  }
  
  if ((dataToShow & 1024) == 1024) {
    display.print(F("Lng: "));
    display.println(currentReading.lng);
  }

  if ((dataToShow & 2048) == 2048) {
    display.print(F("Alt: "));
    display.println(currentReading.alt);
  }

  if ((dataToShow & 4096) == 4096) {
    display.print(F("RunM: "));
    char hexOut[5];
    sprintf(hexOut, "%x", currentReading.runMode);
    display.println(hexOut);
  }

  if ((dataToShow & 8192) == 8192) {
    display.print(F("RXTXConn: "));
    display.println(currentReading.rxtxConn ? F("true") : F("false"));
  }

  if ((dataToShow & 16384) == 16384) {
    display.print(F("RSSI: "));
    display.println(currentReading.rssi);
  }

  if ((dataToShow & 32768) == 32768) {
    display.print(F("txCode: "));
    display.println(lastTxResponse);
  }

  if ((dataToShow & 65536) == 65536) {
    display.print(F("StnId: "));
    display.println(currentReading.stationId);
  }

  if ((dataToShow & 131072) == 131072) {
    display.print(F("\nTemp: "));
    display.print(currentReading.temp);
    display.println(F("C"));
  }
  
  display.display();
}

void ICACHE_FLASH_ATTR printSensorDataVerbose(Stream* client) {

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

void ICACHE_FLASH_ATTR printSensorData(Stream* client) {
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

void ICACHE_FLASH_ATTR serialPrint(Stream* client, String dataToPrint) {
  if ((currentReading.runMode & 2) == 2) {
    client->print(dataToPrint);
  }
}

void ICACHE_FLASH_ATTR serialPrint(Stream* client, long dataToPrint) {
  if ((currentReading.runMode & 2) == 2) {
    client->print(dataToPrint);
  }
}

void ICACHE_FLASH_ATTR serialPrint(Stream* client, int dataToPrint, int dataType) {
  if ((currentReading.runMode & 2) == 2) {
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
  
    displayPrint(F("\nFailed to connect to wifi"));
    delay(2000);
    return false;
  } else {
    serialPrint(&Serial, F("\nWifi connected: "));
    serialPrint(&Serial, (String)WiFi.localIP().toString());
    serialPrint(&Serial, F("\tRSSI: "));
    serialPrint(&Serial, (long)WiFi.RSSI());
  
    displayPrint(F("Wifi connected: "), true);
    displayPrint((String)WiFi.localIP().toString());
    displayPrint(F("   RSSI: "));
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

void ICACHE_FLASH_ATTR getVoltageData() {
  currentReading.windDirection = currentReading.windDirection;
  currentReading.windSpeed = currentReading.windSpeed;
}

void ICACHE_FLASH_ATTR getWindData() {
  currentReading.vIn = currentReading.vIn;
  currentReading.vBat = currentReading.vBat;
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
  getVoltageData();
  getWindData();
  getRxTxData();
}

void ICACHE_FLASH_ATTR setRunMode() {
  currentReading.runMode = 0;

  bool enableSerialComm = digitalRead(SW_ENABLE_SERIAL);
  bool enableOledOutput = digitalRead(SW_ENABLE_OLED);
  bool disableTransceiver = digitalRead(SW_DISABLE_TRANSCE);
  bool enableI2CScanner = digitalRead(SW_I2C_SCAN);

  if (enableSerialComm) {
    currentReading.runMode |= 3;
  }
  if (enableOledOutput) {
    currentReading.runMode |= 5;
  }
  if (disableTransceiver) {
    currentReading.runMode |= 8;
  }
  if (enableI2CScanner) {
    currentReading.runMode |= 16;
  }
}

void ICACHE_FLASH_ATTR scanI2CDevices() {
  byte error, address;
  int nDevices;
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
      serialPrint(&Serial, address, HEX);
      displayPrint(address, false, true);
      displayPrint(", ");
      nDevices++;
    }
    else if (error == 4) {
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
  }
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

void ICACHE_FLASH_ATTR setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SW_ENABLE_SERIAL, INPUT);
  pinMode(SW_ENABLE_OLED, INPUT);
  pinMode(SW_DISABLE_TRANSCE, INPUT);
  pinMode(SW_I2C_SCAN, INPUT);

  currentReading = SensorReading_UnInit;

  setRunMode();

  if ((currentReading.runMode & 2) == 2) {
    Serial.begin(SERIAL_CON_SPD);
    Serial.setDebugOutput(SERIAL_DEBUG);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    serialPrint(&Serial, F("\nBooting weather station..."));
    serialPrint(&Serial, F("\nRun Mode: "));
    serialPrint(&Serial, (int)currentReading.runMode, HEX);
  }

  Wire.begin();

  if ((currentReading.runMode & 4) == 4) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      serialPrint(&Serial, F("\n Error initialising OLED Display."));
    } else {
      serialPrint(&Serial, F("\n OLED device found at address 0x3C"));
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.clearDisplay();
      display.setCursor(0,0);
      display.println(F("Booting...\n"));
      display.println(F("Run Mode: "));
      displayPrint(currentReading.runMode, false, true);
      display.display();
    }
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
      ESP.DEEP_SLEEP_MODE(DEEP_SLEEP_TIMER);
    }
  }

  int rslt;
  while ((rslt = bme.begin()) != true) {
    serialPrint(&Serial, F("\n Error initialising BME280 device. "));
    serialPrint(&Serial, F("I2C Slave Address: "));
    serialPrint(&Serial, (int)settings.bme280Addr, HEX);
    delay(5000);
  }

  if (bme.chipModel() == BME280::ChipModel_BME280) {
    serialPrint(&Serial, F("\n BME280 device found with chip model: "));
    serialPrint(&Serial, (int)bme.chipModel(), HEX);
  } else {
    serialPrint(&Serial, F("\n Invalid chip model found: "));
    serialPrint(&Serial, (String)bme.chipModel());
    serialPrint(&Serial, F("  Will attempt to use, but garbage may ensue. "));
  }

  settings.tempOSR = BME280::OSR_X4;
  bme.setSettings(settings);
  
  updateCurrentReading();

  if ((currentReading.runMode & 8) == 0) {
    txSensorData();
  }

  if ((currentReading.runMode & 1) == 0) {
    ESP.DEEP_SLEEP_MODE(DEEP_SLEEP_TIMER);
  }

}

void ICACHE_FLASH_ATTR loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  
  setRunMode();
  
  if ((currentReading.runMode & 16) == 16) {
    scrollFrame = 0;
    scrollCounter = 0;
    scanI2CDevices();
  } else {
    scrollFrame++;
    if (scrollFrame > 6) {
      scrollCounter++;
      scrollFrame = 0;

      if (scrollCounter > 15) {
        scrollCounter = 0;
      }
    }
    
    updateCurrentReading();
    displaySensorOled((unsigned int)(15 << scrollCounter));
    printSensorDataVerbose(&Serial);
    
    if ((currentReading.runMode & 8) == 0) {
      txSensorData();
    }
  }
}

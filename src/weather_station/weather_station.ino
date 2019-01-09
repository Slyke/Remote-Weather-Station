
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <BME280I2C.h>

#define SW_ENABLE_SERIAL D5
#define SW_ENABLE_OLED D6
#define SW_I2C_SCAN D7
#define SW_DISABLE_TRANSCE D8

#define DEEP_SLEEP_TIMER 5e6
#define DEEP_SLEEP_MODE deepSleep

#define SERIAL_CON_SPD 19200

#define SCROLL_ENABABLED true

#define OLED_RESET -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int scrollCounter = 0;
int scrollFrame = 0;

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
  0
};

typedef struct SensorReading_s SensorReading;

SensorReading currentReading;

void ICACHE_FLASH_ATTR displaySensorOled(unsigned int dataToShow) {
  display.clearDisplay();
  display.setCursor(0,0);

  if ((dataToShow & 1) == 1) {
    display.print("Temp: ");
    display.print(currentReading.temp);
    display.println("C");
  }

  if ((dataToShow & 2) == 2) {
    display.print("Hum: ");
    display.print(currentReading.hum);
    display.println("% RH");
  }

  if ((dataToShow & 4) == 4) {
    display.print("Pres: ");
    display.print(currentReading.pres);
    display.println("Pa");
  }

  if ((dataToShow & 8) == 8) {
    display.print("Light: ");
    display.println(currentReading.lightLevel);
  }

  if ((dataToShow & 16) == 16) {
    display.print("vIn: ");
    display.print(currentReading.vIn);
    display.println("v");
  }

  if ((dataToShow & 32) == 32) {
    display.print("vBat: ");
    display.print(currentReading.vIn);
    display.println("v");
  }
  
  if ((dataToShow & 64) == 64) {
    display.print("wDir: ");
    display.println(currentReading.windDirection);
  }

  if ((dataToShow & 128) == 128) {
    display.print("wSpd: ");
    display.print(currentReading.windSpeed);
    display.println("km/h");
  }
  
  if ((dataToShow & 256) == 256) {
    display.print("cps: ");
    display.print(currentReading.cps);
    display.println("c/s");
  }
  
  if ((dataToShow & 512) == 512) {
    display.print("RunM: ");
    char hexOut[5];
    sprintf(hexOut, "%x", currentReading.runMode);
    display.println(hexOut);
  }

  if ((dataToShow & 1024) == 1024) {
    display.println(" ");
    display.print("Temp: ");
    display.print(currentReading.temp);
    display.println("C");
  }
  
  display.display();
}

void ICACHE_FLASH_ATTR printSensorDataVerbose(Stream* client) {

  client->print("\nTemp: ");
  client->print(currentReading.temp);
  client->print("°C");
  client->print("\tHum: ");
  client->print(currentReading.hum);
  client->print("% RH");
  client->print("\tPres: ");
  client->print(currentReading.pres);
  client->print("Pa");
  client->print("\tLum: ");
  client->print(currentReading.lightLevel);
  client->print("Lux");
  client->print("\tvIn: ");
  client->print(currentReading.vIn);
  client->print("v");
  client->print("\tvBat: ");
  client->print(currentReading.vBat);
  client->print("v");
  client->print("\twDir: ");
  client->print(currentReading.windDirection);
  client->print("°");
  client->print("\twSpd: ");
  client->print(currentReading.windSpeed);
  client->print("km/h");
  client->print("\tcps: ");
  client->print(currentReading.cps);
  client->print("c/s");
  client->print("\tRunM: 0x");
  if (currentReading.runMode < 0x10) {
    client->print("0");
  }
  client->print(currentReading.runMode, HEX);
  client->print(".");
}

void ICACHE_FLASH_ATTR printSensorData(Stream* client) {
  client->print("\n");
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
  client->print(currentReading.runMode, HEX);
}

void ICACHE_FLASH_ATTR serialPrint(Stream* client, String dataToPrint) {
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

void ICACHE_FLASH_ATTR getBME280Data() {
  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(currentReading.pres, currentReading.temp, currentReading.hum, tempUnit, presUnit);
}

void ICACHE_FLASH_ATTR getLightLevelData() {
  currentReading.lightLevel = analogRead(A0);
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
  serialPrint(&Serial, "\nScanning...");

  if (currentReading.runMode & 16 == 16) {
    display.clearDisplay();
    display.setCursor(0,0);
  }
  displayPrint("I2C Scan...\n");
  delay(500);
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      serialPrint(&Serial, "\nI2C device found at address 0x");
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
      serialPrint(&Serial, "\nUnknown error at address 0x");
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
    serialPrint(&Serial, "\nNo I2C devices found");
    displayPrint("No I2C devices found");
  } else {
    serialPrint(&Serial, "\nDone\n");
    displayPrint("Fin~");
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
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
    serialPrint(&Serial, "\nBooting weather station...");
    serialPrint(&Serial, "\nRun Mode: ");
    serialPrint(&Serial, (int)currentReading.runMode, HEX);
  }

  Wire.begin();

  if ((currentReading.runMode & 4) == 4) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      serialPrint(&Serial, "\n Error initialising OLED Display.");
    } else {
      serialPrint(&Serial, "\n OLED device found at address 0x3C");
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.clearDisplay();
      display.setCursor(0,0);
      display.println("Booting...\n");
      display.println("Run Mode: ");
      displayPrint(currentReading.runMode, false, true);
      display.display();
    }
  }

  int rslt;
  while ((rslt = bme.begin()) != true) {
    serialPrint(&Serial, "\n Error initialising BME280 device. ");
    serialPrint(&Serial, "I2C Slave Address: ");
    serialPrint(&Serial, (int)settings.bme280Addr, HEX);
    delay(5000);
  }

  if (bme.chipModel() == BME280::ChipModel_BME280) {
    serialPrint(&Serial, "\n BME280 device found with chip model: ");
    serialPrint(&Serial, (int)bme.chipModel(), HEX);
  } else {
    serialPrint(&Serial, "\n Invalid chip model found: ");
    serialPrint(&Serial, (String)bme.chipModel());
    serialPrint(&Serial, "  Will attempt to use, but garbage may ensue. ");
  }

  settings.tempOSR = BME280::OSR_X4;
  bme.setSettings(settings);

  // Do wifi stuff

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

      if (scrollCounter > 7) {
        scrollCounter = 0;
      }
    }
    
    getLightLevelData();
    getBME280Data();
    displaySensorOled(15 << scrollCounter);
    printSensorDataVerbose(&Serial);
  }
}

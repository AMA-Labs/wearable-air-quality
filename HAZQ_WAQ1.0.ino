//comment out the following line to use TFT screen instead of SD card
//#define USE_SD_CARD 1


#include "Nicla_System.h"
#include <Adafruit_GPS.h>
#include "sps30.h"
#include "Arduino_BHY2.h"
#include <Wire.h>
#include "Adafruit_ILI9341.h"
#include <SPI.h>
#include <SD.h>
#ifdef USE_SD_CARD
#include "Sd2PinMap.h"
#endif

#ifndef USE_SD_CARD
//if we are not using the SD card, then we are using the TFT
#define TFT_CS 6
#define TFT_DC 5
#define TFT_MOSI 7
#define TFT_CLK 9
#define TFT_RST A0
#define TFT_MISO 8
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

#endif

Adafruit_GPS GPS(&Wire);
#define GPSECHO false

uint32_t timer = millis();

Sensor gas(SENSOR_ID_GAS);
SPS30 sps30;
sps_values val;

const int chipSelect = 0;

#ifdef USE_SD_CARD
File dataFile;

Sd2Card card;
SdVolume volume;
SdFile root;
#endif

void setup() {
  nicla::begin();
  Serial.begin(115200);
  Wire.begin();
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  // delay(1000);
  GPS.println(PMTK_Q_RELEASE);
  BHY2.begin();
  // delay(500);
  BHY2.begin();
  gas.begin();
  sps30.begin(&Wire);

#ifndef USE_SD_CARD
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
#endif

#ifdef USE_SD_CARD
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true)
      ;
  }

  // Open the data file
  dataFile = SD.open("datalog.txt", FILE_WRITE);
#endif
}

int year = 0;
int month = 0;
int day = 0;
int hour = 0;
int minute = 0;
int seconds = 0;
float latitude = 0;
float longitude = 0;
char lat = 'X';
char lon = 'X';
float CO2 = 0;
float temperature = 0;
float humidity = 0;


void TFT() {
//TFT Print//
#ifndef USE_SD_CARD
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Lat: ");
  tft.print(latitude / 100, 7);
  tft.print(lat);

  tft.setCursor(0, 21);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Lon: ");
  tft.print(longitude / 100, 7);
  tft.print(lon);

  tft.setCursor(0, 82);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM1: ");
  tft.print(val.MassPM1);
  tft.setCursor(0, 102);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM2: ");
  tft.print(val.MassPM2);
  tft.setCursor(0, 122);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM4: ");
  tft.print(val.MassPM4);
  tft.setCursor(0, 142);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM10: ");
  tft.print(val.MassPM10);
  tft.setCursor(0, 162);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM1: ");
  tft.print(val.NumPM1);
  tft.setCursor(0, 182);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM2: ");
  tft.print(val.NumPM2);
  tft.setCursor(0, 202);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM4: ");
  tft.print(val.NumPM4);
  tft.setCursor(0, 222);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM10: ");
  tft.print(val.NumPM10);
  tft.setCursor(0, 242);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Particle Size: ");
  tft.print(val.PartSize);

  tft.setCursor(0, 62);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("VOC_Rel: ");
  tft.print(gas.value());
#endif
}

void GPSTFT() {
  //TFT Print//
#ifndef USE_SD_CARD
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Lat: ");
  tft.print(latitude, 3);
  tft.print(lat);

  tft.setCursor(0, 21);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Lon: ");
  tft.print(longitude, 3);
  tft.print(lon);
#endif
}

void DATATFT() {
#ifndef USE_SD_CARD
  tft.setCursor(0, 82);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM1: ");
  tft.print(val.MassPM1);
  tft.setCursor(0, 102);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM2: ");
  tft.print(val.MassPM2);
  tft.setCursor(0, 122);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM4: ");
  tft.print(val.MassPM4);
  tft.setCursor(0, 142);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM10: ");
  tft.print(val.MassPM10);
  tft.setCursor(0, 162);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM1: ");
  tft.print(val.NumPM1);
  tft.setCursor(0, 182);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM2: ");
  tft.print(val.NumPM2);
  tft.setCursor(0, 202);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM4: ");
  tft.print(val.NumPM4);
  tft.setCursor(0, 222);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM10: ");
  tft.print(val.NumPM10);
  tft.setCursor(0, 242);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Particle Size: ");
  tft.print(val.PartSize);

  tft.setCursor(0, 62);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("VOC_Rel: ");
  tft.print(gas.value());
#endif
}


void loop() {
#ifdef USE_SD_CARD
  SD.end();
#endif

  BHY2.update();
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {  // this also sets the newNMEAreceived() flag to false
      if (GPS.fix) {
        year = 2000 + GPS.year;
        month = GPS.month;
        day = GPS.day;
        hour = GPS.hour;
        minute = GPS.minute;
        seconds = GPS.seconds;
        latitude = GPS.latitude;
        longitude = GPS.longitude;
        lat = GPS.lat;  //N/S
        lon = GPS.lon;  //E/W
        TFT();
        // GPSTFT();
        // DATATFT();
      }
    }
  }

  if (millis() - timer > 2000) {
    timer = millis();  // reset the timer
    sps30.GetValues(&val);

    static boolean headerprinted = false;
    if (!headerprinted) {
      Serial.println("Year,Month,Day,Hour,Minute,Second,Lat,NS,Lon,EW,P1.0,P2.5,P4.0,P10,P0.5,P1.0,P2.5,P4.0,P10,PartSize,VOCs,CO2,Temperature,Humidity");
      headerprinted = true;
    }

    Serial.print(year);
    Serial.print(",");
    Serial.print(month);
    Serial.print(",");
    Serial.print(day);
    Serial.print(",");
    Serial.print(hour);
    Serial.print(",");
    Serial.print(minute);
    Serial.print(",");
    Serial.print(seconds);
    Serial.print(",");
    Serial.print(latitude / 100.0, 10);
    Serial.print(",");
    Serial.print(lat);
    Serial.print(",");
    Serial.print(longitude / 100.0, 10);
    Serial.print(",");
    Serial.print(lon);
    Serial.print(",");

    Serial.print(val.MassPM1);
    Serial.print(",");
    Serial.print(val.MassPM2);
    Serial.print(",");
    Serial.print(val.MassPM4);
    Serial.print(",");
    Serial.print(val.MassPM10);
    Serial.print(",");
    Serial.print(val.NumPM0);
    Serial.print(",");
    Serial.print(val.NumPM1);
    Serial.print(",");
    Serial.print(val.NumPM2);
    Serial.print(",");
    Serial.print(val.NumPM4);
    Serial.print(",");
    Serial.print(val.NumPM10);
    Serial.print(",");
    Serial.print(val.PartSize);
    Serial.print(",");
    Serial.print(gas.value());
    //Print filler for CO2, Temperature, Humidity
    Serial.print(",");
    Serial.print("0");
    Serial.print(",");
    Serial.print("0");
    Serial.print(",");
    Serial.print("0");
    Serial.println();

#ifdef USE_SD_CARD
    if (dataFile) {
      SD.begin(chipSelect);

      static boolean headerprinted = false;
      if (!headerprinted) {
        dataFile.println("Year,Month,Day,Hour,Minute,Second,Lat,NS,Lon,EW,P1.0,P2.5,P4.0,P10,P0.5,P1.0,P2.5,P4.0,P10,PartSize,VOCs,CO2,Temperature,Humidity");
        headerprinted = true;
      }

      dataFile.print(year);
      dataFile.print(",");
      dataFile.print(month);
      dataFile.print(",");
      dataFile.print(day);
      dataFile.print(",");
      dataFile.print(hour);
      dataFile.print(",");
      dataFile.print(minute);
      dataFile.print(",");

      dataFile.print(seconds);
      dataFile.print(",");
      dataFile.print(latitude / 100.0, 10);
      dataFile.print(",");
      dataFile.print(lat);
      dataFile.print(",");
      dataFile.print(longitude / 100.0, 10);
      dataFile.print(",");
      dataFile.print(lon);
      dataFile.print(",");

      dataFile.print(val.MassPM1);
      dataFile.print(",");
      dataFile.print(val.MassPM2);
      dataFile.print(",");
      dataFile.print(val.MassPM4);
      dataFile.print(",");
      dataFile.print(val.MassPM10);
      dataFile.print(",");
      dataFile.print(val.NumPM0);
      dataFile.print(",");
      dataFile.print(val.NumPM1);
      dataFile.print(",");
      dataFile.print(val.NumPM2);
      dataFile.print(",");
      dataFile.print(val.NumPM4);
      dataFile.print(",");
      dataFile.print(val.NumPM10);
      dataFile.print(",");
      dataFile.print(val.PartSize);
      dataFile.print(",");
      dataFile.print(gas.value());
      //Print filler for CO2, Temperature, Humidity
      dataFile.print(",");
      dataFile.print("0");
      dataFile.print(",");
      dataFile.print("0");
      dataFile.print(",");
      dataFile.print("0");
      dataFile.println();
      dataFile.flush();
    }
#endif
  }
}

#include "Adafruit_ILI9341.h"


#include "Nicla_System.h"
#include "sps30.h"
#include <Wire.h>

#define SP30_COMMS Wire


#define DEBUG 0

void serialTrigger(char * mess);
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfo();
bool read_all();


SPS30 sps30;
#include <Adafruit_GPS.h>

Adafruit_GPS GPS(&Wire);

#define GPSECHO false

uint32_t timer = millis();

#include "Arduino_BHY2.h"
Sensor gas(SENSOR_ID_GAS);
SensorConfig cfg;
struct sps_values spsData;

//#include <SensirionI2CScd4x.h>
//SensirionI2CScd4x scd4x;


#define TFT_CS 6
#define TFT_DC 5
#define TFT_MOSI 7
#define TFT_CLK 9
#define TFT_RST 0
#define TFT_MISO 8
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

#include "SparkFun_SCD4x_Arduino_Library.h" //Click here to get the library: http://librarymanager/All#SparkFun_SCD4x
SCD4x mySensor;

//const int buttonPin = 1;
//int buttonState = 0;


void setup() {
  nicla::begin();
  BHY2.begin();
  gas.begin();
  Serial.begin(115200);
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);
  GPS.println(PMTK_Q_RELEASE);
  sps30.EnableDebugging(DEBUG);
  SP30_COMMS.begin();

//  pinMode(buttonPin, Input)

  if (sps30.begin(&SP30_COMMS) == false) {
    Errorloop((char *) "Could not set I2C communication channel.", 0);
  }
  if (! sps30.probe()) Errorloop((char *) "could not probe / connect with SPS30.", 0);
  if (! sps30.reset()) Errorloop((char *) "could not reset.", 0);
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
}



void loop() {
    BHY2.update();


 char c = GPS.read();
 
 //if (GPSECHO)
  //if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another

        mySensor.begin();
//        if (mySensor.readMeasurement()) {
////            SCD41TFT();
//    Serial.print(F("CO2(ppm):"));
//    Serial.print(mySensor.getCO2());
//        }
//            mySensor.stopPeriodicMeasurement();

          tft.setCursor(0, 62);
          tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
          tft.setTextSize(2);
          tft.print("VOC_Rel: ");
          tft.print(gas.value());
          

  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer
//    Serial.println(String("gas: ") + String(gas.value(),3));
    //GPSSerial1();
    
    if (GPS.fix) {
      //GPSSerial2();
      GPSTFT();     
    }
    read_all();
    TabularTFT();
    SCD41TFT();

  }
}
}


bool read_all()
{
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {

#ifdef USE_50K_SPEED                // update 1.4.3
    SP30_COMMS.setClock(50000);     // set to 50K
    ret = sps30.GetValues(&val);
    SP30_COMMS.setClock(100000);    // reset to 100K in case other sensors are on the same I2C-channel
#else
    ret = sps30.GetValues(&val);

#endif

    // data might not have been ready
    if (ret == SPS30_ERR_DATALENGTH){

        if (error_cnt++ > 3) {
          ErrtoMess((char *) "Error during reading values: ",ret);
          return(false);
        }
        delay(1000);
    }

    // if other error
    else if(ret != SPS30_ERR_OK) {
      ErrtoMess((char *) "Error during reading values: ",ret);
      return(false);
    }

  } while (ret != SPS30_ERR_OK);

  spsData=val;
  return(true);
}

void Errorloop(char *mess, uint8_t r)
{
  if (r) ErrtoMess(mess, r);
  else Serial.println(mess);
  Serial.println(F("Program on hold"));
  for(;;) delay(100000);
}

void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];

  Serial.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}

void TabularTFT()
{
  tft.setCursor(0, 82);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM1: ");
  tft.print(spsData.MassPM1);  
  tft.setCursor(0, 102);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM2: ");
  tft.print(spsData.MassPM2);
  tft.setCursor(0, 122);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM4: ");
  tft.print(spsData.MassPM4);
  tft.setCursor(0, 142);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("M_PM10: ");
  tft.print(spsData.MassPM10);  
  tft.setCursor(0, 162);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM1: ");
  tft.print(spsData.NumPM1);
  tft.setCursor(0, 182);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM2: ");
  tft.print(spsData.NumPM2);  
  tft.setCursor(0, 202);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM4: ");
  tft.print(spsData.NumPM4);
  tft.setCursor(0, 222);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("N_PM10: ");
  tft.print(spsData.NumPM10); 
  tft.setCursor(0, 242);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Particle Size: ");
  tft.print(spsData.PartSize);
}


void TabularSerial()
{
  Serial.print(spsData.MassPM1);
  Serial.print(F("\t"));
  Serial.print(spsData.MassPM2);
  Serial.print(F("\t"));
  Serial.print(spsData.MassPM4);
  Serial.print(F("\t"));
  Serial.print(spsData.MassPM10);
  Serial.print(F("\t"));
  Serial.print(spsData.NumPM0);
  Serial.print(F("\t"));
  Serial.print(spsData.NumPM1);
  Serial.print(F("\t"));
  Serial.print(spsData.NumPM2);
  Serial.print(F("\t"));
  Serial.print(spsData.NumPM4);
  Serial.print(F("\t"));
  Serial.print(spsData.NumPM10);
  Serial.print(F("\t"));
  Serial.print(spsData.PartSize);
  Serial.print(F("\n"));
}

void GPSTFT(){
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Latitude: ");
  tft.print(GPS.latitude, 3);

  //display longitude
  tft.setCursor(0, 21);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Longitude: ");
  tft.print(GPS.longitude, 3);
    
  tft.setCursor(0, 42);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Altitude: ");
  tft.print(GPS.altitude);

}

void SCD41TFT(){
  mySensor.readMeasurement(); 

    Serial.print(F("CO2(ppm):"));
    Serial.print(mySensor.getCO2());
        

            
  tft.setCursor(0, 262);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("CO2: ");
  tft.print(mySensor.getCO2());

  //display longitude
  tft.setCursor(0, 282);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Temp: ");
  tft.print(mySensor.getTemperature(), 1);
    
  tft.setCursor(0, 302);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(2);
  tft.print("Humidity: ");
  tft.print(mySensor.getHumidity(), 1);
              mySensor.stopPeriodicMeasurement();

        
}

void GPSSerial1(){

//Serial.print("\nTime: ");
    //if (GPS.hour < 10) { Serial.print('0'); }
    //Serial.print(GPS.hour, DEC); Serial.print(':');
    //if (GPS.minute < 10) { Serial.print('0'); }
    //Serial.print(GPS.minute, DEC); Serial.print(':');
    //if (GPS.seconds < 10) { Serial.print('0'); }
    //Serial.print(GPS.seconds, DEC); Serial.print('.');
   // if (GPS.milliseconds < 10) {
   //   Serial.print("00");
    //} else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
    //  Serial.print("0");
    //}
    //Serial.println(GPS.milliseconds);
    //Serial.print("Date: ");
    //Serial.print(GPS.day, DEC); Serial.print('/');
    //Serial.print(GPS.month, DEC); Serial.print("/20");
    //Serial.println(GPS.year, DEC);
    //Serial.print("Fix: "); Serial.print((int)GPS.fix);
    //Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
}

void GPSSerial2(){
//  Serial.print("Location: ");
    //  Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        //  Serial.print(", ");
    //  Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    //  Serial.print("Speed (knots): "); Serial.println(GPS.speed);
   //   Serial.print("Angle: "); Serial.println(GPS.angle);
    //  Serial.print("Altitude: "); Serial.println(GPS.altitude);
    //  Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
}

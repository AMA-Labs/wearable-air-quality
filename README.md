# Wearable Air Quality Monitor
The device is based around the Arduino Nicla Sense ME  and incorporates aditional sensors to collect information on VOCs, CO2, Temperature, Humidity, and particulates. The objective is to provide a relatively low cost open design to monitor pollutants of concern.

----------------------
If installing fresh libraries, make sure to change the following:

In sps30.h file make sure to do the following
Comment out: 
#define INCLUDE_SOFTWARE_SERIAL 1

Change I2C length from 32 to 256>>
  #define I2C_LENGTH 256
  
Receieve buffer length change for non SMALLFOOTPRINT from 128 to 256:
#else
#define MAXRECVBUFLENGTH 256

Also install the ArduinoBLE library for the Arduino_BHY2.
----------------------


![20230417_150857](https://user-images.githubusercontent.com/77997125/233186958-e2798079-1cbf-4580-a4c5-0dc3406ac8f2.jpg)

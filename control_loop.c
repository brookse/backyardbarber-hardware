#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>   //Only necessary if not using hardware serial pins

#define GPS_TX 3          //GPS transmit connected to digital pin 3
#define GPS_RX 2          //GPS read connected to digital pin 2
#define GPS_BAUD 9600     //GPS Baud rate

float gpsLong, gpsLat;            //GPS longitude/latitude in degrees
char gpsLongDir, gpsLatDir;       //GPS longitude/latitude cardinal direction
uint8_t gpsHour, gpsMin, gpsSec;  //GPS data time stamp

SoftwareSerial mySerial(GPS_TX,GPS_RX);
Adafruit_GPS GPS(&mySerial);

void setup() {

  GPS.begin(GPS_BAUD);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);    //Minimum recommended data output
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);        //1 update per second
  
}

void loop() {

  char c = GPS.read();    //Not sure if this is needed or not

  if(GPS.newNMEAreceived()){        //If a sentence is received, attempt to parse
    if(!GPS.parse(GPS.lastNMEA()))  //Wait for a successful parse
      return;
  }

  updateGPSData();

}

void updateGPSData(){
  
  gpsLong = GPS.longitude;
  gpsLat = GPS.latitude;
  gpsLongDir = GPS.lon;
  gpsLatDir = GPS.lat;
  gpsHour = GPS.hour;
  gpsMin = GPS.minute;
  gpsSec = GPS.seconds;

}

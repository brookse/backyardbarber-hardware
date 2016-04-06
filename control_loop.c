#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>   //Only necessary if not using hardware serial pins
#include <math.h>

#define GPS_TX 3          //GPS transmit connected to digital pin 3
#define GPS_RX 2          //GPS read connected to digital pin 2
#define GPS_BAUD 9600     //GPS Baud rate

#define PROX1 A0          //
#define PROX2 A1          //
#define PROX3 A2          //Proximity Sensor Array
#define PROX4 A3          //
#define PROX5 A4          //

float gpsLong, gpsLat;                    //GPS longitude/latitude in degrees
uint8_t gpsHour, gpsMin, gpsSec;          //GPS data time stamp
float gpsAngle;                           //Mower angle heading (Check if float)

byte PROX_ARRAY[] = {PROX1, PROX2, PROX3,
                         PROX4, PROX5};   //Proximity Pin Array
uint8_t prox[5];                          //Proximity Sensor Array Values

uint32_t timer = millis();

SoftwareSerial mySerial(GPS_TX,GPS_RX);
Adafruit_GPS GPS(&mySerial);

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

void setup() {

  //Serial.begin(115200);

  GPS.begin(GPS_BAUD);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);    //Minimum recommended data output
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);        //1 update per second

  //Using Interrupt = true
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  delay(1000);
}

void loop() {

  if(GPS.newNMEAreceived()){        //If a sentence is received, attempt to parse
    if(!GPS.parse(GPS.lastNMEA()))  //Wait for a successful parse
      return;
  }

  updateProxData();
  updateGPSData();

  delay(500);
}

void updateGPSData(){
  
  gpsLong = GPS.longitude;
  gpsLat = GPS.latitude;
  gpsHour = GPS.hour;
  gpsMin = GPS.minute;
  gpsSec = GPS.seconds;
  gpsAngle = GPS.angle;

}

void updateProxData(){
  int i, j;
  for(i=0;i<5;i++){
    int senseVal = 0;
    for(j=0;j<10;j++){
      senseVal+=analogRead(PROX_ARRAY[i]);
    }
    prox[i] = (uint8_t) 12878*pow((senseVal/10),-1.179);
  }
}

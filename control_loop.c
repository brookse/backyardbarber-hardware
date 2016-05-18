#include <Adafruit_GPS.h>
#include <AltSoftSerial.h>    //pin 10 pwm used
#include <math.h>
#include "BMSerial.h"
#include "RoboClaw.h"

//Attach GPS power to 5V
#define GPS_TX 3          //GPS transmit connected to digital pin 3
#define GPS_RX 2          //GPS read connected to digital pin 2
#define GPS_BAUD 9600     //GPS Baud rate

//Attach Prox power to 5V
#define PROX_OUT  A0      //Analog MUX output pin

#define PROX_S0   4       //
#define PROX_S1   5       //Analog MUX select pins
#define PROX_S2   6       //

//Attach Accel power to 3.3V
#define TILTX A1          //Tilt Sensor X Axis
#define TILTY A2          //Tilt Sensor Y Axis (Pin doesnt exist)

#define motorAddress 0x80 //RoboClaw Address
#define MOTORPIN1 12      //RoboClaw Serial Pins
#define MOTORPIN2 11      //RoboClaw Serial Pins
#define timeout 10000     //10ms timeout for roboclaw communication

#define BMSERIALBAUD 57600//Baud rate for BMSerial terminal
#define ROBOBAUD 38400    //Baud rate for Roboclaw comms

//Velocity PID coefficients
#define Kp 5890
#define Ki 0.0
#define Kd 0.0
#define qpps 5250

float gpsLong, gpsLat;                    //GPS longitude/latitude in degrees
uint32_t gpsHour, gpsMin, gpsSec;          //GPS data time stamp
float gpsAngle;                           //Mower angle heading (Check if float)

uint32_t prox[5];                          //Proximity Sensor Array Values

uint32_t accelX, accelY;                       //Tilt Sensor output values

uint32_t timer = millis();

int roboLoopCount = 0;

AltSoftSerial mySerial(GPS_TX,GPS_RX);
Adafruit_GPS GPS(&mySerial);

//Define terminal for display. Use hardware serial pins 0 and 1
BMSerial terminal(0,1);

//Setup communication with roboclaw.
RoboClaw roboclaw(MOTORPIN1, MOTORPIN2, timeout);

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
}

void setup() {
    
  GPS.begin(GPS_BAUD);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);    //Minimum recommended data output
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);        //1 update per second

  //Using Interrupt = true
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  //Open terminal and roboclaw serial ports
  terminal.begin(BMSERIALBAUD);
  roboclaw.begin(ROBOBAUD);

  delay(1000);

  roboclaw.SpeedDistanceM1(motorAddress,1000,4000,1);
  roboclaw.SpeedDistanceM2(motorAddress,1000,4000,1);

  roboclaw.SpeedDistanceM1(motorAddress,1000,500,0);
  roboclaw.SpeedDistanceM2(motorAddress,-1000,500,0);

  roboclaw.SpeedDistanceM1(motorAddress,-1000,1000,0);
  roboclaw.SpeedDistanceM2(motorAddress,1000,1000,0);

  roboclaw.SpeedDistanceM1(motorAddress,1000,500,0);
  roboclaw.SpeedDistanceM2(motorAddress,-1000,500,0);
}

void loop() {

  if(GPS.newNMEAreceived()){        //If a sentence is received, attempt to parse
    if(!GPS.parse(GPS.lastNMEA()))  //Wait for a successful parse
      return;
  }

  updateProxData();
  updateGPSData();
  readTiltSensor();
  //roboLoop();

  printData();
  displaySpeed();

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
    setMUX(i);
    int senseVal = 0;
    for(j=0;j<10;j++){
      senseVal+=analogRead(PROX_OUT);
    }
    prox[i] = 12878*pow((senseVal/10),-1.179);
  }
}

void setMUX(int channel){
  switch(channel){
    case 0:
      digitalWrite(PROX_S0, LOW);
      digitalWrite(PROX_S1, LOW);
      digitalWrite(PROX_S2, LOW);
      break;
    case 1:
      digitalWrite(PROX_S0, HIGH);
      digitalWrite(PROX_S1, LOW);
      digitalWrite(PROX_S2, LOW);
      break;
    case 2:
      digitalWrite(PROX_S0, LOW);
      digitalWrite(PROX_S1, HIGH);
      digitalWrite(PROX_S2, LOW);
      break;
    case 3:
      digitalWrite(PROX_S0, HIGH);
      digitalWrite(PROX_S1, HIGH);
      digitalWrite(PROX_S2, LOW);
      break;
    case 4:
      digitalWrite(PROX_S0, LOW);
      digitalWrite(PROX_S1, LOW);
      digitalWrite(PROX_S2, HIGH);
      break;
    default:
      break;
  }
}

void readTiltSensor(){
  accelX = analogRead(TILTX);
  accelY = analogRead(TILTY);
  if(accelX>370 || accelX<300 || accelY>375 || accelY<305){
    //IMMEDIATE MOTOR STOP
    roboclaw.SpeedM1(motorAddress, 0);
    roboclaw.SpeedM2(motorAddress, 0);
    //Set status to stopped
  }
}

//*(2000*roboLoopCount)

void roboLoop(){
  int path = roboLoopCount%4;
  switch(path){
    case 0: 
      roboclaw.SpeedDistanceM1(motorAddress,12000,8000,0);
      roboclaw.SpeedDistanceM2(motorAddress,12000,8000,0);
      break;
    case 1:
      roboclaw.SpeedDistanceM1(motorAddress,6000,8000,0);
      roboclaw.SpeedDistanceM2(motorAddress,-6000,8000,0);
      break;
    case 2:
      roboclaw.SpeedDistanceM1(motorAddress,-12000,8000,0);
      roboclaw.SpeedDistanceM2(motorAddress,-12000,8000,0);
      break;
    case 3:
      roboclaw.SpeedDistanceM1(motorAddress,-6000,8000,0);
      roboclaw.SpeedDistanceM2(motorAddress,6000,8000,0);
      break;
    default:
      break;
  }
  roboLoopCount++;
}

void printData(){
  terminal.print("GPS Time: ");
  terminal.print(gpsHour, DEC); terminal.print(':');
  terminal.print(gpsMin, DEC); terminal.print(':');
  terminal.println(gpsSec, DEC);
  terminal.print("GPS Data: ");
  terminal.print(gpsLat,6);terminal.print(", ");
  terminal.println(gpsLong,6);
  terminal.print("Prox Data: ");
  terminal.print(prox[0]);terminal.print("\t");
  terminal.print(prox[1]);terminal.print("\t");
  terminal.print(prox[2]);terminal.print("\t");
  terminal.print(prox[3]);terminal.print("\t");
  terminal.println(prox[4]);
}

void displaySpeed(void){
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  
  int32_t enc1 = roboclaw.ReadEncM1(motorAddress, &status1, &valid1);
  int32_t enc2 = roboclaw.ReadEncM2(motorAddress, &status2, &valid2);
  int32_t speed1 = roboclaw.ReadSpeedM1(motorAddress, &status3, &valid3);
  int32_t speed2 = roboclaw.ReadSpeedM2(motorAddress, &status4, &valid4);

  terminal.print("Encoder1:");
  if(valid1){
    terminal.print(enc1,DEC);
    terminal.print(" ");
    terminal.print(status1,HEX);
    terminal.print(" ");
  }else{
    terminal.print("failed ");
  }
  terminal.print("Encoder2:");
  if(valid2){
    terminal.print(enc2,DEC);
    terminal.print(" ");
    terminal.print(status2,HEX);
    terminal.print(" ");
  }else{
    terminal.print("failed ");
  }
  terminal.print("Speed1:");
  if(valid3){
    terminal.print(speed1,DEC);
    terminal.print(" ");
  }else{
    terminal.print("failed ");
  }
  terminal.print("Speed2:");
  if(valid4){
    terminal.print(speed2,DEC);
    terminal.print(" ");
  }else{
    terminal.print("failed ");
  }
  terminal.println();
}

#include <Arduino.h>
#include <config.h>

#include "WiFi.h"
#include <SPI.h>
#include <Wire.h>


#include <TinyGPS++.h>
#include "AccelStepper.h"
#include <MultiStepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

#include "BluetoothSerial.h"
#include "CommandHandler.h"

//Bluetooth stuff/////////////////////////////
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
////////////////////////////////////////

// Declaration for hardware objects
Adafruit_LSM303_Accel_Unified accelerometer = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified magnetometer = Adafruit_LSM303_Mag_Unified(12345);
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP); //, "europe.pool.ntp.org", 3600, 60000);
//SoftwareSerial gpsserial(39,34);

AccelStepper stepper1(AccelStepper::DRIVER, s1step, s1dir);
AccelStepper stepper2(AccelStepper::DRIVER, s2step, s2dir);
MultiStepper steppers;
long positions[2]; // Array of desired stepper positions

int mot1steps = 0;
int mot2steps = 0;
int m1step = 300;
int m2step = 300;
bool smode = false;
bool did = false;
float accelxnum = 0;
float accelynum = 0;
float accelznum = 0;
float magxnum = 0;
float magynum = 0;
float magznum = 0;
int s1homing = 0; //0 is not homing, 1 is moving negative, 2 is moving positive
int s2homing = 0; //0 is not homing, 1 is moving negative, 2 is moving positive
//int displaystate = 0; //initial display mode

//serial command handler!!
CommandHandler<> Cmds(SerialBT);

void Cmd_GetAccelStatus(CommandParameter &p)
{
  sensors_event_t eventm;
  sensors_event_t eventa;
  accelerometer.getEvent(&eventa);
  magnetometer.getEvent(&eventm);
  String dtext = "";
  accelxnum = eventa.acceleration.x;
  accelynum = eventa.acceleration.y;
  accelznum = eventa.acceleration.z;
  magxnum = eventm.magnetic.x;
  magynum = eventm.magnetic.y;
  magznum = eventm.magnetic.z;
  String magx = "X: " + String(magxnum);
  String magy = "Y: " + String(magynum);
  String magz = "Z: " + String(magznum);
  String accelx = "X: " + String(accelxnum);
  String accely = "Y: " + String(accelynum);
  String accelz = "Z: " + String(accelznum);
  dtext += "Accel  Mag";
  dtext += "\n";
  dtext += accelx + " " + magx + "\n";
  dtext += accely + " " + magy + "\n";
  dtext += accelz + " " + magz + "\n";
  SerialBT.println(dtext);
  Serial.println("AccelStatus");
}

void Cmd_GetStepperPos(CommandParameter &p)
{
  mot1steps = stepper1.currentPosition();
  mot2steps = stepper2.currentPosition();
  String dtext = "";
  dtext += "stepper1 = "+String(mot1steps)+"\n";
  dtext += "stepper2 = "+String(mot2steps)+"\n";
  SerialBT.println(dtext);
  Serial.println("StepperPos");
}

bool wifiConnect(){
  /* Set ESP32 to WiFi Station mode */
  int retry=0;
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  
  while(WiFi.status() != WL_CONNECTED) {
    
    vTaskDelay(500);
    if (retry++ >= 20) { // timeout for connection is 10 seconds
      
      /* start SmartConfig */
      WiFi.beginSmartConfig();
      /* Wait for SmartConfig packet from mobile */
      
      int retries = 0;
      while (!WiFi.smartConfigDone()) {
        delay(500);
        
        retries += 1;
        if(retries > 50){
          //wifi connection fails
          return false;
        }
      }
      
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        
      }
      
      delay(2000); // Pause for 2 seconds
    }
  }
  //wifi connection worked!
  return true;
}


void setup() {
  if(true){ //pin configs
    pinMode(s1step,OUTPUT);
    pinMode(s2dir,OUTPUT);
    pinMode(s2step,OUTPUT);
    pinMode(s1step,OUTPUT);
    pinMode(s1home, INPUT);
    pinMode(s2home,INPUT);
    pinMode(BTN_UP,INPUT);
    pinMode(BTN_DOWN,INPUT);
    pinMode(BTN_SEL,INPUT);
  }
  // Configure each stepper
  stepper1.setMaxSpeed(230.0);
  stepper2.setMaxSpeed(230.0);
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  ////////////////////////////////
  //Serial
  Serial.begin(9600);
  Serial.println("begin!");
  ///////////////////////////////
  vTaskDelay(1000); // Pause for 1 seconds
  ///////////////////////////////
  //Bluetooth setup
  SerialBT.begin("ISSpointer2");
  //command handler
  Cmds.AddCommand(F("GetAccelStatus"), Cmd_GetAccelStatus);
  Cmds.AddCommand(F("GetStepperPos"),Cmd_GetStepperPos);
  //
  

  //GPS
  SerialGPS.begin(9600,SERIAL_8N1,16,17);
  ///////////////////////////////
  //accelerometer and magneto
  accelerometer.begin();
  magnetometer.enableAutoRange(true);
  magnetometer.begin();
  ///////////////////////////////
  //wifi
  bool wificonnected = wifiConnect(); //connect to wifi
  //UDP client
  if(wificonnected){
    timeClient.begin();
  }
  ////////////////////////////
  s1homing = 0;
  s2homing = 0;
  
}

void loop() {
  Cmds.Process();
  bool imhoming = (s1homing>0) || (s2homing>0);
  if(millis()%5000<3){ //stepper jog for testing
    /* stepper testing */
    if(!did){
      if(smode){
        positions[0] = 200;
        positions[1] = 100;
        steppers.moveTo(positions);
      }else{
        positions[0] = -100;
        positions[1] = -200;
        steppers.moveTo(positions);
      }
      smode = !smode;
      did =true;
    }
  }else{
    did = false;
  }
  if((s1homing>0) || (s2homing>0)){ //stepper homing
    if((s1homing==1) and digitalRead(s1home)){
      stepper1.setSpeed(homespeed);
      stepper1.runSpeed();
    }else if((s1homing==1) and !digitalRead(s1home)){
      s1homing = 2;
      stepper1.setSpeed(homespeed*-.5);
      stepper1.runSpeed();
    }else if((s1homing==2) and digitalRead(s1home)){
      s1homing = 0;
      stepper1.setCurrentPosition(0);
    }
    if((s2homing==1) and digitalRead(s2home)){
      stepper2.setSpeed(homespeed);
      stepper2.runSpeed();
    }else if((s2homing==1) and !digitalRead(s2home)){
      s2homing = 2;
      stepper2.setSpeed(homespeed*-.5);
      stepper2.runSpeed();
    }else if((s2homing==2) and digitalRead(s2home)){
      s2homing = 0;
      stepper2.setCurrentPosition(0);
    }
  }else{
    steppers.run();
  }
}
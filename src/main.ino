#include <Arduino.h>
#include "WiFi.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

//#include "SSD1306Ascii.h"
//#include "SSD1306AsciiWire.h"

#include <Adafruit_GFX.h>
#include <TinyGPS++.h>
#include "AccelStepper.h"
#include <MultiStepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <config.h>
#include "OneButton.h"
//using namespace Menu;

//#include <navMenu.cpp>

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_LSM303_Accel_Unified accelerometer = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified magnetometer = Adafruit_LSM303_Mag_Unified(12345);

//SSD1306AsciiWire display;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TinyGPSPlus gps;

HardwareSerial SerialGPS(1);
//SoftwareSerial gpsserial(39,34);

AccelStepper stepper1(AccelStepper::DRIVER, s1step, s1dir);
AccelStepper stepper2(AccelStepper::DRIVER, s2step, s2dir);
MultiStepper steppers;
long positions[2]; // Array of desired stepper positions

OneButton up_button(BTN_UP,true);
OneButton down_button(BTN_DOWN,true);
OneButton sel_button(BTN_SEL,true);


String stepmsg = "steppers!";
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
int displaystate = 0; //initial display mode
void displayInfo(){
  /*display GPS data */
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.print(F("Location: ")); 
  if (gps.location.isValid()){
    display.print(gps.location.lat(), 6);
    display.print(F(","));
    display.print(gps.location.lng(), 6);
  }else{
    display.print(F("INVALID"));
  }
  display.display();

  display.print(F("  Date/Time: "));
  if (gps.date.isValid()){
    display.print(gps.date.month());
    display.print(F("/"));
    display.print(gps.date.day());
    display.print(F("/"));
    display.print(gps.date.year());
  }else{
    display.print(F("INVALID"));
  }
  display.display();

  display.print(F(" "));
  if (gps.time.isValid()){
    if (gps.time.hour() < 10) display.print(F("0"));
    display.print(gps.time.hour());
    display.print(F(":"));
    if (gps.time.minute() < 10) display.print(F("0"));
    display.print(gps.time.minute());
    display.print(F(":"));
    if (gps.time.second() < 10) display.print(F("0"));
    display.print(gps.time.second());
    display.print(F("."));
    if (gps.time.centisecond() < 10) display.print(F("0"));
    display.print(gps.time.centisecond());
  }else{
    display.print(F("INVALID"));
  }
  display.display();
}

bool wifiConnect(){
  /* Set ESP32 to WiFi Station mode */
  int retry=0;
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  display.println("Attempt to connect to WiFi network...");
  display.display();
  while(WiFi.status() != WL_CONNECTED) {
    display.print(".");
    display.display();
    vTaskDelay(500);
    if (retry++ >= 20) { // timeout for connection is 10 seconds
      display.println("Connection timeout expired! Start Smartconfig...");
      display.display();
      /* start SmartConfig */
      WiFi.beginSmartConfig();
      /* Wait for SmartConfig packet from mobile */
      display.println(F("Waiting for SmartConfig."));
      display.display();
      int retries = 0;
      while (!WiFi.smartConfigDone()) {
        delay(500);
        display.print(F("."));
        display.display();
        retries += 1;
        if(retries > 50){
          //wifi connection fails
          return false;
        }
      }
      display.println(F(""));
      display.println(F("SmartConfig done."));
      /* Wait for WiFi to connect to AP */
      display.println(F("Waiting for WiFi"));
      display.display();
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        display.print(F("."));
        display.display();
      }
      display.println(F("WiFi Connected."));
      display.print(F("IP Address: "));
      display.println(WiFi.localIP());
      display.display();
      delay(2000); // Pause for 2 seconds
    }
  }
  //wifi connection worked!
  return true;
}



void setup() {
  ////////////////////////////////
  //stepper
  pinMode(s1step,OUTPUT);
  pinMode(s2dir,OUTPUT);
  pinMode(s2step,OUTPUT);
  pinMode(s1step,OUTPUT);
  pinMode(s1home, INPUT);
  pinMode(s2home,INPUT);
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
  //Display
  //display.begin(&Adafruit128x64, DISPLAY_ADDRESS);
  //display.setFont(menuFont);
  //display.setScrollMode(SCROLL_MODE_OFF);

  display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS);
  display.println("begin!!");
  //if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
  //  Serial.println(F("SSD1306 allocation failed"));
  //  for(;;); // Don't proceed, loop forever
  //}
  display.display();
  vTaskDelay(1000); // Pause for 1 seconds
  ///////////////////////////////
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
  //////////////////////////////
  //buttons
  //up_button.attachClick(menu_up);
  //down_button.attachClick(menu_down);
  //sel_button.attachClick(menu_select);
  ////////////////////////////
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  s1homing = 1;
  s2homing = 1;
  
}


void loop() {
  bool imhoming = (s1homing>0) || (s2homing>0);
  if(millis()%5000<3){
    /* stepper testing */
    if(!did){
      if(smode){
        stepmsg = "stepping forward";
        positions[0] = 200;
        positions[1] = 100;
        steppers.moveTo(positions);
      }else{
        stepmsg = "stepping reverse";
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
  if((s1homing>0) || (s2homing>0)){
    stepmsg = "homing!";
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
  
  /* magnetometer and accelerometer */
  
  if(micros()%10000 == 0){
    sensors_event_t eventm;
    sensors_event_t eventa;
    accelerometer.getEvent(&eventa);
    magnetometer.getEvent(&eventm);
    mot1steps = stepper1.currentPosition();
    mot2steps = stepper2.currentPosition();
    accelxnum = eventa.acceleration.x;
    accelynum = eventa.acceleration.y;
    accelznum = eventa.acceleration.z;
    magxnum = eventm.magnetic.x;
    magynum = eventm.magnetic.y;
    magznum = eventm.magnetic.z;
  }
  /*
  while(SerialGPS.available()>0){
    display.display();
    if (gps.encode(SerialGPS.read()))
      displayInfo();
    if (millis() > 5000 && gps.charsProcessed() < 10){
      display.println(F("No GPS detected: check wiring."));
      display.display();
      while(true);
    }
  }
  delay(4000);
  */
 if(micros()%20000==0){
  /* magnetometer and accelerometer */
  String accelx = "X: " + String(accelxnum);
  String accely = "Y: " + String(accelynum);
  String accelz = "Z: " + String(accelznum);
  String magx = "X: " + String(magxnum);
  String magy = "Y: " + String(magynum);
  String magz = "Z: " + String(magznum);
   /* make the display work */
  display.clearDisplay();
  //display.setTextSize(1);             // Normal 1:1 pixel scale
  //display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);
  String dtext = "";
  dtext += stepmsg+"\n";
  dtext += String(digitalRead(s1home))+"\n";
  dtext += String(digitalRead(s2home))+"\n";
  //display.println(stepmsg);
  //display.println();
  //display.println(digitalRead(s2home));
  //display.println(mot1steps);
  //display.println(mot2steps);
  if(displaystate==0){
    dtext += "Accel  Mag";
    dtext += "\n";
    dtext += accelx + " " + magx + "\n";
    dtext += accely + " " + magy + "\n";
    dtext += accelz + " " + magz + "\n";
    //display.setCursor(8,22); //setcursor is text cols, pixel rows
    //display.println("Mag");
    //display.setCursor(8,22+8);
    //display.println(magx);
    //display.setCursor(8,22+8*2);
    //display.println(magy);
    //display.setCursor(8,22+8*3);
    //display.println(magz);
  }else if(displaystate == 1){
    while(SerialGPS.available()>0){
      if (gps.encode(SerialGPS.read()))
        displayInfo();
    }
  }
  display.print(dtext);
  display.display();
  
 }
}
# 1 "C:\\Users\\andrey\\AppData\\Local\\Temp\\tmpua_jv_oa"
#include <Arduino.h>
# 1 "D:/Documents/GitHub/esp32_arduino_isspointer/src/main.ino"
#include <Arduino.h>
#include <config.h>

#include "WiFi.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>




#include <Adafruit_GFX.h>
#include <TinyGPS++.h>
#include "AccelStepper.h"
#include <MultiStepper.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



#include <menuIO/altKeyIn.h>
#include <SSD1306_for_menu.h>
#include <menu.h>
#include <menuIO/serialIO.h>
#include <menuIO/chainStream.h>

result doAlert(eventMask e, prompt &item);
result showEvent(eventMask e, navNode& nav, prompt& item);
result action1(eventMask e);
result action2(eventMask e, navNode& nav, prompt &item);
result myLedOn();
result myLedOff();
result alert(menuOut& o, idleEvent e);
result idle(menuOut &o, idleEvent e);
void displayInfo();
bool wifiConnect();
void setup();
void loop();
#line 31 "D:/Documents/GitHub/esp32_arduino_isspointer/src/main.ino"
result showEvent(eventMask e, navNode& nav, prompt& item) {
  Serial.print("event: ");
  Serial.println(e);
  return proceed;
}

int test = 55;

result action1(eventMask e) {
  Serial.print(e);
  Serial.println(" action1 executed, proceed menu"); Serial.flush();
  trace(if (e == enterEvent) display.clear(0, display.displayWidth(), 2, 3));
  return proceed;
}

result action2(eventMask e, navNode& nav, prompt &item) {
  Serial.print(e);
  Serial.println(" action2 executed, quiting menu");
  return quit;
}

int ledCtrl = LOW;

result myLedOn() {
  ledCtrl = HIGH;
  return proceed;
}
result myLedOff() {
  ledCtrl = LOW;
  return proceed;
}

TOGGLE(ledCtrl, setLed, "Led: ", doNothing, noEvent, noStyle
       , VALUE("On", HIGH, doNothing, noEvent)
       , VALUE("Off", LOW, doNothing, noEvent)
      );

int selTest = 0;
SELECT(selTest, selMenu, "Select", doNothing, noEvent, noStyle
       , VALUE("Zero", 0, doNothing, noEvent)
       , VALUE("One", 1, doNothing, noEvent)
       , VALUE("Two", 2, doNothing, noEvent)
      );

int chooseTest = -1;
CHOOSE(chooseTest, chooseMenu, "Choose", doNothing, noEvent, noStyle
       , VALUE("First", 1, doNothing, noEvent)
       , VALUE("Second", 2, doNothing, noEvent)
       , VALUE("Third", 3, doNothing, noEvent)
       , VALUE("Last", -1, doNothing, noEvent)
      );



class altPrompt: public prompt {
  public:
    altPrompt(constMEM promptShadow& p): prompt(p) {}
    Used printTo(navRoot &root, bool sel, menuOut& out, idx_t idx, idx_t len, idx_t) override {
      return out.printRaw(F( "special prompt!"), len);
    }
};

MENU(subMenu, "Sub-Menu", showEvent, anyEvent, noStyle
     , OP("Sub1", showEvent, anyEvent)
     , OP("Sub2", showEvent, anyEvent)
     , OP("Sub3", showEvent, anyEvent)
     , altOP(altPrompt, "", showEvent, anyEvent)
     , EXIT("<Back")
    );

MENU(mainMenu, "Main menu", doNothing, noEvent, wrapStyle
     , OP("Op1", action1, anyEvent)
     , OP("Op2", action2, enterEvent)
     , FIELD(test, "Test", "%", 0, 100, 10, 1, doNothing, noEvent, wrapStyle)
     , SUBMENU(subMenu)
     , SUBMENU(setLed)
     , OP("LED On", myLedOn, enterEvent)
     , OP("LED Off", myLedOff, enterEvent)
     , SUBMENU(selMenu)
     , SUBMENU(chooseMenu)
     , OP("Alert test", doAlert, enterEvent)
     , EXIT("<Back")
    );

keyMap joystickBtn_map[]={
 {BTN_SEL, defaultNavCodes[enterCmd].ch,INPUT} ,
 {BTN_UP, defaultNavCodes[upCmd].ch,INPUT} ,
 {BTN_DOWN, defaultNavCodes[downCmd].ch,INPUT} ,
};

keyIn<3> joystickBtns(joystickBtn_map);
serialIn serial(Serial);

menuIn* inputsList[]={&joystickBtns,&serial};
chainStream<2> in(inputsList);


#define MAX_DEPTH 2


idx_t serialTops[MAX_DEPTH] = {0};
serialOut outSerial(Serial, serialTops);



constMEM panel panels[] MEMMODE = {{0, 0, 128 / fontW, 64 / fontH}};
navNode* nodes[sizeof(panels) / sizeof(panel)];
panelsList pList(panels, nodes, 1);
idx_t tops[MAX_DEPTH] = {0, 0};
Adafruit_SSD1306Out outOLED(&display, tops, pList, fontW, fontH ,menuOut::minimalRedraw);
menuOut* constMEM outputs[] MEMMODE = {&outOLED, &outSerial};
outputsList out(outputs, sizeof(outputs) / sizeof(menuOut*));


NAVROOT(nav, mainMenu, MAX_DEPTH, in, out);

result alert(menuOut& o, idleEvent e) {
  if (e == idling) {
    o.setCursor(0, 0);
    o.print("alert test");
    o.setCursor(0, 1);
    o.print("press [select]");
    o.setCursor(0, 2);
    o.print("to continue...");
  }
  return proceed;
}
result doAlert(eventMask e, prompt &item) {
  nav.idleOn(alert);
  return proceed;
}

result idle(menuOut &o, idleEvent e) {
  o.clear();
  if (&o==&outOLED) {
    if (e==idling) {
      o.println("OLED");
      o.println("Suspended menu");
    }
  } else
    switch (e) {
      case idleStart: o.println("suspending menu!"); break;
      case idling: o.println("suspended..."); break;
      case idleEnd: o.println("resuming menu."); break;
    }
  return proceed;
}

using namespace Menu;



Adafruit_LSM303_Accel_Unified accelerometer = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified magnetometer = Adafruit_LSM303_Mag_Unified(12345);
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);


AccelStepper stepper1(AccelStepper::DRIVER, s1step, s1dir);
AccelStepper stepper2(AccelStepper::DRIVER, s2step, s2dir);
MultiStepper steppers;
long positions[2];

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
int s1homing = 0;
int s2homing = 0;


bool useMenu = true;

void displayInfo(){


  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
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

  int retry=0;
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  if(!useMenu){
    display.println("Attempt to connect to WiFi network...");
    display.display();
  }
  while(WiFi.status() != WL_CONNECTED) {
    if(!useMenu){
      display.print(".");
      display.display();
    }
    vTaskDelay(500);
    if (retry++ >= 20) {
      if(!useMenu){
        display.println("Connection timeout expired! Start Smartconfig...");
        display.display();
      }

      WiFi.beginSmartConfig();

      if(!useMenu){
        display.println(F("Waiting for SmartConfig."));
        display.display();
      }
      int retries = 0;
      while (!WiFi.smartConfigDone()) {
        delay(500);
        if(!useMenu){
          display.print(F("."));
          display.display();
        }
        retries += 1;
        if(retries > 50){

          return false;
        }
      }
      if(!useMenu){
        display.println(F(""));
        display.println(F("SmartConfig done."));

        display.println(F("Waiting for WiFi"));
        display.display();
      }
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if(!useMenu){
          display.print(F("."));
          display.display();
        }
      }
      if(!useMenu){
        display.println(F("WiFi Connected."));
        display.print(F("IP Address: "));
        display.println(WiFi.localIP());
        display.display();
      }
      delay(2000);
    }
  }

  return true;
}



void setup() {


  pinMode(s1step,OUTPUT);
  pinMode(s2dir,OUTPUT);
  pinMode(s2step,OUTPUT);
  pinMode(s1step,OUTPUT);
  pinMode(s1home, INPUT);
  pinMode(s2home,INPUT);

  stepper1.setMaxSpeed(230.0);
  stepper2.setMaxSpeed(230.0);
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);



  Serial.begin(9600);
  Serial.println("begin!");






  pinMode(BTN_UP,INPUT);
  pinMode(BTN_DOWN,INPUT);
  pinMode(BTN_SEL,INPUT);
  if(!useMenu){

    display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS);
    display.println("begin!!");




    display.display();
  }else{
    display.begin(SSD1306_SWITCHCAPVCC, DISPLAY_ADDRESS);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.print("SSD1306 adafruit");
    display.display();
    delay(2000);
    display.clearDisplay();

    nav.idleTask = idle;

  }
  vTaskDelay(1000);


  SerialGPS.begin(9600,SERIAL_8N1,16,17);


  accelerometer.begin();
  magnetometer.enableAutoRange(true);
  magnetometer.begin();
# 399 "D:/Documents/GitHub/esp32_arduino_isspointer/src/main.ino"
  if(!useMenu){

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
  }
  s1homing = 1;
  s2homing = 1;

}


void loop() {
  bool imhoming = (s1homing>0) || (s2homing>0);
  if(millis()%5000<3){

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
# 491 "D:/Documents/GitHub/esp32_arduino_isspointer/src/main.ino"
 if(micros()%10000<5){

    String accelx = "X: " + String(accelxnum);
    String accely = "Y: " + String(accelynum);
    String accelz = "Z: " + String(accelznum);
    String magx = "X: " + String(magxnum);
    String magy = "Y: " + String(magynum);
    String magz = "Z: " + String(magznum);





    String dtext = "";
    dtext += stepmsg+"\n";
    dtext += String(digitalRead(s1home))+"\n";
    dtext += String(digitalRead(s2home))+"\n";
    dtext += String(digitalRead(BTN_UP))+" "+
            String(digitalRead(BTN_DOWN))+" "+
            String(digitalRead(BTN_SEL))+" "+
            "\n";






    dtext += "Accel  Mag";
    dtext += "\n";
    dtext += accelx + " " + magx + "\n";
    dtext += accely + " " + magy + "\n";
    dtext += accelz + " " + magz + "\n";
    if(!useMenu){
      display.clearDisplay();
      display.setCursor(0,0);
      display.print(dtext);
      display.fillRect(0,7*3,128,7,0);
      display.display();
    }else{
      nav.poll();
      display.display();
    }
 }
}
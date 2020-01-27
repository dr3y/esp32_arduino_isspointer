#include <Arduino.h>
#include "WiFi.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <TinyGPS++.h>
#include "AccelStepper.h"
#include <MultiStepper.h>
//#include <SoftwareSerial.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TinyGPSPlus gps;
#define EEPROM_SIZE 1024
HardwareSerial SerialGPS(1);
//SoftwareSerial gpsserial(39,34);
#define s1dir  26 //oogabooga
#define s1step 32 //8
#define senable 25 //12
#define s2dir 27 //10
#define s2step 33 //11



//Stepper motor_1(2, 3);   //STEP pin =  2, DIR pin = 3
//Stepper motor_2(9,10);   //STEP pin =  9, DIR pin = 10


//stepper1.setEnablePin(senable);

AccelStepper stepper1(AccelStepper::DRIVER, s1step, s1dir);
AccelStepper stepper2(AccelStepper::DRIVER, s2step, s2dir);
MultiStepper steppers;
long positions[2]; // Array of desired stepper positions

void setup() {
  //stepper1.enableOutputs();
  //pinMode(senable,OUTPUT);
  pinMode(s1step,OUTPUT);
  pinMode(s2dir,OUTPUT);
  pinMode(s2step,OUTPUT);
  pinMode(s1step,OUTPUT);
  //digitalWrite(senable,HIGH); //enable all steppers!
  // Configure each stepper
  stepper1.setMaxSpeed(300.0);
  stepper2.setMaxSpeed(300.0);
  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);

  int retry=0;
  Serial.begin(9600);
  SerialGPS.begin(9600,SERIAL_8N1,16,17);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  Serial.println("success!");
  display.display();

  vTaskDelay(1000); // Pause for 2 seconds
  
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  

  /* Set ESP32 to WiFi Station mode */
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
      while (!WiFi.smartConfigDone()) {
        delay(500);
        display.print(F("."));
        display.display();
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
  display.clearDisplay();
}
void displayInfo(){
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

String stepmsg = "steppers!";
int mot1steps = 0;
int mot2steps = 0;
int m1step = 300;
int m2step = 300;
bool smode = false;
bool did = false;
void loop() {
  if(micros()%500000==0){
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
  
  
  mot1steps = stepper1.currentPosition();
  mot2steps = stepper2.currentPosition();

  steppers.run();
  //stepper1.setSpeed(300.0);
  //stepper2.setSpeed(300.0);
  //stepper1.runSpeed();
  //stepper2.runSpeed();

  
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
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);
  display.println(stepmsg);
  display.println(mot1steps);
  display.println(mot2steps);
  display.display();
 }
}
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

#include <Preferences.h>

#include <Geometry.h>

#include "BluetoothSerial.h"
#include "CommandHandler.h"

#include <HttpClient.h>

//Bluetooth stuff/////////////////////////////
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;
const char* bluetooth_name = "isspointer3";
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
bool imhoming = false;
float accelxnum = 0;
float accelynum = 0;
float accelznum = 0;
float magxnum = 0;
float magynum = 0;
float magznum = 0;
int s1homing = 0; //0 is not homing, 1 is moving negative, 2 is moving positive
int s2homing = 0; //0 is not homing, 1 is moving negative, 2 is moving positive
//int displaystate = 0; //initial display mode

//wifi
String ssids_array[50];
String network_string;
String connected_string;
unsigned long waitstart = 0; //this is for asynchronous waiting
const char* pref_ssid = "";
const char* pref_pass = "";
String client_wifi_ssid;
String client_wifi_password;
long start_wifi_millis;
long wifi_timeout = 10000;
enum wifi_setup_stages { NONE, SCAN_START, SCAN_COMPLETE, SSID_ENTERED, WAIT_PASS, PASS_ENTERED, WAIT_CONNECT, LOGIN_FAILED };
enum wifi_setup_stages wifi_stage = NONE;


//http
const char kHostname[] =  "celestrak.com";
const char kPath[] = "/NORAD/elements/stations.txt";
const int kNetworkTimeout = 30*1000;
const int kNetworkDelay = 1000;
int getTLEwait = 0;
enum TLE_get_stages { WAITING, GET_BEGIN, GET_READDATA,TLE_FINISHED};
enum TLE_get_stages TLE_stage = WAITING;
String cur_TLE = "";
String getting_TLE = "";
int httpbodyLen = 0;
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
unsigned long httpTimeoutStart = 0;
WiFiClient w;
HttpClient http(w);



//EEPROM
Preferences preferences;
String satname = "";
String twolineelement = "";

//serial command handler!! 
CommandHandler<> Cmds(SerialBT);

void bt_connected_test(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  if(event == ESP_SPP_SRV_OPEN_EVT){
    Serial.println("Client Connected");
  }
}


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

const char *findAndReplace(const char *haystack, const char *needle, const char *repl)
{
    size_t needle_len;
    size_t repl_len;
    size_t haystack_len;
    const char *pos = strstr(haystack, needle);

    if(pos)
    {
        needle_len = strlen(needle);
        repl_len = strlen(repl);
        haystack_len = strlen(haystack);

        if(needle_len != repl_len)
        {
            memmove((void *)(pos + repl_len), (void *)(pos + needle_len), haystack_len - (pos - haystack) + 1);
        }

        memcpy((void *)pos, (void *)repl, repl_len);
    }
    return haystack;
}

char *findReplaceChar(char *haystack, char needle, char repl)
{
  //replace all instances of a character within a c-style string
  for (size_t i = 0; i < sizeof(haystack); i++)
  {
    if(haystack[i]==needle){
      haystack[i]= repl;
      Serial.println(haystack);
    }
    
  }
  return haystack;
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

void Cmd_GetGPS_Status(CommandParameter &p){
  uint32_t time_age = gps.time.age();
  uint32_t chars_processed = gps.charsProcessed();
  uint32_t sentences_passed = gps.passedChecksum();
  uint32_t sentences_failed = gps.failedChecksum();
  uint32_t gps_sats = gps.satellites.value();

  String dtext = "";
  dtext += "last updated "+String(time_age)+"\n";
  dtext += "processed "+String(chars_processed)+" chars\n";
  dtext += String(sentences_passed)+" passed and "+String(sentences_failed)+" failed\n";
  dtext += "found "+ String(gps_sats)+" satellites\n";
  SerialBT.println(dtext);
}

void Cmd_GetGPS_Data(CommandParameter &p){
  double gpslat = gps.location.lat();
  double gpslong = gps.location.lng();
  double gpsalt = gps.altitude.meters();
  uint32_t gpstime = gps.time.value();
  uint32_t gpsdate = gps.date.value();
  String dtext = "";
  dtext += "currently it is "+ String(gpstime) + " on " +String(gpsdate) +"\n";
  dtext += "our position is "+String(gpslat) + " , "+String(gpslong)+"\n";
  SerialBT.println(dtext);
}

void Cmd_SetWifi(CommandParameter &p){
  if(wifi_stage==SCAN_COMPLETE){
    wifi_stage = PASS_ENTERED;
  }
  client_wifi_ssid = p.NextParameter();
  String psk = String(p.NextParameter());
  psk.replace("\\a","!");
  client_wifi_password = psk;
  Serial.println(client_wifi_password);
}

void Cmd_SetLon(CommandParameter &p){
  String inputlong = String(p.NextParameter());
  float inputlongflt = strtof(inputlong.c_str(),NULL);
}

void Cmd_SetLat(CommandParameter &p){
  
}

void Cmd_SetAlt(CommandParameter &p){
  
}
void Cmd_Show_Ip(CommandParameter &p){
  SerialBT.print("my IP is "+WiFi.localIP());
}

void gps_process(){
  //process serial messages from the GPS
  while(SerialGPS.available() >0){
    gps.encode(SerialGPS.read());
  }
}

bool get_wifi_creds(String& ssid, String& psk){

}

void Cmd_get_TLE(CommandParameter &p){
  if(TLE_stage == WAITING){
    TLE_stage = GET_BEGIN;
  }else{
    SerialBT.println("TLE_stage not ready");
  }
}

void maintain_TLE(){
  int err = 0;
  switch(TLE_stage){
    case WAITING:
      break;
    case GET_BEGIN:
      err = 0;
      getting_TLE = "";
      if(WiFi.status() == WL_CONNECTED){
        err = http.get(kHostname, kPath);
        if(err >=0){
          err = http.skipResponseHeaders();
          if (err >= 0){
            httpbodyLen = http.contentLength();
            httpTimeoutStart = millis();
            TLE_stage = GET_READDATA;
          }else{
          SerialBT.println("error2");
          }
        }else{
          SerialBT.println("error1");
        }
      }else{
        SerialBT.println("wifi is off, not getting TLE");
        TLE_stage = WAITING;
      }
      break;
    case GET_READDATA:
      if((http.connected() || http.available()) &&
          ((millis() - httpTimeoutStart) < kNetworkTimeout)){
            while (http.available()){
              // Print out this character
              getting_TLE+=http.read();
              // We read something, reset the timeout counter
            }  
            httpTimeoutStart = millis();
          }else{
            TLE_stage = TLE_FINISHED;
          }
      break;
    case TLE_FINISHED:
      SerialBT.println(getting_TLE);
      getting_TLE = "";
      TLE_stage = WAITING;
      break;
  }
}
  

bool init_wifi(String desired_ssid, String desired_psk)
{
  const char* cstr_desired_ssid = desired_ssid.c_str();
  const char* cstr_desired_psk = desired_psk.c_str();
  Serial.println("attempting connection with");
  Serial.println(desired_ssid);
  Serial.println(desired_psk);

  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);

  start_wifi_millis = millis();
  WiFi.begin(cstr_desired_ssid, cstr_desired_psk);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - start_wifi_millis > wifi_timeout) {
      WiFi.disconnect(true, true);
      return false;
    }
  }
  Serial.println("connection worked! saving data in EEPROM");
  preferences.putString("pref_ssid", desired_ssid);
  preferences.putString("pref_pass", desired_psk);
  return true;
}

void scan_wifi_networks()
{
  WiFi.mode(WIFI_STA);
  // WiFi.scanNetworks will return the number of networks found
  int n =  WiFi.scanNetworks();
  if (n == 0) {
    SerialBT.println("no networks found");
  } else {
    SerialBT.println();
    SerialBT.print(n);
    SerialBT.println(" networks found");
    if(waitstart==0){
        waitstart = millis();
    }else if(millis()-waitstart>1000){
      waitstart = 0;
      for (int i = 0; i < n; ++i) {
        ssids_array[i + 1] = WiFi.SSID(i);
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(ssids_array[i + 1]);
        network_string = i + 1;
        network_string = network_string + ": " + WiFi.SSID(i) + " (Strength:" + WiFi.RSSI(i) + ")";
        SerialBT.println(network_string);
      }
      wifi_stage = SCAN_COMPLETE;
    }
    
  }
}



void wifiMaintain(){
  switch(wifi_stage){
    case SCAN_START:
      WiFi.mode(WIFI_STA);
      Serial.println("scan_start");
      wifi_stage = SCAN_COMPLETE;
      break;
    case SCAN_COMPLETE:
      if(waitstart==0){
          waitstart = millis();
      }else if(millis()-waitstart>500){
        waitstart = 0;
        SerialBT.println("NEEDSSID");
      }
      break;
    case PASS_ENTERED:
      Serial.println("PASS_ENTERED");
      wifi_stage = WAIT_CONNECT;
      
      if (init_wifi(client_wifi_ssid,client_wifi_password)) { // Connected to WiFi
        connected_string = "SUCCESS";
        
        SerialBT.println(connected_string);
      } else { // try again
        wifi_stage = LOGIN_FAILED;
      }
      break;
    
    case LOGIN_FAILED:
      if(waitstart==0){
        Serial.println("LOGIN_FAILED");
        SerialBT.println("Wi-Fi connection failed");
        waitstart = millis();
      }else if(millis()-waitstart>2000){
        wifi_stage = SCAN_START;
        waitstart = 0;
      }
      break;
    
  }
}

void setup() {
  if(true){ //pin configs
    pinMode(s1step,OUTPUT);
    pinMode(s1dir,OUTPUT);
    pinMode(s2dir,OUTPUT);
    pinMode(s2step,OUTPUT);
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
  SerialBT.register_callback(bt_connected_test);
  if(!SerialBT.begin(bluetooth_name)){
    Serial.println("An error occurred initializing Bluetooth");
  }else{
    Serial.println("Bluetooth initialized");
  }
  //file system
  preferences.begin("isspointer", false);
  //command handler
  Cmds.AddCommand(F("GetAccelStatus"), Cmd_GetAccelStatus);
  Cmds.AddCommand(F("GetStepperPos"),Cmd_GetStepperPos);
  Cmds.AddCommand(F("GetGPSStatus"),Cmd_GetGPS_Status);
  Cmds.AddCommand(F("GetGPSData"),Cmd_GetGPS_Data);
  Cmds.AddCommand(F("GetWifiStat"),Cmd_Show_Ip);
  Cmds.AddCommand(F("SWi"),Cmd_SetWifi);
  Cmds.AddCommand(F("GetTLE"),Cmd_get_TLE);
  //GPS
  SerialGPS.begin(9600,SERIAL_8N1,16,17);
  ///////////////////////////////
  //accelerometer and magneto
  accelerometer.begin();
  magnetometer.enableAutoRange(true);
  magnetometer.begin();
  ///////////////////////////////
  //wifi
  String temp_pref_ssid = preferences.getString("pref_ssid","");
  String temp_pref_pass = preferences.getString("pref_pass","");
  client_wifi_ssid = temp_pref_ssid.c_str();
  client_wifi_password = temp_pref_pass.c_str();
  wifi_stage = PASS_ENTERED;
  //EEPROM

  ////////////////////////////
  s1homing = 1;
  s2homing = 1;
  


}

void loop() {
  Cmds.Process();
  gps_process();
  wifiMaintain();
  maintain_TLE();
  imhoming = (s1homing>0) || (s2homing>0);
  if(millis()%5000<3 && !imhoming){ //stepper jog for testing
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
  if(imhoming){ //stepper homing
    bool s1homeval = digitalRead(s1home) && digitalRead(s1home);
    bool s2homeval = digitalRead(s2home) && digitalRead(s2home);

    if((s1homing==1) && s1homeval){
      stepper1.setSpeed(homespeed);
      stepper1.runSpeed();
    }else if((s1homing==1) && !s1homeval){
      s1homing = 2;
      stepper1.setSpeed(homespeed*-.5);
      stepper1.runSpeed();
    }else if((s1homing==2) && s1homeval){
      s1homing = 0;
      stepper1.setCurrentPosition(0);
    }

    if((s2homing==1) && s2homeval){
      stepper2.setSpeed(homespeed);
      stepper2.runSpeed();
    }else if((s2homing==1) && !s2homeval){
      s2homing = 2;
      stepper2.setSpeed(homespeed*-.5);
      stepper2.runSpeed();
    }else if((s2homing==2) && s2homeval){
      s2homing = 0;
      stepper2.setCurrentPosition(0);
    }
  }else if(millis()%5000==0){
    
  }
  
  
  steppers.run();
}
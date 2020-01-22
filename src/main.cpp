#include <Arduino.h>
#include "WiFi.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  Serial.println("succes!");
  display.display();

  vTaskDelay(1000); // Pause for 2 seconds
  
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  

  /* Set ESP32 to WiFi Station mode */
  WiFi.mode(WIFI_AP_STA);
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
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
}
void loop() {
  
  display.println(F("Hello, world!"));
  display.display();
  delay(2000);
  //display.clearDisplay();
}
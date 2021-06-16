/*config.h*/

/* List of Supported Fonts
 *
  Arial14,
  Arial_bold_14,
  Callibri11,
  Callibri11_bold,
  Callibri11_italic,
  Callibri15,
  Corsiva_12,
  fixed_bold10x15,
  font5x7,    //Do not use in LARGE_FONT, can use as default font
  font8x8,
  Iain5x7,    //Do not use in LARGE_FONT, can use as default font
  lcd5x7,     //Do not use in LARGE_FONT, can use as default font
  Stang5x7,   //Do not use in LARGE_FONT, can use as default font
  System5x7,  //Do not use in LARGE_FONT, can use as default font
  TimesNewRoman16,
  TimesNewRoman16_bold,
  TimesNewRoman16_italic,
  utf8font10x16,
  Verdana12,
  Verdana12_bold,
  Verdana12_italic,
  X11fixed7x14,
  X11fixed7x14B,
  ZevvPeep8x16
 *  
 */
#ifndef CONFIG_H
#define CONFIG_H
#define OLED_I2C_ADDRESS 0x78     //Defined OLED I2C Address

/*
 * Define your font from the list. 
 * Default font: lcd5x7
 * Comment out the following for using the default font.
 */
//#define LARGE_FONT Verdana12

//Navigate buttons
#define BTN_SEL     35                 // Select button
#define BTN_UP     34                // Up Button
#define BTN_DOWN    32                 // Down Button
//#define BTN_ESC     1                 // Exit Button
// Comment the following to disable internal pullup for Navigate buttons
//#define NAV_BUTTONS_INPUT_PULLUP

#define TOTAL_NAV_BUTTONS 3       // Total Navigation Button used

/*Demonstrate PWM with LED on D11*/
#define LED_PIN 2               //Built in LED pin in ESP32S board

#define MAX_DEPTH 2

#ifdef LOC
// #define LARGE_FONT
#define INV
#endif
 
 /*Do not change the values(recomended)*/
#ifdef LARGE_FONT
#define menuFont LARGE_FONT
#define fontW 8
#define fontH 16
#else
// #define menuFont System5x7
#define menuFont lcd5x7
#define fontW 6
#define fontH 8
#endif

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define DISPLAY_ADDRESS 0x3C//0x3C //OLED display I2C address
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

//stepper pins

#define senable 2 //12

#define s1step 27 //10
#define s1dir  26 //oogabooga
#define s2step 25 //11
#define s2dir 33 //8

#define s1home 36 //homing switch for stepper 1
#define s2home 39 //homing switch for stepper 2

#define EEPROM_SIZE 1024
#define homespeed -150

#endif
/* -*- C++ -*- */

//using Adafruit_SSD1306
//https://github.com/adafruit/Adafruit_SSD1306

#ifndef RSITE_ARDUINO_MENU_ADAFRUITSSD1306
  #define RSITE_ARDUINO_MENU_ADAFRUITSSD1306

  #include "MenuDefs.h"
  #include <Adafruit_SSD1306.h>
  // #include <Wire.h>
  // #include <LCD.h>

  namespace Menu {

    class Adafruit_SSD1306Out:public gfxOut {
      public:
        Adafruit_SSD1306* device;
        Adafruit_SSD1306Out(
          Adafruit_SSD1306* o,
  				idx_t* t,
  				panelsList &p,
  				idx_t resX=6,
  				idx_t resY=8,
          menuOut::styles s=menuOut::minimalRedraw
  			):gfxOut(resX,resY,t,p,s),device(o) {}
        void init() {
          //resX=7;
          //device->fontHeight();
          //resY=7;
          //device->fontWidth();
          fontMarginX=0;
          fontMarginY=0;
        }
        size_t write(uint8_t ch) override {return device->write(ch);}
        void clearLine(idx_t ln,idx_t panelNr=0,colorDefs color=bgColor,bool selected=false,status stat=enabledStatus,bool edit=false) override {
          const panel p=panels[panelNr];
          int y=(p.y+ln)*resY;
          //int dispwidth = device->width();
          device->fillRect(resX*p.x,y,p.w*resX,resY*1,0);
          //device->clearDisplay(resX*p.x,resX*(p.x+maxX(panelNr))-1,y,y+resY-1);
        }
        void clear() override {
          trace(Serial<<"clear!!"<<endl;);
          device->clearDisplay();
          panels.reset();
        }
        void clear(idx_t panelNr) override {
          const panel p=panels[panelNr];
          trace(Serial<<"clear panel!! "<<panelNr<<"{"<<p.x<<","<<p.y<<","<<p.w<<","<<p.h<<"}"<<endl;);
          device->fillRect(resX*p.x,resY*p.y,resX*p.w,resY*p.h,0);
          //device->clearDisplay(resX*p.x,resX*(p.x+p.w)-1,resY*p.y,resY*(p.y+p.h)-1);
          // setCursor(0,0,panelNr);
          panels.nodes[panelNr]=NULL;
        }
        void setCursor(idx_t x,idx_t y,idx_t panelNr=0) override {
          const panel p=panels[panelNr];
          device->setCursor((p.x+x)*resX,(p.y+y)*resY);
        }
    };

  }//namespace Menu

#endif
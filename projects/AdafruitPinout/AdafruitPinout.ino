//This example implements a simple sliding On/Off button. The example
// demonstrates drawing and touch operations.
//
//Thanks to Adafruit forums member Asteroid for the original sketch!
//
#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ILI9341.h>
#include <stdint.h>
#include "TouchScreen.h"

#define YP 12  // must be an analog pin, use "An" notation!
#define XM 13  // must be an analog pin, use "An" notation!
#define YM 14   // can be a digital pin
#define XP 27   // can be a digital pin

#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any multimeter to read it
// Mine is 322 ohms across the X plate

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 322);

#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_CLK  18
#define TFT_CS   32
#define TFT_DC    2
#define TFT_RST   4
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

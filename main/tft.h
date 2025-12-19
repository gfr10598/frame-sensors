#ifndef TFT_H
#define TFT_H

#include "Adafruit_ST7789.h"
#include "Adafruit_GFX.h"
#include <SPI.h>

#define MOSI 35
#define SCK 36
#define MISO 37
#define TFT_CS 42

#define TFT_I2C_POWER 7
#define TFT_BACKLITE 45

void init_SPI();
void setup_tft();

#endif // TFT_H

#include "lcd_display.h"

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>

#include "config.h"

namespace {

LiquidCrystal_I2C g_lcd27(0x27, LCD_COLS, LCD_ROWS);
LiquidCrystal_I2C g_lcd3f(0x3F, LCD_COLS, LCD_ROWS);
LiquidCrystal_I2C *g_lcd = nullptr;

bool g_lcdAvailable = false;
bool g_lcdWarnPrinted = false;
bool g_lcdShowAltLine = false;
uint8_t g_lcdAddr = LCD_I2C_ADDR;

bool probeI2cAddress(const uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void buildLcdLine1(const SensorData &data, char *out, const size_t outSize) {
  if (data.dhtOk) {
    snprintf(out, outSize, "T:%4.1fC H:%3.0f%%", data.temperatureC, data.humidityPct);
  } else {
    snprintf(out, outSize, "T:ERR H:ERR");
  }
}

void buildLcdLine2Sensor(const SensorData &data, char *out, const size_t outSize) {
  const int luxInt = data.bh1750Ok ? static_cast<int>(data.lux + 0.5f) : -1;
  if (data.bh1750Ok) {
    snprintf(out, outSize, "Lux:%6d lx", luxInt);
  } else {
    snprintf(out, outSize, "Lux:ERR");
  }
}

void buildLcdLineIo1(const SensorData &data, char *out, const size_t outSize) {
  snprintf(out, outSize, "S:%4u R:%4u", data.soilAO, data.rainAO);
}

void buildLcdLineIo2(const SensorData &data, char *out, const size_t outSize) {
  snprintf(out, outSize, "SD:%u RD:%u", data.soilDO, data.rainDO);
}

void fitToLcd16(char *line) {
  const size_t len = strlen(line);
  if (len >= LCD_COLS) {
    line[LCD_COLS] = '\0';
    return;
  }
  for (size_t i = len; i < LCD_COLS; ++i) {
    line[i] = ' ';
  }
  line[LCD_COLS] = '\0';
}

}  // namespace

void initLcdIfPresent() {
#if APP_MODE_SIMULATOR
  g_lcdAvailable = false;
#else
  if (probeI2cAddress(0x27)) {
    g_lcd = &g_lcd27;
    g_lcdAddr = 0x27;
    g_lcdAvailable = true;
  } else if (probeI2cAddress(0x3F)) {
    g_lcd = &g_lcd3f;
    g_lcdAddr = 0x3F;
    g_lcdAvailable = true;
  } else {
    g_lcdAvailable = false;
  }

  if (g_lcdAvailable && g_lcd != nullptr) {
    g_lcd->init();
    g_lcd->backlight();
    g_lcd->clear();
    Serial.print(F("LCD_OK 0x"));
    if (g_lcdAddr < 16) {
      Serial.print('0');
    }
    Serial.println(g_lcdAddr, HEX);
  }
#endif

  if (!g_lcdAvailable && !g_lcdWarnPrinted) {
    Serial.println(F("LCD_WARN (addr 0x27/0x3F not found)"));
    g_lcdWarnPrinted = true;
  }
}

void updateLcd(const SensorData &data) {
  if (!g_lcdAvailable || g_lcd == nullptr) {
    return;
  }

  char line1[LCD_COLS + 1] = {0};
  char line2[LCD_COLS + 1] = {0};

  if (g_lcdShowAltLine) {
    buildLcdLineIo1(data, line1, sizeof(line1));
    buildLcdLineIo2(data, line2, sizeof(line2));
  } else {
    buildLcdLine1(data, line1, sizeof(line1));
    buildLcdLine2Sensor(data, line2, sizeof(line2));
  }
  g_lcdShowAltLine = !g_lcdShowAltLine;

  fitToLcd16(line1);
  fitToLcd16(line2);

  g_lcd->setCursor(0, 0);
  g_lcd->print(line1);
  g_lcd->setCursor(0, 1);
  g_lcd->print(line2);
}

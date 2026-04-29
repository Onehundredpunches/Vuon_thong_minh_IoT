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
uint8_t g_lcdAddr = LCD_I2C_ADDR;
uint8_t g_lcdPage = 0;
uint32_t g_lastLcdTickMs = 0;
uint32_t g_lastPageSwitchMs = 0;
char g_lastLine1[LCD_COLS + 1] = {0};
char g_lastLine2[LCD_COLS + 1] = {0};

static constexpr uint32_t LCD_TICK_INTERVAL_MS = 200;
static constexpr uint32_t LCD_PAGE_INTERVAL_MS = 5000;

bool probeI2cAddress(const uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

const char *modeToken(const SystemMode mode) {
  switch (mode) {
    case SystemMode::Auto:
      return "AU";
    case SystemMode::Manual:
      return "MN";
    case SystemMode::SafeStop:
      return "SS";
    case SystemMode::Boot:
    case SystemMode::Error:
      return "ER";
  }
  return "ER";
}

int roundFloatToInt(const float value) {
  if (value >= 0.0f) {
    return static_cast<int>(value + 0.5f);
  }
  return static_cast<int>(value - 0.5f);
}

void format16(const char *src, char *out) {
  size_t i = 0;
  if (src == nullptr) {
    src = "";
  }
  for (; i < LCD_COLS && src[i] != '\0'; ++i) {
    out[i] = src[i];
  }
  for (; i < LCD_COLS; ++i) {
    out[i] = ' ';
  }
  out[LCD_COLS] = '\0';
}

void buildPage1Line1(const LcdDisplayState &state, char *out) {
  char raw[32] = {0};
  snprintf(raw, sizeof(raw), "M:%s Wi:%s Mq:%s", modeToken(state.mode), state.wifiOk ? "OK" : "ER",
           state.mqttOk ? "OK" : "ER");
  format16(raw, out);
}

void buildPage1Line2(const LcdDisplayState &state, char *out) {
  char raw[32] = {0};
  if (state.sensors.dht_ok && state.sensors.soil_ok) {
    snprintf(raw, sizeof(raw), "T:%d H:%d S:%d%%", roundFloatToInt(state.sensors.temperatureC),
             roundFloatToInt(state.sensors.humidityPct), roundFloatToInt(state.sensors.soilPct));
  } else {
    snprintf(raw, sizeof(raw), "T:ER H:ER S:ER");
  }
  format16(raw, out);
}

void buildPage2Line1(const LcdDisplayState &state, char *out) {
  char raw[32] = {0};
  snprintf(raw, sizeof(raw), "P:%s F:%s L:%s", state.actuators.pumpOn ? "ON" : "OFF",
           state.actuators.fanOn ? "ON" : "OFF", state.actuators.lightOn ? "ON" : "OFF");
  format16(raw, out);
}

void buildPage2Line2(const LcdDisplayState &state, char *out) {
  char raw[32] = {0};
  const char *rain = (state.sensors.rain_ok && state.sensors.rainDO == 0) ? "WET" : "DRY";
  snprintf(raw, sizeof(raw), "R:%s W:%d%% C:%s", rain, roundFloatToInt(state.sensors.soilPct),
           state.controlOwner);
  format16(raw, out);
}

void clearDirtyCache() {
  memset(g_lastLine1, 0, sizeof(g_lastLine1));
  memset(g_lastLine2, 0, sizeof(g_lastLine2));
}

bool expectFormat16(const char *label, const char *input, const char *expected) {
  char out[LCD_COLS + 1] = {0};
  format16(input, out);
  const bool pass = strcmp(out, expected) == 0 && strlen(out) == LCD_COLS;
  if (!pass) {
    Serial.print(F("LCD_FORMAT_VECTOR_FAIL "));
    Serial.print(label);
    Serial.print(F(" got='"));
    Serial.print(out);
    Serial.print(F("' expected='"));
    Serial.print(expected);
    Serial.println(F("'"));
  }
  return pass;
}

bool expectFixedWidth(const char *label, const char *line) {
  const bool pass = strlen(line) == LCD_COLS;
  if (!pass) {
    Serial.print(F("LCD_PAGE_WIDTH_FAIL "));
    Serial.print(label);
    Serial.print(F(" len="));
    Serial.println(strlen(line));
  }
  return pass;
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
    clearDirtyCache();
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

void updateLcd(const uint32_t nowMs, const LcdDisplayState &state) {
  if (!g_lcdAvailable || g_lcd == nullptr) {
    return;
  }
  if (static_cast<uint32_t>(nowMs - g_lastLcdTickMs) < LCD_TICK_INTERVAL_MS) {
    return;
  }
  g_lastLcdTickMs = nowMs;
  if (static_cast<uint32_t>(nowMs - g_lastPageSwitchMs) >= LCD_PAGE_INTERVAL_MS) {
    g_lastPageSwitchMs = nowMs;
    g_lcdPage = (g_lcdPage == 0) ? 1 : 0;
  }

  char line1[LCD_COLS + 1] = {0};
  char line2[LCD_COLS + 1] = {0};

  if (g_lcdPage == 0) {
    buildPage1Line1(state, line1);
    buildPage1Line2(state, line2);
  } else {
    buildPage2Line1(state, line1);
    buildPage2Line2(state, line2);
  }

  if (strcmp(line1, g_lastLine1) == 0 && strcmp(line2, g_lastLine2) == 0) {
    return;
  }
  strncpy(g_lastLine1, line1, sizeof(g_lastLine1));
  strncpy(g_lastLine2, line2, sizeof(g_lastLine2));

  g_lcd->setCursor(0, 0);
  g_lcd->print(line1);
  g_lcd->setCursor(0, 1);
  g_lcd->print(line2);
}

bool lcdDeviceAvailable() {
  return g_lcdAvailable && g_lcd != nullptr;
}

bool runLcdFormatterSelfTest() {
  bool pass = true;
  pass &= expectFormat16("empty", "", "                ");
  pass &= expectFormat16("null", nullptr, "                ");
  pass &= expectFormat16("pad", "OK", "OK              ");
  pass &= expectFormat16("exact16", "1234567890123456", "1234567890123456");
  pass &= expectFormat16("truncate", "12345678901234567890", "1234567890123456");

  LcdDisplayState state{};
  state.mode = SystemMode::Auto;
  state.wifiOk = true;
  state.mqttOk = false;
  state.controlOwner = "AUTO";
  state.sensors.temperatureC = 28.4f;
  state.sensors.humidityPct = 65.2f;
  state.sensors.soilPct = 42.0f;
  state.sensors.dht_ok = true;
  state.sensors.soil_ok = true;
  state.sensors.rain_ok = true;
  state.sensors.rainDO = 1;
  state.actuators.pumpOn = true;
  state.actuators.fanOn = false;
  state.actuators.lightOn = true;

  char out[LCD_COLS + 1] = {0};
  buildPage1Line1(state, out);
  pass &= expectFixedWidth("page1_line1", out);
  buildPage1Line2(state, out);
  pass &= expectFixedWidth("page1_line2", out);
  buildPage2Line1(state, out);
  pass &= expectFixedWidth("page2_line1", out);
  buildPage2Line2(state, out);
  pass &= expectFixedWidth("page2_line2", out);

  Serial.print(F("LCD_FORMATTER_SELF_TEST: "));
  Serial.println(pass ? F("PASS") : F("FAIL"));
  return pass;
}

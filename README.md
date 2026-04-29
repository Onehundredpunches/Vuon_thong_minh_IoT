# ESP32 Dual-Mode Sensor App (PlatformIO)

This project targets **ESP32 DevKit V1** (Arduino framework) and is hardware-first.

- `APP_MODE_SIMULATOR=0`: real hardware reads (default)
- `APP_MODE_SIMULATOR=1`: deterministic simulation fallback

## Fixed Pin Map

- I2C SDA: GPIO21
- I2C SCL: GPIO22
- DHT22 DATA: GPIO27
- Soil AO: GPIO34
- Soil DO: GPIO26
- Rain AO: GPIO35
- Rain DO: GPIO25
- Servo PWM: GPIO19
- Light relay: GPIO18
- Fan relay: GPIO17
- Pump relay: GPIO16

## Build Modes

`platformio.ini` provides two environments:

- `esp32dev_hw` (default): defines `APP_MODE_SIMULATOR=0`
- `esp32dev_sim`: defines `APP_MODE_SIMULATOR=1`

If no build flag is provided, `include/config.h` defaults to hardware mode.

## Commands

- Build default (hardware): `platformio.exe run`
- Build explicit hardware: `platformio.exe run -e esp32dev_hw`
- Build simulator fallback: `platformio.exe run -e esp32dev_sim`
- Upload hardware: `platformio.exe run -e esp32dev_hw -t upload`
- Serial monitor: `platformio.exe device monitor -b 115200`

## Runtime Behavior

- Serial at `115200`
- Non-blocking loop with `2000 ms` interval
- Startup prints mode, pin map, relay active level, and I2C scan results
- Each cycle prints:
  - `Temp, Humidity, Lux, SoilAO, RainAO, SoilDO, RainDO`
  - AO raw + voltage using 3.3V and 12-bit ADC (`0..4095`)
- LCD 16x2:
  - sensor page: `T:xx.xC H:yy%` / `Lux:zzzz lx`
  - IO page: `S:raw R:raw` / `SD:x RD:y`
  - pages alternate each cycle:
    - sensor page: temperature, humidity, lux
    - IO page: SoilAO, RainAO, SoilDO, RainDO

## Error Handling (Hardware Mode)

- DHT read failure: loop continues, serial prints `DHT_ERR`
- BH1750 read failure: loop continues, serial prints `BH1750_ERR`
- LCD missing: app continues serial-only, prints `LCD_WARN` once

## Sample Serial Output (3 cycles)

```text
MODE: HARDWARE
PINMAP I2C_SDA=21 I2C_SCL=22 DHT=27 SoilAO=34 SoilDO=26 RainAO=35 RainDO=25 SERVO=19 LIGHT=18 FAN=17 PUMP=16
RELAY_ACTIVE_LEVEL: ACTIVE_HIGH
I2C_SCAN 23,27
LCD_OK 0x27
SELF_TEST: PASS
DATA Temp:29.4C Hum:67.1%RH Lux:12450.0lx SoilAO:1870(1.507V) RainAO:2422(1.952V) SoilDO:0 RainDO:1
DATA Temp:29.5C Hum:67.0%RH Lux:12410.0lx SoilAO:1865(1.503V) RainAO:2430(1.958V) SoilDO:0 RainDO:1
DATA Temp:29.5C Hum:66.9%RH Lux:12395.0lx SoilAO:1862(1.501V) RainAO:2441(1.967V) SoilDO:0 RainDO:1
```

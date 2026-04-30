# ESP32 Smart Garden IoT Firmware

PlatformIO firmware for an ESP32 DevKit V1 smart garden controller. The current implementation is hardware-first, with a simulator build for deterministic compile/runtime checks.

## Architecture

`src/main.cpp` is intentionally thin and delegates Arduino lifecycle calls to `AppController`.

| Module | Responsibility |
|---|---|
| `app_controller` | Bootstrap and loop orchestration |
| `sensor_manager` | DHT22, BH1750, soil AO, rain DO/AO reads, EMA, debounce, stale flags |
| `actuator_manager` | Relay/servo control, cooldowns, timeouts, roof interlock |
| `mode_controller` | BOOT/AUTO/MANUAL/SAFE_STOP/ERROR state machine |
| `auto_logic` | Strawberry threshold automation and stale-sensor safe behavior |
| `command_handler` | Shared serial/MQTT command execution and mode gate |
| `mqtt_manager` | Wi-Fi, MQTT, reconnect, command ACK, retained state/status |
| `lcd_display` | Non-blocking 16x2 LCD scheduler and fixed-width formatter |
| `serial_cli` | Runtime serial commands, including log level switch |
| `self_test` | Boot validation for pin map, actuators, sensors, FSM, MQTT, integration |

## Locked Hardware Pin Map

Do not change these pins without updating the execution pack and hardware schematic.

| Function | GPIO |
|---|---:|
| I2C SDA | 21 |
| I2C SCL | 22 |
| DHT22 DATA | 27 |
| Soil AO | 34 |
| Soil DO | 26 |
| Rain AO | 35 |
| Rain DO | 25 |
| Roof servo PWM | 19 |
| Light relay | 18 |
| Fan relay | 17 |
| Pump relay | 16 |

## Build And Upload

Use the PlatformIO executable available in the active environment, or the explicit Windows path:

```powershell
C:\Users\nguye\.platformio\penv\Scripts\pio.exe run -e esp32dev_sim
C:\Users\nguye\.platformio\penv\Scripts\pio.exe run -e esp32dev_hw
C:\Users\nguye\.platformio\penv\Scripts\pio.exe run -e esp32dev_hw -t upload --upload-port COM4
```

Serial monitor:

```powershell
C:\Users\nguye\.platformio\penv\Scripts\pio.exe device monitor -p COM4 -b 115200
```

Native contract tests:

```powershell
python test\test_native\test_contracts.py
```

## Runtime Configuration

Build environments:

| Environment | Purpose |
|---|---|
| `esp32dev_hw` | Real hardware, `APP_MODE_SIMULATOR=0` |
| `esp32dev_sim` | Simulator fallback, `APP_MODE_SIMULATOR=1` |

Current demo Wi-Fi/MQTT settings live in `include/config.h`:

| Setting | Value |
|---|---|
| Primary SSID | `Trang 72` |
| MQTT demo host | `broker.hivemq.com` |
| MQTT demo port | `1883` |
| MQTT client ID | `vuon-iot1` |

The Wi-Fi password is configured in firmware for the demo run, but it must not be printed in logs.

## MQTT Contract

Topic names are locked:

| Topic | Direction | Retain |
|---|---|---|
| `vuon-iot/zone1/gateway1/telemetry` | PUB | No |
| `vuon-iot/zone1/gateway1/command` | SUB | No |
| `vuon-iot/zone1/gateway1/command/ack` | PUB | No |
| `vuon-iot/zone1/gateway1/state/sensor` | PUB | Yes |
| `vuon-iot/zone1/gateway1/state/actuator` | PUB | Yes |
| `vuon-iot/zone1/gateway1/status` | PUB | Yes |

Command rules:

- `cmdId` is required, non-empty, and at most 64 characters.
- Duplicate `cmdId` values inside the 16-entry/60s window are rejected with `duplicate_cmdId`.
- `AUTO + set_actuator` is rejected with `invalid_mode`.
- Backend/mobile must switch to `MANUAL` first, wait for ACK/state, then send actuator commands.
- `query_state` ACKs `success` and republishes retained sensor, actuator, and status snapshots.

Reconnect behavior:

- Wi-Fi sleep is disabled.
- Wi-Fi scan is diagnostic only; bounded direct-connect fallback is allowed if scan fails or misses the SSID.
- Wi-Fi and MQTT reconnect are non-blocking and keep local control alive.
- On MQTT reconnect, firmware resubscribes to `command` and republishes retained `state/sensor`, `state/actuator`, and `status`.
- LWT publishes retained offline status; reconnect overwrites status with online boot/reconnect state.

## Serial CLI

Runtime serial commands include:

```text
help
log debug
log info
log warn
log error
mode auto
mode manual
servo on
servo off
servo sweep on
servo sweep off
servo stop
servo 0
servo 90
light on
light off
light toggle
light blink
fan on
fan off
pump on
pump off
```

Direct actuator commands are also mode-gated. In `AUTO`, direct actuator commands are rejected; switch to `MANUAL` first.

## Safety Behavior

- Unified sensor flags are `dht_ok`, `bh1750_ok`, `soil_ok`, `rain_ok`.
- Payloads keep all fields and mark `sensor_invalid=true` when any control sensor is invalid/stale.
- AUTO does not continue actuation from stale control data:
  - stale/invalid soil disables auto-pump,
  - stale/invalid DHT disables auto-fan,
  - stale/invalid BH1750 disables auto-light,
  - stale/invalid rain closes roof to the safe position.
- Rain DO is the primary rain signal.
- Soil AO is the primary soil moisture signal.
- Manual timeout returns to AUTO and runs auto logic on the next control cycle without a broad all-off action unless safety/interlock requires it.

## LCD Behavior

The LCD path is non-blocking:

- 16x2 display.
- 5 seconds per page using `millis()`.
- Every line is fixed-width 16 characters, padded or truncated.
- Dirty-check avoids unnecessary I2C writes.

## Acceptance Checklist

Run this before release-candidate handoff:

```powershell
python test\test_native\test_contracts.py
C:\Users\nguye\.platformio\penv\Scripts\pio.exe run -e esp32dev_sim
C:\Users\nguye\.platformio\penv\Scripts\pio.exe run -e esp32dev_hw
C:\Users\nguye\.platformio\penv\Scripts\pio.exe run -e esp32dev_hw -t upload --upload-port COM4
```

Hardware serial evidence should include:

```text
SELF_TEST: PASS
FULL_INTEGRATION_SELF_TEST: PASS
WIFI_CONNECTED ip=...
MQTT_CONNECTED
MQTT_RETAINED_SNAPSHOT reason=boot
DATA ... Flags dht_ok:1 bh1750_ok:1 soil_ok:1 rain_ok:1
```

MQTT regression checks should confirm:

- AUTO rejects `set_actuator` with `invalid_mode`.
- `set_mode manual` commits mode before ACK success.
- MANUAL actuator command ACKs `success`.
- duplicate `cmdId` ACKs `duplicate_cmdId` and does not execute again.
- retained `state/sensor`, `state/actuator`, and `status` republish after reconnect.

## Known Limits And Pending Validation

- `broker.hivemq.com:1883` is a demo environment only.
- Broker provisioning, ACL, auth, clientId policy, QoS enforcement, and owner-managed retain/LWT validation are project-owner responsibilities.
- The current ACK means firmware validated, committed state where applicable, and invoked the actuator path. It is not closed-loop physical confirmation.
- 12-24 hour soak testing is still pending.

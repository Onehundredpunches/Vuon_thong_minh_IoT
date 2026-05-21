import pathlib
import re
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]


def read_text(relative_path):
    return (ROOT / relative_path).read_text(encoding="utf-8")


class FirmwareContractTests(unittest.TestCase):
    def test_mqtt_topics_are_locked(self):
        topics = read_text("include/mqtt_topics.h")
        expected = {
            "kTelemetry": "vuon-iot/zone1/gateway1/telemetry",
            "kCommand": "vuon-iot/zone1/gateway1/command",
            "kCommandAck": "vuon-iot/zone1/gateway1/command/ack",
            "kStateSensor": "vuon-iot/zone1/gateway1/state/sensor",
            "kStateActuator": "vuon-iot/zone1/gateway1/state/actuator",
            "kStatus": "vuon-iot/zone1/gateway1/status",
        }
        for symbol, value in expected.items():
            self.assertIn(f'{symbol} = "{value}"', topics)

    def test_pin_map_is_unchanged(self):
        config = read_text("include/config.h")
        expected = {
            "PIN_I2C_SDA": "21",
            "PIN_I2C_SCL": "22",
            "PIN_DHT22_DATA": "27",
            "PIN_SOIL_AO": "34",
            "PIN_SOIL_DO": "26",
            "PIN_RAIN_AO": "35",
            "PIN_RAIN_DO": "25",
            "PIN_SERVO_ROOF": "19",
            "PIN_RELAY_LIGHT": "18",
            "PIN_RELAY_FAN": "17",
            "PIN_RELAY_PUMP": "16",
        }
        for symbol, value in expected.items():
            self.assertRegex(config, rf"{symbol}\s*=\s*{value}\b")

    def test_wifi_secret_not_printed(self):
        mqtt_manager = read_text("src/mqtt_manager.cpp")
        self.assertNotRegex(mqtt_manager, r"Serial\.(?:print|println)\s*\(\s*WIFI_PRIMARY_PASSWORD")
        self.assertNotRegex(mqtt_manager, r"Serial\.(?:print|println)\s*\(\s*WIFI_FALLBACK_PASSWORD")
        self.assertNotIn("79797979", mqtt_manager)

    def test_wifi_primary_fallback_contract_is_configured(self):
        config = read_text("include/config.h")
        mqtt_manager = read_text("src/mqtt_manager.cpp")
        self.assertIn('WIFI_PRIMARY_SSID = "pekkunu"', config)
        self.assertIn('WIFI_PRIMARY_PASSWORD = "44444444"', config)
        self.assertIn('WIFI_FALLBACK_SSID = "San bat cuop Thu Duc"', config)
        self.assertIn("WIFI_PRIMARY_CONNECT_TIMEOUT_MS = 15000", config)
        for token in [
            "WIFI_FALLBACK_START reason=primary_scan_miss",
            "WIFI_CONNECT_TIMEOUT profile=primary",
            "WIFI_FALLBACK_START reason=primary_timeout",
            "WIFI_CONNECTED profile=",
        ]:
            self.assertIn(token, mqtt_manager)

    def test_mqtt_runtime_keeps_nonblocking_shape(self):
        mqtt_manager = read_text("src/mqtt_manager.cpp")
        runtime_functions = re.findall(
            r"void (?:tick|attemptMqttConnect|startWifiAttempt|tickWifiScan)\([^)]*\) \{(?P<body>.*?)\n\}",
            mqtt_manager,
            flags=re.DOTALL,
        )
        self.assertTrue(runtime_functions)
        for body in runtime_functions:
            self.assertNotIn("delay(", body)
            self.assertNotRegex(body, r"\bwhile\s*\(")

    def test_required_ack_reasons_are_present(self):
        mqtt_manager = read_text("src/mqtt_manager.cpp")
        command_handler = read_text("src/command_handler.cpp")
        required = [
            "success",
            "invalid_mode",
            "cooldown_active",
            "interlock_violation",
            "invalid_payload",
            "unknown_command",
            "duplicate_cmdId",
            "internal_error",
        ]
        combined = mqtt_manager + command_handler + read_text("src/mode_controller.cpp")
        for reason in required:
            self.assertIn(reason, combined)

    def test_unified_sensor_flags_feed_semantic_mqtt_payloads(self):
        sensor_types = read_text("include/sensor_types.h")
        mqtt_manager = read_text("src/mqtt_manager.cpp")
        for flag in ["dht_ok", "bh1750_ok", "soil_ok", "rain_ok"]:
            self.assertIn(flag, sensor_types)
            self.assertIn(flag, mqtt_manager)
        for field in [
            "airTemperatureC",
            "airHumidityPct",
            "lightLux",
            "soilMoisturePct",
            "health",
            "raw",
            "controlFlags",
            "controlThresholds",
            "tempOverFanOnThreshold",
            "tempBelowFanOffThreshold",
            "soilBelowPumpOnThreshold",
            "soilAbovePumpOffThreshold",
            "luxBelowLightOnThreshold",
            "luxAboveLightOffThreshold",
            "isRaining",
            "airHumidityTooHigh",
            "airHumidityTooLow",
            "fanOnTempC",
            "fanOffTempC",
            "pumpOnSoilMoisturePct",
            "pumpOffSoilMoisturePct",
            "lightOnLux",
            "lightOffLux",
            "airHumidityHighPct",
            "airHumidityLowPct",
            "updateTime",
        ]:
            self.assertIn(field, mqtt_manager)
        config = read_text("include/config.h")
        self.assertIn("AIR_HUMIDITY_HIGH_PCT", config)
        self.assertIn("AIR_HUMIDITY_LOW_PCT", config)
        self.assertIn("MAX_MQTT_PAYLOAD_BYTES = 1024", config)
        for old_field in ['"update_time"', '"sysStatus"', '"servoAngle"']:
            self.assertNotIn(old_field, mqtt_manager)

    def test_boot_roof_debug_tokens_are_present(self):
        combined = (
            read_text("src/app_controller.cpp")
            + read_text("src/auto_logic.cpp")
            + read_text("src/sensor_manager.cpp")
            + read_text("src/lcd_display.cpp")
        )
        for token in [
            "BOOT_PHASE phase=",
            "MODE_RESTORE valid=",
            "RAIN_VALID valid=",
            "RAIN_STATE state=",
            "ROOF_CMD_DECISION reason=",
            "ERROR_REASON reason=",
            "LCD_K_STATE_SOURCE source=actuator_snapshot",
        ]:
            self.assertIn(token, combined)

    def test_rain_debounce_starts_invalid_until_stable(self):
        sensor_manager = read_text("src/sensor_manager.cpp")
        self.assertIn("g_rainDebounceInitialized", sensor_manager)
        self.assertRegex(sensor_manager, r"return false;\s*\}\s*[\s\S]*rawRainDO != g_pendingRainDO")
        self.assertIn("data.rain_ok = false;", sensor_manager)
        self.assertIn('F("settling")', sensor_manager)
        self.assertIn('F("stable")', sensor_manager)

    def test_auto_logic_holds_roof_while_rain_invalid(self):
        auto_logic = read_text("src/auto_logic.cpp")
        policy_start = auto_logic.index("void applyRainPolicy")
        policy_end = auto_logic.index("void applyPumpPolicy")
        policy = auto_logic[policy_start:policy_end]
        invalid_branch = re.search(
            r"if \(!data\.rain_ok\) \{(?P<body>.*?)\n  \}",
            policy,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(invalid_branch)
        self.assertNotIn("AUTO_ROOF_SAFE_ANGLE", invalid_branch.group("body"))
        self.assertIn("rain_sensor_invalid_hold", invalid_branch.group("body"))
        self.assertIn("T06_BOOT_RAIN_GUARD", auto_logic)


if __name__ == "__main__":
    unittest.main()

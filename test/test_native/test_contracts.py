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
        self.assertNotIn("79797979", mqtt_manager)

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

    def test_unified_sensor_flags_are_present_in_runtime_and_payloads(self):
        sensor_types = read_text("include/sensor_types.h")
        mqtt_manager = read_text("src/mqtt_manager.cpp")
        for flag in ["dht_ok", "bh1750_ok", "soil_ok", "rain_ok"]:
            self.assertIn(flag, sensor_types)
            self.assertIn(flag, mqtt_manager)


if __name__ == "__main__":
    unittest.main()

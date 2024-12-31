# LILYGO T-Display S3 MQTT Thermostat
## Smart thermostat, built for LILYGO T-Display S3 with DHT22 sensor

This project is a fork of [M5stack-Core2-MQTT-Thermostat](https://github.com/user/M5stack-Core2-MQTT-Thermostat), modified to work with the LILYGO T-Display S3 and DHT22 temperature/humidity sensor.

## Key features:
 - All thermostat logic is built into this MicroPython script and runs on T-Display S3
 - Thermostat supports auto, manual, fan, heat, cool modes
 - Minimum cycle duration can be set (THERMO_MIN_CYCLE)
 - Swing mode is enabled and can be customized (THERMO_COLD_TOLERANCE and THERMO_HEAT_TOLERANCE)
 - Uses DHT22 sensor for accurate temperature and humidity readings

## Hardware Requirements:
 - LILYGO T-Display S3
 - DHT22 Temperature/Humidity Sensor
 - Relays for controlling HVAC equipment

## Wiring:
 - DHT22 data pin connected to GPIO 15 (configurable in code)
 - Display uses default T-Display S3 pins:
   - SCK: GPIO 18
   - MOSI: GPIO 23
   - MISO: GPIO 19
   - CS: GPIO 14
   - DC: GPIO 27
   - RST: GPIO 33
   - BL: GPIO 32

## Software Requirements:
 - MicroPython for ESP32-S3
 - Required libraries:
   - st7789 (display driver)
   - micropython-umqtt.simple
   - dht (for DHT22 sensor)

## Configuration:
 - Create a `config.py` file to store secrets:
   ```python
   WIFI_SSID = "your_wifi_ssid"
   WIFI_PASS = "your_wifi_password"
   MQTT_IP = "your_mqtt_broker_ip"
   MQTT_PORT = 1883
   MQTT_USER = "your_mqtt_username"
   MQTT_PASS = "your_mqtt_password"
   ```
 - Graphics files for heat/cool/fan need to be stored in the `/res` directory
 - MQTT topics for relay control can be configured in the code (variables starting with RELAY_)

## Home Assistant integration:
 - Integrates with Home Assistant through MQTT (MQTT integration required)
 - Supports MQTT auto-discovery. No configuration needed on the HA side
 - Creates 'T-Display S3 Thermostat' device with following entities:
    - 2 sensors for temperature and humidity (from DHT22)
    - 1 thermostat entity
    - 2 switch entities (for manual furnace/ac control)
 - The thermostat entity allows you to control target temperature and thermostat mode through HA
 - Manual mode is not supported by the HA thermostat entity. Device states are accurately reflected, but the thermostat mode will show as 'off' when in manual mode
 - Switch entities can be used to manually control devices from HA

## Usage notes:
 - Upon start the thermostat will be OFF. Tap the mode label to cycle through modes: OFF - AUTO - MAN - HEAT - COOL - FAN
 - When in manual mode, use the touch interface to control heat, AC, and fan. Only 1 device can be on at a time
 - When min cycle duration requirement isn't met, the display will blink until the change can be implemented
 - Temperature can be displayed in Celsius or Fahrenheit (set DISP_TEMPERATURE accordingly). Default is Fahrenheit
 - Home Assistant will display temperature according to your HA preferences (metric vs imperial)

## Installation:
1. Flash MicroPython to your T-Display S3
2. Install required libraries using mpremote or your preferred method:
   ```bash
   mpremote connect /dev/ttyUSB0 mip install st7789
   mpremote connect /dev/ttyUSB0 mip install micropython-umqtt.simple
   ```
3. Copy all files to the T-Display S3:
   - Thermostat.py
   - tft_config.py
   - config.py (create with your settings)
   - Graphics files in /res directory

## Contributing:
Contributions are welcome! Please feel free to submit a Pull Request.

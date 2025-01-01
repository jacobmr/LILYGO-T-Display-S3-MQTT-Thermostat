from machine import Pin, SPI, Timer, reset
import tft_config
import vga1_bold_16x32 as font
import dht
import network
from umqtt.simple import MQTTClient
import math
from numbers import Number
import json
import config
import time

# Device information
ATTR_MANUFACTURER = "LILYGO"
ATTR_MODEL = "T-Display S3"
ATTR_NAME = "T-Display S3 Thermostat"

# Pin definitions
DHT_PIN = 15  # Adjust this to match your DHT22 connection
BUTTON_1_PIN = 0   # Left button
BUTTON_2_PIN = 14  # Right button

# Button debounce time (ms)
DEBOUNCE_MS = 200

# Default topics used to communicate with Home Assistant
DEFAULT_DISC_PREFIX = "homeassistant/"
DEFAULT_TOPIC_THERMOSTAT_PREFIX = "t-display-s3/thermostat/"
DEFAULT_TOPIC_SENSOR_PREFIX = "t-display-s3/dht22/"
DEFAULT_TOPIC_DEBUG = "t-display-s3/debug/"
DEFAULT_TOPIC_SWITCH_PREFIX = "t-display-s3/switch/"

# Topics to send/receive commands to other sensors directly
MASTER_SWITCH_TOPIC = "test-master-switch/switch/master_switch/state"

# Details on how the information on the T-Display S3 display should be rendered
DISP_R1 = 90
DISP_R2 = 70
DISP_XCOORD = 160
DISP_YCOORD = 98
DISP_COLOR_COOL = 0x3366ff
DISP_COLOR_HEAT = 0xff6600
DISP_LBL_TARGET_OFFSET = -20
DISP_LBL_ACTION_OFFSET = -51
DISP_LBL_MODE_OFFSET = 55
DISP_TEMPERATURE = "F" # change to "C" if your prefer Celsius

# MQTT connection details
MQTT_IP = config.MQTT_IP
MQTT_PORT = config.MQTT_PORT
MQTT_ID = 'Thermostat'
MQTT_USER = config.MQTT_USER
MQTT_PASS = config.MQTT_PASS
MQTT_KEEPALIVE = 300

# JSON Keys used to configure the device with Home Assistant
KEY_AVAILABILITY_TOPIC = "avty_t"
KEY_COMMAND_TOPIC = "cmd_t"
KEY_DEVICE = "dev"
KEY_DEVICE_CLASS = "dev_cla"
KEY_IDENTIFIERS = "ids"
KEY_MANUFACTURER = "mf"
KEY_MODEL = "mdl"
KEY_NAME = "name"
KEY_PAYLOAD_AVAILABLE = "pl_avail"
KEY_PAYLOAD_NOT_AVAILABLE = "pl_not_avail"
KEY_STATE_TOPIC = "stat_t"
KEY_UNIQUE_ID = "uniq_id"
KEY_VALUE_TEMPLATE = "val_tpl"
KEY_ICON = "ic"

KEY_ACTION_TOPIC = "act_t"
KEY_CURRENT_TEMPERATURE_TOPIC = "curr_temp_t"
KEY_CURRENT_TEMPERATURE_TEMPLATE = "curr_temp_tpl"
KEY_INITIAL = "init"
KEY_MAX_TEMP = "max_temp"
KEY_MIN_TEMP = "min_temp"
KEY_MODE_COMMAND_TOPIC = "mode_cmd_t"
KEY_MODE_STATE_TOPIC = "mode_stat_t"
KEY_SEND_IF_OFF = "send_if_off"
KEY_TEMPERATURE_COMMAND_TOPIC = "temp_cmd_t"
KEY_TEMPERATURE_STATE_TOPIC = "temp_stat_t"
KEY_TEMPERATURE_UNIT = "temp_unit"
KEY_MODE_STATE_TEMPLATE = "mode_stat_tpl"
KEY_PAYLOAD_OFF = "pl_off"
KEY_PAYLOAD_ON = "pl_on"
KEY_UNIT_OF_MEASUREMENT = "unit_of_meas"

# Topic and Payload details used to communicate with the furnace, AC, and fan(s)
RELAY_HEAT_TOPIC = "t-display-s3/heat"
RELAY_COOL_TOPIC = "t-display-s3/cool"
RELAY_FAN_TOPIC = "t-display-s3/fan"
RELAY_HEAT_PAYLOAD_ON = "ON"
RELAY_HEAT_PAYLOAD_OFF = "OFF"
RELAY_COOL_PAYLOAD_ON = "ON"
RELAY_COOL_PAYLOAD_OFF = "OFF"
RELAY_FAN_PAYLOAD_ON = "ON"
RELAY_FAN_PAYLOAD_OFF = "OFF"

# Construction of topics for communication with Home Assistant
TOPIC_ANNOUNCE = "announce"
TOPIC_STATUS = "status"
TOPIC_STATE = "state"
TOPIC_MODE_STATE = "mode/state"
TOPIC_ACTION = "action"
TOPIC_MODE_COMMAND = "mode/command"
TOPIC_TEMPERATURE_COMMAND = "temperature/command"
TOPIC_HEATER_COMMAND = "heater/command"
TOPIC_AC_COMMAND = "ac/command"
TOPIC_HEATER_STATUS = "heater/status"
TOPIC_AC_STATUS = "ac/status"
TOPIC_DISCOVERY = "discovery"

# Instructions on how the payload is structured and should be parsed by Home Assistant
TPL_TEMPERATURE = "{{value_json.temperature}}"
TPL_PRESSURE = "{{value_json.pressure}}"
TPL_HUMIDITY = "{{value_json.humidity}}"
TPL_MODE_STATE = '{% set values = {"off":"off", "auto":"auto", "man":"off", "heat":"heat", "cool":"cool", "fan":"fan_only"} %} {{ values[value] }}'
    

WIFI_SSID = config.WIFI_SSID
WIFI_PASS = config.WIFI_PASS

THERMO_MIN_TEMP = 0            # C
THERMO_MAX_TEMP = 40           # C
THERMO_MIN_TARGET = 15         # C
THERMO_MAX_TARGET = 25         # C
THERMO_MIN_CYCLE = 5           # seconds
THERMO_COLD_TOLERANCE = 0.5      # C
THERMO_HEAT_TOLERANCE = 0.5      # C
THERMO_UPDATE_FREQUENCY = 20   # seconds
THERMO_MODES = ["off", "auto", "man", "heat", "cool", "fan"]

# Constants for connection retry
WIFI_RETRY_DELAY = 5000  # ms
MQTT_RETRY_DELAY = 5000  # ms
MAX_FAILURES = 3  # Reset device after this many consecutive failures

# Global connection state
wifi_failures = 0
mqtt_failures = 0
last_successful_reading = None

# Initialize display
tft = tft_config.tft
tft.init(tft_config.MHS35, width=240, height=240, speed=40000000, rst_pin=33, backl_pin=32, miso=19, mosi=23, clk=18, cs=14, dc=27, bgr=True, backl_on=1, invrot=3)
tft.set_brightness(10)

# Initialize buttons
button1 = Pin(BUTTON_1_PIN, Pin.IN, Pin.PULL_UP)
button2 = Pin(BUTTON_2_PIN, Pin.IN, Pin.PULL_UP)
last_button1_time = 0
last_button2_time = 0

def button1_handler(pin):
    global last_button1_time, thermo_state
    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_button1_time) > DEBOUNCE_MS:
        # Cycle through modes
        current_mode = THERMO_MODES.index(thermo_state)
        next_mode = (current_mode + 1) % len(THERMO_MODES)
        thermo_state = THERMO_MODES[next_mode]
        update_display()
        last_button1_time = current_time

def button2_handler(pin):
    global last_button2_time, target_temp
    current_time = time.ticks_ms()
    if time.ticks_diff(current_time, last_button2_time) > DEBOUNCE_MS:
        # Adjust temperature (press to increase, long press to decrease)
        if pin.value() == 0:  # Button pressed
            if current_time - last_button2_time > 1000:  # Long press
                target_temp = max(THERMO_MIN_TARGET, target_temp - 0.5)
            else:
                target_temp = min(THERMO_MAX_TARGET, target_temp + 0.5)
            update_display()
        last_button2_time = current_time

# Set up button interrupts
button1.irq(trigger=Pin.IRQ_FALLING, handler=button1_handler)
button2.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=button2_handler)

# Setup initial comms and register with Home Assistant (through auto-discovery)
def comms_init():
    global m5mqtt
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)
    while not wlan.isconnected():
        pass
    m5mqtt = MQTTClient(MQTT_ID, MQTT_IP, MQTT_PORT, MQTT_USER, MQTT_PASS, MQTT_KEEPALIVE)
    
    # Subscribe to HA thermostat mode changes
    m5mqtt.subscribe(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_MODE_COMMAND, rcv_thermo_state)
    
    # Subscribe to HA target temperature changes
    m5mqtt.subscribe(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_TEMPERATURE_COMMAND, rcv_target_temp)

    # Subscribe to HA manual heater changes
    m5mqtt.subscribe(DEFAULT_TOPIC_SWITCH_PREFIX + TOPIC_HEATER_COMMAND, rcv_heater_status)    
    
    # Subscribe to HA manual AC changes
    m5mqtt.subscribe(DEFAULT_TOPIC_SWITCH_PREFIX + TOPIC_AC_COMMAND, rcv_ac_status)     

    # Subscribe to Master OFF switch commands
    m5mqtt.subscribe(MASTER_SWITCH_TOPIC, rcv_master_off)         

    # Subscribe to Home Assistant registration requests
    m5mqtt.subscribe(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_DISCOVERY, rcv_discovery)
    
    m5mqtt.start()
    mqtt_registration()
    mqtt_initialization()

def mqtt_registration():
    # Register DHT22 Temperature sensor with Home Assistant
    topic = "%ssensor/t-display-s3/t-display-s3-temp/config" % DEFAULT_DISC_PREFIX
    payload = {        
        KEY_NAME: "T-Display S3 Temperature",
        KEY_PAYLOAD_AVAILABLE: "on",
        KEY_PAYLOAD_NOT_AVAILABLE: "off",
        KEY_DEVICE_CLASS: "temperature",
        KEY_UNIQUE_ID: "12234",
        KEY_DEVICE: {
            KEY_IDENTIFIERS: ["12234"],
            KEY_NAME: ATTR_NAME,
            KEY_MODEL: ATTR_MODEL,
            KEY_MANUFACTURER: ATTR_MANUFACTURER
        },
        KEY_UNIT_OF_MEASUREMENT: chr(186) + "C",
        KEY_STATE_TOPIC: "~" + TOPIC_STATE,
        "~": DEFAULT_TOPIC_SENSOR_PREFIX,
        KEY_AVAILABILITY_TOPIC: "~" + TOPIC_STATUS,
        KEY_VALUE_TEMPLATE: TPL_TEMPERATURE
    }
    m5mqtt.publish(topic, json.dumps(payload).encode('utf-8'))

    # Register T-Display S3 as HVAC device with Home Assistant
    topic = "%sclimate/t-display-s3/config" % DEFAULT_DISC_PREFIX
    payload = {
        KEY_NAME: "T-Display S3 Thermostat",
        KEY_PAYLOAD_AVAILABLE: "on",
        KEY_PAYLOAD_NOT_AVAILABLE: "off",
        KEY_UNIQUE_ID: "122348",
        KEY_DEVICE: {
            KEY_IDENTIFIERS: ["12234"],
            KEY_NAME: ATTR_NAME,
            KEY_MODEL: ATTR_MODEL,
            KEY_MANUFACTURER: ATTR_MANUFACTURER
        },
        "~": DEFAULT_TOPIC_THERMOSTAT_PREFIX,
        KEY_ACTION_TOPIC: "~" + TOPIC_ACTION,
        KEY_AVAILABILITY_TOPIC: "~" + TOPIC_STATUS,
        KEY_CURRENT_TEMPERATURE_TOPIC: DEFAULT_TOPIC_SENSOR_PREFIX + TOPIC_STATE,
        KEY_CURRENT_TEMPERATURE_TEMPLATE: TPL_TEMPERATURE,
        KEY_INITIAL: 20,
        KEY_MAX_TEMP: THERMO_MAX_TARGET,
        KEY_MIN_TEMP: THERMO_MIN_TARGET,
        KEY_MODE_COMMAND_TOPIC: "~" + TOPIC_MODE_COMMAND,
        KEY_MODE_STATE_TOPIC: "~" + TOPIC_MODE_STATE,
        KEY_SEND_IF_OFF: True,
        KEY_TEMPERATURE_COMMAND_TOPIC: "~" + TOPIC_TEMPERATURE_COMMAND,
        KEY_TEMPERATURE_STATE_TOPIC: "~" + TOPIC_STATE,
        KEY_TEMPERATURE_UNIT: "C",
        KEY_MODE_STATE_TEMPLATE: TPL_MODE_STATE
        }
    m5mqtt.publish(topic, str(json.dumps(payload)))

    # Register Heater for manual control with Home Assistant
    topic = "%sswitch/t-display-s3/t-display-s3-heater/config" % DEFAULT_DISC_PREFIX
    payload = {
        KEY_NAME: "T-Display S3 Heater",
        KEY_PAYLOAD_AVAILABLE: "on",
        KEY_PAYLOAD_NOT_AVAILABLE: "off",
        KEY_UNIQUE_ID: "122349",
        KEY_DEVICE: {
            KEY_IDENTIFIERS: ["12234"],
            KEY_NAME: ATTR_NAME,
            KEY_MODEL: ATTR_MODEL,
            KEY_MANUFACTURER: ATTR_MANUFACTURER
        },
        "~": DEFAULT_TOPIC_SWITCH_PREFIX,
        KEY_PAYLOAD_OFF: RELAY_HEAT_PAYLOAD_OFF,
        KEY_PAYLOAD_ON: RELAY_HEAT_PAYLOAD_ON,
        KEY_AVAILABILITY_TOPIC: "~" + TOPIC_HEATER_STATUS,
        KEY_COMMAND_TOPIC: "~" + TOPIC_HEATER_COMMAND,
        KEY_STATE_TOPIC: RELAY_HEAT_TOPIC,
        KEY_ICON: "mdi:radiator"
        }
    m5mqtt.publish(topic, str(json.dumps(payload)))    
        
    # Register AC for manual control with Home Assistant
    topic = "%sswitch/t-display-s3/t-display-s3-ac/config" % DEFAULT_DISC_PREFIX
    payload = {
        KEY_NAME: "T-Display S3 AC",
        KEY_PAYLOAD_AVAILABLE: "on",
        KEY_PAYLOAD_NOT_AVAILABLE: "off",
        KEY_UNIQUE_ID: "122350",
        KEY_DEVICE: {
            KEY_IDENTIFIERS: ["12234"],
            KEY_NAME: ATTR_NAME,
            KEY_MODEL: ATTR_MODEL,
            KEY_MANUFACTURER: ATTR_MANUFACTURER
        },
        "~": DEFAULT_TOPIC_SWITCH_PREFIX,
        KEY_PAYLOAD_OFF: RELAY_COOL_PAYLOAD_OFF,
        KEY_PAYLOAD_ON: RELAY_COOL_PAYLOAD_ON,
        KEY_AVAILABILITY_TOPIC: "~" + TOPIC_AC_STATUS,
        KEY_COMMAND_TOPIC: "~" + TOPIC_AC_COMMAND,
        KEY_STATE_TOPIC: RELAY_COOL_TOPIC,
        KEY_ICON: "mdi:snowflake"
        }
    m5mqtt.publish(topic, str(json.dumps(payload)))

def mqtt_initialization():
    # Send Availability notices to Home Assistant
    m5mqtt.publish(DEFAULT_TOPIC_SENSOR_PREFIX + TOPIC_STATUS, "on")
    m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_STATUS, "on")
    m5mqtt.publish(DEFAULT_TOPIC_SWITCH_PREFIX + TOPIC_HEATER_STATUS, "on")
    m5mqtt.publish(DEFAULT_TOPIC_SWITCH_PREFIX + TOPIC_AC_STATUS, "on")
    
    # Send initial state information to Home Assistant
    update_mqtt_state_topics()
    m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_ACTION, "idle") 
    m5mqtt.publish(RELAY_HEAT_TOPIC, RELAY_HEAT_PAYLOAD_OFF)
    m5mqtt.publish(RELAY_COOL_TOPIC, RELAY_COOL_PAYLOAD_OFF)
    
def thermostat_init():
    global action, actual_temp, blink, change_ignored, cycle, delay, ticks, thermo_state, fan_state, cooling_state, heating_state, target_temp, manual_command
    action = 0
    actual_temp = dht22.temperature()
    blink = 0
    change_ignored = 0
    cycle = 0
    delay = 0      # initial delay of 10s so that user can select the right mode without appliances suddenly turning on
    thermo_state = THERMO_MODES[0]
    target_temp = 20  # Default target temperature
    ticks = 0
    
    # initial state of thermostat is OFF and all appliances are OFF
    fan_state = 0
    cooling_state = 0
    heating_state = 0
    manual_command = 0

# update display everytime there is a change (due to incoming HA info, screen interaction, or sensor data changes)
def update_display():
    tft.clear()
    tft.font(font)
    tft.print(thermo_state, DISP_XCOORD, DISP_YCOORD, 0xffffff)
    target_temp_display = round(target_temp if DISP_TEMPERATURE == "C" else target_temp * 9 / 5 + 32)
    actual_temp_display = actual_temp if DISP_TEMPERATURE == "C" else actual_temp * 9 / 5 + 32
    
    # If mode is manual
    if thermo_state == THERMO_MODES[2]:    
        if heating_state ==1:
            tft.print('HEATING', DISP_XCOORD, DISP_YCOORD + 20, DISP_COLOR_HEAT)
        elif cooling_state ==1:
            tft.print('COOLING', DISP_XCOORD, DISP_YCOORD + 20, DISP_COLOR_COOL)
        elif fan_state ==1:
            tft.print('FAN', DISP_XCOORD, DISP_YCOORD + 20, 0xffffff)
        else:
            tft.print('IDLE', DISP_XCOORD, DISP_YCOORD + 20, 0xffffff)
    else:
        tft.print(str(target_temp_display), DISP_XCOORD, DISP_YCOORD + 20, 0xffffff)
        
    # Draw the temperature arc
    t = THERMO_MIN_TEMP
    while t <= THERMO_MAX_TEMP:
        angle = (t * 8 + 200) % 360
        if t > min(round(actual_temp),target_temp) and t < max(round(actual_temp), target_temp) and thermo_state in [THERMO_MODES[index] for index in [1,3,4]]:
            tft.line(
                int(DISP_R1 * math.sin(angle / 180 * math.pi) + DISP_XCOORD),
                int(DISP_YCOORD - DISP_R1 * math.cos(angle / 180 * math.pi)),
                int(DISP_R2 * math.sin(angle / 180 * math.pi) + DISP_XCOORD),
                int(DISP_YCOORD - DISP_R2 * math.cos(angle / 180 * math.pi)),
                0xffffff)
        elif t == round(actual_temp):
            tft.line(
                int(DISP_R1 * math.sin(angle / 180 * math.pi) + DISP_XCOORD),
                int(DISP_YCOORD - DISP_R1 * math.cos(angle / 180 * math.pi)),
                int((DISP_R2 - 10) * math.sin(angle / 180 * math.pi) + DISP_XCOORD),
                int(DISP_YCOORD - (DISP_R2 - 10) * math.cos(angle / 180 * math.pi)),
                0xffffff)
        else:
            tft.line(
                int(DISP_R1 * math.sin(angle / 180 * math.pi) + DISP_XCOORD),
                int(DISP_YCOORD - DISP_R1 * math.cos(angle / 180 * math.pi)),
                int(DISP_R2 * math.sin(angle / 180 * math.pi) + DISP_XCOORD),
                int(DISP_YCOORD - DISP_R2 * math.cos(angle / 180 * math.pi)),
                0x333333)
        t += 0.5    
    if round(actual_temp) >= target_temp or thermo_state in [THERMO_MODES[index] for index in [0,2,5]]:
        tft.print(
            round(actual_temp_display),
            int(DISP_R1 * math.sin(((actual_temp * 8 + 200) % 360 +4) / 180 * math.pi) + DISP_XCOORD),
            int(DISP_YCOORD - DISP_R1 * math.cos(((actual_temp * 8 +200) % 360 +4) / 180 * math.pi)),
            0xffffff)
    else:
        tft.print(
            round(actual_temp_display),
            int(DISP_R1 * math.sin(((actual_temp * 8 + 200) % 360 -20) / 180 * math.pi) + DISP_XCOORD),
            int(DISP_YCOORD - DISP_R1 * math.cos(((actual_temp * 8 +200) % 360 -20) / 180 * math.pi)),
            0xffffff)

# This is where the key decisions happen
# -- First we handle the 'manual' use case.
# -- Then we handle all the use cases wher the thermostat is on (auto/heat/cool/fan)
# -- if thermostat mode is 'off', all devices are turned off.

def thermostat_decision_logic():
    global actual_temp, target_temp, manual_command
    actual_temp = dht22.temperature()
    target_temp = 20  # Default target temperature
    
    if thermo_state == THERMO_MODES[2]: 
        if manual_command == "heating on" and heating_state == 0:
            change_to("heating on")
        elif manual_command == "heating off" and heating_state == 1:
            change_to("heating off")
        elif manual_command == "cooling on" and cooling_state == 0:
            change_to("cooling on")
        elif manual_command == "cooling off" and cooling_state == 1:
            change_to("cooling off")
        elif manual_command == "fan on" and fan_state == 0:
            change_to("fan on")
        elif manual_command == "fan off" and fan_state == 1:
            change_to("fan off")
        else:
            update_display()
        return
    
    if (
        actual_temp <= target_temp - THERMO_COLD_TOLERANCE and
        heating_state == 0 and
        thermo_state in [THERMO_MODES[index] for index in [1,3]]):
        # thermostat needs to turn on heating
        change_to("heating on")
    elif (
        actual_temp >= target_temp + THERMO_HEAT_TOLERANCE and
        cooling_state == 0 and
        thermo_state in [THERMO_MODES[index] for index in [1,4]]):
        # thermostat needs to turn on cooling
        change_to("cooling on")
    elif (
        actual_temp >= target_temp + THERMO_HEAT_TOLERANCE and
        fan_state == 0 and
        thermo_state in [THERMO_MODES[index] for index in [5]]):
        # thermostat needs to turn on fan cooling
        change_to("fan on")

    elif (
        (actual_temp >= target_temp or
         thermo_state in [THERMO_MODES[index] for index in [0,4,5]]) and        
         heating_state == 1):
        # thermostat needs to turn off heating
        change_to("heating off")
    elif (
        (actual_temp <= target_temp or
         thermo_state in [THERMO_MODES[index] for index in [0,3,5]]) and
         cooling_state == 1):
        # thermostat needs to turn off cooling
        change_to ("cooling off")
    elif (
        (actual_temp <= target_temp or
         thermo_state in [THERMO_MODES[index] for index in [0,3,4]]) and
         fan_state == 1):
        # thermostat needs to turn off fan
        change_to ("fan off")
    else:
        # no action
        update_display()

# Here's where the appliances are turned on/off using MQTT messages.
# Here's where we also check for minimum cycle time and ignore change requests
#  if the minimum cycle time hasn't been reached.
def change_to (action):
    global change_ignored, heating_state, cooling_state, fan_state, m5mqtt, delay
    if delay == 0 or THERMO_MIN_CYCLE == 0:
        change_ignored = 0
        if action == "heating on":
            m5mqtt.publish(RELAY_HEAT_TOPIC, RELAY_HEAT_PAYLOAD_ON)
            m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_ACTION, "heating") 
            heating_state = 1
            if cooling_state == 1:
                m5mqtt.publish(RELAY_COOL_TOPIC, RELAY_COOL_PAYLOAD_OFF)
                cooling_state = 0
            if fan_state == 1:
                m5mqtt.publish(RELAY_FAN_TOPIC, RELAY_FAN_PAYLOAD_OFF)
                fan_state = 0
        elif action == "cooling on":
            m5mqtt.publish(RELAY_COOL_TOPIC, RELAY_COOL_PAYLOAD_ON)
            m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_ACTION, "cooling") 
            cooling_state = 1
            if heating_state == 1:
                m5mqtt.publish(RELAY_HEAT_TOPIC, RELAY_HEAT_PAYLOAD_OFF)
                heating_state = 0
            if fan_state == 1:
                m5mqtt.publish(RELAY_FAN_TOPIC, RELAY_FAN_PAYLOAD_OFF)
                fan_state = 0
        elif action == "fan on":
            m5mqtt.publish(RELAY_FAN_TOPIC, RELAY_FAN_PAYLOAD_ON)
            m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_ACTION, "fan") 
            fan_state = 1
            if heating_state == 1:
                m5mqtt.publish(RELAY_HEAT_TOPIC, RELAY_HEAT_PAYLOAD_OFF)
                heating_state = 0
            if cooling_state == 1:
                m5mqtt.publish(RELAY_COOL_TOPIC, RELAY_COOL_PAYLOAD_OFF)
                cooling_state = 0           
        elif action == "heating off":
            m5mqtt.publish(RELAY_HEAT_TOPIC, RELAY_HEAT_PAYLOAD_OFF)
            m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_ACTION, "idle") 
            heating_state = 0
        elif action == "cooling off":
            m5mqtt.publish(RELAY_COOL_TOPIC, RELAY_COOL_PAYLOAD_OFF)
            m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_ACTION, "idle") 
            cooling_state = 0
        elif action == "fan off":
            m5mqtt.publish(RELAY_FAN_TOPIC, RELAY_FAN_PAYLOAD_OFF)
            m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_ACTION, "idle") 
            fan_state = 0           
        delay = THERMO_MIN_CYCLE
        update_display()
    else:
        change_ignored = 1
        update_display()


def update_mqtt_state_topics():
    if not check_wifi() or not check_mqtt():
        return
        
    #update state of DHT22 sensors
    temp, humidity = read_dht()
    payload = {
        "temperature": temp,
        "humidity": humidity
    }
    try:
        m5mqtt.publish(DEFAULT_TOPIC_SENSOR_PREFIX + TOPIC_STATE, str(json.dumps(payload)))
    except Exception as e:
        print("MQTT publish error:", e)
    
    #update state of thermostat target temperature
    m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_STATE, str(target_temp))
    
    #update state of thermostat mode
    m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_MODE_STATE, str(thermo_state))    
        
def rcv_target_temp (topic_data):
    global target_temp
    target_temp = round(float(topic_data))
    thermostat_decision_logic()

def rcv_thermo_state (topic_data):
    global thermo_state
    thermo_state = str(topic_data) if str(topic_data) != "fan_only" else "fan"
    m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_MODE_STATE, str(thermo_state)) 
    thermostat_decision_logic()

def rcv_heater_status (topic_data):
    global thermo_state, manual_command
    thermo_state = THERMO_MODES[2]
    manual_command = "heating on" if str(topic_data) == RELAY_HEAT_PAYLOAD_ON else "heating off"
    thermostat_decision_logic()

def rcv_ac_status (topic_data):
    global thermo_state, manual_command
    thermo_state = THERMO_MODES[2]
    manual_command = "cooling on" if str(topic_data) == RELAY_COOL_PAYLOAD_ON else "cooling off"
    thermostat_decision_logic()
    
def rcv_master_off (topic_data):
    global thermo_state
    thermo_state = THERMO_MODES[0]
    thermostat_decision_logic()

def rcv_discovery (topic_data):
    mqtt_registration()
    m5mqtt.publish(DEFAULT_TOPIC_SENSOR_PREFIX + TOPIC_STATUS, "on")
    m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_STATUS, "on")
    m5mqtt.publish(DEFAULT_TOPIC_SWITCH_PREFIX + TOPIC_HEATER_STATUS, "on")
    m5mqtt.publish(DEFAULT_TOPIC_SWITCH_PREFIX + TOPIC_AC_STATUS, "on")
    thermostat_decision_logic()
    
def check_wifi():
    global wifi_failures
    if not wlan.isconnected():
        print("WiFi disconnected. Attempting to reconnect...")
        try:
            wlan.connect(config.WIFI_SSID, config.WIFI_PASS)
            time.sleep_ms(WIFI_RETRY_DELAY)
            if wlan.isconnected():
                print("WiFi reconnected")
                wifi_failures = 0
                return True
            wifi_failures += 1
            if wifi_failures >= MAX_FAILURES:
                print("Too many WiFi failures. Resetting device...")
                reset()
            return False
        except Exception as e:
            print("WiFi connection error:", e)
            return False
    return True

def check_mqtt():
    global mqtt_failures
    try:
        m5mqtt.ping()
        mqtt_failures = 0
        return True
    except:
        print("MQTT disconnected. Attempting to reconnect...")
        try:
            m5mqtt.connect()
            time.sleep_ms(MQTT_RETRY_DELAY)
            mqtt_failures = 0
            return True
        except Exception as e:
            print("MQTT connection error:", e)
            mqtt_failures += 1
            if mqtt_failures >= MAX_FAILURES:
                print("Too many MQTT failures. Resetting device...")
                reset()
            return False

def read_dht():
    global last_successful_reading
    try:
        dht22.measure()
        temp = dht22.temperature()
        hum = dht22.humidity()
        last_successful_reading = (temp, hum)
        return temp, hum
    except OSError as e:
        print("DHT22 read error:", e)
        if last_successful_reading:
            print("Using last successful reading")
            return last_successful_reading
        return (20, 50)  # Default fallback values
    except Exception as e:
        print("Unexpected DHT22 error:", e)
        if last_successful_reading:
            return last_successful_reading
        return (20, 50)  # Default fallback values

thermostat_init()
comms_init()
thermostat_decision_logic()

dht22 = dht.DHT22(Pin(DHT_PIN))

while True:
    if check_wifi() and check_mqtt():
        thermostat_decision_logic()
        update_mqtt_state_topics()
    time.sleep_ms(2000)

# Smart thermostat, built for LILYGO T-Display S3.
#
# Key features:
# - all thermostat logic is built into this Python script and runs on T-Display S3
# - thermostat supports auto, manual, fan, heat, cool modes
# - minimum cycle duration can be set (THERMO_MIN_CYCLE)
# - swing mode is enabled and can be customized (THERMO_COLD_TOLERANCE and THERMO_HEAT_TOLERANCE)
#
# Configuration considerations:
# - Relies on separate config.py file to store secrets (WiFI and MQTT connection details)
# - Gets actual temperature through DHT22 sensor, connected to the T-Display S3.
# - Uses MQTT to communicate with relays that turn on/off furnace, fan, and AC.
# - Graphics files for heat/cool/fan need to be stored in the /res directory
#
# Home Assistant integration:
# - Integrates with Home Assistant through MQTT (you need MQTT enabled on the HA side)
# - Supports MQTT auto-discovery. No configuration needed on the HA side.
# - Will create 'T-Display S3 Thermostat' device with following entities:
#    - 1 sensor for temperature (if using the DHT22)
#    - 1 thermostat entity
#    - 2 switch entities for manually turning on/off heater/ac (fan is not implemented yet)
# - The thermostat entity allows you to control target temperature and thermostat mode through HA. Any changes will be reflected on the T-Display S3.
# - When you manually switch a device on/off through the HA interface, the thermostat entity will be switched to 'off'
#   while the T-Display S3 will automatically switch to manual mode (HA doesn't support a 'manual mode' for the thermostat)
# - The switch entities in HA are subscribed to the same MQTT topics you've configured to communicate with your appliances. However, when HA issues
#   a switch change, the command will go only to the T-Display S3, who will process it, switch the termostat to manual mode, and then executes a state change.
#
# Usage notes:
# - Upon start the thermostat will be OFF. Tapping the OFF label will run the thermostat through the various modes: OFF - AUTO - MAN - HEAT - COOL - FAN
# - When in manual mode, use the A/B/C buttons to turn on/off heat pump, AC, Fan. Only 1 device can be on at a given time.
# - When min cycle duration requirement isn't met, the T-Display S3 display will blink until it is able to implement the change
# - Blinking is not supported on the Lovelace thermostat card. The HA dashboard will not change until the min cycle duration requirement is met.
# - T-Display S3 can display temperature in Celsius or Fahrenheit (set DISP_TEMPERATURE accordingly). Default is Fahrenheit.
#   Home Assistant will display temperature depending on your HA preferences (metric vs imperial) 

from machine import Pin, SPI
import tft_config
import vga1_bold_16x32 as font
import dht
import network
from umqtt.simple import MQTTClient
import math
from numbers import Number
import json
import config

# Device information
ATTR_MANUFACTURER = "LILYGO"
ATTR_MODEL = "T-Display S3"
ATTR_NAME = "T-Display S3 Thermostat"

# Pin definitions
DHT_PIN = 15  # Adjust this to match your DHT22 connection

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

# Initialize display
tft = tft_config.tft
tft.init(tft_config.MHS35, width=240, height=240, speed=40000000, rst_pin=33, backl_pin=32, miso=19, mosi=23, clk=18, cs=14, dc=27, bgr=True, backl_on=1, invrot=3)
tft.set_brightness(10)

# Using lvgl library to implement an invisible touch area (behind the mode label) to switch thermostat mode
lv.init()
scr = lv.obj() 
scr.set_style_local_bg_color(scr.PART.MAIN, lv.STATE.DEFAULT, lv.color_hex(0x000000))

path_ease_out = lv.anim_path_t()
path_ease_out.init()

# create button
btn = lv.btn(scr) 
btn.set_size(40, 20)
btn.align(None, lv.ALIGN.CENTER, 0, 55)
# set button style to make it invisible and implement a cool touch feedback effect
btn.set_style_local_bg_color(scr.PART.MAIN,lv.STATE.DEFAULT, lv.color_hex(0x0000ff))
styleButton = lv.style_t() # create style
styleButton.set_bg_color(lv.STATE.DEFAULT, lv.color_hex(0x0000ff))
styleButton.set_transition_time(lv.STATE.PRESSED, 300)
styleButton.set_transition_time(lv.STATE.DEFAULT, 0)
styleButton.set_transition_delay(lv.STATE.DEFAULT, 300)
styleButton.set_bg_opa(lv.STATE.DEFAULT, 0)
styleButton.set_bg_opa(lv.STATE.PRESSED, lv.OPA._80)
styleButton.set_border_width(lv.STATE.DEFAULT, 0)
styleButton.set_outline_width(lv.STATE.DEFAULT, 0)
styleButton.set_transform_width(lv.STATE.DEFAULT, -20)
styleButton.set_transform_height(lv.STATE.DEFAULT, -20)
styleButton.set_transform_width(lv.STATE.PRESSED, 0)
styleButton.set_transform_height(lv.STATE.PRESSED, 0)
styleButton.set_transition_path(lv.STATE.DEFAULT, path_ease_out)
styleButton.set_transition_prop_1(lv.STATE.DEFAULT, lv.STYLE.BG_OPA)
styleButton.set_transition_prop_2(lv.STATE.DEFAULT, lv.STYLE.TRANSFORM_WIDTH)
styleButton.set_transition_prop_3(lv.STATE.DEFAULT, lv.STYLE.TRANSFORM_HEIGHT)
btn.add_style(btn.PART.MAIN,styleButton) #define this style

# button press callback action (ie. change thermostat mode)
def change_mode(btn, event):
    global src, thermo_state
    if(event == lv.EVENT.CLICKED):
        btn.set_style_local_bg_color(btn.PART.MAIN, lv.STATE.DEFAULT, lv.color_hex(0xffccf9))
        thermo_state = THERMO_MODES[(THERMO_MODES.index(thermo_state) + 1) % 6]
        m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_MODE_STATE, str(thermo_state)) 
        thermostat_decision_logic()
    
# define callback  
btn.set_event_cb(change_mode)

# load the screen
lv.scr_load(scr)

dht22 = dht.DHT22(Pin(DHT_PIN))

img_BtnA = lv.img(scr)
img_BtnA.set_src("res/heat.png")
img_BtnA.align(None, lv.ALIGN.CENTER, 0, 55)

img_BtnB = lv.img(scr)
img_BtnB.set_src("res/cool.png")
img_BtnB.align(None, lv.ALIGN.CENTER, 0, 55)

img_BtnC = lv.img(scr)
img_BtnC.set_src("res/fan.png")
img_BtnC.align(None, lv.ALIGN.CENTER, 0, 55)

slider_target = lv.slider(scr)
slider_target.set_range(THERMO_MIN_TARGET, THERMO_MAX_TARGET)
slider_target.set_value(20)
slider_target.align(None, lv.ALIGN.CENTER, 0, 55)

lbl_target = lv.label(scr)
lbl_target.set_text("")
lbl_target.set_align(ALIGN_CENTER, 0, DISP_LBL_TARGET_OFFSET)

lbl_action = lv.label(scr)
lbl_action.set_text("")
lbl_action.set_align(ALIGN_CENTER, 0, DISP_LBL_ACTION_OFFSET)

lbl_mode = lv.label(scr)
lbl_mode.set_text("")
lbl_mode.set_align(ALIGN_CENTER, 0, DISP_LBL_MODE_OFFSET)

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
    slider_target.set_value(20)
    thermo_state = THERMO_MODES[0]
    target_temp = slider_target.get_value()
    ticks = 0
    
    # initial state of thermostat is OFF and all appliances are OFF
    fan_state = 0
    cooling_state = 0
    heating_state = 0
    manual_command = 0

# update display everytime there is a change (due to incoming HA info, screen interaction, or sensor data changes)
def update_display():
    tft.clear()
    lbl_mode.set_text(thermo_state)
    lbl_mode.set_align(ALIGN_CENTER, 0, DISP_LBL_MODE_OFFSET)
    target_temp_display = round(target_temp if DISP_TEMPERATURE == "C" else target_temp * 9 / 5 + 32)
    actual_temp_display = actual_temp if DISP_TEMPERATURE == "C" else actual_temp * 9 / 5 + 32
    
    # If mode is manual
    if thermo_state == THERMO_MODES[2]:    
        img_BtnA.set_hidden(False)
        img_BtnB.set_hidden(False)
        img_BtnC.set_hidden(False)
        slider_target.set_hidden(True)
        lbl_target.set_text("")
        lbl_target.set_text_color(0xffffff)
        lbl_target.set_align(ALIGN_CENTER, 0, DISP_LBL_TARGET_OFFSET)
        if heating_state ==1:
            lbl_action.set_text('HEATING')
            lbl_action.set_text_color(DISP_COLOR_HEAT)
            lbl_action.set_align(ALIGN_CENTER, 0, DISP_LBL_TARGET_OFFSET)
            lbl_action.set_text_color(DISP_COLOR_HEAT)
            if change_ignored == 1:
                timerSch.run("blink_now", 305, 0x00)
            else:
                timerSch.stop("blink_now")
                lbl_target.set_text_color(DISP_COLOR_HEAT)
        elif cooling_state ==1:
            lbl_action.set_text('COOLING')
            lbl_action.set_text_color(DISP_COLOR_COOL)
            lbl_action.set_align(ALIGN_CENTER, 0, DISP_LBL_TARGET_OFFSET)
            lbl_target.set_text_color(DISP_COLOR_COOL)
            if change_ignored == 1:
                timerSch.run("blink_now", 305, 0x00)
            else:
                timerSch.stop("blink_now")
                lbl_target.set_text_color(DISP_COLOR_COOL)    
        elif fan_state ==1:
            lbl_action.set_text('FAN')
            lbl_action.set_text_color(0xffffff)
            lbl_action.set_align(ALIGN_CENTER, 0, DISP_LBL_TARGET_OFFSET)
            lbl_target.set_text_color(0xffffff)
            if change_ignored == 1:
                timerSch.run("blink_now", 305, 0x00)
            else:
                timerSch.stop("blink_now")
                lbl_target.set_text_color(0xffffff)
        else:
            lbl_action.set_text('IDLE')
            lbl_action.set_text_color(0xffffff)
            lbl_action.set_align(ALIGN_CENTER, 0, DISP_LBL_TARGET_OFFSET)
            if change_ignored == 1:
                timerSch.run("blink_now", 305, 0x00)
            else:
                timerSch.stop("blink_now")
                lbl_target.set_text_color(0xffffff)            
    else:
        img_BtnA.set_hidden(True)
        img_BtnB.set_hidden(True)
        img_BtnC.set_hidden(True)
        
    # If mode is off
    if thermo_state == THERMO_MODES[0] and change_ignored == 0:
        tft.font(font)
        lbl_action.set_text('')
        lbl_target.set_text("---")
        lbl_target.set_text_color(0xffffff)
        lbl_target.set_align(ALIGN_CENTER, 0, DISP_LBL_TARGET_OFFSET)
        
    # If cooling is on
    elif cooling_state == 1 and thermo_state != THERMO_MODES[2]:
        lbl_action.set_text('COOLING')
        lbl_action.set_text_color(DISP_COLOR_COOL)
        lbl_action.set_align(ALIGN_CENTER, 0, DISP_LBL_ACTION_OFFSET)
        lbl_target.set_text(str(target_temp_display))
        lbl_target.set_text_color(DISP_COLOR_COOL)
        lbl_target.set_align(ALIGN_CENTER, 0, DISP_LBL_TARGET_OFFSET)
        slider_target.set_hidden(False)
        if change_ignored == 1:
            timerSch.run("blink_now", 305, 0x00)
        else:
            timerSch.stop("blink_now")
            lbl_target.set_text_color(DISP_COLOR_COOL)
        
    # if heating is on
    elif heating_state == 1 and thermo_state != THERMO_MODES[2]:
        lbl_action.set_text('HEATING')
        lbl_action.set_text_color(DISP_COLOR_HEAT)
        lbl_action.set_align(ALIGN_CENTER, 0, DISP_LBL_ACTION_OFFSET)
        lbl_target.set_text(str(target_temp_display))
        lbl_target.set_text_color(DISP_COLOR_HEAT)
        lbl_target.set_align(ALIGN_CENTER, 0, DISP_LBL_TARGET_OFFSET)
        slider_target.set_hidden(False)
        if change_ignored == 1:
            timerSch.run("blink_now", 305, 0x00)
        else:
            timerSch.stop("blink_now")
            lbl_target.set_text_color(DISP_COLOR_HEAT)

    # if fan is on
    elif fan_state == 1 and thermo_state != THERMO_MODES[2]:
        lbl_action.set_text('FAN')
        lbl_action.set_text_color(0xffffff)
        lbl_action.set_align(ALIGN_CENTER, 0, DISP_LBL_ACTION_OFFSET)
        lbl_target.set_text(str(target_temp_display))
        lbl_target.set_text_color(0xffffff)
        lbl_target.set_align(ALIGN_CENTER, 0, DISP_LBL_TARGET_OFFSET)
        slider_target.set_hidden(False)
        if change_ignored == 1:
            timerSch.run("blink_now", 305, 0x00)
        else:
            timerSch.stop("blink_now")
            lbl_target.set_text_color(0xffffff)
            
   # if none of the above conditions are true (ie. thermostat is on, but idle)   
    elif thermo_state != THERMO_MODES[2]:
        lbl_action.set_text('IDLE')
        lbl_action.set_text_color(0xffffff)
        lbl_action.set_align(ALIGN_CENTER, 0, DISP_LBL_ACTION_OFFSET)
        lbl_target.set_text(str(target_temp_display))
        lbl_target.set_text_color(0xffffff)
        lbl_target.set_align(ALIGN_CENTER, 0, DISP_LBL_TARGET_OFFSET)
        slider_target.set_hidden(False)
        if change_ignored == 1:
            timerSch.run("blink_now", 305, 0x00)
        else:
            timerSch.stop("blink_now")
            lbl_target.set_text_color(0xffffff)
            
# Draw the temperature arc
    t = THERMO_MIN_TEMP
    while t <= THERMO_MAX_TEMP:
        angle = (t * 8 + 200) % 360
        tft.font(font)
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
    target_temp = slider_target.get_value()
    
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
            timerSch.stop("blink_now")
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
        timerSch.stop("blink_now")
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
        timerSch.run("delayed_start", 1000, 0x00)
        timerSch.stop("blink_now")
        update_display()
    else:
        change_ignored = 1
        update_display()


def update_mqtt_state_topics():
    #update state of DHT22 sensors
    payload = {
        "temperature": dht22.temperature(),
        "humidity": dht22.humidity()
        }
    m5mqtt.publish(DEFAULT_TOPIC_SENSOR_PREFIX + TOPIC_STATE,str(json.dumps(payload)))
    
    #update state of thermostat target temperature
    m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_STATE, str(target_temp))
    
    #update state of thermostat mode
    m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_MODE_STATE, str(thermo_state))    
        
@timerSch.event("delayed_start")
def tdelayed_start():
    global delay
    delay -= 1
    if delay == 0:
        timerSch.stop("delayed_start")
        thermostat_decision_logic()

@timerSch.event("main_loop")
def tmain_loop():
    global ticks
    ticks += 1
            
@timerSch.event("blink_now")
def tblink_now():
    global blink
    if heating_state == 1:
        color = DISP_COLOR_HEAT
    elif cooling_state == 1:
        color = DISP_COLOR_COOL
    else:
        color = 0xffffff
    if blink == 0:
        lbl_target.set_text_color(0x000000)
        lbl_action.set_text_color(0x000000)
        blink = 1
    else:
        lbl_target.set_text_color(color)
        lbl_action.set_text_color(color)
        blink = 0        

def slider_target_changed(target_temp):
    m5mqtt.publish(DEFAULT_TOPIC_THERMOSTAT_PREFIX + TOPIC_STATE, str(target_temp))
    thermostat_decision_logic()

slider_target.changed(slider_target_changed)

def rcv_target_temp (topic_data):
    global target_temp
    slider_target.set_value(round(float(topic_data)))
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
    
thermostat_init()
comms_init()
thermostat_decision_logic()
timerSch.run("main_loop", 999, 0x00)

while True:
    
# We ignore button presses unless the Thermostat is in manual mode
    if btnA.wasPressed() and thermo_state == THERMO_MODES[2]:
        if heating_state == 0:
            manual_command = "heating on"
        elif heating_state == 1:
            manual_command = "heating off"
        thermostat_decision_logic()
        
    if btnB.wasPressed() and thermo_state == THERMO_MODES[2]:
        if cooling_state == 0:
            manual_command = "cooling on"
        elif cooling_state == 1:
            manual_command = "cooling off"
        thermostat_decision_logic()
        
    if btnC.wasPressed() and thermo_state == THERMO_MODES[2]:
        if fan_state == 0:
            manual_command = "fan on"
        elif fan_state == 1:
            manual_command = "fan off"
        thermostat_decision_logic()
            
    if ticks == THERMO_UPDATE_FREQUENCY:
        thermostat_decision_logic()
        update_mqtt_state_topics()
        ticks = 0
    wait_ms(2)

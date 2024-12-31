"""T-Display S3 TFT Display configuration."""
from machine import Pin, SPI
import st7789

# TFT display constants
MHS35 = 0

def config(rotation=0, buffer_size=0, options=0):
    return st7789.ST7789(
        SPI(1, baudrate=40000000, sck=Pin(18), mosi=Pin(23), miso=Pin(19)),
        240,
        240,
        reset=Pin(33, Pin.OUT),
        cs=Pin(14, Pin.OUT),
        dc=Pin(27, Pin.OUT),
        backlight=Pin(32, Pin.OUT),
        rotation=rotation,
        options=options)

tft = config()
